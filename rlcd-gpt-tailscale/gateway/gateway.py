"""RLCD private GPT gateway. Bind only to a Tailscale IP; no forward proxy."""
import argparse
import collections
from datetime import datetime, timezone
import hmac
import io
import ipaddress
import json
import logging
import os
import re
import secrets
import socket
import threading
import time
import urllib.error
import urllib.request
import wave
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit
from zoneinfo import ZoneInfo, ZoneInfoNotFoundError

MAX_AUDIO = 700000
MAX_TEXT = 3000
LOG = logging.getLogger('rlcd-gpt')

class GatewayError(Exception):
    def __init__(self, code, message):
        self.code, self.message = code, message

def clip_utf8(text, limit=3800):
    return text.encode('utf-8')[:limit].decode('utf-8', 'ignore')

def needs_current_information(text):
    return bool(re.search(r'天气|气温|预报|新闻|热搜|最新|今天|今日|明天|实时|查一下|查询|搜索|\b(weather|forecast|news|latest|today|tomorrow|current|search)\b',text,re.I))

def searched_answer(result):
    parts=[];sources=[]
    for item in result.get('output',[]):
        if item.get('type')!='message': continue
        for content in item.get('content',[]):
            if content.get('type')!='output_text': continue
            text=content.get('text','');replacements=[]
            for annotation in content.get('annotations',[]):
                if annotation.get('type')!='url_citation': continue
                url=annotation.get('url','')
                if not isinstance(url,str) or len(url.encode('utf-8'))>600 or any(c.isspace() for c in url): continue
                try:parsed=urlsplit(url)
                except ValueError:continue
                if parsed.scheme not in ('https','http') or not parsed.netloc: continue
                if url not in sources and len(sources)<3:sources.append(url)
                if url not in sources:continue
                start,end=annotation.get('start_index'),annotation.get('end_index')
                if isinstance(start,int) and isinstance(end,int) and 0<=start<end<=len(text):
                    replacements.append((start,end,'['+str(sources.index(url)+1)+']'))
            for start,end,marker in sorted(set(replacements),reverse=True):text=text[:start]+marker+text[end:]
            # LCD fonts cannot render the private-use web citation markers.
            parts.append(re.sub(r'\ue200.*?\ue201','',text))
    answer=''.join(parts).strip()
    references='\n\n来源：\n'+'\n'.join('['+str(i+1)+'] '+url for i,url in enumerate(sources)) if sources else ''
    return clip_utf8(answer,3800-len(references.encode('utf-8')))+references if answer else ''

class OpenAIBackend:
    def __init__(self, config):
        self.key = config['openai_api_key']
        self.model = config.get('gpt_model', 'gpt-6.1-sol')
        self.asr = config.get('transcription_model', 'gpt-4o-mini-transcribe')
        self.user_timezone = config.get('user_timezone','America/Los_Angeles')
        self.last_search_used = False
        self.last_model = None
        # Ignore proxy environment variables: requests must leave this Pi directly.
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def call(self, path, body, content_type):
        request = urllib.request.Request('https://api.openai.com/v1/' + path, data=body,
            headers={'Authorization': 'Bearer ' + self.key, 'Content-Type': content_type})
        try:
            with self.opener.open(request, timeout=65) as response:
                return json.loads(response.read(1048576))
        except urllib.error.HTTPError as exc:
            # Never propagate vendor body, credentials or prompt into device logs.
            LOG.warning('OpenAI HTTP %s at %s', exc.code, path)
            messages = {401:'OpenAI密钥无效',403:'OpenAI访问被拒绝',429:'OpenAI额度或频率受限'}
            raise GatewayError(502, messages.get(exc.code, 'OpenAI服务请求失败')) from None
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, OSError):
            raise GatewayError(504, '树莓派连接OpenAI超时或失败') from None

    def transcribe(self, audio):
        extension, content_type = ('ogg','audio/ogg') if audio.startswith(b'OggS') else ('wav','audio/wav')
        boundary = 'rlcd-' + secrets.token_hex(16)
        body = bytearray()
        for name, value in [('model', self.asr), ('language', 'zh'), ('response_format', 'json')]:
            body.extend(('--'+boundary+'\r\nContent-Disposition: form-data; name="'+name+'"\r\n\r\n'+value+'\r\n').encode())
        body.extend(('--'+boundary+'\r\nContent-Disposition: form-data; name="file"; filename="speech.'+extension+'"\r\nContent-Type: '+content_type+'\r\n\r\n').encode())
        body.extend(audio); body.extend(('\r\n--'+boundary+'--\r\n').encode())
        result = self.call('audio/transcriptions', bytes(body), 'multipart/form-data; boundary='+boundary)
        text = result.get('text', '').strip()
        if not text:
            raise GatewayError(422, '没有识别到有效语音，请重录')
        return text

    def answer(self, text):
        self.last_search_used=False
        self.last_model=None
        require_search=needs_current_information(text)
        now=datetime.now(timezone.utc).isoformat(timespec='seconds')
        try:local_time=datetime.now(ZoneInfo(self.user_timezone)).isoformat(timespec='seconds')
        except ZoneInfoNotFoundError:local_time='未校验；请按上述UTC时间和用户时区换算，不要直接采用UTC日期'
        body = {'model':self.model, 'store':False, 'max_output_tokens':600,
            'tools':[{'type':'web_search'}],
            'tool_choice':'required' if require_search else 'auto',
            'instructions':'用简体中文简洁回答，适合小屏阅读，正文尽量150字、最多350字，不用Markdown标题、表格或逐小时长列表。'
                '涉及天气、新闻或其他实时信息必须先联网搜索，优先权威来源，引用1至2个来源。'
                '天气需说明城市、当地日期、摄氏温度和降雨情况；区分当前观测和预报。'
                '用户未说明天气地点时先询问城市，不把树莓派出口IP当作用户所在地。'
                '搜索未得到所需数据时如实说明，不用历史知识编造实时数据。'
                '当前UTC时间：'+now+'。用户默认时区：'+self.user_timezone+'，当地时间明确为：'+local_time+'。'
                '今天、明天必须按当地日期理解，并以该日期搜索；如果来源只有明天的预报，需说明它不是今日数据。'
                '若用户明确了其他时区的地区，则按该地区当地日期理解今天。',
            'input':text}
        if self.model.startswith('gpt-6.1-sol'):
            body['reasoning']={'effort':'low'}
            # This limit includes reasoning tokens as well as the visible answer.
            # The LCD response remains bounded independently to 3800 UTF-8 bytes.
            body['max_output_tokens']=4096
        result = self.call('responses', json.dumps(body, ensure_ascii=False).encode(), 'application/json')
        self.last_model=result.get('model')
        self.last_search_used=any(item.get('type')=='web_search_call' and item.get('status')=='completed' for item in result.get('output',[]))
        if require_search and not self.last_search_used:raise GatewayError(502,'联网搜索未完成，请稍后重试')
        answer = searched_answer(result)
        if not answer:
            raise GatewayError(502, 'GPT未返回文字，请重试')
        return clip_utf8(answer)

def validate_audio(audio):
    try:
        with wave.open(io.BytesIO(audio), 'rb') as wav:
            if (wav.getnchannels(), wav.getsampwidth(), wav.getframerate(), wav.getcomptype()) != (1,2,16000,'NONE'):
                raise ValueError()
            frames = wav.getnframes()
            if not 5333 <= frames <= 320000 or len(wav.readframes(frames)) != frames*2:
                raise ValueError()
    except (wave.Error, EOFError, ValueError):
        raise GatewayError(400, '录音格式错误，需要16kHz单声道WAV') from None

def validate_opus(audio):
    # Bound both the container duration and decoder output before paid API calls.
    at=0;sequence=0;serial=None;ended=False;granule=0
    while at<len(audio):
        if at+27>len(audio) or audio[at:at+4]!=b'OggS' or audio[at+4]!=0:
            raise GatewayError(400,'Opus录音格式错误')
        flags=audio[at+5];segments=audio[at+26];header=at+27+segments
        if header>len(audio): raise GatewayError(400,'Opus录音不完整')
        end=header+sum(audio[at+27:header])
        current_serial=audio[at+14:at+18];seq=int.from_bytes(audio[at+18:at+22],'little')
        g=int.from_bytes(audio[at+6:at+14],'little')
        if end>len(audio) or seq!=sequence or (serial is not None and serial!=current_serial) or g<granule or g>960000:
            raise GatewayError(400,'Opus录音长度或顺序错误')
        if sequence==0 and (not flags&2 or audio[header:header+10]!=b'OpusHead\x01\x01'):
            raise GatewayError(400,'需要单声道Opus录音')
        serial=current_serial;sequence+=1;granule=g;at=end;ended=bool(flags&4)
        if ended and at!=len(audio): raise GatewayError(400,'Opus录音结束位置错误')
    if not ended or granule<16000: raise GatewayError(400,'Opus录音过短或不完整')
    return audio

class GatewayServer(ThreadingHTTPServer):
    daemon_threads = True
    def __init__(self, address, config, backend=None):
        self.token = config['gateway_token']
        if len(self.token)<32:
            raise ValueError('gateway_token must contain at least 32 characters')
        self.backend = backend if backend is not None else OpenAIBackend(config)
        self.slot = threading.BoundedSemaphore(1)
        self.recent = collections.deque()
        self.rate_lock = threading.Lock()
        super().__init__(address, GatewayHandler)

class GatewayHandler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'
    server_version = 'RLCDGateway/1'
    def setup(self):
        super().setup(); self.connection.settimeout(20)

    def log_message(self, *args):
        pass

    def send_json(self, code, value):
        data = json.dumps(value, ensure_ascii=False).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type','application/json; charset=utf-8')
        self.send_header('Content-Length',str(len(data)))
        self.send_header('Cache-Control','no-store')
        self.send_header('Connection','close')
        self.end_headers(); self.wfile.write(data); self.close_connection=True

    def authorized(self):
        supplied = self.headers.get('Authorization', '')
        if not hmac.compare_digest(supplied.encode(), ('Bearer '+self.server.token).encode()):
            self.send_json(401, {'ok':False,'error':'网关认证失败'})
            return False
        return True

    def do_GET(self):
        if not self.authorized(): return
        if self.path!='/v1/health':
            self.send_json(404, {'ok':False,'error':'接口不存在'}); return
        self.send_json(200, {'ok':True,'service':'rlcd-gpt','protocol':1,'egress':'raspberry-pi','gpt_model':getattr(self.server.backend,'model',None)})

    def do_POST(self):
        if not self.authorized(): return
        if self.path not in ('/v1/ask', '/v1/ask-text'):
            self.send_json(404, {'ok':False,'error':'接口不存在'}); return
        acquired = False
        start=time.monotonic()
        try:
            if self.headers.get('Transfer-Encoding') or len(self.headers.get_all('Content-Length',[]))!=1:
                raise GatewayError(411, '请求必须指定音频长度')
            try: length=int(self.headers['Content-Length'])
            except ValueError: raise GatewayError(400, '请求长度错误')
            maximum=MAX_AUDIO if self.path=='/v1/ask' else 12000
            if not 0<length<=maximum: raise GatewayError(413, '请求过大或为空')
            acquired=self.server.slot.acquire(blocking=False)
            if not acquired: raise GatewayError(409, '网关忙，请稍后重试')
            body=self.rfile.read(length)
            if len(body)!=length: raise GatewayError(400, '上传音频不完整')
            if self.path=='/v1/ask':
                content_type=self.headers.get_content_type()
                if content_type=='audio/ogg': body=validate_opus(body)
                elif content_type!='audio/wav': raise GatewayError(415,'需要WAV或Opus音频')
                if content_type=='audio/wav': validate_audio(body)
                text=None
            else:
                try: text=json.loads(body)['text']
                except (ValueError, KeyError, TypeError): raise GatewayError(400,'文字请求格式错误')
                if not isinstance(text,str) or not text.strip() or len(text)>MAX_TEXT: raise GatewayError(400,'文字长度错误')
            with self.server.rate_lock:
                now=time.monotonic()
                while self.server.recent and now-self.server.recent[0]>60: self.server.recent.popleft()
                if len(self.server.recent)>=8: raise GatewayError(429,'请求频繁，请稍后重试')
                self.server.recent.append(now)
            if text is None: text=self.server.backend.transcribe(body)
            answer=self.server.backend.answer(text)
            elapsed=int((time.monotonic()-start)*1000)
            self.send_json(200, {'ok':True,'answer':answer,'question':clip_utf8(text,2000),'gateway_ms':elapsed,'egress':'raspberry-pi','web_search_used':bool(getattr(self.server.backend,'last_search_used',False)),'model':getattr(self.server.backend,'last_model',None)})
            LOG.info('request completed duration_ms=%s audio=%s', elapsed,self.path=='/v1/ask')
        except GatewayError as exc:
            self.send_json(exc.code, {'ok':False,'error':exc.message})
        except (TimeoutError, socket.timeout):
            self.send_json(408, {'ok':False,'error':'音频上传超时'})
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception:
            LOG.error('request failed without logging request data')
            self.send_json(500, {'ok':False,'error':'家庭网关内部错误'})
        finally:
            if acquired: self.server.slot.release()

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--config',required=True)
    args=parser.parse_args()
    with open(args.config,encoding='utf-8-sig') as f: config=json.load(f)
    address=config.get('listen_address','100.71.158.56')
    if ipaddress.ip_address(address) not in ipaddress.ip_network('100.64.0.0/10'):
        raise SystemExit('Refusing to bind outside the Tailscale IPv4 range')
    if not config.get('openai_api_key'): raise SystemExit('OpenAI API key is not configured')
    logging.basicConfig(level=logging.INFO,format='%(asctime)s %(message)s')
    server=GatewayServer((address,int(config.get('listen_port',8765))),config)
    LOG.info('gateway ready on tailnet port %s',server.server_port)
    server.serve_forever()

if __name__=='__main__': main()
