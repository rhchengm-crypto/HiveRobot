import io,json,threading,unittest,urllib.request,urllib.error,wave
import os,subprocess
from gateway import GatewayServer, GatewayError, clip_utf8, OpenAIBackend, searched_answer
from unittest.mock import patch

class SearchTests(unittest.TestCase):
    def test_gpt61_reasoning_budget_and_model_metadata(self):
        backend=OpenAIBackend({'openai_api_key':'unit-test'})
        response={'model':'gpt-6.1-sol','output':[{'type':'message','content':[{'type':'output_text','text':'二。'}]}]}
        with patch.object(backend,'call',return_value=response) as call:
            self.assertEqual(backend.answer('一加一等于几'),'二。')
        body=json.loads(call.call_args.args[1]);self.assertEqual(body['model'],'gpt-6.1-sol')
        self.assertEqual(body['reasoning'],{'effort':'low'});self.assertGreaterEqual(body['max_output_tokens'],4096)
        self.assertEqual(backend.last_model,'gpt-6.1-sol');self.assertEqual(body['tools'],[{'type':'web_search'}])
    def test_weather_requires_real_search_and_preserves_model(self):
        backend=OpenAIBackend({'openai_api_key':'unit-test','gpt_model':'gpt-4.1-mini'})
        response={'output':[{'type':'web_search_call','status':'completed'},{'type':'message','content':[{'type':'output_text','text':'气象预报：18摄氏度。','annotations':[]}]}]}
        with patch.object(backend,'call',return_value=response) as call:
            answer=backend.answer('查询Sammamish今天的天气')
        body=json.loads(call.call_args.args[1]);self.assertEqual(body['model'],'gpt-4.1-mini')
        self.assertEqual(body['tools'],[{'type':'web_search'}]);self.assertEqual(body['tool_choice'],'required')
        self.assertIn('America/Los_Angeles',body['instructions']);self.assertIn('当前UTC时间',body['instructions'])
        self.assertTrue(backend.last_search_used);self.assertIn('18',answer)
    def test_search_failure_does_not_return_invented_current_answer(self):
        backend=OpenAIBackend({'openai_api_key':'unit-test'})
        with patch.object(backend,'call',return_value={'output':[{'type':'message','content':[{'type':'output_text','text':'今天晴天'}]}]}):
            with self.assertRaises(GatewayError):backend.answer('今天纽约天气怎样')
    def test_simple_question_can_skip_search(self):
        backend=OpenAIBackend({'openai_api_key':'unit-test'})
        with patch.object(backend,'call',return_value={'output':[{'type':'message','content':[{'type':'output_text','text':'二。'}]}]}) as call:
            self.assertEqual(backend.answer('一加一等于几'),'二。')
        self.assertEqual(json.loads(call.call_args.args[1])['tool_choice'],'auto');self.assertFalse(backend.last_search_used)
    def test_lcd_citations_and_utf8_payload_bound(self):
        prefix='天气预报：18摄氏度。';marker='\ue200cite\ue202source\ue201';url='https://forecast.weather.gov/example'
        result={'output':[{'type':'message','content':[{'type':'output_text','text':prefix+marker,'annotations':[{'type':'url_citation','url':url,'start_index':len(prefix),'end_index':len(prefix+marker)}]}]}]}
        answer=searched_answer(result);self.assertIn(prefix+'[1]',answer);self.assertIn(url,answer);self.assertNotIn('\ue200',answer)
        result['output'][0]['content'][0]['text']+='上海天气'*2000
        self.assertLessEqual(len(searched_answer(result).encode()),3800);self.assertIn(url,searched_answer(result))

class FakeBackend:
    def __init__(self):self.calls=[];self.fail=False
    def transcribe(self,audio):self.calls.append('asr');return '一加一等于几'
    def answer(self,text):
        self.calls.append('gpt')
        if self.fail:raise GatewayError(502,'OpenAI额度或频率受限')
        return '一加一等于二。'

class GatewayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.backend=FakeBackend();cls.token='test-token-'+'x'*40
        cls.server=GatewayServer(('127.0.0.1',0),{'gateway_token':cls.token},cls.backend)
        cls.thread=threading.Thread(target=cls.server.serve_forever,daemon=True);cls.thread.start()
        cls.base='http://127.0.0.1:'+str(cls.server.server_port)
        cls.opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    @classmethod
    def tearDownClass(cls):cls.server.shutdown();cls.server.server_close();cls.thread.join()
    def setUp(self):self.backend.calls=[];self.backend.fail=False;self.server.recent.clear()
    def request(self,path,body=None,token=None,ctype='application/json'):
        r=urllib.request.Request(self.base+path,data=body,headers={'Authorization':'Bearer '+(self.token if token is None else token),'Content-Type':ctype})
        try:
            with self.opener.open(r,timeout=3) as response:return response.status,json.load(response)
        except urllib.error.HTTPError as e:
            with e:return e.code,json.load(e)
    def test_authenticated_health(self):
        code,d=self.request('/v1/health');self.assertEqual(code,200);self.assertEqual(d['egress'],'raspberry-pi');self.assertEqual(self.backend.calls,[])
    def test_auth_before_paid_calls(self):
        self.assertEqual(self.request('/v1/ask-text',b'{"text":"hi"}',token='bad')[0],401);self.assertEqual(self.backend.calls,[])
    def test_real_audio_protocol(self):
        b=io.BytesIO()
        with wave.open(b,'wb') as w:w.setnchannels(1);w.setsampwidth(2);w.setframerate(16000);w.writeframes(b'\0\0'*16000)
        code,d=self.request('/v1/ask',b.getvalue(),ctype='audio/wav')
        self.assertEqual(code,200);self.assertIn('二',d['answer']);self.assertEqual(d['question'],'一加一等于几');self.assertEqual(self.backend.calls,['asr','gpt'])
    def test_bad_audio_rejected_before_api(self):
        self.assertEqual(self.request('/v1/ask',b'bad wave',ctype='audio/wav')[0],400);self.assertEqual(self.backend.calls,[])
    def test_bad_opus_rejected_before_api(self):
        self.assertEqual(self.request('/v1/ask',b'OggS'+b'\0'*23,ctype='audio/ogg')[0],400);self.assertEqual(self.backend.calls,[])
    @unittest.skipUnless(os.path.exists('/usr/bin/ffmpeg'),'Pi ffmpeg integration')
    def test_opus_forwarded_to_api(self):
        sample=subprocess.run(['/usr/bin/ffmpeg','-nostdin','-hide_banner','-loglevel','error','-f','lavfi','-i','sine=frequency=440:duration=1','-ar','16000','-ac','1','-c:a','libopus','-b:a','24k','-f','ogg','pipe:1'],capture_output=True,check=True,timeout=10).stdout
        code,d=self.request('/v1/ask',sample,ctype='audio/ogg')
        self.assertEqual(code,200);self.assertEqual(self.backend.calls,['asr','gpt'])
    def test_busy_no_duplicate_payment(self):
        self.server.slot.acquire()
        try:self.assertEqual(self.request('/v1/ask-text',b'{"text":"hi"}')[0],409)
        finally:self.server.slot.release()
        self.assertEqual(self.backend.calls,[])
    def test_text_and_safe_vendor_error(self):
        self.backend.fail=True
        code,d=self.request('/v1/ask-text',b'{"text":"hi"}')
        self.assertEqual(code,502);self.assertNotIn('key',json.dumps(d))
    def test_rate_limit(self):
        import time
        self.server.recent.extend([time.monotonic()]*8)
        self.assertEqual(self.request('/v1/ask-text',b'{"text":"hi"}')[0],429);self.assertEqual(self.backend.calls,[])
    def test_oversize_and_utf8(self):
        self.assertEqual(self.request('/v1/ask-text',b'x'*12001)[0],413)
        text=clip_utf8('上海话'*2000);self.assertLessEqual(len(text.encode()),3800);self.assertTrue(text)

if __name__=='__main__':unittest.main()
