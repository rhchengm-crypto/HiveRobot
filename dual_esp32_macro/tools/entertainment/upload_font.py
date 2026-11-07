"""Copy a validated animation pack to the board's SD over bounded serial chunks."""
import sys,time,struct,zlib,argparse
from pathlib import Path
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE/'deps'))
sys.path.insert(0,r'C:\Users\rhche\Desktop\dual_esp32_macro\.deps')
import serial
def validate(data):
    if len(data)!=917520 or data[:8]!=b'CJK16V1\0':raise ValueError('Font format')

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--port',default='COM4')
    default_pack=HERE.parents[1]/'sdcard/fonts/cjk16.bin'
    parser.add_argument('--file',type=Path,default=default_pack)
    args=parser.parse_args();data=args.file.read_bytes();validate(data)
    log=(HERE/'font-upload.log').open('w',encoding='utf-8')
    port=serial.Serial(port=None,baudrate=115200,timeout=.15,write_timeout=5)
    port.dtr=False;port.rts=False;port.port=args.port;port.open()
    def wait_for(prefix,timeout=12):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            line=port.readline().decode('utf-8','replace').strip()
            if not line:continue
            log.write(line+'\n');log.flush()
            if '[FONT] UPLOAD FAIL' in line:raise RuntimeError(line)
            if prefix in line:return line
        raise TimeoutError(prefix)
    try:
        port.reset_input_buffer()
        port.write(f'fontput {len(data)} {zlib.crc32(data):08x}\n'.encode())
        print(wait_for('[FONT] UPLOAD READY'),flush=True)
        started=time.monotonic()
        for at in range(0,len(data),512):
            block=data[at:at+512];port.write(block)
            target=at+len(block)
            while True:
                line=wait_for('[FONT] ACK ')
                if int(line.rsplit(' ',1)[-1])==target:break
            if target//131072!=at//131072: print(f'{target}/{len(data)} bytes',flush=True)
        print(wait_for('[FONT] UPLOAD OK'),flush=True)
        print(wait_for('[FONT] ready=1'),flush=True)
        print(f'Completed in {time.monotonic()-started:.1f}s; full-file CRC verified.',flush=True)
    finally:port.close();log.close()
if __name__=='__main__':main()
