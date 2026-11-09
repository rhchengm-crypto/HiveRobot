from pathlib import Path
import datetime,json,runpy,traceback
HERE=Path(__file__).resolve().parent
try:
 runpy.run_path(str(HERE/'sync_analysis_data.py'),run_name='__main__')
except BaseException as error:
 failure={'ok':False,'single_attempt_only':True,'retry_attempted':False,'motion_issued':False,'timestamp_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'error_type':type(error).__name__,'error':str(error),'traceback':traceback.format_exc()}
 (HERE/'data_sync_failure.json').write_text(json.dumps(failure,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
 print(json.dumps(failure,ensure_ascii=False),flush=True)
 raise
