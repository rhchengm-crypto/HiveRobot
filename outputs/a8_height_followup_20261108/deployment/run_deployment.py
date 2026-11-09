from pathlib import Path
import hashlib,json,subprocess
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
receipt=json.loads((HERE/'package_receipt.json').read_text());uploaded=json.loads((HERE/'upload_receipt.json').read_text())
assert uploaded['uploaded'] and not uploaded['deployed']
opts=['-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','HostKeyAlias=192.168.201.199','-o','HostKeyAlgorithms=ecdsa-sha2-nistp256','-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','-o','ServerAliveInterval=10','-o','ServerAliveCountMax=3']
r=subprocess.run(['ssh']+opts+['nvidia@192.168.201.199','python3 /tmp/deploy_a8_height_followup_20261108.py'],capture_output=True,timeout=240)
(HERE/'remote_deployment.stdout').write_bytes(r.stdout);(HERE/'remote_deployment.stderr').write_bytes(r.stderr)
assert r.returncode==0,r.stderr.decode(errors='replace')
report=json.loads(r.stdout)
assert report['ok'] and not report['motion_issued'] and not report['web_restarted']
assert set(report['deployed_sha256'])==set(receipt['source_file_sha256'])
assert report['deployed_sha256']==receipt['source_file_sha256']
home=json.loads((ROOT/'outputs/a8_v33_home_20261108/control_evidence/data/board_workspace_trials/a8_v33_home_20261108/source_hashes.json').read_text())
assert all(report['protected_sha256'][name]==expected for name,expected in home.items())
learning=json.loads((ROOT/'outputs/a8_v33_home_20261108/learning_sync/old_new_hash_audit.json').read_text())
assert all(report['protected_sha256']['data/'+row['name']]==row['new_sha256'] for row in learning['rows'])
assert report['prepared_hash_count']==13
assert report['prepared_source_hashes']['task_pose']==receipt['source_file_sha256']['left_arm_board_task_pose.py']
(HERE/'deployment_receipt.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps({'ok':True,'installed':report['deployed_sha256'],'protected_Home_sources':13,'protected_learning_files':5,'tests':report['offline_hardware_free_tests'],'stage':report['stage'],'backup':report['backup'],'motion_issued':False,'web_restarted':False},ensure_ascii=False,indent=2),flush=True)
