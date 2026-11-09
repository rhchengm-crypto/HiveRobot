"""Install explicitly frozen A8 reference metadata while preserving control settings.

The parent must provide sync_ready_manifest.json only after all local files and
checks are complete. Running this file without that manifest fails before any
network access. No robot motion, restart, model-fit activation or GitHub access.
"""
from pathlib import Path, PurePosixPath
import hashlib
import json
import subprocess
import zipfile

HERE = Path(__file__).resolve().parent
OUT = HERE.parent
ROOT = HERE.parents[2]
READY = HERE / 'sync_ready_manifest.json'
REMOTE = '/home/nvidia/hive_robot/DM_Control_Python'
DEST = REMOTE + '/data/board_workspace_trials/a8_accepted_reference_20261108/analysis_evidence'
HOST = 'nvidia@192.168.201.199'
OPTS = ['-i', str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),
        '-o', 'HostKeyAlias=192.168.201.199', '-o', 'HostKeyAlgorithms=ecdsa-sha2-nistp256',
        '-o', 'IdentitiesOnly=yes', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
        '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=2']
BASELINE_MUTABLE = {'left_arm_board_task_pose.py', 'data/left_arm_validated_path_library.json',
                    'data/left_arm_board_workspace_candidate.json'}
REFERENCE_FIELDS = {'approved_task_pose_references', 'validated_path_library_provenance',
                    'validated_path_reference_policy', 'reference_attempt_budget_s'}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def save(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')


def safe_relative(value):
    p = PurePosixPath(value)
    assert value == p.as_posix() and not p.is_absolute() and '..' not in p.parts and ':' not in value
    return p


def ssh(code, timeout=60):
    result = subprocess.run(['ssh'] + OPTS + [HOST, 'python3 -'], input=code.encode(),
                            capture_output=True, timeout=timeout)
    assert result.returncode == 0, result.stderr.decode(errors='replace')
    return json.loads(result.stdout)


def main():
    # No calls to ssh/scp precede this immutable manifest validation.
    ready = read(READY)
    assert ready.get('schema') == 1 and ready.get('ready_for_sync') is True
    assert ready.get('motion_issued') is False and ready.get('activate_fitted_candidate') is False
    assert ready.get('protected_baseline_overrides', {}) == {}, 'No silent baseline changes'
    files = ready['files']
    assert files and all(isinstance(row, dict) for row in files.values())
    installs = {}
    baseline = dict(read(ROOT/'outputs/a8_lower25_execution_20261108/preflight.json')['source_hashes'])
    learning = read(ROOT/'outputs/a8_lower25_execution_20261108/learning_sync/old_new_hash_audit.json')
    assert learning['synchronization_complete']
    for row in learning['rows']:
        baseline['data/'+row['name']] = row['new_sha256']
    assert len(baseline) == 30
    assert baseline['data/left_arm_v2_8_clearance_bias.json'] == '52f1b83bc1a267e673cb1abcfa6981b25465111200daadade69b7ee7f31acd68'
    for relative, row in files.items():
        safe_relative(relative)
        p = ROOT/relative
        assert p.is_file() and p.stat().st_size == row['bytes'] and sha(p) == row['sha256'], relative
        assert '__pycache__' not in p.parts and p.suffix not in {'.zip', '.png', '.jpg', '.npy'}, relative
        # Complete prior control/RGB-D evidence remains at its original location.
        assert not relative.startswith('outputs/a8_lower25_execution_20261108/control_evidence/'), relative
        if row.get('runtime_path'):
            remote = row['runtime_path']
            safe_relative(remote)
            assert relative.startswith('scripts/') and relative[len('scripts/'):] == remote
            allowed_addition = remote == 'build_left_arm_path_reference_library.py' or (
                remote.startswith('test_') and '/' not in remote and remote not in baseline)
            allowed_registry = remote.startswith('data/board_workspace_trials/validation_') and remote.endswith('.json')
            assert remote in BASELINE_MUTABLE or allowed_addition or allowed_registry, remote
            assert remote not in installs
            if remote not in baseline:
                assert 'expected_previous_sha256' in row, 'Freeze previous remote hash or null: '+remote
            installs[remote] = relative
    assert BASELINE_MUTABLE.issubset(installs), 'All three bounded runtime reference updates required'
    original = read(ROOT/'outputs/a8_lower25_execution_20261108/control_evidence/current_runtime_models/left_arm_board_workspace_candidate.json')
    candidate = read(ROOT/installs['data/left_arm_board_workspace_candidate.json'])
    changed = {k for k in set(original) | set(candidate) if original.get(k) != candidate.get(k)}
    assert changed <= REFERENCE_FIELDS, 'Candidate modified non-reference control fields: '+str(changed)
    assert candidate['rigid_model_sha256'] == original['rigid_model_sha256']
    for relative in files:
        if relative.endswith('.py'):
            compile((ROOT/relative).read_text(encoding='utf-8-sig'), relative, 'exec')
    manifest = dict(files)
    helper_rel = Path(__file__).relative_to(ROOT).as_posix()
    manifest[helper_rel] = {'bytes': Path(__file__).stat().st_size, 'sha256': sha(__file__)}
    ready_rel = READY.relative_to(ROOT).as_posix()
    manifest[ready_rel] = {'bytes': READY.stat().st_size, 'sha256': sha(READY)}
    save(HERE/'sync_payload_manifest.json', manifest)
    bundle = HERE/'accepted_reference_evidence.zip'
    with zipfile.ZipFile(bundle, 'w', zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        for relative in sorted(manifest):
            z.write(ROOT/relative, relative)
        z.writestr('manifest.json', json.dumps(manifest, indent=2))
    with zipfile.ZipFile(bundle) as z:
        assert z.testzip() is None
    bundle_sha = sha(bundle)
    pending = DEST+'/accepted_reference_evidence.zip.partial-'+bundle_sha[:12]
    preflight = ssh("from pathlib import Path;import json;p=Path("+repr(DEST)+
                    ");p.mkdir(parents=True,exist_ok=True);print(json.dumps({'directory':str(p)}))")
    assert preflight['directory'] == DEST
    result = subprocess.run(['scp']+OPTS+[str(bundle), HOST+':'+pending], capture_output=True, timeout=100)
    assert result.returncode == 0, result.stderr.decode(errors='replace')
    payload = {'root': REMOTE, 'dest': DEST, 'pending': pending, 'zip_sha256': bundle_sha,
               'zip_bytes': bundle.stat().st_size, 'baseline': baseline, 'manifest': manifest,
               'installs': installs, 'reference_fields': sorted(REFERENCE_FIELDS),
               'mutable': sorted(BASELINE_MUTABLE)}
    code = REMOTE_CODE.replace('__PAYLOAD__', repr(payload))
    report = ssh(code, timeout=100)
    assert report['ok'] and report['archive_sha256'] == bundle_sha
    save(HERE/'sync_receipt.json', report)
    print(json.dumps(report, ensure_ascii=False, indent=2))


REMOTE_CODE = r'''
from pathlib import Path,PurePosixPath
import hashlib,json,os,stat,time,urllib.request,zipfile
P=__PAYLOAD__
ROOT=Path(P['root']).resolve();DEST=Path(P['dest']).resolve();PENDING=Path(P['pending'])
assert ROOT in DEST.parents and DEST==ROOT/'data/board_workspace_trials/a8_accepted_reference_20261108/analysis_evidence'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def state(n):
 p=(ROOT/n).resolve();assert ROOT in p.parents
 return sha(p) if p.is_file() else None
def assert_baseline():
 for n,h in P['baseline'].items():assert state(n)==h,'Unexpected runtime/learning baseline: '+n
def idle_snapshot():
 client=urllib.request.build_opener(urllib.request.ProxyHandler({}));out={}
 for path in ['/api/state','/api/v3/dynamics/status']:
  with client.open('http://127.0.0.1:8100'+path,timeout=5) as response:d=json.load(response)
  assert isinstance(d,dict) and 'run' in d,'Unrecognized Web state: '+path
  run=d['run'];assert run is None or isinstance(run,dict)
  if run:
   for flag in ['running','motion_running','support_running','process_alive']:
    assert run.get(flag) is not True,'Active robot process: '+path+' '+flag
   assert run.get('control_state') not in ['protective-hold','motion','moving','running']
  out[path]={'run':run,'checked_at_unix_s':time.time()}
 return out
assert_baseline();before_web=idle_snapshot()
assert sha(PENDING)==P['zip_sha256'] and PENDING.stat().st_size==P['zip_bytes']
with zipfile.ZipFile(PENDING) as z:
 assert z.testzip() is None
 manifest=json.loads(z.read('manifest.json'));assert manifest==P['manifest']
 assert set(z.namelist())==set(manifest)|{'manifest.json'}
 contents={}
 for n,row in manifest.items():
  relative=PurePosixPath(n);assert not relative.is_absolute() and '..' not in relative.parts and ':' not in n
  assert stat.S_IFMT(z.getinfo(n).external_attr>>16)!=stat.S_IFLNK
  data=z.read(n);assert len(data)==row['bytes'] and hashlib.sha256(data).hexdigest()==row['sha256'],n
  contents[n]=data
  if n.endswith('.py'):compile(data,n,'exec')
 original=json.loads((ROOT/'data/left_arm_board_workspace_candidate.json').read_text())
 candidate=json.loads(contents[P['installs']['data/left_arm_board_workspace_candidate.json']])
 changes={k for k in set(original)|set(candidate) if original.get(k)!=candidate.get(k)}
 assert changes<=set(P['reference_fields']),'Non-reference candidate changes'
 for remote,local in P['installs'].items():
  if remote not in P['baseline']:
   assert state(remote)==manifest[local]['expected_previous_sha256'],'Unexpected preexisting target: '+remote
 # Evidence is never imported and fitting candidates never replace active models.
 for n,data in contents.items():
  target=(DEST/n).resolve();assert DEST in target.parents
  target.parent.mkdir(parents=True,exist_ok=True)
  if target.exists():assert sha(target)==manifest[n]['sha256'],'Evidence already differs: '+n
  else:
   tmp=target.with_name(target.name+'.evidence.pending');tmp.write_bytes(data);os.replace(tmp,target)
 # Stage all exact bytes before a second idle/baseline check and any installation.
 staged={}
 for remote,local in P['installs'].items():
  target=(ROOT/remote).resolve();assert ROOT in target.parents
  target.parent.mkdir(parents=True,exist_ok=True)
  temporary=target.with_name(target.name+'.a8reference.pending')
  temporary.write_bytes(contents[local]);assert sha(temporary)==manifest[local]['sha256']
  staged[remote]=temporary
 assert_baseline();install_web=idle_snapshot()
 backup_root=DEST/('exact_previous_runtime_'+str(time.time_ns()));backup_root.mkdir()
 backups={};installed={}
 try:
  for remote,temporary in staged.items():
   target=ROOT/remote
   if target.exists():
    backup=backup_root/remote;backup.parent.mkdir(parents=True,exist_ok=True);backup.write_bytes(target.read_bytes())
    backups[remote]={'exists':True,'sha256':sha(backup),'path':str(backup)}
   else:backups[remote]={'exists':False,'sha256':None,'path':None}
   os.replace(temporary,target);installed[remote]=sha(target)
   assert installed[remote]==manifest[P['installs'][remote]]['sha256']
  for remote,old in P['baseline'].items():
   expected=installed[remote] if remote in P['mutable'] else old
   assert state(remote)==expected,'Protected file changed: '+remote
 except BaseException:
  # No restart or motor call is made even on failure; revert only our installed files.
  for remote in reversed(list(installed)):
   target=ROOT/remote;info=backups[remote]
   if info['exists']:
    temp=target.with_name(target.name+'.rollback.pending');temp.write_bytes(Path(info['path']).read_bytes());os.replace(temp,target)
   else:target.unlink()
  raise
 archive=DEST/'accepted_reference_evidence.zip'
 if archive.exists():assert sha(archive)==P['zip_sha256']
 else:os.replace(PENDING,archive)
 report={'ok':True,'motion_issued':False,'motors_opened':False,'web_restarted':False,'processes_killed':False,
  'fitted_candidate_activated':False,'GitHub_accessed':False,'runtime_code_updated_without_restart':True,
  'archive_path':str(archive),'archive_sha256':sha(archive),'archive_bytes':P['zip_bytes'],
  'manifest_member_count':len(manifest),'all_member_sha256_verified':True,'all_crc_verified':True,
  'before_web':before_web,'immediately_before_install_web':install_web,
  'baseline_files':len(P['baseline']),'baseline_mutable_files':P['mutable'],
  'protected_baseline_files_unchanged':len(set(P['baseline'])-set(P['mutable'])),
  'learning_files_unchanged':True,'active_general_dynamics_unchanged':True,'active_hold_model_unchanged':True,
  'candidate_changed_fields':sorted(changes),'installed_sha256':installed,'exact_previous_backups':backups}
 (DEST/'sync_receipt.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
 print(json.dumps(report))
'''


if __name__ == '__main__':
    main()
