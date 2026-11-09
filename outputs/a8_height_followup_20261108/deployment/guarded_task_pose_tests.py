"""Run only two staged, hardware-free regression modules against root dependencies."""
import builtins,importlib.abc,json,os,sys,unittest
import unittest.mock
import numpy  # Initialize standard-library dependencies before the strict import fence.
from pathlib import Path

stage=Path(sys.argv[1]).resolve();root=Path(sys.argv[2]).resolve()
sys.dont_write_bytecode=True
sys.path[:0]=[str(stage),str(root)]
violations=[]
def refuse(reason):
    violations.append(reason)
    raise RuntimeError('Hardware-free test guard: '+reason)
class NoDriverImports(importlib.abc.MetaPathFinder):
    def find_spec(self,fullname,path=None,target=None):
        if fullname.split('.')[0] in {'serial','DM_CAN','socket','_socket'}:refuse('forbidden driver import '+fullname)
        return None
assert not any(n.split('.')[0] in {'serial','DM_CAN'} for n in sys.modules)
sys.meta_path.insert(0,NoDriverImports())
original_import=builtins.__import__
def fenced_import(name,*args,**kwargs):
    if name.split('.')[0] in {'serial','DM_CAN','socket','_socket'}:refuse('forbidden driver/network import '+name)
    return original_import(name,*args,**kwargs)
builtins.__import__=fenced_import
def audit(event,args):
    if event in {'subprocess.Popen','os.system','os.posix_spawn','socket.__new__','socket.connect','socket.bind','socket.getaddrinfo'}:
        refuse('forbidden process/network event '+event)
    if event=='open' and args and isinstance(args[0],(str,bytes,os.PathLike)):
        p=Path(os.fsdecode(args[0]))
        if str(p).startswith('/dev/') and p!=Path('/dev/null'):refuse('device open '+str(p))
        if p.name.startswith('DM_CAN') and p.suffix in {'.py','.pyc','.so'}:refuse('SDK source open '+str(p))
        mode=args[1] if len(args)>1 else None
        flags=args[2] if len(args)>2 and isinstance(args[2],int) else 0
        writing=(isinstance(mode,str) and any(c in mode for c in 'wax+')) or bool(flags&(os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC|os.O_APPEND))
        if writing and p!=Path('/dev/null'):
            resolved=p.resolve()
            if resolved!=stage and stage not in resolved.parents:refuse('write outside staging '+str(resolved))
sys.addaudithook(audit)
loader=unittest.defaultTestLoader
suite=loader.loadTestsFromNames(['test_left_arm_board_task_pose','test_left_arm_board_kinematics'])
result=unittest.TextTestRunner(verbosity=2).run(suite)
report={'ok':result.wasSuccessful() and not violations,'tests_run':result.testsRun,
        'failures':len(result.failures),'errors':len(result.errors),'guard_violations':violations,
        'serial_opened':False,'sdk_constructed':False,'motion_issued':False,
        'modules':['test_left_arm_board_task_pose','test_left_arm_board_kinematics'],
        'dependency_root':str(root),'staged_source_root':str(stage)}
print(json.dumps(report))
raise SystemExit(0 if report['ok'] else 1)
