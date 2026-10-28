import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock
from html.parser import HTMLParser

import chessboard_vision_v3_0_web_control as v30


class DynamicsWebTests(unittest.TestCase):
    def test_inline_board_hold_does_not_start_second_process(self):
        state=v30.OperationalRunState('unused.jsonl',v30.build_parser().parse_args([]))
        proc=object();state.proc=proc
        state.current={'action':'dynamics:board-grasp','command':['python','board.py','--inline-model-hold'],
                       'post_move_model_hold':{'state':'waiting-for-motion'}}
        state.last={'action':'dynamics:board-grasp','ok':True,'running':False}
        with (mock.patch.object(v30.v28.FullRunState,'_waiter'),
              mock.patch.object(v30.v28.FullRunState,'start') as start,
              mock.patch.object(v30,'build_dynamics_command') as build):
            state._waiter(proc)
        start.assert_not_called();build.assert_not_called()
        self.assertEqual(state.last['post_move_model_hold']['state'],'complete')
        self.assertTrue(state.last['post_move_model_hold']['same_serial_session'])

    def setUp(self):
        self.args = v30.build_parser().parse_args([])

    def test_generic_targets_and_jogs_get_model_without_action_name_allowlist(self):
        self.args.enable_execute = True
        self.args.rigid_body_model = 'data/general.json'
        self.args.rigid_body_mode = 'apply'
        for action, cmd in (
            ('new-user-point', ['python', v30.GENERAL_MOVE_SCRIPT, '--execute']),
            ('new-jog', ['python', self.args.arm_script, 'nudge-hold', '--joint', 'elbow']),
            ('new-saved-point', ['python', self.args.move_script, 'replay-move', '--name', 'unseen']),
        ):
            env = v30.motion_dynamics_popen_kwargs(self.args, action, cmd, {})['env']
            self.assertIn(v30.RIGID_MODEL_ENV, env)
            self.assertEqual(env[v30.MOTION_ACTION_ENV], action)
        for verb in ('status', 'restart', 'capture-home', 'hold'):
            env = v30.motion_dynamics_popen_kwargs(
                self.args, 'new-command', ['python', self.args.arm_script, verb], {})['env']
            self.assertNotIn(v30.RIGID_MODEL_ENV, env)

    def test_generic_target_builder_validates_and_uses_shared_takeover(self):
        self.args.rigid_body_model = 'data/general.json'
        self.args.rigid_body_mode = 'apply'
        cmd = v30.build_dynamics_command(self.args, 'move-target',
                                        {'targets_rad': {'wrist': -2.}, 'execute': True})
        self.assertIn(v30.GENERAL_MOVE_SCRIPT, cmd)
        self.assertIn('--execute', cmd)
        self.assertIn('--no-model-hold', cmd)
        self.assertTrue(v30.is_post_move_hold_action('dynamics:move-target'))
        for value in ({'claw': 0}, {'elbow': float('nan')}, {}):
            with self.assertRaises(ValueError):
                v30.build_dynamics_command(self.args, 'move-target', {'targets_rad': value})
        cmd = v30.build_dynamics_command(self.args, 'move-target', {'targets_rad': {'elbow': 0}})
        self.assertNotIn('--execute', cmd)
        self.assertIn('General Joint Target', v30.DYNAMICS_PAGE)

    def test_implicit_replay_execution_gets_models_and_unique_workflow_context(self):
        self.args.enable_execute=True
        self.args.motion_dynamics=True
        self.args.rigid_body_model='data/general.json'
        self.args.rigid_body_mode='apply'
        ids=[]
        cases=[(v30.v28.WHITE_BISHOP_PLACEMENT_ACTION,'--white-bishop-placement'),
               (v30.v28.WHITE_KNIGHT_PLACE_B1_ACTION,'--white-knight-place-b1'),
               (v30.V3_C4_ROOK_ACTION,'--white-rook-place-a1'),
               (v30.v28.PLACEMENT1_ACTION,'--placement1'),
               ('replay-move:bishop01',None)]
        for action,flag in cases:
            cmd=['python3','-u',self.args.move_script,'replay-move','--name','bishop01']
            if flag:cmd.append(flag)
            original=list(cmd)
            env=v30.motion_dynamics_popen_kwargs(self.args,action,cmd,{})['env']
            self.assertEqual(cmd,original)
            self.assertNotIn('--execute',cmd)
            self.assertEqual(env[v30.RIGID_MODEL_ENV],os.path.abspath(self.args.rigid_body_model))
            self.assertEqual(env[v30.MOTION_MODEL_ENV],os.path.abspath(self.args.dynamics_model))
            self.assertEqual(env[v30.MOTION_ACTION_ENV],action)
            self.assertRegex(env[v30.MOTION_RUN_ENV],r'^[0-9a-f]{16}$')
            ids.append(env[v30.MOTION_RUN_ENV])
        self.assertEqual(len(set(ids)),len(ids))
        self.args.enable_execute=False
        env=v30.motion_dynamics_popen_kwargs(self.args,cases[0][0],cmd,{})['env']
        self.assertNotIn(v30.RIGID_MODEL_ENV,env)

    def test_implicit_replay_requires_trusted_adapter_and_hold_clears_all_context(self):
        self.args.enable_execute=True
        self.args.motion_dynamics=True
        self.args.rigid_body_model='data/general.json'
        inherited={key:'stale' for key in (v30.MOTION_MODEL_ENV,v30.RIGID_MODEL_ENV,
                   v30.RIGID_MODE_ENV,v30.MOTION_ACTION_ENV,v30.MOTION_RUN_ENV)}
        for action,cmd in [('replay-move:bishop01',['python','/tmp/unrelated.py','replay-move']),
                           ('home',['python',self.args.arm_script,'home']),
                           ('replay-move:bishop01',['python',self.args.move_script,'replay-move','--dry-run']),
                           ('dynamics:model-hold-after:white-bishop-placement',['python','hold','--execute']),
                           ('dynamics:measure-inertia',['python','measure','--execute'])]:
            env=v30.motion_dynamics_popen_kwargs(self.args,action,cmd,{'env':inherited})['env']
            for key in inherited:self.assertNotIn(key,env)

    def test_replay_child_process_inherits_workflow_model_environment(self):
        with tempfile.TemporaryDirectory() as directory:
            script=os.path.join(directory,'move_adapter.py')
            child='import json,os; print(json.dumps({k:os.environ.get(k) for k in '+repr([v30.RIGID_MODEL_ENV,v30.MOTION_ACTION_ENV,v30.MOTION_RUN_ENV])+'}))'
            with open(script,'w',encoding='utf8') as handle:
                handle.write('import subprocess,sys\nsubprocess.run([sys.executable,"-c",'+repr(child)+'],check=True)\n')
            self.args.move_script=script
            self.args.enable_execute=True
            self.args.rigid_body_model='data/general.json'
            cmd=[sys.executable,'-u',script,'replay-move','--name','bishop01','--white-bishop-placement']
            kwargs=v30.motion_dynamics_popen_kwargs(self.args,'white-bishop-placement',cmd,{})
            result=subprocess.run(cmd,**kwargs,capture_output=True,text=True,check=True)
            row=json.loads(result.stdout)
            self.assertEqual(row[v30.RIGID_MODEL_ENV],os.path.abspath(self.args.rigid_body_model))
            self.assertEqual(row[v30.MOTION_ACTION_ENV],'white-bishop-placement')
            self.assertEqual(row[v30.MOTION_RUN_ENV],kwargs['env'][v30.MOTION_RUN_ENV])

    def test_rigid_body_environment_is_operational_only_and_preserves_arguments(self):
        self.args.rigid_body_model='data/general.json'
        self.args.rigid_body_mode='apply'
        inherited={v30.RIGID_MODEL_ENV:'stale.json',v30.RIGID_MODE_ENV:'apply','EXAMPLE':'kept'}
        supplied={'env':inherited,'cwd':'/controller'}
        for action in ('home','table-clearance',v30.V3_C4_ROOK_ACTION,'replay-move:bishop01'):
            env=v30.motion_dynamics_popen_kwargs(self.args,action,['python','--execute'],supplied)['env']
            self.assertEqual(env[v30.RIGID_MODEL_ENV],os.path.abspath(self.args.rigid_body_model))
            self.assertEqual(env[v30.RIGID_MODE_ENV],'apply')
            self.assertEqual(env['EXAMPLE'],'kept')
        for action in ('dynamics:model-hold-after:home','dynamics:model-hold','dynamics:measure-inertia','emergency-stop'):
            env=v30.motion_dynamics_popen_kwargs(self.args,action,['python','--execute'],supplied)['env']
            self.assertNotIn(v30.RIGID_MODEL_ENV,env)
            self.assertNotIn(v30.RIGID_MODE_ENV,env)
        env=v30.motion_dynamics_popen_kwargs(self.args,'home',['python','home'],supplied)['env']
        self.assertNotIn(v30.RIGID_MODEL_ENV,env)
        self.assertEqual(inherited[v30.RIGID_MODEL_ENV],'stale.json')

    def test_motion_dynamics_is_opt_in_and_does_not_leak_to_model_hold(self):
        inherited = {v30.MOTION_MODEL_ENV: 'stale-model.json', 'EXAMPLE': 'preserved'}
        supplied = {'env': inherited, 'cwd': '/controller'}
        original = dict(inherited)
        kwargs = v30.motion_dynamics_popen_kwargs(self.args, 'home', ['python', '--execute'], supplied)
        self.assertNotIn(v30.MOTION_MODEL_ENV, kwargs['env'])
        self.args.motion_dynamics = True
        for action in ('home', 'table-clearance', v30.V3_C4_ROOK_ACTION, 'replay-move:bishop01'):
            kwargs = v30.motion_dynamics_popen_kwargs(self.args, action, ['python', '--execute'], supplied)
            self.assertEqual(kwargs['env'][v30.MOTION_MODEL_ENV], os.path.abspath(self.args.dynamics_model))
            self.assertEqual(kwargs['env']['EXAMPLE'], 'preserved')
        for action in ('dynamics:model-hold-after:home', 'dynamics:model-hold',
                       'dynamics:measure-inertia', 'emergency-stop', 'nudge:wrist:+2deg'):
            kwargs = v30.motion_dynamics_popen_kwargs(self.args, action, ['python', '--execute'], supplied)
            self.assertNotIn(v30.MOTION_MODEL_ENV, kwargs['env'])
        kwargs = v30.motion_dynamics_popen_kwargs(self.args, 'home', ['python', 'home'], supplied)
        self.assertNotIn(v30.MOTION_MODEL_ENV, kwargs['env'])
        self.assertEqual(inherited, original)
        self.assertEqual(supplied['cwd'], '/controller')

    def test_shell_and_page_expose_dynamics_control(self):
        self.assertIn('id="dynamics-tab"', v30.HTML_PAGE)
        self.assertIn('src="/dynamics"', v30.HTML_PAGE)
        self.assertIn("Dynamics Control", v30.DYNAMICS_PAGE)
        self.assertIn("Enable actual motor commands", v30.DYNAMICS_PAGE)
        self.assertIn("/api/v3/dynamics/run", v30.DYNAMICS_PAGE)
        self.assertIn("copyText('output','Output')", v30.DYNAMICS_PAGE)
        self.assertIn("copyText('model','Model Data')", v30.DYNAMICS_PAGE)
        self.assertIn("navigator.clipboard.writeText", v30.DYNAMICS_PAGE)
        self.assertIn("Test Pose Setup", v30.DYNAMICS_PAGE)
        self.assertIn("Home Move", v30.DYNAMICS_PAGE)
        self.assertIn("Table Clearance Move", v30.DYNAMICS_PAGE)
        self.assertIn("Replay Selected Test Pose", v30.DYNAMICS_PAGE)
        self.assertIn("Run Inertia Step Test", v30.DYNAMICS_PAGE)
        self.assertIn("Reprocess Last Trace (No Motion)", v30.DYNAMICS_PAGE)
        self.assertIn("Remove Latest Sample (No Motion)", v30.DYNAMICS_PAGE)
        self.assertIn("Remove Latest Gravity Sample (No Motion)", v30.DYNAMICS_PAGE)
        self.assertIn("5.0 degrees (guarded)", v30.DYNAMICS_PAGE)
        self.assertIn("safe-wrist-nudge", v30.DYNAMICS_PAGE)
        self.assertIn('id="holdMaxVelocity" type="number" value="0.3"', v30.DYNAMICS_PAGE)
        self.assertIn('id="maxErrorDeg" type="number" value="2.5"', v30.DYNAMICS_PAGE)
        self.assertIn('id="autoModelHold" type="checkbox" checked', v30.DYNAMICS_PAGE)
        self.assertIn("/api/v3/model-hold/auto", v30.DYNAMICS_PAGE)
        self.assertIn('id="autoModelHoldOperational" type="checkbox" checked', v30.ARM_HTML_PAGE)
        self.assertIn("/api/action/", v30.DYNAMICS_PAGE)
        self.assertIn("/api/nudge", v30.DYNAMICS_PAGE)
        self.assertIn("/api/move/replay", v30.DYNAMICS_PAGE)
        for joint in v30.DYNAMICS_JOINT_ORDER:
            self.assertIn(joint, v30.DYNAMICS_PAGE)

    def run_browser_javascript(self, page, checks):
        node = shutil.which("node")
        if not node:
            self.skipTest("Node.js required for browser request lifecycle tests")
        source = "\n".join(re.findall(r"<script>(.*?)</script>", page, re.S))
        harness = r"""
const assert = require('node:assert/strict');
const vm = require('node:vm');
const elements = new Map(), events = [], requests = [], timers = new Map();
let nextTimer = 0;
function element(id) {
  if (!elements.has(id)) elements.set(id, {
    dataset: {src: '/'+id}, attrs: {}, value: '', hidden: true,
    appendChild() {},
    hasAttribute(key) { return Object.hasOwn(this.attrs, key); },
    setAttribute(key, value) { this.attrs[key]=value; events.push(['set',id,key,value]); },
    removeAttribute(key) { delete this.attrs[key]; events.push(['remove',id,key]); }
  });
  return elements.get(id);
}
element('vision').attrs.src = '/vision';
const context = vm.createContext({
  document: {getElementById: element, createElement: () => element('new-'+elements.size)},
  AbortController, console,
  setInterval() {},
  setTimeout(fn) { const id=++nextTimer; timers.set(id,fn); return id; },
  clearTimeout(id) { timers.delete(id); },
  fetch(url, options) {
    return new Promise((resolve, reject) => {
      requests.push({url, options, resolve});
      options.signal.addEventListener('abort', () => reject(new Error('aborted')));
    });
  }
});
"""
        script = harness + "\nvm.runInContext(" + json.dumps(source) + ", context);\n"
        script += "(async () => {\n" + checks + "\n})().catch(error => {console.error(error); process.exitCode=1;});"
        result = subprocess.run([node, "-"], input=script, text=True,
                                capture_output=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_shell_loads_one_panel_and_releases_old_stream_documents(self):
        class Frames(HTMLParser):
            def __init__(self):
                super().__init__()
                self.frames = []
            def handle_starttag(self, tag, attrs):
                if tag == "iframe":
                    self.frames.append(dict(attrs))
        parsed = Frames()
        parsed.feed(v30.HTML_PAGE)
        self.assertEqual(len(parsed.frames), 4)
        self.assertEqual([frame["id"] for frame in parsed.frames if "src" in frame], ["vision"])
        self.run_browser_javascript(v30.HTML_PAGE, r"""
const panels = ['vision','arm','crown','dynamics'];
for (const name of ['dynamics','arm','crown','vision','dynamics']) {
  events.length = 0;
  context.showPanel(name);
  assert.deepEqual(panels.filter(id => element(id).hasAttribute('src')), [name]);
  assert.equal(element(name).attrs.src, '/'+name);
  assert.equal(element(name).hidden, false);
  assert.equal(element('open').href, '/'+name);
  const firstLoad = events.findIndex(e => e[0]==='set' && e[2]==='src');
  assert(events.filter(e => e[0]==='remove').every(e => events.indexOf(e)<firstLoad));
}
events.length = 0;
context.showPanel('dynamics');
assert(!events.some(e => e[2]==='src')); // same tab must not reload
context.showPanel('unknown');
assert.deepEqual(panels.filter(id => element(id).hasAttribute('src')), ['dynamics']);
assert.equal(requests.length, 0); // no command submission on tab switch
""")

    def test_dynamics_polling_does_not_queue_and_recovers_after_timeout(self):
        self.run_browser_javascript(v30.DYNAMICS_PAGE, r"""
assert.equal(requests.length, 2); // startup status and saved moves
await context.refresh(); await context.refresh();
await context.loadTestMoves(); await context.loadTestMoves();
assert.equal(requests.length, 2); // slow requests cannot overlap
requests[0].resolve({ok:true, json:async()=>({moves:[]})});
requests[1].resolve({ok:true, json:async()=>({execution_enabled:true, model:{}})});
await new Promise(setImmediate);
assert.equal(timers.size, 0);
const pending = context.refresh();
assert.equal(requests.length, 3);
assert.equal(timers.size, 1);
[...timers.values()][0](); // status request timeout
await pending;
assert(requests[2].options.signal.aborted);
assert.equal(timers.size, 0);
const retry = context.refresh();
assert.equal(requests.length, 4);
requests[3].resolve({ok:true, json:async()=>({model:{}, auto_model_hold_enabled:true})});
await retry;
assert.equal(element('autoModelHold').checked, true);
assert(requests.every(r => !r.options.method || r.options.method==='GET'));
""")

    def test_default_paths_point_to_new_independent_controller(self):
        self.assertTrue(self.args.dynamics_script.endswith("left_arm_v3_1_model_control.py"))
        self.assertTrue(self.args.dynamics_model.endswith("left_arm_v3_1_dynamics.json"))
        self.assertTrue(os.path.isfile(self.args.dynamics_model))

    def test_capture_gravity_command_is_allowlisted(self):
        command = v30.build_dynamics_command(self.args, "capture-gravity", {
            "joints": "elbow,wrist",
            "direction": "negative",
            "samples": 40,
            "interval": 0.03,
        })
        self.assertEqual(command[0], self.args.python_bin)
        self.assertIn(os.path.abspath(self.args.dynamics_script), command)
        self.assertIn("capture-gravity", command)
        self.assertEqual(command[command.index("--joints") + 1], "elbow,wrist")
        self.assertEqual(command[command.index("--direction") + 1], "negative")

    def test_rejects_unknown_action_and_joint(self):
        with self.assertRaisesRegex(ValueError, "unsupported"):
            v30.build_dynamics_command(self.args, "shell", {})
        with self.assertRaisesRegex(ValueError, "unknown joints"):
            v30.build_dynamics_command(self.args, "preview", {"joints": "elbow;whoami"})

    def test_unit_verification_is_explicit(self):
        with self.assertRaisesRegex(ValueError, "explicit"):
            v30.build_dynamics_command(self.args, "set-units-verified", {})
        command = v30.build_dynamics_command(
            self.args, "set-units-verified", {"confirmed": True}
        )
        self.assertEqual(command[-1], "set-units-verified")

    def test_model_hold_requires_explicit_checkbox_value(self):
        with self.assertRaisesRegex(ValueError, "actual motor"):
            v30.build_dynamics_command(self.args, "model-hold", {
                "joints": "elbow", "seconds": 2,
            })
        command = v30.build_dynamics_command(self.args, "model-hold", {
            "joints": "elbow", "seconds": 2, "execute": True,
        })
        self.assertEqual(command[-1], "--execute")
        self.assertEqual(command[command.index("--seconds") + 1], "2.0")

    def test_inertia_step_requires_execution_and_uses_guarded_values(self):
        payload = {
            "joint": "elbow", "step_deg": 2, "test_kp": 40, "test_kd": 3,
            "settle_seconds": 1.5, "test_seconds": 1.2, "return_seconds": 1.5,
            "control_dt": 0.01, "max_velocity": 1.0,
            "max_position_error_deg": 5.0,
        }
        with self.assertRaisesRegex(ValueError, "actual motor"):
            v30.build_dynamics_command(self.args, "measure-inertia", payload)
        payload["execute"] = True
        command = v30.build_dynamics_command(self.args, "measure-inertia", payload)
        self.assertEqual(command[-1], "--execute")
        self.assertEqual(command[command.index("--joint") + 1], "elbow")
        self.assertEqual(command[command.index("--step-deg") + 1], "2.0")

    def test_reprocess_inertia_never_requests_execution(self):
        command = v30.build_dynamics_command(
            self.args, "reprocess-inertia", {"joint": "elbow", "label": "retry"}
        )
        self.assertIn("reprocess-inertia", command)
        self.assertNotIn("--execute", command)

    def test_safe_wrist_nudge_is_fixed_low_gain_and_zero_torque(self):
        command = v30.build_dynamics_command(
            self.args, "safe-wrist-nudge", {"direction": 1}
        )
        self.assertIn("guarded-nudge", command)
        self.assertIn(os.path.abspath(self.args.dynamics_script), command)
        self.assertEqual(command[command.index("--joint") + 1], "wrist")
        self.assertEqual(command[command.index("--step-deg") + 1], "5.0")
        self.assertEqual(command[command.index("--seconds") + 1], "6.0")
        self.assertEqual(command[command.index("--test-kp") + 1], "6.0")
        self.assertEqual(command[command.index("--test-kd") + 1], "0.8")
        self.assertEqual(command[-1], "--execute")
        self.assertEqual(command[command.index("--max-velocity") + 1], "0.3")
        self.assertEqual(command[command.index("--max-gravity-distance-deg") + 1], "5.0")

        with self.assertRaisesRegex(ValueError, "direction"):
            v30.build_dynamics_command(
                self.args, "safe-wrist-nudge", {"direction": 0}
            )

    def test_remove_latest_inertia_never_requests_execution(self):
        command = v30.build_dynamics_command(
            self.args, "remove-latest-inertia", {"joint": "elbow"}
        )
        self.assertIn("remove-latest-inertia", command)
        self.assertNotIn("--execute", command)

    def test_remove_latest_gravity_never_requests_execution(self):
        command = v30.build_dynamics_command(
            self.args, "remove-latest-gravity", {"joint": "shoulder_side"}
        )
        self.assertIn("remove-latest-gravity", command)
        self.assertEqual(command[command.index("--joint") + 1], "shoulder_side")
        self.assertNotIn("--execute", command)

    def test_pose_is_parsed_and_serialized_without_shell(self):
        command = v30.build_dynamics_command(self.args, "preview", {
            "joints": ["elbow"], "pose": '{"elbow": 0.5}',
        })
        pose = json.loads(command[command.index("--pose-json") + 1])
        self.assertEqual(pose, {"elbow": 0.5})

    def test_operational_motion_actions_are_followed_by_model_hold(self):
        for action in (
            "home", "table-clearance", "replay-move:bishop01",
            v30.V3_C4_BISHOP_ACTION, v30.V3_C4_KNIGHT_ACTION,
            v30.V3_C4_ROOK_ACTION,
        ):
            self.assertTrue(v30.is_post_move_hold_action(action), action)
        for action in ("emergency-stop", "claw-home", "dynamics:model-hold"):
            self.assertFalse(v30.is_post_move_hold_action(action), action)

    def test_post_move_hold_uses_validated_all_joint_limits(self):
        command = v30.build_dynamics_command(
            self.args, "model-hold", dict(v30.POST_MOVE_HOLD_PAYLOAD)
        )
        self.assertEqual(
            command[command.index("--joints") + 1],
            ",".join(v30.DYNAMICS_JOINT_ORDER),
        )
        self.assertEqual(command[command.index("--seconds") + 1], "5.0")
        self.assertEqual(command[command.index("--max-velocity") + 1], "0.3")
        self.assertEqual(
            command[command.index("--velocity-spike-max-drift-deg") + 1], "0.1"
        )
        self.assertEqual(command[command.index("--max-error-deg") + 1], "2.5")
        self.assertEqual(
            command[command.index("--preflight-still-timeout") + 1], "6.0"
        )
        self.assertEqual(
            command[command.index("--preflight-still-seconds") + 1], "0.5"
        )
        self.assertEqual(
            command[command.index("--preflight-max-velocity") + 1], "0.25"
        )
        self.assertEqual(
            command[command.index("--preflight-max-drift-deg") + 1], "0.25"
        )
        self.assertEqual(command[-1], "--execute")

    def test_model_hold_can_reuse_verified_existing_motor_enable(self):
        payload = dict(v30.POST_MOVE_HOLD_PAYLOAD)
        payload["motors_already_enabled"] = True
        command = v30.build_dynamics_command(self.args, "model-hold", payload)
        self.assertIn("--motors-already-enabled", command)
        self.assertEqual(command[-1], "--execute")

    def test_enabled_joint_detection_requires_exact_enable_lines(self):
        with tempfile.TemporaryDirectory() as directory:
            full_log = os.path.join(directory, "motion.txt")
            with open(full_log, "w", encoding="utf-8") as stream:
                for joint in v30.DYNAMICS_JOINT_ORDER:
                    stream.write(f"enable {joint}\n")
                stream.write("enable skip shoulder_rotate\n")
            enabled = v30.enabled_joints_from_completed_run({
                "stdout": "truncated tail",
                "full_log_path": full_log,
            })
        self.assertEqual(enabled, set(v30.DYNAMICS_JOINT_ORDER))

    def test_operational_run_state_toggle_defaults_enabled(self):
        state = v30.OperationalRunState("test-runs.jsonl", self.args)
        self.assertTrue(state.get_auto_hold_enabled())
        self.assertFalse(state.set_auto_hold_enabled(False))
        self.assertFalse(state.get_auto_hold_enabled())

    def test_successful_motion_is_sequenced_into_model_hold(self):
        with tempfile.TemporaryDirectory() as directory:
            state = v30.OperationalRunState(
                os.path.join(directory, "runs.jsonl"), self.args
            )
            motion = [sys.executable, "-c", "print('motion complete')"]
            hold = [sys.executable, "-c", "print('hold complete')"]
            processes = []
            real_popen = v30.v28.arm.subprocess.Popen

            def tracked_popen(*args, **kwargs):
                process = real_popen(*args, **kwargs)
                processes.append(process)
                return process

            with (
                mock.patch.object(
                    v30, "enabled_joints_from_completed_run",
                    return_value=set(v30.DYNAMICS_JOINT_ORDER),
                ),
                mock.patch.object(
                    v30, "build_dynamics_command", return_value=hold
                ) as build_hold,
                mock.patch.object(
                    v30.v28.arm.subprocess, "Popen", side_effect=tracked_popen
                ),
            ):
                started = state.start("home", motion, popen_kwargs={})
                self.assertEqual(
                    started["post_move_model_hold"]["state"],
                    "waiting-for-motion",
                )
                deadline = time.time() + 5.0
                finished = None
                while time.time() < deadline:
                    finished = state.snapshot()
                    if (
                        finished
                        and not finished.get("running")
                        and str(finished.get("action", "")).startswith(
                            "dynamics:model-hold-after:"
                        )
                    ):
                        break
                    time.sleep(0.02)
            self.assertIsNotNone(finished)
            self.assertTrue(finished["ok"])
            self.assertEqual(finished["post_move_model_hold"]["state"], "complete")
            self.assertEqual(finished["preceding_motion"]["action"], "home")
            self.assertTrue(finished["preceding_motion"]["ok"])
            self.assertEqual(
                finished["preceding_motion"]["post_move_model_hold"]["state"],
                "motion-complete",
            )
            hold_payload = build_hold.call_args.args[2]
            self.assertTrue(hold_payload["motors_already_enabled"])
            self.assertEqual(
                finished["post_move_model_hold"]["motor_enable_mode"],
                "reused-existing-enable",
            )
            for process in processes:
                if process.stdout is not None:
                    process.stdout.close()
                if process.stderr is not None:
                    process.stderr.close()

    def test_failed_motion_does_not_start_model_hold(self):
        with tempfile.TemporaryDirectory() as directory:
            state = v30.OperationalRunState(
                os.path.join(directory, "runs.jsonl"), self.args
            )
            failed_motion = [sys.executable, "-c", "raise SystemExit(7)"]
            processes = []
            real_popen = v30.v28.arm.subprocess.Popen

            def tracked_popen(*args, **kwargs):
                process = real_popen(*args, **kwargs)
                processes.append(process)
                return process

            with (
                mock.patch.object(v30, "build_dynamics_command") as build_hold,
                mock.patch.object(
                    v30.v28.arm.subprocess, "Popen", side_effect=tracked_popen
                ),
            ):
                state.start("table-clearance", failed_motion, popen_kwargs={})
                deadline = time.time() + 5.0
                finished = None
                while time.time() < deadline:
                    finished = state.snapshot()
                    # The process waiter publishes its exit first, then the
                    # operational waiter records why the hold was skipped.
                    if (finished and not finished.get("running")
                            and finished.get("post_move_model_hold", {}).get("state")
                            == "skipped-motion-failed"):
                        break
                    time.sleep(0.02)
            self.assertIsNotNone(finished)
            self.assertFalse(finished["ok"])
            self.assertEqual(finished["action"], "table-clearance")
            self.assertEqual(finished["returncode"], 7)
            self.assertEqual(
                finished["post_move_model_hold"]["state"],
                "skipped-motion-failed",
            )
            build_hold.assert_not_called()
            for process in processes:
                if process.stdout is not None:
                    process.stdout.close()
                if process.stderr is not None:
                    process.stderr.close()


if __name__ == "__main__":
    unittest.main()
