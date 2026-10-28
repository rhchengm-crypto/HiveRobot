#!/usr/bin/env python3
"""HiveRobot v3.0: confirm a C4 vision result before a trained multi-flow.

This module composes the existing v2.8 controller.  It does not modify the
v2.7/v2.8 implementations, trained moves, calibration, or learned data.
"""
from __future__ import annotations

import json
import os
import secrets
import signal
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from http import HTTPStatus
from http.server import ThreadingHTTPServer
from pathlib import Path

import chessboard_vision_v2_8_web_control as v28


WEB_VERSION = "v3.0-c4-confirmed-dynamics-takeover"
DEFAULT_PORT = 8100
CONFIRMATION_TTL_SECONDS = 120.0
V28_MAKE_HANDLER = v28.make_handler
V30_MOVE_SCRIPT = str(Path(__file__).parent / "left_arm_v3_0_move_library.py")
V31_DYNAMICS_SCRIPT = str(Path(__file__).parent / "left_arm_v3_1_model_control.py")
GENERAL_MOVE_SCRIPT = str(Path(__file__).parent / "left_arm_general_move.py")
BOARD_GRASP_SCRIPT = str(Path(__file__).parent / "left_arm_board_grasp.py")
BOARD_GRASP_MODEL = str(Path(__file__).parent / "data" / "left_arm_board_grasp.json")
_V31_MODEL_IN_DATA = Path(__file__).parent / "data" / "left_arm_v3_1_dynamics.json"
_V31_MODEL_NEXT_TO_SCRIPT = Path(__file__).parent / "left_arm_v3_1_dynamics.json"
V31_DYNAMICS_MODEL = str(
    _V31_MODEL_IN_DATA
    if _V31_MODEL_IN_DATA.is_file() or not _V31_MODEL_NEXT_TO_SCRIPT.is_file()
    else _V31_MODEL_NEXT_TO_SCRIPT
)
V3_C4_BISHOP_ACTION = "v3-c4-white-bishop-placement"
V3_C4_KNIGHT_ACTION = "v3-c4-white-knight-place-b1"
V3_C4_ROOK_ACTION = "v3-c4-white-rook-place-a1"

DYNAMICS_ACTIONS = {
    "init-model", "show-model", "set-units-verified", "capture-gravity",
    "add-inertia", "measure-inertia", "reprocess-inertia", "remove-latest-inertia",
    "remove-latest-gravity",
    "configure", "configure-friction", "preview", "model-hold", "safe-wrist-nudge", "move-target", "board-grasp",
}
DYNAMICS_JOINT_ORDER = (
    "shoulder_front", "shoulder_side", "shoulder_rotate", "elbow",
    "arm_roll", "wrist_side", "wrist",
)
DYNAMICS_JOINTS = set(DYNAMICS_JOINT_ORDER)
POST_MOVE_HOLD_SECONDS = 5.0
MOTION_MODEL_ENV = "HIVE_ROBOT_MOTION_MODEL"
RIGID_MODEL_ENV = "HIVE_ROBOT_RIGID_BODY_MODEL"
RIGID_MODE_ENV = "HIVE_ROBOT_RIGID_BODY_MODE"
MOTION_ACTION_ENV = "HIVE_ROBOT_MOTION_ACTION"
MOTION_RUN_ENV = "HIVE_ROBOT_MOTION_RUN_ID"
POST_MOVE_HOLD_PAYLOAD = {
    "joints": ",".join(DYNAMICS_JOINT_ORDER),
    "seconds": POST_MOVE_HOLD_SECONDS,
    "control_dt": 0.01,
    "takeover_kp": 2.0,
    "takeover_kd": 0.4,
    "max_velocity": 0.3,
    "velocity_spike_max_drift_deg": 0.1,
    "max_error_deg": 2.5,
    "preflight_still_timeout": 6.0,
    "preflight_still_seconds": 0.5,
    "preflight_max_velocity": 0.25,
    "preflight_max_drift_deg": 0.25,
    "log_interval": 0.25,
    "execute": True,
}


def is_post_move_hold_action(action: str) -> bool:
    """Return whether a completed legacy motion should hand off to Model Hold."""
    exact_actions = {
        "home", "table-clearance", "dynamics:move-target", "dynamics:board-grasp",
        getattr(v28, "PLACEMENT1_ACTION", ""),
        getattr(v28, "PLACEMENT_A5_ACTION", ""),
        getattr(v28, "PLACEMENT_C4_ACTION", ""),
        getattr(v28, "WHITE_BISHOP_PLACEMENT_ACTION", ""),
        getattr(v28, "WHITE_KNIGHT_PLACE_B1_ACTION", ""),
        V3_C4_BISHOP_ACTION, V3_C4_KNIGHT_ACTION, V3_C4_ROOK_ACTION,
    }
    return action in exact_actions or action.startswith("replay-move:")


def is_executing_operational_command(args, action, cmd):
    if any(flag in cmd for flag in ("--dry-run", "--no-execute")):
        return False
    # Keep legacy explicit operational commands compatible. Generic commands
    # below are selected by adapter + motion verb, never by saved point name.
    if is_post_move_hold_action(action) and "--execute" in cmd:
        return True
    if not getattr(args, "enable_execute", False):
        return False
    arm_script = os.path.abspath(getattr(args, "arm_script", ""))
    for index, token in enumerate(cmd):
        if index and os.path.abspath(str(cmd[index-1])) == arm_script:
            if token in {"nudge", "nudge-hold"}:
                return True  # Legacy point jogs execute without --execute.
            if token in {"home", "clearance"}:
                return "--execute" in cmd
        if index and os.path.abspath(str(cmd[index-1])) in {
                os.path.abspath(GENERAL_MOVE_SCRIPT), os.path.abspath(BOARD_GRASP_SCRIPT)}:
            return "--execute" in cmd
    # The saved-move CLI executes replay-move without an --execute flag.
    # Web's execution gate is still required; recognize only configured move
    # adapters, not arbitrary commands whose action label resembles a replay.
    if not getattr(args, "enable_execute", False) or "replay-move" not in cmd:
        return False
    index = cmd.index("replay-move")
    if index < 1:
        return False
    scripts = {os.path.abspath(V30_MOVE_SCRIPT),
               os.path.abspath(getattr(args, "move_script", V30_MOVE_SCRIPT))}
    return os.path.abspath(cmd[index-1]) in scripts


def motion_dynamics_popen_kwargs(args, action, cmd, popen_kwargs):
    """Opt in only operational trajectories, never identification/Model Hold."""
    result = dict(popen_kwargs)
    environment = dict(result.get("env") if result.get("env") is not None else os.environ)
    environment.pop(MOTION_MODEL_ENV, None)
    environment.pop(RIGID_MODEL_ENV, None)
    environment.pop(RIGID_MODE_ENV, None)
    environment.pop(MOTION_ACTION_ENV, None)
    environment.pop(MOTION_RUN_ENV, None)
    executing = is_executing_operational_command(args, action, cmd)
    if (getattr(args, "motion_dynamics", False)
            and executing):
        environment[MOTION_MODEL_ENV] = os.path.abspath(args.dynamics_model)
    if (getattr(args, "rigid_body_model", None)
            and executing):
        environment[RIGID_MODEL_ENV] = os.path.abspath(args.rigid_body_model)
        environment[RIGID_MODE_ENV] = args.rigid_body_mode
    if executing and (MOTION_MODEL_ENV in environment or RIGID_MODEL_ENV in environment):
        environment[MOTION_ACTION_ENV] = str(action)
        environment[MOTION_RUN_ENV] = secrets.token_hex(8)
    result["env"] = environment
    return result


def enabled_joints_from_completed_run(run: dict) -> set[str]:
    """Read exact motor-enable events from a completed legacy motion log."""
    output = str(run.get("stdout", ""))
    full_log_path = run.get("full_log_path")
    if full_log_path:
        try:
            output = Path(str(full_log_path)).read_text(encoding="utf-8")
        except (OSError, UnicodeError):
            # The bounded stdout tail remains a safe fallback: failure to find
            # every joint simply keeps the normal explicit-enable path.
            pass
    enabled = set()
    for line in output.splitlines():
        fields = line.strip().split()
        if len(fields) == 2 and fields[0] == "enable" and fields[1] in DYNAMICS_JOINTS:
            enabled.add(fields[1])
    return enabled


def board_hold_motion_gains(run):
    if run.get('action') != 'dynamics:board-grasp':
        return None
    output = str(run.get('stdout',''))
    if run.get('full_log_path'):
        try:
            output = Path(run['full_log_path']).read_text(encoding='utf8')
        except (OSError,UnicodeError):
            pass
    for line in reversed(output.splitlines()):
        if line.startswith('board endpoint report= '):
            data = json.loads(line.split('= ',1)[1]).get('model_hold_motion_gains')
            if data:
                from left_arm_v3_1_model_control import motion_hold_gain_floor
                return motion_hold_gain_floor(json.dumps(data),DYNAMICS_JOINT_ORDER)
            return None
    return None


def _finite_number(payload: dict, name: str, *, minimum=None, maximum=None,
                   required: bool = False):
    value = payload.get(name)
    if value in (None, ""):
        if required:
            raise ValueError(f"{name} is required")
        return None
    number = float(value)
    if not (-float("inf") < number < float("inf")):
        raise ValueError(f"{name} must be finite")
    if minimum is not None and number < minimum:
        raise ValueError(f"{name} must be at least {minimum}")
    if maximum is not None and number > maximum:
        raise ValueError(f"{name} must be at most {maximum}")
    return number


def _joints_arg(payload: dict) -> str:
    raw = payload.get("joints", "")
    names = raw if isinstance(raw, list) else str(raw).split(",")
    names = [str(name).strip() for name in names if str(name).strip()]
    if not names:
        names = list(DYNAMICS_JOINT_ORDER)
    unknown = sorted(set(names) - DYNAMICS_JOINTS)
    if unknown:
        raise ValueError("unknown joints: " + ", ".join(unknown))
    return ",".join(names)


def _joint_arg(payload: dict) -> str:
    joint = str(payload.get("joint", "")).strip()
    if joint not in DYNAMICS_JOINTS:
        raise ValueError("a valid joint is required")
    return joint


def _pose_json_arg(payload: dict) -> str | None:
    pose = payload.get("pose")
    if pose in (None, ""):
        return None
    if isinstance(pose, str):
        pose = json.loads(pose)
    if not isinstance(pose, dict) or not pose:
        raise ValueError("pose must be a non-empty JSON object")
    cleaned = {}
    for joint, value in pose.items():
        if joint not in DYNAMICS_JOINTS:
            raise ValueError(f"unknown pose joint: {joint}")
        cleaned[joint] = float(value)
    return json.dumps(cleaned, separators=(",", ":"), sort_keys=True)


def build_dynamics_command(args, action: str, payload: dict) -> list[str]:
    """Build a shell-free, allowlisted v3.1 controller command."""
    if action not in DYNAMICS_ACTIONS:
        raise ValueError("unsupported dynamics action")
    if action == "board-grasp":
        if payload.get("execute") is True and payload.get("stage", "above") not in {"above", "touch", "approach", "cartesian-check"}:
            raise PermissionError("Only Above, staged Touch and model Approach + Touch are released for hardware validation")
        if not getattr(args, "rigid_body_model", None):
            raise ValueError("board-grasp requires --rigid-body-model")
        x = _finite_number(payload, "x_mm", minimum=0, maximum=440, required=True)
        y = _finite_number(payload, "y_mm", minimum=0, maximum=440, required=True)
        stage = str(payload.get("stage", "above"))
        if stage not in {"above", "touch", "approach", "cartesian-check", "grasp"}:
            raise ValueError("unknown board-grasp stage")
        piece = str(payload.get("piece", "white_bishop"))
        if piece not in {"white_bishop", "white_knight", "white_rook"}:
            raise ValueError("piece has no board grasp profile")
        command = [str(args.python_bin), "-u", BOARD_GRASP_SCRIPT,
                   "--model", os.path.abspath(args.rigid_body_model),
                   "--board-model", BOARD_GRASP_MODEL,
                   "--hold-model", os.path.abspath(args.dynamics_model),
                   "--mode", args.rigid_body_mode, "--x-mm", str(x), "--y-mm", str(y),
                   "--piece", piece, "--stage", stage, "--no-model-hold"]
        height = _finite_number(payload, "grip_height_mm", minimum=10, maximum=150)
        if height is not None:
            command += ["--grip-height-mm", str(height)]
        if payload.get("execute") is True:
            command.append("--execute")
        if payload.get("test_protective_stop") is True:
            command.append("--test-protective-stop")
        if payload.get("direction_check_joint") is not None:
            joint = str(payload["direction_check_joint"])
            if stage != "cartesian-check" or joint not in {"shoulder_side", "shoulder_rotate", "wrist_side"}:
                raise ValueError("invalid isolated-axis direction check")
            step = _finite_number(payload, "direction_check_step_deg", minimum=-3, maximum=3, required=True)
            if step == 0 or payload.get("test_protective_stop") is True:
                raise ValueError("direction check requires a nonzero step without fault injection")
            command += ["--direction-check-joint", joint, "--direction-check-step-deg", str(step)]
        return command
    if action == "move-target":
        from left_arm_general_move import clean_targets
        targets = payload.get("targets_rad")
        if isinstance(targets, str):
            targets = json.loads(targets)
        targets = clean_targets(targets)
        if not getattr(args, "rigid_body_model", None):
            raise ValueError("move-target requires --rigid-body-model")
        command = [str(args.python_bin), "-u", GENERAL_MOVE_SCRIPT,
                   "--model", os.path.abspath(args.rigid_body_model),
                   "--mode", args.rigid_body_mode,
                   "--targets-json", json.dumps(targets, allow_nan=False),
                   "--no-model-hold"]
        for key, option, low, high in (
            ("seconds", "--seconds", 1.0, 120.0),
            ("max_delta_deg", "--max-delta-deg", 0.1, 180.0),
            ("max_velocity", "--max-velocity", 0.01, 2.0),
            ("max_error_deg", "--max-error-deg", 0.1, 10.0),
        ):
            value = _finite_number(payload, key, minimum=low, maximum=high)
            if value is not None:
                command += [option, str(value)]
        if payload.get("execute") is True:
            command.append("--execute")
        return command
    if action == "safe-wrist-nudge":
        direction_value = _finite_number(
            payload, "direction", minimum=-1, maximum=1, required=True
        )
        if direction_value not in {-1.0, 1.0}:
            raise ValueError("direction must be -1 or 1")
        direction = int(direction_value)
        return [
            str(args.python_bin), "-u", os.path.abspath(args.dynamics_script),
            "--model", os.path.abspath(args.dynamics_model), "guarded-nudge",
            "--joint", "wrist", "--step-deg", str(5.0 * direction),
            "--seconds", "6.0", "--test-kp", "6.0", "--test-kd", "0.8",
            "--settle-seconds", "1.5", "--control-dt", "0.01",
            "--max-velocity", "0.3", "--max-position-error-deg", "2.0",
            "--max-gravity-distance-deg", "5.0", "--execute",
        ]
    command = [
        str(args.python_bin), "-u", os.path.abspath(args.dynamics_script),
        "--model", os.path.abspath(args.dynamics_model), action,
    ]
    if action in {"init-model", "show-model"}:
        return command
    if action == "set-units-verified":
        if payload.get("confirmed") is not True:
            raise ValueError("explicit MIT gain and torque unit confirmation is required")
        return command
    if action == "capture-gravity":
        direction = str(payload.get("direction", "stationary"))
        if direction not in {"positive", "negative", "stationary"}:
            raise ValueError("invalid direction")
        command += ["--joints", _joints_arg(payload), "--direction", direction]
        for key, option, low, high in (
            ("samples", "--samples", 3, 1000),
            ("interval", "--interval", 0.001, 2.0),
            ("max_velocity", "--max-velocity", 0.001, 2.0),
        ):
            value = _finite_number(payload, key, minimum=low, maximum=high)
            if value is not None:
                if key == "samples" and not value.is_integer():
                    raise ValueError("samples must be an integer")
                command += [option, str(int(value) if key == "samples" else value)]
        return command
    if action == "add-inertia":
        command += ["--joint", _joint_arg(payload)]
        pose_json = _pose_json_arg(payload)
        if payload.get("from_arm") is True:
            command.append("--from-arm")
        elif pose_json:
            command += ["--pose-json", pose_json]
        else:
            raise ValueError("select current arm pose or provide pose JSON")
        effective = _finite_number(payload, "effective_inertia", minimum=1e-6, maximum=100.0)
        test_kp = _finite_number(payload, "test_kp", minimum=1e-6, maximum=10000.0)
        measured = _finite_number(payload, "measured_omega_n", minimum=1e-6, maximum=1000.0)
        if effective is not None:
            command += ["--effective-inertia", str(effective)]
        elif test_kp is not None and measured is not None:
            command += ["--test-kp", str(test_kp), "--measured-omega-n", str(measured)]
        else:
            raise ValueError("provide effective inertia or both test kp and measured omega_n")
        label = str(payload.get("label", "")).strip()
        if label:
            command += ["--label", label[:120]]
        return command
    if action == "measure-inertia":
        command += ["--joint", _joint_arg(payload)]
        for key, option, low, high in (
            ("step_deg", "--step-deg", -3.0, 3.0),
            ("test_kp", "--test-kp", 1.0, 70.0),
            ("test_kd", "--test-kd", 0.1, 5.0),
            ("settle_seconds", "--settle-seconds", 0.5, 10.0),
            ("test_seconds", "--test-seconds", 0.4, 5.0),
            ("return_seconds", "--return-seconds", 0.5, 10.0),
            ("control_dt", "--control-dt", 0.005, 0.05),
            ("max_velocity", "--max-velocity", 0.1, 2.0),
            ("max_position_error_deg", "--max-position-error-deg", 1.0, 10.0),
        ):
            value = _finite_number(payload, key, minimum=low, maximum=high, required=True)
            command += [option, str(value)]
        if abs(float(payload["step_deg"])) < 0.25:
            raise ValueError("step_deg magnitude must be at least 0.25")
        label = str(payload.get("label", "")).strip()
        if label:
            command += ["--label", label[:120]]
        if payload.get("execute") is not True:
            raise ValueError("Enable actual motor commands before inertia testing")
        command.append("--execute")
        return command
    if action == "reprocess-inertia":
        command += ["--joint", _joint_arg(payload)]
        label = str(payload.get("label", "")).strip()
        if label:
            command += ["--label", label[:120]]
        return command
    if action == "remove-latest-inertia":
        command += ["--joint", _joint_arg(payload)]
        return command
    if action == "remove-latest-gravity":
        command += ["--joint", _joint_arg(payload)]
        return command
    if action == "configure":
        command += ["--joint", _joint_arg(payload)]
        added = 0
        for key in ("omega_n", "zeta", "kp_min", "kp_max", "kd_min", "kd_max"):
            maximum = 5.0 if key in {"kd_min", "kd_max"} else 10000.0
            value = _finite_number(payload, key, minimum=0.0, maximum=maximum)
            if value is not None:
                command += ["--" + key.replace("_", "-"), str(value)]
                added += 1
        if not added:
            raise ValueError("at least one scheduler value is required")
        return command
    if action == "configure-friction":
        command += ["--joint", _joint_arg(payload)]
        for key in ("coulomb", "viscous"):
            value = _finite_number(payload, key, minimum=0.0, maximum=100.0, required=True)
            command += ["--" + key, str(value)]
        return command
    if action == "preview":
        command += ["--joints", _joints_arg(payload)]
        pose_json = _pose_json_arg(payload)
        if pose_json:
            command += ["--pose-json", pose_json]
        return command
    if action == "model-hold":
        if payload.get("execute") is not True:
            raise ValueError("Enable actual motor commands before Model Hold")
        command += ["--joints", _joints_arg(payload)]
        seconds = _finite_number(payload, "seconds", minimum=0.1, maximum=120.0) or 5.0
        command += ["--seconds", str(seconds)]
        for key, option, low, high in (
            ("control_dt", "--control-dt", 0.002, 0.1),
            ("takeover_kp", "--takeover-kp", 0.0, 100.0),
            ("takeover_kd", "--takeover-kd", 0.0, 5.0),
            ("max_velocity", "--max-velocity", 0.01, 10.0),
            ("velocity_spike_max_drift_deg", "--velocity-spike-max-drift-deg", 0.01, 1.0),
            ("max_error_deg", "--max-error-deg", 0.1, 90.0),
            ("preflight_still_timeout", "--preflight-still-timeout", 0.0, 30.0),
            ("preflight_still_seconds", "--preflight-still-seconds", 0.1, 5.0),
            ("preflight_max_velocity", "--preflight-max-velocity", 0.01, 1.0),
            ("preflight_max_drift_deg", "--preflight-max-drift-deg", 0.01, 5.0),
            ("log_interval", "--log-interval", 0.02, 10.0),
        ):
            value = _finite_number(payload, key, minimum=low, maximum=high)
            if value is not None:
                command += [option, str(value)]
        if payload.get("motors_already_enabled") is True:
            command.append("--motors-already-enabled")
        command.append("--execute")
        return command
    raise ValueError("unsupported dynamics action")


class OperationalRunState(v28.FullRunState):
    """Run legacy motion first, then take over with the identified hold model."""

    def __init__(self, log_path: str, args, *, auto_hold_enabled: bool = True) -> None:
        super().__init__(log_path)
        self.dynamics_args = args
        self.auto_hold_enabled = bool(auto_hold_enabled)
        self._pending_post_hold_action = None

    def snapshot(self):
        run = super().snapshot()
        # The process waiter publishes its exit before the post-hold waiter
        # annotates completion. Expose a consistent status in that small gap.
        if (run and not run.get('running') and
                str(run.get('action', '')).startswith('dynamics:model-hold-after:') and
                run.get('post_move_model_hold', {}).get('state') == 'running'):
            info = dict(run['post_move_model_hold'])
            info['state'] = 'complete' if run.get('ok') and run.get('returncode') == 0 else 'failed'
            run['post_move_model_hold'] = info
        if run and run.get("running") and run.get("action") == "dynamics:board-grasp":
            for line in reversed(str(run.get("stdout", "")).splitlines()):
                if line.startswith('board home handoff= '):
                    try:
                        recovery = json.loads(line.split('= ',1)[1])
                    except ValueError:
                        continue
                    if recovery.get('state') == 'starting':
                        run.update(control_state='home-handoff', motion_running=True,
                                   support_running=False, process_alive=True, recovery_action='home')
                        break
                if line.startswith("board protective hold= "):
                    try:
                        report = json.loads(line.split("= ", 1)[1])
                    except (ValueError, IndexError):
                        continue
                    run.update(ok=False, control_state="protective-hold",
                               running=False, motion_running=False, support_running=True, process_alive=True,
                               requires_operator_attention=True, protective_hold=report)
                    run["post_move_model_hold"] = {
                        "enabled": True, "state": "same-process-protective-hold",
                        "automatic_resume": False, "timeout_seconds": None}
                    break
        return run

    def set_auto_hold_enabled(self, enabled: bool) -> bool:
        with self.lock:
            self.auto_hold_enabled = bool(enabled)
            return self.auto_hold_enabled

    def get_auto_hold_enabled(self) -> bool:
        with self.lock:
            return self.auto_hold_enabled

    def start(self, action: str, cmd: list, popen_kwargs: dict) -> dict:
        if action == 'home':
            # General board IK can place the wrist >120deg from Clearance.
            # Keep Home's staged path and timing; widen its request explicitly.
            cmd = list(cmd)
            if '--max-delta-deg' in cmd:
                cmd[cmd.index('--max-delta-deg')+1] = '180'
            else:
                cmd += ['--max-delta-deg','180']
        status = self.snapshot()
        if action == 'home' and status and status.get('support_running'):
            if not status.get('protective_hold', {}).get('home_handoff_available'):
                return {'ok':False,'running':False,'support_running':True,
                        'error':'Current support process predates same-session Home handoff. Support the arm before ending that old process.'}
            with self.lock:
                if self.proc is None or self.proc.poll() is not None:
                    return {'ok':False,'running':False,'error':'Support process changed; refresh status before requesting Home.'}
                if not hasattr(signal,'SIGUSR1'):
                    return {'ok':False,'running':False,'error':'Same-session Home handoff requires Linux SIGUSR1.'}
                os.kill(self.proc.pid, signal.SIGUSR1)
                self.current['home_handoff_requested'] = True
            return {'ok':True,'running':True,'motion_running':False,'support_running':True,
                    'action':'home','control_state':'home-handoff-requested',
                    'same_serial_session':True,'command':cmd,'command_text':' '.join(cmd)}
        should_hold = self.get_auto_hold_enabled() and is_post_move_hold_action(action)
        if action in {"dynamics:move-target", "dynamics:board-grasp"} and "--execute" not in cmd:
            should_hold = False
        if should_hold and action == 'dynamics:board-grasp' and '--test-protective-stop' not in cmd:
            cmd = list(cmd)+['--inline-model-hold']
        if should_hold:
            with self.lock:
                self._pending_post_hold_action = action
        result = super().start(action, cmd, motion_dynamics_popen_kwargs(
            self.dynamics_args, action, cmd, popen_kwargs))
        if not (result.get("ok") and result.get("running")):
            with self.lock:
                if self._pending_post_hold_action == action:
                    self._pending_post_hold_action = None
        if result.get("ok") and result.get("running") and should_hold:
            plan = {
                "enabled": True,
                "state": "waiting-for-motion",
                "seconds": POST_MOVE_HOLD_SECONDS,
                "joints": list(DYNAMICS_JOINT_ORDER),
            }
            with self.lock:
                if self.current is not None and self.current.get("action") == action:
                    self.current["post_move_model_hold"] = plan
                    result = dict(self.current)
        return result

    def _waiter(self, proc) -> None:
        with self.lock:
            original = dict(self.current or {}) if self.proc is proc else {}
            planned = (
                original.get("post_move_model_hold", {}).get("state")
                == "waiting-for-motion"
                or self._pending_post_hold_action == original.get("action")
            )
            if planned and self._pending_post_hold_action == original.get("action"):
                self._pending_post_hold_action = None
        super()._waiter(proc)
        if '--inline-model-hold' in original.get('command',[]):
            # The board process has already completed its Model Hold before
            # exiting. Never reopen the serial device for a second takeover.
            with self.lock:
                if self.last is not None and self.last.get('action')==original.get('action'):
                    self.last['post_move_model_hold'] = {
                        'enabled':True,'state':'complete' if self.last.get('ok') else 'failed',
                        'same_serial_session':True,'seconds':POST_MOVE_HOLD_SECONDS,
                        'joints':list(DYNAMICS_JOINT_ORDER)}
            return
        if str(original.get("action", "")).startswith("dynamics:model-hold-after:"):
            with self.lock:
                if self.last is not None and self.last.get("action") == original.get("action"):
                    info = dict(self.last.get("post_move_model_hold", {}))
                    info["state"] = "complete" if self.last.get("ok") else "failed"
                    self.last["post_move_model_hold"] = info
            return
        if not planned:
            return
        with self.lock:
            primary = dict(self.last or {})
        if (
            primary.get("action") != original.get("action")
            or primary.get("running")
            or not primary.get("ok")
            or primary.get("returncode") != 0
            or primary.get("cancelled_by")
        ):
            with self.lock:
                if self.last is not None and self.last.get("action") == original.get("action"):
                    info = dict(self.last.get("post_move_model_hold", {}))
                    info["state"] = (
                        "skipped-motion-cancelled"
                        if self.last.get("cancelled_by")
                        else "skipped-motion-failed"
                    )
                    self.last["post_move_model_hold"] = info
            return
        primary_hold = dict(primary.get("post_move_model_hold", {}))
        primary_hold["state"] = "motion-complete"
        primary["post_move_model_hold"] = primary_hold
        hold_action = "dynamics:model-hold-after:" + str(primary.get("action", "motion"))
        hold_payload = dict(POST_MOVE_HOLD_PAYLOAD)
        enabled_joints = enabled_joints_from_completed_run(primary)
        reuse_enabled_motors = enabled_joints.issuperset(DYNAMICS_JOINT_ORDER)
        if reuse_enabled_motors:
            hold_payload["motors_already_enabled"] = True
        try:
            hold_command = build_dynamics_command(
                self.dynamics_args, "model-hold", hold_payload
            )
            if primary.get("action") in {"dynamics:move-target", "dynamics:board-grasp"}:
                hold_command += ['--general-fallback-model',
                                 os.path.abspath(self.dynamics_args.rigid_body_model)]
            inherited = board_hold_motion_gains(primary)
            if inherited:
                hold_command += ['--motion-gain-floor-json',json.dumps(inherited,sort_keys=True)]
        except Exception as exc:
            failed = {
                "ok": False,
                "running": False,
                "action": hold_action,
                "error": f"unable to build post-move Model Hold command: {exc}",
                "preceding_motion": primary,
                "post_move_model_hold": {
                    "enabled": True,
                    "state": "failed-to-start",
                    "seconds": POST_MOVE_HOLD_SECONDS,
                    "joints": list(DYNAMICS_JOINT_ORDER),
                },
            }
            self.set_last(failed)
            return
        hold_result = super().start(hold_action, hold_command, popen_kwargs=motion_dynamics_popen_kwargs(
            self.dynamics_args, hold_action, hold_command, {}))
        with self.lock:
            target = self.current if hold_result.get("running") else self.last
            if target is not None and target.get("action") == hold_action:
                target["preceding_motion"] = primary
                target["post_move_model_hold"] = {
                    "enabled": True,
                    "state": "running" if hold_result.get("running") else "failed-to-start",
                    "seconds": POST_MOVE_HOLD_SECONDS,
                    "joints": list(DYNAMICS_JOINT_ORDER),
                    "motor_enable_mode": (
                        "reused-existing-enable"
                        if reuse_enabled_motors
                        else "enabled-by-model-hold"
                    ),
                    "preceding_motion_enabled_joints": sorted(enabled_joints),
                }


def build_shell_page() -> str:
    page = v28.HTML_PAGE.replace("HiveRobot v2.8", "HiveRobot v3.0")
    crown_button = '<button id="crown-tab" aria-selected="false" onclick="showPanel(\'crown\')">Crown Geometry Teaching</button>'
    dynamics_button = crown_button + '\n<button id="dynamics-tab" aria-selected="false" onclick="showPanel(\'dynamics\')">Dynamics Control</button>'
    if crown_button not in page:
        raise RuntimeError("v3.0 shell injection failed: Crown button was not found")
    page = page.replace(crown_button, dynamics_button, 1)
    crown_frame = '<iframe id="crown" src="/crown" title="Crown Geometry Teaching" hidden></iframe>'
    dynamics_frame = crown_frame + '\n<iframe id="dynamics" src="/dynamics" title="Dynamics Control" hidden></iframe>'
    page = page.replace(crown_frame, dynamics_frame, 1)
    page = page.replace("['vision','arm','crown']", "['vision','arm','crown','dynamics']", 1)
    # Hiding an iframe does not stop its MJPEG streams or polling. Keep only
    # the selected panel loaded, so video connections cannot queue controls
    # from another panel behind long-lived browser HTTP connections.
    for name in ("vision", "arm", "crown", "dynamics"):
        source = f'id="{name}" src="/{name}"'
        replacement = f'id="{name}" data-src="/{name}"'
        if name == "vision":
            replacement += f' src="/{name}"'
        page = page.replace(source, replacement, 1)
    old_switch = "<script>function showPanel(name){"
    start = page.index(old_switch)
    end = page.index("</script>", start) + len("</script>")
    page = page[:start] + r"""<script>
function showPanel(name) {
  const panels = ['vision','arm','crown','dynamics'];
  if (!panels.includes(name)) return;
  // Release old documents before starting another panel's requests. This
  // affects browser resources only; server-side robot commands continue.
  for (const id of panels) {
    const frame = document.getElementById(id);
    frame.hidden = id !== name;
    document.getElementById(id+'-tab').setAttribute('aria-selected', String(id===name));
    if (id !== name && frame.hasAttribute('src')) frame.removeAttribute('src');
  }
  const active = document.getElementById(name);
  if (!active.hasAttribute('src')) active.setAttribute('src', active.dataset.src);
  document.getElementById('open').href = '/'+name;
}
</script>""" + page[end:]
    return page


HTML_PAGE = build_shell_page()

DYNAMICS_PAGE = r"""<!doctype html><html lang="en"><meta charset="utf-8">
<title>HiveRobot Dynamics Control</title>
<style>
body{margin:0;background:#101821;color:#eaf0f4;font:15px system-ui;padding:18px}h1{margin:0 0 6px}h2{font-size:18px;margin:0 0 12px}.note{color:#b9c8d2}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(310px,1fr));gap:14px;margin-top:16px}.card{background:#172430;border:1px solid #49677c;border-radius:9px;padding:14px}.wide{grid-column:1/-1}.row{display:flex;gap:8px;flex-wrap:wrap;margin:8px 0}label{display:flex;flex-direction:column;gap:4px;flex:1;min-width:120px}input,select,textarea,button{font:inherit;color:inherit;background:#0e1820;border:1px solid #66849a;border-radius:4px;padding:7px}button{background:#28445b;cursor:pointer}button.primary{background:#326d58}button.danger{background:#713d38}textarea{min-height:62px;width:100%;box-sizing:border-box}pre{white-space:pre-wrap;overflow-wrap:anywhere;background:#091117;padding:10px;border-radius:5px;max-height:46vh;overflow:auto}.status{padding:10px;background:#142c38;border-left:4px solid #59a8c8;margin:12px 0}.check{display:flex;flex-direction:row;align-items:center;gap:8px}.check input{width:auto}.output-heading{display:flex;align-items:center;justify-content:space-between;margin-top:18px}.output-heading h2{margin:0}.copy-button{padding:5px 12px}.joint-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:7px}.joint-row{display:grid;grid-template-columns:110px 1fr 1fr;gap:6px;align-items:center;background:#101c25;padding:7px;border-radius:5px}.joint-name{font-family:monospace;color:#c9d9e3}.angle-options{display:flex;gap:14px;align-items:center;margin:10px 0}.angle-options label{display:flex;flex-direction:row;min-width:auto;align-items:center}small{color:#b9c8d2}
</style><body>
<h1>Dynamics Control</h1>
<div class="note">Identified gravity and inertia scheduling: kp = omega_n^2 M(q), kd = 2 zeta omega_n M(q). Existing robot motion flows are unchanged.</div>
<div id="status" class="status">Loading controller status...</div>
<div class="grid">
 <section class="card wide"><h2>Test Pose Setup</h2><div class="note">Use the existing trained arm controller to reach repeatable identification poses. Wait for each move to finish before collecting a sample. Wrist uses a fixed guarded 5 degree probe with low gains, zero active torque, and watchdogs.</div><label class="check"><input id="autoModelHold" type="checkbox" checked onchange="setAutoModelHold(this.checked)">After a successful Home, Table Clearance, or Replay, hand off all seven joints to Model Hold for 5 seconds</label><div class="row"><button class="primary" onclick="runPoseAction('home','Home Move')">Home Move</button><button class="primary" onclick="runPoseAction('table-clearance','Table Clearance Move')">Table Clearance Move</button><button class="danger" onclick="runPoseAction('emergency-stop','Emergency Stop',true)">Emergency Stop</button></div><div class="row"><label>Existing Saved Move<select id="testMoveSelect"></select></label><button class="primary" onclick="replayTestMove()">Replay Selected Test Pose</button><button onclick="loadTestMoves()">Refresh Saved Moves</button></div><div class="angle-options"><strong>Joint nudge:</strong><label><input type="radio" name="testNudgeDeg" value="2" checked>2 degrees</label><label><input type="radio" name="testNudgeDeg" value="5">5 degrees</label><label><input type="radio" name="testNudgeDeg" value="10">10 degrees</label></div><div id="testJointGrid" class="joint-grid"></div></section>
 <section class="card wide"><h2>General Joint Target</h2><div class="note">Absolute encoder angles in radians. Enter any seven-axis target or only the joints to move; omitted joints hold their current positions. The same dynamics model is used for every target. This follows a joint-space path; it does not plan around obstacles.</div><label>Target JSON<textarea id="generalTargets" rows="4" placeholder='{"wrist": -2.0}'></textarea></label><div class="row"><label>Duration (s)<input id="generalSeconds" type="number" value="8" min="1" max="120"></label><label>Max joint change (deg)<input id="generalMaxDelta" type="number" value="15" min="0.1" max="180"></label></div><label class="check"><input id="executeGeneral" type="checkbox">Enable actual motor commands</label><div class="row"><button onclick="generalTarget(false)">Preview Target</button><button class="primary" onclick="generalTarget(true)">Move to Target</button></div></section>
 <section class="card wide"><h2>Board Point Grasp — Staged Validation</h2><p>v14: Approach uses measured feedback before descent: XY acquisition occurs at the measured height, followed by descent with the measured wrist orientation. Model Approach + Touch solves the requested board XYZ from the current pose via a checked above-board entry and Cartesian descent, without closing the claw. Motion completes without an extra endpoint chase. Model Hold continues the exact terminal motor command in the same serial session. Continuous transit includes pose-relative gravity support. Velocity and tracking errors are recorded without automatic trajectory stops. Touch keeps the current wrist orientation and permits 1–25 mm of descent from matching XY. Home Move uses the existing staged path with a 180 degree entry allowance. Actual execution failures retain load support and explicit same-session Home recovery. Physical and Web Emergency Stop remain available.</p><div class="note">Board millimeters: origin at the outer a1 corner, X toward h1, Y toward a8. Nominal CAD/teaching registration needs on-robot validation. Start with Above Only on a cleared board. Touch descends without closing; Grasp closes on detected contact and lifts. Other pieces are not included in path collision checks.</div><div class="row"><label>X (mm)<input id="boardX" type="number" value="137.5" min="0" max="440" step="0.1"></label><label>Y (mm)<input id="boardY" type="number" value="192.5" min="0" max="440" step="0.1"></label><label>Grasp profile<select id="boardPiece"><option>white_knight</option><option>white_bishop</option><option>white_rook</option></select></label><label>Touch target height (mm)<input id="boardHeight" type="number" min="10" max="150" value="140" step="1"></label><label>Stage<select id="boardStage"><option value="above">Above Only</option><option value="touch">Touch Position (no close)</option><option value="approach">Model Approach + Touch (no close)</option><option value="grasp">Pressure Grasp and Lift</option></select></label></div><label class="check"><input id="executeBoard" type="checkbox">Enable actual motor commands</label><div class="row"><button onclick="boardPoint(false)">Preview Board Path</button><button class="primary" onclick="boardPoint(true)">Execute Selected Stage</button><button onclick="boardStopTest()">Test Protective Hold (no trajectory)</button></div></section>
 <section class="card"><h2>Model</h2><div class="row"><button onclick="run('init-model')">Initialize Model</button><button onclick="run('show-model')">Show Model</button><button onclick="refresh()">Refresh</button></div><label class="check"><input id="unitsConfirmed" type="checkbox">I verified DM MIT gain and torque units</label><button onclick="verifyUnits()">Save Unit Verification</button></section>
 <section class="card"><h2>Gravity Sample</h2><label>Joints<input id="gravityJoints" value="shoulder_front,shoulder_side,shoulder_rotate,elbow,arm_roll,wrist_side,wrist"></label><div class="row"><label>Direction<select id="gravityDirection"><option>stationary</option><option>positive</option><option>negative</option></select></label><label>Samples<input id="gravitySamples" type="number" value="30" min="3"></label><label>Interval (s)<input id="gravityInterval" type="number" value="0.02" step="0.001"></label><label>Max velocity<input id="gravityMaxVelocity" type="number" value="0.03" step="0.001"></label></div><small>This capture reads the current motor status and does not command motion. Removal uses the first joint entered above.</small><div class="row"><button onclick="captureGravity()">Capture Gravity</button><button class="danger" onclick="removeLatestGravity()">Remove Latest Gravity Sample (No Motion)</button></div></section>
 <section class="card"><h2>Guarded Inertia Step Test</h2><div class="note">Runs one small joint step, records the response, returns to the starting angle, and saves only a consistent estimate.</div><div class="row"><label>Joint<select id="measureInertiaJoint"></select></label><label>Step (deg)<input id="inertiaStepDeg" type="number" value="2" min="-3" max="3" step="0.25"></label><label>Test kp<input id="measureTestKp" type="number" value="40" min="1" max="70" step="1"></label><label>Test kd<input id="measureTestKd" type="number" value="3" min="0.1" max="5" step="0.1"></label></div><details><summary>Safety and timing</summary><div class="row"><label>Settle seconds<input id="inertiaSettleSeconds" type="number" value="1.5" step="0.1"></label><label>Test seconds<input id="inertiaTestSeconds" type="number" value="1.2" step="0.1"></label><label>Return seconds<input id="inertiaReturnSeconds" type="number" value="1.5" step="0.1"></label><label>Control dt<input id="inertiaControlDt" type="number" value="0.01" step="0.005"></label><label>Max velocity (rad/s)<input id="inertiaMaxVelocity" type="number" value="1" step="0.1"></label><label>Max other-joint error (deg)<input id="inertiaMaxError" type="number" value="5" step="0.5"></label></div></details><label>Label<input id="measureInertiaLabel" value="elbow_home_step"></label><label class="check"><input id="executeInertiaTest" type="checkbox">Enable actual motor commands for this test</label><div class="row"><button class="danger" onclick="measureInertia()">Run Inertia Step Test</button><button onclick="reprocessInertia()">Reprocess Last Trace (No Motion)</button><button class="danger" onclick="removeLatestInertia()">Remove Latest Sample (No Motion)</button></div><small>Requires at least three gravity samples and configured friction for the selected joint.</small><details><summary>Manual inertia entry</summary><div class="row"><label>Joint<select id="inertiaJoint"></select></label><label>Effective inertia<input id="effectiveInertia" type="number" step="any"></label><label>Test kp<input id="testKp" type="number" step="any"></label><label>Measured omega_n<input id="measuredOmegaN" type="number" step="any"></label></div><label class="check"><input id="inertiaFromArm" type="checkbox" checked>Use current arm pose</label><label>Full seven-joint pose JSON (radians)<textarea id="inertiaPose"></textarea></label><label>Label<input id="inertiaLabel"></label><button onclick="addInertia()">Add Inertia Sample</button></details></section>
 <section class="card"><h2>Gain Scheduler</h2><label>Joint<select id="configJoint"></select></label><div class="row"><label>omega_n<input id="omegaN" type="number" step="any"></label><label>zeta<input id="zeta" type="number" step="any"></label></div><div class="row"><label>kp min<input id="kpMin" type="number" step="any"></label><label>kp max<input id="kpMax" type="number" step="any"></label><label>kd min<input id="kdMin" type="number" step="any"></label><label>kd max<input id="kdMax" type="number" step="any"></label></div><button onclick="configureScheduler()">Save Scheduler</button></section>
 <section class="card"><h2>Friction</h2><label>Joint<select id="frictionJoint"></select></label><div class="row"><label>Coulomb<input id="coulomb" type="number" value="0" step="any"></label><label>Viscous<input id="viscous" type="number" value="0" step="any"></label></div><button onclick="configureFriction()">Save Friction</button></section>
 <section class="card"><h2>Preview and Hold</h2><label>Joints<input id="controlJoints" value="shoulder_front,shoulder_side,shoulder_rotate,elbow,arm_roll,wrist_side,wrist"></label><label>Optional pose JSON (radians)<textarea id="previewPose" placeholder="Leave empty to read the current arm pose"></textarea></label><div class="row"><button onclick="preview()">Preview Scheduled Terms</button><label>Hold seconds<input id="holdSeconds" type="number" value="5" min="0.1" max="120" step="0.1"></label></div><details><summary>Advanced hold limits</summary><div class="row"><label>Control dt<input id="controlDt" type="number" value="0.01" step="0.001"></label><label>Takeover kp<input id="takeoverKp" type="number" value="2" step="0.1"></label><label>Takeover kd<input id="takeoverKd" type="number" value="0.4" step="0.1"></label><label>Max velocity<input id="holdMaxVelocity" type="number" value="0.3" step="0.1"></label><label>Max error (deg)<input id="maxErrorDeg" type="number" value="2.5" step="0.1"></label><label>Log interval<input id="logInterval" type="number" value="0.25" step="0.01"></label></div></details><label class="check"><input id="executeHold" type="checkbox">Enable actual motor commands</label><button class="danger" onclick="modelHold()">Model Hold</button><small>Model Hold also requires Web 3.0 to start with --enable-execute.</small></section>
</div>
<h2>Command</h2><pre id="command">None</pre><div class="output-heading"><h2>Output</h2><button class="copy-button" onclick="copyText('output','Output')">Copy</button></div><pre id="output">No dynamics command has run.</pre><div class="output-heading"><h2>Model Data</h2><button class="copy-button" onclick="copyText('model','Model Data')">Copy</button></div><pre id="model">Not loaded.</pre>
<script>
const joints=['shoulder_front','shoulder_side','shoulder_rotate','elbow','arm_roll','wrist_side','wrist'];
const jointDirections=[['shoulder_front','Backward','Forward'],['shoulder_side','Outward','Inward'],['elbow','Forward','Backward'],['shoulder_rotate','Inward Rotate','Outward Rotate'],['arm_roll','Inward Roll','Outward Roll'],['wrist_side','Wrist Outward','Wrist Inward'],['wrist','Wrist Forward','Wrist Backward']];
for(const id of ['measureInertiaJoint','inertiaJoint','configJoint','frictionJoint']){const select=document.getElementById(id);for(const joint of joints){const option=document.createElement('option');option.value=joint;option.textContent=joint;select.appendChild(option);}}
document.getElementById('measureInertiaJoint').value='elbow';
const val=id=>document.getElementById(id).value;
const optional=id=>val(id).trim()===''?null:Number(val(id));
let refreshing=false,loadingTestMoves=false;
async function pollJson(url){const controller=new AbortController();const timer=setTimeout(()=>controller.abort(),5000);try{const response=await fetch(url,{cache:'no-store',signal:controller.signal});if(!response.ok)throw new Error('HTTP '+response.status);return await response.json();}finally{clearTimeout(timer);}}
function showCommandResult(data,label){document.getElementById('command').textContent=data.command_text||JSON.stringify(data.command||[],null,2);document.getElementById('output').textContent=JSON.stringify(data,null,2);document.getElementById('status').textContent=data.ok?(label+' started.'):(label+' failed: '+(data.error||'unknown error'));}
async function postExisting(path,payload,label){try{const options={method:'POST',cache:'no-store'};if(payload!==undefined){options.headers={'Content-Type':'application/json'};options.body=JSON.stringify(payload);}const response=await fetch(path,options);const data=await response.json();showCommandResult(data,label);return data;}catch(error){document.getElementById('status').textContent=label+' request failed: '+error;}}
async function runPoseAction(action,label,force=false){if(!force&&!confirm(label+'?'))return;if(action==='emergency-stop'&&!confirm('Disable all left-arm and claw motors now?'))return;await postExisting('/api/action/'+action,undefined,label);}
function selectedTestNudgeDeg(){const selected=document.querySelector('input[name="testNudgeDeg"]:checked');return selected?Number(selected.value):2;}
async function runTestNudge(joint,direction){if(joint==='wrist'){const label='guarded wrist '+(direction>0?'forward':'backward')+' 5.0 degrees';if(!confirm('Run '+label+' with low gains, zero active torque, and watchdogs?'))return;await run('safe-wrist-nudge',{direction});return;}const deg=selectedTestNudgeDeg();const label=joint+' '+(direction>0?'+':'-')+deg+' degrees';if(!confirm('Nudge '+label+'?'))return;await postExisting('/api/nudge',{joint,direction,deg},label);}
function buildTestJointGrid(){const grid=document.getElementById('testJointGrid');grid.innerHTML='';for(const [joint,positive,negative] of jointDirections){const row=document.createElement('div');row.className='joint-row';const suffix=joint==='wrist'?' 5.0 degrees (guarded)':'';row.innerHTML='<div class="joint-name">'+joint+'</div><button onclick="runTestNudge(\''+joint+'\',1)">'+positive+suffix+'</button><button onclick="runTestNudge(\''+joint+'\',-1)">'+negative+suffix+'</button>';grid.appendChild(row);}}
async function loadTestMoves(){if(loadingTestMoves)return;loadingTestMoves=true;try{const data=await pollJson('/api/moves?t='+Date.now());const select=document.getElementById('testMoveSelect');const previous=select.value;select.innerHTML='';for(const move of (data.moves||[])){const option=document.createElement('option');option.value=move.name;option.textContent=move.name;select.appendChild(option);}if(previous)select.value=previous;if(data.error)document.getElementById('status').textContent='Saved move list error: '+data.error;}catch(error){document.getElementById('status').textContent='Saved move list request failed: '+error;}finally{loadingTestMoves=false;}}
async function replayTestMove(){const name=val('testMoveSelect');if(!name){document.getElementById('status').textContent='No saved move selected.';return;}if(!confirm('Replay existing saved move via table clearance first: '+name+'?'))return;await postExisting('/api/move/replay',{name},'Saved move '+name);}
async function setAutoModelHold(enabled){try{const response=await fetch('/api/v3/model-hold/auto',{method:'POST',cache:'no-store',headers:{'Content-Type':'application/json'},body:JSON.stringify({enabled:Boolean(enabled)})});const data=await response.json();if(!response.ok||!data.ok)throw new Error(data.error||'update rejected');document.getElementById('autoModelHold').checked=Boolean(data.enabled);document.getElementById('status').textContent='Automatic post-move Model Hold '+(data.enabled?'enabled.':'disabled.');}catch(error){document.getElementById('status').textContent='Unable to update automatic Model Hold: '+error;await refresh();}}
async function copyText(elementId,label){const text=document.getElementById(elementId).textContent;try{await navigator.clipboard.writeText(text);document.getElementById('status').textContent=label+' copied to clipboard.';}catch(error){const area=document.createElement('textarea');area.value=text;area.style.position='fixed';area.style.opacity='0';document.body.appendChild(area);area.select();const copied=document.execCommand('copy');area.remove();document.getElementById('status').textContent=copied?label+' copied to clipboard.':'Unable to copy '+label+'.';}}
async function run(action,payload={}){document.getElementById('status').textContent='Starting '+action+'...';try{const response=await fetch('/api/v3/dynamics/run',{method:'POST',cache:'no-store',headers:{'Content-Type':'application/json'},body:JSON.stringify({action,payload})});const data=await response.json();document.getElementById('command').textContent=data.command_text||JSON.stringify(data.command||[],null,2);document.getElementById('output').textContent=JSON.stringify(data,null,2);document.getElementById('status').textContent=response.ok&&data.ok?'Started: '+action:'Rejected: '+(data.error||action);await refresh();return data;}catch(error){document.getElementById('status').textContent='Request failed: '+error;}}
function verifyUnits(){if(!document.getElementById('unitsConfirmed').checked){alert('Verify the controller units before saving this setting.');return;}run('set-units-verified',{confirmed:true});}
function captureGravity(){run('capture-gravity',{joints:val('gravityJoints'),direction:val('gravityDirection'),samples:Number(val('gravitySamples')),interval:Number(val('gravityInterval')),max_velocity:Number(val('gravityMaxVelocity'))});}
function removeLatestGravity(){const joint=val('gravityJoints').split(',').map(value=>value.trim()).filter(Boolean)[0];if(!joint){alert('Enter one joint first.');return;}if(!confirm('Remove the newest stored gravity sample for '+joint+'? This does not move the robot.'))return;run('remove-latest-gravity',{joint});}
function addInertia(){run('add-inertia',{joint:val('inertiaJoint'),effective_inertia:optional('effectiveInertia'),test_kp:optional('testKp'),measured_omega_n:optional('measuredOmegaN'),from_arm:document.getElementById('inertiaFromArm').checked,pose:val('inertiaPose'),label:val('inertiaLabel')});}
function measureInertia(){if(!document.getElementById('executeInertiaTest').checked){alert('Enable actual motor commands for the inertia test first.');return;}const joint=val('measureInertiaJoint');const step=Number(val('inertiaStepDeg'));if(!confirm('Run a guarded '+step+' degree inertia step on '+joint+' and return to the starting angle? Keep physical fall protection in place.'))return;run('measure-inertia',{joint,step_deg:step,test_kp:Number(val('measureTestKp')),test_kd:Number(val('measureTestKd')),settle_seconds:Number(val('inertiaSettleSeconds')),test_seconds:Number(val('inertiaTestSeconds')),return_seconds:Number(val('inertiaReturnSeconds')),control_dt:Number(val('inertiaControlDt')),max_velocity:Number(val('inertiaMaxVelocity')),max_position_error_deg:Number(val('inertiaMaxError')),label:val('measureInertiaLabel'),execute:true});}
function reprocessInertia(){const joint=val('measureInertiaJoint');if(!confirm('Reprocess the latest recorded '+joint+' inertia trace without moving the robot?'))return;run('reprocess-inertia',{joint,label:val('measureInertiaLabel')+'_reprocessed'});}
function removeLatestInertia(){const joint=val('measureInertiaJoint');if(!confirm('Remove the newest stored inertia sample for '+joint+'? This does not move the robot.'))return;run('remove-latest-inertia',{joint});}
function configureScheduler(){run('configure',{joint:val('configJoint'),omega_n:optional('omegaN'),zeta:optional('zeta'),kp_min:optional('kpMin'),kp_max:optional('kpMax'),kd_min:optional('kdMin'),kd_max:optional('kdMax')});}
function configureFriction(){run('configure-friction',{joint:val('frictionJoint'),coulomb:Number(val('coulomb')),viscous:Number(val('viscous'))});}
function preview(){run('preview',{joints:val('controlJoints'),pose:val('previewPose')});}
function generalTarget(execute){try{if(execute&&!document.getElementById('executeGeneral').checked){alert('Enable actual motor commands first.');return;}const targets=JSON.parse(val('generalTargets'));run('move-target',{targets_rad:targets,seconds:Number(val('generalSeconds')),max_delta_deg:Number(val('generalMaxDelta')),execute});}catch(error){document.getElementById('status').textContent='Invalid target JSON: '+error;}}
function boardStopTest(){if(!document.getElementById('executeBoard').checked){alert('Enable actual motor commands first.');return;}run('board-grasp',{x_mm:Number(val('boardX')),y_mm:Number(val('boardY')),stage:'above',execute:true,test_protective_stop:true});}
function boardPoint(execute){if(execute&&val('boardStage')==='grasp'){alert('Validate Above Only and staged Touch before grasping.');return;}if(execute&&!document.getElementById('executeBoard').checked){alert('Enable actual motor commands first.');return;}run('board-grasp',{x_mm:Number(val('boardX')),y_mm:Number(val('boardY')),piece:val('boardPiece'),stage:val('boardStage'),...(['touch','approach'].includes(val('boardStage'))?{grip_height_mm:Number(val('boardHeight'))}:{}),execute});}
function modelHold(){if(!document.getElementById('executeHold').checked){alert('Enable actual motor commands first.');return;}if(!confirm('Hold the current arm pose with the identified dynamics controller?'))return;run('model-hold',{joints:val('controlJoints'),seconds:Number(val('holdSeconds')),control_dt:Number(val('controlDt')),takeover_kp:Number(val('takeoverKp')),takeover_kd:Number(val('takeoverKd')),max_velocity:Number(val('holdMaxVelocity')),max_error_deg:Number(val('maxErrorDeg')),log_interval:Number(val('logInterval')),execute:true});}
async function refresh(){if(refreshing)return;refreshing=true;try{const data=await pollJson('/api/v3/dynamics/status');document.getElementById('autoModelHold').checked=Boolean(data.auto_model_hold_enabled);document.getElementById('model').textContent=data.model?JSON.stringify(data.model,null,2):(data.model_error||'Model file has not been initialized.');if(data.run){document.getElementById('output').textContent=JSON.stringify(data.run,null,2);}document.getElementById('status').textContent='Execution '+(data.execution_enabled?'enabled':'disabled')+'; automatic post-move hold '+(data.auto_model_hold_enabled?'enabled':'disabled')+'; motion dynamics '+(data.motion_dynamics_enabled?'enabled':'disabled')+'; general dynamics '+(data.rigid_body_mode||'disabled')+'; model '+(data.model?'loaded':'not initialized')+(data.run&&data.run.control_state==='protective-hold'?'; PROTECTIVE HOLD — trajectory stopped, supporting load; support mechanically before any stop/restart':(data.run&&data.run.running?'; command running: '+data.run.action:''));}catch(error){document.getElementById('status').textContent='Status request failed: '+error;}finally{refreshing=false;}}
buildTestJointGrid();loadTestMoves();refresh();setInterval(refresh,1000);setInterval(loadTestMoves,5000);
</script></body></html>"""


def classify_c4_route(piece_class: str) -> dict | None:
    """Return the existing trained multi-flow selected by a C4 class."""
    normalized = str(piece_class).strip().lower().replace("-", "_")
    piece_type = normalized.rsplit("_", 1)[-1]
    if piece_type == "bishop":
        return {
            "piece_type": "bishop",
            "saved_move": "white_knight_c4",
            "flow_key": "white_bishop_placement",
            "flow_label": "White Bishop Placement",
        }
    if piece_type == "knight":
        return {
            "piece_type": "knight",
            "saved_move": "white_knight_c4",
            "flow_key": "white_knight_place_b1",
            "flow_label": "white knight place_B1",
        }
    if piece_type == "rook":
        return {
            "piece_type": "rook",
            "saved_move": "white_knight_c4",
            "flow_key": "white_rook_place_a1",
            "flow_label": "white rook place_A1",
        }
    return None


class PendingC4Confirmations:
    """Short-lived, one-time server-side confirmations."""

    def __init__(self, ttl_seconds: float = CONFIRMATION_TTL_SECONDS) -> None:
        self.ttl_seconds = float(ttl_seconds)
        self.lock = threading.Lock()
        self.pending: dict[str, dict] = {}

    def issue(self, detection: dict, route: dict) -> tuple[str, float]:
        token = secrets.token_urlsafe(24)
        expires_at = time.time() + self.ttl_seconds
        with self.lock:
            self._discard_expired_locked()
            self.pending[token] = {
                "expires_at": expires_at,
                "detection": dict(detection),
                "route": dict(route),
            }
        return token, expires_at

    def consume(self, token: str) -> dict:
        now = time.time()
        with self.lock:
            self._discard_expired_locked(now)
            record = self.pending.pop(str(token), None)
        if record is None:
            raise ValueError("The confirmation has expired or was already used. Inspect C4 again.")
        return record

    def _discard_expired_locked(self, now: float | None = None) -> None:
        current = time.time() if now is None else float(now)
        expired = [token for token, item in self.pending.items()
                   if float(item.get("expires_at", 0.0)) <= current]
        for token in expired:
            self.pending.pop(token, None)


def build_arm_page() -> str:
    page = v28.ARM_HTML_PAGE
    replay_button = '<button type="button" class="primary" onclick="replayMove()">Replay</button>'
    c4_button = replay_button + """
            <button type="button" class="primary" onclick="inspectC4AndConfirm()">
              Inspect C4 and Confirm Execution
            </button>
            <label class="inline-option" title="After a successful Home, Table Clearance, Replay, or multi-step flow, hold the reached arm pose with the identified model for five seconds.">
              <input id="autoModelHoldOperational" type="checkbox" checked onchange="setOperationalAutoModelHold(this.checked)">
              Automatic Model Hold After Motion
            </label>"""
    if replay_button not in page:
        raise RuntimeError("v3.0 page injection failed: Replay button was not found")
    page = page.replace(replay_button, c4_button, 1)
    style_end = "</style>"
    modal_css = """
.v3-modal{position:fixed;inset:0;z-index:9999;background:rgba(0,0,0,.76);display:flex;align-items:center;justify-content:center;padding:24px}
.v3-modal[hidden]{display:none}.v3-dialog{width:min(720px,95vw);max-height:94vh;overflow:auto;background:#172430;border:1px solid #7392a8;border-radius:10px;padding:18px;box-shadow:0 20px 60px #000}
.v3-dialog img{display:block;width:100%;max-height:58vh;object-fit:contain;background:#080d11;margin:12px 0}.v3-actions{display:flex;gap:12px;justify-content:flex-end}.v3-result{font-size:18px;line-height:1.5;white-space:pre-line}
""" + style_end
    if style_end not in page:
        raise RuntimeError("v3.0 page injection failed: style end was not found")
    page = page.replace(style_end, modal_css, 1)
    modal = """
<div id="c4ConfirmModal" class="v3-modal" hidden role="dialog" aria-modal="true" aria-labelledby="c4ConfirmTitle">
  <div class="v3-dialog">
    <h2 id="c4ConfirmTitle">Confirm C4 Piece</h2>
    <div id="c4ConfirmText" class="v3-result"></div>
    <img id="c4ConfirmImage" alt="C4 YOLO prediction">
    <div class="v3-actions">
      <button type="button" onclick="resolveC4Confirmation(false)">Cancel</button>
      <button type="button" class="primary" onclick="resolveC4Confirmation(true)">Recognition Is Correct - Run Robot Arm</button>
    </div>
  </div>
</div>
"""
    if "</body>" not in page:
        raise RuntimeError("v3.0 page injection failed: body end was not found")
    page = page.replace("</body>", modal + "</body>", 1)
    script = r"""
    let pendingC4Confirmation = null;

    async function syncAutoModelHold() {
      try {
        const response = await fetch('/api/v3/dynamics/status', {cache: 'no-store'});
        const data = await response.json();
        document.getElementById('autoModelHoldOperational').checked = Boolean(data.auto_model_hold_enabled);
      } catch (error) {
        setStatus('Unable to read automatic Model Hold state: ' + error);
      }
    }

    async function setOperationalAutoModelHold(enabled) {
      try {
        const response = await fetch('/api/v3/model-hold/auto', {
          method: 'POST', cache: 'no-store',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({enabled: Boolean(enabled)})
        });
        const data = await response.json();
        if (!response.ok || !data.ok) throw new Error(data.error || 'update rejected');
        document.getElementById('autoModelHoldOperational').checked = Boolean(data.enabled);
        setStatus('Automatic post-move Model Hold ' + (data.enabled ? 'enabled.' : 'disabled.'));
      } catch (error) {
        setStatus('Unable to update automatic Model Hold: ' + error);
        syncAutoModelHold();
      }
    }

    function resolveC4Confirmation(confirmed) {
      const modal = document.getElementById('c4ConfirmModal');
      modal.hidden = true;
      const pending = pendingC4Confirmation;
      pendingC4Confirmation = null;
      if (pending) pending.resolve(Boolean(confirmed));
    }

    function showC4Confirmation(data) {
      const confidence = Number(data.confidence || 0) * 100;
      document.getElementById('c4ConfirmText').textContent =
        'C4 recognition result: ' + data.piece_class + ' (' + confidence.toFixed(1) + '%)\n' +
        'After confirmation, run: ' + data.flow_label + '\nVerify that the prediction box and piece class are correct.';
      const image = document.getElementById('c4ConfirmImage');
      image.src = data.prediction_image_url + (data.prediction_image_url.includes('?') ? '&' : '?') + 'ts=' + Date.now();
      document.getElementById('c4ConfirmModal').hidden = false;
      return new Promise((resolve) => { pendingC4Confirmation = {resolve}; });
    }

    async function inspectC4AndConfirm() {
      if (pendingC4Confirmation) return;
      setStatus('Inspecting C4. The robot arm will not move during this step.');
      try {
        const response = await fetch('/api/v3/c4/inspect', {
          method: 'POST', cache: 'no-store',
          headers: {'Content-Type': 'application/json'}, body: '{}'
        });
        const data = await response.json();
        document.getElementById('output').textContent = formatOutput(data);
        if (!response.ok || !data.ok) throw new Error(data.error || 'C4 recognition failed');
        if (!data.actionable) {
          setStatus('C4 was recognized as ' + (data.piece_class || 'none') + '. It is not a bishop, knight, or rook, so the robot arm will not move.');
          alert('C4 recognition result: ' + (data.piece_class || 'no piece detected') + '\nOnly bishop, knight, and rook are currently supported. The robot arm will not move.');
          return;
        }
        const confirmed = await showC4Confirmation(data);
        if (!confirmed) {
          setStatus('Cancelled. The robot arm was not started.');
          return;
        }
        setStatus('Confirmed. Starting ' + data.flow_label + '.');
        const execute = await fetch('/api/v3/c4/execute', {
          method: 'POST', cache: 'no-store',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({
            confirmation_token: data.confirmation_token,
            confirmed: true,
            enable_gate: document.getElementById('enableMotionGate').checked
          })
        });
        const result = await execute.json();
        document.getElementById('command').textContent = result.command_text || JSON.stringify(result.command || [], null, 2);
        document.getElementById('output').textContent = formatOutput(result);
        if (!execute.ok || !result.ok) throw new Error(result.error || 'Failed to start the robot arm flow');
        setStatus('Started: ' + data.flow_label);
      } catch (error) {
        setStatus('C4 automatic flow failed: \n' + error);
      }
      refresh();
    }

    syncAutoModelHold();
"""
    marker = "</script>"
    index = page.rfind(marker)
    if index < 0:
        raise RuntimeError("v3.0 page injection failed: script end was not found")
    return page[:index] + script + page[index:]


ARM_HTML_PAGE = build_arm_page()


def make_handler(vision_state, stream_state, run_state, ctrl_cfg, args):
    base_handler = V28_MAKE_HANDLER(
        vision_state, stream_state, run_state, ctrl_cfg, args
    )
    confirmations = PendingC4Confirmations()

    class Handler(base_handler):
        server_version = "HiveRobotV30/1.0"

        def dynamics_status(self) -> None:
            model = None
            model_error = None
            model_path = Path(args.dynamics_model).resolve()
            try:
                if model_path.is_file():
                    model = json.loads(model_path.read_text(encoding="utf-8"))
                else:
                    model_error = "Dynamics model file has not been initialized."
            except (OSError, json.JSONDecodeError) as exc:
                model_error = f"Unable to read dynamics model: {exc}"
            return self.send_json({
                "ok": True,
                "version": WEB_VERSION,
                "execution_enabled": bool(ctrl_cfg.execute_enabled),
                "auto_model_hold_enabled": run_state.get_auto_hold_enabled(),
                "motion_dynamics_enabled": bool(getattr(args, "motion_dynamics", False)),
                "rigid_body_model": getattr(args, "rigid_body_model", None),
                "rigid_body_mode": getattr(args, "rigid_body_mode", "shadow") if getattr(args, "rigid_body_model", None) else "disabled",
                "dynamics_script": os.path.abspath(args.dynamics_script),
                "dynamics_model": str(model_path),
                "model": model,
                "model_error": model_error,
                "run": run_state.snapshot(),
            })

        def run_dynamics_action(self) -> None:
            try:
                body = self.read_json_body()
                action = str(body.get("action", "")).strip()
                payload = body.get("payload", {})
                if not isinstance(payload, dict):
                    raise ValueError("payload must be an object")
                needs_execution = action in {"model-hold", "measure-inertia", "safe-wrist-nudge"} or (
                    action in {"move-target", "board-grasp"} and payload.get("execute") is True)
                if needs_execution and not ctrl_cfg.execute_enabled:
                    raise PermissionError("execution disabled; restart with --enable-execute")
                command = build_dynamics_command(args, action, payload)
            except PermissionError as exc:
                return self.send_json({"ok": False, "error": str(exc)}, HTTPStatus.FORBIDDEN)
            except (ValueError, TypeError, json.JSONDecodeError) as exc:
                return self.send_json({"ok": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST)
            result = run_state.start(
                "dynamics:" + action,
                command,
                popen_kwargs={},
            )
            result["command"] = command
            result["command_text"] = " ".join(command)
            return self.send_json(
                result,
                HTTPStatus.OK if result.get("ok") else HTTPStatus.CONFLICT,
            )

        def set_auto_model_hold(self) -> None:
            try:
                body = self.read_json_body()
                if not isinstance(body.get("enabled"), bool):
                    raise ValueError("enabled must be true or false")
                enabled = run_state.set_auto_hold_enabled(body["enabled"])
            except (ValueError, TypeError, json.JSONDecodeError) as exc:
                return self.send_json(
                    {"ok": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST
                )
            return self.send_json({
                "ok": True,
                "enabled": enabled,
                "applies_to": "future successful Home, Table Clearance, Replay, and multi-step motions",
                "seconds": POST_MOVE_HOLD_SECONDS,
                "joints": list(DYNAMICS_JOINT_ORDER),
            })

        def _loopback_json(self, path: str, payload: dict | None = None) -> tuple[int, dict]:
            port = int(self.server.server_address[1])
            url = f"http://127.0.0.1:{port}{path}"
            body = None if payload is None else json.dumps(payload).encode("utf-8")
            request = urllib.request.Request(
                url, data=body,
                headers={"Content-Type": "application/json"} if body is not None else {},
                method="POST" if body is not None else "GET",
            )
            try:
                with urllib.request.urlopen(request, timeout=180) as response:
                    return response.status, json.loads(response.read().decode("utf-8"))
            except urllib.error.HTTPError as exc:
                raw = exc.read().decode("utf-8", errors="replace")
                try:
                    data = json.loads(raw)
                except json.JSONDecodeError:
                    data = {"ok": False, "error": raw or str(exc)}
                return exc.code, data

        def inspect_c4(self) -> None:
            config = vision_state.overlay_config
            params = urllib.parse.urlencode({
                "squares": "c4",
                "corners": str(config.get("corners", v28.vision.DEFAULT_CORNERS)),
                "auto_locate": "1" if config.get("auto_locate", False) else "0",
                "square_corners": str(config.get("square_corners", "")),
                "model": str(config.get("yolo_model", "")),
            })
            status, detected = self._loopback_json("/api/yolo/detect?" + params)
            if status >= 400 or not detected.get("ok"):
                return self.send_json(detected, status)
            info = (detected.get("piece_class_results") or {}).get("c4")
            if not isinstance(info, dict):
                return self.send_json({
                    "ok": True,
                    "version": WEB_VERSION,
                    "square": "c4",
                    "piece_class": None,
                    "confidence": 0.0,
                    "actionable": False,
                    "reason": "no_piece_detected_at_c4",
                    "prediction_image_url": detected.get("prediction_image_url"),
                    "detection": detected,
                })
            piece_class = str(info.get("piece_class", "")).strip()
            route = classify_c4_route(piece_class)
            response = {
                "ok": True,
                "version": WEB_VERSION,
                "square": "c4",
                "piece_class": piece_class,
                "confidence": float(info.get("confidence", 0.0)),
                "actionable": route is not None,
                "prediction_image_url": detected.get("prediction_image_url"),
                "detection_info": info,
            }
            if route is None:
                response["reason"] = "unsupported_piece_class"
                return self.send_json(response)
            token, expires_at = confirmations.issue(info, route)
            response.update(route)
            response.update({
                "confirmation_required": True,
                "confirmation_token": token,
                "confirmation_expires_at": expires_at,
                "confirmation_ttl_seconds": CONFIRMATION_TTL_SECONDS,
            })
            return self.send_json(response)

        def execute_confirmed_c4(self) -> None:
            try:
                body = self.read_json_body()
                if body.get("confirmed") is not True:
                    raise ValueError("Confirm that recognition is correct in the dialog before execution")
                enable_gate = body.get("enable_gate") is True
                record = confirmations.consume(str(body.get("confirmation_token", "")))
            except (ValueError, TypeError) as exc:
                return self.send_json({"ok": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST)
            route = record["route"]
            command = v28.arm.build_replay_move_command(ctrl_cfg, route["saved_move"])
            if enable_gate:
                command.append("--enable-gate")
            flow_flags = {
                "bishop": "--white-bishop-placement",
                "knight": "--white-knight-place-b1",
                "rook": "--white-rook-place-a1",
            }
            command.append(flow_flags[route["piece_type"]])
            if not ctrl_cfg.execute_enabled:
                result = {
                    "ok": False,
                    "error": "execution disabled; restart with --enable-execute",
                    "command": command,
                    "command_text": " ".join(command),
                }
                status = HTTPStatus.FORBIDDEN
            else:
                cancelled = run_state.cancel_current(
                    timeout=1.0, reason="v3_confirmed_c4",
                    expected_actions={"low-torque"},
                )
                if cancelled is not None:
                    time.sleep(0.2)
                action_names = {
                    "bishop": V3_C4_BISHOP_ACTION,
                    "knight": V3_C4_KNIGHT_ACTION,
                    "rook": V3_C4_ROOK_ACTION,
                }
                result = run_state.start(
                    action_names[route["piece_type"]],
                    command,
                    popen_kwargs={},
                )
                if cancelled is not None:
                    result["cancelled_previous"] = cancelled
                status = HTTPStatus.OK if result.get("ok") else HTTPStatus.CONFLICT
            result["v3_c4_confirmation"] = {
                "confirmed": True,
                "square": "c4",
                "piece_class": record["detection"].get("piece_class"),
                "confidence": record["detection"].get("confidence"),
                "flow_label": route["flow_label"],
                "gate_enabled": enable_gate,
            }
            return self.send_json(result, status)

        def do_GET(self):
            path = urllib.parse.urlparse(self.path).path
            if path in ("/", "/index.html"):
                return self.send_bytes(
                    HTML_PAGE.encode("utf-8"), "text/html; charset=utf-8"
                )
            if path in ("/dynamics", "/dynamics/"):
                return self.send_bytes(
                    DYNAMICS_PAGE.encode("utf-8"), "text/html; charset=utf-8"
                )
            if path == "/api/v3/dynamics/status":
                return self.dynamics_status()
            if path in ("/arm", "/arm/"):
                return self.send_bytes(
                    ARM_HTML_PAGE.encode("utf-8"), "text/html; charset=utf-8"
                )
            return super().do_GET()

        def do_POST(self):
            path = urllib.parse.urlparse(self.path).path
            if path == "/api/v3/dynamics/run":
                return self.run_dynamics_action()
            if path == "/api/v3/model-hold/auto":
                return self.set_auto_model_hold()
            if path == "/api/v3/c4/inspect":
                return self.inspect_c4()
            if path == "/api/v3/c4/execute":
                return self.execute_confirmed_c4()
            return super().do_POST()

    return Handler


def build_parser():
    parser = v28.build_parser()
    parser.description = "HiveRobot v3.0 confirmed C4 vision-to-motion controller"
    parser.set_defaults(port=DEFAULT_PORT, move_script=V30_MOVE_SCRIPT)
    parser.add_argument("--dynamics-script", default=V31_DYNAMICS_SCRIPT)
    parser.add_argument("--dynamics-model", default=V31_DYNAMICS_MODEL)
    parser.add_argument("--motion-dynamics", action="store_true",
                        help="enable bounded pose-dependent dynamics corrections during operational trajectories")
    parser.add_argument("--rigid-body-model", default=None,
                        help="seven-axis CAD rigid-body model JSON; operational trajectories only")
    parser.add_argument("--rigid-body-mode", choices=("shadow", "apply"), default="shadow",
                        help="shadow records references; apply requires complete physical inertials before serial opens")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    ctrl_cfg = v28.arm.ControlConfig(
        arm_script=os.path.abspath(args.arm_script),
        move_script=os.path.abspath(args.move_script),
        python_bin=args.python_bin,
        use_sudo=False,
        execute_enabled=args.enable_execute,
        run_log=args.run_log,
        moves_file=os.path.abspath(args.moves_file),
    )
    stream_state = v28.arm.StreamState()
    run_state = OperationalRunState(args.run_log, args, auto_hold_enabled=True)
    detector_cfg = v28.arm.DetectorConfig(
        hfov_deg=args.hfov_deg, depth_scale=args.depth_scale,
        depth_scale_set=args.depth_scale_set,
        roi=(args.roi_x0, args.roi_y0, args.roi_x1, args.roi_y1),
        min_depth_m=args.min_depth_m, max_depth_m=args.max_depth_m,
        close_margin_cm=args.close_margin_cm, min_area=args.min_area,
        max_area=args.max_area, depth_radius=args.depth_radius,
        aim_x_frac=args.aim_x_frac, aim_y_frac=args.aim_y_frac,
        target_depth_percentile=args.target_depth_percentile,
        scan_windows=args.scan_windows, window_w=args.window_w,
        window_h=args.window_h, window_step=args.window_step,
        no_rgb_dark_refine=args.no_rgb_dark_refine,
        dark_v_max=args.dark_v_max, dark_min_area=args.dark_min_area,
        dark_max_area=args.dark_max_area,
        arm_offset=(args.arm_offset_x, args.arm_offset_y, args.arm_offset_z),
        camera_forward_cm=args.camera_forward_from_shoulder_cm,
        camera_left_cm=args.camera_left_from_shoulder_cm,
        camera_up_cm=args.camera_up_from_shoulder_cm,
        camera_pitch_down_deg=args.camera_pitch_down_deg,
        grasp_strategy=args.grasp_strategy,
    )
    camera = v28.SharedCamera(stream_state)
    camera_cfg = v28.vision.CameraConfig(
        live_camera=True, rgb_topic=args.rgb_topic,
        depth_topic=args.depth_topic, jpeg_quality=args.jpeg_quality,
    )
    state = v28.vision.VisionState(
        args.calibration, args.output_dir, camera, camera_cfg,
        args.yolo_dataset_dir, args.yolo_docker_image, args.overlay_config,
    )
    v28.vision.start_ros_camera(camera, args.rgb_topic, args.depth_topic)
    import rospy
    from sensor_msgs.msg import CameraInfo

    def info_cb(msg):
        k = msg.K
        if k and k[0] > 0 and k[4] > 0:
            stream_state.set_intr(v28.arm.Intrinsics(
                fx=float(k[0]), fy=float(k[4]), cx=float(k[2]), cy=float(k[5])
            ))

    rospy.Subscriber(args.camera_info_topic, CameraInfo, info_cb, queue_size=1)
    threading.Thread(
        target=v28.arm.detector_loop,
        args=(stream_state, detector_cfg, args.detect_hz), daemon=True,
    ).start()
    server = ThreadingHTTPServer(
        (args.host, args.port),
        make_handler(state, stream_state, run_state, ctrl_cfg, args),
    )
    print(f"{WEB_VERSION}: http://{args.host}:{args.port}/", flush=True)
    print(f"Arm execution enabled: {ctrl_cfg.execute_enabled}; C4 confirmation required: True", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        rospy.signal_shutdown("v3.0 web server stopped")


if __name__ == "__main__":
    main()
