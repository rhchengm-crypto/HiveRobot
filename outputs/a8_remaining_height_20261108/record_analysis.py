"""Record offline remaining-height evidence; do not contact or control hardware."""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent

def read(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def save(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

fixed = read(OUT / "fixed_frame/report.json")
new_frame = read(OUT / "frame_selection/max_descent_new_frame_report.json")
assert fixed["conservative_branch_passed"]
assert fixed["dense_branch_audit"]["complete_swept_edges_checked"]
assert fixed["motion_issued"] is False
assert new_frame["motion_issued"] is False
trial_path = ROOT / "outputs/a8_lower25_execution_20261108/common_trial_config.json"
active_path = ROOT / "scripts/data/left_arm_board_workspace_candidate.json"
assert sha(trial_path) == "b64370a7c0e4264c65b085b6b0db15fbe6f1ce1be66a489260e8640a988c1301"
assert sha(active_path) == "ff4d75df0ef85964ba153d267c5547ba036f63d075409ca0582174c6e836164d"

report_names = [
    "fixed_frame/report.json",
    "fixed_frame/independent_acceptance_boundary_polish.json",
    "frame_selection/frame_selection_report.json",
    "frame_selection/free_orientation_position_crosscheck.json",
    "frame_selection/front_total17/frame_selection_report.json",
    "frame_selection/front_total20/frame_selection_report.json",
    "frame_selection/coupled_posture10/frame_selection_report.json",
    "frame_selection/max_descent_new_frame_report.json",
]
reports = {name: {"bytes": (OUT / name).stat().st_size, "sha256": sha(OUT / name)} for name in report_names}
summary = {
    "schema": 1,
    "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
    "kind": "offline-planning-diagnostic-not-execution",
    "user_question": "能否补足剩余约10mm；否则还能下降多少",
    "basis_trial": "validation_20261108_a8_lower25.json",
    "basis_actual_encoder_feedback": fixed["actual_held_encoder_rad"],
    "basis_model_board_mm": fixed["actual_held_model_board_mm"],
    "approximate_physical_remaining_gap_mm": 10,
    "gap_source": "operator visual observation, not calibrated TCP or contact metrology",
    "requested_actual_feedback_relative_target_board_mm": [
        fixed["actual_held_model_board_mm"][0],
        fixed["actual_held_model_board_mm"][1],
        fixed["actual_held_model_board_mm"][2] - 10,
    ],
    "trial_board_model_sha256": sha(trial_path),
    "active_restored_board_model_sha256": sha(active_path),
    "domain_note": "Calculations use the completed trial's front lower extension of12deg; that temporary configuration is NOT active now. Other extended domains are offline counterfactuals, not mechanical-limit qualification.",
    "full10mm_complete_path_found": False,
    "fixed_actual_desired_frame": {
        "complete_path_passed": True,
        "requested_descent_mm": fixed["conservative_continuous0p5mm_grid_descent_mm"],
        **fixed["dense_branch_audit"],
        "position_tolerance_mm": fixed["position_tolerance_mm"],
        "rotation_tolerance_rad": fixed["strict_frame_tolerance_rad"],
        "rounded_down_model_displacement_mm": 1.0,
        "not_global_or_physical_maximum": True,
        "physical_displacement_verified": False,
    },
    "new_frame_up_to5deg_exact_XY_local_minimum": {
        "endpoint_descent_mm": new_frame["best_endpoint_only"]["endpoint_descent_mm"],
        "complete_connection_plus_descent": new_frame["complete_connection_plus_descent"],
        "connected_model_descent_mm": new_frame["model_predicted_achieved_descent_mm"],
        "improves_best_demonstrated_descent": False,
        "scope": "Local exact-XY endpoint objective differs from the fixed-frame path's unchanged0.5mm position acceptance budget; neither is a proven global maximum.",
    },
    "full10mm_diagnostics": [
        "Actual-held full frame frozen:12 seeds failed; prior total3D residual4.094mm is not height error.",
        "Explicit new requested full frames within1/2/3/5deg: no accepted full10mm endpoint.",
        "Two differential-evolution position-only crosschecks with free orientation: best residual approximately3.937mm; not an accepted orientation task or proof of unreachability.",
        "Offline front lower total17/20deg proposals, other six bounds unchanged: no accepted full10mm endpoint.",
        "Offline coupled proposal:front12 unchanged, roll upper+10deg and shoulder_rotate lower-10deg, new frame<=5deg: no accepted full10mm endpoint.",
    ],
    "numerical_bottleneck": "arm_roll upper software bound on fixed-frame continuation; shoulder_rotate lower and arm_roll upper in the broader full10mm position search. Front lower extension alone did not resolve it.",
    "safety_and_preservation": {
        "motion_issued": False,
        "motor_or_web_command_issued": False,
        "active_model_or_runtime_modified": False,
        "prepared_execution_stamp_created": False,
        "model_hold_changed": False,
        "adaptive_learning_changed": False,
        "ordinary_clearance_shoulder_rotate_skip_changed": False,
        "physical_estop_changed": False,
        "shoulder_rotate_gain_changed": False,
        "pose_tolerances_relaxed": False,
        "per_square_XYZ_offset_added": False,
        "saved_angle_replay_used": False,
    },
    "executed": False,
    "motion_issued": False,
    "height_passed": False,
    "physical_validation_accepted": False,
    "physical_validation_complete": False,
    "fitting_eligible": False,
    "successful_path_library_updated": False,
    "executable": False,
    "global_or_physical_maximum_proven": False,
    "report_files": reports,
}
save(OUT / "summary.json", summary)
registry = dict(summary)
registry["evidence_directory"] = "outputs/a8_remaining_height_20261108"
registry_path = ROOT / "scripts/data/board_workspace_trials/planning_20261108_a8_remaining_height.json"
save(registry_path, registry)
print(json.dumps({"summary": str(OUT / "summary.json"), "registry": str(registry_path), "registry_sha256": sha(registry_path)}, ensure_ascii=False))
