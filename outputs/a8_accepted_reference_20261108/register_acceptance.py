"""Append operator acceptance to the executed A8 trial; no robot access."""
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json
import shutil

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
TRIAL = ROOT / "outputs/a8_lower25_execution_20261108"
REGISTRY = ROOT / "scripts/data/board_workspace_trials/validation_20261108_a8_lower25.json"
TEXT = "好的，那么我认为可以把此次运行标记为通过，并纳入参考库进行拟合，因为现在这个位置目测已到机械臂的极限，可以夹取除兵以外的其他全部棋子。"

def read(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

def main():
    audit = read(TRIAL / "execution_audit.json")
    assert len(audit["checks"]) == 23 and all(audit["checks"].values())
    assert audit["inline_hold"]["ok"] and audit["inline_hold"]["duration_s"] >= 5
    before = OUT / "before_acceptance"
    before.mkdir(parents=True, exist_ok=True)
    for name, source in {
        "validation_20261108_a8_lower25.json": REGISTRY,
        "trial_summary.json": TRIAL / "trial_summary.json",
        "operator_feedback.json": TRIAL / "operator_feedback.json",
        "trial_README.md": TRIAL / "README.md",
        "record_trial.py": TRIAL / "record_trial.py",
    }.items():
        if not (before / name).exists():
            shutil.copyfile(source, before / name)
    original = read(before / "validation_20261108_a8_lower25.json")
    acceptance_path = OUT / "operator_acceptance.json"
    acceptance = read(acceptance_path) if acceptance_path.exists() else {
        "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
        "original_response": TEXT,
        "accepted": True,
        "motion_accepted": True,
        "gripping_height_acceptable": True,
        "height_acceptance_scope": "operator-approved current height for non-pawn pieces",
        "applicable_pieces": ["king", "queen", "bishop", "knight", "rook"],
        "excluded_pieces": ["pawn"],
        "applicability_source": "operator visual assessment; each piece type has not been separately grasped",
        "jitter": False,
        "lateral_drift_observed": False,
        "contact": False,
        "orientation_acceptable": True,
        "hold_normal": True,
        "extra_motion_observed": False,
        "qualitative_only": True,
        "grasp_verified": False,
        "claw_closure_tested": False,
        "loaded_lift_tested": False,
        "operator_estimates_near_mechanical_limit": True,
        "mechanical_limit_independently_measured": False,
        "additional_descent_executed": False,
        "new_motion_authorized": False,
    }
    assert acceptance["original_response"] == TEXT
    save(acceptance_path, acceptance)
    save(TRIAL / "operator_acceptance_update.json", acceptance)
    record = dict(original)
    record.update({
        "experiment_id": "20261108_a8_v33_lower25_nonpawn",
        "status": "operator-accepted complete A8 empty-load path and non-pawn grasp posture at current height",
        "acceptance_updated_at_utc": acceptance["recorded_at_utc"],
        "physical_validation_accepted": True,
        "physical_validation_scope": "observed empty-load path, posture, height and same-serial hold only",
        "accepted": True,
        "height_passed": True,
        "fitting_eligible": True,
        "fitting_roles": ["qualitative-accepted-pose-and-branch", "quasi-static-feedback-torque-proxy"],
        "numeric_xyz_fitting_label_eligible": False,
        "geometry_ground_truth": False,
        "independent_tcp_measurement": False,
        "actual_piece_grasp_validated": False,
        "grasp_verified": False,
        "reference_seed_eligible": True,
        "successful_path_library_updated": False,
        "operator_validation": acceptance,
        "operator_feedback_history": [original["operator_feedback"], acceptance],
        "operator_feedback_is_original_preacceptance_observation": True,
        "encoder_endpoint_rad": {n: row["pos"] for n, row in audit["inline_hold"]["final"].items()},
        "acceptance_update_file": acceptance_path.relative_to(ROOT).as_posix(),
        "acceptance_update_sha256": sha(acceptance_path),
        "prior_registry_sha256": sha(before / "validation_20261108_a8_lower25.json"),
        "mechanical_limit_independently_measured": False,
    })
    evidence = TRIAL / "control_evidence"
    logs = list(evidence.rglob(record["motion"]["primary_log"]))
    traces = list(evidence.rglob(record["motion"]["trace"]))
    assert len(logs) == len(traces) == 1
    assert sha(logs[0]) == "abab93decd65f9f1a25cecab3c1e9b038dda4ed612f4c9d44cf55ca627855709"
    assert sha(traces[0]) == "271e30316bd53f824432c7da7a51afc30d29697ea8427691e0b694923b418582"
    record["evidence"] = {
        "directory": evidence.relative_to(ROOT).as_posix(),
        "control_log": logs[0].name,
        "trace": traces[0].name,
        "manifest": {p.relative_to(evidence).as_posix(): {"bytes": p.stat().st_size, "sha256": sha(p)} for p in [logs[0], traces[0]]},
    }
    record["sources"] = {n: h for n, h in read(TRIAL / "preflight.json")["source_hashes"].items() if not n.startswith("data/")}
    model_files = ["left_arm_board_workspace_candidate.json", "left_arm_general_dynamics.json", "left_arm_v3_1_dynamics.json"]
    record["model_snapshots"] = {name: sha(evidence / "models" / name) for name in model_files}
    assert record["model_snapshots"]["left_arm_board_workspace_candidate.json"] == original["trial_board_model_sha256"]
    save(REGISTRY, record)
    save(TRIAL / "trial_summary.json", record)
    print(json.dumps({"registered": True, "registry": str(REGISTRY), "motion_issued": False, "reference_build_pending": True}, ensure_ascii=False))

if __name__ == "__main__":
    main()
