# Remaining-height frame-selection diagnostics — 2026-11-08

This directory contains local numerical analysis only. It issued no motion, used no network, and changed no runtime model, gain, geometry, active configuration, or prepared execution stamp.

## Configuration and task

- Analysis starts from the actual encoder pose at the end of trial49465's same-session hold, not the saved A8 joint angles.
- Its model position is `[9.639697000830372, 406.65055148807676, 62.175188529761634] mm`.
- The additional10mm task keeps model XY fixed and requests Z`52.175188529761634mm`.
- The original analysis domain is the completed trial configuration SHA`b64370a7c0e4264c65b085b6b0db15fbe6f1ce1be66a489260e8640a988c1301`, including its earlier12-degree front software extension. This is **not** the restored active common configuration SHA`ff4d75df0ef85964ba153d267c5547ba036f63d075409ca0582174c6e836164d`.
- Software-domain changes do not certify mechanical travel. Model FK is not physical TCP, fingertip, or gripping-height metrology.

## Result

No complete additional10mm path was demonstrated. Explicit new-frame searches considered1/2/3/5degrees relative to the actual held wrist frame, retaining strict0.5mm position and0.005rad tracking tolerances and the original CAD obstacles. Tested domains were the original trial12-degree front domain; front-only totals17 and20degrees; and one final counterfactual with front unchanged and only shoulder_rotate lower/arm_roll upper bounds extended10degrees each. All full10mm endpoint searches failed their actual pose criteria; no failed candidate was promoted into an executable path. Unrestricted-orientation position diagnostics also retained approximately3.94mm residual, but this is numerical evidence, **not proof of physical unreachability**.

The final bounded local search minimized model Z at exact model XY, with new full-frame change at most5degrees and the original trial bounds. Its best endpoint was0.6278668mm below the held position. A fully checked in-place orientation transition and short vertical path achieved0.5823464mm model descent. It provides no demonstrated useful10mm solution.

The separate fixed-frame study reported a1.5mm requested path with1.316mm model descent inside the original position/orientation acceptance tolerances. That is a different numerical task from the exact-XY endpoint optimization here. These values must not be presented as directly comparable global maxima, nor as total physical remaining travel.

## Files and reproducibility

- `summary.json`: bounded case index, hashes, status and evidence limitations.
- `frame_selection_report.json`: original trial-domain explicit new-frame searches.
- `front_total17/`, `front_total20/`, `coupled_posture10/`: the three finite counterfactual studies; no activation.
- `free_orientation_position_crosscheck.json`: two independent position-only DE runs plus bounded least-squares diagnostics; free orientation is not an accepted task.
- `max_descent_new_frame_report.json`: final three-seed local minimum-Z endpoint and full connection audit.
- `selection_source_snapshot.py.txt`: exact analysis-source bytes saved with each frame-selection case.

Run from the repository with the existing local SciPy directory`tmp/gripper_geometry_deps` and preserved lower25`control_evidence`; no package installation or hardware is required:

```text
python -B -X utf8 outputs/a8_remaining_height_20261108/frame_selection/select_remaining10_frame.py
python -B -X utf8 outputs/a8_remaining_height_20261108/frame_selection/crosscheck_free_orientation_position.py
python -B -X utf8 outputs/a8_remaining_height_20261108/frame_selection/select_remaining10_frame.py --additional-front-deg 5 --label front_total17
python -B -X utf8 outputs/a8_remaining_height_20261108/frame_selection/select_remaining10_frame.py --additional-front-deg 8 --label front_total20
python -B -X utf8 outputs/a8_remaining_height_20261108/frame_selection/select_remaining10_frame.py --coupled-posture-10deg --label coupled_posture10
python -B -X utf8 outputs/a8_remaining_height_20261108/frame_selection/max_descent_new_frame.py
```

Searches stop here. These files contain no execution authorization or prepared stamp.
