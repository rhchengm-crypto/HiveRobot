# Four-trial RGB-D keyframe subset

This is a temporally subsampled sharing copy, not a replacement for the complete raw archive.

- ZIP: 28,779,607 bytes (28.78 decimal MB).
- Retains 255 matched RGB/depth pairs out of 2,317 pairs.
- Original JPEG and NPY bytes are unchanged; SHA-256 verified after packaging.
- Includes full original frames.jsonl and calibration/metadata files, plus selected_frames.jsonl for each session.
- Use selected_frames.jsonl to iterate the subset. Full frames.jsonl intentionally also references omitted frames.
- Selection retains both sides of action, run, and stage transitions, and prioritizes local RGB/depth changes and time coverage.
- Temporal gaps may reach 31.54 seconds; this subset cannot establish transient peaks or replace full-frame quantitative analysis.
- Complete original archives remain unchanged in ../raw_evidence/.
- selection_report.json and the internal keyframe_selection_manifest.json identify every retained/omitted paired frame and source hash.
- Unpaired RGB images are not included in the paired subset.

Rebuild: python outputs/board_drift_20261102/build_compact_rgbd.py
