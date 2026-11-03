# Original experiment evidence

Eight independent ZIP archives contain 4,747 original files (1,682,199,782 uncompressed bytes). Git stores their byte-exact contents as ordered chunks of at most 2 MiB in `chunks/`, avoiding large-request upload timeouts. `archive_manifest.json` records chunk order and hashes, every source file's path/size/SHA-256, and each original ZIP's SHA-256. `catalog.csv` provides a spreadsheet-readable index. No source bytes or original line endings were changed. Complete original ZIPs also remain on the analysis computer and Jetson.

Included: four complete RGB-D recordings, camera calibration and frame/run associations, local-day run logs, complete motion/hold JSONL history, final runtime JSONs, controller source files and experimental backups. Failed and rejected trials are retained. The recording dates use UTC; the experimental day is November 2, 2026 in America/Los_Angeles.

Restore into a **new empty offline directory**, from the repository root using Python 3.9 or later:

```bash
python outputs/board_drift_20261102/extract_raw_evidence.py --output outputs/board_drift_20261102/restored_dataset
```

The extractor first reassembles missing ZIPs from verified chunks, then verifies archive and member hashes and prevents traversal outside the output directory. To only reassemble ZIPs, run `python outputs/board_drift_20261102/archive_storage.py`. It never opens a serial port or executes archived code. Do not extract over a live Jetson installation. Archived deployment and motion scripts can control real hardware if executed separately.

For interpretation, start with the root `HANDOFF_20261102.md`, the trial CSV/JSON, and `figures/README.md`. MP4 files are lossy browsing copies; use original JPEG/NPY files for analysis. Encoding-based FK, image estimates and operator observations are separate evidence classes. The accepted approximately 2 mm drift is an operator estimate from one empty-load trial, not a measured global accuracy specification.
