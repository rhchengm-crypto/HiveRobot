# 2026-09-13 robot run evidence

This directory preserves the 16 user-provided Web command-output attachments from 14 distinct runs, including two pairs of separately pasted copies. Each `.log` file keeps the attachment text unchanged. `index.json` records attachment IDs, timestamps, return codes, SHA-256 hashes and any parsable final training status. `robot_saved_moves_observed.json` preserves four new pawn/rook saved-move poses observed in an earlier read-only Web response; it is a partial transcription, not a full robot-file export or a claim that these moves are trained.

These are **Web output excerpts**, not complete motor traces. The Web controller retained only the last 8,000 characters of stdout for several runs; the raw attachment files therefore cannot reconstruct earlier C4 motion or prove the cause of visible shaking. The new v2.8 full-run-log feature was added today, but no complete C4 run log was supplied for archival.

Key late-day observations from the archived logs:

| Start time | B1 `arm_roll` final error | B1 wrist final error | Placement_C4 `wrist_side` final error | Note |
|---|---:|---:|---:|---|
| 20:24:03 | +0.918° | −0.809° | +2.579° | Destination hold learning was overwritten by the older source snapshot. |
| 20:37:07 | +0.896° | −0.240° | +2.623° | Hold-learning record again started from the old sample. |
| 20:44:20 | +0.525° | −0.831° | +2.973° | `v2.8-multiflow-anchor-save-merge`; persisted B1 hold-learning sample 3. |
| 20:50:32 | +0.066° | −0.831° | +3.453° | All B1 non-wrist joints inside 0.5°; B1 wrist still outside; operator suspects board contact. |

The last B1 run completed (`returncode=0`) but reported `training incomplete` because of wrist. The Claw Home motion also recorded a −1.880° wrist drift. This is consistent with the operator's observation that the placed piece pushes on the wrist, but the logs alone do not prove physical contact. Placement_C4's separate Clearance `wrist_side` residual remains unresolved and does not block follow-up by user choice.

Robot-private learning files such as `DM_Control_Python/data/left_arm_v2_8_local_target_bias.json`, Clearance bias files and the full saved-moves file were not exported: SSH and the robot Web API were unavailable during the archive step. No repository copy should overwrite those robot files. The local geometry anchor JSON in `scripts/data/chess_crown_geometry_v2_8.json` is tracked separately; it includes the B1 placement anchor confirmed by the operator and imported to the robot earlier today.
