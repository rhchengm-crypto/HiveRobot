# November 8 evidence publication inventory

Scope: 14 experiment directories; 1000 files; 775,814,629 bytes including compressed/extracted duplicates.

| Directory | Files | Bytes | Archives | Archive bytes |
|---|---:|---:|---:|---:|
| outputs/a7_home_20261108 | 34 | 55,919,444 | 2 | 54,837,719 |
| outputs/a7_repeat_20261108 | 90 | 262,341,319 | 2 | 84,929,572 |
| outputs/a8_accepted_reference_20261108 | 38 | 1,581,900 | 1 | 313,868 |
| outputs/a8_height_followup_20261108 | 43 | 1,066,826 | 3 | 375,996 |
| outputs/a8_lower25_execution_20261108 | 220 | 117,033,157 | 3 | 90,316,788 |
| outputs/a8_reference_execution_20261108 | 135 | 99,165,776 | 3 | 82,179,014 |
| outputs/a8_reference_home_20261108 | 48 | 63,167,258 | 2 | 61,222,327 |
| outputs/a8_remaining_height_20261108 | 41 | 834,031 | 1 | 155,907 |
| outputs/a8_saved_reference_20261108 | 22 | 417,486 | 1 | 91,377 |
| outputs/a8_static_support_fix_20261108 | 26 | 563,084 | 1 | 64,181 |
| outputs/a8_test_20261108 | 44 | 3,809,759 | 2 | 1,091,962 |
| outputs/a8_v33_execution_20261108 | 191 | 114,059,551 | 3 | 91,720,317 |
| outputs/a8_v33_home_20261108 | 66 | 55,852,725 | 2 | 53,314,403 |
| outputs/camera_access_20261108 | 2 | 2,313 | 0 | 0 |

Release: retain all 26 original ZIPs unchanged (520,613,431 bytes). Largest 87,788,902 bytes; every existing archive is below 2 GB.
Git text tier: 114,527,893 bytes before deduplication. Keep explanatory reports, source snapshots, trial models, failure diagnostics, fitting inputs/results, acceptance history, plans, and control traces below 50 MiB.

Do not add the 138,752,252-byte rolling motion log directly to Git. Its exact SHA256 is covered by the original A7 control ZIP and listed in data_inventory.json. The 24,801,098-byte rolling hold log is also archive-covered; Git copies are optional. Preserve run-specific traces and summaries in Git.

Unarchived auxiliary media: 4 files, 221,920 bytes; package these in a separate ZIP if nonzero.
Duplicate file-content storage: 18,118,070 bytes (compressed/extracted duplication not all counted here).
High-confidence hygiene findings: 0. No credential value is stored in this report.

## Original archives

| Path | Bytes | SHA256 | RGB | Depth | CRC |
|---|---:|---|---:|---:|---|
| outputs/a7_home_20261108/control_evidence.zip | 189,996 | `ab183e6d7dd7a67197290fb9859d37eea13eaa35d48397c11f822d2b7a0d139d` | 0 | 0 | True |
| outputs/a7_home_20261108/rgbd_evidence.zip | 54,647,723 | `5a7f8cc1446f8106569bd47b304405147f47d082f2fc927236ab30a005ccf621` | 437 | 437 | True |
| outputs/a7_repeat_20261108/control_evidence.zip | 43,385,635 | `253f123d481babf7fa685cdf2438a5dafedc407d1aaae03dbc79576777d08278` | 1 | 1 | True |
| outputs/a7_repeat_20261108/rgbd_evidence.zip | 41,543,937 | `47de71003fcec61672f1e49129ced79344150e9857ef2cbdcf5b2ef365e1179b` | 340 | 340 | True |
| outputs/a8_accepted_reference_20261108/data_sync/accepted_reference_evidence.zip | 313,868 | `b923a26047fdb3b7e63a98d89ad09ca915bb5eeab7f6a18777546fa088474877` | 0 | 0 | True |
| outputs/a8_height_followup_20261108/deployment/a8_height_followup_bundle.zip | 56,704 | `eecd11b97c2b683a52c8ae41bd5fe77a107c57ba70c34a8fa1e939c03f24d66d` | 0 | 0 | True |
| outputs/a8_height_followup_20261108/deployment/analysis_evidence.zip | 245,122 | `509583220c2552b36815862792eb2a16713daf97cf85c4cb06922eac26a7fcc5` | 0 | 0 | True |
| outputs/a8_height_followup_20261108/deployment/deployment_evidence.zip | 74,170 | `82264df1f83dd227382052504f9684d2c9036a7f1f8f736a4f7434c46c04b844` | 0 | 0 | True |
| outputs/a8_lower25_execution_20261108/control_evidence.zip | 4,587,329 | `d20c0877ac2b721db8a9c8405a2617cb34d0d5811d1d25651872417709d6696f` | 0 | 0 | True |
| outputs/a8_lower25_execution_20261108/post_analysis_sync/analysis_evidence.zip | 546,605 | `2624a5274ddad39c333b973209c7e93eceaf450ca7ea686de3a5848b23e0c47c` | 0 | 0 | True |
| outputs/a8_lower25_execution_20261108/rgbd_evidence.zip | 85,182,854 | `84e97b5f18ab31606ff6545c230838d1314c55ede16d3d544198489d2c6aeb2a` | 674 | 674 | True |
| outputs/a8_reference_execution_20261108/analysis_evidence.zip | 60,398 | `8dafcd282b60b6c5027873c319f9b40e1fb7908b56a4192ce3eb58b41e58a1cb` | 0 | 0 | True |
| outputs/a8_reference_execution_20261108/control_evidence.zip | 2,995,741 | `1bb9f48540b4d53a0cf72c9227aa3a3cae093210d86a20febae79cb6ff7abfcc` | 1 | 0 | True |
| outputs/a8_reference_execution_20261108/rgbd_evidence.zip | 79,122,875 | `6d763847a531b8c93da80b3b0ffd58cd0e470e66502a66b3fe1450181094b402` | 638 | 638 | True |
| outputs/a8_reference_home_20261108/control_evidence.zip | 376,867 | `974af8b5d48ec32632edb559a7401cf3a239543b9dd0356d00b00c5b49962917` | 0 | 0 | True |
| outputs/a8_reference_home_20261108/rgbd_evidence.zip | 60,845,460 | `058a71e50cecb71acf9b9f75803dfa25bcc456e2cdd9a88c11efa758a3e8ab39` | 480 | 480 | True |
| outputs/a8_remaining_height_20261108/data_sync/analysis_evidence.zip | 155,907 | `6e8355e933fdb4fd295686fbf644b5f0f2ced0a9a35d9deab28a63352583ce99` | 0 | 0 | True |
| outputs/a8_saved_reference_20261108/saved_reference_evidence.zip | 91,377 | `d2148fc865b56869c20a3e76b0651c0e59cd06bff1717584db1f68576bbec9ea` | 0 | 0 | True |
| outputs/a8_static_support_fix_20261108/a8_v33_static_support_bundle.zip | 64,181 | `0531d8d63722d4c64d82c91ff95647dc456f545be01cabe92e776d5c133789dc` | 0 | 0 | True |
| outputs/a8_test_20261108/offline_evidence.zip | 525,834 | `1dff56e1e6c99bbdb3ac21406158819ca2e34f1ad63e95d3bfd7496e4d2c39f7` | 1 | 0 | True |
| outputs/a8_test_20261108/orientation_retry_evidence.zip | 566,128 | `eeecfa7a8704ece92ce05e9c6b391a7890e4e0fc3c627176932070341994e242` | 1 | 0 | True |
| outputs/a8_v33_execution_20261108/analysis_evidence.zip | 96,795 | `c4d8914d6d5736f0741fc411ce25738dc4709523696ccd6afd7876cd1f39d8c2` | 0 | 0 | True |
| outputs/a8_v33_execution_20261108/control_evidence.zip | 3,834,620 | `ed2da994819b12092abc1dd0c160fb11cba68e5dcd857b040c3905afed935f77` | 2 | 1 | True |
| outputs/a8_v33_execution_20261108/rgbd_evidence.zip | 87,788,902 | `8183672a9a07584cd8dad56443412c7caaa3f9cc356b6bd5cbb1fcdbdc0a8a6b` | 702 | 702 | True |
| outputs/a8_v33_home_20261108/control_evidence.zip | 470,603 | `682552f6e66436c03e337f433e36e0835384d212d52a6a4e5778778ff248083b` | 0 | 0 | True |
| outputs/a8_v33_home_20261108/rgbd_evidence.zip | 52,843,800 | `9f6cc256d306e10b1d66a4eced104a5e6991165a9dc0bb0f7aad67e3457d08b6` | 419 | 419 | True |

This inventory does not publish, download, alter runtime files, query motors, or issue movement. See remote_gap_audit.json for live Jetson filesystem coverage.

## Final publication plan and remote gap closure

All 1,000 original local evidence files have planned coverage in every_file_coverage.json. The 13 generated Python bytecode-cache files are explicitly excluded. Private migration/chat exports, SSH keys and local dependency caches are excluded; they are not scientific source evidence.

29 lossless Release payloads total **571,848,294 bytes**: the 26 original ZIPs, four auxiliary JPGs in auxiliary_media_20261108.zip (212,482 bytes), final_runtime_snapshot_20261108.zip (50,290,144 bytes; 219 payload files), and remote_gap_evidence_20261108.zip (732,237 bytes; 26 source payloads plus manifest). No raw RGB-D frames were removed or recompressed lossily.

Live read-only Jetson inventory found seven November 8 RGB-D sessions, totaling 3,690 RGB/depth pairs. Every original raw filename and byte size exists in a local full RGB-D ZIP; prior download receipts separately verify hashes. A further 1,021 nonraw Jetson records were SHA-covered locally. The remaining 26 unique remote files were captured in the gap ZIP with member hashes and local CRC/SHA verification. The current server web.log grew after inventory; its finite prefix capture preserves the entire earlier inventoried prefix, records both size/mtime samples and is explicitly not sealed. This publication does not stop the server.

Every publication asset has a stable name, exact byte count and SHA256 in asset_plan.json. Actual Git and GitHub upload verification is recorded by the publishing parent agent.
