# November 8 GitHub publication preflight

Read-only inspection completed on 2026-11-08. No robot connection or movement was used.

## Verified publication route

- Repository: `rhchengm-crypto/HiveRobot`; current branch `main`.
- Starting local HEAD: `e3dc7f2` (November 7 final publication).
- Origin: `https://github.com/rhchengm-crypto/HiveRobot.git`.
- Git Credential Manager (`credential.helper=manager`) provides existing credentials noninteractively. Authenticated repository GET succeeded and reports push permission. Credentials were neither printed nor saved.
- `gh` is not installed. Git HTTPS push plus the existing Python GitHub REST/upload implementation is available; no installation is required.
- `auth_preflight.json` records only availability and repository permissions.

## Prior publication pattern and important adaptation

The previous successful publication is `experiments-2026-11-07`, release ID `438072261`, public with 59 assets. Its receipt verifies 57 payloads (1,867,040,745 bytes), plus the manifest and restore helper. Full archive SHA256 values were checked against GitHub-computed asset digests; selected public download probes were checked independently. Code was pushed before bulky assets.

`outputs/github_sync_20261107/publication_tools/README.md` explicitly states that archived scripts are byte-preserving provenance copies with original `Path(__file__).resolve().parents[1]` assumptions. **Do not execute these archived copies in place.** Prepare the November 8 scripts under `tmp/`, or give new tools an explicit repository root. Change every date, tag, path, fixed count and release ID rather than importing November 7 assumptions.

The working upload algorithm is `tmp/github_assets_20261107.py`, also archived in the prior publication tools. It reads `git credential fill` into memory, creates a draft release, uploads streams to `uploads.github.com`, and accepts success only when name, size, state and remote SHA256 match the local original. `upload_nov7_assets_20261107.py` shows resumable per-asset receipts and a maximum of two simultaneous uploads.

## Recommended sequence and concrete commands

1. Inventory today's current source, JSON, registries and all new `outputs/*20261108*` artifacts; separate ordinary Git files from original binary archives. Keep failed trials, original operator feedback and before-acceptance records. Exclude private chat/Mac exports, credentials and temporary dependencies.
2. Run the focused hardware-free suite and save stdout/return code in `offline_verification.txt` and `verification_summary.json` (performed by this audit task).
3. Write `HANDOFF_20261108.md`, an exact code/model/trial mapping, and a SHA256 core manifest. Stage explicit UTF-8 paths with `git add --pathspec-from-file=- --pathspec-file-nul`; do not use blanket `git add .`.
4. Commit source/models/reference metadata/handoff first. Read remote main before any merge; retain unrelated remote commits. Push with `git push origin main`; confirm `git ls-remote origin refs/heads/main` equals the resulting local HEAD. No force push.
5. Publish the selected text evidence in bounded commits (previous run used about 50 MiB of raw file bytes per commit). Verify each committed blob's byte count and SHA256 with a single `git cat-file --batch` process; check remote HEAD after each push.
6. Adapt the asset helper to November 8 with tag `experiments-2026-11-08`, an explicit new `release.json`, and no hardcoded November 7 release/count assumptions. Equivalent intended calls are:

   ```text
   python -X utf8 tmp/github_assets_20261108.py create --commit <verified-core-commit>
   python -X utf8 tmp/github_assets_20261108.py upload <archive-path> --name <unique-asset-name>
   python -X utf8 tmp/github_assets_20261108.py verify
   python -X utf8 tmp/github_assets_20261108.py publish --body-file outputs/github_sync_20261108/release_body.md
   ```

   These describe the required adapted helper; the November 8 helper did not yet exist at inspection time. Do not run the November 7 create/publish commands against the old release.
7. Upload original RGB-D/control/preparation ZIP bytes without reducing frames. Include ignored standalone media/arrays through a lossless auxiliary ZIP with an internal member SHA256 manifest when not already fully covered by original archives. Check every ZIP CRC and member manifest before upload.
8. Upload a dated final runtime/model/learning snapshot if collected, then the publication manifest and a hardware-free restore helper. Publish the release only after all intended payload receipts pass; verify `draft=false` and repeat remote digest checks. Use public download SHA256 probes for the manifest/restore helper and a small payload.
9. Add final receipts, exact URLs, commit identifiers and restoration instructions to the handoff/index; commit/push those files, verify committed bytes and remote HEAD again.

## Code review scope and result

Inspected changes in `left_arm_board_grasp.py`, `left_arm_board_task_pose.py`, `build_left_arm_path_reference_library.py`, the new `left_arm_static_support.py`, their focused tests, and the accepted-reference record. No additional blocking defect was identified in this read-through. This is a focused offline review, not hardware verification.

- v33 static support keeps the original entry load anchor, bounds a separate absolute session residual, requires a fresh stationary seven-axis window, rejects repeated observation end times, and distinguishes estimated from last-applied bias at completion.
- The complete-frame endpoint task is attempted for every applicable numerical seed, with a reduced-axis result used only as another seed for full-frame polishing. Existing path and scene checks remain.
- New reference applicability is checked before seed ranking and when resolving an explicitly named reference. Pawn exclusions take precedence; legacy references retain their former behavior.
- Today's A8 acceptance explicitly keeps `grasp_verified=false`; the non-pawn qualification must not be described as an actual loaded grasp test or external measured TCP label. The newly fitted candidate was not promoted because grouped validation did not improve.
- The current active common configuration need not equal the historical transient trial configuration. Preserve both exact hashes and roles in publication; never overwrite a frozen historical model/source snapshot with the final active version.

## Files produced by this audit

- `publication_preflight.md`: this review and route.
- `auth_preflight.json`: credential availability and authenticated read-only repository permission result; no secret fields.
- `offline_verification.txt` and `verification_summary.json`: focused test output and status.

This audit task did not stage, commit, push, create a release, upload, mutate source/configuration, or contact the robot.
