# November 8 publication tools

These tools use the repository root three parent directories above this folder. They can run in place from a repository clone; they do not import robot control modules or access the robot.

- `github_assets.py`: create/reuse the date-specific draft release, upload original bytes, verify remote SHA256 and public download probes, then explicitly publish. Git Credential Manager credentials exist only in process memory. Existing assets are verified and never deleted or replaced.
- `upload_ready_assets.py`: validate the original inventory archives and final runtime/auxiliary ZIPs, then upload with at most two streams. Individual per-asset receipts allow retries. It cannot publish a release.
- `../restore_publication_data.py`: a public, credential-free downloader. With no flags it lists manifest assets and byte counts. `--download --group <group>` downloads and checks original bytes; it never extracts archives or issues a robot command.

Example commands from the repository root:

```text
python outputs/github_sync_20261108/publication_tools/github_assets.py verify --require-manifest
python outputs/github_sync_20261108/restore_publication_data.py
python outputs/github_sync_20261108/restore_publication_data.py --group all --download
```

The create/upload/publish operations mutate GitHub and should only be invoked within an authorized publication task. Read `HANDOFF_20261108.md`, `publication_manifest.json`, and the final publication receipts for actual commit IDs, asset coverage, checksums, and experiment qualifications. No credential or private chat export is included here.
