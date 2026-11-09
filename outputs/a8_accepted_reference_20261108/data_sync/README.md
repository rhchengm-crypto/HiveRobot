# A8 accepted-reference sync gate

`sync_accepted_reference_template.py` is prepared but must not run before the
parent supplies `sync_ready_manifest.json` and explicitly sends READY.

Manifest shape:

```json
{
  "schema": 1,
  "ready_for_sync": true,
  "motion_issued": false,
  "activate_fitted_candidate": false,
  "protected_baseline_overrides": {},
  "files": {
    "outputs/a8_accepted_reference_20261108/README.md": {
      "bytes": 123,
      "sha256": "exact frozen SHA256"
    },
    "scripts/data/board_workspace_trials/validation_20261108_a8_lower25.json": {
      "bytes": 123,
      "sha256": "exact frozen SHA256",
      "runtime_path": "data/board_workspace_trials/validation_20261108_a8_lower25.json",
      "expected_previous_sha256": "exact old remote registry SHA256"
    }
  }
}
```

Files without `runtime_path` are evidence only. Each new registry/builder/test
installation needs `expected_previous_sha256` (use JSON null only when absent).
The three approved baseline changes are the task-pose module, shared path
library, and workspace candidate. All three are required, and workspace JSON
changes are restricted to shared reference fields. New fitting candidates stay
inside archived evidence; active rigid-body and hold models are protected.

The 30-file baseline is taken from the prior lower25 preflight with all five
learning hashes updated from its verified post-trial sync. The current Clearance
learning SHA is `52f1b83bc1a267e673cb1abcfa6981b25465111200daadade69b7ee7f31acd68`.
Existing protected tests cannot be overwritten by this helper. A separately named
new scope test can be added.

The helper validates local bytes, syntax, and bounded config changes, uploads a
small manifest-bound archive, checks both Web state endpoints twice for idleness,
verifies remote baseline hashes, compiles staged Python without importing it,
backs up previous allowed files, and replaces each allowed path atomically.
It does not open CAN/serial, issue motion, restart Web, kill a process, activate
fitted models, or access GitHub. Prior raw logs and RGB-D archives are not duplicated.
