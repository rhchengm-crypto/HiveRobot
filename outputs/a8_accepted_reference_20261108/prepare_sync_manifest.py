"""Freeze the completed acceptance/reference evidence for a bounded update."""
from pathlib import Path
import hashlib
import json

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    prior_path = OUT / 'data_sync/read_only_prior_sha.json'
    prior = json.loads(prior_path.read_text(encoding='utf-8-sig'))
    if 'files' in prior:
        prior = prior['files']
    files = [p for p in OUT.rglob('*') if p.is_file() and '__pycache__' not in p.parts and 'data_sync' not in p.relative_to(OUT).parts]
    files += [OUT / 'data_sync/README.md', prior_path]
    evidence_only = [
        'outputs/a8_lower25_execution_20261108/trial_summary.json',
        'outputs/a8_lower25_execution_20261108/operator_acceptance_update.json',
        'outputs/a8_lower25_execution_20261108/README.md',
        'outputs/a8_lower25_execution_20261108/record_trial.py',
    ]
    installed = [
        'scripts/left_arm_board_task_pose.py',
        'scripts/build_left_arm_path_reference_library.py',
        'scripts/test_left_arm_path_reference_scope.py',
        'scripts/data/left_arm_validated_path_library.json',
        'scripts/data/left_arm_board_workspace_candidate.json',
        'scripts/data/board_workspace_trials/validation_20261108_a8_lower25.json',
    ]
    files += [ROOT / n for n in evidence_only + installed]
    entries = {}
    for p in sorted(set(files)):
        assert p.is_file() and p.suffix in {'.json', '.md', '.txt', '.py'}, p
        relative = p.relative_to(ROOT).as_posix()
        entries[relative] = dict(bytes=p.stat().st_size, sha256=sha(p))
        if relative in installed:
            runtime = relative[len('scripts/'):]
            entries[relative]['runtime_path'] = runtime
            if runtime in prior:
                value = prior[runtime]
                entries[relative]['expected_previous_sha256'] = value.get('sha256') if isinstance(value, dict) else value
    for n in ['build_left_arm_path_reference_library.py', 'test_left_arm_path_reference_scope.py', 'data/board_workspace_trials/validation_20261108_a8_lower25.json']:
        assert 'expected_previous_sha256' in entries['scripts/' + n], n
    ready = dict(schema=1, ready_for_sync=True, motion_issued=False,
                 activate_fitted_candidate=False, protected_baseline_overrides={}, files=entries)
    path = OUT / 'data_sync/sync_ready_manifest.json'
    path.write_text(json.dumps(ready, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(dict(manifest=str(path), files=len(entries), bytes=sum(r['bytes'] for r in entries.values()), sha256=sha(path)), ensure_ascii=False))

if __name__ == '__main__':
    main()
