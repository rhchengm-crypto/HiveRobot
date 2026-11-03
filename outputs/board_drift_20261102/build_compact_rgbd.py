"""Build a size-limited, lossless keyframe subset; never delete original evidence."""
import hashlib
import heapq
import io
import json
import zipfile
from pathlib import Path

import cv2
import numpy as np

BASE = Path(__file__).resolve().parent
DEST = BASE / 'compact_rgbd'
DEST.mkdir(exist_ok=True)
manifest = json.loads((BASE / 'raw_evidence/archive_manifest.json').read_text())
members = {e['path']: e for e in manifest['members']}
archives = {}
costs = {}
for part in manifest['parts']:
    z = zipfile.ZipFile(BASE / 'raw_evidence' / part['file'])
    archives[part['file']] = z
    costs.update({e.filename: e.compress_size for e in z.infolist()})


def read(path):
    e = members[path]
    raw = archives[e['archive_part']].read(path)
    assert hashlib.sha256(raw).hexdigest() == e['sha256']
    return raw


sessions = []
for case in json.loads((BASE / 'experiment_trials.json').read_text())['trials']:
    folder = 'data/rgbd_audits/' + Path(case['raw_RGBD_source_directory']).name
    rows = [json.loads(s) for s in read(folder + '/frames.jsonl').decode().splitlines()]
    paired, features, sizes = [], [], []
    for row in rows:
        stem = folder + '/%06d' % row['index']
        if stem + '.jpg' not in members or stem + '.npy' not in members:
            continue
        rgb = cv2.imdecode(np.frombuffer(read(stem + '.jpg'), np.uint8), 1)
        depth = np.load(io.BytesIO(read(stem + '.npy')), allow_pickle=False)
        # Spatial descriptors retain local motion despite a largely static background.
        small = cv2.resize(rgb, (32, 24), interpolation=cv2.INTER_AREA).astype(float) / 255
        d = depth.astype(np.float32)
        if np.nanmax(d) > 100:
            d /= 1000  # millimetres -> metres; descriptor only, original bytes unchanged
        valid = np.isfinite(d) & (d > 0)
        d = np.where(valid, np.minimum(d, 3), 0)
        ds = cv2.resize(d, (32, 24), interpolation=cv2.INTER_AREA)
        features.append(np.r_[small.ravel(), ds.ravel() / 3])
        paired.append(row)
        sizes.append(costs[stem + '.jpg'] + costs[stem + '.npy'])
    f = np.asarray(features)
    t = np.asarray([r['rgb_stamp'] for r in paired])
    mandatory = {0, len(paired) - 1}
    def state(r):
        return (r.get('action'), r.get('running'), r.get('full_log_path'),
                (r.get('tracking') or {}).get('stage'))
    for i in range(1, len(paired)):
        if state(paired[i]) != state(paired[i - 1]):
            mandatory.update((i - 1, i))
    sessions.append(dict(id=case['id'], folder=folder, rows=paired, f=f, t=t,
                         sizes=sizes, selected=mandatory, mandatory=set(mandatory)))
    print(case['id'], 'paired frames', len(paired), 'mandatory', len(mandatory), flush=True)

# Split intervals at the frame least represented by interpolation of retained frames.
# The time term also preserves coverage in stationary stretches.
heap = []


def interval(si, a, b):
    if b <= a + 1:
        return
    s = sessions[si]
    times = s['t']
    span = times[b] - times[a]
    u = (times[a + 1:b] - times[a]) / max(span, 1e-9)
    expected = s['f'][a] + u[:, None] * (s['f'][b] - s['f'][a])
    residual = np.abs(s['f'][a + 1:b] - expected)
    local = np.mean(np.sort(residual, axis=1)[:, -max(1, residual.shape[1] // 10):], axis=1)
    scores = local + 0.004 * np.minimum(times[a + 1:b] - times[a], times[b] - times[a + 1:b])
    i = a + 1 + int(np.argmax(scores))
    heapq.heappush(heap, (-float(scores[i - a - 1]), si, a, b, i))


budget = 28_800_000  # reserve room for full metadata, selection manifest and ZIP headers
used = sum(sum(s['sizes'][i] for i in s['selected']) for s in sessions)
assert used < budget, 'Mandatory transition frames exceed budget'
for si, s in enumerate(sessions):
    for a, b in zip(sorted(s['selected']), sorted(s['selected'])[1:]):
        interval(si, a, b)
while heap:
    _, si, a, b, i = heapq.heappop(heap)
    s = sessions[si]
    if used + s['sizes'][i] > budget:
        continue
    s['selected'].add(i)
    used += s['sizes'][i]
    interval(si, a, i)
    interval(si, i, b)

report = {'target_bytes': 30_000_000, 'originals_deleted': False,
          'method': 'Time-coverage and RGB/depth interpolation-error keyframe selection; both sides of action/run/stage transitions retained',
          'pixel_changes': False, 'depth_precision_changes': False,
          'limitation': 'Temporal subsampling: use full original archive for peak/transient or frame-by-frame quantitative analysis.',
          'trials': [], 'files': []}
package = DEST / 'four_trials_rgbd_keyframes.zip'
with zipfile.ZipFile(package, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=9) as out:
    for s in sessions:
        indices = sorted(s['selected'])
        kept = [s['rows'][i] for i in indices]
        gaps = np.diff([r['rgb_stamp'] for r in kept])
        info = dict(trial=s['id'], original_pairs=len(s['rows']), retained_pairs=len(kept),
                    mandatory_transition_pairs=len(s['mandatory']),
                    max_time_gap_s=float(max(gaps, default=0)),
                    median_time_gap_s=float(np.median(gaps)) if len(gaps) else 0,
                    retained_indices=[r['index'] for r in kept],
                    omitted_indices=[r['index'] for i, r in enumerate(s['rows']) if i not in s['selected']])
        report['trials'].append(info)
        paths = [s['folder'] + '/%06d%s' % (r['index'], ext) for r in kept for ext in ('.jpg', '.npy')]
        # Preserve complete original timing metadata, including omitted-frame records.
        paths += [p for p in members if p.startswith(s['folder'] + '/') and Path(p).suffix not in ('.jpg', '.npy')]
        for p in paths:
            raw = read(p)
            out.writestr(p, raw)
            report['files'].append({'path': p, 'size': len(raw), 'sha256': members[p]['sha256']})
        out.writestr(s['folder'] + '/selected_frames.jsonl', ''.join(json.dumps(r) + '\n' for r in kept))
    out.writestr('keyframe_selection_manifest.json', json.dumps(report, indent=2))

with zipfile.ZipFile(package) as z:
    assert z.testzip() is None
    for e in report['files']:
        raw = z.read(e['path'])
        assert hashlib.sha256(raw).hexdigest() == e['sha256']
report['package'] = {'file': package.name, 'size_bytes': package.stat().st_size,
                     'sha256': hashlib.sha256(package.read_bytes()).hexdigest()}
assert package.stat().st_size <= 30_000_000, 'Actual ZIP exceeded target'
(DEST / 'selection_report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
for z in archives.values():
    z.close()
print(json.dumps({'package': report['package'], 'trials': [{k: v for k, v in t.items() if k not in ('retained_indices', 'omitted_indices')} for t in report['trials']]}, indent=2))
