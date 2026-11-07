"""Merge versioned offline coverage reports; preserve every point's source."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reports', type=Path, nargs='+', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--markdown', type=Path, required=True)
    parser.add_argument('--allow-partial', action='store_true')
    args = parser.parse_args()
    rows, sources, model_hashes = {}, {}, set()
    for path in args.reports:
        data = json.loads(path.read_text(encoding='utf-8'))
        if data.get('motion_issued') is not False:
            raise ValueError('only explicit no-motion reports can be merged')
        model_hashes.add(data['board_model_sha256'])
        sources[str(path)] = dict(sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                                  code_sha256=data['source_sha256'])
        for row in data['results']:
            if row['point'] in rows:
                raise ValueError('duplicate point '+row['point'])
            rows[row['point']] = dict(row, source_report=str(path))
    if len(model_hashes) != 1:
        raise ValueError('different geometric model snapshots cannot be silently combined')
    expected = {f+str(r) for f in 'abcd' for r in range(1, 9)} | {'between-'+str(i) for i in range(8)}
    if set(rows)-expected:
        raise ValueError('unexpected point IDs')
    if set(rows) != expected and not args.allow_partial:
        raise ValueError('40-point coverage is not yet complete')
    report = dict(schema=1, motion_issued=False, coverage_complete=set(rows)==expected,
        physical_validation_complete=False, nominal_model_only=True,
        board_model_sha256=next(iter(model_hashes)), reports=sources,
        tested=len(rows), required_points=40, successes=sum(r['success'] for r in rows.values()),
        failure_codes=dict(Counter(r.get('failure_code') for r in rows.values() if not r['success'])),
        results=[rows[key] for key in sorted(rows)],
        remaining_points=sorted(expected-set(rows)),
        scope='full candidate paths under declared geometry; no claim of physical or occupied-board validation')
    args.output.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    def symbol(label):
        row = rows.get(label)
        return '?' if row is None else 'P' if row['success'] else 'U' if row.get('failure_code')=='planning_budget_exhausted' else 'X'
    lines = ['# A1–D8 离线路径覆盖', '',
        f"已检查 {len(rows)}/40；完整路径通过 {report['successes']}；未实机执行。", '',
        'P：声明的模型包络下路径通过；U：搜索预算耗尽；X：其他规划/复核失败；?：未完成。', '',
        '**P 不表示实物抓取已验证；U/X 不证明物理不可达。**', '', '| 排 | A | B | C | D |', '|---|---|---|---|---|']
    for rank in range(8, 0, -1):
        lines.append('| '+str(rank)+' | '+' | '.join(symbol(file+str(rank)) for file in 'abcd')+' |')
    lines += ['', '## 格间及边界点', '', '| 点 | X mm | Y mm | 结果 |', '|---|---:|---:|---|']
    for index in range(8):
        label = 'between-'+str(index)
        if label in rows:
            point = rows[label]['target_board_mm']
            lines.append(f'| {label} | {point[0]} | {point[1]} | {symbol(label)} |')
    lines += ['', '## 未通过点', '']
    lines += ['- '+label+'：'+str(row.get('failure_code')) for label,row in sorted(rows.items()) if not row['success']]
    lines += ['', '每点的模型哈希、代码哈希、余量、失败详情及原报告路径保存在同名 JSON 总表中。',
              '声明包络不含完整手指、线束、支架、其他棋子及全部自碰撞；真实机械限位尚未确认。', '']
    args.markdown.write_text('\n'.join(lines), encoding='utf-8')
    print(json.dumps({key:report[key] for key in ['tested','successes','failure_codes','coverage_complete']}))


if __name__ == '__main__':
    main()
