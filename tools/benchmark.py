"""Bounded, reproducible CLI runs; standard library only; failures stay failures."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import statistics
import subprocess
import time
import platform

SIX = ['uni_bugtrap', 'uni_wall', 'quad2d_bugtrap', 'quad2d_hole', 'quad3d_block', 'quad3d_window']


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--executable', type=Path, required=True)
    p.add_argument('--scenarios', nargs='+', type=Path,
                   default=[Path(__file__).resolve().parents[1]/'examples'/f'dynobench_{s}.yaml' for s in SIX])
    p.add_argument('--seeds', nargs='+', type=int, default=[41])
    p.add_argument('--repeats', type=int, default=1,
                   help='Independent process repetitions per scenario/seed; RNG seed stays fixed')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--process-timeout', type=float, default=15)
    args = p.parse_args()
    if not math.isfinite(args.process_timeout) or args.process_timeout <= 0:
        p.error('process-timeout must be positive and finite')
    if any(s < 0 or s > 2**32-1 for s in args.seeds) or len(set(args.seeds)) != len(args.seeds):
        p.error('seeds must be distinct uint32 integers')
    if args.repeats < 1:
        p.error('repeats must be positive')
    executable = args.executable.resolve(strict=True)
    scenarios = [s.resolve(strict=True) for s in args.scenarios]
    if len({s.stem for s in scenarios}) != len(scenarios):
        p.error('scenario names must be distinct to avoid output collisions')
    args.output.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[1]
    sources = sorted(list((root/'include/flask').glob('*.h')) + list((root/'src').glob('*.cpp')))
    def digest(path):
        return hashlib.sha256(path.read_bytes()).hexdigest()
    manifest = dict(executable=str(executable), executable_sha256=digest(executable),
                    seeds=args.seeds, repeats=args.repeats, process_timeout=args.process_timeout,
                    python=platform.python_version(), platform=platform.platform(),
                    scenarios={str(s): digest(s) for s in scenarios},
                    sources={str(s.relative_to(root)): digest(s) for s in sources},
                    runner_sha256=digest(Path(__file__)))
    (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')
    pattern = re.compile(r'^(SUCCESS|FAILED) (\S+) nodes=(\d+) cost=(\S+) seconds=(\S+) length=(\S+) search_seconds=(\S+)$', re.M)
    fields = ['scenario', 'seed', 'repeat', 'status', 'reason', 'exit_code', 'nodes', 'cost',
              'planning_seconds', 'planning_ms', 'search_seconds', 'trajectory_length_m',
              'process_seconds', 'trajectory_file']
    rows = []
    for repeat in range(1, args.repeats+1):
        for scene in scenarios:
            for seed in args.seeds:
                suffix = f'_r{repeat:02d}' if args.repeats > 1 else ''
                run_name = f'{scene.stem}_{seed}{suffix}'
                result = args.output / f'{run_name}.yaml'
                begin = time.monotonic()
                row = dict(scenario=scene.stem, seed=seed, repeat=repeat, status='ERROR', reason='', exit_code='',
                           nodes='', cost='', planning_seconds='', planning_ms='', search_seconds='',
                           trajectory_length_m='', trajectory_file=str(result))
                try:
                    run = subprocess.run([str(executable), str(scene), str(result.resolve()), str(seed)],
                                         capture_output=True, text=True, timeout=args.process_timeout,
                                         creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
                    log = run.stdout + run.stderr
                    row['exit_code'] = run.returncode
                    match = pattern.search(run.stdout)
                    if match:
                        status, reason, nodes, cost, seconds, length, search = match.groups()
                        row.update(status=status, reason=reason, nodes=int(nodes),
                                   planning_seconds=float(seconds), planning_ms=1000*float(seconds),
                                   search_seconds=float(search))
                        if status == 'SUCCESS' and run.returncode == 0:
                            row['cost'] = float(cost)
                            row['trajectory_length_m'] = float(length)
                        elif status == 'SUCCESS':
                            row['status'] = 'ERROR'
                    else:
                        row['reason'] = 'cli_error'
                except subprocess.TimeoutExpired as exc:
                    row.update(status='PROCESS_TIMEOUT', reason='process_limit')
                    def decode(value):
                        return value.decode(errors='replace') if isinstance(value, bytes) else value or ''
                    log = decode(exc.stdout) + decode(exc.stderr)
                row['process_seconds'] = time.monotonic() - begin
                (args.output / f'{run_name}.log').write_text(log, encoding='utf-8')
                rows.append(row)
                print(f'round {repeat}/{args.repeats}', scene.stem, seed, row['status'], row['reason'], flush=True)
                # Persist each completed run, including errors and timeouts.
                with (args.output / 'runs.csv').open('w', newline='', encoding='utf-8') as out:
                    writer = csv.DictWriter(out, fieldnames=fields)
                    writer.writeheader(); writer.writerows(rows)
    report = ['# FLASK-RRTConnect: six DynoBench scenarios', '',
              'Local measurements; planning time includes shortcut, excludes I/O and length quadrature.', '',
              '| Scenario | Success | Traj. Len. (m), mean | Plan. Time (ms), mean | Time SD (ms) |',
              '| --- | --- | --- | --- | --- |']
    for scene in scenarios:
        group = [row for row in rows if row['scenario'] == scene.stem]
        successes = [row for row in group if row['status'] == 'SUCCESS' and row['exit_code'] == 0]
        times = [row['planning_seconds'] for row in successes]
        mean = f'{1000*statistics.mean(times):.6g}' if times else 'NA'
        std = f'{1000*statistics.stdev(times):.6g}' if len(times) > 1 else 'NA'
        length = f'{statistics.mean(row["trajectory_length_m"] for row in successes):.6g}' if times else 'NA'
        report.append(f'| {scene.stem} | {len(successes)}/{len(group)} | {length} | {mean} | {std} |')
    (args.output / 'summary.md').write_text('\n'.join(report)+'\n', encoding='utf-8')
    return 0 if all(row['status'] == 'SUCCESS' for row in rows) else 2


if __name__ == '__main__':
    raise SystemExit(main())
