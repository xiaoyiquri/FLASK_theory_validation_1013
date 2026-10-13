"""Summarize completed benchmark runs; failed paths never become zero-cost samples."""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics


def summary(values):
    if not values:
        return {'mean': None, 'sample_sd': None}
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Nonfinite metric')
    return {'mean': statistics.mean(values),
            'sample_sd': statistics.stdev(values) if len(values) > 1 else None}


def display(metric):
    if metric['mean'] is None:
        return 'NA'
    sd = f'{metric["sample_sd"]:.3f}' if metric['sample_sd'] is not None else 'NA'
    return f'{metric["mean"]:.3f} +/- {sd}'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('results', type=Path)
    args = parser.parse_args()
    manifest = json.loads((args.results/'manifest.json').read_text(encoding='utf-8'))
    with (args.results/'runs.csv').open(encoding='utf-8', newline='') as source:
        rows = list(csv.DictReader(source))
    repeats = manifest.get('repeats', 1)
    expected_per_scene = len(manifest['seeds'])*repeats
    expected = len(manifest['scenarios'])*expected_per_scene
    if len(rows) != expected:
        raise ValueError(f'Incomplete run: {len(rows)}/{expected} records')
    identifiers = {(r['scenario'], int(r['seed']), int(r.get('repeat', 1))) for r in rows}
    if len(identifiers) != len(rows):
        raise ValueError('Duplicate scenario/seed/repeat record')
    report = ['# FLASK-RRTConnect repeated-run statistics', '',
              f'Seeds: {manifest["seeds"]}; repeats per seed/environment: {repeats}.',
              'Each repetition launches a fresh process; there are no warm-up exclusions.',
              'Metrics below are mean +/- sample SD over successful runs only.',
              'First-solution time excludes shortcut; total planning time includes shortcut.',
              'Cost and length belong to the final path after shortcut. I/O is excluded.', '',
              '| Environment | Success | First solution (ms) | Total planning (ms) | Cost | Length (m) |',
              '| --- | --- | --- | --- | --- | --- |']
    groups = []
    for source in manifest['scenarios']:
        scene = Path(source).stem
        records = [r for r in rows if r['scenario'] == scene]
        if len(records) != expected_per_scene:
            raise ValueError(f'Incomplete scenario {scene}')
        successes = [r for r in records if r['status'] == 'SUCCESS' and int(r['exit_code']) == 0]
        failures = [r for r in records if r not in successes]
        group = {'scenario': scene, 'runs': len(records), 'successes': len(successes),
                 'first_solution_ms': summary([1000*float(r['search_seconds']) for r in successes]),
                 'total_planning_ms': summary([float(r['planning_ms']) for r in successes]),
                 'cost': summary([float(r['cost']) for r in successes]),
                 'length_m': summary([float(r['trajectory_length_m']) for r in successes]),
                 'all_run_planning_ms': summary([float(r['planning_ms']) for r in records if r['planning_ms']]),
                 'failed_planning_ms': summary([float(r['planning_ms']) for r in failures if r['planning_ms']]),
                 'failure_reasons': {reason: sum(r['reason'] == reason for r in failures)
                                     for reason in dict.fromkeys(r['reason'] for r in failures)}}
        groups.append(group)
        report.append('| '+scene.removeprefix('dynobench_')+' | '+f'{len(successes)}/{len(records)}'
                      +' | '+' | '.join(display(group[key]) for key in
                                       ['first_solution_ms', 'total_planning_ms', 'cost', 'length_m'])+' |')
    report += ['', 'Failure latency is reported separately; a timeout is not a measured first solution.']
    for group in groups:
        if group['failure_reasons']:
            report.append(f'- {group["scenario"]}: {group["failure_reasons"]}; '
                          f'failed planning latency (ms): {display(group["failed_planning_ms"])}.')
    report += ['', 'With one fixed seed, repetitions measure timing variability for the same sampling sequence.',
               'They do not measure success probability over independently sampled seeds.',
               'These are local DynoBench-data runs; reduced-r2 quad paths have no official dynamics certification.']
    (args.results/'statistics.json').write_text(json.dumps(groups, indent=2)+'\n', encoding='utf-8')
    (args.results/'statistics.md').write_text('\n'.join(report)+'\n', encoding='utf-8')
    print('\n'.join(report))


if __name__ == '__main__':
    main()
