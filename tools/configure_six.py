"""Regenerate the explicit local FLASK-RRTConnect profiles, not author defaults."""
import json
from pathlib import Path


def main():
    examples = Path(__file__).resolve().parents[1] / 'examples'
    for file in sorted(examples.glob('dynobench_*.yaml')):
        config = json.loads(file.read_text(encoding='utf-8'))
        config['r'] = 2
        config['rho'] = 1
        n = config['n']
        for side in ('lower', 'upper'):
            config['bounds'][side] = config['bounds'][side][:2*n]
        quad = config['robot'].startswith('quad')
        if quad:
            config.pop('lifting', None)
        else:
            config['planner']['unicycle_sample_speed'] = .5
            # Explicit local Eq.20 weight: unit weighting often violates
            # omega limits in the tight Bugtrap's minimum-cost steering.
            config['weight'] = [[10, 0], [0, 10]]
        config['planner'].update(type='rrtconnect', iterations=1000000, max_nodes=20000,
                                 timeout=10, range=.25, shortcut=True,
                                 shortcut_attempts=100)
        config['validation'].update(step=.01, continuous_sphere_collision=True,
                                   enforce_physical_continuity=not quad,
                                   enforce_physical_endpoints=not quad)
        file.write_text(json.dumps(config, indent=2)+'\n', encoding='utf-8')


if __name__ == '__main__':
    main()
