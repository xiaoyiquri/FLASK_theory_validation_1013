"""Plot generated six-scene paths using matplotlib; not a collision validator."""
import argparse
import ast
import csv
import json
from pathlib import Path
import re


def boxes(file):
    # Only the frozen axis-aligned boxes used by these six environments.
    center = None
    for raw in file.read_text(encoding='utf-8').splitlines():
        line = raw.split('#', 1)[0].strip()
        if line.startswith('center:'):
            center = ast.literal_eval(line.split(':', 1)[1])
        if line.startswith('size:') and center is not None:
            yield center, ast.literal_eval(line.split(':', 1)[1])
            center = None


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('results', type=Path)
    args = p.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    root = Path(__file__).resolve().parents[1]
    rows = list(csv.DictReader((args.results/'runs.csv').open(encoding='utf-8')))
    rows = [row for row in rows if row['seed'] == rows[0]['seed']]
    fig = plt.figure(figsize=(15, 9), layout='constrained')
    for index, row in enumerate(rows):
        config = json.loads((root/'examples'/f'{row["scenario"]}.yaml').read_text(encoding='utf-8'))
        dim = 3 if config['robot'] == 'quad3d' else 2
        ax = fig.add_subplot(2, 3, index+1, projection='3d' if dim == 3 else None)
        for center, size in boxes(root/'examples'/config['dynobench']['environment']):
            lo = [c-s/2 for c, s in zip(center, size)]
            if dim == 2:
                ax.add_patch(Rectangle(lo, *size, color='gray', alpha=.65))
            else:
                hi = [c+s/2 for c, s in zip(center, size)]
                vertices = [[x,y,z] for x in [lo[0],hi[0]] for y in [lo[1],hi[1]] for z in [lo[2],hi[2]]]
                faces = [[vertices[j] for j in face] for face in [(0,1,3,2),(4,5,7,6),(0,1,5,4),(2,3,7,6),(0,2,6,4),(1,3,7,5)]]
                ax.add_collection3d(Poly3DCollection(faces, facecolor='gray', alpha=.25))
        if row['status'] == 'SUCCESS':
            trajectory = Path(row['trajectory_file'])
            if not trajectory.exists():
                trajectory = args.results/trajectory.name
            data = trajectory.read_text(encoding='utf-8')
            section = data.split('\nsamples:\n', 1)[1].split('\nvalidation:', 1)[0]
            states = [json.loads(value) for value in re.findall(r'state: (\[[^\n]+\])', section)]
            coordinates = [[state[k] for state in states] for k in range(dim)]
            ax.plot(*coordinates, color='#176eab', linewidth=2)
            ax.scatter(*[state[0] for state in coordinates], color='green', s=40)
            ax.scatter(*[state[-1] for state in coordinates], color='red', s=40)
            title = f'{float(row["trajectory_length_m"]):.2f} m / {float(row["planning_ms"]):.2f} ms'
        else:
            title = f'{row["status"]}: {row["reason"]}'
        ax.set_title(row['scenario'].removeprefix('dynobench_')+'\n'+title)
        ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
        if dim == 3:
            ax.set_zlabel('z (m)'); ax.set_box_aspect((1,1,.6))
        else:
            ax.set_aspect('equal', adjustable='datalim'); ax.autoscale_view()
        ax.grid(alpha=.2)
    fig.suptitle('DF + RRTConnect | local measurements | r=2 | quad physical joins: diagnostic only')
    fig.savefig(args.results/'paths.png', dpi=180)


if __name__ == '__main__':
    main()
