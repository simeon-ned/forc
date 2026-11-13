"""Run finite headless practice checks; never open an interactive viewer."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def run_notebook(path):
    """Run the actual tutorial cells, disabling only optional visualization."""
    import matplotlib
    matplotlib.use('Agg')
    notebook = json.loads(Path(path).read_text())
    namespace = {'__name__': '__main__'}
    for index, cell in enumerate(notebook['cells']):
        if cell['cell_type'] != 'code':
            continue
        source = ''.join(cell['source'])
        exec(compile(source, f'{path}:cell-{index}', 'exec'), namespace)
        if 'parameters' in cell.get('metadata', {}).get('tags', []):
            namespace['VISUALIZE'] = False
    return namespace


def main():
    import yaml
    practices = sorted((ROOT / 'practices').glob('p*/practice.yml'))
    profiles = [p.parent.name.split('-')[0] for p in practices]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--practice', choices=profiles, action='append')
    args = parser.parse_args()
    for metadata in practices:
        if args.practice and metadata.parent.name.split('-')[0] not in args.practice:
            continue
        spec = yaml.safe_load(metadata.read_text())
        entry = metadata.parent / spec['entrypoint']
        if metadata.parent.name == 'p01-simulation':
            entry = entry.with_name('check.py')
        print(f'\nRunning {metadata.parent.name}', flush=True)
        if entry.suffix == '.ipynb':
            run_notebook(entry)
        else:
            subprocess.run([sys.executable, str(entry)], cwd=ROOT, check=True)


if __name__ == '__main__':
    main()
