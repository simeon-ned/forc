"""Run every current practice in this repository and stop on failures."""
from pathlib import Path
import subprocess
import sys

root=Path(__file__).resolve().parents[1]
for entry in sorted((root/"practices").glob("p*/run.py")):
    print(f"\nRunning {entry.parent.name}",flush=True)
    subprocess.run([sys.executable,str(entry)],cwd=root,check=True)
