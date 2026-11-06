"""Install a practice profile from uv.lock into the active course conda environment."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import tomllib

ROOT = Path(__file__).resolve().parents[1]


def main():
    metadata = tomllib.loads((ROOT / "pyproject.toml").read_text())
    profiles = metadata["project"]["optional-dependencies"]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("profile", nargs="?", default="all", choices=sorted(profiles))
    parser.add_argument("--dev", action="store_true", help="Also install test tools")
    args = parser.parse_args()
    prefix = os.environ.get("CONDA_PREFIX")
    if not prefix or Path(prefix).resolve() != Path(sys.prefix).resolve():
        parser.error("Activate the course conda environment and run with its Python first.")
    if not (Path(prefix) / "conda-meta").is_dir():
        parser.error("The selected Python is not in a conda environment.")
    if os.environ.get("CONDA_DEFAULT_ENV") == "base" or (Path(prefix) / "condabin").is_dir():
        parser.error("Create and activate a dedicated course environment, not conda base.")
    uv = shutil.which("uv")
    if uv is None:
        parser.error("uv is missing; create the environment using environment.yml.")
    # Install rather than sync: conda's own packages must never be pruned by uv.
    with tempfile.TemporaryDirectory(prefix="forc-requirements-") as directory:
        requirements = Path(directory) / "requirements.txt"
        command = [uv, "export", "--locked", "--no-default-groups", "--extra", args.profile,
                   "--no-hashes", "--no-annotate", "--no-header", "--output-file", str(requirements)]
        if args.dev:
            command.extend(["--group", "dev"])
        subprocess.run(command, cwd=ROOT, check=True, stdout=subprocess.DEVNULL)
        subprocess.run([uv, "pip", "install", "--python", sys.executable,
                        "--no-deps", "--requirements", str(requirements)], cwd=ROOT, check=True)
    subprocess.run([uv, "pip", "check", "--python", sys.executable], cwd=ROOT, check=True)
    print(f"Installed {args.profile} into {prefix}. Run practices with python, not uv run.")


if __name__ == "__main__":
    main()
