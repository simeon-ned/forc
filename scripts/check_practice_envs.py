"""Run each practice in a fresh environment containing only its declared profile."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
PRACTICES = {entry.parent.name.split("-")[0]: entry.parent
             for entry in sorted((ROOT / "practices").glob("p*/practice.yml"))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=sorted(PRACTICES), action="append")
    args = parser.parse_args()
    uv = shutil.which("uv")
    if uv is None:
        parser.error("Install uv before running environment checks.")
    for profile in args.profile or PRACTICES:
        with tempfile.TemporaryDirectory(prefix=f"forc-{profile}-") as directory:
            directory = Path(directory)
            env = os.environ.copy()
            env.pop("VIRTUAL_ENV", None)
            env.pop("CONDA_PREFIX", None)
            env["UV_PROJECT_ENVIRONMENT"] = str(directory / "venv")
            python = directory / "venv" / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
            entry = PRACTICES[profile] / ("check.py" if profile == "p01" else "run.py")
            reference = ([str(python), str(entry), "--output", str(directory / "output")]
                         if entry.exists() else
                         [str(python), str(ROOT / "scripts/run_practices.py"), "--practice", profile])
            commands = [
                [uv, "sync", "--locked", "--no-default-groups", "--extra", profile],
                [uv, "pip", "check", "--python", str(python)],
                reference,
            ]
            # Guard the import boundary: simple learning labs do not need a physics engine.
            absent = []
            if profile in {"p05", "p06", "p07"}:
                absent += ["mujoco"]
            if profile in {"p06", "p07"}:
                absent += ["scipy", "jupyterlab"]
            if profile == "p02":
                probes_required = "import scipy"
            else:
                probes_required = ""
            if profile == "p03":
                absent += ["scipy"]
            probes = ["import importlib.util"]
            if probes_required:
                probes.append(probes_required)
            probes.extend(f"assert importlib.util.find_spec({name!r}) is None, {name!r}" for name in absent)
            if profile in {"p01", "p02", "p03"}:
                probes.append("import mujoco, mujoco_menagerie, mediapy, matplotlib, ipykernel, jupyterlab")
            probe = "; ".join(probes)
            commands.append([str(python), "-c", probe])
            for command in commands:
                result = subprocess.run(command, cwd=ROOT, env=env, text=True,
                                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
                if result.returncode:
                    print(result.stdout)
                    raise SystemExit(result.returncode)
            print(f"PASS {profile}: clean install, dependency check, reference experiment", flush=True)


if __name__ == "__main__":
    main()
