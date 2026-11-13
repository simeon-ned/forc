"""Run this practice from the repository root."""
from pathlib import Path
from forc.labs import reinforcement
from forc.numerics import run_cli

if __name__ == "__main__":
    run_cli(reinforcement, Path(__file__).resolve().parent)
