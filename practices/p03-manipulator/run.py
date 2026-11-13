"""Run this practice from the repository root."""
from pathlib import Path
from forc.labs import manipulator
from forc.numerics import run_cli

if __name__ == "__main__":
    run_cli(manipulator, Path(__file__).resolve().parent)
