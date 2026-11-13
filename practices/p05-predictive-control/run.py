"""Run this practice from the repository root."""
from pathlib import Path
from forc.labs import predictive
from forc.numerics import run_cli

if __name__ == "__main__":
    run_cli(predictive, Path(__file__).resolve().parent)
