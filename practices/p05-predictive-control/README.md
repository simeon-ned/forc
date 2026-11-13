## Aim

Compare explicit constraints and sampling on a shared vertical-flight model.

**Time:** about 120 minutes. Start with the linked lectures and the course notation contract. The reference program is complete and runnable; your work is to explain it, modify it, and test the results.

## Run from this repository

Run from the repository root. See the [shared setup guide](../README.md) for uv, conda, rendering, and installing all practices.

~~~bash
uv sync --locked --extra p05
uv run --locked --extra p05 python practices/p05-predictive-control/run.py
~~~

With the course conda environment activated:

~~~bash
python scripts/setup_conda.py p05
python practices/p05-predictive-control/run.py
~~~

The program writes configuration, available MuJoCo version (null when not installed), and metrics to outputs/p05-predictive-control/metrics.json. Experiments with a time history also write trajectory.csv. These are generated results and are not source material.

Use a separate configuration and output directory for each variation:

~~~bash
uv run --locked --extra p05 python practices/p05-predictive-control/run.py --config practices/p05-predictive-control/config.yml --output outputs/p05-predictive-control/baseline
~~~

Keep the original configuration as the baseline. Record all changes in your report; avoid changing several physical and numerical parameters at once.

## Tasks

1. Run clipped LQR, constrained MPC, and the MPPI-inspired weighted-shooting baseline.
2. Compare RMS height error, state-limit violations, solve failures, and compute time.
3. Sweep horizon, temperature, and sample count using separate YAML files.
4. Initialize a case outside the recoverable envelope and explain feasibility rather than hiding the failed solve.

## Expected checks

The model is a one-axis double integrator around hover, not a complete quadrotor. MPC checks predicted height constraints. Weighted shooting penalizes violations but does not guarantee feasibility. Timing values depend on the machine.

## Submission

Submit your YAML configuration, a short explanation of the equations and assumptions, labeled plots or numerical comparisons with units, and a paragraph explaining a failure or limitation. Include the exact run command. The weekly practice rubric awards 40% for correctness and checks, 30% for the controlled comparison, 20% for interpretation, and 10% for reproducibility.

## Extension

Use Practice 4's full vehicle in TH1. A general-purpose SLSQP solve teaches formulation; a dedicated QP solver is a subsequent engineering improvement.

## Reading

[@mit-trajopt] · [@cmu-index] · [@mujoco-mjx]
