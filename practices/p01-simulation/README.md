## Aim

Inspect the state and verify a force balance before designing a controller.

**Time:** about 120 minutes. Start with the linked lectures and the course notation contract. The reference program is complete and runnable; your work is to explain it, modify it, and test the results.

## Run from this repository

~~~bash
uv sync --locked
uv run --locked python practices/p01-simulation/run.py
~~~

The program writes configuration, MuJoCo version, and metrics to outputs/p01-simulation/metrics.json. Experiments with a time history also write trajectory.csv. These are generated results and are not source material.

Use a separate configuration and output directory for each variation:

~~~bash
uv run --locked python practices/p01-simulation/run.py --config practices/p01-simulation/config.yml --output outputs/p01-simulation/baseline
~~~

Keep the original configuration as the baseline. Record all changes in your report; avoid changing several physical and numerical parameters at once.

## Tasks

1. Run the reference experiment and identify nq, nv, and nu for the arm and free-joint vehicle.
2. Reconstruct the dense mass matrix, inspect its eigenvalues, and explain every force in the residual.
3. Change timestep while keeping final physical time fixed. Record the resulting state difference.
4. Add a constant command and then a passive damping change as separate experiments.

## Expected checks

The arm has nq=nv=2. The free-joint vehicle has nq=7 and nv=6. The force residual should be close to floating-point precision. All state samples should remain finite.

## Submission

Submit your YAML configuration, a short explanation of the equations and assumptions, labeled plots or numerical comparisons with units, and a paragraph explaining a failure or limitation. Include the exact run command. The weekly practice rubric awards 40% for correctness and checks, 30% for the controlled comparison, 20% for interpretation, and 10% for reproducibility.

## Extension

Add a controller that runs every five physics steps. Compare it with one running every step, and explain why they are different sampled systems.

## Reading

[@mujoco-api] · [@mujoco-computation]
