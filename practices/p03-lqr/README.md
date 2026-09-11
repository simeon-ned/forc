## Aim

Build the same local stabilization pipeline for a hinge model and a quaternion model.

**Time:** about 120 minutes. Start with the linked lectures and the course notation contract. The reference program is complete and runnable; your work is to explain it, modify it, and test the results.

## Run from this repository

~~~bash
uv sync --locked
uv run --locked python practices/p03-lqr/run.py
~~~

The program writes configuration, MuJoCo version, and metrics to outputs/p03-lqr/metrics.json. Experiments with a time history also write trajectory.csv. These are generated results and are not source material.

Use a separate configuration and output directory for each variation:

~~~bash
uv run --locked python practices/p03-lqr/run.py --config practices/p03-lqr/config.yml --output outputs/p03-lqr/baseline
~~~

Keep the original configuration as the baseline. Record all changes in your report; avoid changing several physical and numerical parameters at once.

## Tasks

1. Verify the nominal cart-pole and quadrotor states are equilibria before differentiating.
2. Inspect A and B dimensions and compute the closed-loop eigenvalues.
3. Run small-error nonlinear rollouts with actuator clipping and report initial/final tangent error.
4. Increase the initial pole angle or vehicle tilt. Identify where the local controller fails.

## Expected checks

The full vehicle uses a 12-dimensional tangent error, 13 stored qpos-plus-qvel entries, and four thrust inputs. The closed-loop spectral radius should be below one near the tested equilibrium.

## Submission

Submit your YAML configuration, a short explanation of the equations and assumptions, labeled plots or numerical comparisons with units, and a paragraph explaining a failure or limitation. Include the exact run command. The weekly practice rubric awards 40% for correctness and checks, 30% for the controlled comparison, 20% for interpretation, and 10% for reproducibility.

## Extension

Implement finite-horizon time-varying LQR around a feasible reference. Compare with the fixed hover gain using paired disturbances.

## Reading

[@mit-lqr] · [@cmu-index] · [@mujoco-api]
