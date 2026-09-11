## Aim

Connect task-space kinematics to joint-space feedback and inverse dynamics.

**Time:** about 120 minutes. Start with the linked lectures and the course notation contract. The reference program is complete and runnable; your work is to explain it, modify it, and test the results.

## Run from this repository

~~~bash
uv sync --locked
uv run --locked python practices/p02-manipulator/run.py
~~~

The program writes configuration, MuJoCo version, and metrics to outputs/p02-manipulator/metrics.json. Experiments with a time history also write trajectory.csv. These are generated results and are not source material.

Use a separate configuration and output directory for each variation:

~~~bash
uv run --locked python practices/p02-manipulator/run.py --config practices/p02-manipulator/config.yml --output outputs/p02-manipulator/baseline
~~~

Keep the original configuration as the baseline. Record all changes in your report; avoid changing several physical and numerical parameters at once.

## Tasks

1. Inspect the named tool site and verify the Jacobian by finite differences.
2. Run damped differential IK from the supplied nonsingular pose and measure final tool error.
3. Compare PD, bias-compensated PD, and computed torque under equal force limits.
4. Reduce torque_limit to 5 and sweep the damping parameter. Explain saturation and singularity effects separately.

## Expected checks

The unsaturated computed-torque controller should reduce the joint error substantially. PD can retain an offset under gravity. Damping regularizes IK but does not remove kinematic singularities.

## Submission

Submit your YAML configuration, a short explanation of the equations and assumptions, labeled plots or numerical comparisons with units, and a paragraph explaining a failure or limitation. Include the exact run command. The weekly practice rubric awards 40% for correctness and checks, 30% for the controlled comparison, 20% for interpretation, and 10% for reproducibility.

## Extension

Change one link mass by 30% while holding the controller's nominal model fixed. Distinguish model mismatch from retuning the same model.

## Reading

[@mit-pick] · [@mit-multibody] · [@mujoco-api]
