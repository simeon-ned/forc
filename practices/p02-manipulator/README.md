## Aim

Connect actuator models and task-space kinematics to joint-space feedback, inverse dynamics, and local linearization.

**Time:** about 120 minutes. Start with the linked lectures and the course notation contract. The reference program is complete and runnable; your work is to explain it, modify it, and test the results.

## Run from this repository

Run from the repository root. See the [shared setup guide](../README.md) for uv, conda, rendering, and installing all practices.

~~~bash
uv sync --locked --extra p02
uv run --locked --extra p02 python practices/p02-manipulator/run.py
~~~

With the course conda environment activated:

~~~bash
python scripts/setup_conda.py p02
python practices/p02-manipulator/run.py
~~~

The program writes configuration, MuJoCo version, and metrics to outputs/p02-manipulator/metrics.json. Experiments with a time history also write trajectory.csv. These are generated results and are not source material.

Use a separate configuration and output directory for each variation:

~~~bash
uv run --locked --extra p02 python practices/p02-manipulator/run.py --config practices/p02-manipulator/config.yml --output outputs/p02-manipulator/baseline
~~~

Keep the original configuration as the baseline. Record all changes in your report; avoid changing several physical and numerical parameters at once.

## Control and linearization notebook

[Open the standalone UR5e notebook](control_and_linearization.ipynb). It loads its own Menagerie robot and can run in a fresh kernel without executing p01 first.

```bash
uv run --locked --extra p02 jupyter lab practices/p02-manipulator/control_and_linearization.ipynb
```

For conda, after `python scripts/setup_conda.py p02`:

```bash
python -m jupyterlab practices/p02-manipulator/control_and_linearization.ipynb
```

The notebook covers actuator force laws, direct-joint actuation maps, internal position servos, external PD torque control, gravity sensitivities, and discrete-time UR5e linearization. It checks the local model against an actual perturbed simulation step. All robot data comes from Menagerie; the notebook contains no simplified pendulum model.

The `p02` install profile includes Jupyter, plotting/video tools, and Menagerie. The existing `run.py` reference complements the notebook with its controller/IK comparisons.

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

Use the notebook to vary gains, actuator limits, derivative perturbation size, and timestep separately. Recompute the linearization after changing the nominal model or timestep.

Change one link mass by 30% while holding the controller's nominal model fixed. Distinguish model mismatch from retuning the same model.

## Reading

[@mit-pick] · [@mit-multibody] · [@mujoco-api]
