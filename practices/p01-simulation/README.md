## Aim

Learn robot models and MuJoCo basics using Menagerie’s UR5e, Unitree G1 humanoid, and Unitree Go2 quadruped: state, kinematics, dynamics, energy, floating bases, contacts, and simulation.

**Time:** about 120 minutes for the core path. Use the detailed kinematics and energy checks as guided extensions. Feedback control and linearization continue in [practice 02](../p02-manipulator/control_and_linearization.ipynb).

## Setup and tutorial

Run from the repository root. The [shared setup guide](../README.md) covers uv, conda, graphics, and offline caching.

```bash
uv sync --locked --extra p01
uv run --locked --extra p01 jupyter lab practices/p01-simulation/tutorial.ipynb
```

With the course conda environment activated:

```bash
python scripts/setup_conda.py p01
python -m jupyterlab practices/p01-simulation/tutorial.ipynb
```

[Open the models and MuJoCo basics tutorial](tutorial.ipynb). Each robot is rendered immediately after loading. Structured explanations and display equations precede the calculations.

The notebook covers:

- UR5e model/state arrays, named bodies and joints, and configuration conventions.
- Body and COM kinematics, local/world frames, Jacobians, velocities, and accelerations.
- Inertia, gravity, bias/passive forces, forward/inverse dynamics, energy, and external wrenches.
- Loading and rendering G1 and Go2 before examining floating-base coordinates and contacts.
- Real contact geom names, contact dimensions, friction cones, and generalized force balance.
- Passive Go2 motion, timestep selection, and rendering without a balance controller.
- Editing a Menagerie specification with `MjSpec`, with links to the model-editing documentation.

All robots download on demand; no robot meshes or simplified stand-in models are stored here. Prepare all three models for offline use:

```bash
uv run --locked --extra p01 mujoco-menagerie prefetch universal_robots_ur5e unitree_g1 unitree_go2
```

In conda, use the same `mujoco-menagerie prefetch ...` command without the `uv run` prefix.

## Interactive robot explorer

Run the small [viewer script](run.py) on a machine with a desktop display:

```bash
uv run --locked --extra p01 python practices/p01-simulation/run.py
```

For conda, use `python practices/p01-simulation/run.py`. On macOS, use `mjpython` in place of `python` for the passive viewer.

At the top of `run.py`, leave exactly one of the two `mm.load(...)` lines uncommented to choose G1 or Go2. The script prints position/velocity/input dimensions, body/joint/geom counts, and the physics timestep, then simulates continuously with a **2 ms timestep (500 Hz)**. It calls `viewer.sync()` every **10 physics steps (50 Hz)** to publish state and receive mouse forces. The native viewer manages screen redraws independently; these rates describe physics and state updates. Wall-clock pacing is best effort and slows down if the machine cannot keep up.

- Double-click a robot body to select it.
- Hold **Ctrl + right mouse button** and drag to apply a force.
- Hold **Ctrl + left mouse button** and drag to apply torque.
- Press **Ctrl+C in the terminal**, or close the viewer, to exit.

Both robots use **torque control**: `data.ctrl` contains joint torques in N m. The script removes G1's position-servo feedback and replaces its angle control limits with the model's joint effort limits; Go2's existing motor torque limits are retained. It loads the initial keyframe, then clears its control values to zero. Mouse forces persist between viewer updates. A comment immediately before `mj_step` marks where we will add a 500 Hz torque controller later. With zero commanded torque and no balance controller, a floating-base robot can fall.

This window requires a desktop OpenGL context. Offscreen `egl`/`osmesa` notebook rendering does not provide a desktop window; use the notebook or `check.py` in a headless environment. The default devcontainer is set up for offscreen rendering.

## Headless reference experiment

The reference also uses these Menagerie robots. It checks the UR5e force balance, runs passive arm dynamics with actuation disabled, and reports dimensions for UR5e, G1, and Go2.

```bash
uv run --locked --extra p01 python practices/p01-simulation/check.py
uv run --locked --extra p01 python practices/p01-simulation/check.py --config practices/p01-simulation/config.yml --output outputs/p01-simulation/baseline
```

For conda, run `python practices/p01-simulation/check.py` after setup. Results are written to `outputs/p01-simulation/metrics.json` and `trajectory.csv`. Keep a separate configuration and output directory for each variation.

## Tasks

1. Inspect the UR5e and compare its configuration, velocity, and input dimensions with G1 and Go2.
2. Explain the free-joint position/velocity mismatch and verify a quaternion-aware configuration round trip.
3. Reconstruct inertia and verify the generalized force balance; distinguish bias, passive, applied, and constraint forces.
4. Inspect actual G1 and Go2 contacts, including geom names, contact axes, friction coefficients, and contact dimensions.
5. Change the passive simulation timestep while keeping physical duration fixed. Compare the final configuration and contact forces.

## Expected checks

For the pinned models, UR5e has `nq=nv=nu=6`; G1 has `nq=36`, `nv=35`, `nu=29`; Go2 has `nq=19`, `nv=18`, `nu=12`. The inertia matrix is symmetric positive definite, force residuals are small, and state samples remain finite. A nominal keyframe is not a guarantee of static balance; disabling actuation can make a legged robot collapse.

## Submission

Submit the experiment configuration, explanations of the equations and coordinate conventions, labeled results with units, and one observed limitation. Include the exact run command and model/package versions. The weekly rubric awards 40% for correctness and checks, 30% for controlled comparison, 20% for interpretation, and 10% for reproducibility.

## Extension

Change one model parameter through `MjSpec`, recompile it, and compare the result with the baseline. Try damping or contact friction before changing model structure.

## Reading

[@mujoco-api] · [@mujoco-computation]
