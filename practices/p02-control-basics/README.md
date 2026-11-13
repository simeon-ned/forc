## From a model to a controller

Build intuition for feedback before working with a manipulator: predict a response, simulate the same equations in MuJoCo, then explain the difference. This practice follows [simulation](../../practices/p01-simulation/README.md) and prepares you for [manipulator control](../../practices/p03-manipulator/README.md).

**Before class:** read [lecture 02: Feedback](../../modules/02-feedback/notes.md). **Time:** about 180 minutes; sections 1–5 form the core session, and sections 6–7 can be a guided follow-up.

By the end you should be able to:

- Write a state-space model and map its state and input to MuJoCo arrays.
- Design a PD response using damping ratio and natural frequency.
- Check stability and controllability before placing closed-loop poles.
- Verify a linearization and explain where a nonlinear simulation disagrees.

The web lesson presents the theory, engineering illustrations, and complete Python cells. Run the [self-contained tutorial notebook](tutorial.ipynb) to compute plots and play a MuJoCo video immediately below **every experiment**. All model, simulation, and control functions are defined in the notebook near their first use. The diagrams are embedded, so the downloaded notebook works outside the repository too. No robot assets need to be downloaded.

## Run the tutorial

From the repository root, choose the new `p02` profile. The [shared setup guide](../README.md) covers conda, graphics drivers, and installing all practices.

~~~bash
uv sync --locked --extra p02
uv run --locked --extra p02 jupyter lab practices/p02-control-basics/tutorial.ipynb
~~~

With the course conda environment activated:

~~~bash
python scripts/setup_conda.py p02
python -m jupyterlab practices/p02-control-basics/tutorial.ipynb
~~~

The notebook is the lesson source: edit its cells directly. The website displays the same explanations, code, and attached diagrams. Plots and videos are generated inline when you run it; keep saved outputs empty in the course repository.

Leave `VISUALIZE = True` for the full practice. For numeric-only checks without OpenGL, set it to `False`, or run the shared checker:

~~~bash
uv run --locked --extra p02 python scripts/run_practices.py --practice p02
~~~

Rendering needs OpenGL and FFmpeg. On headless Linux, set `MUJOCO_GL=osmesa` (Mesa) or `MUJOCO_GL=egl` (EGL driver) **before starting Python or the notebook kernel**. The numeric checker runs the notebook's own calculations with visualization disabled.

## Lesson map

1. [State, input, and the simulation loop](#1-state-input-and-the-simulation-loop)
2. [Mass–spring–damper: a linear plant](#2-massspringdamper-a-linear-plant)
3. [Motor control and second-order response](#3-motor-control-and-second-order-response)
4. [Stability, controllability, and pole placement](#4-stability-controllability-and-pole-placement)
5. [A flexible drive with two inertias](#5-a-flexible-drive-with-two-inertias)
6. [Nonlinear equilibria and local models](#6-nonlinear-equilibria-and-local-models)
7. [Cart-pole stabilization](#7-cart-pole-stabilization)
8. [Tasks, checks, and documentation](#8-tasks-checks-and-documentation)
