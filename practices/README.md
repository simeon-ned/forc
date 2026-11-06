# Practice setup

Run commands from the **repository root**. Use Python 3.12, as recorded in `.python-version`. Choose uv or conda; both install the course packages from the committed `uv.lock`.

## Quick start with uv

Install [uv](https://docs.astral.sh/uv/getting-started/installation/), then install the whole current course:

```bash
uv sync --locked --extra all
uv run --locked --extra all python scripts/run_practices.py
uv run --locked --extra all jupyter lab practices/p01-simulation/tutorial.ipynb
```

uv creates `.venv` and obtains Python 3.12 if necessary. No environment activation is needed. In VS Code, select `.venv` as the Python interpreter and notebook kernel.

For just one practice, choose its short ID (`p01` through `p07`):

```bash
uv sync --locked --extra p04
uv run --locked --extra p04 python practices/p04-predictive-control/run.py
```

Keep the same `--extra` on `uv run`: uv synchronizes the environment when running commands. Switching profiles removes packages not required by the new selection. To keep several practices installed, combine flags, for example `--extra p01 --extra p04`, or use `--extra all` throughout.

The profiles follow the code's actual dependencies:

- `p01` — [models and simulation](p01-simulation/README.md): MuJoCo, Jupyter, plotting/video tools, and Menagerie (UR5e, G1, Go2).
- `p02` — [manipulator control](p02-manipulator/README.md): MuJoCo, Jupyter, plotting/video tools, and Menagerie for the independent UR5e control/linearization notebook.
- `p03` — [LQR](p03-lqr/README.md): MuJoCo and SciPy.
- `p04` — [predictive control](p04-predictive-control/README.md): SciPy; no physics engine required.
- `p05` — [imitation learning](p05-imitation/README.md): NumPy and YAML only.
- `p06` — [reinforcement learning](p06-reinforcement-learning/README.md): NumPy and YAML only.
- `p07` — [project reference](p07-project/README.md): MuJoCo and SciPy.

Every profile includes the local editable `forc` package, NumPy, and PyYAML. The `all` profile combines all seven, including the tutorial. The learning reference experiments use NumPy; PyTorch and GPU libraries are not needed. Install `--extra sim` for only the headless simulation stack, or add `--extra notebooks` to another profile for Jupyter and plotting. Optional `mjx` and `comparison` extras remain available for extensions; they are not required by the current sessions.

## Quick start with conda

Install [Miniforge](https://github.com/conda-forge/miniforge), then:

```bash
conda env create --file environment.yml
conda activate forc
python scripts/setup_conda.py all
python scripts/run_practices.py
python -m jupyterlab practices/p01-simulation/tutorial.ipynb
```

For one practice, replace the installation and run commands:

```bash
python scripts/setup_conda.py p04
python practices/p04-predictive-control/run.py
```

Conda supplies Python, uv, and FFmpeg. The helper exports the selected profile from `uv.lock` and installs those exact Python package versions into the **active conda environment**, including the editable course package. It does not create `.venv` or remove conda-managed packages. Use `python` and `python -m jupyterlab` in this workflow; plain `uv run` would select the separate project environment.

Select the `forc` conda interpreter in your editor. If using a Jupyter server outside this environment, first register a kernel with `python -m ipykernel install --user --name forc --display-name "Python (FORC)"`, then select it. Skip registration when launching Jupyter from the environment itself.

Repeating `setup_conda.py` installs the requested locked versions, retaining unrelated packages. For a truly minimal profile after experimenting with other packages, create a new environment with `conda env create --name forc-p04 --file environment.yml`, activate it, and run `python scripts/setup_conda.py p04`. To update conda's interpreter/system tools later, recreate the environment from the YAML before rerunning the helper. The conda YAML pins tool versions; `uv.lock` pins Python packages, not platform-specific conda builds.

## Interactive viewer

Run `uv run --locked --extra p01 python practices/p01-simulation/run.py` on a desktop. Choose one of its three commented robot-loading lines, then use mouse perturbations to apply forces. Stop with Ctrl+C in the terminal or close the window. On macOS, replace `python` with `mjpython`. See [p01’s viewer instructions](p01-simulation/README.md#interactive-robot-explorer).

## Rendering and robot downloads

The finite reference checks work on CPU without a display or GPU. P01’s `run.py` opens an interactive desktop viewer; its separate `check.py` is the headless reference used by the automated runners. The p01 and p02 notebooks need graphics and video encoding. The tutorial’s UR5e, G1, and Go2 XML and meshes download on first use and stay in Menagerie's per-user cache, outside this repository. Prepare for an offline class with:

```bash
# uv environment
uv run --locked --extra p01 mujoco-menagerie prefetch universal_robots_ur5e unitree_g1 unitree_go2
# Or, inside the prepared conda environment
mujoco-menagerie prefetch universal_robots_ur5e unitree_g1 unitree_go2
```

For uv, install FFmpeg with your operating system's package manager (`sudo apt install ffmpeg` on Ubuntu/Debian, `brew install ffmpeg` on macOS). The conda environment and devcontainer include it. Desktop OpenGL usually works without configuration. On headless Linux, install Mesa (`sudo apt install libosmesa6`) and start Jupyter with:

```bash
MUJOCO_GL=osmesa uv run --locked --extra p01 jupyter lab practices/p01-simulation/tutorial.ipynb
```

Alternatively, use `MUJOCO_GL=egl` with a working EGL driver. Set the backend before importing MuJoCo; restart the kernel after changing it. No X11 socket sharing or privileged container is needed for offscreen rendering.

## VS Code devcontainer

Open the **repository root** and select **Dev Containers: Reopen in Container**. The root `.devcontainer` installs Python 3.12, pinned uv, FFmpeg, and software OpenGL, then synchronizes `all` plus test tools. Its environment lives inside the container so host and container packages cannot mix. It uses `MUJOCO_GL=osmesa` for CPU rendering. To use a smaller profile, run the same `uv sync --locked --extra pXX` command inside the container.

## Maintaining dependencies

`pyproject.toml` declares features and practice profiles; `uv.lock` is the single Python resolution for uv, conda, and CI. The old unpinned `practices/requirements.txt` has been removed. Do not install packages from notebook cells or maintain a second dependency list.

After intentionally editing dependencies, run `uv lock`, then verify:

```bash
uv sync --locked --extra all --group dev
uv run --locked --extra all --group dev pytest
uv run --locked --extra all python scripts/run_practices.py
uv run --locked --extra all python scripts/check_practice_envs.py
```

The last command installs and runs each practice in a separate temporary environment, checks dependency consistency, and confirms that the lightweight profiles do not pull in unnecessary engines. CI runs the same checks. To test only one profile, append `--profile p04`. For conda development, use `python scripts/setup_conda.py all --dev` followed by `python -m pytest`.

Generated results go in `outputs/`. Preserve each experiment's configuration and use a separate output directory for each comparison.

Robot modeling is part of the [p01 MuJoCo tutorial](p01-simulation/tutorial.ipynb): configurations, floating bases, body and COM kinematics, force balance, energy, actuation, contacts, and model editing all use MuJoCo and NumPy. Control and numerical linearization are in the separate [p02 notebook](p02-manipulator/control_and_linearization.ipynb). The `p01` profile includes all required dependencies.
