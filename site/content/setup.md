---
layout: course.njk
title: Setup and reproducible workflow
permalink: /setup/
---

## Python with uv

Run from the repository root with Python 3.12 (`.python-version`). Install [uv](https://docs.astral.sh/uv/getting-started/installation/), then choose all seven current practices:

~~~bash
uv sync --locked --extra all
uv run --locked --extra all python scripts/run_practices.py
uv run --locked --extra all jupyter lab practices/p01-simulation/tutorial.ipynb
~~~

For only one practice, use its short ID on both commands:

~~~bash
uv sync --locked --extra p04
uv run --locked --extra p04 python practices/p04-predictive-control/run.py
~~~

Profiles `p01` through `p07` install only their required features. Combine multiple `--extra` flags to keep several selected. The p01 and p02 profiles include their notebooks and Menagerie; p04–p06 do not require MuJoCo. uv manages `.venv`; select it in your editor. Keep the profile on `uv run`, which also synchronizes dependencies.

## Python with conda

~~~bash
conda env create --file environment.yml
conda activate forc
python scripts/setup_conda.py all
python scripts/run_practices.py
python -m jupyterlab practices/p01-simulation/tutorial.ipynb
~~~

Replace `all` with a practice ID to select a smaller install. Conda supplies Python, uv, and FFmpeg. The helper installs Python dependencies from the same `uv.lock` into the active conda environment without pruning conda's packages. Use `python` inside that environment rather than `uv run`.

The repository's `practices/README.md` covers editor kernels, offline Menagerie caching, and environment maintenance. The root `.devcontainer` provides the all-practices environment with FFmpeg and software OpenGL.

## Rendering and verification

The automated reference checks are headless CPU experiments. P01’s `run.py` is an interactive desktop viewer; `check.py` provides its finite headless checks. Run `uv run --locked --extra p01 python practices/p01-simulation/run.py` to open the viewer (use `mjpython` on macOS), and stop it with Ctrl+C or by closing the window. The p01 and p02 notebooks additionally need OpenGL and FFmpeg; install FFmpeg with your OS package manager when using uv. Conda and the devcontainer include it. On headless Linux, set `MUJOCO_GL=osmesa` with Mesa installed, or `MUJOCO_GL=egl` with a working EGL driver, before starting the kernel. Menagerie downloads UR5e, G1, and Go2 on first use and caches them locally.

~~~bash
uv run --locked --extra p01 mujoco-menagerie prefetch universal_robots_ur5e unitree_g1 unitree_go2
uv run --locked --extra all --group dev pytest
uv run --locked --extra all python scripts/check_practice_envs.py
~~~

`pyproject.toml` defines profiles and `uv.lock` pins Python packages. Update the lock only when changing dependencies intentionally. Generated metrics and traces go under `outputs/`. Keep each comparison's YAML configuration and output directory separate. Optional `mjx` and `comparison` extras are available for extensions; they are not part of `all`.

## Notes and slides

The site uses Eleventy, with Reveal.js and KaTeX served locally from the built site.

~~~bash
npm --prefix site ci
npm --prefix site run build
npm --prefix site run check
COURSE_PATH_PREFIX= npm --prefix site run dev
~~~

Local development uses port 8080. Production defaults to /forc/. Set COURSE_PATH_PREFIX to an empty string for root hosting, or to the actual deployment subpath. The generated static site is site/_site/.

## Content ownership

Each lecture lives in modules/<id>/:

- module.yml declares its stable ID, week, objectives, practice, and reference IDs.
- notes.md contains explanations, derivations, worked examples, and exercises.
- slides.md contains short Reveal sections separated by a line containing three hyphens.

Each practice contains practice.yml, config.yml, README.md, and run.py. Reusable Python functions live in src/forc/. course.yml holds course metadata, references/references.yml holds source records, and CITATION.cff supplies citation metadata.

YAML describes data and experiment parameters. Python still uses pyproject.toml and uv.lock, and Eleventy still needs a small JavaScript build configuration. These tool-native files are intentionally retained.

## Editing and release

Changing notes, slides, or YAML rebuilds the local site. Restart the server after changing the hosting path prefix. Run tests and the site link check before release. This edition is a teaching draft; publication requires instructor review and an explicit deployment step. No DOI has been assigned.

See [the course]({{ '/' | url }}), [notation]({{ '/notation/' | url }}), and [source map]({{ '/sources/' | url }}).
