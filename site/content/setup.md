---
layout: docs.njk
title: Setup
subtitle: Environment for practices, take-homes, and shared helpers.
permalink: /setup/
---

## Clone

```bash
git clone https://github.com/simeon-ned/forc.git
cd forc
```

## Python package

Shared helpers live in `src/forc/`. From the repo root:

```bash
pip install -e ".[sim]"
# or, if you use uv:
uv sync
```

Then in notebooks:

```python
from forc.sim import load_model
from forc.plot import states
```

## Practices environment

See [`practices/README.md`](https://github.com/simeon-ned/forc/blob/master/practices/README.md) for the recommended **devcontainer** / Docker setup (MuJoCo + OSMesa).

A conda-style env file is also at the repo root (`environment.yml`) for local installs.

## Software you will touch

Brief, practical exposure as needed by topic (not a full software course):

| Area | Tools |
| --- | --- |
| Dynamics / simulation / learning envs | MuJoCo, Warp, mjlab, Newton, Pinocchio |
| Optimal control / traj opt | Crocoddyl, CasADi, CVXOPT |

Primary lab stack: **MuJoCo + Python** notebooks (local or containerized).

## Site (optional, for contributors)

```bash
cd site
npm install
FORC_PATH_PREFIX= npm run dev   # http://localhost:8080
```

Production builds use path prefix `/forc` for GitHub Project Pages.

## Lectures

Slide decks are static Reveal.js folders under `lectures/`. Open any `index.html` locally, or use the **Open slides** button on week pages after the site is built.

Course overview and assessment: [Syllabus]({{ '/syllabus/' | url }}).
