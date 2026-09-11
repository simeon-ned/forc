---
layout: course.njk
title: Setup and reproducible workflow
permalink: /setup/
---

## Python with uv

Work in this course's repository. It has its own Python package, environment, and lockfile; the other course is not a dependency.

~~~bash
uv sync --locked
uv run --locked python practices/p01-simulation/run.py
uv run --locked pytest
~~~

Use Python 3.12 as recorded in .python-version. Core examples are headless CPU programs and do not require OpenGL, a viewer, or a GPU. Generated metrics and CSV traces go under outputs/. Keep each experiment's YAML file and output directory separate.

Install [uv using Astral's instructions](https://docs.astral.sh/uv/getting-started/installation/) if it is not available. The dependency lock fixes the tested software; a successful install does not replace the course's numerical tests.

## Optional environments

~~~bash
uv sync --locked --extra notebooks
uv sync --locked --extra mjx
uv sync --locked --extra comparison
~~~

Extras are optional and can be combined. MJX and Pinocchio comparisons live in the modeling course's Practice 7. Installing an extra does not mean every possible model feature or hardware backend has been tested.

## Notes and slides

The site uses Eleventy, with Reveal.js and KaTeX served locally from the built site.

~~~bash
npm --prefix site ci
npm --prefix site run build
npm --prefix site run check
COURSE_PATH_PREFIX= npm --prefix site run dev
~~~

Local development uses port 8080. Production defaults to /courses/forc/. Set COURSE_PATH_PREFIX to an empty string for root hosting, or to the actual deployment subpath. The generated static site is site/_site/.

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
