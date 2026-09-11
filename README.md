# Fundamentals of Robot Control (FORC)

Independent course repository: 12 lecture units, Reveal.js slides, Eleventy notes, seven practice briefs, Python reference experiments, and assessments.

## Start

~~~bash
uv sync --locked
uv run --locked pytest
uv run --locked python scripts/run_practices.py
npm --prefix site ci
COURSE_PATH_PREFIX= npm --prefix site run dev
~~~

Open http://localhost:8080/. See [TEACHING_GUIDE.md](TEACHING_GUIDE.md) for the teaching rhythm and release checklist.

## Current material

- [modules/](modules/): module.yml, notes.md, and slides.md per lecture.
- [practices/](practices/): current labs use p01- through p07- directory names, each with YAML metadata/config and a runnable entrypoint.
- [src/forc/](src/forc/): local Python models, numerical utilities, and reference experiments.
- [assessments/](assessments/): two take-homes and the project, with rubrics.
- [course.yml](course.yml), [references](references/references.yml), and [CITATION.cff](CITATION.cff): metadata and attribution.
- [site/](site/): independent static-site build. The modeling course is not a dependency.

## Build and verify

~~~bash
npm --prefix site run build
npm --prefix site run check
~~~

Production uses `/courses/forc/` by default, so the public title page is https://simeon-ned.github.io/courses/forc/. `COURSE_PATH_PREFIX` selects another hosting subpath. Every push to `main` or `master` builds, validates, and publishes only `site/_site` through GitHub Pages; the workflow can also be started manually.

## Earlier material

Historical source notes and notebooks remain under `_legacy/content/`; obsolete Jekyll, Quarto, and standalone lecture-site bundles have been removed. Current teaching content is authored in `modules/` and rendered by Eleventy. The [syllabus](site/content/syllabus.md) is the current 12-lecture schedule.

This is a teaching draft with original explanations and links to the requested CMU/MIT sources. It is not a video transcript or a peer-reviewed release. New material retains the repository's [Apache-2.0 license](LICENSE); linked external material retains its own licensing.
