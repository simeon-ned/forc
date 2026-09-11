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

Open http://localhost:8080/course/. See [TEACHING_GUIDE.md](TEACHING_GUIDE.md) for the teaching rhythm and release checklist.

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

Production uses /forc/ by default; COURSE_PATH_PREFIX selects another hosting subpath. The existing Pages workflow now checks numerical tests and generated links before deployment. No deployment happens until the repository workflow is explicitly triggered by its configured events.

## Earlier material

The original decks under lectures/, older notebooks, hw/ templates, and _legacy/ remain available. New teaching content is authored in modules/ and rendered by Eleventy. The existing [syllabus](site/content/syllabus.md) remains the 12-lecture schedule; its older PDF is historical.

This is a teaching draft with original explanations and links to the requested CMU/MIT sources. It is not a video transcript or a peer-reviewed release. New material retains the repository's [Apache-2.0 license](LICENSE); linked external material retains its own licensing.
