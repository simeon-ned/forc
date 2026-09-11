# Teaching edition 0.2.0

This independent repository contains 12 original lecture units, seven practice briefs, executable reference experiments, and assessments. Current content lives in modules/, practices/p*/, src/forc/, and site/. Earlier material remains available as an archive and may use older conventions.

## Suggested teaching rhythm

For each 90-minute lecture: 10 minutes of prerequisite recall and the opening question, 25 minutes of derivation using the notes, 15 minutes of the worked example, 20 minutes of the linked live experiment, 15 minutes for the exercise and discussion, and 5 minutes to summarize assumptions and assign reading. The eight-slide decks are concise teaching aids, not transcripts of a 90-minute lecture.

Each weekly practice reserves about two hours. Students first reproduce the working baseline, then change one parameter or assumption and explain the result. Reference implementations are public formative material. The take-homes require additional experiments and analysis, so merely rerunning the reference program is not a complete submission.

## Reuse contract

Keep stable module IDs and URLs between offerings. Change schedule metadata rather than duplicating all notes for a new year. Each course owns its code and content; there is no sibling-repository import or runtime dependency. A future template should copy this structure and tests, not force shared teaching content.

## Before an assessed offering

Review mathematical claims, select optional extensions appropriate to prerequisites, confirm grading weights and calendar, run the numerical suite, and review source rights. Mark teaching-draft until instructor review is complete. A lockfile and CITATION.cff make a release reproducible and citable, but do not imply peer review or an assigned DOI.

## Validation commands

~~~bash
uv sync --locked
uv run --locked pytest
uv run --locked python scripts/run_practices.py
npm --prefix site ci
npm --prefix site run build
npm --prefix site run check
~~~

Optional backend checks are documented separately and must be reported honestly when unavailable.
