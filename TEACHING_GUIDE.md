# Teaching edition 0.2.0

This independent repository contains 12 original lecture units, eight practice briefs, executable reference experiments, and assessments. Current content lives in modules/, practices/p*/, src/forc/, and site/. Earlier material remains available as an archive and may use older conventions.

## Suggested teaching rhythm

For each 90-minute lecture: 10 minutes of prerequisite recall and the opening question, 25 minutes of derivation using the notes, 15 minutes of the worked example, 20 minutes of the linked live experiment, 15 minutes for the exercise and discussion, and 5 minutes to summarize assumptions and assign reading. The slide decks are teaching aids; the companion notes contain the detailed derivations and answer checks.

Each weekly practice reserves about two hours. Students first reproduce the working baseline, then change one parameter or assumption and explain the result. Reference implementations are public formative material. The take-homes require additional experiments and analysis, so merely rerunning the reference program is not a complete submission.

## Lecture 01 before practice 01

Start with [the modeling notes](modules/01-modeling/notes.md) and [their slides](modules/01-modeling/slides.md). They preserve the legacy `01_modeling` progression and its eight original figures. Suggested allocation for 90 minutes:

- **0–15 min:** robotic system, dynamical systems, and mathematical models (sections 1–3).
- **15–30 min:** ODEs, state space, linear systems, and the mass–spring–damper (4–6).
- **30–40 min:** nonlinear systems, pendulum, and variable-mass lander (7).
- **40–60 min:** generalized mechanics; floating body, satellite, and cart-pole (8).
- **60–75 min:** Other model classes (9), then MuJoCo state, kinematics, force, energy, and contact connections (10).
- **75–90 min:** numerical simulation, model editing, and questions before practice 01 (11–12).

Use the detailed Jacobian and energy checks as guided reading if the class needs more time on state construction. The analytical examples introduce modeling choices; executable practice uses Menagerie's UR5e, G1, and Go2. Figure captions translate the archived symbols, while the text gives corrected equations. The archive and copied PNGs remain unchanged.

Feedback is lecture 02, linked to practice 02. The course home page, sidebar, slide decks, and citation index derive their order from the module metadata. Previous lecture URLs redirect to the renamed modules.

## Reuse contract

Keep stable module IDs and URLs between offerings. Change schedule metadata rather than duplicating all notes for a new year. Each course owns its code and content; there is no sibling-repository import or runtime dependency. A future template should copy this structure and tests, not force shared teaching content.

## Before an assessed offering

Review mathematical claims, select optional extensions appropriate to prerequisites, confirm grading weights and calendar, run the numerical suite, and review source rights. Mark teaching-draft until instructor review is complete. A lockfile and CITATION.cff make a release reproducible and citable, but do not imply peer review or an assigned DOI.

## Validation commands

~~~bash
uv sync --locked --extra all --group dev
uv run --locked --extra all --group dev pytest
uv run --locked --extra all --group dev python scripts/run_practices.py
npm --prefix site ci
npm --prefix site run build
npm --prefix site run check
~~~

Optional backend checks are documented separately and must be reported honestly when unavailable.

## Control-basics practice

Practice 02 bridges simulation and manipulator control. Its self-contained `tutorial.ipynb` is the single lesson source: the website renders its Markdown/code cells and embedded diagrams directly. Model definitions and all helper functions appear in the notebook; plots and inline MuJoCo videos are generated only when it runs. Keep committed outputs empty. The shared `scripts/run_practices.py --practice p02` checker executes the actual notebook cells with `VISUALIZE = False`, and the physics tests use those same definitions. Sections 1–5 form the core; nonlinear systems and cart-pole form a guided follow-up. Practice IDs and install profiles run from p01 to p08; former practice URLs redirect to their new numbers.
