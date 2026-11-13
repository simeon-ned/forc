---
layout: course.njk
title: Practice schedule and earlier notebooks
permalink: /practices/
---

The current teaching edition has eight practices with runnable Python notebooks or reference experiments and submission guidance. The first labs are formative; the later weeks support the take-homes and project.

~~~bash
uv sync --locked --extra all
uv run --locked --extra all python scripts/run_practices.py
~~~

{% for p in coursePractices %}
<h2 id="{{ p.id }}">{{ p.title }}</h2>
<p>Week {{ p.week }} · {{ p.duration }} minutes</p>

[Open practice]({{ p.url | url }}). The corresponding source is practices/{{ p.id }}/. {% if p.entrypoint == "tutorial.ipynb" %}The notebook contains the implementation, parameters, and inline results.{% else %}The reference program records its configuration and metrics under outputs/.{% endif %}

{% endfor %}

## Assessments

[TH1: constrained quadrotor motion]({{ '/assessments/th1/' | url }}) extends the full quadrotor LQR model with trajectory optimization and MPC. [TH2: learning]({{ '/assessments/th2/' | url }}) compares imitation and reward-based control. The [final project]({{ '/assessments/project/' | url }}) requires a bounded new question and reproducible evidence.

## Earlier notebooks

The older UR5e, IIWA, and Go1 notebooks remain in the repository. They are historical supplements and may use different dependencies or conventions. The current p01- through p08- practices and the MuJoCo notation contract define this edition's tested baseline.
