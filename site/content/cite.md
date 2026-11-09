---
layout: course.njk
title: Cite and reuse this course
permalink: /cite/
---
Cite the version and commit you actually consulted.

{{ course.author }}. *{{ course.title }}*. Version {{ course.version }} ({{ course.status }}). [{{ course.site_url }}/]({{ course.site_url }}/).

Download [CITATION.cff]({{ '/CITATION.cff' | url }}) for machine-readable attribution. New notes, slides, small MJCF models, and Python experiments were developed for this repository. Referenced CMU and MIT courses are independent works; their slides and videos have not been redistributed here. See the [source map]({{ '/sources/' | url }}).

The repository's existing Apache-2.0 license is retained. Third-party assets keep their own licenses. A release DOI has not been minted; do not cite an invented DOI or imply this draft has been externally reviewed.

Report corrections with the lecture ID, equation or code location, expected behavior, and environment version.

## Cite an individual lecture

Use the lecture title and current module ID, plus the course version and commit. The module list below follows the teaching order on the course home page.

{% for m in lessons %}
<p id="{{ m.id }}"><strong>Lecture {{ m.number }}.</strong> {{ course.author }}. “{{ m.title }}.” In <em>{{ course.title }}</em>, version {{ course.version }}. Module <code>{{ m.id }}</code>. <a href="{{ m.notes_url | url }}">Read lecture</a> · <a href="{{ m.slides_url | url }}">Slides</a>.</p>
{% endfor %}

Lecture 01 adapts the repository's archived `01_modeling` lecture and reuses its eight figures. The [source map]({{ '/sources/' | url }}) records this lineage; figure captions explain notation changes. The module citation identifies this edition and does not replace attribution for third-party elements within archived figures.
