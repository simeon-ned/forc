---
layout: course.njk
title: Source map and attribution
permalink: /sources/
---

## How the external material informs this edition

This is original teaching material organized around the existing 12-lecture FORC syllabus. It is not a transcript, copy, or official adaptation of a CMU or MIT course. The public course indexes, linked notes, and documentation were used for topic alignment and conventions. The four supplied playlists are companion viewing. Their complete video contents have **not** been independently reviewed or transcribed, and no video timestamps are asserted.

Source access dates are recorded per entry on the references page. Online notes and stable documentation can change. Preserve a release tag or commit when citing this edition.

## Lecture 01: the original FORC modeling lecture

[Robotic systems, models, and state space]({{ '/notes/01-modeling/' | url }}) adapts `_legacy/content/_sort/01_modeling/01_modeling_notes.md` and its companion notebook. Its structure follows the original progression: robotic system, dynamical systems, mathematical models, differential equations, state space, linear and nonlinear examples, generalized mechanics, other model classes, and simulation.

The eight original figures are copied unchanged into `modules/01-modeling/images/` and included in both the notes and slides. The [figure provenance record]({{ '/assets/lectures/01-modeling/README.md' | url }}) identifies the source paths. Captions translate archived symbols into the MuJoCo convention, and the surrounding text corrects the original cart-pole input placement and polar-coordinate generalized inertia/input pairing. No CMU or MIT figures have been added.

The lecture is the first module in the course navigation and has an [individual citation entry]({{ '/cite/#01-modeling' | url }}).

## CMU dynamics

[@cmu-dynamics] and [@cmu-dynamics-code] support the compact simulation and dynamics recap in lectures 1–5. They are companions, not a second dynamics course inside FORC.

## CMU optimal control and reinforcement learning

[@cmu-control] connects local feedback, optimal control, and learning. [@cmu-index] is a dated lecture index, with topics including LQR, MPC, trajectory optimization, rotations, and quadrotors. FORC lectures 6–8 develop local control and predictive control; lectures 9–11 introduce learning.

The current course index includes multiple editions. Topic alignment here does not imply that lecture numbering matches every linked playlist.

## MIT Underactuated Robotics

[@mit-underactuated-video] accompanies [@mit-underactuated]. The specific [@mit-lqr], [@mit-trajopt], and [@mit-multibody] chapters connect to the corresponding derivations and exercises. We translate their notation into the MuJoCo contract rather than mixing conventions silently.

## MIT Robotic Manipulation

[@mit-manipulation-video] is **Robotic Manipulation, Fall 2023**. [@mit-manipulation] and [@mit-pick] support differential kinematics, imitation, and policy-learning context for FORC lectures 3 and 9–10.

## MuJoCo and other algorithms

[@mujoco-computation] defines the engine's force decomposition; [@mujoco-api] supplies function-level links. [@mujoco-mjx] supports the optional batching material. [@pinocchio] and [@drake-contact] are optional comparison references, not required engines for the core labs.

## Rights and reuse

New notes, code, and slides are authored within this repository and retain its existing Apache-2.0 license. Linked lectures, notes, images, and code remain governed by their owners' licenses. This edition does not add external course slides, transcripts, or notebook code. Lecture 01 reuses figures already present in the FORC archive, including their embedded imagery; original third-party rights remain unchanged. A citation is attribution, not permission to copy an entire source.

Each module lists its relevant references in module.yml. Use the references page for the complete linked records, and the citation page to cite this repository.
