---
layout: course.njk
title: Source map and attribution
permalink: /sources/
---

## How the external material informs this edition

This is original teaching material organized around the existing 12-lecture FORC syllabus. It is not a transcript, copy, or official adaptation of a CMU or MIT course. The public course indexes, linked notes, and documentation were used for topic alignment and conventions. The four supplied playlists are companion viewing. Their complete video contents have **not** been independently reviewed or transcribed, and no video timestamps are asserted.

The source links are recorded with an access date of 11 September 2026. Online notes and stable documentation can change. Preserve a release tag or commit when citing this edition.

## CMU dynamics

[@cmu-dynamics] and [@cmu-dynamics-code] connect modeling assumptions, integration, rigid-body kinematics, recursive algorithms, constraints, and contact. In the modeling course these form the week-by-week conceptual progression. In FORC they support the compact simulation and dynamics recap rather than duplicating an entire dynamics course.

## CMU optimal control and reinforcement learning

[@cmu-control] connects local feedback, optimal control, and learning. [@cmu-index] is a dated lecture index, with topics including LQR, MPC, trajectory optimization, rotations, and quadrotors. FORC lectures 6–8 develop local control and predictive control; lectures 9–11 introduce learning. The modeling course uses discretization and rotation topics as supporting context.

The current course index includes multiple editions. Topic alignment here does not imply that lecture numbering matches every linked playlist.

## MIT Underactuated Robotics

[@mit-underactuated-video] accompanies [@mit-underactuated]. The specific [@mit-lqr], [@mit-trajopt], and [@mit-multibody] chapters connect to the corresponding derivations and exercises. We translate their notation into the MuJoCo contract rather than mixing conventions silently.

## MIT Robotic Manipulation

[@mit-manipulation-video] is **Robotic Manipulation, Fall 2023**. [@mit-manipulation] and [@mit-pick] support differential kinematics, manipulation, contact, and policy-learning context. FORC's Jacobian and imitation units use these connections; the modeling course uses frame and contact examples.

## MuJoCo and other algorithms

[@mujoco-computation] defines the engine's force decomposition; [@mujoco-api] supplies function-level links. [@mujoco-mjx] supports the optional batching material. [@pinocchio] and [@drake-contact] are optional comparison references, not required engines for the core labs.

## Rights and reuse

New notes, code, and slides are authored within this repository and retain its existing Apache-2.0 license. Linked lectures, notes, images, and code remain governed by their owners' licenses. This edition does not redistribute external slides, transcripts, figures, or notebook code. A citation is attribution, not permission to copy an entire source.

Each module lists its relevant references in module.yml. Use the references page for the complete linked records, and the citation page to cite this repository.
