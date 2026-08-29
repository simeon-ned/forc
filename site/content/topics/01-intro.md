---
layout: docs.njk
title: Intro & simulation
subtitle: Topic 1 · Lectures 1–2
permalink: /topics/01-intro/
slides: /lectures/01-intro/
---

## Goals

- Explain what a control system is: feedback vs open-loop, objectives, architecture.
- Briefly recall kinematics/dynamics only as needed for control (prior course assumed).
- Simulate with ODEs and the MuJoCo / course software stack; sensing & actuation in sim.

## Lectures

| # | Title |
| --- | --- |
| **1** | What is control? Feedback, objectives, architecture; **brief** kinematics & dynamics recap |
| **2** | Simulation: ODEs, MuJoCo / software stack, sensing & actuation |

Each lecture is a standalone session: idea and key equations → short demo → 2–3 references + 1 GitHub repository.

## Practice

Guided lab — simple models → MuJoCo: write/inspect state-space toys; load a robot; open-loop sim; plot states, actuators, sensors. Starter material: [`practices/01_mujoco/`](https://github.com/simeon-ned/forc/tree/master/practices/01_mujoco).

## Software peek

Introductory exposure as needed: MuJoCo, Warp, mjlab, Newton, Pinocchio (see [Setup]({{ '/setup/' | url }})).

## Reading

- MuJoCo [Overview](https://mujoco.readthedocs.io/en/stable/overview.html)
- Full course map: [Syllabus]({{ '/syllabus/' | url }})

## Next

[Classical control]({{ '/topics/02-classical/' | url }}) (lectures 3–4)
