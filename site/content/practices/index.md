---
layout: docs.njk
title: Practices
subtitle: MuJoCo (Python) — same stack for guided labs, take-homes, and the project.
permalink: /practices/
---

Practices live in [`practices/`](https://github.com/simeon-ned/forc/tree/master/practices). Install helpers first ([Setup]({{ '/setup/' | url }})), then open the material for the week.

Practice weeks roughly follow the [topics]({{ '/syllabus/#lectures-by-topic' | url }}). Early weeks are **guided / in-class**; later weeks are mainly **take-home and project** work with TA support.

## Schedule

| Week | Mode | Plant / focus | Short description | Location |
| --- | --- | --- | --- | --- |
| <span id="week-1">1</span> | Guided lab | Simple models → MuJoCo | State-space toys; load a robot; open-loop sim; plot states, actuators, sensors | `practices/01_mujoco/` |
| <span id="week-2">2</span> | Guided lab | Manipulator | **IK + inverse dynamics** (one lab): Jacobian / IK tracking; ID / gravity compensation vs PD | TBD |
| <span id="week-3">3</span> | In-class / TH1 start | **Cart-pole + quadrotor** | **LQR in class** (hover / tracking); clarify Take-home 1 | TBD |
| <span id="week-4">4</span> | Take-home | **Quadrotor** | TH1: traj opt + constrained MPC (thrust / rate / box limits) vs LQR; optional MPPI | `hw/th1_mpc/` |
| <span id="week-5">5</span> | Take-home | IL task (provided) | TH2 kickoff: BC + DAgger; early progress with TA | `hw/th2_learning/` |
| <span id="week-6">6</span> | Take-home / project | RL + project | Reward / RL experiments; project progress; TA feedback | `hw/th2_learning/` |
| <span id="week-7">7</span> | Project | Course project | Dry-runs / presentations; final TA feedback | — |

**Split:** LQR = in-class on cart-pole **and** quadrotor (week 3). TH1 = MPC + traj opt on the **same quadrotor** so constraints are clearly useful.

Existing starter notebooks (UR5e / IIWA / Go1) under `practices/` will be remapped as content is rewritten.
