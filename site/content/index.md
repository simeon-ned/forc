---
layout: docs.njk
title: Fundamentals of Robot Control
subtitle: Classical → optimal → learning-based control, with MuJoCo practices.
permalink: /
---

## What this course is

This introductory course surveys robot control from classical methods to optimal and learning-based approaches. Students briefly recall kinematics/dynamics, learn what feedback control is, simulate systems in modern software, design **kinematic (velocity)** and **dynamic (torque)** controllers, and apply optimal control (LQR, trajectory optimization, MPC, MPPI) and learning methods (behavior cloning, DAgger, reinforcement learning).

Style is **breadth over depth**: **12 standalone lectures** grouped into topics, plus weekly practices — core ideas, simulation, and curated references / GitHub code, not deep proofs. Advanced classical and learning methods are omitted; Lyapunov appears only lightly as energy-based reasoning.

We will also have **guest lectures** from experts at **Sber Robotics Center** and **Yandex Robotics** (2–3 speakers) on themes outside the core topics.

Full detail: [Syllabus]({{ '/syllabus/' | url }}) · [PDF]({{ '/syllabus.pdf' | url }})

## Format

| | |
| --- | --- |
| Core teaching | **12 lectures** in **6 topics** + weekly practice |
| Assessment | Practices 20% · TH1 25% · TH2 25% · Course project (exam) 30% |
| Prerequisites | Linear algebra, ODEs, Python, Newtonian mechanics; **prior** kinematics & dynamics course |
| Stack | MuJoCo (Python) + brief exposure to Warp, mjlab, Pinocchio, Newton, Crocoddyl, CasADi, CVXOPT |

## Sections

| Section | Topics | Content | Assessment |
| --- | --- | --- | --- |
| I — Intro & classical | 1–3 | Control intro; kin/dyn recap; simulation; velocity, PD, ID; LQR start | Guided practices |
| II — Optimal control | 3–4 | LQR, traj opt, MPC, MPPI | **TH1** (with topic 3) |
| III — Learning | 5–6 | BC + DAgger; RL + wrap-up | **TH2** (with topic 5); **exam = project** |

## Topics

<div class="week-cards">
{% for topic in nav.topics %}
<a class="week-card" href="{{ topic.url | url }}">
  <div class="meta">Topic {{ topic.id }} · {{ topic.section }} · Lectures {% for lec in topic.lectures %}{{ lec.n }}{% if not loop.last %}, {% endif %}{% endfor %}</div>
  <strong>{{ topic.title }}</strong>
</a>
{% endfor %}
</div>

## Take-homes & exam

| Component | Released / when | Intent |
| --- | --- | --- |
| **TH1 — Optimal** | With topic 3 / LQR | Quadrotor traj opt + constrained MPC; why constraints beat unconstrained LQR |
| **TH2 — Learning** | With topic 5 | BC, DAgger, RL; analyze failure modes |
| **Course project** | End of term | Choose and justify a control stack on a robotics problem (this is the exam) |

In-class **LQR** (cart-pole + quadrotor) is separate from TH1 (**MPC + traj opt** on the same quadrotor plant).

## Getting started

1. Read the [Syllabus]({{ '/syllabus/' | url }})
2. Install the environment ([Setup]({{ '/setup/' | url }}))
3. Open [Topic 1 — Intro & simulation]({{ '/topics/01-intro/' | url }})
