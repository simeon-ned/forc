---
layout: course.njk
title: Syllabus
permalink: /syllabus/
---

**Subject area:** Robotics  
**Format:** **12 lectures** organized by topic, plus a weekly practice; calendar spans about 7 weeks  
**Prior course:** Full kinematics and dynamics are covered previously; here they are only briefly recalled.

We will also host **guest lectures** from experts at **Sber Robotics Center** and **Yandex Robotics** (2–3 speakers) on themes outside the core topics below — not part of the taught topic list.

## Short description

This introductory course surveys robot control from classical methods to optimal and learning-based approaches. Students briefly recall kinematics/dynamics, learn what feedback control is, simulate systems in modern software, design **kinematic (velocity)** and **dynamic (torque)** controllers, and apply optimal control (LQR, trajectory optimization, MPC, MPPI) and learning methods (behavior cloning, DAgger, reinforcement learning).

The course emphasizes intuition, implementation, and method selection over deep proofs. **Advanced methods in both classical and learning-based control are omitted**; Lyapunov theory appears only lightly as energy-based reasoning.

Students also get a **brief, practical introduction** to common robotics software (e.g. MuJoCo, Warp, mjlab, Pinocchio, Newton, Crocoddyl, CasADi, CVXOPT) — enough for labs and take-homes, not a full software course.

## Intended learning outcomes

### Concepts (know / explain)

- What a control system is; feedback vs open-loop; goals of robot control
- Brief recall of kinematics/dynamics as needed for control
- Velocity / Jacobian-based control and linear dynamic control (PD/PID, state feedback)
- Inverse dynamics / gravity compensation; light energy / Lyapunov intuition
- Optimal control: LQR, trajectory optimization, MPC, MPPI
- Behavior cloning; distribution shift; DAgger
- RL: MDP, observations, rewards, policy; locomotion case studies
- When to choose kinematic vs dynamic, classical vs optimal vs learning methods
- Exposure to industrial perspectives via guest talks (beyond the core topics)

### Skills (do)

- Simulate robots in MuJoCo and related stacks; wire simple control loops
- Use, at an introductory level, MuJoCo, Warp, mjlab, Pinocchio, Newton, Crocoddyl, CasADi, CVXOPT
- Implement velocity- and torque-level controllers on provided plants
- Implement and compare LQR / traj-opt / MPC / MPPI
- Train and evaluate BC, DAgger-style loops, and simple RL setups

### Synthesis (apply)

- Select and justify a control stack for a specific robotics problem
- Deliver a course project demonstrating applied understanding

## Course sections

| Section | Topics | Content | Assessment |
| --- | --- | --- | --- |
| I — Intro & classical | 1–3 | Control intro; brief kin/dyn; simulation; velocity, PD, ID; LQR start | Guided practices (simulate & implement) |
| II — Optimal control | 3–4 | LQR, trajectory optimization, MPC, MPPI | **Take-home 1** (released with topic 3 / LQR) |
| III — Learning | 5–6 | BC + DAgger; RL + short wrap-up | **Take-home 2** (released with topic 5); **exam = course project** |

Guest lectures and project presentations happen outside these topic blocks (typically toward the end of the term).

## Lectures by topic

Each lecture is a standalone session (idea and key equations → short demo → 2–3 references + 1 GitHub repository).

### Topic 1 — Intro & simulation

| # | Lecture |
| --- | --- |
| 1 | What is control? Feedback, objectives, architecture; **brief** kinematics & dynamics recap |
| 2 | Simulation: ODEs, MuJoCo / software stack, sensing & actuation |

### Topic 2 — Classical control

| # | Lecture |
| --- | --- |
| 3 | Velocity / Jacobian-based control (tracking, singularities) |
| 4 | Linear control: PD/PID, full-state feedback |

### Topic 3 — Dynamic control & LQR

| # | Lecture |
| --- | --- |
| 5 | Inverse dynamics & gravity compensation (+ light Lyapunov / energy) |
| 6 | LQR (+ trajectory-optimization sketch) |

### Topic 4 — Receding horizon & sampling

| # | Lecture |
| --- | --- |
| 7 | Model predictive control (MPC) |
| 8 | Model predictive path integral (MPPI) |

### Topic 5 — Imitation learning

| # | Lecture |
| --- | --- |
| 9 | Behavior cloning: setup, data, supervised policies, failure modes |
| 10 | DAgger: algorithm, distribution shift, practice patterns, limits |

### Topic 6 — Reinforcement learning

| # | Lecture |
| --- | --- |
| 11 | RL basics: MDP, observations, actions, rewards, policy |
| 12 | Cases: quadruped tracking + rewards; humanoid IL; short course wrap-up |

Trajectory optimization is sketched in lecture 6 and done properly in **Take-home 1** together with **constrained MPC on a quadrotor**. Lectures 7 and 8 give MPC and MPPI full sessions each.

## Guest lectures

Separately from the topics above, the course includes **guest lectures** by experts from **Sber Robotics Center** and **Yandex Robotics** (2–3 speakers). Themes are **not** in the core list; possible examples:

- Text-to-motion and generative modeling
- VLA, VLM, and foundation models
- State estimation and perception for legged robots
- Joint optimal design and control for complex articulated systems

## Practice schedule

Practices are held in **MuJoCo** (Python): guided labs, take-homes, and the course project all run in the same simulation stack. Practice weeks roughly follow the topic calendar.

| Week | Mode | Plant / focus | Short description |
| --- | --- | --- | --- |
| 1 | Guided lab | Simple models → MuJoCo | Write/inspect state-space toys; load a MuJoCo robot; open-loop sim; plot states, actuators, sensors |
| 2 | Guided lab | Manipulator | **IK + inverse dynamics** (one lab): Jacobian / IK tracking; ID / gravity compensation vs PD |
| 3 | In-class lab / TH1 start | **Cart-pole + quadrotor** | **LQR in class** on a cart-pole and on a quadrotor (hover / tracking); clarify Take-home 1 |
| 4 | Take-home | **Quadrotor** | Work on TH1: **trajectory optimization + constrained MPC** (thrust / rate / box limits); contrast with unconstrained LQR; optional MPPI; checkpoint |
| 5 | Take-home | IL task (provided) | Kick off Take-home 2: behavior cloning + DAgger; early progress with TA |
| 6 | Take-home / project | RL + project | Reward / RL experiments; course-project progress; TA feedback |
| 7 | Project | Course project | Dry-runs / presentations; final TA feedback |

**LQR (practice week 3):** in-class on **cart-pole and quadrotor**. **TH1:** **MPC + traj opt on quadrotor** — same plant as in-class quadrotor LQR so constraints are clearly useful.

Weeks 1–3 are guided / in-class labs. Later weeks are mainly take-home and project work with TA support.

See also [Practices]({{ '/practices/' | url }}).

## Assessment

### Take-homes

Released at the **start of each advanced section**. Practice sessions are for questions, TA help, and showing progress.

| Take-home | Released | Covers | Intent |
| --- | --- | --- | --- |
| **TH1 — Optimal control** | With topic 3 / LQR | Trajectory optimization, constrained MPC (quadrotor); optional MPPI | Plan and control a **quadrotor** with traj opt + MPC; show why constraints beat unconstrained LQR |
| **TH2 — Learning-based control** | With topic 5 | BC, DAgger, RL | Imitation and RL on a control task; analyze failure modes |

**Suggested due dates:** TH1 after topic 4; TH2 after topic 6 or with the project.

### Exam — course project

No classical written exam. Students apply course knowledge to a **specific robotics problem** and present:

1. Problem and model
2. Method choice (kinematic / classical / optimal / learning) with justification
3. Implementation and results
4. Limitations and next steps

The presentation / demo is the exam (typically near the end of the term, alongside guest talks).

### Grade breakdown (draft)

| Component | Weight |
| --- | --- |
| Practices / participation & progress check-ins | 20% |
| Take-home 1 | 25% |
| Take-home 2 | 25% |
| Course project (exam) | 30% |

## Prerequisites

**Required**

- Linear algebra (vectors, matrices, eigenvalues)
- Ordinary differential equations / basic dynamical systems
- Python programming (NumPy-level comfort; notebooks and simple debugging)
- Basic Newtonian mechanics (forces, energy, rigid-body intuition)
- **Previous course** covering robot kinematics, dynamics, and modeling / simulation

**Helpful (not assumed in depth)**

- Basic linear / classical control (PID, feedback intuition)
- Exposure to numerical optimization
- Exposure to supervised machine learning

Kinematics and dynamics are **only briefly reviewed**. Deep prior experience with MPC, RL, or CasADi / MuJoCo is **not** required. This should **not** be a first course in programming or feedback systems.

## Organization & tools

- Interactive lectures + weekly practice
- Guest lectures from Sber Robotics Center and Yandex Robotics (2–3 speakers), outside the core topic list
- Python; notebooks / local or containerized environments as provided
- Brief hands-on exposure to robotics software, including (as needed by topic):
  - **Dynamics / simulation / learning envs:** MuJoCo, Warp, mjlab, Newton, Pinocchio
  - **Optimal control / traj opt:** Crocoddyl, CasADi, CVXOPT
- One running plant across topics 2–6 where possible so methods are comparable
- Handwritten notes encouraged

Install instructions: [Setup]({{ '/setup/' | url }}).

## References (indicative)

### Textbooks

- Spong, Hutchinson, Vidyasagar — *Robot Modeling and Control*
- Siciliano, Sciavicco, Villani, Oriolo — *Robotics: Modelling, Planning and Control*
- Lynch & Park — *Modern Robotics: Mechanics, Planning, and Control* ([online](http://modernrobotics.org/))
- Murray, Li, Sastry — *A Mathematical Introduction to Robotic Manipulation* ([PDF](https://www.cds.caltech.edu/~murray/mlswiki))
- Slotine & Li — *Applied Nonlinear Control*
- Kirk — *Optimal Control Theory: An Introduction*
- Rawlings, Mayne, Diehl — *Model Predictive Control: Theory, Computation, and Design*
- Sutton & Barto — *Reinforcement Learning: An Introduction* ([online](http://incompleteideas.net/book/the-book-2nd.html))
- Brunton & Kutz — *Data-Driven Science and Engineering* ([databookuw.com](http://www.databookuw.com/))

### Online courses & lecture series

- [Underactuated Robotics](https://underactuated.mit.edu/) — Russ Tedrake (MIT); notes, videos, code
- [Control Bootcamp](https://www.youtube.com/playlist?list=PLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085m) — Steve Brunton; linear / nonlinear control
- [Optimal Control and RL CMU 16-745](https://www.youtube.com/watch?v=SvAYJC7jug8&list=PLZnJoM76RM6IAJfMXd1PgGNXn3dxhkVgI) — Zachary Manchester; LQR, traj opt, MPC, RL
- [UC Berkeley CS285 Deep RL](https://rail.eecs.berkeley.edu/deeprlcourse/) — Sergey Levine; imitation learning + RL
- [Stanford CS224R](https://cs224r.stanford.edu/) — RL and learning for robots (optional depth)
- [Slotine Nonlinear Control](https://www.bilibili.com/video/BV1yb411e7t5/) — nonlinear control lecture series
- [Data-Driven Dynamical Systems & Control](http://www.databookuw.com/) — Brunton & Kutz; book + short videos
- [Coursera: Robotics Specialization (Penn)](https://www.coursera.org/specializations/robotics) — kinematics, dynamics, control refreshers
- [OpenAI Spinning Up in Deep RL](https://spinningup.openai.com/) — practical RL algorithms overview
- [MuJoCo documentation & tutorials](https://mujoco.readthedocs.io/) — simulation stack used in labs

Exact paper and GitHub links are provided per lecture.

## Out of scope (this edition)

**Advanced methods in classical and learning-based control are omitted** from the taught core (adaptive, robust / H∞, sliding mode, full Lyapunov theory, CLF/CBF, advanced RL, foundation / VLA–VLM stacks, generative motion models, etc.). Related themes may appear in **guest lectures**.

Full kinematics/dynamics derivations belong to the **previous course**. Software coverage is introductory only.
