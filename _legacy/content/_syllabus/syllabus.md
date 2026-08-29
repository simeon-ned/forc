# Fundamentals of Robot Control — Syllabus

**Subject area:** Robotics  
**Format:** 7 weeks × (2 lectures + 1 practice day) = **14 lectures**  
**Style:** Breadth over depth — core ideas, simulation, and curated references / GitHub code per topic

**Prior course:** Full kinematics and dynamics are covered previously; here they are only briefly recalled.

---

## 1. Short description

This introductory course surveys robot control from classical methods to optimal and learning-based approaches. Students briefly recall kinematics/dynamics, learn what feedback control is, simulate systems in modern software, design **kinematic (velocity)** and **dynamic (torque)** controllers, and apply optimal control (LQR, trajectory optimization, MPC, MPPI) and learning methods (behavior cloning, DAgger, reinforcement learning).

The final week features **guest lectures from experts in the field** from **Sber Robotics Center** and **Yandex Robotics** (2–3 speakers) on topics **not covered** in the core syllabus.

The course emphasizes intuition, implementation, and method selection over deep proofs. **Advanced methods in both classical and learning-based control are omitted**; Lyapunov theory appears only lightly as energy-based reasoning.

Students also get a **brief, practical introduction** to common robotics software (e.g. MuJoCo, Warp, mjlab, Pinocchio, Newton, Crocoddyl, CasADi, CVXOPT) — enough for labs and take-homes, not a full software course.

---

## 2. Intended learning outcomes

### Concepts (know / explain)
- What a control system is; feedback vs open-loop; goals of robot control
- Brief recall of kinematics/dynamics as needed for control
- Velocity / Jacobian-based control and linear dynamic control (PD/PID, state feedback)
- Inverse dynamics / gravity compensation; light energy / Lyapunov intuition
- Optimal control: LQR, trajectory optimization, MPC, MPPI
- Behavior cloning; distribution shift; DAgger
- RL: MDP, observations, rewards, policy; locomotion case studies
- When to choose kinematic vs dynamic, classical vs optimal vs learning methods
- Exposure to industrial perspectives on topics beyond the core lectures

### Skills (do)
- Simulate robots in MuJoCo and related stacks; wire simple control loops
- Use, at an introductory level, MuJoCo, Warp, mjlab, Pinocchio, Newton, Crocoddyl, CasADi, CVXOPT
- Implement velocity- and torque-level controllers on provided plants
- Implement and compare LQR / traj-opt / MPC / MPPI
- Train and evaluate BC, DAgger-style loops, and simple RL setups

### Synthesis (apply)
- Select and justify a control stack for a specific robotics problem
- Deliver a course project demonstrating applied understanding

---

## 3. Course sections

| Section | Weeks | Content | Assessment |
| --- | --- | --- | --- |
| I — Intro & classical | 1–3 | Control intro; brief kin/dyn; simulation; velocity, PD, ID; LQR start | Guided practices (simulate & implement) |
| II — Optimal control | 3–4 | LQR, trajectory optimization, MPC, MPPI | **Take-home 1** (released start of week 3) |
| III — Learning | 5–6 | Week 5: BC + DAgger; week 6: RL + short wrap-up | **Take-home 2** (released start of week 5); **exam = course project** |
| IV — Industry guests | 7 | Guest lectures by experts from Sber Robotics Center and Yandex Robotics (topics not covered in class) | Project  / Q&A; project presentations as scheduled |

---

## 4. Lecture schedule (14 lectures)

| Week | Theme | Lecture A | Lecture B |
| --- | --- | --- | --- |
| 1 | Intro & simulation | What is control? Feedback, objectives, architecture; **brief** kinematics & dynamics recap | Simulation: ODEs, MuJoCo / software stack, sensing & actuation |
| 2 | Classical control | Velocity / Jacobian-based control (tracking, singularities) | Linear control: PD/PID, full-state feedback |
| 3 | Dynamic control & LQR | Inverse dynamics & gravity compensation (+ light Lyapunov / energy) | LQR (+ trajectory-optimization sketch) |
| 4 | Receding horizon & sampling | MPC | MPPI |
| 5 | Imitation learning | Behavior cloning: setup, data, supervised policies, failure modes | DAgger: algorithm, distribution shift, practice patterns, limits |
| 6 | Reinforcement learning | RL basics: MDP, observations, actions, rewards, policy | Cases: quadruped tracking + rewards; humanoid IL; short course wrap-up |
| 7 | Industry guest lectures | Guest talk(s) by experts from Sber Robotics Center / Yandex Robotics | Guest talk(s) continued; Q&A |

Trajectory optimization is sketched with LQR (week 3B) and done properly in **Take-home 1** together with **constrained MPC on a quadrotor**. MPC and MPPI each get a full lecture in week 4.

**Possible guest lecture topics** (2–3 talks by field experts from **Sber Robotics Center** and **Yandex Robotics**; final set depends on speakers):
- Text-to-motion and generative modeling 
- VLA, VLM, and foundation models
- State estimation and perception for legged robots
- Joint optimal design and control for complex articulated systems

Each core lecture: idea and key equations → short demo → 2–3 references + 1 GitHub repository.

---

## 5. Practice schedule

Practices are held in **MuJoCo** (Python): guided labs, take-home, and the course project all run in the same simulation stack.

| Week | Mode | Plant / focus | Short description |
| --- | --- | --- | --- |
| 1 | Guided lab | Simple models → MuJoCo | Write/inspect state-space toys; load a MuJoCo robot; open-loop sim; plot states, actuators, sensors |
| 2 | Guided lab | Manipulator | **IK + inverse dynamics** (one lab): Jacobian / IK tracking; ID / gravity compensation vs PD |
| 3 | In-class lab / TH1 start | **Cart-pole + quadrotor** | **LQR in class** on a cart-pole and on a quadrotor (hover / tracking); clarify Take-home 1 |
| 4 | Take-home  | **Quadrotor** | Work on TH1: **trajectory optimization + constrained MPC** on quadrotor (thrust / rate / box limits); contrast with unconstrained LQR; optional MPPI; checkpoint |
| 5 | Take-home  | IL task (provided) | Kick off Take-home 2: behavior cloning + DAgger; early progress with TA |
| 6 | Take-home / project  | RL + project | Reward / RL experiments; course-project progress; TA feedback |
| 7 | Project  | Course project | Dry-runs / presentations; guest Q&A spillover; final TA feedback |

**LQR (week 3):** in-class on **cart-pole and quadrotor**. **TH1 (weeks 3–4):** **MPC + traj opt on quadrotor** — same plant as in-class quadrotor LQR so constraints are clearly useful.

Weeks 1–3 are guided / in-class labs. Weeks 4–7 are mainly take-home and project  with TA support.

---

## 6. Assessment

### Take-homes

Released at the **start of each advanced section**. Practice sessions are for questions, TA help, and showing progress.

| Take-home | Released | Covers | Intent |
| --- | --- | --- | --- |
| **TH1 — Optimal control** | Start of week 3 | Trajectory optimization, constrained MPC (quadrotor); optional MPPI | Plan and control a **quadrotor** with traj opt + MPC; show why constraints beat unconstrained LQR |
| **TH2 — Learning-based control** | Start of week 5 | BC, DAgger, RL | Imitation and RL on a control task; analyze failure modes |

**Suggested due dates:** TH1 end of week 4; TH2 end of week 6 or with the project (week 7).

### Exam — course project

No classical written exam. Students apply course knowledge to a **specific robotics problem** and present:

1. Problem and model  
2. Method choice (kinematic / classical / optimal / learning) with justification  
3. Implementation and results  
4. Limitations and next steps  

The presentation / demo is the exam (typically during or around week 7).

### Grade breakdown (draft)

| Component | Weight |
| --- | --- |
| Practices / participation & progress check-ins | 20% |
| Take-home 1 | 25% |
| Take-home 2 | 25% |
| Course project (exam) | 30% |

---

## 7. Prerequisites

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

---

## 8. Organization & tools

- Interactive lectures + weekly practice
- Week 7: guest lectures by experts from Sber Robotics Center and Yandex Robotics (2–3 speakers)
- Python; notebooks / local or containerized environments as provided
- Brief hands-on exposure to robotics software, including (as needed by topic):
  - **Dynamics and Simulation / learning envs:** MuJoCo, Warp, mjlab, Newton, Pinocchio
  - **Optimal control / traj opt:** Crocoddyl, CasADi, CVXOPT
- One running plant across weeks 2–6 where possible so methods are comparable
- Handwritten notes encouraged

---

## 9. References (indicative)

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
- [Stanford CS224R Deep Learnings](https://cs224r.stanford.edu/) — RL and learning for robots (optional depth)
- [Slotine Nonlinear Control](https://www.bilibili.com/video/BV1yb411e7t5/) — nonlinear control lecture series
- [Data-Driven Dynamical Systems & Control](http://www.databookuw.com/) — Brunton & Kutz; book + short videos
- [Coursera: Robotics Specialization (Penn)](https://www.coursera.org/specializations/robotics) — kinematics, dynamics, control refreshers
- [OpenAI Spinning Up in Deep RL](https://spinningup.openai.com/) — practical RL algorithms overview
- [MuJoCo documentation & tutorials](https://mujoco.readthedocs.io/) — simulation stack used in labs

Exact paper and GitHub links are provided per lecture.

---

## 10. Out of scope (this edition)

**Advanced methods in classical and learning-based control are omitted** from the taught core (adaptive, robust / H∞, sliding mode, full Lyapunov theory, CLF/CBF, advanced RL, foundation / VLA–VLM stacks, generative motion models, etc.). Related themes may appear in **week 7 guest lectures**.

Full kinematics/dynamics derivations belong to the **previous course**. Software coverage is introductory only.
