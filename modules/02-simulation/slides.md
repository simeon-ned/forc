## Dynamics convention

$$M(q)\dot v+c(q,v)=\tau+J^\top f$$

Bias, actuator force, passive force, and constraint force have distinct roles.

---
## State and derived quantities

qpos and qvel store state.

mj_forward refreshes derived quantities.

mj_step advances time.

---
## Controller timing

Physics: 500 Hz

Controller: 100 Hz

Five physics steps share one held command.

---
## Force-balance diagnostic

$$r=M\dot v+c-\tau-J^\top f$$

A small residual checks internal consistency.

Physical accuracy needs measurements or an independent model.

---
## MJX experiment

Use identical models and initial states.

Separate compilation from execution.

Synchronize before stopping the timer.

---
## Exercise

Halve the physics timestep while keeping the controller rate fixed.

Which setting must change?

Reference: [@mujoco-api] [@mujoco-mjx]
