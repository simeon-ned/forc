---
layout: course.njk
title: MuJoCo notation and API contract
permalink: /notation/
---

## One convention across both independent courses

The executable baseline is **MuJoCo 3.13.0**, fixed in pyproject.toml and uv.lock. The Python baseline is 3.12. Links to stable documentation may advance; use the version selector when comparing with these examples. SI units and radians are the default. Use $\Delta t$ for timestep, so h cannot be confused with a bias-force symbol.

## Equations and fields

$$M(q)\dot v+c(q,v)=\tau+J^\top f.$$

- $q$: configuration, stored in **data.qpos**, length **model.nq**.
- $v$: generalized velocity, **data.qvel**, length **model.nv**. In general $\dot q\ne v$.
- $\dot v$: **data.qacc**, length nv.
- $M(q)$: generalized inertia, nv by nv. Reconstruct with **mj_fullM(model, data, M)** in the tested version. Internal storage is not a dense array to reshape.
- $c(q,v)$: **data.qfrc_bias**, combining Coriolis, centrifugal, and gravitational bias.
- $h(q,v)$: an alternative textbook name for the same bias vector in this course. It is **not an additional force**. A decomposition $h=Cv+g$ is possible, but C itself is not unique.
- $\tau$: total nonconstraint generalized force. Separate **qfrc_actuator**, **qfrc_passive**, and **qfrc_applied**. Cartesian loads in **xfrc_applied** also need their generalized-force mapping.
- $J^\top f$: engine constraint contribution, **qfrc_constraint**. Constraint-space data include **efc_J** and **efc_force**, with storage and active rows determined by the model/solver.

The simple force-residual practices deliberately leave xfrc_applied zero. Do not reuse that abbreviated residual for a model with nonzero Cartesian applied loads without including their mapping. **ctrl** has length nu and is an actuator input, not generally an nv-dimensional torque vector. **act** stores optional activation state.

Reference: [MuJoCo computation](https://mujoco.readthedocs.io/en/stable/computation/index.html).

## Quaternions and frames

Use scalar-first **(w, x, y, z)**. Identity is **(1, 0, 0, 0)**. We write Q for one quaternion and q for the whole configuration. Q represents a body-to-world active rotation: $r^W=R_{WB}r^B$. Hamilton multiplication composes rotations, and body-frame angular velocity updates on the right.

A positive $90^\circ$ z rotation has Q = $(\sqrt2/2,0,0,\sqrt2/2)$ and sends x to y. Test this at every boundary to scalar-last software. Q and −Q represent the same orientation.

For a free joint, configuration has **[px, py, pz, qw, qx, qy, qz]**. Velocity has **[vx, vy, vz, wx, wy, wz]**: translational velocity in the world frame and angular velocity in the local body frame. Querying other spatial quantities can use different origins or component order. In particular, mj_objectVelocity returns a rotational-then-linear spatial vector in the selected orientation frame.

## Tangent operations

Use [mj_integratePos](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-integratepos) for full-configuration updates and [mj_differentiatePos](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-differentiatepos) for differences. Calling differentiatePos with dt=1 gives a local configuration displacement, not a measured velocity over one second.

~~~python
dq = np.empty(model.nv)
mujoco.mj_differentiatePos(model, dq, 1.0, q_reference, data.qpos)
error = np.concatenate([dq, data.qvel - v_reference])
~~~

For direct actuators without activation, the local dynamics state has 2 nv coordinates. With activation, it has 2 nv + na. [mjd_transitionFD](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mjd-transitionfd) uses these tangent dimensions. The teaching examples use Euler; this derivative API does not support RK4 in the referenced version.

## Function connections

- Refresh a state without integrating: [mj_forward](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-forward).
- Advance physics: [mj_step](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-step).
- Dense inertia for small examples: [mj_fullM](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-fullm).
- Tool Jacobians: [mj_jacSite](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-jacsite).
- Requested-acceleration force query: [mj_inverse](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-inverse). Set qacc first and interpret passive/constraint contributions.
- Contact-frame wrench: [mj_contactForce](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-contactforce).
- Object spatial velocity: [mj_objectVelocity](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-objectvelocity).

Always refresh derived quantities after editing a state before reading them.

## MJX boundary

The optional example selects **MJX-JAX** explicitly. Functional data updates, jit, and vmap differ from mutable CPU MuJoCo. Synchronize device outputs when timing. **MJX-Warp is a different backend with different support; do not assume JAX autodiff guarantees apply to it.** Read the [MJX feature documentation](https://mujoco.readthedocs.io/en/stable/mjx.html) before extending a model.
