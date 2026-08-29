---
layout: docs.njk
title: Dynamic control & LQR
subtitle: Topic 3 · Lectures 5–6 · TH1 opens
permalink: /topics/03-lqr/
---

## Goals

- Use inverse dynamics and gravity compensation; light energy / Lyapunov intuition.
- Design LQR; sketch trajectory optimization as a bridge to MPC / TH1.

## Lectures

| # | Title |
| --- | --- |
| **5** | Inverse dynamics & gravity compensation (+ light Lyapunov / energy) |
| **6** | LQR (+ trajectory-optimization sketch) |

Trajectory optimization is only sketched in lecture 6; it is done properly in **Take-home 1** with constrained MPC on a quadrotor.

## Practice & assessment

In-class lab on **cart-pole and quadrotor**: LQR for hover / tracking; clarify Take-home 1.

**TH1 — Optimal control** is released with this topic (quadrotor traj opt + constrained MPC; optional MPPI). Stub: [`hw/th1_mpc/`](https://github.com/simeon-ned/forc/tree/master/hw/th1_mpc).

## Next

[Receding horizon & sampling]({{ '/topics/04-mpc/' | url }}) (lectures 7–8)
