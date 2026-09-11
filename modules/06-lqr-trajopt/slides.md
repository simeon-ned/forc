## Tangent-state linearization

$$\delta x_{k+1}=A\delta x_k+B\delta u_k$$

A free body's local state error has 12 coordinates.

Raw quaternion subtraction gives the wrong representation.

---
## LQR objective

$$\sum_k \delta x_k^\top Q\delta x_k+\delta u_k^\top R\delta u_k$$

Weights encode units and priorities.

---
## Feedback gain

$$K=(R+B^\top PB)^{-1}B^\top PA$$

$$\delta u=-K\delta x$$

The eigenvalues of $A-BK$ provide a local discrete-time check.

---
## Scalar example

$x_{k+1}=x_k+u_k$, $Q=R=1$

$P\approx1.618$, $K\approx0.618$

Closed-loop multiplier: 0.382

---
## Robot examples

Cart-pole upright balance.

Quadrotor hover with scalar-first quaternions.

Large errors and saturation need nonlinear rollout tests.

---
## Trajectory optimization

Shooting optimizes controls through a rollout.

Transcription introduces states and dynamics constraints.

Reading: [@mit-lqr] [@mit-trajopt]
