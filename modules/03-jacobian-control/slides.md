## Task velocity

$$\dot p=J_p(q)v$$

The Jacobian maps generalized velocity to tool velocity in a declared frame.

---
## Differential IK

$$b=\dot p^\star+K_p(p^\star-p)$$

$$v=J_p^\top(J_pJ_p^\top+\lambda^2I)^{-1}b$$

Damping trades tracking accuracy for smaller commands.

---
## A straight two-link arm

$$J_p(0,0)=\begin{bmatrix}0&0\\2&1\end{bmatrix}$$

Vertical motion is locally available.

Horizontal motion needs a change of configuration.

---
## Limits

Damping does not impose a speed constraint.

Joint limits and collisions require explicit treatment.

---
## MuJoCo connection

mj_jacSite gives translational and rotational blocks.

mj_integratePos provides configuration perturbations for finite differences.

---
## Practice question

What happens to task error as damping grows?

Compare speed, error, and the smallest singular value.

Reading: [@mit-pick]
