## A local map from joints to task motion

Let $p(q)\in\mathbb R^3$ be the position of a tool point in the world frame. Its velocity is $\dot p=J_p(q)v$, where $J_p$ has three rows and nv columns. This relation is local. It does not choose a globally collision-free path, and it says nothing about available torque.

For a position target, choose a task velocity $b=\dot p^\star+K_p(p^\star-p)$. The ideal task error satisfies $\dot e=-K_pe$ if $J_pv=b$ can be realized exactly. Rank deficiency, joint limits, and downstream tracking error all break this idealization.

## Damped least squares

Choose velocity by minimizing
$$\frac12\|J_pv-b\|^2+\frac{\lambda^2}{2}\|v\|^2.$$
The normal equations give $(J_p^\top J_p+\lambda^2I)v=J_p^\top b$. Equivalently,
$$v=J_p^\top(J_pJ_p^\top+\lambda^2I)^{-1}b.$$
Implement a linear solve rather than an explicit matrix inverse. Damping limits amplification along small singular values, at the price of task error. It does not enforce a hard speed bound. Clip only for a simple baseline; a constrained least-squares problem provides a better interpretation when limits matter.

For redundancy, a nullspace term can bias the posture. With a damped inverse the resulting projector is approximate, so the secondary motion can leak into the primary task.

## Worked planar arm

For two links of length one,
$$p=\begin{bmatrix}\cos q_1+\cos(q_1+q_2)\\
\sin q_1+\sin(q_1+q_2)\end{bmatrix}.$$
At $q=(0,0)$,
$$J_p=\begin{bmatrix}0&0\\2&1\end{bmatrix}.$$
The arm can initially move the tip vertically, but cannot produce a first-order horizontal velocity. A target that requires only horizontal motion gives zero local descent direction at this exact configuration. Damping prevents enormous commands, but cannot create a missing direction. Start away from the singularity or plan a finite motion.

## MuJoCo implementation

Use a named site at the tool. [mj_jacSite](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-jacsite) provides separate translational and rotational Jacobians. Match their frame to the task error. Check one column by perturbing the configuration with mj_integratePos, refreshing kinematics, and comparing the tool displacement to the predicted value.

Orientation errors belong in a three-dimensional tangent space. Subtracting four quaternion components produces a representation-dependent error and treats opposite signs of the same orientation as different poses.

## Practice and discussion

Practice 2 first moves the arm kinematically, then uses dynamics-aware torque control. Compare the final task error and maximum joint speed for several damping values. Read the manipulation notes for extensions to differential IK with constraints [@mit-pick].

**Check:** If $J_p$ is $3\times7$, how many tool-position constraints can it satisfy independently? At most three, and fewer at a singular configuration. The remaining velocity directions need an additional selection rule.
