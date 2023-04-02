## **Home Work 2: Nonlinear Control: Inverse Dynamics, Robust and Adaptive control**

### **Vessel on the River**

Consider a folowing vessel on the river:

<p align="center">
<img src="https://blog.arduino.cc/wp-content/uploads/2021/11/DawveRf-1024x683.jpeg" alt="drawing" width="35%" style="margin:auto"/>
</p>




With differential kinematics described as follows:

$$
\dot{\mathbf{x}} =
\begin{bmatrix}
\dot{x}
\\
\dot{y}
\\
\dot{\theta}
\end{bmatrix}
=\begin{bmatrix}
 \cos \theta & -\sin \theta & 0\\
 \sin \theta & \cos \theta & 0\\
 0 & 0 & 1\\
\end{bmatrix}
\begin{bmatrix}
v_\tau
\\
v_n
\\
\omega
\end{bmatrix} 
+
\begin{bmatrix}
\delta \\
0 \\
0\\
\end{bmatrix}
= \mathbf{R}(\theta)\mathbf{u} + \boldsymbol{\delta}(t)
$$

where $x, y$ are cartesian coordinates w.r.t global frame, $\theta$ heading angle, $0<\delta<\delta_{max}$ river flow $v_\tau, v_n, \omega$ are linear and angular velocity of vessel w.r.t. local coordinate frame.

<p align="center">
<img src="https://drive.google.com/uc?id=14Y5dheMte0hTe7MGCLE143wXmgJx96NR" alt="drawing" hspace="300px" width="50%" style="margin:auto"/>
</p>




---

The goal is to design feedback controller $\mathbf{u}(\mathbf{x})$ to regulate and track given trajectories even in the presense of river flow:
* **[45 points]** Inverse dynamics:
  * Assuming for now that river flow $\delta = 0$, propose the inverse dynamics control that regulate vessel to the desired state 
  * Modify controller to track time varying trajectories, take for instance $x_d = 10\cos t/5, y_d = 10\sin t/5, \theta_d = \pi/2 + t/5$
  * Tune the controller such that you will have critically damped response and less controll effort in the $n$ direction.
  * Simulate the response for both tracking and regulation. 
 )
* **[30 points]** Flow disturbance and sliding mode:
  * Suppose now that there is disturbance due to river flow $\delta < 1$ (as example you may take $\delta(t) = 2/3+\sin(0.1t)/3$, introduce this to the dynamics and study response of inverse dynamics controller.
  * Use sliding mode technique to propose the modification of the inverse dynamics controller that will eleminate the effect of the disturbance. Simulate the response.
  * Tune controller such that convergence to the boundary layer given by $\|\tilde{\mathbf{x}}\| < 0.1$ is achieved
* **[25 points]** Adaptive control:
  * Assume now that bounds on the river flow is not known in advance, propose the adaptive control that will estimate the unknown disturbance while tracking the same trajectories
  * Simulate response and show the convergence of tracking error and flow estimate $\hat{\delta}$ 
---

**[BONUS]** Overactuated vessel and Input mapping

In practice the actual inputs are not the linear and angular velocities of the cessel. For instance consider the case of four thrusters installed on the vessel such that the mapping between thrusters and vessel velocity is given by:

$$
\mathbf{u} = \begin{bmatrix}
v_\tau \\
v_n \\
\omega
\end{bmatrix} = 
\begin{bmatrix}
 -\cos \phi & -\cos \phi & 1 & 1\\
 \sin \phi & \sin \phi & 0 & 0\\
 -\ell_2\cos \phi  + \ell_1\sin \phi &  \ell_2\cos \phi  - \ell_1\sin \phi & -\ell & \ell\\
\end{bmatrix} 
\begin{bmatrix}
v_1 \\
v_2 \\
v_3 \\
v_4
\end{bmatrix} = \mathbf{H}(\phi)\mathbf{u}^*
$$

* Modify system dynamics, and either adaptive or robust controller and find appropriate $\mathbf{u}^*$ such that boat will have the same response as if you control $\mathbf{u}$ directly.
* Assume now that the $v_1, v_2$ are twice less powerfull then $v_3, v_4$. How would you modify your controller to tackle this 
* Introduce the trajectory planner to your controller such that vessel will favour the velocity along the $\tau$ direction. (read notes below)
 


**[NOTES] Feasible trajectory via differential flatness**

In many mobile robots including the vessels we do favour the motion that are directed along the $\tau$ component of robot, while the $n$ component remain zero or small. However the resulting trajectory should still be still feasible for our robot. 

Let us set $v_n = 0$, thus arriving to following expression:
$$
\dot{\mathbf{x}} =
\begin{bmatrix}
\dot{x}
\\
\dot{y}
\\
\dot{\theta}
\end{bmatrix}
=\begin{bmatrix}
 v_\tau \cos \theta  \\
 v_\tau \sin \theta \\
 \omega\\
\end{bmatrix}
$$
Here we neglects the disturbance effects.

Assume that we have desired position in task space $x(t)$, $y(t)$, the question is can we calculate the reminding $\theta,v,\omega$ as a function of $x,y$ and it's derivatives. 
First we take the squares of first and second terms yields:

$$
v_\tau = \eta\sqrt{\dot{x}^2 + \dot{y}^2} 
$$
here $\eta$ is 
the division of the first and second yields:

$$
\theta =\operatorname {atan2} (\eta \dot{y}, \eta \dot{x})
$$

Taking the derivatives of first and second equation after a bit of algebra yields: 
$$
\omega = \eta\frac{\dot{x}\ddot{y}-\dot{y}\ddot{x}}{v_\tau^2}
$$

Not how the all signals are given as **algebraic functions** of $x, y, \dot{x}, \dot{y}, \ddot{x}, \ddot{y}$. Thus once the desired trajectory is given for the $x, y$, one can automatically recover the open loop controller $\omega, v$ and remaining state $\theta$, this property is known as differential flatness and grately simplify the trajectory planning.
