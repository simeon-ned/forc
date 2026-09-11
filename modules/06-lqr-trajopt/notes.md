## Local coordinates around a reference

Let $(q^\star,v^\star,u^\star)$ be an equilibrium. Construct a small error $\delta x=(\delta q,\delta v)$ with $\delta q$ in the nv-dimensional configuration tangent space. For a free body, this gives twelve error coordinates rather than thirteen raw position-plus-velocity entries. Actuator activation adds na more entries when present.

Linearizing the actual sampled dynamics gives
$$\delta x_{k+1}=A\delta x_k+B\delta u_k.$$
MuJoCo's [mjd_transitionFD](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mjd-transitionfd) provides discrete transition derivatives for supported integrators. The teaching examples use Euler and direct actuators. Check the nominal step first: a non-equilibrium reference creates an affine residual that this equation otherwise hides.

## Discrete LQR

Minimize the infinite-horizon cost
$$\sum_{k=0}^\infty(\delta x_k^\top Q\delta x_k+\delta u_k^\top R\delta u_k).$$
With $R\succ0$ and the usual stabilizability/detectability assumptions, the stabilizing Riccati solution satisfies
$$P=Q+A^\top PA-A^\top PB(R+B^\top PB)^{-1}B^\top PA.$$
Then $K=(R+B^\top PB)^{-1}B^\top PA$ and $\delta u=-K\delta x$. Use a linear solve. Check the eigenvalues of $A-BK$ and test nonlinear rollouts from several initial errors. A Riccati solver returning an array is not itself a validation.

## Worked scalar problem

For $x_{k+1}=x_k+u_k$ with $Q=R=1$, the Riccati equation reduces to $P^2-P-1=0$. The positive solution is $(1+\sqrt5)/2$. Therefore $K=P/(1+P)\approx0.618$, and the closed-loop multiplier is about 0.382. This small case provides a unit test for the implementation.

## Cart-pole and quadrotor

Practice 3 linearizes a cart-pole at its upright equilibrium and a full free-joint quadrotor at hover. Both use the same feedback pipeline. Quadrotor orientation error uses MuJoCo's quaternion convention and tangent difference. Thrust limits introduce nonlinear behavior that LQR does not enforce during design. Large initial tilts require a different reference or nonlinear planning.

## Beyond equilibrium

For a trajectory, linearize along $(x_k^\star,u_k^\star)$ and run a finite-horizon Riccati recursion backward to obtain time-varying gains. To find the reference itself, optimize controls by shooting through the dynamics or optimize both states and controls with transcription constraints. CMU's index and MIT's notes develop these directions [@cmu-index] [@mit-trajopt].

**Exercise:** Double a position coordinate's numerical scale without changing its physical meaning. How should its diagonal Q entry change? Divide it by four to preserve the same cost. This is why units and normalization belong in the experiment configuration.
