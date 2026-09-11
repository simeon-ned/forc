## Optimizing with rollouts

A derivative-free predictive controller can evaluate many candidate input sequences by simulation. Start with a nominal sequence $U=(u_0,\ldots,u_{N-1})$. Sample perturbations $\epsilon^{(i)}$, roll out $U+\epsilon^{(i)}$, and score each trajectory with $S_i$.

A common weighted update uses
$$w_i=\frac{\exp(-(S_i-\min_j S_j)/\lambda)}
{\sum_j\exp(-(S_j-\min_\ell S_\ell)/\lambda)},\qquad
U\leftarrow U+\sum_iw_i\epsilon^{(i)}.$$
Subtracting the smallest score improves numerical stability without changing normalized weights. A small temperature $\lambda$ concentrates weight on a few candidates. A large temperature averages broadly and may barely improve the plan.

## What the baseline does and does not claim

Our runnable exercise is an MPPI-inspired weighted stochastic-shooting baseline. Formal MPPI derivations impose a relationship between stochastic dynamics, control cost, and sampling distributions, and can require importance-sampling correction terms. We do not present the simplified update as that complete derivation.

When bounds clip a candidate, compute the update from the *actual candidate minus nominal sequence*. Otherwise the weighted perturbation can point toward an input that was never evaluated. Shift the sequence after applying its first command and warm-start with a reasonable terminal command.

## Worked two-candidate update

Suppose $S_1=2$, $S_2=4$, $\lambda=1$, and the first-input perturbations are $+1$ and $-1$. The weights are approximately 0.881 and 0.119. The first nominal input changes by about $+0.762$. With $\lambda$ near zero, the update approaches the best sample; with very large $\lambda$, it approaches zero in this symmetric example.

Effective sample size, $1/\sum_iw_i^2$, helps diagnose whether only one rollout dominates. It is an optimization diagnostic, not a confidence interval.

## Constraint handling and computation

Clipping controls enforces input bounds. A large cost for penetrating an obstacle only discourages penetration; it does not establish hard feasibility. Sampling may miss narrow feasible regions, especially with long horizons and independent noise at every time step. Correlated noise, a good nominal trajectory, or a reduced control basis can improve search.

Batching compatible rollouts with MJX is an optional acceleration route [@mujoco-mjx]. Keep the same sample count and seeds when comparing backends. GPU throughput and single-rollout latency answer different questions.

## Practice and exercise

Practice 4 compares constrained MPC, clipped LQR, and weighted shooting on the same vertical-flight task. Record seed sensitivity as well as tracking and violations. Increase sample count while holding the control period fixed and measure deadline overruns.

**Check:** If every candidate has nearly identical cost, what is the update? Approximately the average perturbation, often near zero for symmetric noise. This can mean a flat local objective, too little exploration, or a cost bug. Inspect trajectories before merely lowering the temperature.
