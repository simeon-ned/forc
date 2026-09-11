## Candidate rollouts

Perturb a nominal control sequence.

Simulate each candidate with the same initial state.

Score the resulting trajectories.

---
## Weighted update

$$w_i\propto\exp(-(S_i-S_{\min})/\lambda)$$

$$U\leftarrow U+\sum_i w_i\epsilon^{(i)}$$

Temperature controls weight concentration.

---
## Two candidates

Costs: 2 and 4

Temperature: 1

Weights: 0.881 and 0.119

Perturbations $+1,-1$ give an update of about $+0.762$.

---
## Algorithm scope

The lab implements weighted stochastic shooting inspired by MPPI.

Formal MPPI needs additional assumptions and correction terms.

---
## Constraints and batching

Clipping enforces input bounds.

Penalty costs alone do not guarantee collision avoidance.

MJX can batch compatible rollouts.

---
## Experiment

Sweep temperature, sample count, and noise scale.

Record effective sample size and missed control deadlines.

Reference: [@mujoco-mjx]
