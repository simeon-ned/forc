## The control problem as an MDP

A Markov decision process specifies state, action, transition, reward, and termination. A policy maximizes expected discounted return,
$$J(\pi)=\mathbb E_\pi\left[\sum_{k=0}^{T-1}\gamma^kr_k\right].$$
This is still a feedback-control problem. The learning algorithm changes how the policy is obtained and what model information it requires.

A reward $r=-\ell$ turns a cost into a learning objective, but its meaning depends on timestep and discount. If $\ell$ is a continuous cost rate, use approximately $r_k=-\Delta t\,\ell_k$ when comparing different update rates. Otherwise changing simulation frequency also changes the optimization objective.

## Bellman backup

For a tabular action-value estimate,
$$Q(s,a)\leftarrow Q(s,a)+\alpha\left[r+
\gamma\max_{a'}Q(s',a')-Q(s,a)\right].$$
At a true terminal state, omit the bootstrap term. A training time limit is different: if the underlying task continues and time is not part of the state, treating truncation as terminal biases the value target. A finite-horizon task can instead include remaining time in its state.

The practice uses a deliberately small finite-state regulation problem. Q-learning can then be implemented and inspected directly, without a neural network or an unverified claim of learning robot locomotion.

## Worked update

Take $Q(s,a)=2$, reward $r=-1$, $\gamma=0.9$, maximum next value 3, and learning rate $\alpha=0.2$. The target is $-1+0.9(3)=1.7$. The new estimate is $2+0.2(1.7-2)=1.94$. For a terminal transition the target is instead $-1$, giving 1.4.

This difference is large enough to affect behavior near resets. Unit-test terminal transitions before interpreting a reward curve.

## Exploration and evaluation

An epsilon-greedy policy explores during training. Evaluation freezes the policy and disables exploratory action noise unless robustness to that noise is explicitly being tested. Keep independent random generators for training and final evaluation. Compare with a hand-designed controller and a random policy under the same task definition.

Report success, state error, constraint violations, environment transitions, and wall time. A high return can reflect a poorly specified reward. For example, a robot rewarded only for staying alive may stand still when the intended task is locomotion.

## Extension to robots

Continuous state and action spaces motivate function approximation, actor-critic methods, and policy-gradient methods. Batched simulation accelerates data collection, but does not resolve observation design or sim-to-real mismatch. These are extension topics, not capabilities claimed by the tabular baseline.

**Exercise:** Add a reward bonus for moving quickly and inspect whether the policy overshoots its target. Explain the outcome using the objective, rather than calling it a training bug. The CMU and MIT sources provide the next level of optimal-control and learning context [@cmu-control] [@mit-underactuated].
