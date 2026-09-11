## Markov decision process

State, action, transition, reward, termination.

$$J(\pi)=\mathbb E_\pi\sum_k\gamma^kr_k$$

The task definition determines what the policy learns.

---
## Q-learning

$$Q\leftarrow Q+\alpha(r+\gamma\max_{a'}Q(s',a')-Q)$$

True terminal states omit the bootstrap term.

---
## Numerical update

$Q=2$, $r=-1$, $\gamma=0.9$, $\max Q'=3$, $\alpha=0.2$

New value: 1.94

With terminal transition: 1.4

---
## Training and evaluation

Training explores.

Evaluation freezes the policy and uses independent episodes.

---
## Reward design

High return can coexist with unwanted behavior.

Report task success and violations alongside return.

---
## Scope of the practice

A tabular regulation task makes the learning loop inspectable.

Continuous robot policies are the extension.
