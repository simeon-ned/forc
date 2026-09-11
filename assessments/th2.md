## TH2: learning a control policy

**Weight:** 25%. **Suggested release:** after Lecture 9. **Estimated effort:** 8–12 hours. Use Practices 5 and 6 as reference implementations, not as final results.

### Question

How do the training distribution, expert-query budget, and reward affect closed-loop performance?

### Required work

Choose a compact dynamical task with a declared state, observation, action space, and force limits. A pendulum or cart-pole is appropriate. A finite-state teaching task is acceptable only if you add a substantive new modeling or evaluation question.

1. Implement an expert and explain its operating region. Collect whole demonstration episodes and split by initial condition or episode.
2. Train a behavior-cloning student. Report held-out action error and independent closed-loop performance. Fit normalization on training data only.
3. Add DAgger with expert labels at student-visited states. Compare against BC under both equal episode count and a matched expert-label budget. Save intermediate policies and report nonmonotonic behavior.
4. Train a reward-based baseline appropriate to your chosen action space. Explain the return, exploration, discount, and treatment of true termination versus training truncation.
5. Evaluate all methods on the same nominal and perturbed test conditions. Use at least three training seeds and ten evaluation episodes per seed, or enumerate all states for a genuinely small finite MDP.

### Controls against misleading results

Do not give the deployed student simulator quantities absent from its declared observation. Do not tune on final evaluation episodes. Report failed episodes and constraint violations even if they receive high reward. Keep the lower-level servo and action clipping identical when comparing policies.

### Evidence and rubric

- 20%: task definition, expert, and data separation.
- 25%: BC and DAgger implementation with query accounting.
- 20%: RL formulation and terminal/discount correctness.
- 25%: paired evaluation, seed variation, and failure analysis.
- 10%: reproducibility and source attribution.

Submit code, configs, a dataset manifest, training/evaluation commands, metrics, and a report of at most six pages. For large datasets provide a documented retrieval route and checksum rather than committing generated data blindly. A claimed improvement must be visible in independent rollout evidence, not only training loss.
