## Final project: a defensible robot-control comparison

**Weight:** 30%. **Format:** small engine-based study and demonstration. The project is the course exam.

Choose one question with a clear baseline: constrained flight, payload-robust arm tracking, cart-pole recovery, imitation under observation noise, or contact-aware control. Agree the scope with the instructor before using a large robot or expensive training system.

### Milestones

1. **Proposal:** one-page task definition, assumptions, baseline, candidate method, and measurable success criteria.
2. **Baseline checkpoint:** a clean-run command reproduces one working result. Include units, frames, actuator semantics, and loop rates.
3. **Comparison:** paired nominal and perturbed trials, at least one ablation, and explicit failure conditions.
4. **Handoff:** code, environment lock, YAML configuration, metrics, figures, and a short demonstration.

### Report structure

Explain the physical model and controller interfaces. State which decisions are model-based and which learned. Describe the evaluation distribution before showing results. Include a failure case and separate evidence from speculation. Discuss simulation fidelity and what additional testing would be needed for hardware.

### Rubric

- 20%: coherent question, assumptions, and baseline.
- 25%: correct controller implementation.
- 25%: controlled experiments and reproducible evidence.
- 20%: interpretation, limitations, and failure analysis.
- 10%: clear presentation and reusable repository organization.

A sophisticated method is not automatically a stronger project. A small experiment with correct conventions and a defensible conclusion can earn full credit. The Practice 7 harness is only a starting baseline and does not supply the assessed contribution.
