## Aim

Start a project from a known working robot baseline, then investigate one bounded question.

**Time:** about 120 minutes. Start with the linked lectures and the course notation contract. The reference program is complete and runnable; your work is to explain it, modify it, and test the results.

## Run from this repository

~~~bash
uv sync --locked
uv run --locked python practices/p07-project/run.py
~~~

The program writes configuration, MuJoCo version, and metrics to outputs/p07-project/metrics.json. Experiments with a time history also write trajectory.csv. These are generated results and are not source material.

Use a separate configuration and output directory for each variation:

~~~bash
uv run --locked python practices/p07-project/run.py --config practices/p07-project/config.yml --output outputs/p07-project/baseline
~~~

Keep the original configuration as the baseline. Record all changes in your report; avoid changing several physical and numerical parameters at once.

## Tasks

1. Run this harness to reproduce the cart-pole and quadrotor LQR baseline from Practice 3.
2. Choose one task and define success, failure, actuator limits, and evaluation conditions before modification.
3. Implement one controller change and one ablation. Keep paired test cases.
4. Submit the final project brief's report, commands, configuration, metrics, and a short demonstration.

## Expected checks

The supplied harness is a starting baseline, not a completed project. Your assessed contribution must answer a new question and include failures and limitations.

## Submission

Submit your YAML configuration, a short explanation of the equations and assumptions, labeled plots or numerical comparisons with units, and a paragraph explaining a failure or limitation. Include the exact run command. The weekly practice rubric awards 40% for correctness and checks, 30% for the controlled comparison, 20% for interpretation, and 10% for reproducibility.

## Extension

Use a contact-rich robot only after a small CPU experiment runs reliably. Simulator success alone does not establish hardware safety.

## Reading

[@mit-underactuated] · [@mit-manipulation]
