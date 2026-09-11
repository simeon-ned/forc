## Robot-control architecture

Task planning supplies references.

Estimation supplies state information.

Feedback and the actuator interface determine the applied forces.

---
## Contact and authority

Flight and stance permit different forces.

Friction and unilateral contact constrain ground reaction forces.

---
## Action-space comparison

Joint targets include a downstream servo.

Torque commands expose a different interface.

Controller comparisons must include that distinction.

---
## Timing example

Policy: 50 Hz

PD servo: 1 kHz

Moving PD to 50 Hz changes the system even with identical policy weights.

---
## Project evidence

A baseline, paired evaluation cases, an ablation, and failure analysis.

Configuration and numerical outputs accompany the demonstration.

---
## Final question

Which conclusions follow from your experiments?

Which claims would require a different robot, distribution, or physical test?

Reading: [@mit-underactuated] [@mit-manipulation]
