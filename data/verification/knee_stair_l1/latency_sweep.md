# Latency jitter sweep

The horizontal axis is the upper end of a uniform delay, in milliseconds, applied to the intent and sensor before the safety supervisor.
It is not the fixed simulation-loop latency, and torque is not delayed again after the supervisor.

Top panel, from the top of the legend order in the generator:
- blue: admittance human-torque RMS reduction, Nm
- green: AssistPolicy human-torque RMS reduction, Nm
- light blue: admittance positive-work reduction, J
- dark green: AssistPolicy positive-work reduction, J

Bottom panel:
- orange: admittance peak strap torque, Nm
- brown: AssistPolicy peak strap torque, Nm

Steady supervisor state first leaves NORMAL at 100 ms: {'off': 'SAFE_HOLD', 'admittance': 'SAFE_HOLD', 'assist_policy': 'SAFE_HOLD'}.

Admittance's positive-work reduction stays below AssistPolicy across this sweep. That ordering is reported as it is.
