# Latency jitter sweep

The horizontal axis is the upper end of a uniform command-hold delay, in milliseconds.
This is injected delay. It is not the fixed simulation-loop latency.

Top panel, from the top of the legend order in the generator:
- blue: admittance human-torque RMS reduction, Nm
- green: AssistPolicy human-torque RMS reduction, Nm
- light blue: admittance positive-work reduction, J
- dark green: AssistPolicy positive-work reduction, J

Bottom panel:
- orange: admittance peak strap torque, Nm
- brown: AssistPolicy peak strap torque, Nm

No delay from 0 to 50 ms made assist worse than OFF or left the supervisor in a degraded state.

Admittance's positive-work reduction stays below AssistPolicy across this sweep. That ordering is reported as it is.
