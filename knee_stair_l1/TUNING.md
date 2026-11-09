# Tuning record, 30-seed round

No assist-controller gain was changed. Admittance mass, damping and stiffness, and the PID gains including the 24 Nm limit, are the values from the previous round. Positive-work reduction was not a tuning target. Admittance's smaller positive-work reduction is left as it is.

Seeds 1000–1019 are not in the reported set 0–29. They were used only to check that the new draws do not hit a hard stop and that admittance saturation stays at 0 before the reported seeds were run. That probe had 0 hard-stop contacts and admittance saturation fraction 0. No parameter was changed after the probe.

The flexion stop is 2.2 rad. The previous 1.55 rad stop had been placed just above an earlier non-randomized trajectory. The wider stop is a model-range assumption, about 126 degrees, so a heavier drawn body and a higher step are not clipped by that old wall. It was set before the reported 0–29 run.

A separate probe on seeds 1000–1005 extended the command-hold delay to 1.5 s. Human-torque RMS reduction stayed positive and the supervisor stayed in NORMAL. No controller change followed from that probe. The committed sweep itself stops at 50 ms, which is the requested range.
