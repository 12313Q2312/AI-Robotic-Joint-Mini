# Layer 1 paired metrics

Experimental simulation results. Synthetic intent. Not a human, EMG, or fall-prevention result.

Seeds: 12 (0..11). Sample std (n-1).

## Absolute, mean ± std

| Metric | OFF | admittance | AssistPolicy baseline |
|---|---:|---:|---:|
| assist_timing_error_s | 1.072917 ± 0.003965 | 0.336667 ± 0.030476 | 0.015417 ± 0.002575 |
| end_to_end_latency_s | 0.010000 ± 0.000000 | 0.010000 ± 0.000000 | 0.010000 ± 0.000000 |
| peak_interaction_torque_nm | 0.000000 ± 0.000000 | 24.000000 ± 0.000000 | 39.561140 ± 1.520257 |
| saturation_limit_trigger_count | 0.000000 ± 0.000000 | 26.416667 ± 0.514929 | 42.916667 ± 25.706237 |

## Paired difference versus OFF, mean ± std

| Metric | admittance − OFF | AssistPolicy baseline − OFF |
|---|---:|---:|
| assist_timing_error_s | -0.736250 ± 0.033920 | -1.057500 ± 0.005000 |
| end_to_end_latency_s | 0.000000 ± 0.000000 | 0.000000 ± 0.000000 |
| peak_interaction_torque_nm | 24.000000 ± 0.000000 | 39.561140 ± 1.520257 |
| saturation_limit_trigger_count | 26.416667 ± 0.514929 | 42.916667 ± 25.706237 |

Timing for OFF is censored to the horizon because assist torque stays at 0.
Latency is the configured transport delay and is shared by every condition.
