# Layer 1 paired metrics

Experimental simulation. Synthetic intent. Not a human, EMG, or fall-prevention result.

Seeds: 12 (0..11). Sample std (n-1). 95% CI is the Student t interval.

Comfort threshold: 15.0 Nm. Design assumption only, not a comfort study: 15 Nm is 0.08 m cuff moment arm times 187.5 N strap force. Peaks at or above this value fail the interaction criterion.

Load: Theoretical stair-support moment. A synthetic 70 kg body, not a measured person, places knee_weight_share of its weight on this knee. The moment arm is moment_arm_m*sin(flexion), zero when the knee is straight and moment_arm_m at 90 degrees. The load is a flexion moment. It is not an inverse-dynamics result from a gait lab.

The censored timing-error metric is not used.

## Primary reductions versus OFF (OFF − assisted), mean ± std, 95% CI

| Metric | admittance | admittance 95% CI | AssistPolicy baseline | AssistPolicy 95% CI |
|---|---:|---:|---:|---:|
| human_torque_rms_nm | 4.364935 ± 0.046957 | [4.335100, 4.394770] | 4.301118 ± 0.020974 | [4.287791, 4.314444] |
| human_positive_work_j | 1.735550 ± 0.137519 | [1.648174, 1.822925] | 1.993925 ± 0.128890 | [1.912032, 2.075818] |

## Secondary metrics, mean ± std

| Metric | OFF | admittance | AssistPolicy baseline |
|---|---:|---:|---:|
| tracking_error_rms_rad | 0.268971 ± 0.001786 | 0.149790 ± 0.000534 | 0.152643 ± 0.001242 |
| peak_interaction_torque_nm | 0.964905 ± 0.000000 | 5.064853 ± 0.014566 | 4.907122 ± 0.000000 |
| saturation_time_fraction | 0.000000 ± 0.000000 | 0.000000 ± 0.000000 | 0.000000 ± 0.000000 |
| hard_stop_contact_count | 0.000000 ± 0.000000 | 0.000000 ± 0.000000 | 0.000000 ± 0.000000 |

## Measured latency, pooled over seeds (command time − intent time)

| Condition | p50 s | p95 s | max s |
|---|---:|---:|---:|
| off | 0.010000 | 0.010000 | 0.010000 |
| admittance | 0.010000 | 0.010000 | 0.010000 |
| assist_policy | 0.010000 | 0.010000 | 0.010000 |

## Criteria

### admittance

- human_torque_rms_reduction_ci_excludes_0: True
- human_torque_rms_reduction_ci95: [4.335099688801, 4.394769870796]
- tracking_error_no_more_than_10_percent_worse: True
- tracking_error_ratio_max: 0.559924513643
- hard_stop_contacts_zero: True
- peak_interaction_below_comfort: True
- comfort_peak_interaction_nm: 15.0
- saturation_time_fraction_mean: 0.0
- saturation_time_fraction_below_5_percent: True

### assist_policy

- human_torque_rms_reduction_ci_excludes_0: True
- human_torque_rms_reduction_ci95: [4.287791168679, 4.314443913205]
- tracking_error_no_more_than_10_percent_worse: True
- tracking_error_ratio_max: 0.56863136088
- hard_stop_contacts_zero: True
- peak_interaction_below_comfort: True
- comfort_peak_interaction_nm: 15.0
- saturation_time_fraction_mean: 0.0
- saturation_time_fraction_below_5_percent: None
