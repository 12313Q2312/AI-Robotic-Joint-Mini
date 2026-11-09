# Layer 1 paired metrics

Experimental simulation. Synthetic intent. Not a human, EMG, or fall-prevention result.

Seeds: 30 (0..29). Sample std (n-1). 95% CI is the Student t interval.

Admittance has a smaller positive-work reduction than AssistPolicy. That ordering is a result of this run, not a tuning target.

Latency p50/p95/max below are simulation loop latency, not real-system latency.

Comfort threshold: 15.0 Nm. Design assumption only, not a comfort study: 15 Nm is 0.08 m cuff moment arm times 187.5 N strap force. Peaks at or above this value fail the interaction criterion.

Load: Theoretical stair-support moment, not a gait-lab inverse-dynamics result. Body mass is drawn from 50-85 kg around the 70 kg synthetic adult used as the nominal anchor. knee_weight_share of that weight loads this knee. The moment arm is moment_arm_m * (step_height / reference_step_height_m) * sin(flexion). reference_step_height_m is 0.15 m, the middle of the 0.13-0.18 m draw, a round riser height rather than a measured step. The moment is zero at a straight knee and is a flexion moment.

The censored timing-error metric is not used.

## Primary reductions versus OFF (OFF − assisted), mean ± std, 95% CI

| Metric | admittance | admittance 95% CI | AssistPolicy baseline | AssistPolicy 95% CI |
|---|---:|---:|---:|---:|
| human_torque_rms_nm | 4.458133 ± 0.963660 | [4.098296, 4.817969] | 4.475351 ± 0.923152 | [4.130641, 4.820062] |
| human_positive_work_j | 1.792768 ± 0.446677 | [1.625976, 1.959560] | 2.112837 ± 0.536153 | [1.912634, 2.313040] |

## Secondary metrics, mean ± std

| Metric | OFF | admittance | AssistPolicy baseline |
|---|---:|---:|---:|
| tracking_error_rms_rad | 0.279325 ± 0.055248 | 0.157478 ± 0.029354 | 0.158177 ± 0.030331 |
| peak_interaction_torque_nm | 0.984658 ± 0.190896 | 5.182988 ± 1.042536 | 5.151859 ± 0.935798 |
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
- human_torque_rms_reduction_ci95: [4.098295872604, 4.817969173633]
- tracking_error_no_more_than_10_percent_worse: True
- tracking_error_ratio_max: 0.596925457078
- hard_stop_contacts_zero: True
- peak_interaction_below_comfort: True
- comfort_peak_interaction_nm: 15.0
- saturation_time_fraction_mean: 0.0
- saturation_time_fraction_below_5_percent: True

### assist_policy

- human_torque_rms_reduction_ci_excludes_0: True
- human_torque_rms_reduction_ci95: [4.130640500984, 4.820061826509]
- tracking_error_no_more_than_10_percent_worse: True
- tracking_error_ratio_max: 0.574553667053
- hard_stop_contacts_zero: True
- peak_interaction_below_comfort: True
- comfort_peak_interaction_nm: 15.0
- saturation_time_fraction_mean: 0.0
- saturation_time_fraction_below_5_percent: True
