# 单侧膝关节上楼助力闭环

只在电脑里运行。没有真人，没有真实表面肌电，也没有防跌倒结果。

## 模块

参数都在 `config/default.json`。话题名在 `topics.py`。

| 模块 | 文件 | 合同 |
|---|---|---|
| 意图源 | `intent.py` | `/knee_stair/intent_synthetic`。v1 是合成期望轨迹，日志写明 synthetic |
| 有界人体力矩 | `human.py` | 跟踪上述轨迹，并被 `human.torque_limit_nm` 截断 |
| 助力 | `admittance.py`、`torque_pid.py`、`assist_policy.py`、`controller.py` | 期望运动和 `/knee_stair/assist_command_raw`。PID 上限保持 24 Nm |
| 安全监督 | `safety.py` | 唯一发出 `/knee_stair/assist_command_safe` 的对象，并发布只读的 `/joint/safety_state` |
| 植物 | `plant.py` | 人体小腿、外骨骼连杆、重力矩和弹簧阻尼绑带 |
| 遥测 | `telemetry.py`、`../scripts/run_knee_stair_l1_metrics.py` | 原始结果 `data/verification/knee_stair_l1/metrics_raw.json` |

## 负载、意图和交互

重力矩使膝关节屈曲：

`knee_weight_share * body_mass_kg * gravity_m_s2 * moment_arm_m * sin(flexion)`

伸直时力矩为 0，90° 时力臂等于 `moment_arm_m`。文字假设在配置的 `load.assumption`。这是 **theoretical**。

合成意图是一条平滑的屈膝下降轨迹，种子只改变下降幅度，而且终点离伸膝限位还有余量。人体力矩是 PD 跟踪并限幅。关闭助力时，这组参数不把膝推到限位；一旦碰到限位，植物记一条 `anomaly hard_stop_contact`。

绑带力矩 `K*(q_exo - q_human) + B*(qd_exo - qd_human)` 作用在人体上，反作用在外骨骼上。峰值交互力矩读的是这个约束，不是电机力矩。舒适阈值 15 Nm 的来源写在 `metrics.comfort_basis`：0.08 m 力臂乘 187.5 N 的设计假设，不是舒适性试验。

## 指标

同一种子的轨迹在关闭、导纳和 AssistPolicy 之间相同。标准差用 n−1。降幅是关闭减去助力。95% 区间是 df=11 的 t 区间。

主指标：人体膝关节力矩 RMS 的降幅，人体膝关节正功的降幅。

次指标：跟踪误差 RMS、绑带交互峰值、饱和时间比例、硬限位接触次数。

时延是每一步的力矩命令时间戳减去该命令所消费的意图时间戳，再取 p50、p95 和最大值。不再使用充数的时机误差。

通过标准，导纳和 AssistPolicy 都相对关闭来评：

- 人体力矩 RMS 降幅的 95% 区间下界大于 0
- 每个种子的跟踪误差不超过关闭时的 1.1 倍
- 硬限位接触次数为 0
- 交互峰值低于配置里的 15 Nm

导纳饱和时间比例还要低于 5%。做到这一点时没有提高 24 Nm 上限。结果无论通过与否都写在原始 JSON 的 `criteria` 里。

## 状态

`E_STOP` 高于 `DAMPING_ONLY`，再高于 `LIMITED_ASSIST`，再高于 `NORMAL`。

| 注入 | 状态 | 电机力矩 |
|---|---|---|
| 有限但超出命令上限 | `LIMITED_ASSIST` | 钳位 |
| 传感器非有限 | `E_STOP` | 0，被动阻尼，锁存到 `reset` |
| 传感器掉线 | `DAMPING_ONLY` | 粘性力矩，不保留助力 |
| 通信超时 | `DAMPING_ONLY` | 同上 |

`E_STOP` 不把 `/joint/safe_target_angle` 改写为 0。对应测试是 `test_estop_command_stays_zero_and_angle_does_not_jump`。

## 标记

| 内容 | 标记 |
|---|---|
| 上表模块、重力负载、绑带、只读状态话题 | implemented |
| 故障注入、E_STOP 力矩与角度测试、24 Nm 上限仍在 | verified，见 `tests/test_knee_stair_l1_*.py` |
| 12 种子表和 `criteria` | experimental |
| 重力矩公式、15 Nm 舒适假设、固定髋的单自由度 | theoretical |

## 复现

```text
python scripts/run_knee_stair_l1_metrics.py --check
python -m unittest tests.test_knee_stair_l1_faults tests.test_knee_stair_l1_safety -v
```

在仓库根目录执行。`--check` 对照 `metrics_raw.json`、`metrics_summary.md` 和 `metrics_trials.csv`。

## 回退

撤销本分支上的新提交，或关闭草稿 PR #4 并删除 `feat/knee-stair-assist-l1`。`main` 不会被这些提交改写。
