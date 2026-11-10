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
| 遥测 | `telemetry.py`、`../scripts/run_knee_stair_l1_metrics.py` | 仓库里是汇总和小样本。逐步原始 JSON 只在 CI artifact `knee-stair-l1-metrics-raw` |

## 负载、意图和交互

重力矩使膝关节屈曲。有种子因子时，`plant.py` 先按台阶高度缩放力臂（第 59–61 行）：

```text
reference_height = float(load["reference_step_height_m"])
self.body_mass = float(factors.body_mass_kg)
self.moment_arm *= float(factors.step_height_m) / reference_height
```

屈曲力矩再用缩放后的力臂（`gravity_flexion_nm`，第 85 行）：

```text
self.weight_share * self.body_mass * self.gravity * self.moment_arm * math.sin(flexion_rad)
```

没有因子时不乘台阶高度，力臂保持配置里的 `moment_arm_m`。伸直时 `sin` 为 0，力矩为 0。文字假设在配置的 `load.assumption`。这是 **theoretical**。

合成意图是一条平滑的屈膝下降轨迹，种子只改变下降幅度，而且终点离伸膝限位还有余量。人体力矩是 PD 跟踪并限幅。关闭助力时，这组参数不把膝推到限位；一旦碰到限位，植物记一条 `anomaly hard_stop_contact`。

绑带力矩 `K*(q_exo - q_human) + B*(qd_exo - qd_human)` 作用在人体上，反作用在外骨骼上。峰值交互力矩读的是这个约束，不是电机力矩。舒适阈值 15 Nm 的来源写在 `metrics.comfort_basis`：0.08 m 力臂乘 187.5 N 的设计假设，不是舒适性试验。

## 指标

同一种子的轨迹在关闭、导纳和 AssistPolicy 之间相同。标准差用 n−1。降幅是关闭减去助力。95% 区间是 30 个种子、df=29 的 t 区间。

30 个种子（0–29）。每个种子抽取一次体重、台阶高、意图幅度、起始时间、绑带刚度和传感噪声水平，关闭、导纳和 AssistPolicy 共用。噪声序列也按种子配对。

主指标：人体膝关节力矩 RMS 的降幅，人体膝关节正功的降幅。跨种子标准差必须大于 0。导纳的正功降幅小于 AssistPolicy，这是结果。

次指标：跟踪误差 RMS、绑带交互峰值、饱和时间比例、硬限位接触次数。

时延是每一步的力矩命令时间戳减去该命令所消费的意图时间戳，再取 p50、p95 和最大值。不再使用充数的时机误差。

通过标准，导纳和 AssistPolicy 都相对关闭来评：

- 人体力矩 RMS 降幅的 95% 区间下界大于 0
- 每个种子的跟踪误差不超过关闭时的 1.1 倍
- 硬限位接触次数为 0
- 交互峰值低于配置里的 15 Nm

导纳饱和时间比例还要低于 5%。做到这一点时没有提高 24 Nm 上限。结果无论通过与否都写在原始 JSON 的 `criteria` 里。

## 力矩限幅分层表

三层配置值与代码一致。运行时没有把它们收成一次 `min()`。导纳路径先被 PID 钳住，再交给监督器。AssistPolicy 不经过 PID，只受自己的内部上限和监督器约束。植物拒绝大于 `boundary_abs_limit_nm` 的电机力矩，而该边界必须等于监督器上限。

| 层 | 值 | 作用 | 代码 |
|---|---|---|---|
| PID 执行器硬限幅 | 24 Nm | 导纳内环把控制力矩钳在 `±torque_limit` | `config/default.json` 第 66 行；钳位在 `torque_pid.py` 第 80–83 行；由 `controller.py` 第 53 行传入 |
| 监督器命令合理性检查 | 40 Nm | 有限命令的绝对值大于 `command_limit_nm` 时进入 `LIMITED_ASSIST` 并钳到该值 | `config/default.json` 第 73 行；判断在 `safety.py` 第 154–155 行；钳位在第 168 行 |
| AssistPolicy 内部上限 | 80 Nm | 策略目标钳在 `±limit`，然后才交给监督器 | `config/default.json` 第 53 行；钳位在 `assist_policy.py` 第 39–42 行；由 `controller.py` 第 55 行传入 |
| 植物边界 | 40 Nm | 与监督器上限必须相同；更大的电机力矩被拒绝 | `config/default.json` 第 88 行；`plant.py` 第 105–106 行；`config.py` 第 84–85 行 |

三层配置值的最小值是 PID 的 24 Nm。导纳路径到植物的力矩不超过 24 Nm。AssistPolicy 路径到植物的力矩不超过监督器／植物边界的 40 Nm，不是 24 Nm。可选 Webots 几何里的电机 `maxTorque` 也是 40，见 `webots/generate_world.py` 第 53 行；那一文件没有绑带动力学，CI 不运行它。

配置加载时断言这三层的最小值不得超过 24 Nm（`config.py` 第 95–98 行）。当前配置的最小值是 24 Nm，断言通过。这些数值没有为了和“有效上限等于 24 Nm”对齐而改写。

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
| 30 种子汇总、`criteria`、延迟图和敏感性扫描 | experimental |
| 重力矩公式、15 Nm 舒适假设、固定髋的单自由度 | theoretical |

## 复现

```text
python scripts/run_knee_stair_l1_metrics.py --check
python -m unittest tests.test_knee_stair_l1_faults tests.test_knee_stair_l1_safety -v
```

在仓库根目录执行。`--check` 对照汇总 JSON、指标表、试验 CSV 和小样本。`run_knee_stair_l1_sweeps.py --check` 对照延迟扫描和敏感性 JSON。逐步原始文件写到 gitignore 的 `local_raw/`，CI 再上传。

## 回退

这一层已由 PR #4 合入 `main`（92befe7）。回退时用新的反向提交，不要改写 `main` 历史，也不要强推。
