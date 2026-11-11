# 单侧膝关节上楼助力闭环

只在电脑里运行。没有真人，没有真实表面肌电，也没有防跌倒结果。

## 模块

参数都在 `config/default.json`。话题名在 `topics.py`。

| 模块 | 文件 | 合同 |
|---|---|---|
| 意图源 | `intent.py` | `/knee_stair/intent_synthetic`。v1 是合成期望轨迹，日志写明 synthetic |
| 有界人体力矩 | `human.py` | 跟踪上述轨迹，并被 `human.torque_limit_nm` 截断 |
| 助力 | `admittance.py`、`torque_pid.py`、`assist_policy.py`、`controller.py` | 期望运动和 `/knee_stair/assist_command_raw`。PID 抗饱和上限保持 24 Nm |
| 安全监督 | `safety.py` | 发出监督后命令，并发布只读的 `/joint/safety_state`。40 Nm 是命令合理性检查，不是力矩上限 |
| 执行器上限 | `actuator.py`，`include/actuator_torque.hpp` | 唯一数字定义是 `actuator_torque_limit_nm`。Python 与四个 C++ 控制器都读它。见 [issue #11](https://github.com/12313Q2312/AI-Robotic-Joint-Mini/issues/11) |
| 植物 | `plant.py` | 人体小腿、外骨骼连杆、重力矩和弹簧阻尼绑带。边界等于执行器上限 |
| 遥测 | `telemetry.py`、`../scripts/run_knee_stair_l1_metrics.py` | 仓库里是汇总和小样本。逐步原始 JSON 只在 CI artifact `knee-stair-l1-metrics-raw` |

## 负载、意图和交互

重力矩使膝关节屈曲。有种子因子时，`plant.py` 先按台阶高度缩放力臂（第 66–68 行）：

```text
reference_height = float(load["reference_step_height_m"])
self.body_mass = float(factors.body_mass_kg)
self.moment_arm *= float(factors.step_height_m) / reference_height
```

屈曲力矩再用缩放后的力臂（`gravity_flexion_nm`，第 92 行）：

```text
self.weight_share * self.body_mass * self.gravity * self.moment_arm * math.sin(flexion_rad)
```

没有因子时不乘台阶高度，力臂保持配置里的 `moment_arm_m`。伸直时 `sin` 为 0，力矩为 0。文字假设在配置的 `load.assumption`。这是 **theoretical**。

合成意图是一条平滑的屈膝下降轨迹，种子只改变下降幅度，而且终点离伸膝限位还有余量。人体力矩是 PD 跟踪并限幅。关闭助力时，这组参数不把膝推到限位；一旦碰到限位，植物记一条 `anomaly hard_stop_contact`。

绑带力矩 `K*(q_exo - q_human) + B*(qd_exo - qd_human)` 作用在人体上，反作用在外骨骼上。峰值交互力矩读的是这个约束，不是电机力矩。舒适阈值 15 Nm 的来源写在 `metrics.comfort_basis`：0.08 m 力臂乘 187.5 N 的设计假设，不是舒适性试验。

## 指标

同一种子的轨迹在关闭、导纳和 AssistPolicy 之间相同。标准差用 n−1。降幅是关闭减去助力。95% 区间是学生 t 区间，自由度是 n−1，临界值按这个自由度计算。报告用 30 个种子，所以 df=29。

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

最终硬上限只有一处数字定义：`knee_stair_l1/config/default.json` 的 `actuator_torque_limit_nm`。Python 的 `clamp_actuator_torque` 和 C++ 的 `actuator_torque::Gate::apply` 都读这个键。关闭、导纳、AssistPolicy 进入植物之前走 `torque_to_motor`。`joint_safety_node` 发到 `/knee_stair/assist_command_safe` 的是钳位后的力矩。四个历史 C++ 控制器在负载力矩加上之后再经过同一个门。这是 [issue #11](https://github.com/12313Q2312/AI-Robotic-Joint-Mini/issues/11)。

| 层 | 值 | 作用 | 代码 |
|---|---|---|---|
| 执行器硬上限 | 24 Nm | `clamp_actuator_torque` 把送到植物或安全话题的力矩收到 ±24 Nm。NaN 和 inf 抛出 `ValueError`，函数本身不返回 0 | `config/default.json` 第 25 行；`actuator.py` 第 15–28 行；植物侧 `plant.py` 第 115 行 |
| 非有限命令的发布 | 0 Nm，并锁存 `E_STOP` | `on_assist_command` 接住上面的 `ValueError`，再发布 0 | `safe_publish.py` 的 `_estop_after_rejected_clamp`，第 59–68 行 |
| 监督器合理性检查 | 40 Nm | 有限命令超出 ±`command_limit_nm` 时进入 `LIMITED_ASSIST`。这不是力矩上限。监督器输出仍可到 40 Nm，随后由执行器钳位收到 24 Nm | `config/default.json` 第 74–75 行；判断 `safety.py` 第 157–158 行；合理性钳位 `safety.py` 第 170–172 行 |
| PID 抗饱和上限 | 24 Nm | 只约束导纳内环的提案，不是唯一的最终出口 | `config/default.json` 第 67 行；`torque_pid.py` 第 80–83 行；`controller.py` 第 53 行 |
| AssistPolicy 内部上限 | 24 Nm | 策略目标钳在 ±24 Nm。PR #3 抄来的头文件是 80 Nm；PR #10 起本树改为 24 Nm。绕过策略直接注入的命令仍由 `clamp_actuator_torque` 截住 | `config/default.json` 第 54 行；`assist_policy.py` 第 42–45 行；`controller.py` 第 55 行 |
| 植物边界 | 24 Nm | 必须等于 `actuator_torque_limit_nm` | `config/default.json` 第 90 行；`plant.py` 第 51–54 行；`config.py` 第 72–73 行 |
| Webots 单膝电机 | 同一配置值 | `generate_world.py` 把该键写成 `maxTorque`。该世界没有绑带动力学，CI 不运行它 | `webots/generate_world.py`；`webots/worlds/knee_stair_l1.wbt` |
| `joint_controller.cpp` | 同一配置值 | 控制器内环仍是 0.10 Nm，负载加上之后 `Gate::apply` 再钳一次。非有限输入输出 0 并锁存 `E_STOP` | `simulation/webots/controllers/joint_controller/joint_controller.cpp` |
| `JointControllerPlugin.cpp` | 同一配置值 | 同上，钳位在 `control + load` 之后、`wb_motor_set_torque` 之前 | `ros2_ws/.../JointControllerPlugin.cpp` |
| `ManualJointControllerPlugin.cpp` | 同一配置值 | 内环 `limit_` 之后再加上 `load_`，然后进入共享门 | `ros2_ws/.../ManualJointControllerPlugin.cpp` |
| `StudyJointControllerPlugin.cpp` | 同一配置值 | 两个电机的 `commanded_torque_nm` 在写出前进入各自的门。连杆上的 `wb_supervisor_node_add_torque` 是研究扰动，不是电机命令，没有改 | `ros2_ws/.../StudyJointControllerPlugin.cpp` |

PR #10 合并之前，AssistPolicy 内 cap 是 80 Nm，植物边界等于监督器的 40 Nm，文档曾把 AssistPolicy 到植物的命令写成不超过 40 Nm。那是合并前的历史。当前三条 `knee_stair_l1` 路径和 `joint_safety_node` 的最终硬上限都是 24 Nm。

配置加载沿用 `load_config` 里已有的检查：`0 < actuator_torque_limit_nm <= 24`（`config.py` 第 67–68 行）。这里不再另写一套 `min()` 断言。

## 状态

`E_STOP` 高于 `DAMPING_ONLY`，再高于 `LIMITED_ASSIST`，再高于 `NORMAL`。

| 注入 | 状态 | 送到植物或安全话题的力矩 |
|---|---|---|
| 有限但超出 40 Nm 合理性检查 | `LIMITED_ASSIST` | 监督器先收到 40 Nm，执行器再收到 24 Nm |
| 传感器非有限 | `E_STOP` | 0，被动阻尼，锁存到 `reset` |
| 命令为 NaN 或 inf | `E_STOP` | `clamp_actuator_torque` 抛出 `ValueError`；`safe_publish` 发布 0 并锁存 |
| 传感器掉线 | `DAMPING_ONLY` | 粘性力矩，不保留助力 |
| 通信超时 | `DAMPING_ONLY` | 同上 |

`E_STOP` 不把 `/joint/safe_target_angle` 改写为 0。对应测试是 `test_estop_command_stays_zero_and_angle_does_not_jump`。

## 标记

| 内容 | 标记 |
|---|---|
| 上表模块、重力负载、绑带、只读状态话题 | implemented |
| 故障注入、E_STOP 力矩与角度测试、24 Nm PID 上限仍在 | verified，见 `tests/test_knee_stair_l1_*.py` |
| 注入 80 Nm 后，关闭、导纳、AssistPolicy 和 ROS 处理函数交到植物或电机话题的力矩 ≤ 24 Nm | verified，软件测试，见 `tests/test_actuator_torque_limit.py`。没有启动 ROS |
| 30 种子汇总、`criteria`、延迟图和敏感性扫描 | experimental |
| 重力矩公式、15 Nm 舒适假设、固定髋的单自由度 | theoretical |

上表的四个 C++ 控制器已经接入共享门，对应 [issue #11](https://github.com/12313Q2312/AI-Robotic-Joint-Mini/issues/11)。仓库里没有另带的第三方 Webots 示例控制器。C++ 侧 NaN、+inf 和 -inf 的结果是输出 0 并锁存 `E_STOP`，与 Python 发布路径一致；Python 的 `clamp_actuator_torque` 本身仍然抛出 `ValueError`，不改成返回 0。

## 复现

```text
python scripts/run_knee_stair_l1_metrics.py --check
python -m unittest tests.test_knee_stair_l1_faults tests.test_knee_stair_l1_safety -v
```

在仓库根目录执行。`--check` 对照汇总 JSON、指标表、试验 CSV 和小样本。`run_knee_stair_l1_sweeps.py --check` 对照延迟扫描和敏感性 JSON。逐步原始文件写到 gitignore 的 `local_raw/`，CI 再上传。

## 回退

PR #10 已在 `main`（`d79d5c9`）。本分支只改正文档，并沿用 main 的配置检查。回退时撤销本分支的提交，不要改写 `main` 历史，也不要强推。
