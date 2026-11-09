# 单侧膝关节上楼助力闭环

只在电脑里运行。没有真人，没有真实表面肌电，也没有防跌倒结果。全身十关节、危险预测、缓冲柔顺和云端连接不在这里。

## 模块

参数都在 `config/default.json`。话题名在 `topics.py`，ROS 2 节点和进程内总线使用同一组名字。

| 模块 | 文件 | 合同 |
|---|---|---|
| 意图源 | `intent.py` | 发布 `/knee_stair/intent_synthetic`。v1 类名和日志写明 synthetic，可换成别的 `IntentSource` |
| 助力 | `admittance.py`、`torque_pid.py`、`assist_policy.py`、`controller.py` | 发布期望运动和 `/knee_stair/assist_command_raw` |
| 安全监督 | `safety.py` | 唯一允许发出 `/knee_stair/assist_command_safe` 和 `/joint/safety_state` 的对象 |
| 植物 | `plant.py` | 订阅安全力矩，发布 `/knee_stair/plant_state` |
| 遥测 | `telemetry.py`、`../scripts/run_knee_stair_l1_metrics.py` | `/knee_stair/telemetry` |

`ros2_ws/src/ai_robotic_joint_ros2/scripts/joint_safety_node.py` 构造一个 `JointSafetyCore`。角度口和膝力矩口都进这一个对象。

## 状态

优先顺序是 `E_STOP`，然后 `DAMPING_ONLY`，然后 `LIMITED_ASSIST`，然后 `NORMAL`。

| 注入 | 状态 | 力矩 |
|---|---|---|
| 有限、但超出 `command_limit_nm` 的命令 | `LIMITED_ASSIST` | 钳到该上限 |
| 传感器变为非有限 | `E_STOP` | 0，锁存到 `reset` |
| 最后一次有限传感器早于 `sensor_dropout_s` | `DAMPING_ONLY` | 只剩与屈膝角速度相反的阻尼 |
| 最后一次有限命令早于 `command_timeout_s` | `DAMPING_ONLY` | 同上，不保留助力命令 |

断言在 `tests/test_knee_stair_l1_faults.py`。

既有 `/joint/target_angle` 仍被钳到配置里的 ±90°，非有限样本被丢弃，超过 1 s 没有新命令时安全角度为 0。膝侧 `E_STOP` 不改这个角度数字；若要让急停同时改写该话题，会改变原接口，需要另行决定。

## 指标

每种子跑关闭、导纳和 AssistPolicy 各一次。人体合成力矩只随种子变，不随条件变。汇总是均值 ± 样本标准差（n−1）。

- `assist_timing_error_s`：施加力矩达到 `assist_onset_nm` 的时刻，减去合成力矩达到 `onset_effort_nm` 的时刻。关闭时助力不出现，误差记成 `duration_s - intent_onset_s`。
- `end_to_end_latency_s`：植物施加时刻减去监督器产生该命令的时刻，再对步取平均。由 `command_transport_delay_steps` 决定，三种条件相同。
- `peak_interaction_torque_nm`：穿过人机边界的力矩峰值。人体自身力矩不算在内。
- `saturation_limit_trigger_count`：PID 饱和、AssistPolicy 饱和或状态为 `LIMITED_ASSIST` 的控制步数。

已提交的表和原始行：

- [metrics_summary.md](../data/verification/knee_stair_l1/metrics_summary.md)
- [metrics_trials.csv](../data/verification/knee_stair_l1/metrics_trials.csv)

导纳峰值在这组参数下每个种子都等于 PID 力矩上限，所以标准差是 0。这是该次实验的结果，不是测量失败。

## 标记

| 内容 | 标记 |
|---|---|
| 上表五个模块和可选 Webots 几何文件 | implemented |
| 故障注入、历史角度口数字、PID 首步与 AssistPolicy 饱和/变化率 | verified，见 `tests/test_knee_stair_l1_*.py` |
| 12 种子配对表 | experimental |
| 固定髋、理想边界、默认定值负载 0 的单膝模型 | theoretical |

真人、真实肌电和防跌倒不属于这些标记。

## 复现

```text
python scripts/run_knee_stair_l1_metrics.py --check
python -m unittest tests.test_knee_stair_l1_faults -v
```

在仓库根目录执行。`--check` 重新计算并对照已提交文件。Webots 世界由 `webots/generate_world.py` 生成，CI 不启动它。

## 回退

关掉本分支的草稿 PR 并删除分支 `feat/knee-stair-assist-l1`。`main` 没有这些提交。
