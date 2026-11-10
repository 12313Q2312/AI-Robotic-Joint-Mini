# 既有工程结构

## 入口与隔离

| 入口 | 代码/场景 | 职责 |
|---|---|---|
| 独立 Webots | `simulation/webots/controllers/joint_controller` | 单关节控制基础 |
| 早期 ROS 2 | `JointControllerPlugin.cpp`、`joint_minimal_ros2.wbt` | 目标、监督、状态采集 |
| 人工演示 | `ManualJointControllerPlugin.cpp`、`joint_manual_demo.wbt` | 人工参考、所有权和诊断 |
| 配对夹具 | `StudyJointControllerPlugin.cpp`、`joint_study.wbt` | 两侧相同参考与 PID，比较几何补偿模型 |

ROS 2 包位于 `ros2_ws/src/ai_robotic_joint_ros2`；头文件 `ManualReference.hpp` 与 `StudyPair.hpp` 包含离线可验证的逻辑。面板核心与 HTTP/ROS 适配分离，Node.js 回归使用合成 DOM 验证前端行为。

## 单位与接口

内部控制主要使用弧度、秒、牛顿米和 SI 物理参数，界面可显示角度。CAD 规格使用毫米，API与仿真使用米；转换位置须显式保留。CSV 字段名的单位后缀及源代码定义优先于界面文字。

历史 `JointState.effort` 表示合成 API 命令，不能当作传感器测得的电机力矩。监督、STOP、回零和硬件急停的语义不同，见 `docs/LIMITATIONS.md`。

几何补偿通过弹簧势能关于角度的导数得到等效力矩；当前模型没有验证真实凸轮接触、摩擦滞回或制造误差。几何模块与控制器的可复用边界，以现有头文件/CSV为准，不宣称已经建立跨平台通用框架。

## 单侧膝仿真闭环

`knee_stair_l1` 与既有单关节场景分开。模块之间的合同是下列话题；无 ROS 2 时，同一字段走进程内总线，CI 不启动 Webots。

| 话题 | 方向 |
|---|---|
| `/knee_stair/intent_synthetic` | 意图源 → 控制器。v1 的 `source_label` 为 synthetic |
| `/knee_stair/desired_motion` | 导纳期望屈膝运动。关闭和 AssistPolicy 不生成该运动 |
| `/knee_stair/assist_command_raw` | 控制器提案 |
| `/knee_stair/assist_command_safe` | `knee_stair_l1` 与 `joint_safety_node` 在执行器钳位之后发出的力矩。历史 C++ 控制器不使用这个话题 |
| `/knee_stair/plant_state` | 植物的屈膝角、角速度和边界力矩 |
| `/joint/safety_state` | 同一个监督器的状态 |
| `/joint/target_angle` → `/joint/safe_target_angle` | 原角度口，数字行为保持 |

命令归属：意图源给出合成期望轨迹，有界人体跟踪器产生人体力矩，控制器只提案，`JointSafetyCore` 做合理性检查。`knee_stair_l1` 和 `joint_safety_node` 再经 `clamp_actuator_torque` 把力矩送进植物或安全命令话题。这不是全仓库电机的共同出口。植物把人体力矩、重力矩、绑带约束和这份电机力矩分开积分。交互力矩是绑带力矩。`E_STOP` 时电机力矩为 0，外骨骼只剩被动阻尼，`/joint/safe_target_angle` 沿用原合同。`/joint/safety_state` 只发布。故障时状态按 `E_STOP`、`DAMPING_ONLY`、`LIMITED_ASSIST`、`NORMAL` 的优先顺序取更严格者。
