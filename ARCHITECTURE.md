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
