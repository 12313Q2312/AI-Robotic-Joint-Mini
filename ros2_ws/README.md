# ROS 2 集成入口

## RJ-INT-01A: Linux native Webots + ROS 2 smoke

The new `joint_native_launch.py` brings up the legacy single-axis Webots
world, driver and safety supervisor together. Use the separately installed
`rj_int_01_smoke.py` command to capture actual position, torque-command,
safe-reference and timeout-state messages.

**[Full setup and acceptance instructions](../docs/RJ-INT-01.md)**

This is not yet the knee-assistance physics integration. ROS 2 build passing
alone does not prove Webots motion. The original WSL/Windows remote controller
launch remains available unchanged.


包：`ai_robotic_joint_ros2`。当前 CI 的 `ros2-build-jazzy` 在 Ubuntu 24.04 容器 `ros:jazzy-ros-base` 里构建本包、跑 CTest，并安装校验过的 Webots R2025a 后做无头冒烟。历史环境记录仍是 ROS 2 Humble、Ubuntu 22.04 与 Webots R2025a。包依赖见 `src/ai_robotic_joint_ros2/package.xml`。

`JointControllerPlugin` 在位置传感器不是有限值时把电机力矩写成 0，并锁存 `E_STOP`。这个锁存不会因为后来的有限读数而解除。清除它的方式只有重启该插件进程，或重启仿真（仿真重启会重新构造插件）。故障期间仍发布 `/joint/state`：位置为 NaN，`name` 在 `joint_axis` 之后带有 `fault`。

| 场景 | Launch/世界 |
|---|---|
| 早期单关节 | `joint_launch.py` / `joint_minimal_ros2.wbt` |
| 人工面板 | `manual_demo_launch.py` / `joint_manual_demo.wbt` |
| 配对夹具 | `study_demo_launch.py` / `joint_study.wbt` |
| 单侧膝仿真监督 | 同一 `joint_safety_node.py`。膝话题见仓库根目录 `knee_stair_l1/README.md`。离线闭环不需要本包构建 |

在已配置兼容依赖的 Linux/WSL 环境，常规构建入口是：

```text
cd ros2_ws
colcon build --packages-select ai_robotic_joint_ros2
```

CI 的 `ros2-build-jazzy` 执行的是同一条 `colcon build --packages-select ai_robotic_joint_ros2`，并额外安装 `python3-colcon-common-extensions`、`ros-jazzy-webots-ros2-driver`、`ros-jazzy-rosidl-default-generators` 和 `ros-jazzy-pluginlib`。本机启动前仍需核对 Webots 路径、共享工作区、端口、URDF、控制器和消息参数；不同场景保持隔离。面板绑定本机回环地址，不对外部署。

面板静态预览可通过包下 `scripts/manual_panel_server.py --preview` 或 `scripts/study_panel_server.py --preview` 启动，它不会生成机器人假反馈，也不等于系统联通。实际运动应按 `../docs/LIMITATIONS.md` 单独验收。
