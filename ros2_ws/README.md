# ROS 2 集成入口

包：`ai_robotic_joint_ros2`。历史环境为 ROS 2 Humble、Ubuntu22.04 与 Webots R2025a。包依赖见 `src/ai_robotic_joint_ros2/package.xml`。本轮未重新构建或启动ROS 2/Webots。

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

这只是构建命令说明，本轮未执行。启动前需核对Webots路径、共享工作区、端口、URDF、控制器和消息参数；不同场景保持隔离。面板绑定本机回环地址，不对外部署。

面板静态预览可通过包下 `scripts/manual_panel_server.py --preview` 或 `scripts/study_panel_server.py --preview` 启动，它不会生成机器人假反馈，也不等于系统联通。实际运动应按 `../docs/LIMITATIONS.md` 单独验收。
