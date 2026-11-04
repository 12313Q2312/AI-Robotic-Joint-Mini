# AI Robotic Joint Mini

[![Repository CI](https://github.com/12313Q2312/AI-Robotic-Joint-Mini/actions/workflows/ci.yml/badge.svg)](https://github.com/12313Q2312/AI-Robotic-Joint-Mini/actions/workflows/ci.yml)

**C++ × ROS 2 × Webots × CAD：单关节控制与工程验证展示。**

本仓库展示已有的力矩 PID、ROS 2 通信、人工面板、日志、CAD 测试零件及几何补偿离线验证。重点是把控制逻辑、物理量、测试配置与结果连接起来，形成可检查、可复用的工程资产。

## 当前完成度

| 模块 | 已有资产 | 验证边界 |
|---|---|---|
| 独立控制 | Webots C++ 力矩 PID、P/PD/PID 与抗饱和示例数据 | 历史实验配置各自独立，不代表当前全系统验收 |
| ROS 2 集成 | C++ 插件、消息、监督、启动与采集代码 | 本次 CI 不构建 ROS 2 或启动 Webots |
| 人工面板 | 角度/键盘输入、控制权、心跳、状态与日志 | 合成 HTTP/前端测试不替代真实反馈验收 |
| 配对夹具 | 同参考、同参 PID、几何势能等效补偿 | 理想固定轴模型；实际双侧 Webots 运动未验收 |
| CAD | T001 原生 SolidWorks 测试零件及名义参考图 | 不是完整装配、质量/惯量认证或加工图 |
| 回归检查 | Python、C++、前端及理想动力学测试 | 只支持对应的软件行为与模型假设 |

## 快速开始

1. 克隆仓库，进入根目录。
2. 确认已有 Python 3.12+、Node.js 22、支持 C++17 的 g++ 或 clang++。
3. 执行以下离线检查，无需安装 Python/npm 依赖：

```text
git clone https://github.com/12313Q2312/AI-Robotic-Joint-Mini.git
cd AI-Robotic-Joint-Mini
python scripts/check_repository.py
python scripts/verify_joint_study.py --output data/verification/local-check.json
```

Linux 可将 `python` 改为 `python3`。验证器不会启动仿真或发送运动指令；返回 0=通过、1=失败、2=阻塞。详细步骤见 [复现说明](docs/REPRODUCIBILITY.md)。

## 控制与证据链

```mermaid
flowchart LR
  A[人工输入] --> B[本地面板与控制权监督]
  B --> C[ROS 2 命令]
  C --> D[C++ 参考轨迹与力矩 PID]
  D --> E[Webots 单关节]
  E --> F[反馈与诊断]
  F --> B
  F --> G[CSV 与离线分析]
```

图示为已有工程的数据流。写入力矩 API 的命令不等于电机实测力矩；参考限速不保证实际关节全过程限速，详见 [局限](docs/LIMITATIONS.md)。

## 目录与入口

| 路径 | 用途 |
|---|---|
| `simulation/webots/` | 独立世界与控制器 |
| `ros2_ws/src/ai_robotic_joint_ros2/` | ROS 2 包、C++ 插件、面板和三类场景 |
| `research/gravity_compensation/` | 已有几何/势能计算与公开来源 |
| `mechanical/` | T001 原生测试件与 URDF 检查夹具 |
| `data/processed/`、`assets/plots/` | 历史小体积数据与曲线 |
| `data/verification/` | 本公开副本的离线验证摘要和文件清单 |
| `scripts/`、`tests/` | 生成、分析、验证入口和回归用例 |
| `.github/` | CI、责任人、Issue/PR模板与依赖更新 |

继续阅读 [工程证据](docs/ENGINEERING_EVIDENCE.md)、[当前状态](PROJECT_STATUS.md)、[贡献规范](CONTRIBUTING.md)、[许可状态](LICENSE.md) 和 [交付记录](docs/REPOSITORY_DELIVERY.md)。

本次发布范围为现有工程展示。仓库可公开阅读，具体使用与分发权利以许可和第三方声明为准。
