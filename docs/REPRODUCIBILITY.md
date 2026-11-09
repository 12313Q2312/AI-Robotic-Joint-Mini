# 复现说明

## 离线检查

在根目录确认已有 Python 3.12+、Node.js 22、支持 C++17 的 g++ 或 clang++。CI 使用 Ubuntu 24.04，语言版本在 `.python-version`、`.node-version` 中声明。

```text
python --version
node --version
g++ --version
python scripts/check_repository.py
python scripts/verify_joint_study.py --output data/verification/local-check.json
```

Linux 可用 `python3`。检查只使用 Python 标准库和 Node.js 内置模块，无需安装 pip/npm 包；Windows 编译器依赖的运行库须可访问。验证器运行 Python 测试、两个 C++ 程序、12个理想固定轴动力学工况以及两个前端回归，临时 C++ 二进制会清理。结果按口径分别记录，不能直接相加。

本地 JSON 记录工具版本、源码 SHA256、结果与失败原因。返回 0=通过、1=失败、2=阻塞。若环境禁止本机 HTTP 通信或执行编译器，保留失败记录，在具备相应权限的环境验证；不要因此删除测试或关闭保护设置。

公开的基线摘要位于 `data/verification/software_baseline_20261104.json`，不含本机私人路径；本地完整日志与临时运行结果默认不入库。

## ROS 2 与 Webots

已有工程曾使用 Windows/Webots R2025a 与 WSL Ubuntu22.04/ROS 2 Humble。本次没有重建或运行这些组件，也未重新验证历史环境问题。实际启动需要准备兼容的 Webots、ROS 2、webots_ros2_driver 及包依赖，先阅读 [ROS 2入口](../ros2_ws/README.md) 和 [局限](LIMITATIONS.md)。

基础 CI 只验证离线逻辑，不把 ROS 2 插件构建、Webots 联通或机械运动写成已通过。当前仓库不是一键运行整机演示。

## 单侧膝上楼助力闭环

在仓库根目录用标准库复现合成闭环。`--check` 对照已提交的原始试验行和汇总，不启动 Webots 或 ROS 2。

```text
python scripts/run_knee_stair_l1_metrics.py --check
python -m unittest tests.test_knee_stair_l1_faults -v
```

结果说明见 [knee_stair_l1/README.md](../knee_stair_l1/README.md) 和 [指标表](../data/verification/knee_stair_l1/metrics_summary.md)。这组数字是仿真实验，不是真人、肌电或防跌倒证据。CI 是否通过以该次工作流记录为准。

## CAD 和绘图

`mechanical/cad_native/import_test/T001_test_link_r01.SLDPRT` 为保留的原生文件。名义规格为180×30×8 mm、两孔直径12/6 mm、孔距150 mm；材料、质量/惯量与完整装配仍需独立核对。

`scripts/generate_test_link_solidworks.py --check-only` 只检查规格。实际 CAD 自动化依赖未随仓库发布的外部执行器，使用 `JOINT_MINI_SW_EXECUTOR` 指定；默认本地路径为 `local/solidworks_executor.py`，不会自动安装或下载。`generate_test_link_views.py` 使用 reportlab 及 Windows SimHei 字体；`plot_experiment_001.py` 使用 matplotlib，均为可选工具，不属于基础 CI。

历史 CSV、名义图和曲线是特定时间/配置的工程资产，不证明当前程序或全系统验收。数据说明见 [工程证据](ENGINEERING_EVIDENCE.md)。
