# 人机协同助力仿真演示

当前可供研发分项验收的电脑原型。先阅读[验收报告](ACCEPTANCE_REPORT.md)与[模型与实现说明](MODEL_AND_IMPLEMENTATION.md)。未验证真人效果或全身跌倒缓冲。

Windows双击 `打开仿真演示.cmd`，或直接打开 `viewer/index.html`，可回放11个已生成场景。显示的是实际仿真状态抽样，不是在线物理计算。使用已有Python，无需安装分析依赖即可回放。

## 已验证的范围

- 合成人体浮动基座、10关节、重力/接触与C++直接力矩控制；正常任务无外部支撑或重定位。
- 六次平地迈步并停稳；同质量配对的人体等效正功降低15.54%，是机械代理指标。
- 三阶各0.10m上楼，47秒完成；上楼80Nm助力与4kg设备重量均为尚未硬件闭合的假设。
- 本地停止、模拟候选计划拒绝与模拟云端离线后的有限任务完成。
- 小扰动完成；大扰动及旧上楼配置失稳，保留失败回放。
- 独立一维缓冲与行程不足触底对照；没有接入全身跌倒部署。

## 软件检查

已有Python 3.12+、C++17编译器和Node.js；分析依赖确切版本见 `requirements-analysis.txt`。在仓库根目录执行：

```text
python mobility_demo/verify.py --software-only
```

该命令检验软件行为和选定的实际几何采样，不启动Webots，也不重建完整证据。完整本地验收记录另有原始证据重建检查。

## 重新运行物理仿真

使用已有Webots R2025a与C++17编译器；当前运行入口针对Windows。必要时设置 `WEBOTS_HOME`、`CXX`，不会自动下载工具。

```text
python mobility_demo/run_webots.py --scenario flat --mode assist --run my_new_run
python mobility_demo/analyze.py my_new_run
```

场景支持 `stand/flat/stairs/stop/push-small/push-large`，模式支持 `assist/off/no-device`。每次使用新编号，禁止覆盖历史证据。上楼配对关闭助力尚未通过，因此不报告上楼省力比例。

## 数据分发

公开仓库含源码、参数、实际状态回放、几何采样、运行指纹与结果摘要。约39.3MB完整CSV/事件/世界/源码快照包保存在本地，SHA256见 `../data/verification/mobility_20261105/full_local_evidence_manifest.json`，未随Git分发。没有完整包时不要运行 `build_demo.py/report.py` 重建已有报告；新运行的数据可独立分析。需要重建本次完整报告时，把原包和对应manifest放入本目录，再运行 `unpack_evidence.py`、`build_demo.py`、`report.py`。

本地完整包与公开展示的区别见验收报告。私人学习记录、专利筛选资料和研究断点不在本目录。
