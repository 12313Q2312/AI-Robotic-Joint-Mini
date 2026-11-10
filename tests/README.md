# 软件验证

在仓库根目录运行 `python scripts/verify_joint_study.py --output data/verification/local-check.json`。需要Python、Node.js及C++17编译器；不需要ROS 2、Webots或第三方Python/npm包。

检查包括Python逻辑/HTTP、C++参考与配对夹具、理想固定轴动力学、新旧面板前端行为。结果分别记录测试用例、行为组和动力学工况，不相加。CI检查名为 `validation`。

这些检查不认证真实仿真、机械或硬件；详细边界见 `../docs/LIMITATIONS.md`。

`tests/test_knee_stair_l1_*.py` 覆盖合成意图、助力、合并监督、植物边界、故障注入和配对指标复现。`load_config` 拒绝 `actuator_torque_limit_nm` 大于 24 Nm 的文件，用的是 main 已有的那条检查。故障注入在 CI 中另有同名步骤。它们不启动 Webots 或 ROS 2。历史 C++ 控制器不在这条钳位测试里，见 issue #11。
