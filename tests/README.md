# 软件验证

在仓库根目录运行 `python scripts/verify_joint_study.py --output data/verification/local-check.json`。需要Python、Node.js及C++17编译器；不需要ROS 2、Webots或第三方Python/npm包。

检查包括Python逻辑/HTTP、C++参考与配对夹具、理想固定轴动力学、新旧面板前端行为。结果分别记录测试用例、行为组和动力学工况，不相加。CI检查名为 `validation`。

这些检查不认证真实仿真、机械或硬件；详细边界见 `../docs/LIMITATIONS.md`。
