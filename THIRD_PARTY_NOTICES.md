# 第三方来源与依赖

| 项目 | 用途和边界 |
|---|---|
| Webots / webots_ros2 / ROS 2 | 本地仿真和运行依赖；不分发其安装包，适用各官方项目的许可证 |
| SolidWorks | 原生测试件的建模工具；商业软件和许可证不随仓库分发 |
| `research/gravity_compensation/sources.csv` | 历史几何补偿背景资料的公开引用；不附第三方全文 |
| GitHub Actions | 引用官方 checkout/setup-python/setup-node 固定提交，并配置更新提醒 |
| matplotlib / reportlab | 可选作图/参考PDF生成依赖，未纳入基础验证必需项 |

T001 是用户保存的原生测试件；参考 PDF/PNG 为辅助名义规格图，不是 CAD 关联制造图。没有将第三方软件、外部 CAD 执行器、模型权重或训练语料随仓库发布。

本声明记录已识别来源和边界，不声称已完成逐文件许可审计。项目许可状态见 `LICENSE`。
