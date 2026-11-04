# 公开工程交付记录

日期：2026-11-04。仓库：`12313Q2312/AI-Robotic-Joint-Mini`，用户明确选择保持公开，仅发布工程展示内容。

## 交付内容

保留筛选后的单关节控制、ROS 2包、面板、Webots场景、几何计算、CAD测试件、历史小体积数据与软件测试。重新编写公开说明，增加离线CI、固定Action提交、只读工作流权限、CODEOWNERS、Issue/PR模板、Dependabot、复现与许可状态。

公开副本不包含私人学习记录、完整项目规划或内部文档。原项目、完整交付草稿和原 Git 暂存区保留在本地。本次仅运行离线测试，不启动仿真。

## 验证与服务端设置

公开软件验证摘要与资产哈希清单位于 `data/verification/`。本地受限环境的失败不被改写成通过，最终结果由实际回归和CI给出。

初始工程提交为`b5d67022f8828df1854fd4355702099e68a88079`，123个远程文件的Git blob指纹与准备的公开副本全部一致，无多余文件。[首次Repository CI](https://github.com/12313Q2312/AI-Robotic-Joint-Mini/actions/runs/41819620393)已完成且结果为success。后续资料提交由对应提交的CI单独确认。

原项目的既有文件与Git暂存区指纹检查无变化。参考PDF已核对唯一页面、提取文字、元数据及附件情况，属于T001名义尺寸图，无嵌入附件。

服务端实查仓库为公开、默认分支main；main的protected=false，rulesets为空。当前连接器对分支保护管理接口返回权限不足。CODEOWNERS与贡献规范不代表强制保护已启用；可应用规则和设置步骤见[GITHUB_SETTINGS.md](GITHUB_SETTINGS.md)，尚未执行该服务端修改。

没有将公开可读误称为已选择项目级开源许可证，具体许可状态见 `LICENSE.md`。
