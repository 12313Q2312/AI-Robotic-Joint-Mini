# GitHub 服务端设置

2026-11-04已核对：仓库公开、默认分支main、首次Repository CI通过；main的protected=false，rulesets为空。当前连接器的管理权限不足，不能替仓库启用分支保护。以下配置是可应用的规则文件，文件存在不表示规则已生效。

## 单人维护的建议规则

要求通过Pull Request合并、`validation`检查通过且分支更新、解决评审讨论、线性历史，并禁止强推和删除主分支。当前仅一位维护者，独立批准数设为0，避免自己无法批准自己的PR；有独立协作者后再改为至少1人批准，并酌情要求CODEOWNERS评审。

## 网页设置步骤

1. 打开仓库的Settings，进入Branches。
2. 选择Add classic branch protection rule，分支模式填`main`。
3. 勾选Require a pull request before merging。单人维护阶段不强制Require approvals。
4. 勾选Require status checks to pass before merging，添加已有检查`validation`，并要求分支保持最新。
5. 勾选Require conversation resolution before merging、Require linear history，以及禁止绕过规则的管理员约束。
6. 不允许force push或删除该分支，保存规则。
7. 回到main确认保护已生效，再通过一条文档PR核对检查和合并流程。

GitHub界面文字可能因语言与版本不同而略有差异，语义与仓库内规则文件对应。

## 已登录命令行的等价入口

仓库根目录内，在拥有该仓库管理权限且已完成官方GitHub登录的环境执行：

```text
gh api --method PUT repos/12313Q2312/AI-Robotic-Joint-Mini/branches/main/protection --input .github/branch-protection.json
gh api repos/12313Q2312/AI-Robotic-Joint-Mini/branches/main/protection
```

第一次调用应用规则，第二次回读检查。不要把令牌写进命令、配置或仓库。本轮没有执行这项服务端修改。

建议仓库使用squash合并、合并后清理短期分支。当前未通过服务端接口配置这些选项，不把它们计为已完成。
