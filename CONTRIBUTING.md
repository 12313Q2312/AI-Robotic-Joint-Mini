# 贡献规范

1. 通过 Issue 描述具体问题、配置、最小复现输入和预期行为。
2. 从 `main` 创建短期分支，如 `fix/watchdog-state` 或 `docs/reproduction`。
3. 提交保持单一目的，建议使用 `fix:`、`feat:`、`docs:`、`test:` 前缀；避免全库无关格式化。
4. 运行 `python scripts/check_repository.py`。涉及软件时再运行 [离线验证](docs/REPRODUCIBILITY.md)，注明实际运行结果和未运行项。
5. 提交 Pull Request，说明行为变化、证据、单位/时序/接口影响与回退方式；通过后由维护者合并。

保留历史数据，新增证据需记录配置、版本、单位和处理方式。普通本地输出写入 `data/verification/local-check.json`、`data/raw/local/`，默认忽略。不要提交编译文件、安装包、密钥、个人记录或未经许可的第三方材料。

当前只有一位维护者，CODEOWNERS 标识责任人。是否强制评审和 CI 检查以 GitHub 服务端规则为准，见 [交付记录](docs/REPOSITORY_DELIVERY.md)。

不得用合成测试声称真实运动、机械安全、测得节能或用户掌握已验收。使用/贡献前先阅读 [许可状态](LICENSE.md)。
