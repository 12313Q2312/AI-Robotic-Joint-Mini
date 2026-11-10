# 可选 Webots 几何

`worlds/knee_stair_l1.wbt` 从 PR #3 的单腿铰链裁成一侧膝关节。`HUMAN_SHANK` 是腿，`EXO_LINK` 是外骨骼，唯一的电机是 `exo_knee`。

CI 和指标脚本都不启动 Webots。被测闭环是上一级目录里的 `plant.py`。
