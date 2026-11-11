"""Fail if the actuator hard-limit number is defined more than once.

The only definition is actuator_torque_limit_nm in
knee_stair_l1/config/default.json. The generated Webots world may repeat that
same number as maxTorque, because VRML cannot reference the JSON key. Any
other assignment of that number as a torque limit fails this check.
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "knee_stair_l1" / "config" / "default.json"
WORLD = ROOT / "knee_stair_l1" / "webots" / "worlds" / "knee_stair_l1.wbt"
CODE_SUFFIXES = {".py", ".cpp", ".hpp", ".h", ".json"}
SCAN_ROOTS = (
    ROOT / "knee_stair_l1",
    ROOT / "simulation",
    ROOT / "ros2_ws",
    ROOT / "scripts",
    ROOT / "tests",
)


def limit_token(value: float) -> str:
    if value == int(value):
        return str(int(value))
    return str(value)


def main() -> int:
    config = json.loads(CONFIG.read_text(encoding="utf-8"))
    limit = float(config["actuator_torque_limit_nm"])
    token = limit_token(limit)
    number = rf"(?<![\d.]){re.escape(token)}(?:\.0+)?(?![\d.])"
    assignment = re.compile(
        rf'(?:actuator_torque_limit_nm|torque_limit_nm|boundary_abs_limit_nm|limit_nm)"\s*:\s*{number}'
        rf"|maxTorque\s+{number}"
        rf"|(?:<=|>=|==|!=)\s*{number}"
        rf"|(?:assertEqual|assertAlmostEqual|assertLessEqual)\([^\n]*{number}"
    )
    allowed_config = CONFIG.resolve()
    allowed_world = WORLD.resolve()
    config_hits = []
    other_hits = []
    world_hits = []
    for root in SCAN_ROOTS:
        for path in root.rglob("*"):
            if not path.is_file():
                continue
            if path.suffix not in CODE_SUFFIXES and path.resolve() != allowed_world:
                continue
            text = path.read_text(encoding="utf-8")
            for line_number, line in enumerate(text.splitlines(), start=1):
                if not assignment.search(line):
                    continue
                record = f"{path.relative_to(ROOT).as_posix()}:{line_number}: {line.strip()}"
                if path.resolve() == allowed_config and "actuator_torque_limit_nm" in line:
                    config_hits.append(record)
                elif path.resolve() == allowed_world and "maxTorque" in line:
                    world_hits.append(record)
                else:
                    other_hits.append(record)
    errors = []
    if len(config_hits) != 1:
        errors.append(f"expected one actuator_torque_limit_nm definition, found {len(config_hits)}")
        errors.extend(config_hits)
    if len(world_hits) != 1:
        errors.append(f"expected one generated maxTorque line, found {len(world_hits)}")
        errors.extend(world_hits)
    errors.extend(other_hits)
    if errors:
        print("FAIL: duplicated actuator torque-limit literal")
        for error in errors:
            print(error)
        return 1
    print(f"PASS: one actuator torque-limit definition ({token}) plus its generated world maxTorque")
    return 0


if __name__ == "__main__":
    sys.exit(main())
