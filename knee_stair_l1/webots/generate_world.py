"""Trimmed single-knee Webots world.

Geometry is reduced from the hinge builder on PR #3
(demo/cooperative-assistance-v0.1, mobility_demo/prepare.py): one thigh,
one knee hinge, one shank, and a visual exoskeleton link. The 10-joint
biped, arms, and stair flight are not included. CI does not launch Webots.
The closed loop under test is the headless plant in knee_stair_l1.plant.
"""

from __future__ import annotations

import json
from pathlib import Path

WORLD_PATH = Path(__file__).resolve().parent / "worlds" / "knee_stair_l1.wbt"
CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "default.json"


def actuator_torque_limit_nm() -> float:
    """Read the one hard-limit definition. Do not write that number here."""
    config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    return float(config["actuator_torque_limit_nm"])


def generate_world() -> str:
    limit = actuator_torque_limit_nm()
    text = """#VRML_SIM R2025a utf8
# Optional visualization only. Trimmed from PR #3 mobility_demo/prepare.py leg().
# Single-side knee. Not the 10-joint model. Not executed by CI.
# DEF HUMAN_SHANK is the human leg body. DEF EXO_LINK is the exoskeleton body.
# The only motor is the exoskeleton knee. Human effort is not a second motor.
WorldInfo { basicTimeStep 8 gravity 9.81 coordinateSystem "ENU" }
Viewpoint { orientation 0 1 0 0.6 position 1.4 -1.6 0.7 }
Background { skyColor [ 0.86 0.91 0.96 ] }
DirectionalLight { direction -0.4 0.3 -1 intensity 1.2 }
Solid {
  name "floor"
  translation 0 0 -0.05
  children [ Shape { appearance PBRAppearance { baseColor 0.9 0.9 0.92 } geometry Box { size 2 2 0.1 } } ]
  boundingObject Box { size 2 2 0.1 }
}
Solid {
  name "one_stair"
  translation 0.45 0 0.05
  children [ Shape { appearance PBRAppearance { baseColor 0.62 0.68 0.74 } geometry Box { size 0.4 0.8 0.1 } } ]
  boundingObject Box { size 0.4 0.8 0.1 }
}
Robot {
  name "single_knee_stair"
  translation 0 0 0.5
  children [
    DEF HUMAN_THIGH Solid {
      translation 0 0 0
      children [ Shape { appearance PBRAppearance { baseColor 0.22 0.31 0.42 } geometry Box { size 0.12 0.13 0.28 } } ]
      name "human_thigh"
      boundingObject Box { size 0.12 0.13 0.28 }
      physics Physics { density -1 mass 7.0 }
    }
    DEF EXO_KNEE HingeJoint {
      jointParameters HingeJointParameters { axis 0 1 0 anchor 0 0 -0.14 minStop 0.05 maxStop 2.2 dampingConstant 0.2 }
      device [
        RotationalMotor { name "exo_knee" maxTorque LIMIT_TOKEN maxVelocity 6 }
        PositionSensor { name "exo_knee_sensor" }
      ]
      endPoint DEF HUMAN_SHANK Solid {
        translation 0 0 -0.14
        children [
          Shape { appearance PBRAppearance { baseColor 0.22 0.31 0.42 } geometry Capsule { radius 0.044 height 0.33 } }
          DEF EXO_LINK Solid {
            translation 0.06 0 -0.16
            children [ Shape { appearance PBRAppearance { baseColor 0.08 0.55 0.55 } geometry Box { size 0.03 0.03 0.28 } } ]
            name "exoskeleton_link"
            boundingObject Box { size 0.03 0.03 0.28 }
            physics Physics { density -1 mass 0.4 }
          }
        ]
        name "human_shank"
        boundingObject Capsule { radius 0.044 height 0.33 }
        physics Physics { density -1 mass 3.5 }
      }
    }
  ]
  controller "<none>"
  supervisor FALSE
}
"""
    rendered = f"{limit:.0f}" if limit == int(limit) else str(limit)
    return text.replace("LIMIT_TOKEN", rendered)


def write_world(path: Path = WORLD_PATH) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(generate_world(), encoding="utf-8")
    return path


if __name__ == "__main__":
    print(write_world())
