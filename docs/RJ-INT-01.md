# RJ-INT-01A: native ROS 2 <-> Webots single-joint smoke

**Scope:** the **existing** legacy single-axis ROS 2 angle pipeline only:

```text
/joint/target_angle (Float64)
  -> joint_safety_node.py
  -> /joint/safe_target_angle (Float64)
  -> JointControllerPlugin.cpp (C++ PID, actuator Gate)
  -> Webots joint_motor + joint_sensor
  -> /joint/state (sensor_msgs/JointState)
```

This milestone does **not** connect `knee_stair_l1`'s assistance torque to the physics
world, and does not prove a human/exoskeleton model, safety certification, fall
prevention or hardware. The existing synthetic L1/L2 results are separate.

## Environment and build

Target: Ubuntu 24.04, ROS 2 Jazzy, `ros-jazzy-webots-ros2`,
Webots R2025a or a version compatible with the checked-in world.
This launch starts Webots **on the same Linux machine**. For the historical
Windows-Webots + WSL2 setup, keep using the existing `joint_launch.py` with its
documented host/port arrangement; do not launch both modes together.

Install ROS 2 Jazzy and Webots using the official documentation:
- https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
- https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/Webots/Installation-Ubuntu.html

After dependencies are installed, from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
sudo apt-get install ros-jazzy-webots-ros2 python3-colcon-common-extensions
cd ros2_ws
colcon build --packages-select ai_robotic_joint_ros2
source install/setup.bash
ros2 launch ai_robotic_joint_ros2 joint_native_launch.py
```

In a **second terminal**, with the same sourced ROS environment and workspace
overlay, run (with the repository root as current directory for the report path):

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 run ai_robotic_joint_ros2 rj_int_01_smoke.py \
  --target-rad 0.6 --move-seconds 8 --timeout-seconds 2.2 \
  --output local/rj_int_01_smoke.json
```

The launch sets `AI_ROBOTIC_JOINT_CONFIG` to the package-installed
`config/default.json`; the actuator C++ gate now prioritizes this explicit
path. If that file is missing or invalid, **the gate latches 0 Nm**. The fallback
relative search remains for historical local invocations only.

## Actual acceptance (not CI's offline green check)

The live smoke client waits for Webots telemetry, publishes an angle target
at 10 Hz, then stops publishing and checks all of:

1. At least one actual `/joint/state` message was received from the plugin.
2. The angle moved at least 0.03 rad in the target direction.
3. The safety supervisor forwarded the requested target (tolerance 0.02 rad).
4. After the command timeout, the safe target became 0 rad.
5. The safety state became `DAMPING_ONLY` after the timeout.
6. Recorded angle/effort values are finite.

It exits nonzero if a check fails and saves the complete observations,
wall-clock timestamps, ROS timestamps, checks and limitations to a JSON report.
This is **a real simulator-observation test**, not the synthetic unit fixtures
used by `tests/test_rj_int_01_assess.py`. The `JointState.effort` field contains
the plugin's commanded torque, **not an independently measured torque**.

Expected duration is the startup grace period plus motion and timeout windows;
no fixed timing claim is made until an actual run is captured.

## CI evidence boundaries

- `validation`: existing synthetic/offline regressions, RJ-INT-01 assessment
  regression tests and C++ config-path fail-closed test; Python source syntax.
- `ros2-build-jazzy`: actual ROS 2 C++ package build and CTest.
- **Not covered by CI:** Webots world launched, live motor movement,
  ROS-to-Webots message round-trip or video capture.

The integration is only **fully verified** after a real ROS 2+Webots launch,
the live smoke exits 0, and its report is reviewed. Failures should remain
visible; do not relabel a passing ROS build as a successful Webots run.

## Next integration boundary

RJ-INT-01B, separate from this legacy-bench smoke, would need an explicit
interface adapter between `knee_stair_l1` torque commands, a correct
Webots knee plant, sensor/intent source timestamps and safety supervision.
Do not reuse the legacy angle bench as proof of synthetic stair assistance.
