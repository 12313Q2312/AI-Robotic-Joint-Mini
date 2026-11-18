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

These are the same packages the `ros2-build-jazzy` job installs, plus the pinned Webots R2025a image:

```bash
source /opt/ros/jazzy/setup.bash
sudo apt-get update
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
  python3-colcon-common-extensions \
  ros-jazzy-webots-ros2-driver \
  ros-jazzy-rosidl-default-generators \
  ros-jazzy-pluginlib \
  curl xvfb xauth
curl --fail --location --retry 3 \
  --output /tmp/webots_2025a_amd64.deb \
  https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb
echo "6253d58c9b625a83ed7b62cd85a640fd0542d441c48d633a60932208b40b0657  /tmp/webots_2025a_amd64.deb" | sha256sum --check
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends /tmp/webots_2025a_amd64.deb
cd ros2_ws
colcon build --packages-select ai_robotic_joint_ros2
source install/setup.bash
export WEBOTS_HOME=/usr/local/webots
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
3. The last move-phase angle is within 0.02 rad of the target. That is about 1.1 degrees, the same band used for forwarded targets. The recorded run ended the move at the commanded angle, so this rejects a joint that only twitched.
4. Every recorded `effort_nm` stays within `actuator_torque_limit_nm` (24 Nm in the committed config).
5. The safety supervisor forwarded the requested target (tolerance 0.02 rad).
6. After commands stop, the safe target is 0 and the state is `DAMPING_ONLY` within 1.5 s. The supervisor timeout is 1.0 s (`legacy_timeout_s`); 1.5 s adds the 0.05 s publish timer and delivery, and is inside the 2.2 s observation window.
7. Recorded angle/effort values are finite.

It exits nonzero if a check fails and saves the complete observations,
wall-clock timestamps, ROS timestamps, checks and limitations to a JSON report.
This is **a real simulator-observation test**, not the synthetic unit fixtures
used by `tests/test_rj_int_01_assess.py`. The `JointState.effort` field contains
the plugin's commanded torque, **not an independently measured torque**.

Expected duration is the startup grace period plus motion and timeout windows;
no fixed timing claim is made until an actual run is captured.

## CI evidence boundaries

- `validation` runs existing synthetic/offline regressions, acceptance-logic unit tests and the C++ fail-closed configuration test.
- `ros2-build-jazzy` performs an actual ROS 2 C++ build and CTest, installs Webots R2025a, starts a headless Webots world and runs the live movement/timeout smoke.
- The CI step **fails** when no motor motion, no safe target, no DAMPING_ONLY state or non-finite measurements are detected. On success or failure it uploads `rj-int-01-webots-evidence` containing the full JSON and launch logs.
- **Not tested:** the `knee_stair_l1` human/strap-assisted torque loop, the real robot, human subjects or physical emergency-stop hardware.

First actual PASS: [2026-11-17 CI run 44046498775](https://github.com/12313Q2312/AI-Robotic-Joint-Mini/actions/runs/44046498775).
Evidence: 65,038 `/joint/state` samples, 0.754137 rad maximum positive target-direction motion, 391 safe-target and 391 safety-state messages, no acceptance failures.
[Raw observation artifact](https://github.com/12313Q2312/AI-Robotic-Joint-Mini/actions/runs/44046498775/artifacts/14312136058).
The `JointState.effort` field is controller commanded torque, **not** physical torque sensing.

## Next integration boundary

RJ-INT-01B, separate from this legacy-bench smoke, would need an explicit
interface adapter between `knee_stair_l1` torque commands, a correct
Webots knee plant, sensor/intent source timestamps and safety supervision.
Do not reuse the legacy angle bench as proof of synthetic stair assistance.
