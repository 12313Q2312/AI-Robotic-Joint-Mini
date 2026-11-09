"""ROS 2 topic names shared by the in-process bus and the ROS nodes.

Types are std_msgs so the contract does not require a new .msg build.
Float64MultiArray field order is fixed here and in the publishers.
"""

# Legacy single-joint angle port. Numeric behavior is preserved.
TOPIC_LEGACY_TARGET = "/joint/target_angle"
TOPIC_LEGACY_SAFE_TARGET = "/joint/safe_target_angle"

# Read-only supervisor state for coordination. The node publishes it.
# Nothing writes this topic; it is not a command.
TOPIC_SAFETY_STATE = "/joint/safety_state"

# v1 intent. The topic name itself says synthetic.
TOPIC_INTENT_SYNTHETIC = "/knee_stair/intent_synthetic"
# Float64MultiArray: time_s, effort_extension_nm, phase, ascent_active (1 or 0)

TOPIC_DESIRED_MOTION = "/knee_stair/desired_motion"
# Float64MultiArray: flexion_rad, flexion_velocity_rad_s, generated (1 or 0)

TOPIC_ASSIST_COMMAND_RAW = "/knee_stair/assist_command_raw"
# Float64, extension-positive newton metres, before the supervisor.

TOPIC_ASSIST_COMMAND_SAFE = "/knee_stair/assist_command_safe"
# Float64, the only exo torque the plant may apply.

TOPIC_PLANT_STATE = "/knee_stair/plant_state"
# Float64MultiArray: flexion_rad, flexion_velocity_rad_s, interaction_extension_nm

TOPIC_TELEMETRY = "/knee_stair/telemetry"
# std_msgs/String, one JSON object per sample. source_label is "synthetic" for v1.

LAYER1_TOPICS = (
    TOPIC_LEGACY_TARGET,
    TOPIC_LEGACY_SAFE_TARGET,
    TOPIC_SAFETY_STATE,
    TOPIC_INTENT_SYNTHETIC,
    TOPIC_DESIRED_MOTION,
    TOPIC_ASSIST_COMMAND_RAW,
    TOPIC_ASSIST_COMMAND_SAFE,
    TOPIC_PLANT_STATE,
    TOPIC_TELEMETRY,
)
