#include "ai_robotic_joint_ros2/ManualJointControllerPlugin.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <pluginlib/class_list_macros.hpp>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <webots_ros2_driver/WebotsNode.hpp>

namespace ai_robotic_joint_ros2 {
void ManualJointControllerPlugin::init(webots_ros2_driver::WebotsNode* node,
    std::unordered_map<std::string, std::string>& parameters) {
  node_ = node;
  auto value = [&](const std::string& key, const std::string& fallback) {
    const auto found = parameters.find(key);
    return found == parameters.end() ? fallback : found->second;
  };
  auto number = [&](const std::string& key, double fallback) {
    const double parsed = std::stod(value(key, std::to_string(fallback)));
    if (!std::isfinite(parsed)) throw std::invalid_argument("Non-finite manual parameter");
    return parsed;
  };
  sensor_ = wb_robot_get_device(value("sensorName", "joint_sensor").c_str());
  motor_ = wb_robot_get_device(value("motorName", "joint_motor").c_str());
  if (!node_ || !sensor_ || !motor_) throw std::runtime_error("Manual joint devices missing");
  limit_ = number("outputTorqueLimitNm", 0.1);
  load_ = number("loadTorqueNm", 0);
  kp_ = number("kp", 0.15); ki_ = number("ki", 0.05); kd_ = number("kd", 0.04);
  if (limit_ <= 0 || kp_ < 0 || ki_ < 0 || kd_ < 0) throw std::invalid_argument("Invalid PID/torque limits");
  gravity_enabled_ = value("gravityFeedforwardEnabled", "false") == "true";
  gravity_x_nm_ = number("gravityMomentXNm", 0);
  gravity_z_nm_ = number("gravityMomentZNm", 0);
  const int step_ms = static_cast<int>(wb_robot_get_basic_time_step());
  if (step_ms <= 0) throw std::runtime_error("Invalid Webots timestep");
  wb_position_sensor_enable(sensor_, step_ms);
  const auto prefix = value("topicPrefix", "/manual_demo/joint");
  command_subscription_ = node_->create_subscription<msg::ManualCommand>(
      prefix + "/manual/safe_command", 1, [this](msg::ManualCommand::ConstSharedPtr command) {
        if ((command->action != msg::ManualCommand::STOP && command->action != msg::ManualCommand::MOVE) ||
            command->mode < 1 || command->mode > 3 ||
            !std::isfinite(command->target_angle_rad) ||
            (command->action == msg::ManualCommand::MOVE &&
             (command->mode == msg::ManualCommand::AI_PLACEHOLDER ||
              std::abs(command->target_angle_rad) > 45 * rad))) return;
        std::lock_guard<std::mutex> lock(mutex_);
        pending_ = *command;
        received_ = true;
        last_command_ = std::chrono::steady_clock::now();
      });
  diagnostic_publisher_ = node_->create_publisher<msg::ControlTelemetry>(prefix + "/control_diagnostics", rclcpp::SensorDataQoS());
  state_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(prefix + "/state", rclcpp::SensorDataQoS());
  last_step_ = std::chrono::steady_clock::now();
}

void ManualJointControllerPlugin::step() {
  const auto wall = std::chrono::steady_clock::now();
  const double time = wb_robot_get_time();
  const double actual = wb_position_sensor_get_value(sensor_);
  msg::ManualCommand command;
  bool fresh;
  { std::lock_guard<std::mutex> lock(mutex_);
    command = pending_;
    fresh = received_ && std::chrono::duration<double>(wall - last_command_).count() < 0.5;
  }
  std::string status = "ready";
  const bool reset = !initialized_ || time <= previous_time_;
  const bool pause_gap = std::chrono::duration<double>(wall - last_step_).count() >= 0.5;
  last_step_ = wall;
  double velocity = 0, p = 0, i = 0, d = 0, ff = 0, torque = 0;
  bool saturated = false, anti = false;
  try {
    if (!std::isfinite(actual) || !std::isfinite(time)) throw std::runtime_error("Invalid sensor/time");
    if (reset) {
      reference_.reset(actual);
      integral_ = 0;
      consumed_sequence_ = command.sequence;  // Never replay a pre-reset command.
      initialized_ = true;
    }
    const double dt = reset ? wb_robot_get_basic_time_step() / 1000.0 : time - previous_time_;
    if (!reset) velocity = (actual - previous_angle_) / dt;
    if (pause_gap || !fresh) {
      reference_.stop();
      consumed_sequence_ = command.sequence;
      status = "holding_watchdog";
    } else if (command.action == msg::ManualCommand::STOP) {
      reference_.stop();
      consumed_sequence_ = command.sequence;
    } else if (command.sequence != consumed_sequence_) {
      consumed_sequence_ = command.sequence;
      if (command.action == msg::ManualCommand::MOVE) reference_.moveTo(command.target_angle_rad);
      else reference_.stop();
    }
    reference_.step(dt);
    const double error = reference_.position() - actual;
    p = kp_ * error; d = -kd_ * velocity;
    // +q lifts about the -Y axis, q=0 horizontal. CAD-derived mass moments.
    ff = gravity_enabled_ ? gravity_x_nm_ * std::cos(actual) - gravity_z_nm_ * std::sin(actual) : 0;
    const double candidate = integral_ + error * dt;
    const double candidate_total = p + ki_ * candidate + d + ff;
    anti = (candidate_total > limit_ && error >= 0) || (candidate_total < -limit_ && error <= 0);
    if (!anti) integral_ = candidate;
    i = ki_ * integral_;
    const double total = p + i + d + ff;
    torque = std::clamp(total, -limit_, limit_);
    saturated = std::abs(total) > limit_;
    wb_motor_set_torque(motor_, torque + load_);
    previous_time_ = time; previous_angle_ = actual;
  } catch (const std::exception& error) {
    status = "fault";
    initialized_ = false;
    wb_motor_set_torque(motor_, 0);
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "Manual controller fault: %s. Pause simulation.", error.what());
  }
  msg::ControlTelemetry telemetry;
  telemetry.sim_time_s = time;
  telemetry.requested_angle_rad = command.target_angle_rad;
  telemetry.reference_angle_rad = reference_.position();
  telemetry.reference_velocity_rad_s = reference_.velocity();
  telemetry.actual_angle_rad = actual; telemetry.velocity_rad_s = velocity;
  telemetry.p_torque_nm = p; telemetry.i_torque_nm = i; telemetry.d_torque_nm = d;
  telemetry.feedforward_torque_nm = ff;
  telemetry.commanded_torque_nm = torque + (status == "fault" ? 0 : load_);
  telemetry.saturated = saturated; telemetry.anti_windup = anti;
  telemetry.mode = command.mode; telemetry.status = status;
  telemetry.command_sequence = consumed_sequence_;
  diagnostic_publisher_->publish(telemetry);
  sensor_msgs::msg::JointState state;
  const double stamp_time = std::isfinite(time) ? std::clamp(time, 0.0, 2147483647.0) : 0;
  state.header.stamp.sec = static_cast<int32_t>(stamp_time);
  state.header.stamp.nanosec = static_cast<uint32_t>((stamp_time - state.header.stamp.sec) * 1e9);
  state.name = {"joint_axis"}; state.position = {actual};
  state.velocity = {velocity}; state.effort = {telemetry.commanded_torque_nm};
  state_publisher_->publish(state);
}
}
PLUGINLIB_EXPORT_CLASS(ai_robotic_joint_ros2::ManualJointControllerPlugin, webots_ros2_driver::PluginInterface)
