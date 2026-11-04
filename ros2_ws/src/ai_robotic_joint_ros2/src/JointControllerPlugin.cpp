#include "ai_robotic_joint_ros2/JointControllerPlugin.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

#include <pluginlib/class_list_macros.hpp>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <webots_ros2_driver/WebotsNode.hpp>

namespace ai_robotic_joint_ros2 {
namespace {

constexpr double kMinAngleRad = -1.5708;
constexpr double kMaxAngleRad = 1.5708;
constexpr double kKp = 0.15;
constexpr double kKi = 0.05;
constexpr double kKd = 0.04;

std::string parameterOr(
    const std::unordered_map<std::string, std::string> &parameters,
    const std::string &key,
    const std::string &fallback) {
  const auto entry = parameters.find(key);
  return entry == parameters.end() ? fallback : entry->second;
}

double numericParameterOr(
    const std::unordered_map<std::string, std::string> &parameters,
    const std::string &key, double fallback) {
  try {
    return std::stod(parameterOr(parameters, key, std::to_string(fallback)));
  } catch (const std::exception &) {
    return fallback;
  }
}

}  // namespace

void JointControllerPlugin::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters) {
  node_ = node;
  if (node_ == nullptr)
    throw std::runtime_error("Webots ROS 2 node is null.");

  const std::string sensor_name =
      parameterOr(parameters, "sensorName", "joint_sensor");
  const std::string motor_name =
      parameterOr(parameters, "motorName", "joint_motor");

  sensor_ = wb_robot_get_device(sensor_name.c_str());
  motor_ = wb_robot_get_device(motor_name.c_str());
  if (sensor_ == 0 || motor_ == 0) {
    throw std::runtime_error(
        "joint_sensor or joint_motor was not found in the Webots robot.");
  }

  time_step_ms_ = static_cast<int>(wb_robot_get_basic_time_step());
  dt_s_ = static_cast<double>(time_step_ms_) / 1000.0;
  if (time_step_ms_ <= 0 || dt_s_ <= 0.0)
    throw std::runtime_error("Webots basic time step must be positive.");

  gear_ratio_ = numericParameterOr(parameters, "gearRatio", 1.0);
  transmission_efficiency_ = numericParameterOr(parameters, "transmissionEfficiency", 1.0);
  motor_torque_limit_nm_ = numericParameterOr(parameters, "motorTorqueLimitNm", 0.10);
  load_torque_nm_ = numericParameterOr(parameters, "loadTorqueNm", -0.02);
  output_mass_kg_ = numericParameterOr(parameters, "outputMassKg", 1.0);
  output_inertia_z_kg_m2_ =
      numericParameterOr(parameters, "outputInertiaZKgM2", 0.0002);
  if (gear_ratio_ <= 0.0 || transmission_efficiency_ <= 0.0 ||
      transmission_efficiency_ > 1.0 || motor_torque_limit_nm_ <= 0.0 ||
      output_mass_kg_ <= 0.0 || output_inertia_z_kg_m2_ <= 0.0)
    throw std::runtime_error("Invalid drivetrain parameters.");
  output_torque_limit_nm_ =
      motor_torque_limit_nm_ * gear_ratio_ * transmission_efficiency_;

  wb_position_sensor_enable(sensor_, time_step_ms_);

  target_subscription_ = node_->create_subscription<std_msgs::msg::Float64>(
      "joint/safe_target_angle", 10,
      std::bind(
          &JointControllerPlugin::onTargetAngle,
          this,
          std::placeholders::_1));

  state_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      "joint/state", rclcpp::SensorDataQoS());

  RCLCPP_INFO(
      node_->get_logger(),
      "Joint ROS 2 torque controller ready: target topic=/joint/target_angle, "
      "state topic=/joint/state, gear_ratio=%.3f efficiency=%.3f "
      "output_torque_limit=%.6f Nm, output_mass=%.3f kg, "
      "output_inertia_z=%.7f kg m^2",
      gear_ratio_, transmission_efficiency_, output_torque_limit_nm_,
      output_mass_kg_, output_inertia_z_kg_m2_);
}

void JointControllerPlugin::onTargetAngle(
    const std_msgs::msg::Float64::SharedPtr message) {
  if (!std::isfinite(message->data)) {
    RCLCPP_WARN(node_->get_logger(), "Rejected non-finite target angle.");
    return;
  }

  const double safe_target =
      std::clamp(message->data, kMinAngleRad, kMaxAngleRad);
  if (safe_target != message->data) {
    RCLCPP_WARN(
        node_->get_logger(),
        "Target %.6f rad exceeded mechanical limits and was clamped to %.6f rad.",
        message->data,
        safe_target);
  }

  target_angle_rad_.store(safe_target);
  RCLCPP_INFO(
      node_->get_logger(), "Accepted target angle: %.6f rad", safe_target);
}

void JointControllerPlugin::step() {
  const double actual_angle_rad = wb_position_sensor_get_value(sensor_);
  if (!std::isfinite(actual_angle_rad))
    return;

  const double target_angle_rad = target_angle_rad_.load();
  const double error_rad = target_angle_rad - actual_angle_rad;

  double angular_velocity_rad_s = 0.0;
  if (have_previous_sample_) {
    angular_velocity_rad_s =
        (actual_angle_rad - previous_angle_rad_) / dt_s_;
  }
  previous_angle_rad_ = actual_angle_rad;
  have_previous_sample_ = true;

  const double p_torque_nm = kKp * error_rad;
  const double d_torque_nm = -kKd * angular_velocity_rad_s;

  const double candidate_integral =
      error_integral_rad_s_ + error_rad * dt_s_;
  const double candidate_i_torque_nm = kKi * candidate_integral;
  const double candidate_raw_torque_nm =
      p_torque_nm + candidate_i_torque_nm + d_torque_nm;

  const bool saturated_high = candidate_raw_torque_nm > output_torque_limit_nm_;
  const bool saturated_low = candidate_raw_torque_nm < -output_torque_limit_nm_;
  const bool integration_reduces_saturation =
      (saturated_high && error_rad < 0.0) ||
      (saturated_low && error_rad > 0.0);
  const bool anti_windup_active =
      (saturated_high || saturated_low) &&
      !integration_reduces_saturation;

  if (!anti_windup_active)
    error_integral_rad_s_ = candidate_integral;

  const double i_torque_nm = kKi * error_integral_rad_s_;
  const double raw_control_torque_nm =
      p_torque_nm + i_torque_nm + d_torque_nm;
  const double control_torque_nm = std::clamp(
      raw_control_torque_nm, -output_torque_limit_nm_, output_torque_limit_nm_);
  const double applied_torque_nm = control_torque_nm + load_torque_nm_;

  wb_motor_set_torque(motor_, applied_torque_nm);

  sensor_msgs::msg::JointState state;
  state.header.stamp = node_->get_clock()->now();
  state.name = {"joint_axis"};
  state.position = {actual_angle_rad};
  state.velocity = {angular_velocity_rad_s};
  state.effort = {applied_torque_nm};
  state_publisher_->publish(std::move(state));

  if (log_counter_++ % 10 == 0) {
    RCLCPP_INFO(
        node_->get_logger(),
        "time=%.3f target=%.6f actual=%.6f error=%.6f velocity=%.6f "
        "control_torque=%.6f applied_torque=%.6f anti_windup=%d",
        wb_robot_get_time(),
        target_angle_rad,
        actual_angle_rad,
        error_rad,
        angular_velocity_rad_s,
        control_torque_nm,
        applied_torque_nm,
        anti_windup_active ? 1 : 0);
  }
}

}  // namespace ai_robotic_joint_ros2

PLUGINLIB_EXPORT_CLASS(
    ai_robotic_joint_ros2::JointControllerPlugin,
    webots_ros2_driver::PluginInterface)
