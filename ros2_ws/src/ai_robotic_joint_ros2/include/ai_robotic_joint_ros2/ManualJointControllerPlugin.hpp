#pragma once
#include <chrono>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <webots_ros2_driver/PluginInterface.hpp>
#include <webots/types.h>
#include "ai_robotic_joint_ros2/ManualReference.hpp"
#include "../../../../../knee_stair_l1/include/actuator_torque.hpp"
#include "ai_robotic_joint_ros2/msg/manual_command.hpp"
#include "ai_robotic_joint_ros2/msg/control_telemetry.hpp"

namespace webots_ros2_driver { class WebotsNode; }
namespace ai_robotic_joint_ros2 {
class ManualJointControllerPlugin final : public webots_ros2_driver::PluginInterface {
 public:
  void init(webots_ros2_driver::WebotsNode*, std::unordered_map<std::string, std::string>&) override;
  void step() override;
 private:
  static constexpr double rad = 3.14159265358979323846 / 180;
  ManualReference reference_{3 * rad, 5 * rad, -45 * rad, 45 * rad};
  webots_ros2_driver::WebotsNode* node_ = nullptr;
  WbDeviceTag sensor_ = 0, motor_ = 0;
  bool initialized_ = false;
  double previous_time_ = -1, previous_angle_ = 0, integral_ = 0;
  double limit_ = 0.1, load_ = 0, kp_ = 0.15, ki_ = 0.05, kd_ = 0.04;
  actuator_torque::Gate actuator_gate_{};
  bool gravity_enabled_ = false;
  double gravity_x_nm_ = 0, gravity_z_nm_ = 0;
  std::mutex mutex_;
  msg::ManualCommand pending_;
  bool received_ = false;
  uint64_t consumed_sequence_ = 0;
  std::chrono::steady_clock::time_point last_command_, last_step_;
  rclcpp::Subscription<msg::ManualCommand>::SharedPtr command_subscription_;
  rclcpp::Publisher<msg::ControlTelemetry>::SharedPtr diagnostic_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_publisher_;
};
}
