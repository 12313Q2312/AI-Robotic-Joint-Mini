#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <webots/types.h>
#include <webots_ros2_driver/PluginInterface.hpp>

#include "../../../../../knee_stair_l1/include/actuator_torque.hpp"

namespace webots_ros2_driver {
class WebotsNode;
}

namespace ai_robotic_joint_ros2 {

class JointControllerPlugin final
    : public webots_ros2_driver::PluginInterface {
 public:
  void init(
      webots_ros2_driver::WebotsNode *node,
      std::unordered_map<std::string, std::string> &parameters) override;
  void step() override;

 private:
  void onTargetAngle(const std_msgs::msg::Float64::SharedPtr message);

  webots_ros2_driver::WebotsNode *node_ = nullptr;
  WbDeviceTag sensor_ = 0;
  WbDeviceTag motor_ = 0;

  int time_step_ms_ = 0;
  double dt_s_ = 0.0;
  double gear_ratio_ = 1.0;
  double transmission_efficiency_ = 1.0;
  double motor_torque_limit_nm_ = 0.10;
  double output_torque_limit_nm_ = 0.10;
  double load_torque_nm_ = -0.02;
  double output_mass_kg_ = 1.0;
  double output_inertia_z_kg_m2_ = 0.0002;
  std::atomic<double> target_angle_rad_{0.0};

  double error_integral_rad_s_ = 0.0;
  double previous_angle_rad_ = 0.0;
  bool have_previous_sample_ = false;
  int log_counter_ = 0;
  actuator_torque::Gate actuator_gate_{};

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_publisher_;
};

}  // namespace ai_robotic_joint_ros2
