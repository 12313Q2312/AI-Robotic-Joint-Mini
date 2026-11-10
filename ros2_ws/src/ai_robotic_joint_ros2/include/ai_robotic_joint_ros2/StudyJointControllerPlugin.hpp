#pragma once
#include "ai_robotic_joint_ros2/StudyPair.hpp"
#include "../../../../../knee_stair_l1/include/actuator_torque.hpp"
#include "ai_robotic_joint_ros2/msg/study_command.hpp"
#include "ai_robotic_joint_ros2/msg/study_telemetry.hpp"
#include <chrono>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <webots_ros2_driver/PluginInterface.hpp>
#include <webots/supervisor.h>

namespace webots_ros2_driver { class WebotsNode; }
namespace ai_robotic_joint_ros2 {
class StudyJointControllerPlugin final : public webots_ros2_driver::PluginInterface {
 public:
  void init(webots_ros2_driver::WebotsNode*, std::unordered_map<std::string, std::string>&) override;
  void step() override;
 private:
  struct Body { double mass; std::array<double, 3> com, diagonal, products; };
  struct PhysicalSide { WbNodeRef joint = nullptr, link = nullptr, physics = nullptr; };
  void applyBody(std::size_t side, uint8_t payload);
  bool bodyMatches(std::size_t side, uint8_t payload) const;
  void startReset(const msg::StudyCommand& command);
  webots_ros2_driver::WebotsNode* node_ = nullptr;
  std::array<WbDeviceTag, 2> motors_{}, sensors_{};
  std::array<actuator_torque::Gate, 2> actuator_gates_{};
  std::array<PhysicalSide, 2> physical_{};
  std::array<std::array<Body, 2>, 3> bodies_{};
  std::array<std::unique_ptr<EnergyCurve>, 3> curves_;
  std::unique_ptr<StudyPair> pair_;
  std::string label_;
  double loss_nm_ = 0, last_time_ = -1;
  uint8_t payload_ = 2;
  uint64_t reset_ack_ = 0, reset_requested_ = 0;
  bool reset_pending_ = false, plugin_fault_ = false;
  int reset_wait_steps_ = 0;
  std::mutex mutex_;
  msg::StudyCommand pending_;
  bool received_ = false;
  std::chrono::steady_clock::time_point last_command_, last_step_;
  rclcpp::Subscription<msg::StudyCommand>::SharedPtr subscription_;
  rclcpp::Publisher<msg::StudyTelemetry>::SharedPtr diagnostics_;
  std::array<rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr, 2> states_;
};
}
