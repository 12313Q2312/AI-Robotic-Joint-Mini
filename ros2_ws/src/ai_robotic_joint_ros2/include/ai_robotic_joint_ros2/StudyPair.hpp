#pragma once
#include "ai_robotic_joint_ros2/ManualReference.hpp"
#include <array>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace ai_robotic_joint_ros2 {
struct EnergyPoint { double angle_rad, energy_j, compression_m; };
struct PassiveState { double energy_j = 0, compression_m = 0, torque_nm = 0; };
inline double equivalentLoss(double speed_rad_s, double assumed_coulomb_nm) {
  if (!std::isfinite(speed_rad_s) || !std::isfinite(assumed_coulomb_nm) || assumed_coulomb_nm < 0)
    throw std::invalid_argument("Invalid equivalent loss parameters");
  // Smooth assumed loss, not measured hysteresis or a stiction/contact model.
  return -assumed_coulomb_nm * std::tanh(speed_rad_s / .005);
}

// Interpolate ENERGY, not torque. Its exact piecewise derivative is conservative.
// At an interior knot the right derivative is used; the last knot uses the left.
class EnergyCurve {
 public:
  explicit EnergyCurve(std::vector<EnergyPoint> points) : points_(std::move(points)) {
    if (points_.size() < 2) throw std::invalid_argument("At least two energy samples required");
    for (std::size_t i = 0; i < points_.size(); ++i) {
      const auto& p = points_[i];
      if (!std::isfinite(p.angle_rad) || !std::isfinite(p.energy_j) ||
          !std::isfinite(p.compression_m) || p.energy_j < 0 || p.compression_m <= 0 ||
          (i && p.angle_rad <= points_[i - 1].angle_rad))
        throw std::invalid_argument("Invalid energy sample ordering or units");
    }
  }
  PassiveState sample(double angle) const {
    if (!std::isfinite(angle) || angle < points_.front().angle_rad || angle > points_.back().angle_rad)
      throw std::out_of_range("Angle outside conservative energy table");
    auto found = std::upper_bound(points_.begin(), points_.end(), angle,
        [](double q, const EnergyPoint& point) { return q < point.angle_rad; });
    const auto index = found == points_.end() ? points_.size() - 2 :
        static_cast<std::size_t>(found - points_.begin() - 1);
    const auto& a = points_[index]; const auto& b = points_[index + 1];
    const double ratio = (angle - a.angle_rad) / (b.angle_rad - a.angle_rad);
    return {a.energy_j + ratio * (b.energy_j - a.energy_j),
            a.compression_m + ratio * (b.compression_m - a.compression_m),
            -(b.energy_j - a.energy_j) / (b.angle_rad - a.angle_rad)};
  }
  bool covers(double low, double high) const {
    return points_.front().angle_rad <= low && points_.back().angle_rad >= high;
  }
 private:
  std::vector<EnergyPoint> points_;
};

struct StudyInput {
  uint64_t sequence = 0;
  uint8_t mode = 2, action = 0;
  double target_angle_rad = 0;
};
struct StudyPid {
  double actual_angle_rad = 0, velocity_rad_s = 0;
  double p_torque_nm = 0, i_torque_nm = 0, d_torque_nm = 0, commanded_torque_nm = 0;
  bool saturated = false, anti_windup = false;
};
struct StudyOutput {
  double sim_time_s = 0, requested_angle_rad = 0, reference_angle_rad = 0, reference_velocity_rad_s = 0;
  std::array<StudyPid, 2> sides;
  std::string status = "waiting_feedback";
  uint64_t command_sequence = 0;
  uint8_t mode = 2;
};

// One reference is advanced once per simulation step. Passive mechanics never
// enters this module's motor command or active torque limit.
class StudyPair {
 public:
  static constexpr double rad = 3.14159265358979323846 / 180;
  StudyPair(double kp = 4, double ki = 4, double kd = .05, double limit = 1)
      : kp_(kp), ki_(ki), kd_(kd), limit_(limit) {
    if (!std::isfinite(kp) || !std::isfinite(ki) || !std::isfinite(kd) ||
        !std::isfinite(limit) || kp < 0 || ki < 0 || kd < 0 || limit <= 0)
      throw std::invalid_argument("Invalid paired PID parameters");
  }
  void reset(double angle, uint64_t consumed_sequence, double time = NAN) {
    reference_.reset(angle);
    integrals_ = {0, 0};
    previous_angles_ = {angle, angle};
    consumed_ = consumed_sequence;
    initialized_ = std::isfinite(time);
    if (initialized_) previous_time_ = time;
    requested_ = angle;
    fault_ = false;
  }
  StudyOutput step(double time, std::array<double, 2> angles,
                   const StudyInput& command, bool fresh, bool pause_or_multiple_publishers) {
    StudyOutput out;
    out.sim_time_s = time;
    out.mode = command.mode;
    bool first = !initialized_;
    try {
      if (fault_) throw std::runtime_error("Latched fault: explicit reset required");
      for (double q : angles)
        if (!std::isfinite(q) || std::abs(q) > 60 * rad + .005)
          throw std::runtime_error("Invalid joint feedback");
      if (!std::isfinite(time) || time < 0) throw std::runtime_error("Invalid simulation time");
      if (initialized_ && time <= previous_time_) {
        initialized_ = false; first = true;
      }
      if (first) {
        if (std::abs(angles[0] - angles[1]) > .01)
          throw std::runtime_error("Initial pair angles differ; reset required");
        reference_.reset((angles[0] + angles[1]) / 2);
        integrals_ = {0, 0};
        consumed_ = command.sequence;
        requested_ = reference_.position();
        initialized_ = true;
      }
      const double dt = first ? .016 : time - previous_time_;
      if (dt <= 0 || dt > .1) throw std::runtime_error("Invalid physics timestep");
      if (pause_or_multiple_publishers || !fresh) {
        reference_.stop();
        consumed_ = std::max(consumed_, command.sequence);
        requested_ = reference_.goal();
        out.status = "holding_watchdog";
      } else if (command.action == 0) {
        reference_.stop();
        consumed_ = std::max(consumed_, command.sequence);
        requested_ = reference_.goal();
        out.status = "ready";
      } else if (command.sequence > consumed_) {
        if (command.action != 1 || command.mode == 3 || command.mode < 1 || command.mode > 3)
          throw std::invalid_argument("Invalid or disabled motion mode");
        reference_.moveTo(command.target_angle_rad);
        consumed_ = command.sequence;
        requested_ = command.target_angle_rad;
        out.status = "ready";
      } else out.status = "ready";
      reference_.step(dt);
      for (std::size_t i = 0; i < 2; ++i) {
        auto& side = out.sides[i];
        side.actual_angle_rad = angles[i];
        side.velocity_rad_s = first ? 0 : (angles[i] - previous_angles_[i]) / dt;
        const double error = reference_.position() - angles[i];
        side.p_torque_nm = kp_ * error;
        side.d_torque_nm = -kd_ * side.velocity_rad_s;
        const double next_integral = integrals_[i] + error * dt;
        const double provisional = side.p_torque_nm + side.d_torque_nm + ki_ * next_integral;
        side.anti_windup = (provisional > limit_ && error >= 0) || (provisional < -limit_ && error <= 0);
        if (!side.anti_windup) integrals_[i] = next_integral;
        side.i_torque_nm = ki_ * integrals_[i];
        const double active = side.p_torque_nm + side.i_torque_nm + side.d_torque_nm;
        side.saturated = std::abs(active) > limit_;
        side.commanded_torque_nm = std::clamp(active, -limit_, limit_);
      }
      previous_time_ = time; previous_angles_ = angles;
    } catch (const std::exception&) {
      fault_ = true;
      out.status = "fault_pause_webots";
      out.sides = {};
      for (std::size_t i = 0; i < 2; ++i) out.sides[i].actual_angle_rad = angles[i];
    }
    out.reference_angle_rad = reference_.position();
    out.reference_velocity_rad_s = reference_.velocity();
    out.requested_angle_rad = requested_;
    out.command_sequence = consumed_;
    return out;
  }
  bool moving() const { return reference_.moving(); }
  bool faulted() const { return fault_; }
  double referenceVelocity() const { return reference_.velocity(); }
 private:
  ManualReference reference_{3 * rad, 5 * rad, -45 * rad, 45 * rad};
  double kp_, ki_, kd_, limit_, previous_time_ = 0, requested_ = 0;
  std::array<double, 2> integrals_ = {0, 0}, previous_angles_ = {0, 0};
  uint64_t consumed_ = 0;
  bool initialized_ = false, fault_ = false;
};
}  // namespace ai_robotic_joint_ros2
