#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ai_robotic_joint_ros2 {

// Output-side reference, SI units. Analytic trapezoidal/triangular profiles:
// velocity <= vmax and acceleration <= amax even at short moves and endpoints.
class ManualReference {
 public:
  ManualReference(double vmax, double amax, double minimum, double maximum)
      : vmax_(vmax), amax_(amax), minimum_(minimum), maximum_(maximum) {
    if (!std::isfinite(vmax) || !std::isfinite(amax) || !std::isfinite(minimum) ||
        !std::isfinite(maximum) || vmax <= 0 || amax <= 0 || minimum >= maximum)
      throw std::invalid_argument("Invalid reference limits");
  }

  void reset(double position) {
    validate(position);
    position_ = goal_ = start_ = position;
    velocity_ = elapsed_ = 0;
    phase_ = Phase::Hold;
    pending_ = false;
  }

  void moveTo(double goal) {
    validate(goal);
    if ((pending_ && std::abs(goal - pending_goal_) < 1e-12) ||
        (!pending_ && phase_ == Phase::Move && std::abs(goal - goal_) < 1e-12))
      return;
    if (std::abs(velocity_) > 1e-12) {
      stop();
      pending_ = true;
      pending_goal_ = goal;
    } else {
      beginMove(goal);
    }
  }

  void stop() {
    pending_ = false;
    if (phase_ == Phase::Stop) return;
    if (std::abs(velocity_) <= 1e-12) {
      phase_ = Phase::Hold;
      goal_ = position_;
      velocity_ = 0;
      return;
    }
    start_ = position_;
    initial_velocity_ = velocity_;
    direction_ = std::copysign(1.0, velocity_);
    duration_ = std::abs(velocity_) / amax_;
    goal_ = start_ + initial_velocity_ * duration_ / 2;
    if (goal_ < minimum_ - 1e-9 || goal_ > maximum_ + 1e-9)
      throw std::logic_error("Reference braking would cross a limit");
    goal_ = std::clamp(goal_, minimum_, maximum_);
    elapsed_ = 0;
    phase_ = Phase::Stop;
  }

  void step(double dt) {
    if (!std::isfinite(dt) || dt <= 0) throw std::invalid_argument("Invalid dt");
    while (dt > 1e-12 && phase_ != Phase::Hold) {
      const double part = std::min(dt, std::max(0.0, duration_ - elapsed_));
      elapsed_ += part;
      dt -= part;
      if (phase_ == Phase::Stop) {
        position_ = start_ + initial_velocity_ * elapsed_ -
                    direction_ * amax_ * elapsed_ * elapsed_ / 2;
        velocity_ = initial_velocity_ - direction_ * amax_ * elapsed_;
      } else if (elapsed_ <= accel_time_) {
        position_ = start_ + direction_ * amax_ * elapsed_ * elapsed_ / 2;
        velocity_ = direction_ * amax_ * elapsed_;
      } else if (elapsed_ <= accel_time_ + cruise_time_) {
        position_ = start_ + direction_ *
            (amax_ * accel_time_ * accel_time_ / 2 +
             peak_velocity_ * (elapsed_ - accel_time_));
        velocity_ = direction_ * peak_velocity_;
      } else {
        const double remaining = std::max(0.0, duration_ - elapsed_);
        position_ = goal_ - direction_ * amax_ * remaining * remaining / 2;
        velocity_ = direction_ * amax_ * remaining;
      }
      if (elapsed_ >= duration_ - 1e-12) {
        position_ = goal_;
        velocity_ = 0;
        phase_ = Phase::Hold;
        if (pending_) {
          const double next = pending_goal_;
          pending_ = false;
          beginMove(next);
        }
      }
    }
  }

  double position() const { return position_; }
  double velocity() const { return velocity_; }
  double goal() const { return pending_ ? pending_goal_ : goal_; }
  bool moving() const { return phase_ != Phase::Hold; }

 private:
  enum class Phase { Hold, Move, Stop };
  void validate(double goal) const {
    if (!std::isfinite(goal) || goal < minimum_ || goal > maximum_)
      throw std::invalid_argument("Angle outside reference limits");
  }
  void beginMove(double goal) {
    start_ = position_;
    goal_ = goal;
    elapsed_ = 0;
    velocity_ = 0;
    const double distance = std::abs(goal - start_);
    if (distance < 1e-12) { phase_ = Phase::Hold; return; }
    direction_ = std::copysign(1.0, goal - start_);
    peak_velocity_ = std::min(vmax_, std::sqrt(distance * amax_));
    accel_time_ = peak_velocity_ / amax_;
    cruise_time_ = std::max(0.0, (distance - peak_velocity_ * accel_time_) / peak_velocity_);
    duration_ = 2 * accel_time_ + cruise_time_;
    phase_ = Phase::Move;
  }
  double vmax_, amax_, minimum_, maximum_;
  double position_ = 0, velocity_ = 0, start_ = 0, goal_ = 0;
  double elapsed_ = 0, duration_ = 0, direction_ = 1, initial_velocity_ = 0;
  double peak_velocity_ = 0, accel_time_ = 0, cruise_time_ = 0;
  double pending_goal_ = 0;
  bool pending_ = false;
  Phase phase_ = Phase::Hold;
};
}  // namespace ai_robotic_joint_ros2
