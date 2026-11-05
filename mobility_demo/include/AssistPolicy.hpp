#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mobility {
// Synthetic human torque is an observable simulation input, not measured EMG.
class AssistPolicy {
 public:
  AssistPolicy(double gain, double limit, double slew)
      : gain_(gain), limit_(limit), slew_(slew) {
    if (!std::isfinite(gain) || !std::isfinite(limit) || !std::isfinite(slew) ||
        gain < 0 || gain > 1 || limit <= 0 || slew <= 0)
      throw std::invalid_argument("invalid assistance parameters");
  }
  double step(double human_torque, double dt, bool enabled) {
    if (!std::isfinite(human_torque) || !std::isfinite(dt) || dt <= 0) {
      output_ = 0;
      return 0;
    }
    const double target = enabled ? std::clamp(gain_ * human_torque, -limit_, limit_) : 0;
    output_ += std::clamp(target - output_, -slew_ * dt, slew_ * dt);
    return output_;
  }
 private:
  double gain_, limit_, slew_, output_ = 0;
};
}
