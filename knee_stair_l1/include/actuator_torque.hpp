#pragma once

// Shared actuator hard limit. The only numeric definition is
// actuator_torque_limit_nm in knee_stair_l1/config/default.json.
// Finite torque is clamped to [-limit, +limit]. NaN and inf yield 0 and
// latch E_STOP, which is what the Python publish path does after
// clamp_actuator_torque raises. This header does not return a finite
// substitute for a non-finite input.

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace actuator_torque {

enum class State { kNormal, kEStop };

struct Gate {
  double limit_nm = 0.0;
  bool estop_latched = false;

  double apply(double torque_nm) {
    if (estop_latched || !std::isfinite(torque_nm) || !std::isfinite(limit_nm) || limit_nm <= 0.0) {
      estop_latched = true;
      return 0.0;
    }
    if (torque_nm > limit_nm) return limit_nm;
    if (torque_nm < -limit_nm) return -limit_nm;
    return torque_nm;
  }

  State state() const { return estop_latched ? State::kEStop : State::kNormal; }

  void reset() { estop_latched = false; }
};

inline double limit_from_json_text(const std::string& text) {
  const std::string key = "\"actuator_torque_limit_nm\"";
  const auto key_at = text.find(key);
  if (key_at == std::string::npos)
    throw std::runtime_error("actuator config is missing actuator_torque_limit_nm");
  const auto colon = text.find(':', key_at + key.size());
  if (colon == std::string::npos)
    throw std::runtime_error("actuator config limit is missing a value");
  const auto number = text.find_first_of("0123456789", colon + 1);
  if (number == std::string::npos)
    throw std::runtime_error("actuator config limit is not numeric");
  std::size_t end = number;
  while (end < text.size() && (std::isdigit(static_cast<unsigned char>(text[end])) || text[end] == '.'))
    ++end;
  const double limit = std::stod(text.substr(number, end - number));
  if (!std::isfinite(limit) || limit <= 0.0)
    throw std::runtime_error("actuator config limit must be positive and finite");
  return limit;
}

inline double limit_from_file(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open actuator config: " + path);
  std::stringstream buffer;
  buffer << input.rdbuf();
  return limit_from_json_text(buffer.str());
}

inline std::string find_default_config() {
  const char* candidates[] = {
      "knee_stair_l1/config/default.json",
      "../knee_stair_l1/config/default.json",
      "../../knee_stair_l1/config/default.json",
      "../../../knee_stair_l1/config/default.json",
      "../../../../knee_stair_l1/config/default.json",
      "../../../../../knee_stair_l1/config/default.json",
  };
  for (const char* candidate : candidates) {
    std::ifstream input(candidate);
    if (input) return candidate;
  }
  return {};
}

inline Gate gate_from_default_config() {
  Gate gate;
  const std::string path = find_default_config();
  if (path.empty()) {
    gate.estop_latched = true;
    return gate;
  }
  try {
    gate.limit_nm = limit_from_file(path);
  } catch (const std::exception&) {
    gate.estop_latched = true;
  }
  return gate;
}

}  // namespace actuator_torque
