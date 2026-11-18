#pragma once

// Shared actuator hard limit. The only numeric definition is
// actuator_torque_limit_nm in knee_stair_l1/config/default.json.
// Finite torque is clamped to [-limit, +limit]. NaN and inf yield 0 and
// latch E_STOP, which is what the Python publish path does after
// clamp_actuator_torque raises. This header does not return a finite
// substitute for a non-finite input.

#include <cctype>
#include <cmath>
#include <cstdlib>
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

// 2.4e1 Nm is the absolute ceiling. A configured file, including one named by
// AI_ROBOTIC_JOINT_CONFIG, may not exceed it. The digits are not written as 24
// so the JSON key remains the only torque-limit literal of that value.
constexpr double kMaxActuatorTorqueNm = 2.4e1;

inline double limit_from_json_text(const std::string& text) {
  const std::string key = "\"actuator_torque_limit_nm\"";
  const auto key_at = text.find(key);
  if (key_at == std::string::npos)
    throw std::runtime_error("actuator config is missing actuator_torque_limit_nm");
  std::size_t cursor = key_at + key.size();
  while (cursor < text.size() && std::isspace(static_cast<unsigned char>(text[cursor])))
    ++cursor;
  if (cursor >= text.size() || text[cursor] != ':')
    throw std::runtime_error("actuator config limit is missing a value");
  ++cursor;
  while (cursor < text.size() && std::isspace(static_cast<unsigned char>(text[cursor])))
    ++cursor;
  if (cursor >= text.size())
    throw std::runtime_error("actuator config limit is missing a value");
  const char* begin = text.c_str() + cursor;
  char* consumed = nullptr;
  const double limit = std::strtod(begin, &consumed);
  if (consumed == begin)
    throw std::runtime_error("actuator config limit is not a number");
  const unsigned char next = static_cast<unsigned char>(*consumed);
  if (*consumed != '\0' && !std::isspace(next) && next != ',' && next != '}' && next != ']')
    throw std::runtime_error("actuator config limit token was not fully consumed");
  if (!std::isfinite(limit) || !(limit > 0.0) || limit > kMaxActuatorTorqueNm)
    throw std::runtime_error("actuator config limit must be finite and in (0, ceiling]");
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
  // Explicit launch-time path takes priority. A bad configured path must
  // fail closed rather than silently loading an unrelated working-tree file.
  const char* configured = std::getenv("AI_ROBOTIC_JOINT_CONFIG");
  if (configured != nullptr) return configured;
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
