#include "actuator_torque.hpp"
#include <cmath>
#include <cstdlib>
#include <iostream>

int main() {
#if defined(_WIN32)
  _putenv_s("AI_ROBOTIC_JOINT_CONFIG", "knee_stair_l1/config/default.json");
#else
  setenv("AI_ROBOTIC_JOINT_CONFIG", "knee_stair_l1/config/default.json", 1);
#endif
  auto good = actuator_torque::gate_from_default_config();
  if (good.estop_latched || !std::isfinite(good.limit_nm) || good.limit_nm <= 0.0 ||
      good.apply(1e10) > good.limit_nm) {
    std::cerr << "Explicit actuator config was not loaded correctly\n";
    return 1;
  }
#if defined(_WIN32)
  _putenv_s("AI_ROBOTIC_JOINT_CONFIG", "nonexistent/actuator_config_should_not_exist.json");
#else
  setenv("AI_ROBOTIC_JOINT_CONFIG", "nonexistent/actuator_config_should_not_exist.json", 1);
#endif
  auto missing = actuator_torque::gate_from_default_config();
  if (!missing.estop_latched || missing.apply(5.0) != 0.0) {
    std::cerr << "Missing explicit config must fail closed\n";
    return 2;
  }
  std::cout << "PASS: explicit actuator path and fail-closed behavior\n";
  return 0;
}
