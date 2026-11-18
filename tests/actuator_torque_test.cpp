#include "actuator_torque.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

namespace {

int g_failed = 0;
int g_passed = 0;

void expect(bool condition, const std::string& name) {
  if (condition) {
    ++g_passed;
    std::cout << "PASS " << name << "\n";
  } else {
    ++g_failed;
    std::cout << "FAIL " << name << "\n";
  }
}

}  // namespace

int main() {
  const char* path = "knee_stair_l1/config/default.json";
  const double limit = actuator_torque::limit_from_file(path);
  expect(std::isfinite(limit) && limit > 0.0, "config limit is positive and finite");

  actuator_torque::Gate gate;
  gate.limit_nm = limit;

  expect(gate.apply(100.0) == limit && gate.state() == actuator_torque::State::kNormal,
         "plus 100 Nm clamps to the configured limit");
  expect(gate.apply(-100.0) == -limit && gate.state() == actuator_torque::State::kNormal,
         "minus 100 Nm clamps to the configured limit");
  expect(gate.apply(limit) == limit, "plus limit is unchanged");
  expect(gate.apply(-limit) == -limit, "minus limit is unchanged");

  const double above = std::nextafter(limit, std::numeric_limits<double>::infinity());
  const double below = std::nextafter(-limit, -std::numeric_limits<double>::infinity());
  expect(gate.apply(above) == limit, "just above the limit clamps");
  expect(gate.apply(below) == -limit, "just below the negative limit clamps");
  expect(gate.apply(limit / 2.0) == limit / 2.0, "interior command is unchanged");

  expect(gate.apply(std::numeric_limits<double>::quiet_NaN()) == 0.0 &&
             gate.state() == actuator_torque::State::kEStop,
         "NaN yields 0 and E_STOP");
  expect(gate.apply(1.0) == 0.0 && gate.estop_latched, "E_STOP stays latched after NaN");
  gate.reset();
  expect(gate.apply(std::numeric_limits<double>::infinity()) == 0.0 && gate.estop_latched,
         "+inf yields 0 and E_STOP");
  gate.reset();
  expect(gate.apply(-std::numeric_limits<double>::infinity()) == 0.0 && gate.estop_latched,
         "-inf yields 0 and E_STOP");
  gate.reset();
  expect(gate.apply(limit / 2.0) == limit / 2.0 && gate.state() == actuator_torque::State::kNormal,
         "reset clears E_STOP");

  const double sensors[] = {
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity(),
  };
  const char* sensor_names[] = {"NaN sensor", "+Inf sensor", "-Inf sensor"};
  for (int index = 0; index < 3; ++index) {
    actuator_torque::Gate sensor_gate;
    sensor_gate.limit_nm = limit;
    const double stopped = actuator_torque::motor_command_for_feedback(
        sensor_gate, sensors[index], limit / 2.0);
    expect(stopped == 0.0 && sensor_gate.state() == actuator_torque::State::kEStop,
           std::string(sensor_names[index]) + " commands 0 and latches E_STOP");
    const double held = actuator_torque::motor_command_for_feedback(
        sensor_gate, 0.2, limit / 2.0);
    expect(held == 0.0 && sensor_gate.estop_latched,
           std::string(sensor_names[index]) + " stays latched after a finite sensor");
    sensor_gate.reset();
    const double cleared = actuator_torque::motor_command_for_feedback(
        sensor_gate, 0.2, limit / 2.0);
    expect(cleared == limit / 2.0 && sensor_gate.state() == actuator_torque::State::kNormal,
           std::string(sensor_names[index]) + " reset clears the latch");
  }

  const int total = g_passed + g_failed;
  std::cout << "actuator_torque_tests: " << g_passed << " passed, " << g_failed
            << " failed, " << total << " total\n";
  return g_failed == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
