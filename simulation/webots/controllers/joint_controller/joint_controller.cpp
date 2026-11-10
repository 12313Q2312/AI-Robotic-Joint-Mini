#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>

#include <iomanip>
#include <iostream>

#include "../../../../knee_stair_l1/include/actuator_torque.hpp"

int main() {
  webots::Robot robot;
  const int time_step = static_cast<int>(robot.getBasicTimeStep());
  const double dt = time_step / 1000.0;

  webots::PositionSensor *sensor =
      robot.getPositionSensor("joint_sensor");
  webots::Motor *motor = robot.getMotor("joint_motor");

  if (sensor == nullptr || motor == nullptr) {
    std::cerr << "Controller error: joint_sensor or joint_motor was not found."
              << std::endl;
    return 1;
  }

  sensor->enable(time_step);

  constexpr double target_angle = 0.523599;   // 30 deg, rad
  constexpr double kp = 0.15;                 // N*m/rad
  constexpr double ki = 0.05;                 // N*m/(rad*s)
  constexpr double kd = 0.04;                 // N*m*s/rad
  constexpr double torque_limit = 0.10;       // controller limit, N*m
  constexpr double load_torque = -0.02;       // external resisting load, N*m

  actuator_torque::Gate actuator_gate = actuator_torque::gate_from_default_config();
  double error_integral = 0.0;
  double previous_angle = 0.0;
  bool have_previous_sample = false;
  int print_counter = 0;

  while (robot.step(time_step) != -1) {
    const double actual_angle = sensor->getValue();
    const double error = target_angle - actual_angle;

    double angular_velocity = 0.0;
    if (have_previous_sample)
      angular_velocity = (actual_angle - previous_angle) / dt;
    previous_angle = actual_angle;
    have_previous_sample = true;

    const double p_torque = kp * error;
    const double d_torque = -kd * angular_velocity;

    // I remembers the area under the error curve. Test a new accumulated
    // value first, then accept it only when it does not worsen saturation.
    const double candidate_integral = error_integral + error * dt;
    const double candidate_i_torque = ki * candidate_integral;
    const double candidate_raw_torque =
        p_torque + candidate_i_torque + d_torque;

    const bool saturated_high = candidate_raw_torque > torque_limit;
    const bool saturated_low = candidate_raw_torque < -torque_limit;
    const bool integration_reduces_saturation =
        (saturated_high && error < 0.0) ||
        (saturated_low && error > 0.0);
    const bool anti_windup_active =
        (saturated_high || saturated_low) &&
        !integration_reduces_saturation;

    if (!anti_windup_active)
      error_integral = candidate_integral;

    const double i_torque = ki * error_integral;
    const double raw_control_torque = p_torque + i_torque + d_torque;
    double control_torque = raw_control_torque;

    if (control_torque > torque_limit)
      control_torque = torque_limit;
    else if (control_torque < -torque_limit)
      control_torque = -torque_limit;

    // Minimal plant disturbance model: net joint torque equals controller
    // torque plus the constant resisting load torque.
    const double applied_torque = actuator_gate.apply(control_torque + load_torque);
    motor->setTorque(applied_torque);

    if (print_counter++ % 2 == 0) {
      std::cout << std::fixed << std::setprecision(6)
                << "time_s=" << robot.getTime()
                << ", target_rad=" << target_angle
                << ", actual_rad=" << actual_angle
                << ", error_rad=" << error
                << ", velocity_rad_s=" << angular_velocity
                << ", error_integral_rad_s=" << error_integral
                << ", anti_windup_active=" << anti_windup_active
                << ", p_torque_nm=" << p_torque
                << ", i_torque_nm=" << i_torque
                << ", d_torque_nm=" << d_torque
                << ", raw_control_torque_nm=" << raw_control_torque
                << ", control_torque_nm=" << control_torque
                << ", load_torque_nm=" << load_torque
                << ", applied_torque_nm=" << applied_torque << std::endl;
    }
  }

  return 0;
}



