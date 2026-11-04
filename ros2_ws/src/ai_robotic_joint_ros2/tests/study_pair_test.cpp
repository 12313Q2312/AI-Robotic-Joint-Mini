#include "ai_robotic_joint_ros2/StudyPair.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

struct Plant {
  double mass, com_x, com_z, inertia;
  double q = 0, v = 0;
  void advance(double motor, double passive, double dt) {
    const double gravity = -mass * 9.81 * (com_x * std::cos(q) - com_z * std::sin(q));
    v += (motor + passive + gravity - .005 * v) / inertia * dt;
    q += v * dt;
  }
};
std::vector<std::string> split(const std::string& line) {
  std::vector<std::string> values; std::istringstream stream(line); std::string part;
  while (std::getline(stream, part, ',')) values.push_back(part);
  return values;
}
std::array<std::array<Plant, 2>, 3> loadPlants(const std::string& resource) {
  std::ifstream file(resource + "/study_bodies.csv");
  if (!file) throw std::runtime_error("Cannot read surrogate body file");
  std::string line; std::getline(file, line);
  std::array<std::array<Plant, 2>, 3> result{};
  while (std::getline(file, line)) {
    if (line.empty()) continue;
    auto row = split(line);
    const int payload = std::stoi(row[0]), side = row[1] == "baseline" ? 0 : 1;
    const double mass = std::stod(row[2]), x = std::stod(row[3]), z = std::stod(row[5]);
    result[payload][side] = {mass, x, z, std::stod(row[7]) + mass * (x * x + z * z)};
  }
  return result;
}
std::vector<ai_robotic_joint_ros2::EnergyPoint> loadEnergy(const std::string& resource, int payload) {
  std::ifstream file(resource + "/study_energy_" + std::to_string(payload) + ".csv");
  if (!file) throw std::runtime_error("Cannot read surrogate energy file");
  std::string line; std::getline(file, line);
  std::vector<ai_robotic_joint_ros2::EnergyPoint> result;
  while (std::getline(file, line)) {
    if (line.empty()) continue;
    auto row = split(line); result.push_back({std::stod(row[0]), std::stod(row[1]), std::stod(row[2])});
  }
  return result;
}
struct Trial {
  int payload; double dt, target;
  std::array<double, 2> max_error_deg{}, hold_error_deg{}, max_speed_deg_s{};
  std::array<double, 2> motion_error_deg{}, overshoot_deg{}, motion_speed_deg_s{};
  std::array<int, 2> saturated_samples{}, torque_reversals{};
  bool finite = true;
};
Trial trial(const std::string& resource, int payload, double target, double dt,
            double kp, double ki, double kd) {
  using namespace ai_robotic_joint_ros2;
  auto plants = loadPlants(resource)[payload];
  EnergyCurve curve(loadEnergy(resource, payload));
  StudyPair pair(kp, ki, kd, 1);
  StudyInput command{1, 2, 0, 0};
  Trial result{payload, dt, target};
  auto output = pair.step(0, {0, 0}, command, true, false);
  std::array<double, 2> previous_torque{};
  // Ideal fixed-axis rigid body surrogate, semi-implicit Euler. This is NOT
  // Webots/ODE validation. Pre-hold establishes the same zero reference; then
  // one +/-30deg trajectory runs, with a long hold that reveals limit cycles.
  for (int tick = 1; tick <= static_cast<int>(30 / dt); ++tick) {
    const double time = tick * dt;
    try {
      const double passive = curve.sample(plants[1].q).torque_nm;
      for (std::size_t side = 0; side < 2; ++side) plants[side].advance(output.sides[side].commanded_torque_nm, side ? passive : 0, dt);
      if (time >= 5 && command.sequence == 1) command = {2, 2, 1, target * StudyPair::rad};
      output = pair.step(time, {plants[0].q, plants[1].q}, command, true, false);
      if (output.status.rfind("fault", 0) == 0) { result.finite = false; break; }
      for (std::size_t side = 0; side < 2; ++side) {
        const double error = std::abs(output.reference_angle_rad - plants[side].q) / StudyPair::rad;
        result.max_error_deg[side] = std::max(result.max_error_deg[side], error);
        result.max_speed_deg_s[side] = std::max(result.max_speed_deg_s[side], std::abs(plants[side].v) / StudyPair::rad);
        if (time >= 5) {
          result.motion_error_deg[side] = std::max(result.motion_error_deg[side], error);
          result.motion_speed_deg_s[side] = std::max(result.motion_speed_deg_s[side], std::abs(plants[side].v) / StudyPair::rad);
          result.overshoot_deg[side] = std::max(result.overshoot_deg[side], std::copysign(1., target) * (plants[side].q / StudyPair::rad - target));
        }
        if (time >= 18.6) result.hold_error_deg[side] = std::max(result.hold_error_deg[side], std::abs(target - plants[side].q / StudyPair::rad));
        if (output.sides[side].saturated) ++result.saturated_samples[side];
        const auto torque = output.sides[side].commanded_torque_nm;
        if (time >= 18.6 && torque * previous_torque[side] < 0 && std::abs(torque - previous_torque[side]) > .1) ++result.torque_reversals[side];
        previous_torque[side] = torque;
      }
    } catch (const std::exception&) { result.finite = false; break; }
  }
  return result;
}

using namespace ai_robotic_joint_ros2;
int main(int argc, char** argv) {
  EnergyCurve curve({{-1.1, 2.2, .020}, {0, 1.1, .015}, {1.1, 0, .010}});
  const auto value = curve.sample(.3);
  assert(std::abs(value.torque_nm - 1.0) < 1e-12);
  assert(std::abs(value.energy_j - .8) < 1e-12);
  std::cout << "PASS conservative table derives torque from energy, not target curve\n";
  StudyPair pair;
  StudyInput command{1, 2, 1, .5};
  auto first = pair.step(0, {0.0, 0.0}, command, true, false);
  assert(first.reference_angle_rad == 0);
  assert(!pair.moving());  // Never replay a command present during initialization.
  command.sequence = 2;
  double last_velocity = 0;
  for (int tick = 1; tick <= 100; ++tick) {
    auto result = pair.step(tick * .016, {0.0, 0.0}, command, true, false);
    assert(std::abs(result.reference_velocity_rad_s) <= 3 * StudyPair::rad + 1e-10);
    assert(std::abs(result.reference_velocity_rad_s - last_velocity) <= 5 * StudyPair::rad * .016 + 1e-10);
    assert(result.sides[0].commanded_torque_nm == result.sides[1].commanded_torque_nm);
    last_velocity = result.reference_velocity_rad_s;
  }
  assert(pair.moving());
  std::cout << "PASS pair has one bounded reference and identical active PID commands\n";
  pair.step(1.616, {0.0, 0.0}, command, false, false);
  for (int tick = 102; tick <= 180; ++tick) pair.step(tick * .016, {0.0, 0.0}, command, true, false);
  assert(!pair.moving());
  auto held = pair.step(2.896, {0.0, 0.0}, command, true, false);
  assert(held.reference_velocity_rad_s == 0);
  pair.reset(0, command.sequence);
  auto reset = pair.step(2.912, {0.0, 0.0}, command, true, false);
  assert(reset.reference_angle_rad == 0 && !pair.moving());
  auto bad = pair.step(2.928, {NAN, 0}, command, true, false);
  assert(bad.status == "fault_pause_webots" && bad.sides[0].commanded_torque_nm == 0);
  auto recovery = pair.step(2.944, {0, 0}, command, true, false);
  assert(recovery.status == "fault_pause_webots");
  pair.reset(0, command.sequence);
  assert(pair.step(2.96, {0, 0}, command, true, false).status == "ready");
  std::cout << "PASS watchdog cancels motion, reset rejects stale actions, invalid feedback latches fault\n";
  bool rejected = false;
  try { curve.sample(1.2); } catch (const std::out_of_range&) { rejected = true; }
  assert(rejected);
  for (double v : {-1.0, -.001, 0.0, .001, 1.0}) assert(equivalentLoss(v, .2) * v <= 0);
  std::cout << "PASS table domain rejects extrapolation and equivalent loss is dissipative\n";
  StudyPair limited(8, 3, .4, 1);
  StudyInput limit_command{1, 2, 0, 0};
  limited.step(0, {0, 0}, limit_command, true, false);
  limit_command = {2, 2, 1, 45 * StudyPair::rad};
  StudyOutput saturated;
  for (int tick = 1; tick <= 1200; ++tick) {
    saturated = limited.step(tick * .016, {0, 0}, limit_command, true, false);
    for (const auto& side : saturated.sides) assert(std::abs(side.commanded_torque_nm) <= 1);
  }
  assert(saturated.sides[0].saturated && saturated.sides[0].anti_windup);
  const double integral = saturated.sides[0].i_torque_nm;
  auto next = limited.step(19.216, {0, 0}, limit_command, true, false);
  assert(next.sides[0].i_torque_nm == integral);
  auto rewind = limited.step(0, {0, 0}, limit_command, true, false);
  assert(rewind.reference_angle_rad == 0 && !limited.moving());
  std::cout << "PASS motor limit/anti-windup excludes passive mechanics and time rewind cancels old motion\n";
  StudyPair ordered;
  ordered.step(0, {0, 0}, StudyInput{10, 2, 0, 0}, true, false);
  ordered.step(.016, {0, 0}, StudyInput{12, 2, 0, 0}, true, false);
  auto stale_move = ordered.step(.032, {0, 0}, StudyInput{11, 2, 1, .5}, true, false);
  assert(stale_move.reference_angle_rad == 0 && !ordered.moving());
  assert(stale_move.command_sequence == 12);
  ordered.step(.048, {0, 0}, StudyInput{9, 2, 0, 0}, true, false);
  assert(ordered.step(.064, {0, 0}, StudyInput{11, 2, 1, .5}, true, false).reference_angle_rad == 0);
  auto fresh_move = ordered.step(.080, {0, 0}, StudyInput{13, 2, 1, .5}, true, false);
  assert(fresh_move.reference_angle_rad > 0 && ordered.moving());
  std::cout << "PASS older ROS moves cannot revive after a newer stop or reduce consumed sequence\n";
  if (argc > 1) {
    if (argc > 2 && std::string(argv[2]) == "--search") {
      double best_score = 1e9;
      for (double kp : {4., 6., 8., 10.}) for (double ki : {2., 3., 4.}) for (double kd : {.02, .03, .04, .05}) {
        double worst = 0, max_error = 0; int reversals = 0; bool valid = true;
        for (int payload = 0; payload < 3; ++payload) for (double target : {-30., 30.}) for (double dt : {.016, .008}) {
          const auto check = trial(argv[1], payload, target, dt, kp, ki, kd);
          valid = valid && check.finite;
          for (std::size_t side = 0; side < 2; ++side) {
            worst = std::max(worst, check.hold_error_deg[side]);
            max_error = std::max(max_error, check.motion_error_deg[side]);
            reversals += check.torque_reversals[side];
          }
        }
        const double score = worst + max_error + reversals;
        if (valid && worst <= 1 && max_error <= 1 && score < best_score) {
          best_score = score;
          std::cout << "STABLE kp=" << kp << " ki=" << ki << " kd=" << kd << " worst_hold=" << worst << " peak_tracking=" << max_error << " reversals=" << reversals << std::endl;
        }
      }
      return best_score < 1e9 ? 0 : 1;
    }
    std::ofstream report;
    if (argc > 2) {
      report.open(argv[2]);
      if (!report) throw std::runtime_error("Cannot write surrogate report artifact");
      report << "{\"model\":\"Ideal fixed-axis rigid body; semi-implicit Euler; not Webots/ODE validation\","
             << "\"units\":\"SI internally; reported angles deg\",\"motor_limit_nm\":1,\"joint_damping_nm_s_rad\":0.005,"
             << "\"prehold_s\":5,\"hold_window_s\":[18.6,30],\"controller\":{\"kp\":4,\"ki\":4,\"kd\":0.05},"
             << "\"acceptance\":\"All 12 cases: finite; slow-motion tracking<=1deg; hold<=1deg; no large alternating hold torque\",\"trials\":[";
    }
    bool all_pass = true; bool first_trial = true;
    for (int payload = 0; payload < 3; ++payload) for (double target : {-30., 30.}) for (double dt : {.016, .008}) {
      const auto check = trial(argv[1], payload, target, dt, 4, 4, .05);
      std::cout << "SURROGATE payload=" << payload << " target=" << target << " dt=" << dt << " finite=" << check.finite
                << " hold_error=" << check.hold_error_deg[0] << "," << check.hold_error_deg[1]
                << " motion_error=" << check.motion_error_deg[0] << "," << check.motion_error_deg[1]
                << " startup_peak_error=" << check.max_error_deg[0] << "," << check.max_error_deg[1]
                << " hold_reversals=" << check.torque_reversals[0] << "," << check.torque_reversals[1] << "\n";
      if (!check.finite) all_pass = false;
      for (std::size_t side = 0; side < 2; ++side)
        if (check.hold_error_deg[side] > 1 || check.motion_error_deg[side] > 1 || check.torque_reversals[side]) all_pass = false;
      if (report) {
        if (!first_trial) report << ',';
        first_trial = false;
        report << "{\"payload_index\":" << payload << ",\"target_deg\":" << target << ",\"dt_s\":" << dt << ",\"finite\":" << (check.finite ? "true" : "false");
        auto array = [&](const std::string& name, const auto& values) { report << ",\"" << name << "\":[" << values[0] << ',' << values[1] << ']'; };
        array("max_tracking_error_including_startup_deg", check.max_error_deg);
        array("motion_tracking_error_after_prehold_deg", check.motion_error_deg);
        array("hold_error_deg", check.hold_error_deg); array("overshoot_deg", check.overshoot_deg);
        array("max_actual_speed_including_startup_deg_s", check.max_speed_deg_s);
        array("max_actual_speed_after_prehold_deg_s", check.motion_speed_deg_s);
        array("saturated_samples", check.saturated_samples); array("large_hold_torque_reversals", check.torque_reversals);
        report << '}';
      }
    }
    if (report) report << "],\"pass\":" << (all_pass ? "true" : "false") << ",\"limitations\":[\"5s pre-hold required; startup sag and velocities are reported, not hidden\",\"Real friction/backlash/cam contact not validated\",\"Gain search reused these surrogate cases; Webots and CAD remain independent acceptance\"]}";
    return all_pass ? 0 : 1;
  }
}
