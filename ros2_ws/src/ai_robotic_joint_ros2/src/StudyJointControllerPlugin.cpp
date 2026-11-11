#include "ai_robotic_joint_ros2/StudyJointControllerPlugin.hpp"
#include <fstream>
#include <sstream>
#include <pluginlib/class_list_macros.hpp>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <webots_ros2_driver/WebotsNode.hpp>

namespace ai_robotic_joint_ros2 {
namespace {
std::vector<std::string> cells(const std::string& line) {
  std::vector<std::string> values;
  std::istringstream input(line); std::string value;
  while (std::getline(input, value, ',')) {
    if (!value.empty() && value.back() == '\r') value.pop_back();
    values.push_back(value);
  }
  return values;
}
double number(const std::string& text) {
  std::size_t used = 0;
  const double value = std::stod(text, &used);
  if (used != text.size() || !std::isfinite(value)) throw std::invalid_argument("Invalid numeric model field");
  return value;
}
std::unique_ptr<EnergyCurve> readEnergy(const std::string& path) {
  std::ifstream file(path); std::string line;
  if (!file || !std::getline(file, line) || cells(line) != std::vector<std::string>{"angle_rad", "energy_j", "compression_m"})
    throw std::runtime_error("Missing or invalid geometry energy table: " + path);
  std::vector<EnergyPoint> points;
  while (std::getline(file, line)) {
    if (line.empty()) continue;
    auto row = cells(line);
    if (row.size() != 3) throw std::runtime_error("Invalid geometry energy row");
    points.push_back({number(row[0]), number(row[1]), number(row[2])});
  }
  auto result = std::make_unique<EnergyCurve>(std::move(points));
  if (!result->covers(-60 * StudyPair::rad, 60 * StudyPair::rad))
    throw std::runtime_error("Energy table must cover mechanical +/-60 degrees");
  return result;
}
bool near(double a, double b) { return std::isfinite(a) && std::abs(a - b) <= 1e-8 * std::max(1.0, std::abs(b)); }
msg::ControlTelemetry telemetry(const StudyOutput& output, const StudyPid& side) {
  msg::ControlTelemetry t;
  t.sim_time_s = output.sim_time_s;
  t.requested_angle_rad = output.requested_angle_rad;
  t.reference_angle_rad = output.reference_angle_rad;
  t.reference_velocity_rad_s = output.reference_velocity_rad_s;
  t.actual_angle_rad = side.actual_angle_rad; t.velocity_rad_s = side.velocity_rad_s;
  t.p_torque_nm = side.p_torque_nm; t.i_torque_nm = side.i_torque_nm; t.d_torque_nm = side.d_torque_nm;
  t.feedforward_torque_nm = 0; t.commanded_torque_nm = side.commanded_torque_nm;
  t.saturated = side.saturated; t.anti_windup = side.anti_windup;
  t.mode = output.mode; t.status = output.status; t.command_sequence = output.command_sequence;
  return t;
}
}  // namespace

void StudyJointControllerPlugin::init(webots_ros2_driver::WebotsNode* node,
    std::unordered_map<std::string, std::string>& parameters) {
  node_ = node;
  if (!node_) throw std::runtime_error("Study Webots node is required");
  const std::string share = node_->has_parameter("study_package_share") ?
      node_->get_parameter("study_package_share").as_string() :
      node_->declare_parameter<std::string>("study_package_share", "");
  auto value = [&](const std::string& key, const std::string& fallback) {
    const auto found = parameters.find(key); return found == parameters.end() ? fallback : found->second;
  };
  auto numeric = [&](const std::string& key, double fallback) { return number(value(key, std::to_string(fallback))); };
  auto modelPath = [&](const std::string& key) {
    auto path = value(key, "");
    const std::string marker = "__PACKAGE_SHARE__";
    auto location = path.find(marker);
    if (location != std::string::npos) {
      if (share.empty()) throw std::runtime_error("Launch must supply study_package_share");
      path.replace(location, marker.size(), share);
    }
    return path;
  };
  pair_ = std::make_unique<StudyPair>(numeric("kp", 4), numeric("ki", 4), numeric("kd", .05), numeric("outputTorqueLimitNm", 1));
  loss_nm_ = numeric("equivalentLossNm", 0);
  if (loss_nm_ < 0) throw std::invalid_argument("Equivalent loss cannot be negative");
  label_ = value("candidateLabel", "F-N geometry trial, unoptimized / not final CAD");
  auto energy_path = modelPath("energyTablePath");
  for (std::size_t payload = 0; payload < 3; ++payload) {
    auto path = energy_path;
    auto location = path.find("{payload}");
    if (location == std::string::npos) throw std::runtime_error("Energy path must contain {payload} presets");
    path.replace(location, 9, std::to_string(payload));
    curves_[payload] = readEnergy(path);
  }
  std::ifstream body_file(modelPath("bodyTablePath")); std::string line;
  const std::vector<std::string> expected = {"payload_index", "side", "mass_kg", "com_x_m", "com_y_m", "com_z_m", "ixx", "iyy", "izz", "ixy", "ixz", "iyz"};
  if (!body_file || !std::getline(body_file, line) || cells(line) != expected) throw std::runtime_error("Missing or invalid body table");
  std::array<std::array<bool, 2>, 3> seen{};
  while (std::getline(body_file, line)) {
    if (line.empty()) continue;
    auto row = cells(line);
    if (row.size() != 12) throw std::runtime_error("Invalid body row");
    const double index = number(row[0]);
    if (index < 0 || index > 2 || std::floor(index) != index) throw std::runtime_error("Invalid body payload index");
    const auto payload = static_cast<std::size_t>(index);
    const std::size_t side = row[1] == "baseline" ? 0 : row[1] == "candidate" ? 1 : 2;
    if (side > 1 || seen[payload][side]) throw std::runtime_error("Invalid or duplicated body side");
    Body body{number(row[2]), {number(row[3]), number(row[4]), number(row[5])},
              {number(row[6]), number(row[7]), number(row[8])}, {number(row[9]), number(row[10]), number(row[11])}};
    const double x = body.diagonal[0], y = body.diagonal[1], z = body.diagonal[2];
    const double xy = body.products[0], xz = body.products[1], yz = body.products[2];
    const double determinant = x * y * z + 2 * xy * xz * yz - x * yz * yz - y * xz * xz - z * xy * xy;
    if (body.mass <= 0 || x <= 0 || y <= 0 || z <= 0 || x * y - xy * xy <= 0 || determinant <= 0 ||
        x + y < z || x + z < y || y + z < x)
      throw std::runtime_error("Invalid mass or inertia tensor");
    bodies_[payload][side] = body; seen[payload][side] = true;
  }
  for (const auto& payload : seen) for (bool present : payload)
    if (!present) throw std::runtime_error("All three payloads and both bodies are required");
  const std::array<std::string, 2> sides = {"baseline", "candidate"};
  const std::array<const char*, 2> joint_defs = {"STUDY_BASELINE_JOINT", "STUDY_CANDIDATE_JOINT"};
  const std::array<const char*, 2> link_defs = {"STUDY_BASELINE_LINK", "STUDY_CANDIDATE_LINK"};
  const int step_ms = static_cast<int>(wb_robot_get_basic_time_step());
  if (!node_ || step_ms <= 0 || !wb_robot_get_supervisor()) throw std::runtime_error("Study requires a supervisor Robot");
  for (auto& gate : actuator_gates_) gate = actuator_torque::gate_from_default_config();
  for (std::size_t side = 0; side < 2; ++side) {
    motors_[side] = wb_robot_get_device((sides[side] + "_motor").c_str());
    sensors_[side] = wb_robot_get_device((sides[side] + "_sensor").c_str());
    physical_[side].joint = wb_supervisor_node_get_from_def(joint_defs[side]);
    physical_[side].link = wb_supervisor_node_get_from_def(link_defs[side]);
    if (!motors_[side] || !sensors_[side] || !physical_[side].joint || !physical_[side].link)
      throw std::runtime_error("Study devices or supervisor nodes missing");
    physical_[side].physics = wb_supervisor_field_get_sf_node(wb_supervisor_node_get_field(physical_[side].link, "physics"));
    if (!physical_[side].physics) throw std::runtime_error("Study physical body missing");
    wb_position_sensor_enable(sensors_[side], step_ms);
    wb_motor_set_torque(motors_[side], actuator_gates_[side].apply(0.0));
    applyBody(side, payload_);
  }
  const auto prefix = value("topicPrefix", "/joint_study");
  pending_.control.mode = msg::ManualCommand::ANGLE;
  subscription_ = node_->create_subscription<msg::StudyCommand>(prefix + "/manual/safe_command", 1,
      [this](msg::StudyCommand::ConstSharedPtr command) {
        const auto& c = command->control;
        if (command->payload_index > 2 || c.action > 1 || c.mode < 1 || c.mode > 3 ||
            !std::isfinite(c.target_angle_rad) || (c.action == 1 && (c.mode == 3 || std::abs(c.target_angle_rad) > 45 * StudyPair::rad))) return;
        std::lock_guard<std::mutex> lock(mutex_);
        pending_ = *command; received_ = true; last_command_ = std::chrono::steady_clock::now();
      });
  diagnostics_ = node_->create_publisher<msg::StudyTelemetry>(prefix + "/diagnostics", rclcpp::SensorDataQoS());
  for (std::size_t side = 0; side < 2; ++side)
    states_[side] = node_->create_publisher<sensor_msgs::msg::JointState>(prefix + "/" + sides[side] + "/state", rclcpp::SensorDataQoS());
  last_step_ = std::chrono::steady_clock::now();
}

void StudyJointControllerPlugin::applyBody(std::size_t side, uint8_t payload) {
  const auto& body = bodies_[payload][side]; const auto physics = physical_[side].physics;
  auto mass = wb_supervisor_node_get_field(physics, "mass");
  auto com = wb_supervisor_node_get_field(physics, "centerOfMass");
  auto inertia = wb_supervisor_node_get_field(physics, "inertiaMatrix");
  if (!mass || !com || !inertia || wb_supervisor_field_get_count(com) != 1 || wb_supervisor_field_get_count(inertia) != 2)
    throw std::runtime_error("Body fields require one COM and full inertia matrix");
  wb_supervisor_field_set_sf_float(mass, body.mass);
  wb_supervisor_field_set_mf_vec3f(com, 0, body.com.data());
  wb_supervisor_field_set_mf_vec3f(inertia, 0, body.diagonal.data());
  wb_supervisor_field_set_mf_vec3f(inertia, 1, body.products.data());
}
bool StudyJointControllerPlugin::bodyMatches(std::size_t side, uint8_t payload) const {
  const auto& body = bodies_[payload][side]; const auto physics = physical_[side].physics;
  if (!near(wb_supervisor_field_get_sf_float(wb_supervisor_node_get_field(physics, "mass")), body.mass)) return false;
  auto com = wb_supervisor_field_get_mf_vec3f(wb_supervisor_node_get_field(physics, "centerOfMass"), 0);
  for (std::size_t i = 0; i < 3; ++i) if (!near(com[i], body.com[i])) return false;
  auto inertia = wb_supervisor_node_get_field(physics, "inertiaMatrix");
  auto diagonal = wb_supervisor_field_get_mf_vec3f(inertia, 0);
  for (std::size_t i = 0; i < 3; ++i) if (!near(diagonal[i], body.diagonal[i])) return false;
  auto products = wb_supervisor_field_get_mf_vec3f(inertia, 1);
  for (std::size_t i = 0; i < 3; ++i) if (!near(products[i], body.products[i])) return false;
  return true;
}
void StudyJointControllerPlugin::startReset(const msg::StudyCommand& command) {
  payload_ = command.payload_index;
  for (std::size_t side = 0; side < 2; ++side) {
    applyBody(side, payload_);
    wb_motor_set_torque(motors_[side], actuator_gates_[side].apply(0.0));
    wb_supervisor_node_set_joint_position(physical_[side].joint, 0, 1);
    wb_supervisor_node_reset_physics(physical_[side].link);
    const auto parameters = wb_supervisor_field_get_sf_node(wb_supervisor_node_get_field(physical_[side].joint, "jointParameters"));
    const auto position = wb_supervisor_node_get_field(parameters, "position");
    const auto rotation = wb_supervisor_field_get_sf_rotation(wb_supervisor_node_get_field(physical_[side].link, "rotation"));
    if (!position || !near(wb_supervisor_field_get_sf_float(position), 0) || !near(rotation[3], 0) || !bodyMatches(side, payload_))
      throw std::runtime_error("Reset fields did not reach requested physical state");
  }
  pair_->reset(0, command.control.sequence, wb_robot_get_time());
  reset_requested_ = command.reset_sequence;
  reset_pending_ = true; reset_wait_steps_ = 0; plugin_fault_ = false;
}
void StudyJointControllerPlugin::step() {
  const auto wall = std::chrono::steady_clock::now();
  msg::StudyCommand command; bool fresh;
  { std::lock_guard<std::mutex> lock(mutex_);
    command = pending_;
    fresh = received_ && std::chrono::duration<double>(wall - last_command_).count() < .5;
  }
  const bool pause_gap = std::chrono::duration<double>(wall - last_step_).count() >= .5;
  last_step_ = wall;
  const bool conflicting = node_->count_publishers(subscription_->get_topic_name()) > 1;
  const double time = wb_robot_get_time();
  const std::array<double, 2> angles = {wb_position_sensor_get_value(sensors_[0]), wb_position_sensor_get_value(sensors_[1])};
  StudyOutput output; PassiveState passive; double loss = 0;
  bool reset_started = false;
  try {
    if (last_time_ >= 0 && time <= last_time_) {
      // Webots Reset restores world properties: restore matching selected tables,
      // clear all previously received actions and require a new command sequence.
      for (std::size_t side = 0; side < 2; ++side) applyBody(side, payload_);
      pair_->reset(0, command.control.sequence);
      reset_ack_ = 0; reset_requested_ = 0; reset_pending_ = false;
      plugin_fault_ = false; fresh = false;
      std::lock_guard<std::mutex> lock(mutex_); received_ = false;
    }
    last_time_ = time;
    if (fresh && !conflicting && !reset_pending_ && command.reset_sequence > reset_ack_) {
      if (command.control.action != msg::ManualCommand::STOP || pair_->moving())
        throw std::runtime_error("Reset requested before reference stopped");
      startReset(command);
      reset_started = true;
    }
    if (reset_pending_) {
      ++reset_wait_steps_;
      if (reset_wait_steps_ > 1 && std::isfinite(angles[0]) && std::isfinite(angles[1]) &&
          std::abs(angles[0]) < .035 && std::abs(angles[1]) < .035 &&
          std::abs(angles[0] - angles[1]) < .035 && bodyMatches(0, payload_) && bodyMatches(1, payload_)) {
        reset_ack_ = reset_requested_; reset_pending_ = false;
      } else if (reset_wait_steps_ > 5) throw std::runtime_error("Physical reset verification failed");
    }
    StudyInput input{command.control.sequence, command.control.mode, command.control.action, command.control.target_angle_rad};
    if (reset_pending_) input.action = 0;
    if (reset_started) {
      output.sim_time_s = time; output.command_sequence = command.control.sequence;
      output.mode = command.control.mode; output.status = "resetting_verify_physics";
      for (std::size_t side = 0; side < 2; ++side) output.sides[side].actual_angle_rad = angles[side];
    } else output = pair_->step(time, angles, input, fresh, pause_gap || conflicting || reset_pending_);
    if (plugin_fault_ || pair_->faulted()) throw std::runtime_error("Latched study fault");
    passive = curves_[payload_]->sample(reset_started ? 0 : angles[1]);
    loss = equivalentLoss(output.sides[1].velocity_rad_s, loss_nm_);
    for (std::size_t side = 0; side < 2; ++side)
      wb_motor_set_torque(motors_[side], actuator_gates_[side].apply(output.sides[side].commanded_torque_nm));
    const double external[3] = {0, -(passive.torque_nm + loss), 0};
    wb_supervisor_node_add_torque(physical_[1].link, external, false);
    if (reset_pending_) output.status = "resetting_verify_physics";
    else if (conflicting) output.status = "holding_multiple_publishers";
  } catch (const std::exception& error) {
    plugin_fault_ = true; output.status = "fault_pause_webots";
    output.sim_time_s = time;
    passive = {}; loss = 0;
    for (std::size_t side = 0; side < 2; ++side) {
      wb_motor_set_torque(motors_[side], actuator_gates_[side].apply(0.0));
      output.sides[side].commanded_torque_nm = 0;
      output.sides[side].actual_angle_rad = angles[side];
    }
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "Study fault: %s. Pause Webots; zero torque is NOT a brake.", error.what());
  }
  msg::StudyTelemetry message;
  message.baseline = telemetry(output, output.sides[0]); message.candidate = telemetry(output, output.sides[1]);
  message.spring_compression_m = passive.compression_m;
  message.compensation_torque_nm = passive.torque_nm; message.loss_torque_nm = loss;
  message.payload_mass_kg = std::array<double, 3>{0, .125, .25}[payload_];
  message.candidate_label = label_;
  message.model_status = "synthetic fixture; geometry-equivalent passive torque; no CAD/contact/physical validation";
  message.can_reset = !reset_pending_ && !pair_->moving() && std::isfinite(angles[0]) && std::isfinite(angles[1]);
  message.reset_sequence = reset_ack_;
  diagnostics_->publish(message);
  for (std::size_t side = 0; side < 2; ++side) {
    sensor_msgs::msg::JointState state;
    const double stamp = std::isfinite(time) ? std::clamp(time, 0.0, 2147483647.0) : 0;
    state.header.stamp.sec = static_cast<int32_t>(stamp);
    state.header.stamp.nanosec = static_cast<uint32_t>((stamp - state.header.stamp.sec) * 1e9);
    state.name = {side == 0 ? "baseline_axis" : "candidate_axis"};
    state.position = {angles[side]}; state.velocity = {output.sides[side].velocity_rad_s};
    state.effort = {output.sides[side].commanded_torque_nm}; states_[side]->publish(state);
  }
}
}  // namespace ai_robotic_joint_ros2
PLUGINLIB_EXPORT_CLASS(ai_robotic_joint_ros2::StudyJointControllerPlugin, webots_ros2_driver::PluginInterface)
