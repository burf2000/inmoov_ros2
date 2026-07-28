// Copyright 2026 Burf Robotics
#include "inmoov_hardware/inmoov_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace inmoov_hardware
{

namespace
{

std::string param(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  const std::string & fallback)
{
  const auto it = params.find(key);
  return it == params.end() ? fallback : it->second;
}

bool paramBool(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  bool fallback)
{
  const auto it = params.find(key);
  if (it == params.end()) {return fallback;}
  std::string v = it->second;
  std::transform(v.begin(), v.end(), v.begin(), ::tolower);
  return v == "true" || v == "1" || v == "yes";
}

double paramDouble(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  double fallback)
{
  const auto it = params.find(key);
  if (it == params.end()) {return fallback;}
  try {
    return std::stod(it->second);
  } catch (const std::exception &) {
    return fallback;
  }
}

}  // namespace

hardware_interface::CallbackReturn InmoovSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & hp = info_.hardware_parameters;

  std::string map_path = param(hp, "joint_map", "");
  if (map_path.empty()) {
    map_path = ament_index_cpp::get_package_share_directory("inmoov_hardware") +
      "/config/inmoov_joint_map.yaml";
  }

  try {
    joint_map_ = JointMap::fromFile(map_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to load joint map '%s': %s", map_path.c_str(), e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Staged bring-up, exactly like the Quest bridge's DISABLED_GROUPS. Default is HEAD
  // ONLY: the head is the low-risk group, and arms/hands/torso have unverified direction
  // signs that must be eyeballed by a human before they are allowed to move.
  enabled_groups_ = splitCsv(param(hp, "enabled_groups", "head"));

  mrl_cfg_.host = param(hp, "mrl_host", "pi5-inmoov.local");
  mrl_cfg_.port = static_cast<int>(paramDouble(hp, "mrl_port", 8888));
  mrl_cfg_.dry_run = paramBool(hp, "dry_run", false);
  mrl_cfg_.move_epsilon_deg = paramDouble(hp, "move_epsilon_deg", 1.0);
  mrl_cfg_.hold_refresh_s = paramDouble(hp, "hold_refresh_s", 0.0);
  mrl_cfg_.worker_rate_hz = paramDouble(hp, "worker_rate_hz", 20.0);
  mrl_cfg_.request_timeout_s = paramDouble(hp, "request_timeout_s", 1.0);
  refresh_limits_ = paramBool(hp, "refresh_limits", true);
  rest_on_deactivate_ = paramBool(hp, "rest_on_deactivate", true);
  // When false, the driver still activates with MRL unreachable. Useful for checking the
  // wiring on a desk; it is NOT the dry-run flag (dry_run keeps MRL out of the loop
  // entirely and logs every intended move).
  require_mrl_ = paramBool(hp, "require_mrl", true);

  // Every URDF joint must expose a position command+state interface or the controller
  // manager refuses to load. Joints with no servo simply have their commands dropped.
  for (const auto & j : info_.joints) {
    const auto * m = joint_map_.find(j.name);
    if (m == nullptr) {
      RCLCPP_WARN(
        get_logger(),
        "URDF joint '%s' is not in the joint map at all -- it will be parked (no servo).",
        j.name.c_str());
      parked_joints_.push_back(j.name);
      continue;
    }
    if (!m->enabled) {
      parked_joints_.push_back(j.name);
      continue;
    }
    if (std::find(enabled_groups_.begin(), enabled_groups_.end(), m->group) ==
      enabled_groups_.end())
    {
      parked_joints_.push_back(j.name);
      continue;
    }
    ActiveJoint aj;
    aj.map = joint_map_.find(j.name);
    aj.last_cmd_rad = m->urdf_home_rad;
    active_.push_back(aj);
  }

  std::ostringstream groups;
  for (size_t i = 0; i < enabled_groups_.size(); ++i) {
    groups << (i ? "," : "") << enabled_groups_[i];
  }
  RCLCPP_INFO(
    get_logger(),
    "InMoov hardware: %zu joint(s) DRIVEN, %zu parked. enabled_groups=[%s] mrl=%s:%d%s",
    active_.size(), parked_joints_.size(), groups.str().c_str(),
    mrl_cfg_.host.c_str(), mrl_cfg_.port, mrl_cfg_.dry_run ? "  *** DRY RUN ***" : "");
  for (const auto & aj : active_) {
    RCLCPP_INFO(
      get_logger(), "  drive %-24s -> %-24s rest=%.0f [%.0f..%.0f] %s%s verified=%s",
      aj.map->urdf_joint.c_str(), aj.map->service.c_str(), aj.map->servo_rest,
      aj.map->effectiveMin(), aj.map->effectiveMax(),
      aj.map->mode == Mode::kCentred ? "centred" : "floored",
      aj.map->invert ? " inverted" : "", aj.map->verified.c_str());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn InmoovSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_logger();
  mrl_.configure(
    mrl_cfg_,
    [logger](const std::string & m) {RCLCPP_INFO(logger, "%s", m.c_str());},
    [logger](const std::string & m) {RCLCPP_WARN(logger, "%s", m.c_str());});

  if (mrl_cfg_.dry_run) {
    RCLCPP_WARN(get_logger(), "DRY RUN: no servo command will reach MRL.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  if (!mrl_.ping()) {
    RCLCPP_ERROR(
      get_logger(), "MRL not reachable at %s:%d", mrl_cfg_.host.c_str(), mrl_cfg_.port);
    if (require_mrl_) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_WARN(get_logger(), "require_mrl=false, continuing with MRL down.");
  } else {
    RCLCPP_INFO(get_logger(), "MRL reachable at %s:%d", mrl_cfg_.host.c_str(), mrl_cfg_.port);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn InmoovSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Re-read the calibrated limits live. The physical robot's min/max/rest differ a lot
  // left vs right and from any static table, and they change whenever Simon recalibrates
  // in MRL -- so trusting the YAML alone would eventually drive a servo into a stop.
  if (refresh_limits_ && !mrl_cfg_.dry_run) {
    size_t ok = 0;
    for (const auto & aj : active_) {
      const auto live = mrl_.getLimits(aj.map->service);
      if (live.has_value()) {
        joint_map_.applyLiveLimits(aj.map->service, live->min, live->max, live->rest);
        ++ok;
        const auto * y = aj.map;
        if (std::abs(y->servo_min - live->min) > 0.5 ||
          std::abs(y->servo_max - live->max) > 0.5 ||
          std::abs(y->servo_rest - live->rest) > 0.5)
        {
          RCLCPP_WARN(
            get_logger(),
            "%s live limits differ from YAML: live [%.0f..%.0f] rest %.0f",
            y->service.c_str(), live->min, live->max, live->rest);
        }
      } else {
        RCLCPP_WARN(
          get_logger(), "%s: no live limits from MRL, keeping YAML defaults",
          aj.map->service.c_str());
      }
    }
    RCLCPP_INFO(get_logger(), "Refreshed live limits for %zu/%zu servos", ok, active_.size());
  }

  // Bind interface handles once; read()/write() then touch no maps or strings.
  for (auto & aj : active_) {
    const std::string iface = aj.map->urdf_joint + "/" + hardware_interface::HW_IF_POSITION;
    aj.cmd = get_command_interface_handle(iface);
    aj.state = get_state_interface_handle(iface);
    // Re-resolve the mapping pointer: applyLiveLimits mutated the entries in place, and
    // the pointers stay valid, but be explicit that we want the refreshed one.
    aj.map = joint_map_.find(aj.map->urdf_joint);

    // Start from where MRL says the robot is, so RViz does not snap the model (and then
    // the robot) to some other pose the instant the controller activates.
    double start_rad = aj.map->urdf_home_rad;
    if (!mrl_cfg_.dry_run) {
      const auto pos = mrl_.getPosition(aj.map->service);
      if (pos.has_value()) {
        start_rad = aj.map->toJointRadians(*pos);
      }
    }
    aj.last_cmd_rad = start_rad;
    set_state(aj.state, start_rad, true);
    set_command(aj.cmd, start_rad, true);
  }

  mrl_.start();
  RCLCPP_INFO(
    get_logger(), "InMoov hardware ACTIVE%s", mrl_cfg_.dry_run ? " (dry run)" : "");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn InmoovSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (rest_on_deactivate_) {
    // Park everything we were driving back at its rest position before letting go, so the
    // robot is left in a known pose rather than wherever the last command happened to be.
    std::map<std::string, double> rest;
    for (const auto & aj : active_) {
      rest[aj.map->service] = aj.map->servo_rest;
    }
    mrl_.setTargets(rest);
    mrl_.forceResend();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
  mrl_.stop();
  RCLCPP_INFO(
    get_logger(), "InMoov hardware deactivated (sent=%lu errors=%lu)",
    static_cast<unsigned long>(mrl_.sentCount()),
    static_cast<unsigned long>(mrl_.errorCount()));
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type InmoovSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // MRL gives no true position feedback: InMoov servos are open-loop hobby servos with no
  // encoder, and MRL's getPos just echoes its own last commanded target. Polling it here
  // would cost one HTTP round trip per servo per cycle to learn nothing. So the state
  // interface reports the last COMMANDED position. This is an open-loop driver and the
  // /joint_states it produces are a model of the robot, not a measurement of it.
  for (auto & aj : active_) {
    set_state(aj.state, aj.last_cmd_rad, false);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type InmoovSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::map<std::string, double> targets;
  for (auto & aj : active_) {
    double q = aj.last_cmd_rad;
    if (!get_command(aj.cmd, q, false)) {
      continue;               // nothing written yet this cycle; hold the last value
    }
    if (!std::isfinite(q)) {
      continue;               // never turn a NaN into a servo command
    }
    aj.last_cmd_rad = q;
    targets[aj.map->service] = aj.map->toServoDegrees(q);
  }
  // Hand off to the background worker. It applies the move-epsilon and the anti-droop
  // refresh and does the HTTP; write() itself never blocks on the network.
  mrl_.setTargets(targets);
  return hardware_interface::return_type::OK;
}

}  // namespace inmoov_hardware

PLUGINLIB_EXPORT_CLASS(inmoov_hardware::InmoovSystem, hardware_interface::SystemInterface)
