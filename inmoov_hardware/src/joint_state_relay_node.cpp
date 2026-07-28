// Copyright 2026 Burf Robotics
//
// joint_state_relay
// =================
// Subscribes to /joint_states and drives the real InMoov through MyRobotLab, using the
// SAME JointMap + MrlClient the ros2_control hardware interface uses. Not one line of
// mapping arithmetic is duplicated.
//
// Why this exists: Simon wants the RViz **joint_state_publisher_gui sliders** to move the
// real robot. Those sliders publish straight to /joint_states -- they do not go through a
// controller, so ros2_control never sees them. This node is the bridge for that path.
//
// It is the same "one thing drives MRL at a time" rule as everything else: do not run this
// AND the ros2_control driver AND the pi5 services at once.

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "inmoov_hardware/joint_map.hpp"
#include "inmoov_hardware/mrl_client.hpp"

namespace inmoov_hardware
{

class JointStateRelay : public rclcpp::Node
{
public:
  JointStateRelay()
  : rclcpp::Node("inmoov_joint_state_relay")
  {
    const std::string default_map =
      ament_index_cpp::get_package_share_directory("inmoov_hardware") +
      "/config/inmoov_joint_map.yaml";

    const auto map_path = declare_parameter<std::string>("joint_map", default_map);
    // Same default as the hardware interface: head only. Widen deliberately.
    const auto groups_csv = declare_parameter<std::string>("enabled_groups", "head");
    MrlClientConfig cfg;
    cfg.host = declare_parameter<std::string>("mrl_host", "pi5-inmoov.local");
    cfg.port = static_cast<int>(declare_parameter<int>("mrl_port", 8888));
    cfg.dry_run = declare_parameter<bool>("dry_run", false);
    cfg.move_epsilon_deg = declare_parameter<double>("move_epsilon_deg", 1.0);
    cfg.hold_refresh_s = declare_parameter<double>("hold_refresh_s", 0.5);
    cfg.worker_rate_hz = declare_parameter<double>("worker_rate_hz", 20.0);
    const bool refresh_limits = declare_parameter<bool>("refresh_limits", true);
    rest_on_shutdown_ = declare_parameter<bool>("rest_on_shutdown", true);

    try {
      map_ = JointMap::fromFile(map_path);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(get_logger(), "Cannot load joint map '%s': %s", map_path.c_str(), e.what());
      throw;
    }
    enabled_groups_ = splitCsv(groups_csv);
    active_ = map_.active(enabled_groups_);

    auto logger = get_logger();
    mrl_.configure(
      cfg,
      [logger](const std::string & m) {RCLCPP_INFO(logger, "%s", m.c_str());},
      [logger](const std::string & m) {RCLCPP_WARN(logger, "%s", m.c_str());});

    if (!cfg.dry_run) {
      if (!mrl_.ping()) {
        RCLCPP_ERROR(
          get_logger(), "MRL not reachable at %s:%d -- relay will keep trying but nothing "
          "will move.", cfg.host.c_str(), cfg.port);
      } else if (refresh_limits) {
        size_t ok = 0;
        for (const auto * j : active_) {
          const auto live = mrl_.getLimits(j->service);
          if (live.has_value()) {
            map_.applyLiveLimits(j->service, live->min, live->max, live->rest);
            ++ok;
          }
        }
        // active_ points into map_'s storage, which applyLiveLimits mutated in place, so
        // the pointers stay valid and now carry the live numbers. Re-resolve anyway to
        // make that non-load-bearing.
        active_ = map_.active(enabled_groups_);
        RCLCPP_INFO(get_logger(), "Refreshed live limits for %zu/%zu servos", ok, active_.size());
      }
    } else {
      RCLCPP_WARN(get_logger(), "DRY RUN: nothing will be sent to MRL.");
    }

    RCLCPP_INFO(
      get_logger(), "joint_state_relay driving %zu joint(s) from /joint_states; groups=[%s]",
      active_.size(), groups_csv.c_str());
    for (const auto * j : active_) {
      RCLCPP_INFO(
        get_logger(), "  %-24s -> %-24s rest=%.0f [%.0f..%.0f] %s%s",
        j->urdf_joint.c_str(), j->service.c_str(), j->servo_rest,
        j->effectiveMin(), j->effectiveMax(),
        j->mode == Mode::kCentred ? "centred" : "floored", j->invert ? " inverted" : "");
    }

    mrl_.start();
    sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(10),
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {onJointStates(*msg);});
  }

  ~JointStateRelay() override
  {
    if (rest_on_shutdown_) {
      std::map<std::string, double> rest;
      for (const auto * j : active_) {
        rest[j->service] = j->servo_rest;
      }
      mrl_.setTargets(rest);
      mrl_.forceResend();
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    mrl_.stop();
  }

private:
  void onJointStates(const sensor_msgs::msg::JointState & msg)
  {
    std::map<std::string, double> targets;
    for (size_t i = 0; i < msg.name.size() && i < msg.position.size(); ++i) {
      const auto * j = map_.find(msg.name[i]);
      if (j == nullptr || !j->enabled) {
        continue;              // unmapped or parked joint -- ignored cleanly, not an error
      }
      if (std::find(enabled_groups_.begin(), enabled_groups_.end(), j->group) ==
        enabled_groups_.end())
      {
        continue;              // group not enabled for this run
      }
      if (!std::isfinite(msg.position[i])) {
        continue;
      }
      targets[j->service] = j->toServoDegrees(msg.position[i]);
    }
    if (!targets.empty()) {
      mrl_.setTargets(targets);
    }
  }

  JointMap map_;
  std::vector<std::string> enabled_groups_;
  std::vector<const JointMapping *> active_;
  MrlClient mrl_;
  bool rest_on_shutdown_{true};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
};

}  // namespace inmoov_hardware

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<inmoov_hardware::JointStateRelay>();
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
