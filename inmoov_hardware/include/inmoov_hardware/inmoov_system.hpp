// Copyright 2026 Burf Robotics
#ifndef INMOOV_HARDWARE__INMOOV_SYSTEM_HPP_
#define INMOOV_HARDWARE__INMOOV_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "inmoov_hardware/joint_map.hpp"
#include "inmoov_hardware/mrl_client.hpp"

namespace inmoov_hardware
{

/// ros2_control SystemInterface that drives the real InMoov through MyRobotLab.
///
/// Runs on skynet; MRL lives on pi5-inmoov and is reached over plain HTTP. Nothing is
/// installed on the pi5.
class InmoovSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct ActiveJoint
  {
    const JointMapping * map{nullptr};
    hardware_interface::CommandInterface::SharedPtr cmd;
    hardware_interface::StateInterface::SharedPtr state;
    double last_cmd_rad{0.0};
  };

  JointMap joint_map_;
  std::vector<std::string> enabled_groups_;
  std::vector<ActiveJoint> active_;
  /// Joints in the URDF that this driver deliberately does not drive. They still get
  /// state/command interfaces (ros2_control demands it) but their commands go nowhere.
  std::vector<std::string> parked_joints_;

  MrlClient mrl_;
  MrlClientConfig mrl_cfg_;
  bool refresh_limits_{true};
  bool rest_on_deactivate_{true};
  bool require_mrl_{true};
};

}  // namespace inmoov_hardware

#endif  // INMOOV_HARDWARE__INMOOV_SYSTEM_HPP_
