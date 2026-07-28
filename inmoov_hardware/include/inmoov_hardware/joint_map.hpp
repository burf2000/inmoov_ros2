// Copyright 2026 Burf Robotics
//
// URDF joint <-> MyRobotLab servo mapping for the InMoov.
//
// This header is deliberately free of ROS and of any I/O so that the arithmetic can be
// unit-tested offline (see test/test_joint_map.cpp). Both consumers -- the ros2_control
// SystemInterface and the /joint_states relay node -- share this one implementation, so
// the "controller" path and the "RViz slider" path can never drift apart.

#ifndef INMOOV_HARDWARE__JOINT_MAP_HPP_
#define INMOOV_HARDWARE__JOINT_MAP_HPP_

#include <optional>
#include <string>
#include <vector>

namespace inmoov_hardware
{

/// How a joint's travel is anchored around the servo's rest position.
enum class Mode
{
  /// urdf_home_rad is inside the URDF range and servo_rest is mid-travel; the joint
  /// swings both ways from rest (head pan, arm rotate, wrist roll).
  kCentred,
  /// urdf_home_rad sits at one end of the URDF range and servo_rest is the neutral
  /// bottom of servo travel; only one direction is useful (jaw, fingers, bicep).
  kFloored,
};

struct JointMapping
{
  std::string urdf_joint;
  std::string group;        ///< bring-up group: head / eyes / rightArm / ... / torso
  std::string service;      ///< MRL service path, e.g. "i01.head.rothead"
  std::string mrl_group;
  std::string mrl_servo;

  double urdf_min_rad{0.0};
  double urdf_max_rad{0.0};
  double urdf_home_rad{0.0};

  double servo_min{0.0};
  double servo_max{180.0};
  double servo_rest{90.0};

  /// Optional hard safety cap, applied AFTER the [servo_min, servo_max] clamp.
  std::optional<double> safety_min;
  std::optional<double> safety_max;

  Mode mode{Mode::kCentred};
  bool invert{false};
  bool enabled{false};
  std::string verified{"none"};
  std::string reason;       ///< why a disabled joint is disabled

  /// Radians on the URDF joint -> degrees on the MRL servo.
  ///
  ///   d = (q - home), negated when `invert`
  ///   d >= 0 -> rest + (d / pos_span) * (servo_max - rest)
  ///   d <  0 -> rest + (d / neg_span) * (rest - servo_min)
  ///
  /// The result is clamped into [servo_min, servo_max] and then into the safety cap, so
  /// it is impossible for this function to return a value the servo cannot take.
  double toServoDegrees(double q_rad) const;

  /// Inverse of toServoDegrees. Used to seed the position state interface at activation
  /// from whatever pose MRL reports, so RViz starts where the robot actually is.
  double toJointRadians(double degrees) const;

  /// Span of URDF travel on the positive-d side (after `invert` is applied).
  double posSpanRad() const;
  /// Span of URDF travel on the negative-d side (after `invert` is applied).
  double negSpanRad() const;

  /// Effective lower/upper servo bound including any safety cap.
  double effectiveMin() const;
  double effectiveMax() const;
};

/// The whole mapping table.
class JointMap
{
public:
  /// Load from a YAML file laid out like config/inmoov_joint_map.yaml.
  /// Throws std::runtime_error with a specific message on any malformed entry -- a
  /// silently-wrong mapping on a humanoid is worse than a hard failure at startup.
  static JointMap fromFile(const std::string & path);

  /// Same, from a YAML string (used by the tests).
  static JointMap fromString(const std::string & yaml);

  const std::vector<JointMapping> & joints() const {return joints_;}
  const std::vector<std::string> & groups() const {return groups_;}

  /// nullptr when the joint is not in the table at all.
  const JointMapping * find(const std::string & urdf_joint) const;

  /// Joints that are `enabled: true` AND whose group is in `enabled_groups`.
  /// Everything else is ignored cleanly -- never an error.
  std::vector<const JointMapping *> active(
    const std::vector<std::string> & enabled_groups) const;

  /// Overwrite servo_min/servo_max/servo_rest for one service from a live MRL read.
  /// Returns false if no enabled joint uses that service (harmless, just reported).
  bool applyLiveLimits(
    const std::string & service, double min_deg, double max_deg, double rest_deg);

private:
  std::vector<JointMapping> joints_;
  std::vector<std::string> groups_;
};

/// Split "a,b , c" into {"a","b","c"}, dropping empties. Used for enabled_groups.
std::vector<std::string> splitCsv(const std::string & csv);

}  // namespace inmoov_hardware

#endif  // INMOOV_HARDWARE__JOINT_MAP_HPP_
