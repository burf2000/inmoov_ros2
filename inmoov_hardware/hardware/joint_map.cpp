// Copyright 2026 Burf Robotics
#include "inmoov_hardware/joint_map.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace inmoov_hardware
{

namespace
{

double clampd(double v, double lo, double hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}

Mode parseMode(const std::string & s, const std::string & joint)
{
  if (s == "centred" || s == "centered") {return Mode::kCentred;}
  if (s == "floored") {return Mode::kFloored;}
  throw std::runtime_error(
          "joint '" + joint + "': mode must be centred or floored, got '" + s + "'");
}

double req(const YAML::Node & n, const char * key, const std::string & joint)
{
  if (!n[key]) {
    throw std::runtime_error(
            "joint '" + joint + "': missing required key '" + std::string(key) + "'");
  }
  return n[key].as<double>();
}

/// Parse one `joints:` entry. Disabled entries are allowed to carry nothing else.
JointMapping parseJoint(const YAML::Node & n)
{
  JointMapping j;
  if (!n["urdf_joint"]) {
    throw std::runtime_error("joint map: an entry has no 'urdf_joint'");
  }
  j.urdf_joint = n["urdf_joint"].as<std::string>();
  j.group = n["group"] ? n["group"].as<std::string>() : std::string{};
  j.enabled = n["enabled"] ? n["enabled"].as<bool>() : false;
  j.reason = n["reason"] ? n["reason"].as<std::string>() : std::string{};
  j.verified = n["verified"] ? n["verified"].as<std::string>() : std::string{"none"};

  if (!j.enabled) {
    // The clean-ignore path: ~25 URDF joints (finger sub-links, the second eye pan)
    // have no servo behind them. They are parked, not an error.
    return j;
  }

  if (!n["service"]) {
    throw std::runtime_error("joint '" + j.urdf_joint + "': enabled but has no MRL 'service'");
  }
  j.service = n["service"].as<std::string>();
  j.mrl_group = n["mrl_group"] ? n["mrl_group"].as<std::string>() : std::string{};
  j.mrl_servo = n["mrl_servo"] ? n["mrl_servo"].as<std::string>() : std::string{};

  j.urdf_min_rad = req(n, "urdf_min_rad", j.urdf_joint);
  j.urdf_max_rad = req(n, "urdf_max_rad", j.urdf_joint);
  j.urdf_home_rad = req(n, "urdf_home_rad", j.urdf_joint);
  j.servo_min = req(n, "servo_min", j.urdf_joint);
  j.servo_max = req(n, "servo_max", j.urdf_joint);
  j.servo_rest = req(n, "servo_rest", j.urdf_joint);
  j.mode = parseMode(
    n["mode"] ? n["mode"].as<std::string>() : std::string{"centred"}, j.urdf_joint);
  j.invert = n["invert"] ? n["invert"].as<bool>() : false;
  if (n["safety_min"]) {j.safety_min = n["safety_min"].as<double>();}
  if (n["safety_max"]) {j.safety_max = n["safety_max"].as<double>();}

  // Structural sanity. Any of these being wrong on a humanoid means a servo is asked to
  // go somewhere it physically cannot, so fail loudly at load rather than at 50 Hz.
  if (j.urdf_min_rad >= j.urdf_max_rad) {
    throw std::runtime_error("joint '" + j.urdf_joint + "': urdf_min_rad >= urdf_max_rad");
  }
  if (j.servo_min > j.servo_max) {
    throw std::runtime_error("joint '" + j.urdf_joint + "': servo_min > servo_max");
  }
  if (j.urdf_home_rad < j.urdf_min_rad - 1e-9 || j.urdf_home_rad > j.urdf_max_rad + 1e-9) {
    throw std::runtime_error("joint '" + j.urdf_joint + "': urdf_home_rad outside URDF limits");
  }
  if (j.servo_rest < j.servo_min - 1e-9 || j.servo_rest > j.servo_max + 1e-9) {
    throw std::runtime_error(
            "joint '" + j.urdf_joint + "': servo_rest outside [servo_min,servo_max]");
  }
  if (j.mode == Mode::kFloored) {
    // Floored means home sits AT one end of the URDF range: exactly one side has travel.
    const bool at_low = std::abs(j.urdf_home_rad - j.urdf_min_rad) < 1e-6;
    const bool at_high = std::abs(j.urdf_home_rad - j.urdf_max_rad) < 1e-6;
    if (!at_low && !at_high) {
      // Fingers are the deliberate exception: home 0 with ~5 deg of unused
      // hyper-extension below it. Tolerate that much and no more.
      if ((j.urdf_home_rad - j.urdf_min_rad) > 0.2) {
        throw std::runtime_error(
                "joint '" + j.urdf_joint +
                "': mode floored but urdf_home_rad is well inside the range");
      }
    }
  }
  return j;
}

void checkNoDuplicateServices(const std::vector<JointMapping> & joints)
{
  for (size_t a = 0; a < joints.size(); ++a) {
    if (!joints[a].enabled) {continue;}
    for (size_t b = a + 1; b < joints.size(); ++b) {
      if (joints[b].enabled && joints[a].service == joints[b].service) {
        throw std::runtime_error(
                "joints '" + joints[a].urdf_joint + "' and '" + joints[b].urdf_joint +
                "' both drive servo " + joints[a].service);
      }
    }
  }
}

}  // namespace

// ---------------------------------------------------------------------- mapping ----

double JointMapping::posSpanRad() const
{
  return invert ? (urdf_home_rad - urdf_min_rad) : (urdf_max_rad - urdf_home_rad);
}

double JointMapping::negSpanRad() const
{
  return invert ? (urdf_max_rad - urdf_home_rad) : (urdf_home_rad - urdf_min_rad);
}

double JointMapping::effectiveMin() const
{
  return safety_min.has_value() ? std::max(servo_min, *safety_min) : servo_min;
}

double JointMapping::effectiveMax() const
{
  return safety_max.has_value() ? std::min(servo_max, *safety_max) : servo_max;
}

double JointMapping::toServoDegrees(double q_rad) const
{
  // Stay inside the URDF limits first -- a controller or a rogue /joint_states message
  // must not be able to push us past what the model itself says the joint can do.
  const double q = clampd(q_rad, urdf_min_rad, urdf_max_rad);

  double d = q - urdf_home_rad;
  if (invert) {
    d = -d;
  }

  double deg = servo_rest;
  if (d >= 0.0) {
    const double span = posSpanRad();
    // span == 0 means there is no travel on this side (a floored joint anchored at its
    // own end stop). Holding at rest is the correct answer, not a divide by zero.
    if (span > 1e-9) {
      deg = servo_rest + (d / span) * (servo_max - servo_rest);
    }
  } else {
    const double span = negSpanRad();
    if (span > 1e-9) {
      deg = servo_rest + (d / span) * (servo_rest - servo_min);
    }
  }

  // Two clamps, in this order: the servo's own calibrated travel, then the safety cap.
  deg = clampd(deg, servo_min, servo_max);
  return clampd(deg, effectiveMin(), effectiveMax());
}

double JointMapping::toJointRadians(double degrees) const
{
  const double deg = clampd(degrees, servo_min, servo_max);
  const double d_deg = deg - servo_rest;

  double d_rad = 0.0;
  if (d_deg >= 0.0) {
    const double deg_span = servo_max - servo_rest;
    if (deg_span > 1e-9) {
      d_rad = (d_deg / deg_span) * posSpanRad();
    }
  } else {
    const double deg_span = servo_rest - servo_min;
    if (deg_span > 1e-9) {
      d_rad = (d_deg / deg_span) * negSpanRad();
    }
  }

  if (invert) {
    d_rad = -d_rad;
  }
  return clampd(urdf_home_rad + d_rad, urdf_min_rad, urdf_max_rad);
}

// ------------------------------------------------------------------------- map ----

const JointMapping * JointMap::find(const std::string & urdf_joint) const
{
  for (const auto & j : joints_) {
    if (j.urdf_joint == urdf_joint) {return &j;}
  }
  return nullptr;
}

std::vector<const JointMapping *> JointMap::active(
  const std::vector<std::string> & enabled_groups) const
{
  std::vector<const JointMapping *> out;
  for (const auto & j : joints_) {
    if (!j.enabled) {continue;}
    if (std::find(enabled_groups.begin(), enabled_groups.end(), j.group) ==
      enabled_groups.end())
    {
      continue;
    }
    out.push_back(&j);
  }
  return out;
}

bool JointMap::applyLiveLimits(
  const std::string & service, double min_deg, double max_deg, double rest_deg)
{
  bool hit = false;
  for (auto & j : joints_) {
    if (j.enabled && j.service == service) {
      j.servo_min = min_deg;
      j.servo_max = max_deg;
      j.servo_rest = clampd(rest_deg, min_deg, max_deg);
      hit = true;
    }
  }
  return hit;
}

JointMap JointMap::fromFile(const std::string & path)
{
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("joint map: cannot open '" + path + "'");
  }
  std::stringstream buf;
  buf << in.rdbuf();
  return fromString(buf.str());
}

JointMap JointMap::fromString(const std::string & yaml)
{
  const YAML::Node root = YAML::Load(yaml);

  JointMap map;
  if (root["groups"]) {
    for (const auto & g : root["groups"]) {
      map.groups_.push_back(g.as<std::string>());
    }
  }
  if (!root["joints"] || !root["joints"].IsSequence()) {
    throw std::runtime_error("joint map: missing 'joints' sequence");
  }
  for (const auto & n : root["joints"]) {
    map.joints_.push_back(parseJoint(n));
  }
  checkNoDuplicateServices(map.joints_);
  return map;
}

std::vector<std::string> splitCsv(const std::string & csv)
{
  std::vector<std::string> out;
  std::stringstream ss(csv);
  std::string item;
  while (std::getline(ss, item, ',')) {
    const auto b = item.find_first_not_of(" \t");
    if (b == std::string::npos) {continue;}
    const auto e = item.find_last_not_of(" \t");
    out.push_back(item.substr(b, e - b + 1));
  }
  return out;
}

}  // namespace inmoov_hardware
