// Copyright 2026 Burf Robotics
//
// Offline arithmetic validation of the InMoov joint map.
//
// This is the gate: it runs against the SHIPPED config/inmoov_joint_map.yaml and proves
// the three properties that actually matter on a physical humanoid --
//   1. the URDF home pose lands exactly on each servo's REST position;
//   2. both URDF limits land INSIDE the servo's [min,max] (and inside any safety cap);
//   3. nothing, including garbage input, can produce a value outside those bounds.
// Nothing is allowed near the robot until this passes.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "inmoov_hardware/joint_map.hpp"

using inmoov_hardware::JointMap;
using inmoov_hardware::JointMapping;
using inmoov_hardware::Mode;

namespace
{
JointMap shippedMap()
{
  return JointMap::fromFile(INMOOV_JOINT_MAP_PATH);
}

std::vector<const JointMapping *> allEnabled(const JointMap & m)
{
  std::vector<const JointMapping *> out;
  for (const auto & j : m.joints()) {
    if (j.enabled) {out.push_back(&j);}
  }
  return out;
}
}  // namespace

TEST(JointMap, ShippedMapLoads)
{
  const auto m = shippedMap();
  EXPECT_FALSE(m.joints().empty());
  EXPECT_FALSE(allEnabled(m).empty());
}

// The headline property: RViz/URDF home == the robot's rest pose. If this is wrong the
// robot lurches the moment the driver activates.
TEST(JointMap, HomeMapsExactlyToRest)
{
  const auto m = shippedMap();
  for (const auto * j : allEnabled(m)) {
    const double deg = j->toServoDegrees(j->urdf_home_rad);
    EXPECT_NEAR(deg, j->servo_rest, 1e-9)
      << j->urdf_joint << " home " << j->urdf_home_rad << " rad should map to rest "
      << j->servo_rest << " but gave " << deg;
  }
}

TEST(JointMap, UrdfLimitsStayInsideServoRange)
{
  const auto m = shippedMap();
  for (const auto * j : allEnabled(m)) {
    for (const double q : {j->urdf_min_rad, j->urdf_max_rad}) {
      const double deg = j->toServoDegrees(q);
      EXPECT_GE(deg, j->effectiveMin() - 1e-9) << j->urdf_joint << " at " << q << " rad";
      EXPECT_LE(deg, j->effectiveMax() + 1e-9) << j->urdf_joint << " at " << q << " rad";
    }
  }
}

// A joint that cannot reach at least one of its servo bounds is wasting travel -- that is
// exactly the "narrow servo range floors and barely moves" failure the Quest bridge hit.
TEST(JointMap, EachJointReachesAServoExtreme)
{
  const auto m = shippedMap();
  for (const auto * j : allEnabled(m)) {
    const double at_lo = j->toServoDegrees(j->urdf_min_rad);
    const double at_hi = j->toServoDegrees(j->urdf_max_rad);
    const bool reaches_max =
      std::abs(at_lo - j->effectiveMax()) < 1e-6 || std::abs(at_hi - j->effectiveMax()) < 1e-6;
    const bool reaches_min =
      std::abs(at_lo - j->effectiveMin()) < 1e-6 || std::abs(at_hi - j->effectiveMin()) < 1e-6;
    EXPECT_TRUE(reaches_max || reaches_min)
      << j->urdf_joint << ": URDF limits map to [" << at_lo << ".." << at_hi
      << "] which never reaches servo [" << j->effectiveMin() << ".." << j->effectiveMax()
      << "] -- travel is being wasted";
    // Reaching an extreme is not enough on its own. A floored joint pinned at rest sits
    // exactly on servo_min, so it satisfies the check above while being completely dead --
    // which is precisely how a broken shoulder_out shipped and moved no servo at all.
    EXPECT_GT(std::abs(at_hi - at_lo), 1e-6)
      << j->urdf_joint << ": both ends of the URDF range map to the same servo angle ("
      << at_lo << ") -- this joint cannot move";
  }
}

// Direction sanity: moving away from home in the URDF must move the servo away from rest,
// in the direction the `invert` flag claims.
TEST(JointMap, DirectionFollowsInvertFlag)
{
  const auto m = shippedMap();
  for (const auto * j : allEnabled(m)) {
    const double span = j->urdf_max_rad - j->urdf_min_rad;
    const double step = span * 0.1;

    if (j->posSpanRad() > 1e-6) {
      const double q = j->invert ? (j->urdf_home_rad - step) : (j->urdf_home_rad + step);
      const double deg = j->toServoDegrees(q);
      if (j->servo_max > j->servo_rest + 1e-9 && j->effectiveMax() > j->servo_rest + 1e-9) {
        EXPECT_GT(deg, j->servo_rest)
          << j->urdf_joint << ": positive-side travel should raise the servo above rest";
      }
    }
    if (j->negSpanRad() > 1e-6) {
      const double q = j->invert ? (j->urdf_home_rad + step) : (j->urdf_home_rad - step);
      const double deg = j->toServoDegrees(q);
      if (j->servo_min < j->servo_rest - 1e-9 && j->effectiveMin() < j->servo_rest - 1e-9) {
        EXPECT_LT(deg, j->servo_rest)
          << j->urdf_joint << ": negative-side travel should drop the servo below rest";
      }
    }
  }
}

// Sweep the whole URDF range plus a wide margin of nonsense either side. Nothing may
// escape the servo box. This is the "never send a value outside [min,max]" guarantee.
TEST(JointMap, NothingEscapesTheServoBoxEver)
{
  const auto m = shippedMap();
  for (const auto * j : allEnabled(m)) {
    const double lo = j->urdf_min_rad - 10.0;
    const double hi = j->urdf_max_rad + 10.0;
    for (int i = 0; i <= 400; ++i) {
      const double q = lo + (hi - lo) * (static_cast<double>(i) / 400.0);
      const double deg = j->toServoDegrees(q);
      ASSERT_TRUE(std::isfinite(deg)) << j->urdf_joint << " produced a non-finite degree";
      ASSERT_GE(deg, j->effectiveMin() - 1e-9) << j->urdf_joint << " at " << q;
      ASSERT_LE(deg, j->effectiveMax() + 1e-9) << j->urdf_joint << " at " << q;
    }
  }
}

TEST(JointMap, RoundTripsThroughRadiansAndBack)
{
  const auto m = shippedMap();
  for (const auto * j : allEnabled(m)) {
    // Only meaningful where the safety cap is not squeezing the range.
    if (j->effectiveMin() != j->servo_min || j->effectiveMax() != j->servo_max) {continue;}
    for (int i = 0; i <= 20; ++i) {
      const double q = j->urdf_min_rad +
        (j->urdf_max_rad - j->urdf_min_rad) * (static_cast<double>(i) / 20.0);
      const double deg = j->toServoDegrees(q);
      const double back = j->toJointRadians(deg);
      const double again = j->toServoDegrees(back);
      EXPECT_NEAR(again, deg, 1e-6) << j->urdf_joint << " is not idempotent through degrees";
    }
  }
}

TEST(JointMap, RestRoundTripsToHome)
{
  const auto m = shippedMap();
  for (const auto * j : allEnabled(m)) {
    EXPECT_NEAR(j->toJointRadians(j->servo_rest), j->urdf_home_rad, 1e-9) << j->urdf_joint;
  }
}

// Concrete, hand-checked cases so a future edit to the YAML that quietly breaks the head
// or the jaw fails here rather than on the robot.
TEST(JointMap, KnownHeadPanNumbers)
{
  const auto m = shippedMap();
  const auto * j = m.find("head_pan_joint");
  ASSERT_NE(j, nullptr);
  ASSERT_TRUE(j->enabled);
  EXPECT_EQ(j->service, "i01.head.rothead");
  // head_pan is INVERTED: Simon watched it on the robot and it drove the wrong way, so
  // positive radians now map BELOW rest, not above it. Home still lands exactly on rest.
  EXPECT_TRUE(j->invert) << "head_pan direction was confirmed by eye; do not silently unset";
  EXPECT_NEAR(j->toServoDegrees(0.0), 90.0, 1e-9);                   // home -> rest
  EXPECT_NEAR(j->toServoDegrees(1.5707963267948966), 30.0, 1e-6);    // +90 deg -> servo min
  EXPECT_NEAR(j->toServoDegrees(-1.7453292519943295), 150.0, 1e-6);  // -100 deg -> clamped max
  // Half the positive travel is half the servo span BELOW rest. The whole mapping is
  // a clean mirror of the pre-flip numbers about rest: 150->30, 120->60.
  EXPECT_NEAR(j->toServoDegrees(0.7853981633974483), 60.0, 1e-6);
}

TEST(JointMap, KnownJawNumbersAreFloored)
{
  const auto m = shippedMap();
  const auto * j = m.find("jaw_joint");
  ASSERT_NE(j, nullptr);
  EXPECT_EQ(j->mode, Mode::kFloored);
  EXPECT_NEAR(j->toServoDegrees(0.0), 51.0, 1e-9);                    // shut == rest
  EXPECT_NEAR(j->toServoDegrees(0.08726646259971647), 131.0, 1e-6);   // wide == servo max
  EXPECT_NEAR(j->toServoDegrees(-5.0), 51.0, 1e-9);                   // below range clamps to rest
}

// The bicep is the awkward one: its servo rest IS its servo min, so all of its travel is on
// one side of rest. If the mapping model is wrong anywhere it is wrong here.
//
// Re-based 2026-07-28: the range used to run entirely NEGATIVE with home at the top of it,
// so from rest the slider could only travel one way and that way felt backwards. It is now
// [0, span] with home 0 — slider zero is the straight arm, positive flexes it — and invert
// flipped to keep the physical motion identical.
TEST(JointMap, KnownRightElbowNumbers)
{
  const auto m = shippedMap();
  const auto * j = m.find("r_elbow_flex_joint");
  ASSERT_NE(j, nullptr);
  EXPECT_FALSE(j->invert);
  EXPECT_NEAR(j->servo_rest, j->servo_min, 1e-9);
  EXPECT_NEAR(j->urdf_home_rad, 0.0, 1e-9) << "slider zero must be the rest pose";
  EXPECT_NEAR(j->toServoDegrees(0.0), 5.0, 1e-9);                   // straight == rest
  // Span matched to the left elbow 2026-07-28: config.yaml had the right at 65 degrees and
  // the left at 70, so at full slider the model showed one arm more bent than the other
  // while MRL had them level.
  EXPECT_NEAR(j->toServoDegrees(1.2217304763960306), 90.0, 1e-6);   // fully flexed == servo max
  EXPECT_NEAR(j->toServoDegrees(1.2217304763960306 / 2.0), 47.5, 1e-6);
}

TEST(JointMap, SafetyCapIsHonoured)
{
  const auto m = shippedMap();
  const auto * j = m.find("head_tilt_joint");
  ASSERT_NE(j, nullptr);
  ASSERT_TRUE(j->safety_min.has_value());
  ASSERT_TRUE(j->safety_max.has_value());
  EXPECT_LE(j->toServoDegrees(j->urdf_max_rad), *j->safety_max + 1e-9);
  EXPECT_GE(j->toServoDegrees(j->urdf_min_rad), *j->safety_min - 1e-9);
}

TEST(JointMap, ParkedJointsAreIgnoredNotErrors)
{
  const auto m = shippedMap();
  const auto * j = m.find("l_eye_pan_joint");
  ASSERT_NE(j, nullptr);
  EXPECT_FALSE(j->enabled);
  EXPECT_FALSE(j->reason.empty());

  const auto * finger_sub = m.find("r_index1_joint");
  ASSERT_NE(finger_sub, nullptr);
  EXPECT_FALSE(finger_sub->enabled);

  // ...while the one joint per finger that IS driven is enabled.
  const auto * finger = m.find("r_index_joint");
  ASSERT_NE(finger, nullptr);
  EXPECT_TRUE(finger->enabled);
}

TEST(JointMap, DefaultEnabledGroupIsHeadOnly)
{
  const auto m = shippedMap();
  const auto act = m.active(inmoov_hardware::splitCsv("head"));
  ASSERT_FALSE(act.empty());
  for (const auto * j : act) {
    EXPECT_EQ(j->group, "head");
  }
  // Sanity: widening picks up more.
  EXPECT_GT(m.active(inmoov_hardware::splitCsv("head,eyes,rightArm")).size(), act.size());
}

TEST(JointMap, EveryEnabledJointHasAUniqueService)
{
  const auto m = shippedMap();
  const auto en = allEnabled(m);
  for (size_t a = 0; a < en.size(); ++a) {
    EXPECT_FALSE(en[a]->service.empty()) << en[a]->urdf_joint;
    for (size_t b = a + 1; b < en.size(); ++b) {
      EXPECT_NE(en[a]->service, en[b]->service)
        << en[a]->urdf_joint << " and " << en[b]->urdf_joint << " share a servo";
    }
  }
}

TEST(JointMap, RejectsRestOutsideServoRange)
{
  const std::string bad = R"(
groups: [head]
joints:
  - urdf_joint: bad_joint
    group: head
    service: i01.head.rothead
    urdf_min_rad: -1.0
    urdf_max_rad: 1.0
    urdf_home_rad: 0.0
    servo_min: 30.0
    servo_max: 150.0
    servo_rest: 200.0
    mode: centred
    enabled: true
)";
  EXPECT_THROW(JointMap::fromString(bad), std::runtime_error);
}

TEST(JointMap, RejectsTwoJointsOnOneServo)
{
  const std::string bad = R"(
groups: [head]
joints:
  - urdf_joint: a
    group: head
    service: i01.head.rothead
    urdf_min_rad: -1.0
    urdf_max_rad: 1.0
    urdf_home_rad: 0.0
    servo_min: 30.0
    servo_max: 150.0
    servo_rest: 90.0
    mode: centred
    enabled: true
  - urdf_joint: b
    group: head
    service: i01.head.rothead
    urdf_min_rad: -1.0
    urdf_max_rad: 1.0
    urdf_home_rad: 0.0
    servo_min: 30.0
    servo_max: 150.0
    servo_rest: 90.0
    mode: centred
    enabled: true
)";
  EXPECT_THROW(JointMap::fromString(bad), std::runtime_error);
}

TEST(JointMap, LiveLimitsOverrideYamlDefaults)
{
  auto m = shippedMap();
  const auto * before = m.find("head_pan_joint");
  ASSERT_NE(before, nullptr);
  ASSERT_TRUE(m.applyLiveLimits("i01.head.rothead", 40.0, 140.0, 95.0));
  const auto * after = m.find("head_pan_joint");
  EXPECT_NEAR(after->servo_min, 40.0, 1e-9);
  EXPECT_NEAR(after->servo_max, 140.0, 1e-9);
  EXPECT_NEAR(after->servo_rest, 95.0, 1e-9);
  // ...and home still lands on the NEW rest, which is the whole point of re-reading.
  EXPECT_NEAR(after->toServoDegrees(after->urdf_home_rad), 95.0, 1e-9);
  EXPECT_FALSE(m.applyLiveLimits("i01.not.a.servo", 0.0, 180.0, 90.0));
}
