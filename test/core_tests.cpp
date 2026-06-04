#include "vrpn_router/core/health_monitor.h"
#include "vrpn_router/core/input_rate_detector.h"
#include "vrpn_router/core/jump_detector.h"
#include "vrpn_router/core/pose_transformer.h"
#include "vrpn_router/core/rate_limiter.h"
#include "vrpn_router/core/reference_delta_detector.h"
#include "vrpn_router/core/stuck_detector.h"
#include "vrpn_router/core/timeout_detector.h"
#include "vrpn_router/core/types.h"
#include "vrpn_router/route_config.h"
#include "vrpn_router/topic_utils.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

using vrpn_router::core::HealthConfig;
using vrpn_router::core::HealthMonitor;
using vrpn_router::core::InputRateDetector;
using vrpn_router::core::JumpDetector;
using vrpn_router::core::Pose;
using vrpn_router::core::PoseTransformer;
using vrpn_router::core::Quat;
using vrpn_router::core::RateLimiter;
using vrpn_router::core::ReferenceDeltaDetector;
using vrpn_router::core::StuckDetector;
using vrpn_router::core::TimeoutDetector;
using vrpn_router::core::Transform;
using vrpn_router::core::Vec3;

TEST(PoseTransformer, AppliesTrackerToBodyAndFieldOffset) {
  PoseTransformer transformer;
  transformer.setTrackerToBody(Transform{{1.0, 0.0, 0.0}, {}});
  transformer.setFieldOffset(Vec3{10.0, -2.0, 0.5});

  const Pose out = transformer.apply(Pose{{1.0, 2.0, 3.0}, {}});
  EXPECT_NEAR(out.position.x, 12.0, 1e-9);
  EXPECT_NEAR(out.position.y, 0.0, 1e-9);
  EXPECT_NEAR(out.position.z, 3.5, 1e-9);
}

TEST(PoseTransformer, AppliesRotationToExtrinsicTranslation) {
  PoseTransformer transformer;
  transformer.setTrackerToBody(Transform{{2.0, 0.0, 0.0}, {}});

  const double s = std::sqrt(0.5);
  const Pose out = transformer.apply(Pose{{0.0, 0.0, 0.0}, {0.0, 0.0, s, s}});
  EXPECT_NEAR(out.position.x, 0.0, 1e-6);
  EXPECT_NEAR(out.position.y, 2.0, 1e-6);
  EXPECT_NEAR(out.position.z, 0.0, 1e-6);
}

TEST(RateLimiter, AllowsApproximateUniformDownsampling) {
  RateLimiter limiter(50.0);
  int published = 0;
  for (int i = 0; i < 120; ++i) {
    const double t = static_cast<double>(i) / 120.0;
    if (limiter.shouldPublish(t)) {
      limiter.markPublished(t);
      ++published;
    }
  }
  EXPECT_GE(published, 35);
  EXPECT_LE(published, 55);
}

TEST(InputRateDetector, ReportsRateBand) {
  InputRateDetector detector;
  for (int i = 0; i < 20; ++i) {
    detector.record(static_cast<double>(i) * 0.01);
  }

  EXPECT_NEAR(detector.rateHz(), 100.0, 1e-6);
  EXPECT_FALSE(detector.belowMin(50.0));
  EXPECT_TRUE(detector.aboveMax(80.0));
}

TEST(TimeoutDetector, ReportsMissingAndStaleInput) {
  TimeoutDetector detector;

  EXPECT_TRUE(detector.timedOut(1.0, 0.3));
  EXPECT_NEAR(detector.ageS(1.0), -1.0, 1e-9);

  detector.recordInput(1.0);
  EXPECT_FALSE(detector.timedOut(1.2, 0.3));
  EXPECT_TRUE(detector.timedOut(1.4, 0.3));
}

TEST(JumpDetector, ReportsTranslationJump) {
  JumpDetector detector;
  detector.evaluate(Pose{{0.0, 0.0, 0.0}, {}}, 0.5, 45.0);
  EXPECT_FALSE(detector.detected());

  detector.evaluate(Pose{{2.0, 0.0, 0.0}, {}}, 0.5, 45.0);

  EXPECT_TRUE(detector.detected());
  EXPECT_NEAR(detector.lastTranslationM(), 2.0, 1e-9);
}

TEST(JumpDetector, ReportsRotationJumpWithoutTranslationJump) {
  JumpDetector detector;
  const double s = std::sqrt(0.5);

  detector.evaluate(Pose{{0.0, 0.0, 0.0}, {}}, 0.5, 20.0);
  detector.evaluate(Pose{{0.0, 0.0, 0.0}, {0.0, 0.0, s, s}}, 0.5, 20.0);

  EXPECT_TRUE(detector.detected());
  EXPECT_NEAR(detector.lastTranslationM(), 0.0, 1e-9);
  EXPECT_GT(detector.lastRotationDeg(), 20.0);
}

TEST(JumpDetector, UsesAdjacentInputFrames) {
  JumpDetector detector;

  detector.evaluate(Pose{{0.0, 0.0, 0.0}, {}}, 0.5, 45.0);
  detector.evaluate(Pose{{10.0, 0.0, 0.0}, {}}, 0.5, 45.0);
  EXPECT_TRUE(detector.detected());

  detector.evaluate(Pose{{10.1, 0.0, 0.0}, {}}, 0.5, 45.0);
  EXPECT_FALSE(detector.detected());
  EXPECT_NEAR(detector.lastTranslationM(), 0.1, 1e-9);
}

TEST(StuckDetector, ReportsRepeatedPoseAfterTimeout) {
  StuckDetector detector;
  const Pose pose{{1.0, 0.0, 0.0}, {}};

  detector.observe(pose, 0.0, 0.001, 0.2, 0.1);
  detector.observe(pose, 0.05, 0.001, 0.2, 0.1);
  detector.observe(pose, 0.10, 0.001, 0.2, 0.1);
  EXPECT_TRUE(detector.stuck());

  detector.observe(Pose{{1.1, 0.0, 0.0}, {}}, 0.21, 0.001, 0.2, 0.1);
  EXPECT_FALSE(detector.stuck());
}

TEST(StuckDetector, DoesNotReportWhenWindowNoiseExceedsThreshold) {
  StuckDetector detector;

  detector.observe(Pose{{1.0, 0.0, 0.0}, {}}, 0.00, 5.0e-8, 0.2, 0.10);
  detector.observe(Pose{{1.0 + 1.0e-7, 0.0, 0.0}, {}}, 0.05, 5.0e-8, 0.2, 0.10);
  detector.observe(Pose{{1.0 - 1.0e-7, 0.0, 0.0}, {}}, 0.10, 5.0e-8, 0.2, 0.10);

  EXPECT_FALSE(detector.stuck());
}

TEST(StuckDetector, UsesDiscreteWindowBoundarySample) {
  StuckDetector detector;
  const Pose pose{{1.0, 0.0, 0.0}, {}};

  for (int i = 0; i <= 11; ++i) {
    detector.observe(pose, static_cast<double>(i) * 0.01, 0.001, 0.2, 0.10);
  }

  EXPECT_TRUE(detector.stuck());
}

TEST(ReferenceDeltaDetector, ReportsLargeDelta) {
  ReferenceDeltaDetector detector;
  detector.setOutput(Pose{{0.0, 0.0, 0.0}, {}});
  detector.setReference(Pose{{2.0, 0.0, 0.0}, {}});

  EXPECT_NEAR(detector.deltaM(), 2.0, 1e-9);
  EXPECT_TRUE(detector.aboveMax(0.5));
}

TEST(HealthMonitor, ReportsJumpWithoutBlockingPolicy) {
  HealthConfig config;
  config.max_translation_jump_m = 0.5;
  HealthMonitor health(config);

  health.onInput(0.0);
  health.onPublish(Pose{{0.0, 0.0, 0.0}, {}});
  health.detectJump(Pose{{0.0, 0.0, 0.0}, {}}, 0.0);
  health.detectJump(Pose{{2.0, 0.0, 0.0}, {}}, 0.1);
  const auto snapshot = health.snapshot(0.1);

  EXPECT_NE(std::find(snapshot.problems.begin(), snapshot.problems.end(), "vrpn_jump"), snapshot.problems.end());
  EXPECT_EQ(snapshot.published_count, 1u);
}

TEST(HealthMonitor, HoldsTransientJumpLongEnoughForDiagnostics) {
  HealthConfig config;
  config.max_rotation_jump_deg = 10.0;
  config.jump_report_hold_s = 0.5;
  HealthMonitor health(config);

  const double s = std::sqrt(0.5);
  health.detectJump(Pose{{0.0, 0.0, 0.0}, {}}, 1.0);
  health.detectJump(Pose{{0.0, 0.0, 0.0}, {0.0, 0.0, s, s}}, 1.1);
  health.detectJump(Pose{{0.0, 0.0, 0.0}, {0.0, 0.0, s, s}}, 1.2);

  const auto held = health.snapshot(1.2);
  EXPECT_NE(std::find(held.problems.begin(), held.problems.end(), "vrpn_jump"), held.problems.end());

  const auto expired = health.snapshot(1.61);
  EXPECT_EQ(std::find(expired.problems.begin(), expired.problems.end(), "vrpn_jump"), expired.problems.end());
}

TEST(HealthMonitor, ReportsTimeoutStuckAndReferenceDelta) {
  HealthConfig config;
  config.stale_timeout_s = 0.2;
  config.stuck_timeout_s = 0.1;
  config.max_reference_delta_m = 0.5;
  HealthMonitor health(config);

  const Pose pose{{0.0, 0.0, 0.0}, {}};
  health.onInput(0.0);
  health.updateStuckState(pose, 0.0);
  health.updateStuckState(pose, 0.05);
  health.updateStuckState(pose, 0.1);
  health.onPublish(pose);
  health.onReference(Pose{{2.0, 0.0, 0.0}, {}});
  const auto snapshot = health.snapshot(0.3);

  EXPECT_NE(std::find(snapshot.problems.begin(), snapshot.problems.end(), "vrpn_timeout"), snapshot.problems.end());
  EXPECT_NE(std::find(snapshot.problems.begin(), snapshot.problems.end(), "vrpn_stuck"), snapshot.problems.end());
  EXPECT_NE(std::find(snapshot.problems.begin(), snapshot.problems.end(), "reference_delta_high"),
            snapshot.problems.end());
}

TEST(RouteConfig, LoadsParameterizedRoute) {
  XmlRpc::XmlRpcValue route;
  route["tracker"] = "uav1";
  route["mavros_ns"] = "/uav1/mavros";
  route["max_output_rate_hz"] = 40.0;

  const vrpn_router::RouteConfig config =
      vrpn_router::loadRoute(route, 0, "pose", "vision_pose/pose", "map", vrpn_router::RouteConfig{});
  EXPECT_EQ(config.input_topic, "/vrpn_client_node/uav1/pose");
  EXPECT_EQ(config.output_topic, "/uav1/mavros/vision_pose/pose");
  EXPECT_EQ(config.output_frame_id, "map");
  EXPECT_NEAR(config.max_output_rate_hz, 40.0, 1e-9);
}

TEST(RouteConfig, RejectsInvalidNumericLimits) {
  XmlRpc::XmlRpcValue route;
  route["tracker"] = "uav1";
  route["mavros_ns"] = "/uav1/mavros";
  route["max_output_rate_hz"] = -1.0;

  EXPECT_THROW(vrpn_router::loadRoute(route, 0, "pose", "vision_pose/pose", "map", vrpn_router::RouteConfig{}),
               std::runtime_error);
}

TEST(RouteConfig, RejectsEmptyDerivedTopics) {
  XmlRpc::XmlRpcValue route;
  route["tracker"] = "uav1";
  route["mavros_ns"] = "";

  EXPECT_THROW(vrpn_router::loadRoute(route, 0, "", "", "map", vrpn_router::RouteConfig{}), std::runtime_error);
}

TEST(RouteConfig, RejectsEmptyTracker) {
  XmlRpc::XmlRpcValue route;
  route["tracker"] = "";
  route["mavros_ns"] = "/uav1/mavros";

  EXPECT_THROW(vrpn_router::loadRoute(route, 0, "pose", "vision_pose/pose", "map", vrpn_router::RouteConfig{}),
               std::runtime_error);
}

TEST(RouteConfig, RejectsNonFiniteDefaultLimit) {
  XmlRpc::XmlRpcValue route;
  route["tracker"] = "uav1";
  route["mavros_ns"] = "/uav1/mavros";

  vrpn_router::RouteConfig defaults;
  defaults.max_output_rate_hz = std::numeric_limits<double>::quiet_NaN();

  EXPECT_THROW(vrpn_router::loadRoute(route, 0, "pose", "vision_pose/pose", "map", defaults), std::runtime_error);
}

TEST(RouteConfig, RejectsZeroQuaternion) {
  XmlRpc::XmlRpcValue route;
  route["tracker"] = "uav1";
  route["mavros_ns"] = "/uav1/mavros";
  route["tracker_to_body"]["rotation"]["x"] = 0.0;
  route["tracker_to_body"]["rotation"]["y"] = 0.0;
  route["tracker_to_body"]["rotation"]["z"] = 0.0;
  route["tracker_to_body"]["rotation"]["w"] = 0.0;

  EXPECT_THROW(vrpn_router::loadRoute(route, 0, "pose", "vision_pose/pose", "map", vrpn_router::RouteConfig{}),
               std::runtime_error);
}

TEST(TopicUtils, JoinsEmptySuffixWithoutReadingPastEnd) {
  EXPECT_EQ(vrpn_router::joinTopic("", ""), "");
  EXPECT_EQ(vrpn_router::joinTopic("/", ""), "/");
  EXPECT_EQ(vrpn_router::joinTopic("/uav1/mavros", ""), "/uav1/mavros");
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
