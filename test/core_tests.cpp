#include "vrpn_router/core/health_monitor.h"
#include "vrpn_router/core/pose_transformer.h"
#include "vrpn_router/core/rate_limiter.h"
#include "vrpn_router/core/types.h"
#include "vrpn_router/route_config.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

using vrpn_router::core::HealthConfig;
using vrpn_router::core::HealthMonitor;
using vrpn_router::core::Pose;
using vrpn_router::core::PoseTransformer;
using vrpn_router::core::Quat;
using vrpn_router::core::RateLimiter;
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

TEST(HealthMonitor, ReportsJumpWithoutBlockingPolicy) {
  HealthConfig config;
  config.max_translation_jump_m = 0.5;
  HealthMonitor health(config);

  health.onInput(0.0);
  health.onPublish(Pose{{0.0, 0.0, 0.0}, {}});
  health.detectJump(Pose{{2.0, 0.0, 0.0}, {}});
  const auto snapshot = health.snapshot(0.1);

  EXPECT_NE(std::find(snapshot.problems.begin(), snapshot.problems.end(), "vrpn_jump"), snapshot.problems.end());
  EXPECT_EQ(snapshot.published_count, 1u);
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
  health.onPublish(pose);
  health.onReference(Pose{{2.0, 0.0, 0.0}, {}});
  health.updateStuckState(pose, 0.2);
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

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
