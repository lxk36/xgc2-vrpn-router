#pragma once

#include "vrpn_router/core/health_monitor.h"
#include "vrpn_router/core/pose_transformer.h"
#include "vrpn_router/core/rate_limiter.h"

namespace vrpn_router::core {

struct RouteProcessorConfig {
  Transform tracker_to_body;
  Vec3 field_offset;
  double max_output_rate_hz = 50.0;
  HealthConfig health;
};

struct RouteProcessResult {
  bool should_publish = false;
  bool dropped_by_rate = false;
  Pose output;
};

class RouteProcessor {
public:
  explicit RouteProcessor(const RouteProcessorConfig& config = {})
      : rate_limiter_(config.max_output_rate_hz), health_(config.health) {
    transformer_.setTrackerToBody(config.tracker_to_body);
    transformer_.setFieldOffset(config.field_offset);
  }

  void setConfig(const RouteProcessorConfig& config) {
    transformer_.setTrackerToBody(config.tracker_to_body);
    transformer_.setFieldOffset(config.field_offset);
    rate_limiter_.setMaxRate(config.max_output_rate_hz);
    health_ = HealthMonitor(config.health);
  }

  void onReference(const Pose& pose) { health_.onReference(pose); }

  RouteProcessResult onInput(const Pose& pose, double now_s) {
    health_.onInput(now_s);

    RouteProcessResult result;
    result.output = transformer_.apply(pose);
    health_.updateStuckState(result.output, now_s);
    health_.detectJump(result.output, now_s);

    if (!rate_limiter_.shouldPublish(now_s)) {
      health_.onRateDrop();
      result.dropped_by_rate = true;
      return result;
    }

    rate_limiter_.markPublished(now_s);
    health_.onPublish(result.output);
    result.should_publish = true;
    return result;
  }

  HealthSnapshot snapshot(double now_s) const { return health_.snapshot(now_s); }

private:
  PoseTransformer transformer_;
  RateLimiter rate_limiter_;
  HealthMonitor health_;
};

} // namespace vrpn_router::core
