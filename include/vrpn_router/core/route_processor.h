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
  explicit RouteProcessor(const RouteProcessorConfig& config = {});

  void setConfig(const RouteProcessorConfig& config);

  void onReference(const Pose& pose);

  RouteProcessResult onInput(const Pose& pose, double now_s);

  HealthSnapshot snapshot(double now_s) const;

private:
  PoseTransformer transformer_;
  RateLimiter rate_limiter_;
  HealthMonitor health_;
};

} // namespace vrpn_router::core
