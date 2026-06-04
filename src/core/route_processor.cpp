#include "vrpn_router/core/route_processor.h"

namespace vrpn_router::core {

RouteProcessor::RouteProcessor(const RouteProcessorConfig& config)
    : rate_limiter_(config.max_output_rate_hz), health_(config.health) {
  transformer_.setTrackerToBody(config.tracker_to_body);
  transformer_.setFieldOffset(config.field_offset);
}

void RouteProcessor::setConfig(const RouteProcessorConfig& config) {
  transformer_.setTrackerToBody(config.tracker_to_body);
  transformer_.setFieldOffset(config.field_offset);
  rate_limiter_.setMaxRate(config.max_output_rate_hz);
  health_ = HealthMonitor(config.health);
}

void RouteProcessor::onReference(const Pose& pose) {
  health_.onReference(pose);
}

RouteProcessResult RouteProcessor::onInput(const Pose& pose, double now_s) {
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

HealthSnapshot RouteProcessor::snapshot(double now_s) const {
  return health_.snapshot(now_s);
}

} // namespace vrpn_router::core
