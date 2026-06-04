#include "vrpn_router/diagnostics_publisher.h"
#include "vrpn_router/pose_route.h"
#include "vrpn_router/route_config.h"

#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <memory>
#include <stdexcept>
#include <vector>

namespace {

void validateNodeParameters(int queue_size, double diagnostics_rate_hz) {
  if (queue_size <= 0) {
    throw std::runtime_error("queue_size must be positive");
  }
  if (!std::isfinite(diagnostics_rate_hz) || diagnostics_rate_hz <= 0.0) {
    throw std::runtime_error("diagnostics_rate_hz must be finite and positive");
  }
}

} // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "vrpn_router");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  int queue_size = 10;
  double diagnostics_rate_hz = 1.0;
  pnh.param<int>("queue_size", queue_size, queue_size);
  pnh.param<double>("diagnostics_rate_hz", diagnostics_rate_hz, diagnostics_rate_hz);

  try {
    validateNodeParameters(queue_size, diagnostics_rate_hz);
    const auto route_configs = vrpn_router::loadRoutes(pnh);
    if (route_configs.empty()) {
      throw std::runtime_error("at least one route is required");
    }

    std::vector<std::unique_ptr<vrpn_router::PoseRoute>> routes;
    routes.reserve(route_configs.size());
    for (const auto& config : route_configs) {
      routes.emplace_back(std::make_unique<vrpn_router::PoseRoute>(nh, config, queue_size));
    }

    vrpn_router::DiagnosticsPublisher diagnostics(nh, routes);
    ros::Timer diagnostics_timer = nh.createTimer(ros::Duration(1.0 / std::max(0.1, diagnostics_rate_hz)),
                                                  &vrpn_router::DiagnosticsPublisher::publish, &diagnostics);

    ros::spin();
  } catch (const std::exception& ex) {
    ROS_FATAL_STREAM("failed to start vrpn_router: " << ex.what());
    return 1;
  }

  return 0;
}
