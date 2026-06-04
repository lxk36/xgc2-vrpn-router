#include "vrpn_router/vrpn_router_node.h"

#include "vrpn_router/route_config.h"

#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace vrpn_router {

VrpnRouterNode::VrpnRouterNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh) : diagnostics_(nh, routes_) {
  int queue_size = 10;
  double diagnostics_rate_hz = 1.0;
  private_nh.param<int>("queue_size", queue_size, queue_size);
  private_nh.param<double>("diagnostics_rate_hz", diagnostics_rate_hz, diagnostics_rate_hz);
  validateParameters(queue_size, diagnostics_rate_hz);

  const auto route_configs = loadRoutes(private_nh);
  if (route_configs.empty()) {
    throw std::runtime_error("at least one route is required");
  }

  routes_.reserve(route_configs.size());
  for (const auto& config : route_configs) {
    routes_.emplace_back(std::make_unique<PoseRoute>(nh, config, queue_size));
  }

  diagnostics_timer_ = nh.createTimer(ros::Duration(1.0 / std::max(0.1, diagnostics_rate_hz)),
                                      &DiagnosticsPublisher::publish, &diagnostics_);
}

void VrpnRouterNode::validateParameters(int queue_size, double diagnostics_rate_hz) {
  if (queue_size <= 0) {
    throw std::runtime_error("queue_size must be positive");
  }
  if (!std::isfinite(diagnostics_rate_hz) || diagnostics_rate_hz <= 0.0) {
    throw std::runtime_error("diagnostics_rate_hz must be finite and positive");
  }
}

} // namespace vrpn_router
