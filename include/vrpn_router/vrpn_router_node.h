#pragma once

#include "vrpn_router/diagnostics_publisher.h"
#include "vrpn_router/pose_route.h"

#include <ros/node_handle.h>
#include <ros/timer.h>

#include <memory>
#include <vector>

namespace vrpn_router {

class VrpnRouterNode {
public:
  VrpnRouterNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

private:
  static void validateParameters(int queue_size, double diagnostics_rate_hz);

  std::vector<std::unique_ptr<PoseRoute>> routes_;
  DiagnosticsPublisher diagnostics_;
  ros::Timer diagnostics_timer_;
};

} // namespace vrpn_router
