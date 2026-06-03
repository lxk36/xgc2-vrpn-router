#pragma once

#include "vrpn_router/pose_route.h"

#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/timer.h>

#include <memory>
#include <vector>

namespace vrpn_router {

class DiagnosticsPublisher {
 public:
  DiagnosticsPublisher(ros::NodeHandle& nh, const std::vector<std::unique_ptr<PoseRoute>>& routes)
      : routes_(routes) {
    publisher_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10);
  }

  void publish(const ros::TimerEvent&) {
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    for (const auto& route : routes_) {
      array.status.push_back(route->diagnosticStatus(array.header.stamp));
    }
    publisher_.publish(array);
  }

 private:
  const std::vector<std::unique_ptr<PoseRoute>>& routes_;
  ros::Publisher publisher_;
};

}  // namespace vrpn_router
