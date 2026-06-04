#pragma once

#include "vrpn_router/core/types.h"

#include <geometry_msgs/Pose.h>

namespace vrpn_router {

inline core::Pose toCorePose(const geometry_msgs::Pose& pose) {
  return {{pose.position.x, pose.position.y, pose.position.z},
          {pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w}};
}

inline geometry_msgs::Pose toRosPose(const core::Pose& pose) {
  geometry_msgs::Pose ros_pose;
  ros_pose.position.x = pose.position.x;
  ros_pose.position.y = pose.position.y;
  ros_pose.position.z = pose.position.z;
  ros_pose.orientation.x = pose.orientation.x;
  ros_pose.orientation.y = pose.orientation.y;
  ros_pose.orientation.z = pose.orientation.z;
  ros_pose.orientation.w = pose.orientation.w;
  return ros_pose;
}

} // namespace vrpn_router
