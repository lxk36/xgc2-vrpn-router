#include "vrpn_router/vrpn_router_node.h"

#include <ros/ros.h>

#include <exception>

int main(int argc, char** argv) {
  ros::init(argc, argv, "vrpn_router");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  try {
    vrpn_router::VrpnRouterNode node(nh, private_nh);
    ros::spin();
  } catch (const std::exception& ex) {
    ROS_FATAL_STREAM("failed to start vrpn_router: " << ex.what());
    return 1;
  }

  return 0;
}
