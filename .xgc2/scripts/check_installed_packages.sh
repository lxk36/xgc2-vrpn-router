#!/usr/bin/env bash
set -euo pipefail

dpkg -s ros-noetic-xgc2-vrpn-router >/dev/null
test -x /opt/ros/noetic/lib/vrpn_router/vrpn_router_node
test -f /opt/ros/noetic/lib/libvrpn_router_core.so
test -f /opt/ros/noetic/lib/libvrpn_router_ros.so
test -f /opt/ros/noetic/include/vrpn_router/core/route_processor.h
test -f /opt/ros/noetic/include/vrpn_router/vrpn_router_node.h
"$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/check_core_libraries.sh"
test -f /opt/ros/noetic/share/vrpn_router/launch/vrpn_router.launch
test -f /opt/ros/noetic/share/vrpn_router/config/routes.example.yaml
roslaunch --files vrpn_router vrpn_router.launch >/dev/null
roslaunch --dump-params vrpn_router vrpn_router.launch \
  use_server_time:=true \
  refresh_tracker_frequency:=0.0 \
  trackers:=uav1 \
  | grep -F "/vrpn_client_node/use_server_time: true" >/dev/null
roslaunch --dump-params vrpn_router vrpn_router.launch \
  refresh_tracker_frequency:=0.0 \
  trackers:=uav1 \
  | grep -F "/vrpn_client_node/broadcast_tf: false" >/dev/null
roslaunch --dump-params vrpn_router vrpn_router.launch \
  use_server_time:=true \
  refresh_tracker_frequency:=0.0 \
  trackers:=uav1 \
  | grep -F -- "- uav1" >/dev/null
