#!/usr/bin/env bash
set -euo pipefail

dpkg -s ros-noetic-xgc2-vrpn-router >/dev/null
test -x /opt/ros/noetic/lib/vrpn_router/vrpn_router_node
test -f /opt/ros/noetic/share/vrpn_router/launch/vrpn_router.launch
test -f /opt/ros/noetic/share/vrpn_router/config/routes.example.yaml
roslaunch --files vrpn_router vrpn_router.launch >/dev/null
