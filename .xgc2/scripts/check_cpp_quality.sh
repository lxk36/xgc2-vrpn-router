#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/../.." && pwd)"
ros_distro="${ROS_DISTRO:-noetic}"
workspace="${XGC2_VRPN_ROUTER_QUALITY_WS:-${repo_root}/.ci/cpp-quality}"

sources=(
  include/vrpn_router/core/health_monitor.h
  include/vrpn_router/core/input_rate_detector.h
  include/vrpn_router/core/jump_detector.h
  include/vrpn_router/core/pose_transformer.h
  include/vrpn_router/core/rate_limiter.h
  include/vrpn_router/core/reference_delta_detector.h
  include/vrpn_router/core/route_processor.h
  include/vrpn_router/core/stuck_detector.h
  include/vrpn_router/core/timeout_detector.h
  include/vrpn_router/core/types.h
  include/vrpn_router/diagnostics_publisher.h
  include/vrpn_router/diagnostics_utils.h
  include/vrpn_router/pose_route.h
  include/vrpn_router/ros_pose_adapter.h
  include/vrpn_router/route_config.h
  include/vrpn_router/topic_utils.h
  include/vrpn_router/vrpn_router_node.h
  include/vrpn_router/xmlrpc_config.h
  src/core/route_processor.cpp
  src/vrpn_router_main.cpp
  src/vrpn_router_node.cpp
  test/core_tests.cpp
  test/vrpn_test_server_node.cpp
)

for tool in catkin_make clang-format clang-tidy; do
  if ! command -v "${tool}" >/dev/null 2>&1; then
    echo "Missing required C++ quality tool: ${tool}" >&2
    exit 1
  fi
done

if [[ ! -f "/opt/ros/${ros_distro}/setup.bash" ]]; then
  echo "Missing ROS setup: /opt/ros/${ros_distro}/setup.bash" >&2
  exit 1
fi

cd "${repo_root}"

clang-format --dry-run --Werror "${sources[@]}"

rm -rf "${workspace}"
mkdir -p "${workspace}/src"
ln -s "${repo_root}" "${workspace}/src/vrpn_router"

# shellcheck source=/dev/null
source "/opt/ros/${ros_distro}/setup.bash"

catkin_make -C "${workspace}" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DCMAKE_CXX_FLAGS="-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual -Werror" \
  vrpn_router_core \
  vrpn_router_ros \
  vrpn_router_node \
  vrpn_router_core_tests \
  vrpn_router_test_vrpn_server_node

clang_tidy_sources=(
  "${workspace}/src/vrpn_router/src/vrpn_router_main.cpp"
  "${workspace}/src/vrpn_router/src/vrpn_router_node.cpp"
  "${workspace}/src/vrpn_router/test/core_tests.cpp"
  "${workspace}/src/vrpn_router/test/vrpn_test_server_node.cpp"
)
clang-tidy --quiet -p "${workspace}/build" "${clang_tidy_sources[@]}" 2>&1 |
  sed -E '/^[0-9]+ warnings generated\.$/d'

echo "C++ quality checks passed."
