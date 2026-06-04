# XGC2 VRPN Router

`vrpn_router` routes VRPN rigid-body pose streams into MAVROS vision pose inputs for PX4 EKF2 fusion. It is used when a motion-capture source, a Gazebo VRPN server, or another VRPN-compatible tracker source needs to drive one or more MAVROS namespaces with controlled rate, frame compensation, and health diagnostics.

The router does not implement the VRPN protocol client itself. It depends on `vrpn_client_ros` for tracker subscription and focuses on routing, pose correction, downsampling, and health reporting.

## Package

- Product id: `xgc2-vrpn-router`
- Source path: `products/ros1/perception/vrpn-router`
- Release branch: `mavros-noetic`
- Package type: `ros1-apt`
- ROS package: `vrpn_router`
- Published package:
  - `ros-noetic-xgc2-vrpn-router`
- Main runtime launch:
  - `roslaunch vrpn_router vrpn_router.launch`

## Install

```bash
sudo apt update
sudo apt install ros-noetic-xgc2-vrpn-router
```

The package installs runtime files under:

```text
/opt/ros/noetic/share/vrpn_router
/opt/ros/noetic/include/vrpn_router
/opt/ros/noetic/lib/vrpn_router
```

## Smoke Test

```bash
roslaunch --files vrpn_router vrpn_router.launch
```

This only checks that the installed launch file is discoverable. A real runtime test also needs a VRPN server or a Gazebo VRPN server.

## Runtime Model

The default launch file can start `vrpn_client_ros` and then start `vrpn_router`:

```bash
roslaunch vrpn_router vrpn_router.launch \
  server:=127.0.0.1 \
  port:=3883 \
  config:=$(rospack find vrpn_router)/config/routes.example.yaml
```

For an externally managed `vrpn_client_ros`, disable the built-in client:

```bash
roslaunch vrpn_router vrpn_router.launch \
  start_vrpn_client:=false \
  config:=/path/to/routes.yaml
```

For a downstream quickstart that injects route parameters itself, disable the
default YAML load:

```bash
roslaunch vrpn_router vrpn_router.launch \
  load_config:=false \
  start_vrpn_client:=true
```

To preserve VRPN server timestamps from the client output and restrict the
client to one tracker, inject the client parameters explicitly:

```bash
roslaunch vrpn_router vrpn_router.launch \
  server:=127.0.0.1 \
  port:=3883 \
  config:=/path/to/routes.yaml \
  use_server_time:=true \
  refresh_tracker_frequency:=0.0 \
  trackers:=uav1
```

The built-in `vrpn_client_ros` launch path does not publish TF by default.
The router path is responsible for forwarding checked pose messages into
MAVROS, not for owning the global TF tree.  If a downstream visualization-only
workflow explicitly needs tracker TF frames, opt in with `broadcast_tf:=true`.

Input topics normally follow the `vrpn_client_ros` convention:

```text
/vrpn_client_node/<tracker>/pose
```

Output topics normally follow the MAVROS vision pose convention:

```text
/<mavros_ns>/vision_pose/pose
```

Example:

```text
/vrpn_client_node/uav1/pose -> /uav1/mavros/vision_pose/pose
```

## Route Configuration

Routes are configured through YAML loaded under the private `vrpn_router` namespace. The installed example is:

```bash
roscd vrpn_router
cat config/routes.example.yaml
```

Minimal route:

```yaml
vrpn_router:
  output_frame_id: map
  max_output_rate_hz: 50.0
  routes:
    - tracker: uav1
      mavros_ns: /uav1/mavros
```

Explicit route:

```yaml
vrpn_router:
  routes:
    - name: uav1
      tracker: uav1
      input_topic: /vrpn_client_node/uav1/pose
      output_topic: /uav1/mavros/vision_pose/pose
      reference_topic: /uav1/mavros/local_position/pose
      output_frame_id: map
      max_output_rate_hz: 50.0
```

Each route can override global defaults, including output rate, health thresholds, field offset, and tracker-to-body transform.

## Pose Pipeline

For each input pose, the router applies:

1. tracker-to-body SE3 compensation;
2. field XYZ offset;
3. output frame id override;
4. approximate rate limiting;
5. publication to the configured MAVROS vision pose topic.

The rate limiter is intentionally simple: it drops frames to keep the approximate output period near the configured target. It does not interpolate, resample, or smooth pose data.

## Health Diagnostics

The router publishes `diagnostic_msgs/DiagnosticArray` on `/diagnostics`.

Current checks:

- `vrpn_timeout`: no recent input frame.
- `input_rate_low`: input rate below configured minimum.
- `input_rate_high`: input rate above configured maximum.
- `vrpn_jump`: adjacent input frames have either translation jump or rotation jump beyond threshold.
- `vrpn_stuck`: input pose remains within a configured tolerance over a time window.
- `reference_delta_high`: routed output deviates too far from an optional MAVROS reference pose topic.

Important semantics:

- Translation jump and rotation jump are an OR relation. Either one is enough to report `vrpn_jump`.
- Jump detection compares adjacent input frames, not input against last published output.
- Stuck detection uses a time window. Thresholds should be configured with expected mocap noise in mind.
- Health checks do not block forwarding. They report status for downstream safety and fault-handling modules.

## Key Parameters

Global defaults:

```yaml
vrpn_router:
  max_output_rate_hz: 50.0
  min_input_rate_hz: 30.0
  max_input_rate_hz: 150.0
  stale_timeout_s: 0.3
  stuck_timeout_s: 0.5
  stuck_position_epsilon_m: 0.001
  stuck_angle_epsilon_deg: 0.2
  max_translation_jump_m: 1.0
  max_rotation_jump_deg: 45.0
  max_reference_delta_m: 2.0
```

Tracker-to-body and field compensation:

```yaml
vrpn_router:
  field_offset:
    x: 0.0
    y: 0.0
    z: 0.0
  tracker_to_body:
    translation: {x: 0.0, y: 0.0, z: 0.0}
    rotation: {roll: 0.0, pitch: 0.0, yaw: 0.0}
```

## What This Product Owns

- `vrpn_router_node`
- route parsing and topic mapping;
- VRPN pose to MAVROS vision pose forwarding;
- output rate limiting;
- SE3 tracker-to-body compensation;
- field XYZ offset;
- diagnostics for timeout, stuck, jump, rate, and reference deviation;
- installed headers for downstream tests or reuse of core logic.

## What This Product Does Not Own

- VRPN server implementation;
- `vrpn_client_ros` implementation;
- MAVROS or PX4 EKF2 configuration;
- Gazebo model-to-VRPN tracker export;
- vehicle-specific safety policy.

## Dependencies

Runtime dependencies:

- `ros-noetic-vrpn-client-ros`
- `ros-noetic-mavros`
- `ros-noetic-diagnostic-msgs`
- `ros-noetic-geometry-msgs`
- `ros-noetic-roscpp`

Test dependencies include `rostest`, `rosunit`, `rospy`, and VRPN test utilities.

## Source Layout

```text
include/vrpn_router/core/       ROS-independent core detectors and transforms
include/vrpn_router/            ROS route, diagnostics, and adapter code
src/vrpn_router_node.cpp        Runtime node
config/routes.example.yaml      Example route configuration
launch/vrpn_router.launch       Main launch entrypoint
scripts/vrpn_router_extreme_sim.py
test/                           Unit, integration, and VRPN protocol tests
.xgc2/product.yml               Release metadata
.xgc2/scripts/                  Packaging and apt publish helpers
```

## Build And Test

```bash
mkdir -p /tmp/xgc2-vrpn-router-ws/src
rsync -a --delete ./ /tmp/xgc2-vrpn-router-ws/src/vrpn_router/
source /opt/ros/noetic/setup.bash
cd /tmp/xgc2-vrpn-router-ws
catkin_make run_tests_vrpn_router
catkin_test_results
```

Build the Debian package locally:

```bash
mkdir -p /tmp/xgc2-vrpn-router-release-ws/src /tmp/xgc2-vrpn-router-release-debs
rsync -a --delete ./ /tmp/xgc2-vrpn-router-release-ws/src/vrpn_router/
source /opt/ros/noetic/setup.bash
cd /tmp/xgc2-vrpn-router-release-ws
DESTDIR=/tmp/xgc2-vrpn-router-release-ws/install-root \
  catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic -DCATKIN_ENABLE_TESTING=OFF
/tmp/xgc2-vrpn-router-release-ws/src/vrpn_router/.xgc2/scripts/package_debs.sh \
  --install-root /tmp/xgc2-vrpn-router-release-ws/install-root \
  --output-dir /tmp/xgc2-vrpn-router-release-debs
```

## Release Notes

- ROS distro: Noetic
- Apt distribution: focal
- Architectures: amd64, arm64
- Apt repository: `https://xgc2.apt.xiaokang.ink`
- CI workflow: `.github/workflows/build-debs.yml`
