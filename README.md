# XGC2 VRPN Router

`xgc2-vrpn-router` is a small protocol-level VRPN tracker relay.  It connects to
one upstream VRPN server as a client and exposes selected trackers through a
local VRPN server for downstream clients on another interface or subnet.

It does not depend on ROS and does not publish ROS topics.

## Data Flow

```text
upstream VRPN server
  -> vrpn_Tracker_Remote
xgc2-vrpn-router
  -> vrpn_Tracker_Server
downstream VRPN clients
```

The first version forwards tracker pose, velocity, and acceleration reports.
Tracker names are configured statically because VRPN does not provide a reliable
portable tracker enumeration API.

## Build

```bash
git clone --depth 1 --branch v07.36 https://github.com/vrpn/vrpn.git /tmp/vrpn
cmake -S /tmp/vrpn -B /tmp/vrpn-build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/tmp/vrpn-install \
  -DBUILD_SHARED_LIBS=OFF \
  -DVRPN_INSTALL=ON \
  -DVRPN_BUILD_CLIENTS=OFF \
  -DVRPN_BUILD_SERVERS=OFF \
  -DVRPN_BUILD_CLIENT_LIBRARY=OFF \
  -DVRPN_BUILD_SERVER_LIBRARY=ON
cmake --build /tmp/vrpn-build --target vrpnserver quat
cmake --install /tmp/vrpn-build

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DVRPN_ROOT=/tmp/vrpn-install
cmake --build build
```

The CI package build uses the same pinned upstream VRPN release and links it
statically, so the Debian package does not depend on an Ubuntu-provided
`libvrpn-dev` package.

## Run

```bash
xgc2-vrpn-router --config config/router.conf
```

Downstream clients connect to the local server using the configured downstream
tracker names, for example:

```text
uav1@192.168.51.14
ugv1@192.168.51.14
```

## Configuration

```ini
[General]
UpstreamHost = 192.168.10.20
UpstreamPort = 3883
BindAddress = 192.168.51.14
ListenPort = 3883
MainloopRate = 240
UpstreamUpdateRate = 120

[Tracker uav1]
Upstream = uav1
Downstream = uav1
Sensors = 1

[Tracker ugv1]
Upstream = ugv1
Downstream = ugv1
Sensors = 1
```

`BindAddress` can be empty to listen on all interfaces.  `MainloopRate` controls
how often the router services VRPN connections.  `UpstreamUpdateRate` requests a
tracker update rate from the upstream server when supported.

## End-to-End Test

CI runs an installed-package E2E test with official VRPN tools:

```bash
.xgc2/scripts/run_official_vrpn_e2e.sh \
  --router-binary /usr/bin/xgc2-vrpn-router
```

The test starts official `vrpn_server` with `vrpn_Tracker_NULL`, runs
`xgc2-vrpn-router`, and verifies the downstream endpoint with official
`vrpn_print_devices`.
