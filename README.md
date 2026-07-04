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
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

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
