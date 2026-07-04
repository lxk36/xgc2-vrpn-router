#!/usr/bin/env bash
set -euo pipefail

dpkg -s xgc2-vrpn-router >/dev/null
command -v xgc2-vrpn-router >/dev/null
xgc2-vrpn-router --version
xgc2-vrpn-router --check-config --config /etc/xgc2/vrpn-router/router.conf
test -f /etc/xgc2/vrpn-router/router.conf
test -f /lib/systemd/system/xgc2-vrpn-router.service -o -f /usr/lib/systemd/system/xgc2-vrpn-router.service
