#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

VRPN_VERSION="${VRPN_VERSION:-v07.36}"
VRPN_SOURCE=""
WORK_DIR="${WORK_DIR:-${REPO_ROOT}/.work/official-vrpn-e2e}"
ROUTER_BINARY="${ROUTER_BINARY:-${REPO_ROOT}/build/xgc2-vrpn-router}"
UPSTREAM_PORT="${UPSTREAM_PORT:-43883}"
ROUTER_PORT="${ROUTER_PORT:-43884}"
TRACKER_NAME="${TRACKER_NAME:-Tracker0}"
TIMEOUT_SECONDS="${TIMEOUT_SECONDS:-12}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --vrpn-source)
      VRPN_SOURCE="$2"
      shift 2
      ;;
    --vrpn-version)
      VRPN_VERSION="$2"
      shift 2
      ;;
    --work-dir)
      WORK_DIR="$2"
      shift 2
      ;;
    --router-binary)
      ROUTER_BINARY="$2"
      shift 2
      ;;
    --upstream-port)
      UPSTREAM_PORT="$2"
      shift 2
      ;;
    --router-port)
      ROUTER_PORT="$2"
      shift 2
      ;;
    --tracker-name)
      TRACKER_NAME="$2"
      shift 2
      ;;
    --timeout)
      TIMEOUT_SECONDS="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ ! -x "${ROUTER_BINARY}" ]]; then
  echo "router binary is not executable: ${ROUTER_BINARY}" >&2
  exit 1
fi

mkdir -p "${WORK_DIR}"
if [[ -z "${VRPN_SOURCE}" ]]; then
  VRPN_SOURCE="${WORK_DIR}/vrpn-src"
  if [[ ! -d "${VRPN_SOURCE}/.git" ]]; then
    rm -rf "${VRPN_SOURCE}"
    git clone --depth 1 --branch "${VRPN_VERSION}" https://github.com/vrpn/vrpn.git "${VRPN_SOURCE}"
  fi
fi

VRPN_BUILD="${WORK_DIR}/vrpn-build"
rm -rf "${VRPN_BUILD}"

cmake -S "${VRPN_SOURCE}" -B "${VRPN_BUILD}" \
  -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=OFF \
  -DVRPN_INSTALL=OFF \
  -DVRPN_BUILD_CLIENTS=ON \
  -DVRPN_BUILD_SERVERS=ON \
  -DVRPN_BUILD_CLIENT_LIBRARY=ON \
  -DVRPN_BUILD_SERVER_LIBRARY=ON \
  -DVRPN_BUILD_PYTHON=OFF \
  -DVRPN_BUILD_PYTHON_HANDCODED_2X=OFF \
  -DVRPN_BUILD_PYTHON_HANDCODED_3X=OFF \
  -DVRPN_BUILD_JAVA=OFF
cmake --build "${VRPN_BUILD}" --target vrpn_server vrpn_print_devices

VRPN_SERVER="${VRPN_BUILD}/server_src/vrpn_server"
VRPN_PRINT_DEVICES="${VRPN_BUILD}/client_src/vrpn_print_devices"
test -x "${VRPN_SERVER}"
test -x "${VRPN_PRINT_DEVICES}"

RUN_DIR="${WORK_DIR}/run"
rm -rf "${RUN_DIR}"
mkdir -p "${RUN_DIR}"

cat > "${RUN_DIR}/vrpn.cfg" <<EOF
vrpn_Tracker_NULL ${TRACKER_NAME} 2 30.0
EOF

cat > "${RUN_DIR}/router.conf" <<EOF
[General]
UpstreamHost = 127.0.0.1
UpstreamPort = ${UPSTREAM_PORT}
BindAddress = 127.0.0.1
ListenPort = ${ROUTER_PORT}
MainloopRate = 500
UpstreamUpdateRate = 30

[Tracker ${TRACKER_NAME}]
Upstream = ${TRACKER_NAME}
Downstream = ${TRACKER_NAME}
Sensors = 2
EOF

pids=()
cleanup() {
  local pid
  for pid in "${pids[@]:-}"; do
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill "${pid}" >/dev/null 2>&1 || true
    fi
  done
  wait "${pids[@]:-}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

"${VRPN_SERVER}" -quiet -f "${RUN_DIR}/vrpn.cfg" "${UPSTREAM_PORT}" \
  >"${RUN_DIR}/server.log" 2>&1 &
pids+=("$!")
sleep 1

"${ROUTER_BINARY}" --config "${RUN_DIR}/router.conf" \
  >"${RUN_DIR}/router.log" 2>&1 &
pids+=("$!")
sleep 1

client_status=0
timeout -s INT "${TIMEOUT_SECONDS}" stdbuf -oL -eL "${VRPN_PRINT_DEVICES}" \
  -nobutton -noanalog -nodial -notext -trackerstride 1 \
  "${TRACKER_NAME}@127.0.0.1:${ROUTER_PORT}" \
  >"${RUN_DIR}/client.log" 2>&1 || client_status=$?

if [[ "${client_status}" != "0" && "${client_status}" != "124" ]]; then
  echo "official vrpn_print_devices exited with status ${client_status}" >&2
  cat "${RUN_DIR}/client.log" >&2 || true
  exit 1
fi

if ! grep -Eq "Tracker ${TRACKER_NAME}@127\\.0\\.0\\.1:${ROUTER_PORT}, sensor [0-9]+:" "${RUN_DIR}/client.log"; then
  echo "official vrpn_print_devices did not receive a routed tracker report" >&2
  echo "---- server.log ----" >&2
  cat "${RUN_DIR}/server.log" >&2 || true
  echo "---- router.log ----" >&2
  cat "${RUN_DIR}/router.log" >&2 || true
  echo "---- client.log ----" >&2
  cat "${RUN_DIR}/client.log" >&2 || true
  exit 1
fi

echo "official VRPN E2E passed: ${TRACKER_NAME}@127.0.0.1:${UPSTREAM_PORT} -> xgc2-vrpn-router -> ${TRACKER_NAME}@127.0.0.1:${ROUTER_PORT}"
