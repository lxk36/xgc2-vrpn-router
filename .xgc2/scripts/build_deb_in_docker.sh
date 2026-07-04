#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

UBUNTU_VERSION="${UBUNTU_VERSION:-20.04}"
DOCKER_IMAGE="${DOCKER_IMAGE:-ubuntu:${UBUNTU_VERSION}}"
WORK_DIR="${WORK_DIR:-${REPO_ROOT}/.work/docker-${UBUNTU_VERSION}}"
OUTPUT_DIR="${OUTPUT_DIR:-${REPO_ROOT}/debs}"
INSTALL_CHECK="${INSTALL_CHECK:-true}"
VRPN_VERSION="${VRPN_VERSION:-v07.36}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ubuntu-version)
      UBUNTU_VERSION="$2"
      DOCKER_IMAGE="ubuntu:${UBUNTU_VERSION}"
      shift 2
      ;;
    --image)
      DOCKER_IMAGE="$2"
      shift 2
      ;;
    --work-dir)
      WORK_DIR="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --skip-install-check)
      INSTALL_CHECK=false
      shift
      ;;
    --vrpn-version)
      VRPN_VERSION="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

case "${UBUNTU_VERSION}" in
  20.04)
    APT_REPO_DISTRIBUTION="focal"
    ;;
  22.04)
    APT_REPO_DISTRIBUTION="jammy"
    ;;
  24.04)
    APT_REPO_DISTRIBUTION="noble"
    ;;
  *)
    echo "unsupported Ubuntu version: ${UBUNTU_VERSION}" >&2
    exit 1
    ;;
esac

mkdir -p "${WORK_DIR}" "${OUTPUT_DIR}"

docker pull "${DOCKER_IMAGE}"
docker run --rm \
  -e DEBIAN_FRONTEND=noninteractive \
  -e INSTALL_CHECK="${INSTALL_CHECK}" \
  -e APT_REPO_DISTRIBUTION="${APT_REPO_DISTRIBUTION}" \
  -e VRPN_VERSION="${VRPN_VERSION}" \
  -v "${REPO_ROOT}:/workspace/vrpn-router:ro" \
  -v "${WORK_DIR}:/workspace/work" \
  -v "${OUTPUT_DIR}:/workspace/out" \
  "${DOCKER_IMAGE}" \
  bash -lc '
    set -euo pipefail

    export DEBIAN_FRONTEND=noninteractive
    apt-get update
    apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      cmake \
      dpkg-dev \
      fakeroot \
      file \
      git \
      ninja-build \
      pkg-config \
      rsync \
      systemd

    rm -rf \
      /workspace/work/src \
      /workspace/work/build \
      /workspace/work/install-root \
      /workspace/work/vrpn-src \
      /workspace/work/vrpn-build \
      /workspace/work/vrpn-install

    git clone --depth 1 --branch "${VRPN_VERSION}" https://github.com/vrpn/vrpn.git /workspace/work/vrpn-src
    cmake -S /workspace/work/vrpn-src -B /workspace/work/vrpn-build \
      -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/workspace/work/vrpn-install \
      -DBUILD_SHARED_LIBS=OFF \
      -DBUILD_TESTING=OFF \
      -DVRPN_INSTALL=ON \
      -DVRPN_BUILD_CLIENTS=OFF \
      -DVRPN_BUILD_SERVERS=OFF \
      -DVRPN_BUILD_CLIENT_LIBRARY=OFF \
      -DVRPN_BUILD_SERVER_LIBRARY=ON \
      -DVRPN_BUILD_PYTHON=OFF \
      -DVRPN_BUILD_PYTHON_HANDCODED_2X=OFF \
      -DVRPN_BUILD_PYTHON_HANDCODED_3X=OFF \
      -DVRPN_BUILD_JAVA=OFF
    cmake --build /workspace/work/vrpn-build --target vrpnserver quat
    cmake --install /workspace/work/vrpn-build

    mkdir -p /workspace/work/src
    rsync -a --delete /workspace/vrpn-router/ /workspace/work/src/

    cd /workspace/work/src
    cmake -S . -B /workspace/work/build \
      -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr \
      -DVRPN_ROOT=/workspace/work/vrpn-install
    cmake --build /workspace/work/build
    DESTDIR=/workspace/work/install-root cmake --install /workspace/work/build

    /workspace/vrpn-router/.xgc2/scripts/package_deb.sh \
      --install-root /workspace/work/install-root \
      --output-dir /workspace/out \
      --distro "${APT_REPO_DISTRIBUTION}"

    if [[ "${INSTALL_CHECK}" == "true" ]]; then
      apt-get install -y /workspace/out/xgc2-vrpn-router_*.deb
      /workspace/vrpn-router/.xgc2/scripts/check_installed_package.sh
    fi
  '

echo "Debian package output:"
find "${OUTPUT_DIR}" -maxdepth 1 -type f -name "*.deb" -print | sort
