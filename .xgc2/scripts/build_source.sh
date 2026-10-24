#!/usr/bin/env bash
# Source build/install only. Caller supplies toolchain and dependency prefixes.
# No apt, Docker, publication or robot startup.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
work="${1:?usage: build_source.sh BUILD_DIR INSTALL_ROOT FS150_ASSET_SOURCE}"
stage="${2:?install root (DESTDIR)}"
assets="${3:?FS150 asset source}"
jobs="${BUILD_JOBS:-2}"
[[ "$jobs" =~ ^[1-9][0-9]*$ ]] || exit 2
prefix_list="${CMAKE_PREFIX_PATH//:/;}"
cmake -S "$root/src/sensing-gpu" -B "$work/gpu" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr \
  -DBUILD_TESTING="${XGC2_SOURCE_TESTS:-OFF}" \
  -DCMAKE_PREFIX_PATH="$prefix_list" -DXGC2_MATH_INCLUDE="${XGC2_MATH_INCLUDE:-/usr/local/include}"
cmake --build "$work/gpu" --parallel "$jobs"
DESTDIR="$stage" cmake --install "$work/gpu"
cmake -S "$root/src/xsim" -B "$work/server" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/xgc2/xsim \
  -DCMAKE_PREFIX_PATH="$prefix_list" -DXSIM_ROS=ON -DXSIM_TESTS="${XGC2_SOURCE_TESTS:-OFF}" \
  -DXGC2_MATH_INCLUDE="${XGC2_MATH_INCLUDE:-/usr/local/include}" \
  -DFS150_ASSET_SOURCE_ROOT="$assets" \
  ${XGC2_XSIM_PYTHON:+-DPython3_EXECUTABLE="$XGC2_XSIM_PYTHON"}
cmake --build "$work/server" --parallel "$jobs"
DESTDIR="$stage" cmake --install "$work/server"
if [[ "${XGC2_SOURCE_TESTS:-OFF}" == ON ]]; then
  (cd "$work/gpu" && ctest --output-on-failure)
  (cd "$work/server" && ctest --output-on-failure)
fi
