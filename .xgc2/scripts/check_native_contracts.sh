#!/usr/bin/env bash
set -euo pipefail
[[ $# == 2 ]] || { echo 'usage: check_native_contracts.sh SCENE_SOURCE FS150_ASSET_SOURCE' >&2; exit 2; }
sim_source="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
scene_source="$(realpath "$1")"
fs150_source="$(realpath "$2")"
[[ -f "$scene_source/sensors/world_lidar/library/CMakeLists.txt" && -f "$fs150_source/scripts/generate_native_flight_model.py" ]] || { echo 'the owning scene and FS150 sources are required' >&2; exit 2; }
native_build="$(mktemp -d /tmp/xgc2-xsim-native.XXXXXX)"
trap 'rm -rf -- "$native_build"' EXIT
native_prefix="${CMAKE_PREFIX_PATH:-}"

# A private CPU-only dependency prefix keeps this check separate from the ROS/GPU package.
cmake -S "$scene_source/sensors/world_lidar/library" -B "$native_build/geometry" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$native_build/install" \
  -DXGC_WORLD_LIDAR_GPU=OFF
cmake --build "$native_build/geometry" --target world_lidar --parallel 2
cmake --install "$native_build/geometry"
cmake -S "$sim_source/src/xsim" -B "$native_build/server" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$native_build/install;${native_prefix//:/;}" \
  -DXGC2_MATH_INCLUDE="${XGC2_MATH_INCLUDE:-/usr/include}" \
  -DFS150_ASSET_SOURCE_ROOT="$fs150_source" \
  -DXSIM_ROS=OFF -DXSIM_GPU=OFF -DXSIM_TESTS=ON
# These Python integration checks only need xsim; the package build runs the C++ suite.
cmake --build "$native_build/server" --target xsim --parallel 2
(
  cd "$native_build/server"
  contracts='^xsim_(cli_config|simulation_v1|chassis_hold_native)$'
  # CMake 3.16 has JSON test discovery but no --no-tests=error option.
  ctest --show-only=json-v1 -R "$contracts" > native-contracts.json
  python3 - native-contracts.json <<'PY'
import json
import sys

with open(sys.argv[1]) as source:
    registered = {test["name"] for test in json.load(source)["tests"]}
expected = {"xsim_cli_config", "xsim_simulation_v1", "xsim_chassis_hold_native"}
if registered != expected:
    raise SystemExit(f"native CTest registration: expected {sorted(expected)}, got {sorted(registered)}")
PY
  ctest --output-on-failure -R "$contracts"
)
