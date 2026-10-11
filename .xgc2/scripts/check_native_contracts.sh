#!/usr/bin/env bash
set -euo pipefail
[[ $# == 1 ]] || { echo 'usage: check_native_contracts.sh FS150_ASSET_SOURCE' >&2; exit 2; }
sim_source="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
fs150_source="$(realpath "$1")"
[[ -f "$fs150_source/scripts/generate_native_flight_model.py" && -f "${XGC2_MATH_INCLUDE:-/usr/include}/xgc2_math/scene.hpp" ]] || { echo 'FS150 assets and installed Math 0.6.0 scene headers are required' >&2; exit 2; }
native_build="$(mktemp -d /tmp/xgc2-xsim-native.XXXXXX)"
trap 'rm -rf -- "$native_build"' EXIT
native_prefix="${CMAKE_PREFIX_PATH:-}"

cmake -S "$sim_source/src/xsim" -B "$native_build/server" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="${native_prefix//:/;}" \
  -DXGC2_MATH_INCLUDE="${XGC2_MATH_INCLUDE:-/usr/include}" \
  -DFS150_ASSET_SOURCE_ROOT="$fs150_source" \
  -DXSIM_ROS=OFF -DXSIM_TESTS=ON
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
