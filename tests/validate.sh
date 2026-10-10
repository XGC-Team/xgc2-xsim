#!/usr/bin/env bash
# One private build/ROS job. No station master, runtime cache or host install.
set -euo pipefail
[[ $# == 1 ]] || { echo 'usage: tests/validate.sh NEW_PRIVATE_OUTPUT_DIRECTORY' >&2; exit 2; }
owner="$(cd -- "$(dirname -- "$0")/.." && pwd)"
products="${XSIM_PRODUCTS_ROOT:-$(cd -- "$owner/../../.." && pwd)}"
output="$(realpath -m -- "$1")"
[[ ! -e "$output" ]] || { echo 'output directory must be new' >&2; exit 2; }
mkdir -p "$output"
exec 9>/tmp/xsim-private-validation.lock
flock -n 9 || { echo 'another xsim heavy validation job owns the lock' >&2; exit 2; }
name="xsim-private-validation-$$"
image="${XSIM_TEST_IMAGE:-ghcr.io/xgc-team/xgc2-images/xgc2-build-focal-full-noetic:1.0.0}"
cleanup() { docker stop --time 10 "$name" >/dev/null 2>&1 || true; }
trap cleanup EXIT INT TERM
{
  git -C "$owner" rev-parse HEAD
  git -C "$products/ros1/common/scene" rev-parse HEAD
  docker image inspect "$image" --format '{{.Id}}'
} > "$output/provenance.txt"
affinity=()
if [[ -n "${XSIM_TEST_CPUSET:-}" ]]; then affinity=(--cpuset-cpus "$XSIM_TEST_CPUSET"); fi
docker run -d --rm --name "$name" --network none --cpus 2 --memory 5g --pids-limit 256 \
  "${affinity[@]}" --entrypoint sleep -v "$products:/source:ro" -v "$output:/work" "$image" infinity > "$output/container-id.txt"
docker exec -i "$name" bash -s > "$output/validation.log" 2>&1 <<'INNER'
set -euo pipefail
source /opt/ros/noetic/setup.bash
sim=/source/ros1/simulator/xsim
cmake -S /source/ros1/common/scene/sensors/world_lidar/library -B /work/lidar -DCMAKE_INSTALL_PREFIX=/work/install -DCMAKE_BUILD_TYPE=Release
cmake --build /work/lidar -j1
cmake --install /work/lidar
cmake -S "$sim/src/xsim" -B /work/xsim \
  -DCMAKE_PREFIX_PATH='/work/install;/opt/ros/noetic' -DCMAKE_INSTALL_PREFIX=/work/install \
  -DCMAKE_BUILD_TYPE=Release -DXGC2_MATH_INCLUDE=/source/common/math/include \
  -DFS150_ASSET_SOURCE_ROOT=/source/ros1/simulator/gazebo-sim/fs150-sitl \
  -DXSIM_ROS=ON -DXSIM_TESTS=ON -DPYTHON_EXECUTABLE=/usr/bin/python3
cmake --build /work/xsim -j1
cmake --install /work/xsim
cd /work/xsim
ctest --output-on-failure
cpack --config CPackConfig.cmake
ldd /work/install/bin/xsim > /work/xsim-linked-libraries.txt
/work/install/bin/xsim --help
INNER
