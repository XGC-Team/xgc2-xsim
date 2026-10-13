#!/usr/bin/env bash
set -euo pipefail
[[ $# == 2 ]] || { echo 'usage: build_package.sh FS150_ASSET_SOURCE OUTPUT' >&2; exit 2; }
sim_source="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
fs150_source="$(realpath "$1")"
package_output="$(realpath -m "$2")"
[[ -f "$sim_source/src/xsim/main.cpp" && -f "$fs150_source/scripts/generate_native_flight_model.py" && -f "${XGC2_MATH_INCLUDE:-/usr/include}/xgc2_math/scene.hpp" ]] || { echo 'FS150 assets and installed Math 0.6.0 scene headers are required' >&2; exit 2; }
build_cmake_prefix="${CMAKE_PREFIX_PATH:-}"
set +u
source /opt/ros/noetic/setup.bash
set -u
export CMAKE_PREFIX_PATH="$build_cmake_prefix:${CMAKE_PREFIX_PATH:-}"
cmake_prefix_list="${CMAKE_PREFIX_PATH//:/;}"
mkdir -p "$package_output"
build_output="$(mktemp -d "$package_output/build.XXXXXX")"
trap 'rm -rf -- "$build_output"' EXIT
python3 "$sim_source/.xgc2/scripts/check_gpu_module_sources.py"
cmake -S "$sim_source/src/sensing-gpu" -B "$build_output/gpu" \
 -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr \
 -DCMAKE_PREFIX_PATH="$cmake_prefix_list" -DXGC2_MATH_INCLUDE="${XGC2_MATH_INCLUDE:-/usr/include}"
cmake --build "$build_output/gpu" --parallel 2
(cd "$build_output/gpu" && ctest --output-on-failure)
DESTDIR="$build_output/stage" cmake --install "$build_output/gpu"
cmake -S "$sim_source/src/xsim" -B "$build_output/server" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/xgc2/xsim \
 -DCMAKE_PREFIX_PATH="$cmake_prefix_list" \
 -DXGC2_MATH_INCLUDE="${XGC2_MATH_INCLUDE:-/usr/include}" -DFS150_ASSET_SOURCE_ROOT="$fs150_source" -DXSIM_TESTS=ON
cmake --build "$build_output/server" --parallel 2
(cd "$build_output/server" && ctest --output-on-failure)
DESTDIR="$build_output/stage" cmake --install "$build_output/server"
"$sim_source/.xgc2/scripts/package_debs.sh" --install-root "$build_output/stage" --output-dir "$package_output"
cp -a "$build_output/stage/opt" "$build_output/stage/usr" "$package_output/"
sha256sum "$package_output"/*.deb > "$package_output/SHA256SUMS"
