#!/usr/bin/env bash
set -euo pipefail
[[ $# == 3 ]] || { echo 'usage: build_package.sh SCENE_SOURCE FS150_ASSET_SOURCE OUTPUT' >&2; exit 2; }
sim_source="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
scene_source="$(realpath "$1")"
fs150_source="$(realpath "$2")"
package_output="$(realpath -m "$3")"
[[ -f "$sim_source/src/xsim/main.cpp" && -f "$scene_source/sensors/world_lidar/library/CMakeLists.txt" && -f "$fs150_source/scripts/generate_native_flight_model.py" ]] || { echo 'the three owning sources are required' >&2; exit 2; }
build_cmake_prefix="${CMAKE_PREFIX_PATH:-}"
set +u
source /opt/ros/noetic/setup.bash
set -u
export CMAKE_PREFIX_PATH="$build_cmake_prefix:${CMAKE_PREFIX_PATH:-}"
cmake_prefix_list="${CMAKE_PREFIX_PATH//:/;}"
build_output="$(mktemp -d /tmp/xgc2-xsim-build.XXXXXX)"
trap 'rm -rf -- "$build_output"' EXIT
mkdir -p "$package_output" "$build_output/stage/DEBIAN"
cmake -S "$scene_source/sensors/world_lidar/library" -B "$build_output/geometry" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/xgc2/xsim -DXGC_WORLD_LIDAR_GPU=ON -DXGC_WORLD_LIDAR_METADATA_TEST=ON
cmake --build "$build_output/geometry" --parallel 2
(cd "$build_output/geometry" && ctest --output-on-failure)
DESTDIR="$build_output/stage" cmake --install "$build_output/geometry"
cmake -S "$sim_source/src/xsim" -B "$build_output/server" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/xgc2/xsim \
 -DCMAKE_PREFIX_PATH="$build_output/stage/opt/xgc2/xsim;${cmake_prefix_list}" \
 -DXGC2_MATH_INCLUDE="${XGC2_MATH_INCLUDE:-/usr/include}" -DFS150_ASSET_SOURCE_ROOT="$fs150_source" -DXSIM_GPU=ON -DXSIM_TESTS=ON
cmake --build "$build_output/server" --parallel 2
(cd "$build_output/server" && ctest --output-on-failure)
DESTDIR="$build_output/stage" cmake --install "$build_output/server"
# Public sensor library exports are a build dependency. Only shaders/licenses
# are runtime assets; there is no copied engine or DTO compatibility package.
rm -rf "$build_output/stage/opt/xgc2/xsim/include" "$build_output/stage/opt/xgc2/xsim/lib"
find "$build_output/stage" -name __pycache__ -type d -exec rm -rf {} +
mkdir -p "$build_output/shlibs/debian"
printf 'Source: xsim\nSection: science\nPriority: optional\nMaintainer: XGC Team <dev@xgc.team>\n\nPackage: xsim\nArchitecture: any\nDescription: Standalone ECS simulation server\n' > "$build_output/shlibs/debian/control"
# dpkg-shlibdeps uses the actual linked ELF. Explicit libglfw3 is included for
# builds using a caller-supplied CMake prefix instead of an installed dev package.
dependencies="$(cd "$build_output/shlibs"; dpkg-shlibdeps -O -e"$build_output/stage/opt/xgc2/xsim/bin/xsim" | sed 's/^shlibs:Depends=//')"
python3 "$(dirname "${BASH_SOURCE[0]}")/package_debs.py" --stage "$build_output/stage" \
 --depends "$dependencies, libglfw3 (>= 3.3), ros-noetic-roscpp, ros-noetic-mavros-msgs, ros-noetic-rosgraph-msgs" \
 --output "$package_output/xsim_1.2.0-1_$(dpkg --print-architecture).deb"
cp -a "$build_output/stage/opt" "$build_output/stage/usr" "$package_output/"
sha256sum "$package_output"/*.deb > "$package_output/SHA256SUMS"
