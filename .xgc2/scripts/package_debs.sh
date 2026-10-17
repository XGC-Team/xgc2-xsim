#!/usr/bin/env bash
set -euo pipefail
[[ $# == 4 && "$1" == --install-root && "$3" == --output-dir ]] || { echo 'usage: package_debs.sh --install-root ROOT --output-dir DIR' >&2; exit 2; }
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
stage="$(realpath "$2")";out="$(realpath -m "$4")";mkdir -p "$out"
export LD_LIBRARY_PATH="/opt/ros/noetic/lib:${LD_LIBRARY_PATH:-}"
shlibs="$(mktemp -d "$out/shlibs.XXXXXX")"
trap 'rm -rf -- "$shlibs"' EXIT
mkdir -p "$shlibs/debian"
printf 'Source: xsim\nSection: science\nPriority: optional\nMaintainer: XGC Team <dev@xgc.team>\n\nPackage: xsim\nArchitecture: any\n' > "$shlibs/debian/control"
deps="$(cd "$shlibs" && dpkg-shlibdeps -O -e"$stage/opt/xgc2/xsim/bin/xsim")";deps="${deps#shlibs:Depends=}"
gpu_deps="$(cd "$shlibs" && dpkg-shlibdeps -O -e"$stage/usr/lib/xgc2-sensing/libxgc2-sensing-gpu.so")";gpu_deps="${gpu_deps#shlibs:Depends=}"
version="$(awk -F': *' '/^version:/ {print $2;exit}' "$root/.xgc2/product.yml")"
python3 "$root/.xgc2/scripts/package_debs.py" --version "$version" --stage "$stage" --output-dir "$out" \
 --depends "$deps, ros-noetic-roscpp, ros-noetic-mavros-msgs, ros-noetic-rosgraph-msgs, ros-noetic-xgc2-geometry-msgs (>= 1.5.0-4)" \
 --gpu-depends "$gpu_deps, libglfw3 (>= 3.3)"
