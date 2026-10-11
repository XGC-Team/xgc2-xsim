#!/usr/bin/env bash
set -euo pipefail
[[ $# == 4 && "$1" == --install-root && "$3" == --output-dir ]] || { echo 'usage: package_debs.sh --install-root ROOT --output-dir DIR' >&2; exit 2; }
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
stage="$2";out="$4";mkdir -p "$out" "$stage/debian"
export LD_LIBRARY_PATH="/opt/ros/noetic/lib:${LD_LIBRARY_PATH:-}"
printf 'Source: xsim\nSection: science\nPriority: optional\nMaintainer: XGC Team <dev@xgc.team>\n\nPackage: xsim\nArchitecture: any\n' > "$stage/debian/control"
deps="$(cd "$stage" && dpkg-shlibdeps -O -e"$stage/opt/xgc2/xsim/bin/xsim")";deps="${deps#shlibs:Depends=}"
rm -rf "$stage/debian"
python3 "$root/.xgc2/scripts/package_debs.py" --stage "$stage" --output "$out/xsim_1.2.0-1_$(dpkg --print-architecture).deb" --depends "$deps, libglfw3, ros-noetic-roscpp, ros-noetic-mavros-msgs, ros-noetic-rosgraph-msgs"
