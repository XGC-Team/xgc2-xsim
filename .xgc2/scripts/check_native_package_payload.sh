#!/usr/bin/env bash
set -euo pipefail
[[ $# == 2 ]] || exit 2
install_root="$1";deb_dir="$2";work="$(mktemp -d)"
trap 'rm -rf -- "$work"' EXIT
shopt -s nullglob
debs=("$deb_dir"/xsim_*.deb);[[ ${#debs[@]} == 1 ]]
gpu_debs=("$deb_dir"/xsim-lidar-gpu_*.deb);[[ ${#gpu_debs[@]} == 1 ]]
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
version="$(awk -F': *' '/^version:/ {print $2;exit}' "$root/.xgc2/product.yml")"
for deb in "${debs[0]}" "${gpu_debs[0]}"; do
  [[ "$(dpkg-deb -f "$deb" Version)" == "$version" ]]
done
dpkg-deb --extract "${debs[0]}" "$work/payload"
dpkg-deb --extract "${gpu_debs[0]}" "$work/gpu"
for path in /opt/xgc2/xsim/bin/xsim /opt/xgc2/xsim/share/xsim/fs150_native_flight_model.json /usr/share/xgc2/process-definitions/xsim-world.json;do cmp "$install_root$path" "$work/payload$path";done
! dpkg-deb -c "${debs[0]}" | grep -E 'liblightweight_vehicle|libros_io|xgc-rt-host|simulation_records_v1|/opt/ros/'
! dpkg-deb -c "${debs[0]}" | grep -E 'libxsim-lidar-gpu|share/xsim-lidar-gpu'
for path in /opt/xgc2/xsim/lib/xsim/libxsim-lidar-gpu.so /opt/xgc2/xsim/share/xsim-lidar-gpu/LICENSE;do cmp "$install_root$path" "$work/gpu$path";done
python3 "$work/gpu/opt/xgc2/xsim/share/xsim-lidar-gpu/source/.xgc2/scripts/check_gpu_module_sources.py"
echo 'PASS: standalone xsim installed payload and native management interface'
