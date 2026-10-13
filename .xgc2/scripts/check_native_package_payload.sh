#!/usr/bin/env bash
set -euo pipefail
[[ $# == 2 ]] || exit 2
install_root="$1";deb_dir="$2";work="$(mktemp -d)"
trap 'rm -rf -- "$work"' EXIT
shopt -s nullglob
debs=("$deb_dir"/xsim_*.deb);[[ ${#debs[@]} == 1 ]]
gpu_debs=("$deb_dir"/libxgc2-sensing-gpu_*.deb);[[ ${#gpu_debs[@]} == 1 ]]
dev_debs=("$deb_dir"/libxgc2-sensing-dev_*.deb);[[ ${#dev_debs[@]} == 1 ]]
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
version="$(awk -F': *' '/^version:/ {print $2;exit}' "$root/.xgc2/product.yml")"
for deb in "${debs[0]}" "${gpu_debs[0]}" "${dev_debs[0]}"; do
  [[ "$(dpkg-deb -f "$deb" Version)" == "$version" ]]
done
dpkg-deb --extract "${debs[0]}" "$work/payload"
dpkg-deb --extract "${gpu_debs[0]}" "$work/gpu"
dpkg-deb --extract "${dev_debs[0]}" "$work/dev"
for header in gpu.h loader.hpp;do cmp "$install_root/usr/include/xgc2_sensing/$header" "$work/dev/usr/include/xgc2_sensing/$header";done
for path in /opt/xgc2/xsim/bin/xsim /opt/xgc2/xsim/share/xsim/fs150_native_flight_model.json /usr/share/xgc2/process-definitions/xsim-world.json;do cmp "$install_root$path" "$work/payload$path";done
! dpkg-deb -c "${debs[0]}" | grep -E 'liblightweight_vehicle|libros_io|xgc-rt-host|simulation_records_v1|/opt/ros/'
! dpkg-deb -c "${debs[0]}" | grep -E 'libxgc2-sensing-gpu|share/xgc2-sensing'
for path in /usr/lib/xgc2-sensing/libxgc2-sensing-gpu.so /usr/share/xgc2-sensing/LICENSE /usr/libexec/xgc2-sensing-probe;do cmp "$install_root$path" "$work/gpu$path";done
python3 "$work/gpu/usr/share/xgc2-sensing/source/.xgc2/scripts/check_gpu_module_sources.py"
echo 'PASS: standalone xsim installed payload and native management interface'
