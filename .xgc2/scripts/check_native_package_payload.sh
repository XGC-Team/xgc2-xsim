#!/usr/bin/env bash
set -euo pipefail
[[ $# == 2 ]] || exit 2
install_root="$1";deb_dir="$2";work="$(mktemp -d)"
trap 'rm -rf -- "$work"' EXIT
shopt -s nullglob
debs=("$deb_dir"/xsim_*.deb);[[ ${#debs[@]} == 1 ]]
dpkg-deb --extract "${debs[0]}" "$work/payload"
for path in /opt/xgc2/xsim/bin/xsim /opt/xgc2/xsim/share/xsim/fs150_native_flight_model.json /usr/share/xgc2/process-definitions/xsim-world.json;do cmp "$install_root$path" "$work/payload$path";done
! dpkg-deb -c "${debs[0]}" | grep -E 'liblightweight_vehicle|libros_io|xgc-rt-host|simulation_records_v1|/opt/ros/'
echo 'PASS: standalone xsim installed payload and native management interface'
