#!/usr/bin/env bash
set -euo pipefail
set +u
source /opt/ros/noetic/setup.bash
set -u
dpkg -s xsim xsim-lidar-gpu >/dev/null
depends="$(dpkg-query -W -f='${Depends}' xsim)"
grep -Eq '(^|, )libxgc2-xrpc2( |[(,]|$)' <<<"$depends"
grep -Eq '(^|, )libxgc2-chassis-hold1( |[(,]|$)' <<<"$depends"
test -s /usr/share/xgc2/process-definitions/xsim-world.json
dpkg-query -S /opt/xgc2/xsim/bin/xsim /opt/xgc2/xsim/share/xsim/fs150_native_flight_model.json /usr/share/xgc2/process-definitions/xsim-world.json
ldd /opt/xgc2/xsim/bin/xsim | awk '/not found/ {missing=1} END {exit missing ? 1 : 0}'
! ldd /opt/xgc2/xsim/bin/xsim | grep -E 'libglfw|libGL|libpcl|libgomp'
dpkg-query -S /opt/xgc2/xsim/lib/xsim/libxsim-lidar-gpu.so /opt/xgc2/xsim/share/xsim-lidar-gpu/LICENSE
ldd /opt/xgc2/xsim/lib/xsim/libxsim-lidar-gpu.so | awk '/not found/ {missing=1} END {exit missing ? 1 : 0}'
python3 /opt/xgc2/xsim/share/xsim-lidar-gpu/source/.xgc2/scripts/check_gpu_module_sources.py
/opt/xgc2/xsim/bin/xsim --help
! dpkg -L xsim | grep -E 'liblightweight_vehicle|libros_io|xgc-rt-host|simulation_records_v1|/opt/ros/'
