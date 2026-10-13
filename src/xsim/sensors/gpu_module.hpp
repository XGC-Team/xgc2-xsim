#pragma once
#include <xgc2_sensing/gpu.h>

namespace xsim {
// Loaded on first GPU preparation and retained until process exit.
const xgc2_sensing_gpu_api &load_gpu_module();
}
