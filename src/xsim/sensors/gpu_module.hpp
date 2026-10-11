#pragma once
#include "lidar_gpu_module.h"

namespace xsim {
// Loaded on first GPU preparation and retained until process exit.
const xsim_lidar_gpu_api &load_gpu_module();
}
