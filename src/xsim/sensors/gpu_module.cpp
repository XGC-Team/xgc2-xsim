#include "gpu_module.hpp"
#include <xgc2_sensing/loader.hpp>
namespace xsim {
const xgc2_sensing_gpu_api& load_gpu_module() { return xgc2_sensing::gpuLibrary(); }
}
