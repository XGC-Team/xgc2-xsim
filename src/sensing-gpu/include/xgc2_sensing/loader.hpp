#pragma once
#include "gpu.h"
#include <cstdlib>
#include <dlfcn.h>
#include <stdexcept>
#include <string>

namespace xgc2_sensing {
inline const xgc2_sensing_gpu_api& gpuLibrary() {
  static const auto* api = [] {
    const char* configured = std::getenv("XGC2_SENSING_GPU_LIBRARY");
    const std::string path = configured ? configured : "/usr/lib/xgc2-sensing/libxgc2-sensing-gpu.so";
    void* handle = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!handle) throw std::runtime_error("GPU sensing library unavailable: " + path + ": " + dlerror());
    auto entry = reinterpret_cast<decltype(&xgc2_sensing_gpu_entry)>(dlsym(handle,"xgc2_sensing_gpu_entry"));
    const auto* table = entry ? entry(XGC2_SENSING_GPU_ABI) : nullptr;
    if (!table || table->abi != XGC2_SENSING_GPU_ABI || table->size != sizeof(xgc2_sensing_gpu_api)
        || !table->create || !table->scan || !table->destroy)
      throw std::runtime_error("GPU sensing library ABI mismatch: " + path);
    return table;
  }();
  return *api;
}
}
