#include "gpu_module.hpp"
#include <cstdlib>
#include <dlfcn.h>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <vector>

namespace xsim {
const xsim_lidar_gpu_api &load_gpu_module() {
  static const auto *api = [] {
    std::string path;
    if (const char *configured = std::getenv("XSIM_LIDAR_GPU_MODULE")) {
      path = configured;
    } else {
      std::vector<char> executable(256);
      ssize_t length;
      for (;;) {
        length = readlink("/proc/self/exe", executable.data(), executable.size());
        if (length < 0)
          throw std::runtime_error("GPU module executable path unavailable; no CPU fallback");
        if (size_t(length) < executable.size()) break;
        executable.resize(executable.size() * 2);
      }
      path.assign(executable.data(), size_t(length));
      path.resize(path.rfind('/'));
      path += "/../lib/xsim/libxsim-lidar-gpu.so";
    }
    // The module and its GL globals stay loaded, including after initialization
    // errors. Context destruction remains the GPU owner's responsibility.
    void *handle = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!handle)
      throw std::runtime_error("GPU module not found or could not load: " + path +
                               ": " + dlerror() + "; no CPU fallback");
    auto entry = reinterpret_cast<decltype(&xsim_lidar_gpu_entry)>(
        dlsym(handle, "xsim_lidar_gpu_entry"));
    const auto *table = entry ? entry(XSIM_LIDAR_GPU_ABI) : nullptr;
    if (!table || table->abi != XSIM_LIDAR_GPU_ABI ||
        table->size < sizeof(xsim_lidar_gpu_api) ||
        !table->create || !table->scan || !table->destroy)
      throw std::runtime_error("GPU module ABI mismatch: " + path + "; no CPU fallback");
    return table;
  }();
  return *api;
}
}
