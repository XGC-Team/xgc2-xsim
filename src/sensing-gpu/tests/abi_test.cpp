#include <xgc2_sensing/gpu.h>
#include <xgc2_math/scene/sensor_metadata.hpp>
#include <cassert>
#include <cmath>
#include <cstring>
#include <dlfcn.h>
#include <iostream>
#include <limits>
#include <vector>

int main(int argc, char **argv) {
  assert(argc == 2);
  void *module = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
  if (!module) { std::cerr << dlerror() << '\n'; return 1; }
  auto entry = reinterpret_cast<decltype(&xgc2_sensing_gpu_entry)>(dlsym(module, "xgc2_sensing_gpu_entry"));
  assert(entry && !entry(0) && !entry(XGC2_SENSING_GPU_ABI + 1));
  const auto *api = entry(XGC2_SENSING_GPU_ABI);
  assert(api && api->abi == XGC2_SENSING_GPU_ABI && api->size >= sizeof(*api));
  assert(api->create && api->scan && api->destroy);
  const xgc2_sensing_gpu_projection valid{20, .2, 120, 60, 240, 120, .1, 10};
  std::vector<xgc2_sensing_gpu_projection> invalid;
  for (double value : {0., -1., std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN()}) {
    auto p = valid; p.range_m = value; invalid.push_back(p);
    p = valid; p.min_range_m = value; invalid.push_back(p);
    p = valid; p.point_cover_spacing_m = value; invalid.push_back(p);
    p = valid; p.publish_rate_hz = value; invalid.push_back(p);
    p = valid; p.h_fov_deg = value; invalid.push_back(p);
    p = valid; p.v_fov_deg = value; invalid.push_back(p);
  }
  auto p = valid; p.h_res = 0; invalid.push_back(p);
  p = valid; p.v_res = -1; invalid.push_back(p);
  p = valid; p.h_res = INT32_MAX; p.v_res = 2; invalid.push_back(p);
  p = valid; p.h_res = 241; invalid.push_back(p);
  p = valid; p.v_res = 30; invalid.push_back(p);
  p = valid; p.point_cover_spacing_m = 1; invalid.push_back(p);
  p = valid; p.min_range_m = p.range_m; invalid.push_back(p);
  p = valid; p.h_fov_deg = 361; invalid.push_back(p);
  p = valid; p.v_fov_deg = 181; invalid.push_back(p);
  p = valid; p.publish_rate_hz = double(INT32_MAX) + 1; invalid.push_back(p);
  const float xyz[3] = {1, 2, 3};
  for (const auto &bad : invalid) {
    char error[256] = {};
    xgc2_math::scene::SensorMetadata m;
    m.backend = "gpu"; m.observation_model = "lidar_scan";
    m.frame_id = "world"; m.pose_type = "geometry_msgs/PoseStamped"; m.stamp_policy = "pose";
    m.range_m = bad.range_m; m.min_range_m = bad.min_range_m;
    m.h_fov_deg = bad.h_fov_deg; m.v_fov_deg = bad.v_fov_deg;
    m.h_res = bad.h_res; m.v_res = bad.v_res;
    m.point_cover_spacing_m = bad.point_cover_spacing_m; m.publish_rate_hz = bad.publish_rate_hz;
    bool refused = false;
    try { xgc2_math::scene::validateGpuSensorMetadata(m); }
    catch (const std::invalid_argument &e) {
      refused = true;
      assert(!api->create(xyz, 1, &bad, error, sizeof(error)));
      assert(std::strcmp(error, e.what()) == 0);
    }
    assert(refused);
  }
  char error[128] = {};
  assert(!api->create(nullptr, 0, &valid, error, sizeof(error)) && error[0]);
  assert(!api->create(xyz, 1, nullptr, error, sizeof(error)) && error[0]);
  assert(api->scan(nullptr, nullptr, nullptr, 0, &valid, nullptr, nullptr, error, sizeof(error)) != 0 && error[0]);
  char tiny[2] = {'x', 'x'};
  assert(!api->create(nullptr, 0, &valid, tiny, sizeof(tiny)) && tiny[1] == '\0');
  api->destroy(nullptr);
  // Keep the module resident, as the core does; no GL context was acquired.
  std::cout << "PASS: real module dlopen, ABI and " << invalid.size() << " invalid projections without GL\n";
}
