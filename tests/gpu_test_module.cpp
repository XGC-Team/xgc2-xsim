#include <xgc2_sensing/gpu.h>
#include <atomic>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <pthread.h>

namespace {
struct Renderer { pthread_t owner; };
std::atomic<unsigned> creates{0}, scans{0}, destroys{0};
std::atomic<size_t> map_count{0};
std::atomic<uint64_t> map_hash{0};
bool mode(const char *value) {
  const char *selected = std::getenv("XSIM_GPU_TEST_MODE");
  return selected && std::strcmp(selected, value) == 0;
}
void owner(Renderer *renderer = nullptr) {
  char name[16] = {};
  assert(pthread_getname_np(pthread_self(), name, sizeof(name)) == 0);
  assert(std::strcmp(name, "xsim-gpu") == 0);
  assert(!renderer || pthread_equal(renderer->owner, pthread_self()));
}
void *create(const float *xyz, size_t count, const xgc2_sensing_gpu_projection *p,
             char *error, size_t size) {
  owner(); ++creates;
  assert(xyz && count && p && p->h_res == 240);
  uint64_t hash = 14695981039346656037ULL;
  const auto *bytes = reinterpret_cast<const unsigned char *>(xyz);
  for (size_t n = 0; n < count * 12; ++n) { hash ^= bytes[n]; hash *= 1099511628211ULL; }
  map_count = count; map_hash = hash;
  if (mode("create")) {
    std::snprintf(error, size, "%s", "test initialization failure"); return nullptr;
  }
  return new Renderer{pthread_self()};
}
int scan(void *handle, const double position[3], const double q[4], double stamp,
         const xgc2_sensing_gpu_projection *p, xgc2_sensing_gpu_grow grow, void *context,
         char *error, size_t size) {
  owner(static_cast<Renderer *>(handle)); ++scans;
  if (mode("scan")) {
    std::snprintf(error, size, "%s", "test scan failure"); return -1;
  }
  const float points[8] = {float(position[0]), float(q[0]), float(stamp), float(p->h_res),
      float(map_count.load()), float(p->min_range_m), float(p->point_cover_spacing_m), float(p->publish_rate_hz)};
  void *bytes = grow(context, sizeof(points));
  assert(bytes); std::memcpy(bytes, points, sizeof(points));
  return 0;
}
void destroy(void *handle) {
  owner(static_cast<Renderer *>(handle)); ++destroys; delete static_cast<Renderer *>(handle);
}
}
extern "C" const xgc2_sensing_gpu_api *xgc2_sensing_gpu_entry(uint32_t abi) {
  static xgc2_sensing_gpu_api api{XGC2_SENSING_GPU_ABI, sizeof(xgc2_sensing_gpu_api), create, scan, destroy};
  if (abi != XGC2_SENSING_GPU_ABI) return nullptr;
  if (mode("abi")) api.abi = 99;
  if (mode("size")) api.size = sizeof(api) - 1;
  return &api;
}
extern "C" unsigned xsim_gpu_test_creates() { return creates; }
extern "C" unsigned xsim_gpu_test_scans() { return scans; }
extern "C" unsigned xsim_gpu_test_destroys() { return destroys; }
extern "C" size_t xsim_gpu_test_map_count() { return map_count; }
extern "C" uint64_t xsim_gpu_test_map_hash() { return map_hash; }
