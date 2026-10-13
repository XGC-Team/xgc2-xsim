// SPDX-License-Identifier: GPL-2.0-only
// Original fixed-window entry against the C ABI's shared-context projection switches.
// Run only with real hardware and a display; software GL is refused by load().
#include "../module.cpp"
#include <xgc2_math/scene.hpp>
#include <cassert>
#include <vector>

int main() {
  omp_set_dynamic(0);
  omp_set_num_threads(1);
  using namespace xgc2_math::scene;
  WorldLidar source(SensorConfig{});
  source.setScene(std::make_shared<LidarScene>(std::vector<Obstacle>{Obstacle::box({5, 0, 0}, {1, 6, 4})}));
  std::vector<float> map;
  for (const auto &p : source.globalMap(.1))
    for (int k = 0; k < 3; ++k) map.push_back(static_cast<float>(p[k]));
  xgc2_sensing_gpu_projection configs[2] = {{20, .2, 90, 30, 120, 40, .1, 10},
                                        {20, .2, 90, 30, 240, 80, .1, 10}};
  pcl::PointCloud<pcl::PointXYZI> references[2];
  for (int i = 0; i < 2; ++i) {
    Renderer original;
    original.load(map.data(), map.size() / 3, metadata(&configs[i]));
    references[i] = original.scan({0, double(i), 0}, Eigen::Quaterniond::Identity(), 1700000000);
    assert(!references[i].empty());
  }
  const auto *api = xgc2_sensing_gpu_entry(XGC2_SENSING_GPU_ABI);
  char error[512] = {};
  auto *shared = api->create(map.data(), map.size() / 3, configs, error, sizeof(error));
  if (!shared) { std::cerr << error << '\n'; return 1; }
  std::vector<unsigned char> bytes;
  const auto grow = [](void *context, size_t size) -> void * {
    auto &output = *static_cast<std::vector<unsigned char> *>(context);
    output.resize(size); return output.data();
  };
  double maximum = 0;
  for (int k = 0; k < 12; ++k) {
    const int i = k % 2;
    const double position[3] = {0, double(i), 0}, orientation[4] = {1, 0, 0, 0};
    assert(api->scan(shared, position, orientation, 1700000000 + k * .1,
                      &configs[i], grow, &bytes, error, sizeof(error)) == 0);
    assert(bytes.size() == references[i].size() * 16);
    for (size_t n = 0; n < references[i].size(); ++n) {
      float actual[4]; std::memcpy(actual, bytes.data() + n * 16, 16);
      const auto &p = references[i][n];
      const float expected[4] = {p.x, p.y, p.z, p.intensity};
      for (int axis = 0; axis < 4; ++axis) {
        const float delta = actual[axis] - expected[axis];
        assert(std::isfinite(delta));
        maximum = std::max(maximum, double(std::abs(delta)));
      }
    }
  }
  api->destroy(shared);
  assert(maximum <= 1e-6);
  std::cout << "{\"result\":\"PASS\",\"switches\":12,\"original_points\":["
            << references[0].size() << ',' << references[1].size()
            << "],\"max_abs_xyzi_error\":" << maximum << ",\"tolerance\":1e-6}\n";
}
