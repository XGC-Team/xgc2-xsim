#include "systems/sensors.hpp"
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <iostream>
#include <thread>

using namespace xsim;
namespace {
std::shared_ptr<Entity> entity(uint64_t id) {
  auto e = std::make_shared<Entity>(parse_entity({{"name", "gpu" + std::to_string(id)}, {"kind", "scout"}}));
  e->id = id; e->alive = e->enabled = true; e->generation = 1;
  return e;
}
Json config(int h = 240, int v = 120) {
  return {{"backend", "gpu"}, {"mode", "raycast"}, {"min_range", .2}, {"range", 20},
          {"h_fov_deg", 120}, {"v_fov_deg", 60}, {"h_res", h}, {"v_res", v},
          {"point_cover_spacing", .1}, {"surface_spacing", .25}, {"keep_buried", true}, {"rate_hz", 10}};
}
float number(const std::vector<uint8_t> &bytes, size_t index) {
  float value; std::memcpy(&value, bytes.data() + 4 * index, 4); return value;
}
template<class T> T symbol(void *handle, const char *name) {
  auto value = reinterpret_cast<T>(dlsym(handle, name)); assert(value); return value;
}
}
int main(int argc, char **argv) {
  assert(argc == 3);
  const std::string mode = argv[1], path = argv[2];
  setenv("XGC2_SENSING_GPU_LIBRARY", mode == "missing" ? "/nonexistent/xgc2-sensing-gpu.so" : path.c_str(), 1);
  setenv("XSIM_GPU_TEST_MODE", mode.c_str(), 1);
  const Json scene = {{"obstacles", Json::array({
      {{"type", "box"}, {"position", {4, 0, 0}}, {"size", {1, 2, 2}}},
      {{"type", "box"}, {"position", {4, .5, 0}}, {"size", {1, 2, 2}}}})}};
  // CPU preparation never loads the optional module, even with a missing path.
  assert(!dlopen(path.c_str(), RTLD_NOW | RTLD_NOLOAD));
  {
    Sensors sensors(scene);
    auto cpu = entity(100);
    cpu->sensor = sensors.prepare(cpu, {{"backend", "cpu"}, {"h_res", 8}, {"v_res", 2}});
    assert(!dlopen(path.c_str(), RTLD_NOW | RTLD_NOLOAD));
    auto bad = config(); bad["v_res"] = 30;
    bool refused = false;
    try { sensors.prepare(entity(101), bad); }
    catch (const std::invalid_argument &) { refused = true; }
    assert(refused && !dlopen(path.c_str(), RTLD_NOW | RTLD_NOLOAD));
  }
  Sensors sensors(scene);
  auto a = entity(1), b = entity(2);
  if (mode == "missing" || mode == "abi" || mode == "size" || mode == "create") {
    bool refused = false;
    try { sensors.prepare(a, config()); }
    catch (const std::runtime_error &e) {
      const std::string error = e.what();
      const char *expected = mode == "missing" ? "unavailable" : mode == "create" ? "test initialization failure" : "ABI mismatch";
      assert(error.find(expected) != std::string::npos);
      refused = true;
    }
    assert(refused); sensors.stop();
  } else {
    a->sensor = sensors.prepare(a, config());
    b->sensor = sensors.prepare(b, config(120, 60));
    void *handle = dlopen(path.c_str(), RTLD_NOW | RTLD_NOLOAD); assert(handle);
    const auto creates = symbol<unsigned (*)()>(handle, "xsim_gpu_test_creates");
    const auto scans = symbol<unsigned (*)()>(handle, "xsim_gpu_test_scans");
    const auto destroys = symbol<unsigned (*)()>(handle, "xsim_gpu_test_destroys");
    assert(creates() == 1 && destroys() == 0);
    using namespace xgc2_math::scene;
    SensorConfig map_config; map_config.penetrating_keep_buried = true;
    WorldLidar source(map_config);
    source.setScene(std::make_shared<LidarScene>(std::vector<Obstacle>{
        Obstacle::box({4, 0, 0}, {1, 2, 2}), Obstacle::box({4, .5, 0}, {1, 2, 2})}));
    std::vector<float> map;
    for (const auto &p : source.globalMap(.25)) for (int k = 0; k < 3; ++k) map.push_back(float(p[k]));
    uint64_t hash = 14695981039346656037ULL;
    const auto *bytes = reinterpret_cast<const unsigned char *>(map.data());
    for (size_t n = 0; n < map.size() * 4; ++n) { hash ^= bytes[n]; hash *= 1099511628211ULL; }
    assert(symbol<size_t (*)()>(handle, "xsim_gpu_test_map_count")() == map.size() / 3);
    assert(symbol<uint64_t (*)()>(handle, "xsim_gpu_test_map_hash")() == hash);
    for (auto robot : {a, b}) {
      State pose; pose.entity = robot; pose.key = {robot->id, robot->generation};
      pose.enabled = true; pose.stamp = 1700000000000000000LL;
      pose.position = {double(robot->id), 2, 3};
      pose.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitZ()));
      sensors.submit(pose);
      const auto deadline = Clock::now() + std::chrono::seconds(5);
      Sample sample; std::vector<uint8_t> data, beams;
      if (mode == "scan") {
        while (!robot->sensor->errors) {
          assert(Clock::now() < deadline); std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        assert(!Sensors::take(robot->sensor, sample, data, &beams));
        assert(sensors.status()[robot->id - 1]["error"].get<std::string>().find("test scan failure") != std::string::npos);
      } else {
        while (!Sensors::take(robot->sensor, sample, data, &beams)) {
          assert(Clock::now() < deadline); std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        assert(sample.stamp == pose.stamp && sample.key.id == pose.key.id && beams.empty());
        assert(data.size() == 32 && number(data, 0) == float(pose.position.x()));
        assert(number(data, 1) == float(pose.orientation.w()) && number(data, 2) == float(double(pose.stamp) * 1e-9));
        assert(number(data, 3) == (robot == a ? 240.f : 120.f));
        assert(number(data, 4) == float(map.size() / 3) && number(data, 5) == .2f);
        assert(number(data, 6) == .1f && number(data, 7) == 10.f && robot->sensor->errors == 0);
      }
    }
    assert(scans() == 2);
    sensors.stop(); assert(destroys() == 1);
  }
  std::cout << "PASS: GPU module " << mode << '\n';
}
