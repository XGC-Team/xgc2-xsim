#pragma once
#include "core/world.hpp"
#include "io/config.hpp"
#include "sensors/cadence.hpp"
#include <xgc2_math/scene.hpp>
#include <xgc2_math/scene/body_returns.hpp>
#include "sensors/gpu_module.hpp"
namespace xsim {
struct Sample {
  Key key;
  int64_t stamp = 0;
  uint64_t scene_version = 1;
  uint64_t geometry_revision = 0;
  bool body_observation = false;
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Clock::time_point submitted;
  std::shared_ptr<const Frame> bodies;
};
struct SensorPayload {
  std::vector<uint8_t> data, beam_data;
};
struct GpuScene {
  std::vector<float> points;
  xgc2_sensing_gpu_projection initial{};
  void *renderer = nullptr; // Created and destroyed only by the GPU owner thread.
  bool initialized = false;
  std::string error;
};
struct Sensor {
  static constexpr size_t payload_pool_size = 4;
  std::weak_ptr<Entity> entity;
  // One sensor per scanner: a scan advances the sensor's own scan index and scratch, and only the worker that
  // holds `busy` scans it.
  std::unique_ptr<xgc2_math::scene::WorldLidar> cpu;
  // Worker-owned beam scratch of a publish_beams sensor, reused by every scan.
  xgc2_math::scene::BeamScan beams;
  xgc2_sensing_gpu_projection gpu_config{};
  std::shared_ptr<GpuScene> gpu_scene;
  bool gpu = false, with_bodies=false, publish_beams=false;
  bool cacheable = false, cache_valid = false;
  Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
  int64_t period = 100000000, next = 0, next_requested = 0, last_submitted_stamp = 0;
  uint64_t schedule_generation = 0, observed_generation = 0;
  int64_t last_sample_stamp = 0;
  AdaptiveCadence cadence;
  std::atomic<int64_t> effective_period_ns{100000000}, observed_period_ns{0};
  std::string topic, frame = "world";
  std::mutex mutex;
  Sample pending, completed;
  bool has_pending = false, busy = false, has_completed = false;
  bool queued = false; // Sensors::mutex_; one ready token, never one per sample
  // Fixed owners; only the single scan worker may rewrite a unique payload.
  std::array<std::shared_ptr<SensorPayload>, payload_pool_size> payload_pool;
  size_t payload_cursor = 0;
  std::shared_ptr<const SensorPayload> completed_payload;
  // Worker-owned immutable geometry cache. Hits share bytes without copying.
  Sample cached_sample;
  std::shared_ptr<const SensorPayload> cached_payload;
  std::vector<xgc2_math::scene::VehicleBody> others;
  xgc2_math::scene::SensorConfig body_config;
  xgc2_math::scene::GpuBodyMergeScratch body_scratch;
  std::vector<xgc2_math::scene::TaggedPoint> gpu_static_points, gpu_body_points;
  std::atomic<uint64_t> misses{0}, scans{0}, errors{0};
  std::atomic<uint64_t> throttled{0}, cache_hits{0}, computed_scans{0}, published{0};
  std::atomic<uint64_t> rate_degradations{0}, rate_recoveries{0};
  std::atomic<uint64_t> payload_grows{0}, payload_misses{0};
  uint64_t previous_misses = 0;
  std::atomic<int64_t> latency_ns{0}, max_latency_ns{0};
  std::string error;
};
class Sensors {
public:
  explicit Sensors(const Json &scene, unsigned workers = 2,
                   std::chrono::milliseconds initialization_timeout = std::chrono::seconds(10),
                   const int64_t *phase_t0_ns = nullptr);
  ~Sensors();
  std::shared_ptr<Sensor> prepare(const std::shared_ptr<Entity> &, const Json &config);
  // Single output-thread producer. A whole frame keeps sensor/body observations
  // on the same simulation stamp without reading the live World.
  void submit_frame(const std::shared_ptr<const Frame> &);
  void submit_frame(const Frame &); // ad-hoc fixture copy, not production
  void submit(const State &, std::shared_ptr<const Frame> bodies = {});
  std::vector<uint8_t> reference_cloud() const;
  void stop();
  static bool take_shared(const std::shared_ptr<Sensor> &, Sample &,
                          std::shared_ptr<const SensorPayload> &);
  // Fixture compatibility: copies payload bytes after releasing the sensor lock.
  static bool take(const std::shared_ptr<Sensor> &, Sample &,
                   std::vector<uint8_t> &, std::vector<uint8_t> *beams=nullptr);
  Json status() const;

private:
  // Reused circular storage; capacity grows only when sensors are prepared.
  // Weak tokens do not retain removed sensors or their payload/frame pools.
  struct Ready {
    std::vector<std::weak_ptr<Sensor>> slots;
    size_t head = 0, count = 0;
    void reserve(size_t);
    void push(const std::shared_ptr<Sensor>&);
    std::shared_ptr<Sensor> pop();
  };
  struct FixtureGeometry {
    Key key;
    bool enabled = false;
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  };
  bool phased_ = false;
  int64_t phase_t0_ns_ = 0;
  std::shared_ptr<const xgc2_math::scene::LidarScene> scene_;
  std::vector<xgc2_math::scene::Obstacle> obstacles_;
  double spacing_ = 0.1;
  bool buried_ = false;
  unsigned workers_;
  std::chrono::milliseconds initialization_timeout_;
  int64_t last_frame_stamp_ = 0;
  std::atomic<int64_t> source_period_ns_{0};
  std::vector<FixtureGeometry> fixture_geometry_;
  uint64_t fixture_geometry_revision_ = 0;
  std::map<std::pair<double,bool>,std::shared_ptr<const xgc2_math::scene::LidarScene>> sampled_scenes_;
  mutable std::mutex mutex_;
  std::condition_variable wake_;
  std::condition_variable cpu_wake_;
  Ready cpu_ready_, gpu_ready_;
  size_t gpu_initializations_ = 0;
  std::vector<std::weak_ptr<Sensor>> sensors_;
  std::vector<std::thread> cpu_threads_;
  std::thread gpu_thread_;
  bool stopping_ = false, gpu_creation_running_ = false;
  Clock::time_point gpu_initialization_deadline_;
  std::map<std::pair<double, bool>, std::shared_ptr<GpuScene>> gpu_scenes_;
  const xgc2_sensing_gpu_api *gpu_api_ = nullptr;
  void work(bool gpu);
  void enqueue(const std::shared_ptr<Sensor>&);
  void submit_sample(const State &, const std::shared_ptr<const Frame> &, uint64_t body_revision = 0);
  uint64_t fixture_geometry_revision(const Frame &);
};
} // namespace xsim
