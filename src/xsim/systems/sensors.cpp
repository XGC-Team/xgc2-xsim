#include "sensors/scene_document.hpp"
#include "sensors.hpp"
#include <xgc2_math/scene/scene_conversion.hpp>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <limits>
#include <pthread.h>
#ifdef __linux__
#include <cerrno>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif
namespace xsim {
namespace {
[[noreturn]] void initialization_expired(std::chrono::milliseconds timeout) {
  std::fprintf(stderr, "xsim: GPU initialization deadline exceeded after %lld ms; terminating process\n",
               static_cast<long long>(timeout.count()));
  std::_Exit(EXIT_FAILURE);
}
Eigen::Vector3d v3(const Json &j) {
  if (j.size() != 3)
    throw std::invalid_argument("sensor vector needs 3 values");
  return {j.at(0).get<double>(), j.at(1).get<double>(), j.at(2).get<double>()};
}
Eigen::Quaterniond quat(const Json &j) {
  if (j.size() != 4)
    throw std::invalid_argument("sensor quaternion needs wxyz");
  Eigen::Quaterniond q(j[0].get<double>(), j[1].get<double>(),
                       j[2].get<double>(), j[3].get<double>());
  if (!q.coeffs().allFinite() || q.norm() < 1e-12)
    throw std::invalid_argument("sensor quaternion invalid");
  return q.normalized();
}
bool same_observation(const Sample &a, const Sample &b) {
  if (a.key.id != b.key.id || a.key.generation != b.key.generation ||
      a.scene_version != b.scene_version || a.scene.get() != b.scene.get() ||
      !(a.position.array() == b.position.array()).all() ||
      !(a.orientation.coeffs().array() == b.orientation.coeffs().array()).all())
    return false;
  return a.body_observation == b.body_observation &&
      (!a.body_observation || (a.geometry_revision &&
                              a.geometry_revision == b.geometry_revision));
}
} // namespace
void Sensors::Ready::reserve(size_t needed) {
  if (slots.size() >= needed) return;
  std::vector<std::weak_ptr<Sensor>> larger(std::max(needed, slots.size() * 2 + 8));
  for (size_t i = 0; i < count; ++i) larger[i] = std::move(slots[(head + i) % slots.size()]);
  slots.swap(larger);
  head = 0;
}
void Sensors::Ready::push(const std::shared_ptr<Sensor>& sensor) {
  assert(count < slots.size());
  slots[(head + count++) % slots.size()] = sensor;
}
std::shared_ptr<Sensor> Sensors::Ready::pop() {
  assert(count);
  auto sensor = slots[head].lock();
  slots[head].reset();
  head = (head + 1) % slots.size();
  --count;
  return sensor;
}
Sensors::Sensors(const Json &j, unsigned workers, std::chrono::milliseconds initialization_timeout,
                 const int64_t *phase_t0_ns)
    : workers_(workers), initialization_timeout_(initialization_timeout) {
  if (phase_t0_ns) {
    if (*phase_t0_ns < 0) throw std::invalid_argument("phase T0 must be nonnegative");
    phased_ = true;
    phase_t0_ns_ = *phase_t0_ns;
  }
  if (initialization_timeout.count() <= 0 || initialization_timeout > std::chrono::minutes(10))
    throw std::invalid_argument("sensor initialization timeout must be 1 through 600000 ms");
  if (!workers)
    throw std::invalid_argument("sensor_workers must be positive");
  spacing_ = j.value("surface_spacing", 0.1);
  buried_ = j.value("keep_buried", false);
  // Compiled.v1 is read once here. The reader decodes each geometry once and
  // shares mesh handles; this constructor does not insert a ground solid.
  if(j.contains("document")) {
    const auto compiled = compiledSceneDocument(j.at("document"));
    obstacles_ = compiled.obstacles;
    dynamic_obstacles_ = compiled.dynamic_obstacles;
    dynamic_scene_ = !dynamic_obstacles_.empty();
    dynamic_ids_.reserve(dynamic_obstacles_.size());
    for (const auto &item : dynamic_obstacles_) dynamic_ids_.push_back(item.id);
  }
  if (j.contains("obstacles"))
    throw std::invalid_argument("scene.obstacles is retired; supply scene.document with named parts");
  // One immutable compiled geometry/index for this world, never one map per
  // robot. A static document publishes it immediately. A dynamic document
  // keeps the same object but does not scan until an external state arrives.
  scene_ = std::make_shared<xgc2_math::scene::LidarScene>(obstacles_, spacing_,
                                                          buried_);
  if (dynamic_scene_) {
    scene_ready_.store(false, std::memory_order_release);
    scene_version_.store(0, std::memory_order_release);
  }
}
bool quaternion_usable(const Eigen::Quaterniond &q) {
  if (!q.coeffs().allFinite()) return false;
  return q.coeffs().cwiseAbs().maxCoeff() > 1e-9;
}
std::shared_ptr<const xgc2_math::scene::LidarScene>
Sensors::scene_for(const xgc2_math::scene::SensorConfig &config) const {
  if (config.mode == xgc2_math::scene::SensorConfig::kPenetrating &&
      (config.surface_spacing != spacing_ || config.penetrating_keep_buried != buried_)) {
    const auto found = sampled_scenes_.find(std::make_pair(config.surface_spacing, config.penetrating_keep_buried));
    if (found != sampled_scenes_.end() && found->second) return found->second;
  }
  return scene_;
}
namespace {
bool same_pose(const xgc2_math::Pose3 &a, const xgc2_math::Pose3 &b) {
  if (a.position != b.position || a.orientation.coeffs() != b.orientation.coeffs()) return false;
  return true;
}
}
bool Sensors::claim_scene_job(bool bootstrap, SceneJob *job) {
  if (!dynamic_scene_ || build_inflight_ || !has_pending_job_ || !job) return false;
  if (bootstrap) {
    if (!bootstrap_queued_ || scene_ready_.load(std::memory_order_relaxed)) return false;
  } else if (!scene_ready_.load(std::memory_order_relaxed)) return false;
  if (pending_job_.ticket <= fence_ticket_ || pending_job_.ticket <= installed_ticket_) return false;
  *job = pending_job_;
  build_inflight_ = true;
  if (bootstrap) bootstrap_queued_ = false;
  return true;
}
void Sensors::release_scene_claim() {
  std::lock_guard<std::mutex> lock(mutex_);
  build_inflight_ = false;
  if (has_pending_job_ &&
      (pending_job_.ticket <= installed_ticket_ || pending_job_.ticket <= fence_ticket_)) {
    has_pending_job_ = false;
    pending_job_ = {};
  }
  if (!scene_ready_.load(std::memory_order_relaxed) && has_pending_job_ &&
      pending_job_.ticket > fence_ticket_ && pending_job_.ticket > installed_ticket_)
    bootstrap_queued_ = true;
}
void Sensors::invalidate_dynamic_scene() {
  identity_invalidated_ = true;
  fence_ticket_ = std::max(fence_ticket_, next_ticket_);
  has_pending_job_ = false;
  pending_job_ = {};
  bootstrap_queued_ = false;
  scene_ready_.store(false, std::memory_order_release);
}
bool Sensors::bind_dynamic_scene(const std::string &epoch, uint64_t document_revision,
                                 const std::string &frame_id,
                                 const std::vector<std::string> &dynamic_ids) {
  if (!dynamic_scene_) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  if (identity_invalidated_) return false;
  const bool frame_ok = frame_id == "world";
  const bool ids_ok = dynamic_ids == dynamic_ids_;
  if (!identity_bound_) {
    if (!frame_ok || !ids_ok || epoch.empty() || document_revision == 0) return false;
    bound_epoch_ = epoch;
    bound_revision_ = document_revision;
    identity_bound_ = true;
    return true;
  }
  if (frame_ok && ids_ok && epoch == bound_epoch_ && document_revision == bound_revision_) return true;
  invalidate_dynamic_scene();
  return false;
}
void Sensors::publish_scene_job(const SceneJob &job) {
  std::lock_guard<std::mutex> serial(update_mu_);
  std::shared_ptr<const xgc2_math::scene::LidarScene> previous;
  std::map<std::pair<double, bool>, std::shared_ptr<const xgc2_math::scene::LidarScene>> previous_policies;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ || job.ticket <= fence_ticket_ || job.ticket <= installed_ticket_) return;
    previous = scene_;
    previous_policies = sampled_scenes_;
  }
  if (!previous) return;
  auto next_obstacles = obstacles_;
  auto next_dynamic = dynamic_obstacles_;
  try {
    for (auto &dyn : next_dynamic) {
      const auto found = job.poses.find(dyn.id);
      if (found == job.poses.end()) throw std::invalid_argument("dynamic obstacle missing from scene state");
      const auto &parent = found->second;
      dyn.pose = parent;
      for (const auto &part : dyn.parts) {
        if (part.obstacle_index >= next_obstacles.size())
          throw std::invalid_argument("dynamic part index is out of range");
        const auto local_q = xgc2_math::normalizedQuaternion(part.local_pose.orientation);
        auto &shape = next_obstacles[part.obstacle_index];
        shape.position = parent.position + parent.orientation * part.local_pose.position;
        shape.orientation = parent.orientation * local_q;
      }
    }
    auto next_scene = std::make_shared<const xgc2_math::scene::LidarScene>(
        next_obstacles, spacing_, buried_, *previous);
    std::map<std::pair<double, bool>, std::shared_ptr<const xgc2_math::scene::LidarScene>> rebuilt;
    for (const auto &entry : previous_policies) {
      if (!entry.second) continue;
      rebuilt.emplace(entry.first, std::make_shared<const xgc2_math::scene::LidarScene>(
          next_obstacles, entry.first.first, entry.first.second, *entry.second));
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ || job.ticket <= fence_ticket_ || job.ticket <= installed_ticket_) return;
    scene_ = std::move(next_scene);
    for (auto &entry : rebuilt) {
      const auto snap = previous_policies.find(entry.first);
      auto it = sampled_scenes_.find(entry.first);
      if (it == sampled_scenes_.end())
        sampled_scenes_.emplace(entry.first, std::move(entry.second));
      else if (snap != previous_policies.end() && it->second == snap->second)
        it->second = std::move(entry.second);
    }
    obstacles_.swap(next_obstacles);
    dynamic_obstacles_.swap(next_dynamic);
    installed_ticket_ = job.ticket;
    scene_version_.fetch_add(1, std::memory_order_release);
    scene_ready_.store(true, std::memory_order_release);
  } catch (...) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fence_ticket_ < job.ticket) fence_ticket_ = job.ticket;
    if (has_pending_job_ && pending_job_.ticket <= fence_ticket_) has_pending_job_ = false;
    scene_ready_.store(false, std::memory_order_release);
  }
}
std::shared_ptr<const xgc2_math::scene::LidarScene> Sensors::cold_policy(double spacing, bool buried) {
  const auto key = std::make_pair(spacing, buried);
  std::lock_guard<std::mutex> serial(update_mu_);
  for (;;) {
    std::shared_ptr<const xgc2_math::scene::LidarScene> existing;
    uint64_t version = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_) throw std::runtime_error("sensor owner stopped during preparation");
      version = scene_version_.load(std::memory_order_relaxed);
      const auto found = sampled_scenes_.find(key);
      if (found != sampled_scenes_.end() && found->second) return found->second;
    }
    auto copy = obstacles_;
    auto built = std::make_shared<const xgc2_math::scene::LidarScene>(copy, spacing, buried);
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_) throw std::runtime_error("sensor owner stopped during preparation");
    if (scene_version_.load(std::memory_order_relaxed) != version) continue;
    auto &slot = sampled_scenes_[key];
    if (!slot) slot = std::move(built);
    existing = slot;
    return existing;
  }
}
bool Sensors::update_scene_state(const std::string &frame_id, const std::string &epoch, uint64_t revision,
                                 const std::vector<SceneObstaclePose> &poses) {
  if (!dynamic_scene_) return false;
  std::map<std::string, xgc2_math::Pose3> incoming;
  bool bad_pose = false;
  for (const auto &item : poses) {
    if (!item.pose.position.allFinite() || !quaternion_usable(item.pose.orientation)) {
      bad_pose = true;
      break;
    }
    xgc2_math::Pose3 pose;
    pose.position = item.pose.position;
    pose.orientation = xgc2_math::normalizedQuaternion(item.pose.orientation);
    if (!incoming.emplace(item.id, std::move(pose)).second) {
      bad_pose = true;
      break;
    }
  }
  bool wake = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (identity_invalidated_ || !identity_bound_) return false;
    if (frame_id != "world" || epoch != bound_epoch_ || revision != bound_revision_) {
      invalidate_dynamic_scene();
      return false;
    }
    const auto fence_poses = [&] {
      fence_ticket_ = std::max(fence_ticket_, next_ticket_);
      has_pending_job_ = false;
      pending_job_ = {};
      bootstrap_queued_ = false;
      scene_ready_.store(false, std::memory_order_release);
    };
    if (bad_pose || incoming.size() != dynamic_obstacles_.size()) {
      fence_poses();
      return false;
    }
    for (const auto &dyn : dynamic_obstacles_) {
      if (incoming.find(dyn.id) == incoming.end()) {
        fence_poses();
        return false;
      }
    }
    if (has_pending_job_) {
      bool same = true;
      for (const auto &dyn : dynamic_obstacles_) {
        const auto left = incoming.find(dyn.id);
        const auto right = pending_job_.poses.find(dyn.id);
        if (left == incoming.end() || right == pending_job_.poses.end() || !same_pose(left->second, right->second))
          same = false;
      }
      if (same) return true;
    }
    // Installed A, then B, then A again, with no build in flight: B is stale.
    // Drop it instead of minting a ticket that rebuilds A. An in-flight B can
    // still publish, so that A stays pending below.
    if (scene_ready_.load(std::memory_order_relaxed) && !build_inflight_) {
      bool same = true;
      for (const auto &dyn : dynamic_obstacles_) {
        const auto found = incoming.find(dyn.id);
        if (found == incoming.end() || !same_pose(found->second, dyn.pose)) same = false;
      }
      if (same) {
        has_pending_job_ = false;
        pending_job_ = {};
        return true;
      }
    }
    SceneJob job;
    job.ticket = ++next_ticket_;
    job.poses = std::move(incoming);
    pending_job_ = std::move(job);
    has_pending_job_ = true;
    if (!scene_ready_.load(std::memory_order_relaxed) && !bootstrap_queued_ && !build_inflight_) {
      bootstrap_queued_ = true;
      wake = true;
    }
  }
  if (wake) cpu_wake_.notify_one();
  return true;
}
std::vector<uint8_t> Sensors::reference_cloud() const {
  std::shared_ptr<const xgc2_math::scene::LidarScene> scene;
  if (!dynamic_scene_) scene = scene_;
  else {
    std::lock_guard<std::mutex> lock(mutex_);
    scene = scene_;
  }
  const auto& points=scene->samples();
  std::vector<uint8_t> data(points.size()*12);size_t at=0;for(const auto& p:points){float xyz[3]={float(p.x()),float(p.y()),float(p.z())};std::memcpy(data.data()+at,xyz,12);at+=12;}return data;
}
Sensors::~Sensors() { stop(); }
std::shared_ptr<Sensor> Sensors::prepare(const std::shared_ptr<Entity> &e, const Json &config) {
  const auto &j = config;
  if (j.empty())
    return {};
  auto s = std::make_shared<Sensor>();
  for (auto &payload : s->payload_pool) payload = std::make_shared<SensorPayload>();
  s->entity = e;
  s->topic = j.value("topic", "/" + e->config.name + "/cloud");
  s->frame = j.value("frame", "world");
  if (s->frame != "world" && s->frame != "map")
    throw std::invalid_argument("existing sensor emits world/map points");
  const auto hz = j.value("rate_hz", 10.0);
  if (!std::isfinite(hz) || hz <= 0 || hz > 1e9)
    throw std::invalid_argument("invalid sensor rate");
  const double requested_period = 1e9 / hz;
  if (!std::isfinite(requested_period) ||
      requested_period >= double(std::numeric_limits<int64_t>::max()))
    throw std::invalid_argument("sensor period is not representable");
  s->period = std::max<int64_t>(1, std::llround(requested_period));
  s->cadence = AdaptiveCadence(s->period);
  s->effective_period_ns = s->period;
  if (j.contains("translation"))
    s->translation = v3(j.at("translation"));
  if (!s->translation.allFinite())
    throw std::invalid_argument("invalid sensor translation");
  if (j.contains("rotation"))
    s->rotation = quat(j.at("rotation"));
  const auto backend = j.value("backend", std::string("cpu"));
  if (backend != "auto" && backend != "cpu" && backend != "gpu")
    throw std::invalid_argument("backend must be auto, cpu or gpu");
  const bool automatic = backend == "auto";
  s->gpu = backend != "cpu";
  if (dynamic_scene_ && s->gpu) {
    if (!automatic)
      throw std::invalid_argument("GPU sensor backend does not support dynamic scene obstacles");
    s->gpu = false;
  }
  const bool requested_bodies = j.value("world_bodies",false);
  s->with_bodies=requested_bodies;s->publish_beams=j.value("publish_beams",false);
  if (s->publish_beams && automatic) s->gpu = false;
  if (s->gpu && s->publish_beams) throw std::invalid_argument("GPU point observations do not publish rays");
  xgc2_math::scene::SensorConfig c;
  c.range = j.value("range", 20.0);
  c.min_range = j.value("min_range", 0.0);
  c.h_res = j.value("h_res", 360);
  c.v_res = j.value("v_res", 32);
  c.h_fov_deg = j.value("h_fov_deg", 360.0);
  c.v_fov_deg = j.value("v_fov_deg", 30.0);
  c.noise_std = j.value("noise_std", 0.0);
  // Static observation bytes depend on pose/geometry, not the transport stamp.
  s->cacheable = c.noise_std == 0;
  c.seed = j.value("seed", 0u);
  c.surface_spacing = spacing_;
  c.penetrating_keep_buried = j.value("keep_buried",buried_);
  c.surface_spacing=j.value("surface_spacing",spacing_);
  c.penetrating_heading_crop=j.value("heading_crop",false);
  c.heading_cos_min=j.value("heading_cos_min",0.0);
  c.vertical_slab_tan=j.value("vertical_slab_tan",0.5773502691896258);
  const auto mode = j.value("mode", std::string("raycast"));
  if (mode == "raycast")
    c.mode = xgc2_math::scene::SensorConfig::kRaycast;
  else if (mode == "penetrating")
    c.mode = xgc2_math::scene::SensorConfig::kPenetrating;
  else if (mode == "depth") {
    c.mode = xgc2_math::scene::SensorConfig::kDepthFrustum;
    c.width = j.value("width", 160);
    c.height = j.value("height", 120);
    c.fx = j.value("fx", 0.0);
    c.fy = j.value("fy", 0.0);
    c.cx = j.value("cx", 0.0);
    c.cy = j.value("cy", 0.0);
  } else throw std::invalid_argument("unknown sensor model");
  if(s->publish_beams && c.mode==xgc2_math::scene::SensorConfig::kPenetrating)throw std::invalid_argument("penetrating model has no beams");
  s->body_config = c;
  auto prepare_cpu = [&] {
    s->with_bodies = requested_bodies;
    s->cpu = std::make_unique<xgc2_math::scene::WorldLidar>(c);
    const bool custom = c.mode == xgc2_math::scene::SensorConfig::kPenetrating &&
                        (c.surface_spacing != spacing_ || c.penetrating_keep_buried != buried_);
    if (!dynamic_scene_) {
      if (custom) {
        const auto key = std::make_pair(c.surface_spacing, c.penetrating_keep_buried);
        std::lock_guard<std::mutex> l(mutex_);
        auto &scene = sampled_scenes_[key];
        if (!scene) scene = std::make_shared<xgc2_math::scene::LidarScene>(obstacles_, key.first, key.second);
        s->cpu->setScene(scene);
      } else s->cpu->setScene(scene_);
      return;
    }
    if (custom) {
      s->cpu->setScene(cold_policy(c.surface_spacing, c.penetrating_keep_buried));
      return;
    }
    std::shared_ptr<const xgc2_math::scene::LidarScene> handle;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      handle = scene_;
    }
    s->cpu->setScene(std::move(handle));
  };
  if (!s->gpu) prepare_cpu();
  else {
    try {
    if (c.mode == xgc2_math::scene::SensorConfig::kDepthFrustum || c.noise_std != 0)
      throw std::invalid_argument(
          "GPU supports angular and penetrating observations without range noise");
    xgc2_math::scene::SensorMetadata m;
    m.backend = "gpu";
    m.observation_model = "lidar_scan";
    m.range_m = c.range;
    m.min_range_m = c.min_range;
    m.h_fov_deg = c.h_fov_deg;
    m.v_fov_deg = c.v_fov_deg;
    m.h_res = c.h_res;
    m.v_res = c.v_res;
    m.point_cover_spacing_m = j.value("point_cover_spacing", spacing_);
    m.publish_rate_hz = hz;
    m.frame_id = s->frame;
    m.stamp_policy = "pose";
    m.pose_type = "geometry_msgs/PoseStamped";
    if (c.mode != xgc2_math::scene::SensorConfig::kPenetrating) xgc2_math::scene::validateGpuSensorMetadata(m);
    s->gpu_config = {m.range_m, m.min_range_m, m.h_fov_deg, m.v_fov_deg,
                     m.h_res, m.v_res, m.point_cover_spacing_m, m.publish_rate_hz,
                     uint32_t(c.mode == xgc2_math::scene::SensorConfig::kPenetrating),
                     uint32_t(c.penetrating_heading_crop), c.heading_cos_min, c.vertical_slab_tan};
    } catch (...) { if (!automatic) throw; s->gpu=false;prepare_cpu(); }
  }
  std::unique_lock<std::mutex> lock(mutex_);
  if (stopping_) throw std::runtime_error("sensor owner stopped during preparation");
  if (s->gpu && !gpu_thread_.joinable()) {
    try { gpu_api_ = &load_gpu_module(); } catch (...) {
      if (!automatic) throw;
      s->gpu = false;lock.unlock();prepare_cpu();lock.lock();
    }
  }
  if (s->gpu) {
    const auto key = std::make_pair(c.surface_spacing, c.penetrating_keep_buried);
    auto &group = gpu_scenes_[key];
    if (!group) {
      group = std::make_shared<GpuScene>();
      auto map_scene = scene_;
      if (key != std::make_pair(spacing_, buried_)) {
        auto &sampled = sampled_scenes_[key];
        if (!sampled)
          sampled = std::make_shared<xgc2_math::scene::LidarScene>(obstacles_, key.first, key.second);
        map_scene = sampled;
      }
      group->points.reserve(map_scene->sampleCount() * 3);
      for (const auto &p : map_scene->samples())
        for (int axis = 0; axis < 3; ++axis) group->points.push_back(float(p[axis]));
      group->initial = s->gpu_config;
      ++gpu_initializations_;
    }
    s->gpu_scene = group;
  }
  sensors_.erase(
      std::remove_if(sensors_.begin(), sensors_.end(),
                     [](const auto &weak) { return weak.expired(); }),
      sensors_.end());
  sensors_.push_back(s);
  // Include stale weak tokens as well as the current roster. Each live sensor
  // can enqueue at most once, including after an automatic GPU fallback.
  cpu_ready_.reserve(cpu_ready_.count + sensors_.size());
  gpu_ready_.reserve(gpu_ready_.count + sensors_.size());
  if (s->gpu) {
    if (!gpu_thread_.joinable()) {
      gpu_thread_ = std::thread([this] { work(true); });
    }
    wake_.notify_all();
    if (!wake_.wait_for(lock, initialization_timeout_, [&] { return s->gpu_scene->initialized || stopping_; })) {
      // A foreign graphics call cannot be unwound or joined safely. Ending this
      // process lets the OS reclaim its context; never detach the owner thread.
      initialization_expired(initialization_timeout_);
    }
    if (stopping_) throw std::runtime_error("sensor owner stopped during initialization");
    if (!s->gpu_scene->renderer) {
      if (!automatic) throw std::runtime_error(s->gpu_scene->error.empty() ? "sensor owner stopped" : s->gpu_scene->error);
      s->gpu = false;lock.unlock();prepare_cpu();lock.lock();
    }
  }
  if (!s->gpu && cpu_threads_.empty())
    for (unsigned i = 0; i < workers_; ++i)
      cpu_threads_.emplace_back([this] { work(false); });
  wake_.notify_all();
  return s;
}
void Sensors::submit_frame(const std::shared_ptr<const Frame> &frame) {
  if (!frame) return;
  if (last_frame_stamp_ && frame->stamp > last_frame_stamp_)
    source_period_ns_ = frame->stamp - last_frame_stamp_;
  last_frame_stamp_ = frame->stamp;
  const auto body_revision = fixture_geometry_revision(*frame);
  for (const auto &v : frame->states) submit_sample(v, frame, body_revision);
}
uint64_t Sensors::fixture_geometry_revision(const Frame &frame) {
  // Only eligible target geometry invalidates observations. The default-off
  // roster never makes scanner caches depend on invisible neighbour motion.
  bool changed = !fixture_geometry_revision_;
  size_t count = 0;
  for (const auto& current : frame.states) {
    if (!current.body_returns) continue;
    if (count >= fixture_geometry_.size()) changed = true;
    else {
      const auto& previous = fixture_geometry_[count];
      if (previous.key.id != current.key.id || previous.key.generation != current.key.generation ||
          !(previous.position.array() == current.position.array()).all()) changed = true;
    }
    ++count;
  }
  changed |= count != fixture_geometry_.size();
  if (changed) {
    ++fixture_geometry_revision_;
    fixture_geometry_.resize(count);
    size_t i = 0;
    for (const auto& current : frame.states)
      if (current.body_returns) fixture_geometry_[i++] = {current.key, current.enabled, current.position};
  }
  return fixture_geometry_revision_;
}
void Sensors::submit_frame(const Frame &frame) {
  auto fixture = std::make_shared<Frame>(frame);
  submit_frame(std::shared_ptr<const Frame>(std::move(fixture)));
}
void Sensors::submit(const State &v, std::shared_ptr<const Frame> bodies) {
  const auto revision = bodies ? fixture_geometry_revision(*bodies) : 0;
  submit_sample(v, bodies, revision);
}
void Sensors::submit_sample(const State &v, const std::shared_ptr<const Frame> &bodies, uint64_t body_revision) {
  auto owner = v.entity.lock();
  auto s = owner ? owner->sensor : nullptr;
  if (!s)
    return;
  if (dynamic_scene_ && !scene_ready_.load(std::memory_order_acquire))
    return;
  if (!owner->alive || !owner->enabled || owner->generation != v.key.generation) {
    std::unique_lock<std::mutex> lock(s->mutex, std::try_to_lock);
    if (lock.owns_lock() && s->has_pending) {
      s->has_pending = false;
      s->pending.bodies.reset();
      ++s->misses;
    }
    return;
  }
  if (s->schedule_generation != v.key.generation) {
    s->schedule_generation = v.key.generation;
    s->next = s->next_requested = v.stamp;
    s->effective_period_ns = s->period;
    s->observed_period_ns = 0;
    s->last_submitted_stamp = 0;
  }
  if (s->effective_period_ns > s->period) {
    std::unique_lock<std::mutex> lock(s->mutex, std::try_to_lock);
    if (lock.owns_lock() && !s->busy && !s->has_pending && !s->has_completed) {
      const auto old_period = s->effective_period_ns.load();
      const auto wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          Clock::now().time_since_epoch()).count();
      const auto recovered = s->cadence.recover(wall_ns);
      if (recovered < old_period) {
        s->effective_period_ns = recovered;
        ++s->rate_recoveries;
        if (s->last_submitted_stamp) {
          const auto interval = std::max(recovered, source_period_ns_.load());
          if (phased_) {
            s->next = observation_deadline(s->next, s->last_submitted_stamp, interval, &phase_t0_ns_);
          } else {
            const auto limit = std::numeric_limits<int64_t>::max();
            const auto candidate = s->last_submitted_stamp > limit - interval
                ? limit : s->last_submitted_stamp + interval;
            s->next = std::min(s->next, candidate);
          }
        }
      }
    }
  }
  // The worker publishes a new effective period only. Rebase this producer's
  // deadline from the latest submission before eligibility. Absent T0 keeps
  // the previous-sample grid above.
  if (phased_ && s->last_submitted_stamp) {
    const auto interval = std::max(s->effective_period_ns.load(), source_period_ns_.load());
    if (interval > 0)
      s->next = observation_deadline(s->next, s->last_submitted_stamp, interval, &phase_t0_ns_);
  }
  const int64_t *phase = phased_ ? &phase_t0_ns_ : nullptr;
  if (v.stamp < s->next_requested) return;
  if (!s->next_requested) s->next_requested = v.stamp;
  const auto due = uint64_t((v.stamp - s->next_requested) / s->period) + 1;
  s->next_requested = observation_deadline(s->next_requested, v.stamp, s->period, phase);
  if (v.stamp < s->next) {
    s->throttled += due;
    return;
  }
  // Account separately for deliberately lower observation cadence. The input
  // frame cadence is an upper bound, even while the configured Hz stays intact.
  if (due > 1) s->throttled += due - 1;
  const auto period = std::max(s->effective_period_ns.load(), source_period_ns_.load());
  s->next = observation_deadline(s->next, v.stamp, period, phase);
  std::shared_ptr<const xgc2_math::scene::LidarScene> active_scene;
  uint64_t active_version = 1;
  if (dynamic_scene_) {
    std::lock_guard<std::mutex> scene_lock(mutex_);
    if (!scene_ready_.load(std::memory_order_relaxed)) return;
    active_version = scene_version_.load(std::memory_order_relaxed);
    active_scene = scene_for(s->body_config);
    if (!active_scene) return;
  }
  std::unique_lock<std::mutex> lock(s->mutex, std::try_to_lock);
  if (!lock.owns_lock()) {
    ++s->misses;
    return;
  }
  if (s->has_pending)
    ++s->misses;
  s->pending = {};
  s->pending.key = v.key;
  s->pending.stamp = v.stamp;
  s->pending.position = v.position + v.orientation * s->translation;
  s->pending.orientation = v.orientation * s->rotation;
  s->pending.submitted = Clock::now();
  if (dynamic_scene_) {
    s->pending.scene = std::move(active_scene);
    s->pending.scene_version = active_version;
  }
  bool observe_bodies = false;
  if (bodies) for (const auto& body : bodies->states)
    observe_bodies |= body.body_returns && body.key.id != v.key.id;
  s->pending.body_observation = observe_bodies;
  if (observe_bodies) {
    s->pending.bodies = bodies;
    s->pending.geometry_revision = body_revision;
  }
  s->has_pending = true;
  s->last_submitted_stamp = v.stamp;
  const bool ready = !s->busy;
  lock.unlock();
  if (ready) enqueue(s);
}
void Sensors::enqueue(const std::shared_ptr<Sensor>& sensor) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ || sensor->queued) return;
    std::lock_guard<std::mutex> sl(sensor->mutex);
    if (sensor->busy || !sensor->has_pending) return;
    sensor->queued = true;
    (sensor->gpu ? gpu_ready_ : cpu_ready_).push(sensor);
  }
  if (sensor->gpu) wake_.notify_all(); // GPU owner and prepare's initialization waiter
  else cpu_wake_.notify_one();
}
void Sensors::work(bool gpu) {
  pthread_setname_np(pthread_self(), gpu ? "xsim-gpu" : "xsim-sensor");
#ifdef __linux__
  // Linux nice is per task: lower only this best-effort observation worker,
  // leaving World/input/output unchanged. Respect an already lower priority;
  // an unavailable adjustment simply keeps the inherited scheduler policy.
  const auto tid = static_cast<id_t>(syscall(SYS_gettid));
  errno = 0;
  const int inherited_nice = getpriority(PRIO_PROCESS, tid);
  if (!errno && inherited_nice < 5)
    (void)setpriority(PRIO_PROCESS, tid, 5);
#endif
  for (;;) {
    std::shared_ptr<Sensor> selected;
    Sample sample;
    SceneJob claimed_job;
    bool have_claim = false;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      if (stopping_)
        break;
      if (!gpu && claim_scene_job(true, &claimed_job)) {
        have_claim = true;
        lock.unlock();
      }
    }
    if (have_claim && !selected) {
      try { publish_scene_job(claimed_job); }
      catch (...) { release_scene_claim(); throw; }
      release_scene_claim();
      continue;
    }
    {
      std::unique_lock<std::mutex> lock(mutex_);
      if (stopping_)
        break;
      if (gpu && gpu_initializations_) {
        // Geometry policies share one index/context each; robots do not own maps.
        for (const auto &entry : gpu_scenes_) {
          auto &group = *entry.second;
          if (stopping_) break;
          if (group.initialized) continue;
          gpu_creation_running_ = true;
          gpu_initialization_deadline_ = Clock::now() + initialization_timeout_;
          lock.unlock();
          char error[512] = {};
          void *renderer = nullptr;
          std::string failure;
          try {
            renderer = gpu_api_->create(group.points.data(), group.points.size() / 3,
                                        &group.initial, error, sizeof(error));
            if (!renderer) failure = std::string("GPU module create failed: ") + error;
          } catch (const std::exception& exception) {
            failure = std::string("GPU module create failed: ") + exception.what();
          } catch (...) {
            failure = "GPU module create failed: unknown exception";
          }
          lock.lock();
          gpu_creation_running_ = false;
          group.renderer = renderer;
          group.error = std::move(failure);
          group.initialized = true;
          --gpu_initializations_;
          std::vector<float>().swap(group.points);
          wake_.notify_all();
        }
      }
      auto& ready = gpu ? gpu_ready_ : cpu_ready_;
      while (ready.count) {
        auto s = ready.pop();
        if (!s) continue;
        s->queued = false;
        auto e = s->entity.lock();
        if (!e || !e->alive || !e->enabled) {
          std::lock_guard<std::mutex> sl(s->mutex);
          if (s->has_pending) {
            s->has_pending = false;
            s->pending.bodies.reset();
            ++s->misses;
          }
          continue;
        }
        std::lock_guard<std::mutex> sl(s->mutex);
        if (s->has_pending && s->pending.key.generation != e->generation) {
          s->has_pending = false;
          s->pending.bodies.reset();
          ++s->misses;
        }
        if (!s->busy && s->has_pending) {
          s->busy = true;
          s->has_pending = false;
          sample = std::move(s->pending);
          s->pending = {};
          selected = s;
          break;
        }
      }
      if (!selected) {
        const auto available = [&] {
          return stopping_ || ready.count || (gpu && gpu_initializations_) || (!gpu && bootstrap_queued_);
        };
        if (gpu) wake_.wait(lock, available);
        else cpu_wake_.wait(lock, available);
        continue;
      }
      if (!gpu && claim_scene_job(false, &claimed_job)) have_claim = true;
    }
    if (have_claim) {
      try { publish_scene_job(claimed_job); }
      catch (...) { release_scene_claim(); throw; }
      release_scene_claim();
    }
    auto &s = *selected;
    bool ok = true, reused = false, payload_pressure = false;
    std::shared_ptr<const SensorPayload> payload;
    SensorPayload *writable = nullptr;
    size_t previous_data_capacity = 0, previous_beam_capacity = 0;
    if (s.cached_sample.key.generation != sample.key.generation) {
      s.cache_valid = false;
      s.cached_payload.reset();
    }
    try {
      if (s.cacheable && s.cache_valid && same_observation(s.cached_sample, sample)) {
        payload = s.cached_payload;
        reused = true;
      } else {
        for (size_t offset = 0; offset < Sensor::payload_pool_size; ++offset) {
          const size_t at = (s.payload_cursor + offset) % Sensor::payload_pool_size;
          if (s.payload_pool[at].unique()) {
            std::atomic_thread_fence(std::memory_order_acquire);
            writable = s.payload_pool[at].get();
            payload = s.payload_pool[at];
            s.payload_cursor = (at + 1) % Sensor::payload_pool_size;
            break;
          }
        }
        if (!writable) {
          ok = false;
          payload_pressure = true;
          ++s.payload_misses;
        } else {
          previous_data_capacity = writable->data.capacity();
          previous_beam_capacity = writable->beam_data.capacity();
          auto &data = writable->data;
          auto &beam_data = writable->beam_data;
          beam_data.clear();
          s.others.clear();
          if (sample.bodies) for (const auto& body : sample.bodies->states)
            if (body.body_returns && body.key.id != sample.key.id)
              s.others.push_back({int(body.key.id), body.position, 0.3});
          if (!gpu) {
            if (sample.scene) s.cpu->setScene(sample.scene);
            if(!s.publish_beams)s.cpu->scanInto(sample.position,sample.orientation,s.others,s.with_bodies,&data);
            else {
              s.cpu->scanBeamsInto(sample.position,sample.orientation,s.others,&s.beams);
              const auto& scan=s.beams;
              const size_t point_stride=s.with_bodies?16:12,beam_stride=s.with_bodies?36:32;
              data.clear();beam_data.resize(scan.size()*beam_stride);size_t at=0;
              float origin[3];for(int k=0;k<3;++k)origin[k]=scan.origin[k];
              for(size_t i=0;i<scan.size();++i) {
                float record[8];for(int k=0;k<3;++k){record[k]=origin[k];record[3+k]=scan.direction[i][k];}record[6]=scan.range[i];record[7]=scan.hit[i]?1.f:0.f;
                std::memcpy(beam_data.data()+at,record,32);if(s.with_bodies)std::memcpy(beam_data.data()+at+32,&scan.vehicle_id[i],4);at+=beam_stride;
                if(scan.hit[i]){const auto p=scan.point(i);float xyz[3]={float(p.x()),float(p.y()),float(p.z())};const auto old=data.size();data.resize(old+point_stride);std::memcpy(data.data()+old,xyz,12);if(s.with_bodies)std::memcpy(data.data()+old+12,&scan.vehicle_id[i],4);}
              }
            }
          }
          else {
            const double position[3] = {sample.position.x(), sample.position.y(), sample.position.z()};
            const double orientation[4] = {sample.orientation.w(), sample.orientation.x(),
                                           sample.orientation.y(), sample.orientation.z()};
            const auto grow = [](void *context, size_t bytes) -> void * {
              auto &output = *static_cast<std::vector<uint8_t> *>(context);
              try { output.resize(bytes); return output.data(); }
              catch (...) { return nullptr; }
            };
            char error[512] = {};
            if (gpu_api_->scan(s.gpu_scene->renderer, position, orientation,
                  double(sample.stamp) * 1e-9, &s.gpu_config, grow, &data, error, sizeof(error)) != 0)
              throw std::runtime_error(std::string("GPU module scan failed: ") + error + "; no CPU fallback");
            if (data.size() % 16)
              throw std::runtime_error("GPU module payload is not XYZ/intensity records; no CPU fallback");
            if (!s.others.empty()) {
              s.gpu_static_points.clear();
              s.gpu_static_points.reserve(data.size() / 16);
              for (size_t at = 0; at < data.size(); at += 16) {
                float xyz[3]; std::memcpy(xyz, data.data() + at, 12);
                s.gpu_static_points.push_back({{xyz[0], xyz[1], xyz[2]}, -1});
              }
              xgc2_math::scene::mergeGpuBodyCloud(s.body_config, sample.position, sample.orientation,
                  s.others, s.gpu_static_points, &s.gpu_body_points, &s.body_scratch);
              data.resize(s.gpu_body_points.size() * 16);
              for (size_t i = 0; i < s.gpu_body_points.size(); ++i) {
                const auto& point = s.gpu_body_points[i];
                const float xyz[4] = {float(point.point.x()), float(point.point.y()), float(point.point.z()), 0};
                std::memcpy(data.data() + i * 16, xyz, 16);
                if (s.with_bodies) {
                  const int32_t id = point.vehicle_id;
                  std::memcpy(data.data() + i * 16 + 12, &id, 4);
                }
              }
            } else if (s.with_bodies) {
              const int32_t id = -1;
              for (size_t at = 12; at < data.size(); at += 16) std::memcpy(data.data() + at, &id, 4);
            }
          }
          if (s.cacheable) {
            s.cached_sample = sample;
            s.cached_sample.bodies.reset();
            s.cached_payload = payload;
            s.cache_valid = true;
          }
        }
      }
    } catch (const std::exception &error) {
      ok = false;
      std::lock_guard<std::mutex> l(s.mutex);
      s.error = error.what();
      ++s.errors;
    }
    if (writable) {
      if (writable->data.capacity() != previous_data_capacity) ++s.payload_grows;
      if (writable->beam_data.capacity() != previous_beam_capacity) ++s.payload_grows;
    }
    // Raycasting is done. Metadata/publication and the geometry cache must not
    // retain the world frame or compete with later snapshots for pool slots.
    sample.bodies.reset();
    bool again = false;
    {
      std::lock_guard<std::mutex> l(s.mutex);
      s.busy = false;
      auto e = s.entity.lock();
      if (e && e->alive && e->enabled && e->generation == sample.key.generation &&
          (ok || payload_pressure)) {
        const bool output_pressure = ok && s.has_completed;
        if (output_pressure)
          ++s.misses;
        if (ok) {
          s.completed = sample;
          s.completed_payload = std::move(payload);
          s.has_completed = true;
          ++s.scans;
          if (reused) ++s.cache_hits;
          else ++s.computed_scans;
        } else ++s.misses;
        const auto elapsed =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                Clock::now() - sample.submitted)
                .count();
        s.latency_ns = elapsed;
        if (elapsed > s.max_latency_ns)
          s.max_latency_ns = elapsed;
        if (s.observed_generation != sample.key.generation) {
          s.observed_generation = sample.key.generation;
          s.last_sample_stamp = 0;
          s.observed_period_ns = 0;
          s.cadence = AdaptiveCadence(s.period);
          s.previous_misses = s.misses.load();
        }
        if (ok && s.last_sample_stamp && sample.stamp > s.last_sample_stamp) {
          const auto interval = sample.stamp - s.last_sample_stamp;
          const auto previous = s.observed_period_ns.load();
          s.observed_period_ns = previous ? previous - previous / 8 + interval / 8 : interval;
        }
        if (ok) s.last_sample_stamp = sample.stamp;
        const auto misses = s.misses.load();
        const auto old_period = s.effective_period_ns.load();
        const auto wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            Clock::now().time_since_epoch()).count();
        const auto new_period = s.cadence.observe(elapsed,
            output_pressure || payload_pressure || misses > s.previous_misses, wall_ns);
        s.previous_misses = misses;
        s.effective_period_ns = new_period;
        if (new_period > old_period) ++s.rate_degradations;
        else if (new_period < old_period) ++s.rate_recoveries;
      } else
        ++s.misses;
      again = s.has_pending;
    }
    if (again) enqueue(selected);
  }
  if (gpu) {
    for (const auto &entry : gpu_scenes_)
      if (entry.second->renderer) {
        gpu_api_->destroy(entry.second->renderer);
        entry.second->renderer = nullptr;
      }
  }
}
bool Sensors::take_shared(const std::shared_ptr<Sensor> &s, Sample &p,
                          std::shared_ptr<const SensorPayload> &payload) {
  std::lock_guard<std::mutex> l(s->mutex);
  if (!s->has_completed)
    return false;
  auto e = s->entity.lock();
  s->has_completed = false;
  if (!e || !e->alive || !e->enabled ||
      e->generation != s->completed.key.generation) {
    s->completed_payload.reset();
    ++s->misses;
    return false;
  }
  p = s->completed;
  payload = std::move(s->completed_payload);
  ++s->published;
  return true;
}
bool Sensors::take(const std::shared_ptr<Sensor> &s, Sample &p,
                   std::vector<uint8_t> &out, std::vector<uint8_t> *beams) {
  std::shared_ptr<const SensorPayload> payload;
  if (!take_shared(s, p, payload)) return false;
  out.assign(payload->data.begin(), payload->data.end());
  if (beams) beams->assign(payload->beam_data.begin(), payload->beam_data.end());
  return true;
}
Json Sensors::status() const {
  Json j = Json::array();
  std::lock_guard<std::mutex> l(mutex_);
  for (const auto &weak : sensors_)
    if (auto s = weak.lock()) {
      auto e = s->entity.lock();
      if (!e || !e->alive)
        continue;
      std::lock_guard<std::mutex> sl(s->mutex);
      const auto source_period = source_period_ns_.load();
      const auto effective_period = std::max(source_period, s->effective_period_ns.load());
      const auto observed_period = s->observed_period_ns.load();
      j.push_back({{"id", e->id},
                   {"generation", e->generation.load()},
                   {"scans", s->scans.load()},
                   {"misses", s->misses.load()},
                   {"throttled_samples", s->throttled.load()},
                   {"requested_rate_hz", 1e9 / double(s->period)},
                   {"effective_rate_hz", 1e9 / double(effective_period)},
                   {"source_rate_hz", source_period ? 1e9 / double(source_period) : 0.0},
                   {"observed_rate_hz", observed_period ? 1e9 / double(observed_period) : 0.0},
                   {"cache_hits", s->cache_hits.load()},
                   {"computed_scans", s->computed_scans.load()},
                   {"published", s->published.load()},
                   {"rate_degradations", s->rate_degradations.load()},
                   {"rate_recoveries", s->rate_recoveries.load()},
                   {"payload_slots", Sensor::payload_pool_size},
                   {"payload_grows", s->payload_grows.load()},
                   {"payload_misses", s->payload_misses.load()},
                   {"latency_ns", s->latency_ns.load()},
                   {"max_latency_ns", s->max_latency_ns.load()},
                   {"errors", s->errors.load()},
                   {"error", s->error},
                   {"backend", s->gpu ? "gpu" : "cpu"}});
    }
  return j;
}
void Sensors::stop() {
  {
    std::unique_lock<std::mutex> lock(mutex_);
    stopping_ = true;
    wake_.notify_all();
    if (gpu_creation_running_ &&
        !wake_.wait_until(lock, gpu_initialization_deadline_, [&] { return !gpu_creation_running_; }))
      initialization_expired(initialization_timeout_);
  }
  wake_.notify_all();
  cpu_wake_.notify_all();
  for (auto &t : cpu_threads_)
    if (t.joinable())
      t.join();
  if (gpu_thread_.joinable())
    gpu_thread_.join();
  std::lock_guard<std::mutex> l(mutex_);
  cpu_ready_ = {};
  gpu_ready_ = {};
  for (const auto &weak : sensors_)
    if (auto sensor = weak.lock()) {
      std::lock_guard<std::mutex> sl(sensor->mutex);
      sensor->has_pending = false;
      sensor->queued = false;
      sensor->pending.bodies.reset();
      sensor->completed.bodies.reset();
      sensor->cached_sample.bodies.reset();
      sensor->completed_payload.reset();
      sensor->cached_payload.reset();
      sensor->has_completed = false;
    }
}
} // namespace xsim
