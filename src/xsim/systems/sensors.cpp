#include "sensors.hpp"
#include <xgc2_math/scene/scene_conversion.hpp>
#include <cstring>
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
      a.scene_version != b.scene_version ||
      !(a.position.array() == b.position.array()).all() ||
      !(a.orientation.coeffs().array() == b.orientation.coeffs().array()).all())
    return false;
  return a.body_observation == b.body_observation &&
      (!a.body_observation || (a.geometry_revision &&
                              a.geometry_revision == b.geometry_revision));
}
} // namespace
Sensors::Sensors(const Json &j, unsigned workers) : workers_(workers) {
  if (!workers)
    throw std::invalid_argument("sensor_workers must be positive");
  spacing_ = j.value("surface_spacing", 0.1);
  buried_ = j.value("keep_buried", false);
  if(j.contains("document")) {
    std::vector<xgc2_math::scene::SceneObstacleDescription> descriptions;
    const auto pose=[](const Json& p){xgc2_math::Pose3 r;
      r.position=v3(p.value("position",Json::array({0,0,0})));
      auto q=p.value("orientation",Json::array({0,0,0,1}));
      r.orientation=Eigen::Quaterniond(q[3].get<double>(),q[0].get<double>(),q[1].get<double>(),q[2].get<double>()).normalized();return r;};
    for(const auto& o:j.at("document").at("obstacles")) {
      xgc2_math::scene::SceneObstacleDescription d;d.id=o.at("id");d.pose=pose(o.value("pose",Json::object()));
      for(const auto& part:o.at("parts")) {
        const auto& g=part.at("geometry");xgc2_math::scene::ScenePartDescription p;p.type=g.at("type");p.pose=pose(part.value("pose",Json::object()));
        if(g.contains("size"))p.size=v3(g.at("size"));p.radius=g.value("radius",0.0);p.height=g.value("height",0.0);
        if(g.contains("vertices"))for(const auto& v:g.at("vertices"))p.vertices.push_back(v3(v));d.parts.push_back(p);
      }descriptions.push_back(std::move(d));
    }obstacles_=xgc2_math::scene::toObstacles(descriptions);
  }
  for (const auto &o : j.value("obstacles", Json::array())) {
    const auto type = o.at("type").get<std::string>();
    auto pos = v3(o.at("position"));
    auto q = o.contains("orientation") ? quat(o.at("orientation"))
                                       : Eigen::Quaterniond::Identity();
    using O = xgc2_math::scene::Obstacle;
    if (type == "box")
      obstacles_.push_back(O::box(pos, v3(o.at("size")), q));
    else if (type == "sphere")
      obstacles_.push_back(O::sphere(pos, o.at("radius")));
    else if (type == "cylinder")
      obstacles_.push_back(O::cylinder(pos, o.at("radius"), o.at("height"), q));
    else if (type == "capsule")
      obstacles_.push_back(O::capsule(pos, o.at("radius"), o.at("height"), q));
    else if (type == "convex") {
      std::vector<Eigen::Vector3d> v;
      for (auto &x : o.at("vertices"))
        v.push_back(v3(x));
      obstacles_.push_back(O::convexHull(v, pos, q));
    } else
      throw std::invalid_argument("unknown obstacle type");
  }
  // One immutable compiled geometry/index for this world, never one map per
  // robot.
  scene_ = std::make_shared<xgc2_math::scene::LidarScene>(obstacles_, spacing_,
                                                          buried_);
}
std::vector<uint8_t> Sensors::reference_cloud() const {
  const auto& points=scene_->samples();
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
    if(c.mode==xgc2_math::scene::SensorConfig::kPenetrating && (c.surface_spacing!=spacing_ || c.penetrating_keep_buried!=buried_)) {
      const auto key=std::make_pair(c.surface_spacing,c.penetrating_keep_buried);
      std::lock_guard<std::mutex> l(mutex_);auto& scene=sampled_scenes_[key];
      if(!scene)scene=std::make_shared<xgc2_math::scene::LidarScene>(obstacles_,key.first,key.second);
      s->cpu->setScene(scene);
    } else s->cpu->setScene(scene_);
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
    }
    s->gpu_scene = group;
  }
  sensors_.erase(
      std::remove_if(sensors_.begin(), sensors_.end(),
                     [](const auto &weak) { return weak.expired(); }),
      sensors_.end());
  sensors_.push_back(s);
  if (s->gpu) {
    if (!gpu_thread_.joinable()) {
      gpu_thread_ = std::thread([this] { work(true); });
    }
    wake_.notify_all();
    wake_.wait(lock, [&] { return s->gpu_scene->initialized || stopping_; });
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
          const auto limit = std::numeric_limits<int64_t>::max();
          const auto candidate = s->last_submitted_stamp > limit - interval
              ? limit : s->last_submitted_stamp + interval;
          s->next = std::min(s->next, candidate);
        }
      }
    }
  }
  if (v.stamp < s->next_requested) return;
  if (!s->next_requested) s->next_requested = v.stamp;
  const auto due = uint64_t((v.stamp - s->next_requested) / s->period) + 1;
  s->next_requested = observation_deadline(s->next_requested, v.stamp, s->period);
  if (v.stamp < s->next) {
    s->throttled += due;
    return;
  }
  // Account separately for deliberately lower observation cadence. The input
  // frame cadence is an upper bound, even while the configured Hz stays intact.
  if (due > 1) s->throttled += due - 1;
  const auto period = std::max(s->effective_period_ns.load(), source_period_ns_.load());
  s->next = observation_deadline(s->next, v.stamp, period);
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
  wake_.notify_all();
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
  size_t cursor = 0;
  for (;;) {
    std::shared_ptr<Sensor> selected;
    Sample sample;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      if (stopping_)
        break;
      if (gpu) {
        // Geometry policies share one index/context each; robots do not own maps.
        for (const auto &entry : gpu_scenes_) {
          auto &group = *entry.second;
          if (group.initialized) continue;
          lock.unlock();
          char error[512] = {};
          auto *renderer = gpu_api_->create(group.points.data(), group.points.size() / 3,
                                            &group.initial, error, sizeof(error));
          lock.lock();
          group.renderer = renderer;
          group.error = renderer ? "" : std::string("GPU module create failed: ") + error;
          group.initialized = true;
          std::vector<float>().swap(group.points);
          wake_.notify_all();
        }
      }
      for (size_t n = 0; n < sensors_.size(); ++n) {
        auto s = sensors_[(cursor + n) % sensors_.size()].lock();
        if (!s || s->gpu != gpu)
          continue;
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
          cursor = (cursor + n + 1) % sensors_.size();
          break;
        }
      }
      if (!selected) {
        wake_.wait_for(lock, std::chrono::milliseconds(2));
        continue;
      }
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
    }
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
    std::lock_guard<std::mutex> l(mutex_);
    stopping_ = true;
  }
  wake_.notify_all();
  for (auto &t : cpu_threads_)
    if (t.joinable())
      t.join();
  if (gpu_thread_.joinable())
    gpu_thread_.join();
  std::lock_guard<std::mutex> l(mutex_);
  for (const auto &weak : sensors_)
    if (auto sensor = weak.lock()) {
      std::lock_guard<std::mutex> sl(sensor->mutex);
      sensor->has_pending = false;
      sensor->pending.bodies.reset();
      sensor->completed.bodies.reset();
      sensor->cached_sample.bodies.reset();
      sensor->completed_payload.reset();
      sensor->cached_payload.reset();
      sensor->has_completed = false;
    }
}
} // namespace xsim
