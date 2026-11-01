#include "scene_fixture.hpp"
#include "systems/sensors.hpp"
#include <atomic>
#include <cassert>
#include <cstring>
#include <limits>
#include <iostream>
#include <string>
#include <thread>

using namespace xsim;
namespace {
constexpr int64_t epoch = 1700000000000000000LL;

std::shared_ptr<Entity> entity(uint64_t id, const char *name, bool body_returns = false) {
  auto e = std::make_shared<Entity>(parse_entity({{"name", name}, {"kind", "scout"}, {"body_returns", body_returns}}));
  e->id = id;
  e->alive = e->enabled = true;
  e->generation = 1;
  return e;
}
State state(const std::shared_ptr<Entity> &e, int64_t stamp) {
  State s;
  s.entity = e;
  s.key = {e->id, e->generation};
  s.enabled = true;
  s.body_returns = e->config.body_returns;
  s.stamp = stamp;
  return s;
}
Sample receive(const std::shared_ptr<Sensor> &sensor, std::vector<uint8_t> &cloud,
               std::vector<uint8_t> &beams) {
  const auto deadline = Clock::now() + std::chrono::seconds(3);
  Sample sample;
  while (!Sensors::take(sensor, sample, cloud, &beams)) {
    if (Clock::now() >= deadline) {
      std::cerr << "receive timeout scans=" << sensor->scans.load()
                << " misses=" << sensor->misses.load()
                << " errors=" << sensor->errors.load()
                << " computed=" << sensor->computed_scans.load()
                << " error=" << sensor->error << '\n';
      assert(false);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return sample;
}
std::shared_ptr<const SensorPayload> receive_shared(const std::shared_ptr<Sensor> &sensor,
                                                   Sample &sample) {
  const auto deadline = Clock::now() + std::chrono::seconds(3);
  std::shared_ptr<const SensorPayload> payload;
  while (!Sensors::take_shared(sensor, sample, payload)) {
    assert(Clock::now() < deadline);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return payload;
}
void cadence_checks() {
  // A 100 Hz frame source can deliver 30 Hz without drifting down to 25 Hz.
  int64_t next = 0;
  unsigned accepted = 0;
  for (int k = 0; k < 200; ++k) {
    const auto stamp = epoch + k * 10000000LL;
    if (stamp >= next) {
      ++accepted;
      next = observation_deadline(next, stamp, 33333333);
    }
  }
  assert(accepted == 60);
  const auto limit = std::numeric_limits<int64_t>::max();
  assert(observation_deadline(limit - 10, limit - 5, 20) == limit);

  AdaptiveCadence cadence(1000000);
  assert(cadence.observe(100000, true, 0) == 1250000);
  const auto backed_off = cadence.effective;
  cadence.recover(AdaptiveCadence::recovery_window_ns);
  assert(cadence.effective < backed_off && cadence.effective >= cadence.period);
  assert(cadence.observe(8000000, false, 500000000) >= cadence.latency);
  AdaptiveCadence bounded(1000000);
  for (int k = 0; k < 200; ++k) bounded.observe(100000, true, k * 1000000LL);
  assert(bounded.effective == bounded.ceiling && bounded.ceiling == 1000000000);
  for (int k = 1; k <= 200; ++k)
    bounded.recover(199000000 + k * AdaptiveCadence::recovery_window_ns);
  assert(bounded.effective == bounded.period);
  AdaptiveCadence extreme(limit / 2 + 1);
  for (int k = 0; k < 8; ++k) extreme.observe(1, true, k);
  assert(extreme.effective == limit);

  const int64_t origin = epoch + 123;
  const int64_t scan_period = 33333333;
  const auto aligned = observation_deadline(0, epoch, scan_period, &origin);
  assert(aligned > epoch && (aligned - origin) % scan_period == 0);
  const auto late = aligned + 5 * scan_period + 10;
  const auto after = observation_deadline(aligned, late, scan_period, &origin);
  assert(after > late && after - late <= scan_period);
  assert((after - origin) % scan_period == 0);
  const auto widened = observation_deadline(after, late, scan_period * 2, &origin);
  assert((widened - origin) % (scan_period * 2) == 0);
  const int64_t other_epoch = epoch + 999999;
  const auto same_phase = observation_deadline(0, epoch, scan_period, &origin);
  const auto other_start = observation_deadline(0, epoch, scan_period, &origin);
  assert(same_phase == other_start);
  assert(observation_deadline(0, other_epoch, scan_period, &origin) !=
         observation_deadline(0, other_epoch, scan_period));
  const int64_t near = limit - 5;
  const int64_t near_origin = near - 3;
  assert(observation_deadline(0, near, 20, &near_origin) == limit);
  assert(observation_deadline(0, epoch + 10000000, 33333333, nullptr) ==
         observation_deadline(0, epoch + 10000000, 33333333));
}

// Cross-thread boundary only: the worker publishes cadence.observe through
// effective_period_ns while the sensor mutex is held. It does not write the
// producer's next. No product hook and no slowed scan.
int64_t inject_observed_period(const std::shared_ptr<Sensor> &sensor, bool overloaded) {
  std::lock_guard<std::mutex> lock(sensor->mutex);
  const auto wall = std::chrono::duration_cast<std::chrono::nanoseconds>(
      Clock::now().time_since_epoch()).count();
  if (!overloaded) sensor->cadence.recovery_at = 0;
  const auto updated = sensor->cadence.observe(1000, overloaded, wall);
  sensor->effective_period_ns = updated;
  return updated;
}

void phased_worker_period_boundary() {
  const Json scene = sceneFixture({{"obstacles", Json::array({{{"type", "box"},
      {"position", {4, 0, 0}}, {"size", {1, 2, 2}}}})}});
  const Json config = {{"backend", "cpu"}, {"h_fov_deg", 20}, {"v_fov_deg", 10},
      {"h_res", 8}, {"v_res", 4}, {"rate_hz", 10}};
  std::vector<uint8_t> cloud, beams;
  const int64_t phase = epoch;

  {
    Sensors phased(scene, 1, std::chrono::seconds(10), &phase);
    auto robot = entity(11, "phased_backoff");
    robot->sensor = phased.prepare(robot, config);
    State sample_state = state(robot, epoch + 1000);
    phased.submit(sample_state);
    auto sample = receive(robot->sensor, cloud, beams);
    assert(sample.stamp == sample_state.stamp && robot->sensor->scans == 1);
    assert(robot->sensor->next > sample_state.stamp);
    assert((robot->sensor->next - phase) % robot->sensor->period == 0);
    const auto previous_deadline = robot->sensor->next;
    const auto backed = inject_observed_period(robot->sensor, true);
    assert(backed > robot->sensor->period && robot->sensor->next == previous_deadline);
    const auto backed_deadline = observation_deadline(
        robot->sensor->next, robot->sensor->last_submitted_stamp, backed, &phase);
    assert(backed_deadline > previous_deadline);
    const auto scans = robot->sensor->scans.load();
    const auto throttled = robot->sensor->throttled.load();
    sample_state.stamp = previous_deadline;
    phased.submit(sample_state);
    assert(robot->sensor->scans == scans && robot->sensor->throttled > throttled);
  }

  {
    Sensors phased(scene, 1, std::chrono::seconds(10), &phase);
    auto robot = entity(14, "phased_backoff_stamp");
    robot->sensor = phased.prepare(robot, config);
    State sample_state = state(robot, epoch + 1000);
    phased.submit(sample_state);
    auto sample = receive(robot->sensor, cloud, beams);
    const auto backed = inject_observed_period(robot->sensor, true);
    const auto backed_deadline = observation_deadline(
        robot->sensor->next, robot->sensor->last_submitted_stamp, backed, &phase);
    const auto scans = robot->sensor->scans.load();
    sample_state.stamp = backed_deadline;
    phased.submit(sample_state);
    sample = receive(robot->sensor, cloud, beams);
    assert(sample.stamp == backed_deadline && robot->sensor->scans == scans + 1);
  }

  {
    Sensors phased(scene, 1, std::chrono::seconds(10), &phase);
    auto robot = entity(12, "phased_recovery");
    robot->sensor = phased.prepare(robot, config);
    State sample_state = state(robot, epoch + 1000);
    phased.submit(sample_state);
    auto sample = receive(robot->sensor, cloud, beams);
    assert(sample.stamp == sample_state.stamp);
    const auto backed = inject_observed_period(robot->sensor, true);
    const auto longer_deadline = observation_deadline(
        robot->sensor->next, robot->sensor->last_submitted_stamp, backed, &phase);
    {
      // The producer had already stored this longer deadline. Recovery below
      // only publishes the shorter effective period.
      std::lock_guard<std::mutex> lock(robot->sensor->mutex);
      robot->sensor->next = longer_deadline;
    }
    const auto recovered = inject_observed_period(robot->sensor, false);
    assert(recovered < backed && robot->sensor->next == longer_deadline);
    const auto recovered_deadline = observation_deadline(
        robot->sensor->next, robot->sensor->last_submitted_stamp, recovered, &phase);
    assert(recovered_deadline < longer_deadline);
    const auto scans = robot->sensor->scans.load();
    sample_state.stamp = recovered_deadline;
    phased.submit(sample_state);
    sample = receive(robot->sensor, cloud, beams);
    assert(sample.stamp == recovered_deadline && robot->sensor->scans == scans + 1);
  }

  {
    Sensors plain(scene, 1);
    auto robot = entity(13, "unphased_period");
    robot->sensor = plain.prepare(robot, config);
    State sample_state = state(robot, epoch + 1000);
    plain.submit(sample_state);
    auto sample = receive(robot->sensor, cloud, beams);
    assert(sample.stamp == sample_state.stamp);
    const auto previous_deadline = robot->sensor->next;
    const auto backed = inject_observed_period(robot->sensor, true);
    assert(backed > robot->sensor->period && robot->sensor->next == previous_deadline);
    sample_state.stamp = previous_deadline;
    plain.submit(sample_state);
    sample = receive(robot->sensor, cloud, beams);
    assert(sample.stamp == previous_deadline && robot->sensor->scans == 2);
  }
}
} // namespace

Json dynamic_document() {
  const Json box = {{"type", "box"}, {"size", {1.0, 4.0, 4.0}}};
  const Json identity = {{"position", {0.0, 0.0, 0.0}}, {"orientation", {0.0, 0.0, 0.0, 1.0}}};
  const Json local = {{"position", {0.0, 0.5, 0.0}}, {"orientation", {0.0, 0.0, 0.0, 1.0}}};
  Json document = {{"schema", "xgc2.scene.compiled.v1"}, {"id", "dynamic"}, {"frame", "world"},
                   {"ground", {{"mode", "none"}}}, {"geometries", Json::array({box})},
                   {"models", {{"mover", {{"parts", Json::array({{{"id", "body"}, {"pose", local}, {"geometryIndex", 0}}})}}},
                               {"post", {{"parts", Json::array({{{"id", "body"}, {"pose", identity}, {"geometryIndex", 0}}})}}}}},
                   {"obstacles", Json::array({
                       {{"id", "mover"}, {"name", "Mover"}, {"model", "mover"},
                        {"pose", {{"position", {4.0, 0.0, 0.0}}, {"orientation", {0.0, 0.0, 0.0, 1.0}}}},
                        {"motion", {{"type", "constant_twist"}}}},
                       {{"id", "post"}, {"name", "Post"}, {"model", "post"},
                        {"pose", {{"position", {0.0, 30.0, 0.0}}, {"orientation", {0.0, 0.0, 0.0, 1.0}}}},
                        {"motion", {{"type", "hold"}}}}})}};
  return {{"document", std::move(document)}};
}
bool bind_mover(Sensors &sensors) {
  return sensors.bind_dynamic_scene("store-epoch", 1, "world", {"mover"});
}
Json heavy_dynamic_document() {
  auto wrapped = dynamic_document();
  auto &document = wrapped["document"];
  for (int i = 0; i < 24; ++i) {
    const auto name = "hold-" + std::to_string(i);
    document["models"][name] = document["models"]["post"];
    document["obstacles"].push_back(
        {{"id", name}, {"name", name}, {"model", name},
         {"pose", {{"position", {0.0, 40.0 + i, 0.0}}, {"orientation", {0.0, 0.0, 0.0, 1.0}}}},
         {"motion", {{"type", "hold"}}}});
  }
  wrapped["surface_spacing"] = 0.05;
  return wrapped;
}
SceneObstaclePose pose_at(const char *id, double x) {
  SceneObstaclePose pose;
  pose.id = id;
  pose.pose.position = Eigen::Vector3d(x, 0, 0);
  pose.pose.orientation = Eigen::Quaterniond::Identity();
  return pose;
}
void wait_scene(Sensors &sensors, uint64_t version) {
  const auto deadline = Clock::now() + std::chrono::seconds(3);
  while (Clock::now() < deadline) {
    if (sensors.scene_ready() && sensors.scene_version() >= version) return;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  std::cerr << "scene wait timeout ready=" << sensors.scene_ready()
            << " version=" << sensors.scene_version() << " want>=" << version << '\n';
  assert(false);
}
float mean_x(const std::vector<uint8_t> &cloud) {
  assert(!cloud.empty() && cloud.size() % 12 == 0);
  double sum = 0;
  int count = 0;
  for (size_t i = 0; i < cloud.size(); i += 12) {
    float x = 0;
    std::memcpy(&x, cloud.data() + i, sizeof x);
    sum += x;
    ++count;
  }
  return float(sum / count);
}
void dynamic_scene_checks() {
  Sensors still(sceneFixture({{"obstacles", Json::array({{{"type", "box"}, {"position", {4, 0, 0}}, {"size", {1, 1, 1}}}})}}), 2);
  assert(!still.has_dynamic_obstacles() && still.scene_ready() && still.scene_version() == 1);
  assert(!still.bind_dynamic_scene("store-epoch", 1, "world", {"mover"}));
  assert(!still.update_scene_state("world", "store-epoch", 1, {pose_at("mover", 4)}));
  assert(still.scene_ready() && still.scene_version() == 1);

  Sensors unbound(dynamic_document(), 2);
  assert(unbound.has_dynamic_scene() && !unbound.scene_ready() && unbound.scene_version() == 0);
  assert(!unbound.update_scene_state("world", "first-state", 1, {pose_at("mover", 4)}));
  assert(!unbound.scene_ready() && unbound.scene_version() == 0);

  Sensors sensors(dynamic_document(), 2);
  auto robot = entity(7, "dyn");
  Json config = {{"backend", "cpu"}, {"h_fov_deg", 10}, {"v_fov_deg", 10}, {"h_res", 4}, {"v_res", 2},
                 {"rate_hz", 10}, {"range", 30}};
  robot->sensor = sensors.prepare(robot, config);
  assert(robot->sensor && !robot->sensor->gpu && !sensors.scene_ready());
  auto sample_state = state(robot, epoch);
  sensors.submit(sample_state);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  assert(robot->sensor->scans == 0);

  assert(!sensors.update_scene_state("map", "store-epoch", 1, {pose_at("mover", 4)}));
  assert(!sensors.update_scene_state("world", "other-epoch", 1, {pose_at("mover", 4)}));
  assert(!sensors.update_scene_state("world", "store-epoch", 0, {pose_at("mover", 4)}));
  assert(!sensors.update_scene_state("world", "store-epoch", 1, {}));
  auto degenerate = pose_at("mover", 4);
  degenerate.pose.orientation = Eigen::Quaterniond(0, 0, 0, 0);
  assert(!sensors.update_scene_state("world", "store-epoch", 1, {degenerate}));
  assert(!sensors.scene_ready() && sensors.scene_version() == 0);
  assert(!sensors.bind_dynamic_scene("store-epoch", 1, "world", {}));
  assert(!sensors.bind_dynamic_scene("store-epoch", 1, "map", {"mover"}));
  assert(bind_mover(sensors));
  assert(bind_mover(sensors));
  auto duplicated = pose_at("mover", 4);
  duplicated.pose.position.x() = std::numeric_limits<double>::quiet_NaN();
  assert(!sensors.update_scene_state("world", "store-epoch", 1, {pose_at("mover", 4), duplicated}));
  assert(!sensors.scene_ready() && sensors.scene_version() == 0);
  assert(!sensors.update_scene_state("world", "store-epoch", 1, {pose_at("mover", 4), pose_at("stray", 1)}));
  assert(!sensors.scene_ready() && sensors.scene_version() == 0);

  assert(sensors.update_scene_state("world", "store-epoch", 1, {pose_at("mover", 4)}));
  assert(robot->sensor->scans == 0);
  wait_scene(sensors, 1);
  assert(robot->sensor->scans == 0 && sensors.scene_version() == 1);
  assert(sensors.update_scene_state("world", "store-epoch", 1, {pose_at("mover", 4)}));
  assert(sensors.scene_version() == 1);

  std::vector<uint8_t> cloud, beams;
  sensors.submit(sample_state);
  auto first = receive(robot->sensor, cloud, beams);
  const auto near = mean_x(cloud);
  assert(first.scene && first.scene_version == 1);
  assert(near > 2.5f && near < 4.0f);
  assert(std::abs(first.scene->obstacles()[0].position.x() - 4) < 1e-9);
  assert(std::abs(first.scene->obstacles()[0].position.y() - 0.5) < 1e-9);
  assert(std::abs(first.scene->obstacles()[1].position.y() - 30) < 1e-9);
  const auto held = first.scene;
  const auto hits = robot->sensor->cache_hits.load();
  sample_state.stamp += 100000000;
  sensors.submit(sample_state);
  auto cached = receive(robot->sensor, cloud, beams);
  assert(cached.scene.get() == held.get() && cached.scene_version == 1);
  assert(robot->sensor->cache_hits > hits);
  assert(sensors.scene_version() == 1);

  assert(sensors.update_scene_state("world", "store-epoch", 1, {pose_at("mover", 8)}));
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  assert(sensors.scene_version() == 1);
  sample_state.stamp += 100000000;
  sensors.submit(sample_state);
  auto second = receive(robot->sensor, cloud, beams);
  assert(second.scene.get() == held.get() && second.scene_version == 1);
  assert(std::abs(mean_x(cloud) - near) < 0.2f);
  assert(sensors.scene_version() == 2);
  sample_state.stamp += 100000000;
  sensors.submit(sample_state);
  auto third = receive(robot->sensor, cloud, beams);
  const auto farther = mean_x(cloud);
  assert(third.scene && third.scene.get() != held.get() && third.scene_version == 2);
  assert(farther > near + 2.0f);
  assert(std::abs(held->obstacles()[0].position.x() - 4) < 1e-9);
  assert(std::abs(third.scene->obstacles()[1].position.y() - 30) < 1e-9);

  auto penetrating = entity(8, "pen");
  Json penetrating_config = config;
  penetrating_config["mode"] = "penetrating";
  penetrating_config["surface_spacing"] = 0.4;
  penetrating->sensor = sensors.prepare(penetrating, penetrating_config);
  auto penetrating_state = state(penetrating, sample_state.stamp + 100000000);
  sensors.submit(penetrating_state);
  auto penetrating_sample = receive(penetrating->sensor, cloud, beams);
  assert(penetrating_sample.scene && penetrating_sample.scene.get() != third.scene.get());
  assert(std::abs(penetrating_sample.scene->spacing() - 0.4) < 1e-12);
  assert(std::abs(penetrating_sample.scene->obstacles()[0].position.x() - 8) < 1e-9);
  const auto prepared_policy = penetrating->sensor->cpu->scene();
  assert(sensors.update_scene_state("world", "store-epoch", 1, {pose_at("mover", 5)}));
  penetrating_state.stamp += 100000000;
  sensors.submit(penetrating_state);
  auto claimed = receive(penetrating->sensor, cloud, beams);
  assert(claimed.scene.get() == penetrating_sample.scene.get());
  assert(std::abs(claimed.scene->obstacles()[0].position.x() - 8) < 1e-9);
  assert(sensors.scene_version() == 3);
  penetrating_state.stamp += 100000000;
  sensors.submit(penetrating_state);
  auto moved_penetrating = receive(penetrating->sensor, cloud, beams);
  assert(moved_penetrating.scene.get() != penetrating_sample.scene.get());
  assert(std::abs(moved_penetrating.scene->obstacles()[0].position.x() - 5) < 1e-9);
  assert(std::abs(penetrating_sample.scene->obstacles()[0].position.x() - 8) < 1e-9);
  assert(prepared_policy.get() == penetrating_sample.scene.get());
  assert(std::abs(moved_penetrating.scene->obstacles()[1].position.y() - 30) < 1e-9);

  std::atomic<bool> stop{false};
  std::thread mover([&] {
    bool far = false;
    while (!stop.load()) {
      far = !far;
      sensors.update_scene_state("world", "store-epoch", 1, {pose_at("mover", far ? 6.0 : 4.0)});
    }
  });
  for (int i = 0; i < 8; ++i) {
    // A producer try_lock miss consumes no scene, so keep offering a later
    // stamp until this sensor actually publishes one paired sample.
    Sample taken;
    bool got = false;
    const auto deadline = Clock::now() + std::chrono::seconds(3);
    while (!got && Clock::now() < deadline) {
      sample_state.stamp += 2000000000;
      sensors.submit(sample_state);
      got = Sensors::take(robot->sensor, taken, cloud, &beams);
      if (!got) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    assert(got && taken.scene && taken.scene_version > 0);
    const auto face = float(taken.scene->obstacles()[0].position.x() - 0.5);
    assert(std::abs(mean_x(cloud) - face) < 0.75f);
    assert(std::abs(held->obstacles()[0].position.x() - 4) < 1e-9);
  }
  stop.store(true);
  mover.join();

  Sensors gpu(dynamic_document(), 2);
  auto gpu_robot = entity(9, "gpu");
  bool rejected = false;
  try {
    gpu_robot->sensor = gpu.prepare(gpu_robot, {{"backend", "gpu"}, {"h_res", 4}, {"v_res", 2}});
  } catch (const std::invalid_argument &error) {
    rejected = std::string(error.what()).find("does not support dynamic") != std::string::npos;
  }
  assert(rejected && !gpu.scene_ready());

  Sensors automatic(dynamic_document(), 2);
  auto auto_robot = entity(10, "auto");
  auto_robot->sensor = automatic.prepare(auto_robot, {{"backend", "auto"}, {"h_fov_deg", 10}, {"v_fov_deg", 10},
                                                      {"h_res", 4}, {"v_res", 2}, {"range", 30}});
  assert(auto_robot->sensor && !auto_robot->sensor->gpu);
  assert(bind_mover(automatic));
  assert(automatic.update_scene_state("world", "store-epoch", 1, {pose_at("mover", 4)}));
  wait_scene(automatic, 1);
  auto auto_state = state(auto_robot, epoch);
  automatic.submit(auto_state);
  auto auto_sample = receive(auto_robot->sensor, cloud, beams);
  assert(auto_sample.scene && !cloud.empty() && cloud.size() % 12 == 0);

  {
    Sensors live(heavy_dynamic_document(), 2);
    auto watcher = entity(30, "watch");
    Json tiny = {{"backend", "cpu"}, {"h_fov_deg", 10}, {"v_fov_deg", 10}, {"h_res", 4}, {"v_res", 2}, {"range", 40}};
    watcher->sensor = live.prepare(watcher, tiny);
    assert(bind_mover(live));
    std::atomic<bool> run{true};
    std::atomic<int> posted{0};
    std::thread flood([&] {
      double x = 4;
      while (run.load()) {
        x += 0.05;
        live.update_scene_state("world", "store-epoch", 1, {pose_at("mover", x)});
        posted.fetch_add(1, std::memory_order_relaxed);
      }
    });
    wait_scene(live, 1);
    assert(watcher->sensor->scans == 0 && posted.load() > 1);
    const auto pinned_version = live.scene_version();
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    assert(live.scene_ready() && live.scene_version() == pinned_version && watcher->sensor->scans == 0);
    run.store(false);
    flood.join();
  }

  {
    Sensors late(dynamic_document(), 2);
    auto robot = entity(31, "late");
    Json tiny = {{"backend", "cpu"}, {"h_fov_deg", 10}, {"v_fov_deg", 10}, {"h_res", 4}, {"v_res", 2}, {"range", 40}};
    robot->sensor = late.prepare(robot, tiny);
    assert(bind_mover(late));
    assert(late.update_scene_state("world", "store-epoch", 1, {pose_at("mover", 4)}));
    wait_scene(late, 1);
    assert(robot->sensor->scans == 0);
    Json pen_config = tiny;
    pen_config["mode"] = "penetrating";
    pen_config["surface_spacing"] = 0.4;
    auto pen = entity(32, "late_pen");
    pen->sensor = late.prepare(pen, pen_config);
    const auto policy_v1 = pen->sensor->cpu->scene();
    assert(policy_v1 && std::abs(policy_v1->obstacles()[0].position.x() - 4) < 1e-9);
    Json other_config = pen_config;
    other_config["surface_spacing"] = 0.25;
    auto other = entity(33, "late_other");
    other->sensor = late.prepare(other, other_config);
    const auto other_v1 = other->sensor->cpu->scene();
    assert(other_v1 && other_v1.get() != policy_v1.get());
    assert(std::abs(other_v1->obstacles()[0].position.x() - 4) < 1e-9);
    for (int i = 1; i <= 30; ++i)
      assert(late.update_scene_state("world", "store-epoch", 1, {pose_at("mover", 8.0 + i * 0.05)}));
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    assert(late.scene_version() == 1);
    assert(std::abs(policy_v1->obstacles()[0].position.x() - 4) < 1e-9);
    auto sample_state = state(robot, epoch);
    late.submit(sample_state);
    auto captured = receive(robot->sensor, cloud, beams);
    assert(captured.scene_version == 1 && std::abs(captured.scene->obstacles()[0].position.x() - 4) < 1e-9);
    assert(late.scene_version() == 2);
    assert(pen->sensor->cpu->scene().get() == policy_v1.get());
    sample_state.stamp += 100000000;
    late.submit(sample_state);
    auto installed = receive(robot->sensor, cloud, beams);
    assert(installed.scene_version == 2);
    assert(std::abs(installed.scene->obstacles()[0].position.x() - 9.5) < 1e-6);
    auto pen_again = entity(34, "late_pen_again");
    pen_again->sensor = late.prepare(pen_again, pen_config);
    const auto policy_v2 = pen_again->sensor->cpu->scene();
    assert(policy_v2.get() != policy_v1.get());
    assert(std::abs(policy_v2->obstacles()[0].position.x() - 9.5) < 1e-6);
    auto other_again = entity(35, "late_other_again");
    other_again->sensor = late.prepare(other_again, other_config);
    assert(other_again->sensor->cpu->scene().get() != other_v1.get());
    assert(std::abs(other_again->sensor->cpu->scene()->obstacles()[0].position.x() - 9.5) < 1e-6);
    assert(pen->sensor->cpu->scene().get() == policy_v1.get());
    assert(other->sensor->cpu->scene().get() == other_v1.get());
    const auto scans_before_fence = robot->sensor->scans.load();
    const auto fenced = late.scene_version();
    assert(!late.update_scene_state("world", "other-epoch", 1, {pose_at("mover", 1)}));
    assert(!late.scene_ready());
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    assert(!late.scene_ready() && late.scene_version() == fenced);
    assert(!late.bind_dynamic_scene("store-epoch", 1, "world", {"mover"}));
    assert(!late.update_scene_state("world", "store-epoch", 2, {pose_at("mover", 3)}));
    assert(!late.scene_ready() && late.scene_version() == fenced);
    assert(robot->sensor->scans == scans_before_fence);
  }

  {
    Sensors raced(dynamic_document(), 2);
    auto seed = entity(40, "seed");
    Json tiny = {{"backend", "cpu"}, {"h_fov_deg", 10}, {"v_fov_deg", 10}, {"h_res", 4}, {"v_res", 2}, {"range", 40}};
    seed->sensor = raced.prepare(seed, tiny);
    assert(bind_mover(raced));
    std::atomic<bool> run{true};
    std::thread flood([&] {
      double x = 1;
      while (run.load()) {
        x += 0.2;
        raced.update_scene_state("world", "store-epoch", 1, {pose_at("mover", x)});
      }
    });
    std::vector<std::shared_ptr<Sensor>> prepared;
    std::vector<std::shared_ptr<const xgc2_math::scene::LidarScene>> handles;
    std::vector<Eigen::Vector3d> poses;
    for (int i = 0; i < 6; ++i) {
      auto created = entity(50 + i, "prep");
      created->sensor = raced.prepare(created, tiny);
      auto scene = created->sensor->cpu->scene();
      assert(scene && std::abs(scene->obstacles()[0].position.y() - 0.5) < 1e-9);
      prepared.push_back(created->sensor);
      handles.push_back(scene);
      poses.push_back(scene->obstacles()[0].position);
    }
    Json pen_config = tiny;
    pen_config["mode"] = "penetrating";
    pen_config["surface_spacing"] = 0.4;
    auto pen = entity(70, "prep_pen");
    pen->sensor = raced.prepare(pen, pen_config);
    const auto pen_scene = pen->sensor->cpu->scene();
    assert(pen_scene && std::abs(pen_scene->spacing() - 0.4) < 1e-12);
    assert(std::abs(pen_scene->obstacles()[0].position.y() - 0.5) < 1e-9);
    const auto pen_pose = pen_scene->obstacles()[0].position;
    run.store(false);
    flood.join();
    assert(raced.update_scene_state("world", "store-epoch", 1, {pose_at("mover", 2)}));
    if (!raced.scene_ready()) wait_scene(raced, 1);
    for (size_t i = 0; i < handles.size(); ++i) {
      assert(prepared[i]->cpu->scene().get() == handles[i].get());
      assert((prepared[i]->cpu->scene()->obstacles()[0].position - poses[i]).norm() < 1e-9);
    }
    assert(pen->sensor->cpu->scene().get() == pen_scene.get());
    assert((pen_scene->obstacles()[0].position - pen_pose).norm() < 1e-9);
    assert(seed->sensor->scans == 0 && pen->sensor->scans == 0);
  }

  {
    Sensors missing(dynamic_document(), 2);
    auto robot = entity(83, "missing");
    Json tiny = {{"backend", "cpu"}, {"h_fov_deg", 10}, {"v_fov_deg", 10}, {"h_res", 4}, {"v_res", 2}, {"range", 40}};
    robot->sensor = missing.prepare(robot, tiny);
    assert(bind_mover(missing));
    assert(missing.update_scene_state("world", "store-epoch", 1, {pose_at("mover", 4)}));
    wait_scene(missing, 1);
    const auto installed = missing.scene_version();
    // Equal cardinality does not prove that every cold dynamic ID is present.
    // Reject before any due sample, then recover through the existing fence.
    assert(!missing.update_scene_state("world", "store-epoch", 1, {pose_at("stray", 1)}));
    assert(!missing.scene_ready() && missing.scene_version() == installed);
    assert(robot->sensor->scans == 0);
    assert(missing.update_scene_state("world", "store-epoch", 1, {pose_at("mover", 6)}));
    wait_scene(missing, installed + 1);
    assert(missing.scene_ready() && missing.scene_version() == installed + 1);
    assert(robot->sensor->scans == 0);
  }

  {
    Sensors again(dynamic_document(), 2);
    auto robot = entity(81, "again");
    Json tiny = {{"backend", "cpu"}, {"h_fov_deg", 10}, {"v_fov_deg", 10}, {"h_res", 4}, {"v_res", 2}, {"range", 40}};
    robot->sensor = again.prepare(robot, tiny);
    assert(bind_mover(again));
    assert(again.update_scene_state("world", "store-epoch", 1, {pose_at("mover", 4)}));
    wait_scene(again, 1);
    assert(again.update_scene_state("world", "store-epoch", 1, {pose_at("mover", 8)}));
    assert(again.update_scene_state("world", "store-epoch", 1, {pose_at("mover", 4)}));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    assert(again.scene_ready() && again.scene_version() == 1);
    auto sample_state = state(robot, epoch);
    again.submit(sample_state);
    auto kept = receive(robot->sensor, cloud, beams);
    assert(kept.scene && kept.scene_version == 1 && kept.stamp == epoch);
    assert(std::abs(kept.scene->obstacles()[0].position.x() - 4) < 1e-9);
    assert(again.scene_version() == 1);
    const auto hits = robot->sensor->cache_hits.load();
    sample_state.stamp += 100000000;
    again.submit(sample_state);
    auto cached = receive(robot->sensor, cloud, beams);
    assert(cached.scene.get() == kept.scene.get() && cached.scene_version == 1);
    assert(robot->sensor->cache_hits > hits && again.scene_version() == 1);
  }

  {
    // An accepted sample keeps the captured pointer, version, and stamp.
    // Identity invalidation blocks later submits and installs only.
    Sensors drain(dynamic_document(), 2);
    auto robot = entity(82, "drain");
    Json tiny = {{"backend", "cpu"}, {"h_fov_deg", 10}, {"v_fov_deg", 10}, {"h_res", 4}, {"v_res", 2}, {"range", 40}};
    robot->sensor = drain.prepare(robot, tiny);
    assert(bind_mover(drain));
    assert(drain.update_scene_state("world", "store-epoch", 1, {pose_at("mover", 4)}));
    wait_scene(drain, 1);
    auto sample_state = state(robot, epoch);
    drain.submit(sample_state);
    assert(!drain.update_scene_state("world", "other-epoch", 1, {pose_at("mover", 1)}));
    assert(!drain.scene_ready());
    auto kept = receive(robot->sensor, cloud, beams);
    assert(kept.scene && kept.scene_version == 1 && kept.stamp == epoch);
    assert(std::abs(kept.scene->obstacles()[0].position.x() - 4) < 1e-9);
    const auto scans = robot->sensor->scans.load();
    sample_state.stamp += 100000000;
    drain.submit(sample_state);
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    assert(robot->sensor->scans == scans && !drain.scene_ready());
    assert(!drain.update_scene_state("world", "store-epoch", 1, {pose_at("mover", 3)}));
    assert(!drain.bind_dynamic_scene("fresh-epoch", 1, "world", {"mover"}));
    assert(!drain.scene_ready() && drain.scene_version() == 1);
  }
}

void completion_queue_checks() {
  Sensors sensors(Json::object(), 1);
  auto robot = entity(91, "completion");
  const Json config{{"backend", "cpu"}, {"h_res", 4}, {"v_res", 2}, {"rate_hz", 10}};
  robot->sensor = sensors.prepare(robot, config);
  auto sensor = robot->sensor;
  auto input = state(robot, epoch);
  auto scan = [&](bool retained = true) {
    const auto previous = sensor->scans.load();
    sensors.submit(input);
    const auto deadline = Clock::now() + std::chrono::seconds(3);
    while (sensor->scans == previous) {
      assert(Clock::now() < deadline);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::lock_guard<std::mutex> lock(sensor->mutex);
    if (retained) assert(sensor->has_completed);
  };
  sensors.start_completions();
  scan();
  assert(!sensors.try_completion()); // A scan before late binding has no token.
  sensors.enable_completions(sensor);
  std::atomic<unsigned> notifications{0};
  sensors.bind_completion_wake([&] { ++notifications; });
  assert(notifications == 1); // Binding after a completed scan cannot lose its wake.
  assert(sensors.has_completion());
  auto token = sensors.wait_completion();
  assert(token == sensor);
  Sample sample;
  std::shared_ptr<const SensorPayload> payload;
  assert(Sensors::take_completion(token, sample, payload) && sample.stamp == epoch);
  assert(!sensors.has_completion());
  assert(!sensors.try_completion());
  std::atomic<bool> scan_waiting{false}, delivered{false};
  std::thread scan_consumer([&] {
    scan_waiting = true;
    const auto ready = sensors.wait_completion();
    Sample received;
    std::shared_ptr<const SensorPayload> bytes;
    assert(ready == sensor && Sensors::take_completion(ready, received, bytes));
    delivered = true;
  });
  while (!scan_waiting) std::this_thread::yield();
  input.stamp += 1000000000; scan(false);
  scan_consumer.join();
  assert(delivered);
  assert(notifications >= 2); // A new scan wakes the existing output owner.
  for (int k = 1; k <= 3; ++k) { input.stamp += 1000000000; scan(); }
  sensors.interrupt_completions();
  assert(!sensors.wait_completion());
  sensors.start_completions();
  token = sensors.wait_completion();
  assert(token == sensor);
  assert(Sensors::take_completion(token, sample, payload) && sample.stamp == input.stamp);
  assert(!sensors.try_completion()); // Latest overwrite did not enqueue three tokens.
  input.stamp += 1000000000; scan();
  token = sensors.wait_completion();
  robot->alive = false;
  assert(!Sensors::take_completion(token, sample, payload));
  { std::lock_guard<std::mutex> lock(sensor->mutex); assert(!sensor->completion_queued); }
  robot->alive = true;
  input.stamp += 1000000000; scan();
  token = sensors.wait_completion();
  assert(Sensors::take_completion(token, sample, payload)); // Invalid token did not strand the next result.
  sensors.bind_completion_wake({});
  const auto notifications_after_unbind = notifications.load();
  token.reset(); payload.reset();
  input.stamp += 1000000000; scan();
  assert(notifications == notifications_after_unbind);
  std::weak_ptr<Sensor> weak = sensor;
  robot->sensor.reset(); sensor.reset(); robot.reset();
  const auto retired_by = Clock::now() + std::chrono::seconds(3);
  while (!weak.expired()) { assert(Clock::now() < retired_by); std::this_thread::yield(); }
  assert(!sensors.try_completion()); // Queue never owns a retired sensor.
  std::atomic<bool> waiting{false}, returned{false};
  std::thread consumer([&] { waiting = true; assert(!sensors.wait_completion()); returned = true; });
  while (!waiting) std::this_thread::yield();
  sensors.interrupt_completions();
  consumer.join();
  assert(returned);
  sensors.stop();
}

int main() {
  completion_queue_checks();
  dynamic_scene_checks();
  cadence_checks();
  phased_worker_period_boundary();
  Sensors sensors(sceneFixture({{"obstacles", Json::array({{{"type", "box"},
      {"position", {4, 0, 0}}, {"size", {1, 2, 2}}}})}}), 2);
  auto source = entity(1, "source"), body = entity(2, "body", true);
  Json config = {{"backend", "cpu"}, {"h_fov_deg", 60}, {"v_fov_deg", 30},
      {"h_res", 32}, {"v_res", 16}, {"rate_hz", 10},
      {"world_bodies", true}, {"publish_beams", true}, {"translation", {0.5, 0, 0}}};
  source->sensor = sensors.prepare(source, config);
  Frame frame;
  frame.stamp = epoch;
  frame.states = {state(source, frame.stamp), state(body, frame.stamp)};
  frame.states[1].position.x() = 2;
  auto advance = [&] {
    frame.stamp += 100000000;
    for (auto &s : frame.states) s.stamp = frame.stamp;
    sensors.submit_frame(frame);
  };
  std::vector<uint8_t> cloud, beams;
  sensors.submit_frame(frame);
  auto sample = receive(source->sensor, cloud, beams);
  assert(sample.stamp == frame.stamp && sample.position.x() == .5);
  assert(!sample.bodies && sample.body_observation && sample.geometry_revision > 0);
  assert(!cloud.empty() && cloud.size() % 16 == 0);
  assert(beams.size() == 32 * 16 * 36);
  bool body_hit = false;
  for (size_t i = 12; i < cloud.size(); i += 16) {
    int32_t id;
    std::memcpy(&id, cloud.data() + i, sizeof id);
    body_hit |= id == 2;
  }
  assert(body_hit);
  const auto original_cloud = cloud, original_beams = beams;
  const auto computed = source->sensor->computed_scans.load();
  advance();
  sample = receive(source->sensor, cloud, beams);
  assert(sample.stamp == frame.stamp);
  assert(cloud == original_cloud && beams == original_beams);
  assert(source->sensor->cache_hits == 1 && source->sensor->computed_scans == computed);

  // Moving any observed body invalidates the observation cache without changing
  // point/beam layout or requiring a read from the live World.
  frame.states[1].position.y() = 10;
  advance();
  receive(source->sensor, cloud, beams);
  assert(cloud != original_cloud && source->sensor->computed_scans == computed + 1);
  assert(cloud.size() % 16 == 0 && beams.size() == original_beams.size());

  ++source->generation;
  frame.states[0].key.generation = source->generation;
  advance();
  sample = receive(source->sensor, cloud, beams);
  assert(sample.key.generation == 2 && source->sensor->computed_scans == computed + 2);
  advance();
  sample = receive(source->sensor, cloud, beams);
  assert(sample.key.generation == 2 && sample.stamp == frame.stamp);
  assert(source->sensor->cache_hits == 2);

  // The production overload retains the original immutable frame only while
  // scanning. Completed metadata and cached geometry never pin pool slots.
  auto shared_source = entity(5, "shared_source");
  shared_source->sensor = sensors.prepare(shared_source, config);
  auto mutable_shared = std::make_shared<Frame>(frame);
  mutable_shared->states[0] = state(shared_source, mutable_shared->stamp);
  mutable_shared->geometry_revision = 999;
  std::shared_ptr<const Frame> immutable = std::move(mutable_shared);
  sensors.submit_frame(immutable);
  const auto first_payload = receive_shared(shared_source->sensor, sample);
  const auto shared_growth = shared_source->sensor->payload_grows.load();
  assert(sample.geometry_revision > 0 && sample.body_observation && !sample.bodies);
  assert(immutable.use_count() == 1);
  {
    std::lock_guard<std::mutex> lock(shared_source->sensor->mutex);
    assert(!shared_source->sensor->pending.bodies);
    assert(!shared_source->sensor->completed.bodies);
    assert(!shared_source->sensor->cached_sample.bodies);
  }
  mutable_shared = std::make_shared<Frame>(*immutable);
  mutable_shared->stamp += 100000000;
  for (auto &s : mutable_shared->states) s.stamp = mutable_shared->stamp;
  immutable = std::move(mutable_shared);
  sensors.submit_frame(immutable);
  const auto cached_payload = receive_shared(shared_source->sensor, sample);
  const auto shared_cloud = cached_payload->data;
  assert(cached_payload.get() == first_payload.get());
  assert(shared_source->sensor->payload_grows == shared_growth);
  assert(shared_source->sensor->computed_scans == 1 && shared_source->sensor->cache_hits == 1);
  assert(immutable.use_count() == 1);
  mutable_shared = std::make_shared<Frame>(*immutable);
  mutable_shared->stamp += 100000000;
  ++mutable_shared->geometry_revision;
  mutable_shared->states[1].position.y() = 0;
  for (auto &s : mutable_shared->states) s.stamp = mutable_shared->stamp;
  immutable = std::move(mutable_shared);
  sensors.submit_frame(immutable);
  const auto moved_payload = receive_shared(shared_source->sensor, sample);
  assert(moved_payload.get() != first_payload.get());
  assert(shared_source->sensor->computed_scans == 2 && moved_payload->data != shared_cloud);
  assert(first_payload->data == shared_cloud);
  assert(immutable.use_count() == 1);
  // A stopped body-observing sensor must also release queued frame ownership.
  mutable_shared = std::make_shared<Frame>(*immutable);
  mutable_shared->stamp += 100000000;
  for (auto &s : mutable_shared->states) s.stamp = mutable_shared->stamp;
  immutable = std::move(mutable_shared);
  sensors.submit_frame(immutable);
  shared_source->enabled = false;
  const auto release_deadline = Clock::now() + std::chrono::seconds(3);
  while (immutable.use_count() > 1) {
    assert(Clock::now() < release_deadline);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  assert(immutable.use_count() == 1);

  // The producer never waits for a worker/output lock; a busy queue is measured.
  const auto misses = source->sensor->misses.load();
  {
    std::lock_guard<std::mutex> lock(source->sensor->mutex);
    advance();
  }
  assert(source->sensor->misses == misses + 1);

  auto noisy = entity(3, "noisy");
  config["noise_std"] = .1;
  config["world_bodies"] = false;
  config["publish_beams"] = false;
  noisy->sensor = sensors.prepare(noisy, config);
  assert(!noisy->sensor->cacheable);
  State n = state(noisy, frame.stamp);
  sensors.submit(n);
  receive(noisy->sensor, cloud, beams);
  const auto first_noise = cloud;
  n.stamp += 100000000;
  sensors.submit(n);
  receive(noisy->sensor, cloud, beams);
  assert(noisy->sensor->cache_hits == 0 && noisy->sensor->computed_scans == 2);
  assert(cloud != first_noise);
  bool rejected = false;
  try {
    sensors.prepare(noisy, {{"rate_hz", 1e-20}});
  } catch (const std::invalid_argument &) { rejected = true; }
  assert(rejected);

  // Consumers may retain all four immutable payloads. Further scans skip
  // rather than allocate or overwrite any held bytes, and resume after release.
  auto pooled = entity(6, "pooled_payload");
  pooled->sensor = sensors.prepare(pooled, config);
  State pool_state = state(pooled, n.stamp + 100000000);
  std::vector<std::shared_ptr<const SensorPayload>> held_payloads;
  std::vector<std::vector<uint8_t>> held_bytes;
  for (size_t k = 0; k < Sensor::payload_pool_size; ++k) {
    sensors.submit(pool_state);
    auto payload = receive_shared(pooled->sensor, sample);
    for (const auto &previous : held_payloads) assert(previous.get() != payload.get());
    held_bytes.push_back(payload->data);
    held_payloads.push_back(std::move(payload));
    pool_state.stamp += 200000000;
  }
  const auto payload_growth = pooled->sensor->payload_grows.load();
  const auto payload_misses = pooled->sensor->payload_misses.load();
  sensors.submit(pool_state);
  const auto pool_deadline = Clock::now() + std::chrono::seconds(3);
  while (pooled->sensor->payload_misses == payload_misses) {
    assert(Clock::now() < pool_deadline);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  {
    std::lock_guard<std::mutex> lock(pooled->sensor->mutex);
    assert(!pooled->sensor->has_completed);
  }
  assert(pooled->sensor->scans == Sensor::payload_pool_size);
  assert(pooled->sensor->payload_grows == payload_growth);
  for (size_t k = 0; k < held_payloads.size(); ++k) assert(held_payloads[k]->data == held_bytes[k]);
  const auto *const freed = held_payloads.front().get();
  held_payloads.front().reset();
  pool_state.stamp += pooled->sensor->effective_period_ns.load() + 200000000;
  sensors.submit(pool_state);
  const auto recovered_payload = receive_shared(pooled->sensor, sample);
  assert(recovered_payload.get() == freed);
  assert(pooled->sensor->payload_grows == payload_growth);
  for (size_t k = 1; k < held_payloads.size(); ++k) assert(held_payloads[k]->data == held_bytes[k]);

  // Actual completed-output pressure reaches a finite cadence ceiling. After
  // the consumer drains it, idle producer frames recover by wall time even
  // before eight further scans complete.
  auto pressured = entity(4, "pressured");
  pressured->sensor = sensors.prepare(pressured, config);
  State pressure = state(pressured, n.stamp);
  sensors.submit(pressure);
  receive(pressured->sensor, cloud, beams);
  for (int k = 0; k < 32 && pressured->sensor->effective_period_ns < 1000000000; ++k) {
    const auto count = pressured->sensor->scans.load();
    pressure.stamp += std::max<int64_t>(100000000, pressured->sensor->effective_period_ns) + 100000000;
    sensors.submit(pressure);
    const auto deadline = Clock::now() + std::chrono::seconds(3);
    while (pressured->sensor->scans == count) {
      assert(Clock::now() < deadline);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    // Completion increments scans before updating cadence; wait for the same
    // completion lock without consuming its output-pressure evidence.
    std::lock_guard<std::mutex> completion(pressured->sensor->mutex);
    assert(pressured->sensor->has_completed);
  }
  assert(pressured->sensor->effective_period_ns == 1000000000);
  receive(pressured->sensor, cloud, beams);
  const auto before_recovery = pressured->sensor->scans.load();
  const auto recover_until = Clock::now() + std::chrono::milliseconds(350);
  while (Clock::now() < recover_until) {
    pressure.stamp += 10000000;
    sensors.submit(pressure);
    Sensors::take(pressured->sensor, sample, cloud, &beams);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  assert(pressured->sensor->effective_period_ns < 1000000000);
  assert(pressured->sensor->rate_recoveries > 0);
  assert(pressured->sensor->scans - before_recovery < 8);

  // public_id bytes, not the numeric id World assigns after prepare.
  // Eight by four beams, two scans. noise_std=0 keeps the cache.
  {
    Sensors noise_sensors(sceneFixture({{"obstacles", Json::array({{{"type", "box"},
        {"position", {4, 0, 0}}, {"size", {1, 2, 2}}}})}}), 2);
    Json noise = {{"backend", "cpu"}, {"h_fov_deg", 60}, {"v_fov_deg", 30},
                  {"h_res", 8}, {"v_res", 4}, {"rate_hz", 10},
                  {"world_bodies", false}, {"publish_beams", false},
                  {"noise_std", 0.1}, {"seed", 7}};
    auto first = entity(0, "noise_a");
    auto second = entity(0, "noise_b");
    auto again = entity(0, "noise_a");
    assert(first->id == 0 && second->id == 0 && again->id == 0);
    first->sensor = noise_sensors.prepare(first, noise);
    second->sensor = noise_sensors.prepare(second, noise);
    again->sensor = noise_sensors.prepare(again, noise);
    first->id = 0x9ull;
    assert(!first->sensor->cacheable && !second->sensor->cacheable && !again->sensor->cacheable);
    Frame noise_frame;
    noise_frame.stamp = epoch;
    noise_frame.states = {state(first, noise_frame.stamp), state(second, noise_frame.stamp),
                          state(again, noise_frame.stamp)};
    std::vector<uint8_t> a, b, a2, noise_beams;
    noise_sensors.submit_frame(noise_frame);
    receive(first->sensor, a, noise_beams);
    receive(second->sensor, b, noise_beams);
    receive(again->sensor, a2, noise_beams);
    assert(a == a2);
    assert(a != b);
    noise_frame.stamp += 100000000;
    for (auto &s : noise_frame.states) s.stamp = noise_frame.stamp;
    noise_sensors.submit_frame(noise_frame);
    std::vector<uint8_t> a_next, a2_next;
    receive(first->sensor, a_next, noise_beams);
    receive(again->sensor, a2_next, noise_beams);
    assert(a_next == a2_next);
    assert(a_next != a);

    const std::string utf8 = std::string("id_") + "\xC2\xA9";
    auto utf = std::make_shared<Entity>(parse_entity({{"name", "utf_name"}, {"kind", "scout"}}), utf8);
    auto utf_again = std::make_shared<Entity>(parse_entity({{"name", "utf_name"}, {"kind", "scout"}}), utf8);
    auto utf_other = std::make_shared<Entity>(
        parse_entity({{"name", "utf_name"}, {"kind", "scout"}}), std::string("id_A"));
    utf->id = utf_again->id = utf_other->id = 0;
    utf->alive = utf_again->alive = utf_other->alive = true;
    utf->enabled = utf_again->enabled = utf_other->enabled = true;
    utf->generation = utf_again->generation = utf_other->generation = 1;
    utf->sensor = noise_sensors.prepare(utf, noise);
    utf_again->sensor = noise_sensors.prepare(utf_again, noise);
    utf_other->sensor = noise_sensors.prepare(utf_other, noise);
    utf->id = 7;
    Frame utf_frame;
    utf_frame.stamp = epoch;
    utf_frame.states = {state(utf, utf_frame.stamp), state(utf_again, utf_frame.stamp),
                        state(utf_other, utf_frame.stamp)};
    std::vector<uint8_t> u, u2, u3;
    noise_sensors.submit_frame(utf_frame);
    receive(utf->sensor, u, noise_beams);
    receive(utf_again->sensor, u2, noise_beams);
    receive(utf_other->sensor, u3, noise_beams);
    assert(u == u2);
    assert(u != u3);

    Json quiet = noise;
    quiet["noise_std"] = 0.0;
    auto still = entity(303, "noise_off");
    still->sensor = noise_sensors.prepare(still, quiet);
    assert(still->sensor->cacheable);
    Frame quiet_frame;
    quiet_frame.stamp = epoch;
    quiet_frame.states = {state(still, quiet_frame.stamp)};
    noise_sensors.submit_frame(quiet_frame);
    std::vector<uint8_t> q0;
    receive(still->sensor, q0, noise_beams);
    const auto quiet_computed = still->sensor->computed_scans.load();
    quiet_frame.stamp += 100000000;
    quiet_frame.states[0].stamp = quiet_frame.stamp;
    noise_sensors.submit_frame(quiet_frame);
    std::vector<uint8_t> q1;
    receive(still->sensor, q1, noise_beams);
    assert(q0 == q1);
    assert(still->sensor->cache_hits == 1);
    assert(still->sensor->computed_scans == quiet_computed);
    noise_sensors.stop();
  }

  const auto status = sensors.status();
  assert(status[0].at("requested_rate_hz") == 10.0);
  assert(status[0].at("cache_hits") == 2);
  assert(status[0].at("observed_rate_hz").get<double>() > 0);
  sensors.stop();
  std::cout << Json({{"result", "PASS"}, {"checks", {"absolute_sensor_cadence",
      "adaptive_backoff_recovery", "bounded_pressure_wall_recovery", "cached_cloud_beams", "same_frame_bodies",
      "motion_invalidates_cache", "generation_invalidates_cache", "nonblocking_producer",
      "shared_frame_release", "geometry_revision_cache",
      "shared_cached_payload", "bounded_payload_pool_recovery",
      "fresh_noise", "independent_noise_seed", "rate_boundary", "phased_worker_backoff",
      "phased_worker_recovery", "unphased_period_unchanged"}}, {"sensors", status}}).dump() << '\n';
}
