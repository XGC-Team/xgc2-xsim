#include "scene_fixture.hpp"
#include "systems/sensors.hpp"
#include <cassert>
#include <cstring>
#include <iostream>
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
    assert(Clock::now() < deadline);
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

int main() {
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
      "fresh_noise", "rate_boundary", "phased_worker_backoff", "phased_worker_recovery",
      "unphased_period_unchanged"}}, {"sensors", status}}).dump() << '\n';
}
