// Focused ROS-SDK proof: include the production implementation, not a copy of
// the lane loop. Link xsim_ros/xsim_sensors/xsim_world with the existing SDK.
// No ROS master or World thread is started by this fixture.
#include "../src/xsim/io/ros/runtime.cpp"
#include <cassert>
#include <map>

using namespace xsim;
namespace {
constexpr int64_t epoch = 1700000000000000000LL;

template <class Predicate> void until(Predicate ready) {
  const auto deadline = Clock::now() + std::chrono::seconds(5);
  while (!ready()) {
    assert(Clock::now() < deadline);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

std::shared_ptr<Frame> snapshot(
    int64_t stamp, const std::vector<std::shared_ptr<Entity>> &entities) {
  auto frame = std::make_shared<Frame>();
  frame->stamp = stamp;
  frame->revision = 7; // Deliberately identical across a roster/order change.
  for (const auto &entity : entities) {
    State state;
    state.entity = entity;
    state.key = {entity->id, entity->generation.load()};
    state.stamp = stamp;
    frame->states.push_back(std::move(state));
  }
  return frame;
}

void captured_frame_routing_and_join() {
  Sensors sensors(Json::object(), 1);
  RosRuntime runtime({{"publish_workers", 2}}, sensors);
  std::mutex mutex;
  std::condition_variable wake;
  bool old_entered = false, release_old = false, new_seen = false;
  bool sensor_entered = false, release_sensor = false;
  std::vector<std::pair<uint64_t, int64_t>> telemetry;
  std::vector<uint64_t> scans;
  std::map<uint64_t, unsigned> pending;
  std::vector<std::shared_ptr<Entity>> entities;
  for (uint64_t id = 1; id <= 5; ++id) {
    Config config;
    config.name = "output-" + std::to_string(id);
    auto entity = std::make_shared<Entity>(config);
    entity->id = id;
    entity->generation = 1;
    entity->alive = true;
    entity->io = std::make_shared<EntityIO>();
    entity->io->publish = [&, id](const State &state, const TelemetryRates &rates) {
      assert(id % 2 == 0); // Only lane zero runs in this deterministic fixture.
      assert(state.key.id == id);
      std::unique_lock<std::mutex> lock(mutex);
      telemetry.emplace_back(id, state.stamp);
      const bool old = state.stamp == epoch;
      assert(rates.periods[0] == (old ? 8000000 : 16000000));
      if (state.stamp == epoch + 16000000 && id == 2) {
        assert(state.key.generation == 2);
        pending[4] = pending[2] = 1;
      }
      if (old && id == 2) {
        old_entered = true;
        wake.notify_all();
        wake.wait(lock, [&] { return release_old; });
      }
      if (!old && id == 2) new_seen = true;
      wake.notify_all();
    };
    entity->io->publish_sensor = [&, id] {
      assert(id % 2 == 0);
      std::unique_lock<std::mutex> lock(mutex);
      if (!pending[id]) return false;
      --pending[id];
      scans.push_back(id);
      if (scans.size() == 5) {
        sensor_entered = true;
        wake.notify_all();
        wake.wait(lock, [&] { return release_sensor; });
      }
      wake.notify_all();
      return true;
    };
    entities.push_back(std::move(entity));
  }
  auto first = snapshot(epoch, {entities[0], entities[1], entities[2], entities[3]});
  auto second = snapshot(epoch + 8000000, {entities[3], entities[2], entities[1], entities[4]});
  auto old_rates = std::make_shared<TelemetryRates>();
  auto new_rates = std::make_shared<TelemetryRates>();
  new_rates->periods[0] = 16000000;
  runtime.running = true;
  runtime.lanes[0]->thread = std::thread([&] { runtime.work(0); });
  runtime.submit(std::make_shared<Frame>(), old_rates);
  until([&] {
    std::lock_guard<std::mutex> lock(runtime.lanes[0]->mutex);
    return !runtime.lanes[0]->next;
  });
  runtime.submit(first, old_rates);
  until([&] { std::lock_guard<std::mutex> lock(mutex); return old_entered; });
  runtime.submit(second, new_rates);
  {
    std::lock_guard<std::mutex> lock(mutex);
    release_old = true;
  }
  wake.notify_all();
  until([&] { std::lock_guard<std::mutex> lock(mutex); return new_seen; });
  {
    std::lock_guard<std::mutex> lock(mutex);
    assert((telemetry == std::vector<std::pair<uint64_t, int64_t>>{
        {2, epoch}, {4, epoch}, {4, epoch + 8000000}, {2, epoch + 8000000}}));
    pending[4] = pending[2] = 2;
  }
  runtime.lanes[0]->wake.notify_one();
  until([&] { std::lock_guard<std::mutex> lock(mutex); return scans.size() == 4; });
  {
    std::lock_guard<std::mutex> lock(mutex);
    assert((scans == std::vector<uint64_t>{4, 2, 4, 2}));
  }
  entities[1]->generation = 2;
  auto third = snapshot(epoch + 16000000, {entities[3], entities[1]});
  runtime.submit(third, new_rates); // Old cursor 3 must wrap to 1 in this two-state frame.
  until([&] { std::lock_guard<std::mutex> lock(mutex); return sensor_entered; });
  {
    std::lock_guard<std::mutex> lock(mutex);
    assert(scans.back() == 2);
  }
  std::weak_ptr<const Frame> held = third;
  first.reset();
  second.reset();
  third.reset();
  std::atomic<bool> stopped{false};
  std::thread stop([&] { runtime.stop(); stopped = true; });
  until([&] { return !runtime.running.load(); });
  assert(!stopped && !held.expired()); // stop must join the active publication.
  {
    std::lock_guard<std::mutex> lock(mutex);
    release_sensor = true;
  }
  wake.notify_all();
  stop.join();
  assert(stopped && held.expired()); // Both current and queued Frame owners left.
  assert(runtime.errors == 0);
}
} // namespace

int main() {
  captured_frame_routing_and_join();
  std::cout << "{\"result\":\"PASS\",\"scope\":\"output_lane_indices\"}\n";
}
