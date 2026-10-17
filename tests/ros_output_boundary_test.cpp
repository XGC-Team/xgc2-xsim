// Uses the production output workers with the ROS SDK, without a master/World.
#include "../src/xsim/io/ros/runtime.cpp"
#include <cassert>
#include <new>
#include <sys/mman.h>
#include <unistd.h>

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
std::shared_ptr<Frame> snapshot(int64_t stamp,
    const std::vector<std::shared_ptr<Entity>> &entities) {
  auto frame = std::make_shared<Frame>();
  frame->stamp = stamp;
  frame->revision = 7; // Same revision is not a roster/order identity.
  for (const auto &entity : entities) {
    State state;
    state.entity = entity;
    state.key = {entity->id, entity->generation.load()};
    state.stamp = stamp;
    frame->states.push_back(std::move(state));
  }
  return frame;
}
void complete(Sensors &sensors, const std::shared_ptr<Sensor> &sensor, int64_t stamp) {
  {
    std::lock_guard<std::mutex> lock(sensor->mutex);
    const auto entity = sensor->entity.lock();
    sensor->completed.key = {entity->id, entity->generation.load()};
    sensor->completed.stamp = stamp;
    sensor->completed_payload = sensor->payload_pool[0];
    sensor->has_completed = true;
  }
  sensors.enable_completions(sensor);
}
void independent_roles_and_restart() {
  Sensors sensors(Json::object(), 1);
  RosRuntime runtime(Json::object(), sensors);
  std::mutex mutex;
  std::condition_variable wake;
  bool cloud_entered = false, release_cloud = false, claimed_handoff = false;
  std::vector<std::pair<uint64_t, int64_t>> telemetry, clouds;
  std::vector<std::shared_ptr<Entity>> entities;
  for (uint64_t id = 1; id <= 5; ++id) {
    Config config;
    config.name = "output-" + std::to_string(id);
    auto entity = std::make_shared<Entity>(config);
    entity->id = id;
    entity->generation = 1;
    entity->alive = entity->enabled = true;
    entity->sensor = sensors.prepare(entity, {{"backend", "cpu"}, {"h_res", 4}, {"v_res", 2}});
    entity->io = std::make_shared<EntityIO>();
    entity->io->publish = [&, id](const State &state, const TelemetryRates &rates) {
      assert(state.key.id == id);
      std::lock_guard<std::mutex> lock(mutex);
      assert(rates.periods[0] == (state.stamp == epoch ? 8000000 : 16000000));
      if (state.stamp != epoch && id == 2) assert(state.key.generation == 2);
      telemetry.emplace_back(id, state.stamp);
    };
    entity->io->publish_sensor = [&, id](const Sample &sample,
        const std::shared_ptr<const SensorPayload> &payload) {
      assert(payload && sample.key.id == id);
      std::unique_lock<std::mutex> lock(mutex);
      clouds.emplace_back(id, sample.stamp);
      if (id == 1) {
        cloud_entered = true;
        wake.notify_all();
        wake.wait(lock, [&] { return release_cloud; });
        assert(!runtime.peer_ready); // Discovery loss cannot discard an already claimed handoff.
        claimed_handoff = true;
      }
    };
    entities.push_back(std::move(entity));
  }
  runtime.set_peer("ready", "");
  runtime.start_lanes();
  complete(sensors, entities[0]->sensor, epoch);
  until([&] { std::lock_guard<std::mutex> lock(mutex); return cloud_entered; });
  auto first = snapshot(epoch, entities);
  auto rates = std::make_shared<TelemetryRates>();
  runtime.submit(first, rates);
  until([&] { std::lock_guard<std::mutex> lock(mutex); return telemetry.size() == 5; });
  entities[1]->generation = 2;
  auto second = snapshot(epoch + 8000000, {entities[4], entities[3], entities[2], entities[1]});
  auto next_rates = std::make_shared<TelemetryRates>();
  next_rates->periods[0] = 16000000;
  runtime.submit(second, next_rates);
  until([&] { std::lock_guard<std::mutex> lock(mutex); return telemetry.size() == 9; });
  {
    std::lock_guard<std::mutex> lock(mutex);
    assert((telemetry == std::vector<std::pair<uint64_t,int64_t>>{
      {1,epoch},{2,epoch},{3,epoch},{4,epoch},{5,epoch},
      {5,epoch+8000000},{4,epoch+8000000},{3,epoch+8000000},{2,epoch+8000000}}));
  }
  std::weak_ptr<const Frame> old = first, latest = second;
  first.reset(); second.reset();
  until([&] { return old.expired() && latest.expired(); }); // Idle telemetry owns no Frame.
  complete(sensors, entities[1]->sensor, epoch);
  complete(sensors, entities[1]->sensor, epoch + 16000000); // One token, newest completion.
  complete(sensors, entities[2]->sensor, epoch);
  std::atomic<bool> stopped{false};
  runtime.set_peer("master_unavailable", "fixture outage after take_completion");
  std::thread stop([&] { runtime.stop_lanes(); stopped = true; });
  until([&] { return !runtime.running.load(); });
  assert(!stopped); // Must join an in-flight publication, not discard its payload.
  {
    std::lock_guard<std::mutex> lock(mutex);
    release_cloud = true;
  }
  wake.notify_all();
  stop.join();
  assert(stopped);
  {
    std::lock_guard<std::mutex> lock(mutex);
    assert(clouds.size() == 1 && claimed_handoff);
  }
  runtime.start_lanes(); // Outage did not erase latest/queued completions.
  until([&] { std::lock_guard<std::mutex> lock(mutex); return clouds.size() == 3; });
  runtime.stop(); // Both idle CV waits are interruptible, no periodic polling.
  {
    std::lock_guard<std::mutex> lock(mutex);
    assert((clouds == std::vector<std::pair<uint64_t,int64_t>>{
      {1,epoch},{2,epoch+16000000},{3,epoch}}));
  }
  assert(runtime.errors == 0);
}
// Match main's ownership order: World outlives Sensors, while its Entity IO
// retains RosEntity::runtime_owner after the explicit output stop.
void stopped_runtime_outlives_sensors() {
  const auto page = size_t(sysconf(_SC_PAGESIZE));
  assert(page > 0);
  const auto bytes = ((sizeof(Sensors) + page - 1) / page) * page;
  void *storage = mmap(nullptr, bytes, PROT_READ | PROT_WRITE,
                       MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
  assert(storage != MAP_FAILED);
  std::weak_ptr<RosRuntime> retained;
  std::weak_ptr<Entity> owned;
  {
    World world(epoch);
    auto *sensors = new (storage) Sensors(Json::object(), 1);
    auto runtime = std::make_shared<RosRuntime>(Json::object(), *sensors);
    retained = runtime;
    Config config;
    config.name = "delayed_runtime";
    config.kind = Kind::Scout;
    auto entity = std::make_shared<Entity>(config);
    owned = entity;
    auto attachment = std::make_shared<RosEntity>(entity, world, runtime->inputs,
        runtime->services, Json::object(), runtime);
    entity->io = std::make_shared<EntityIO>();
    entity->io->reconcile = [attachment] { attachment->request_reconcile(); };
    auto add = std::make_shared<Command>();
    add->op = Op::Add;
    add->prepared = std::make_unique<Prepared>(Prepared{entity, prepare_model(entity->config)});
    world.submit(add);
    world.apply_commands();
    assert(add->result.success);
    runtime->start_lanes();
    runtime->stop(); // Both real output workers have joined while Sensors lives.
    attachment.reset(); entity.reset(); add.reset(); runtime.reset();
    assert(!retained.expired() && !owned.expired()); // World is now the only owner.
    sensors->~Sensors();
    // Deterministic invalid-access fence, rather than a timing/UAF lottery.
    assert(mprotect(storage, bytes, PROT_NONE) == 0);
  } // World releases Entity -> EntityIO -> RosEntity -> stopped RosRuntime.
  assert(retained.expired() && owned.expired());
  assert(munmap(storage, bytes) == 0);
}

} // namespace
int main() {
  independent_roles_and_restart();
  stopped_runtime_outlives_sensors();
  std::cout << "{\"result\":\"PASS\",\"scope\":\"independent_output_roles_and_completion_resume\"}\n";
}
