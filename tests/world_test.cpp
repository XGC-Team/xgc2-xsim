#include "scene_fixture.hpp"
#include "systems/sensors.hpp"
#include "io/runtime_io.hpp"
#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <thread>
using namespace xsim;
Ticket request(World &w, Op op, Key k = {}) {
  auto t = std::make_shared<Command>();
  t->op = op;
  t->key = k;
  return t;
}
void execute(World &w, const Ticket &t) {
  w.submit(t);
  w.boundary();
  assert(t->phase == 2);
}
std::shared_ptr<Entity> add(World &w, const char *kind,
                            const std::string &name) {
  auto e = std::make_shared<Entity>(parse_entity({{"name", name},
                                                  {"kind", kind},
                                                  {"position", {2, -1, 0}},
                                                  {"yaw", .35}}));
  auto t = request(w, Op::Add);
  t->prepared =
      std::make_unique<Prepared>(Prepared{e, prepare_model(e->config)});
  execute(w, t);
  assert(t->result.success);
  return e;
}
void start(World &w, const std::shared_ptr<Entity> &e) {
  auto t = request(w, Op::Provider, {e->id, e->generation});
  t->action = 1;
  t->prepared =
      std::make_unique<Prepared>(Prepared{e, prepare_model(e->config)});
  execute(w, t);
  assert(t->result.success && e->enabled);
}
int main() {
  const auto rejected_ros_key = [](const char *kind, const char *key) {
    bool rejected = false;
    try {
      parse_entity({{"name", "obsolete"}, {"kind", kind},
                    {"ros", {{key, "/obsolete/interface"}}}});
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    assert(rejected);
  };
  for (const char *kind : {"fs150", "scout", "mecanum"})
    for (const char *key : {"provider_service", "truth_topic", "reset_service",
                            "mocap_topic", "mocap_velocity_topic"})
      rejected_ros_key(kind, key);
  for (const char *key : {"pose_topic", "velocity_topic", "odometry_topic"}) {
    for (const char *kind : {"scout", "mecanum"})
      rejected_ros_key(kind, key);
    assert(parse_entity({{"name", "flight"}, {"kind", "fs150"},
                         {"ros", {{key, "/flight/mavros"}}}}).kind == Kind::FS150);
  }
  // The single Entity owns its concrete IO attachment. Weak snapshots and
  // public projections cannot extend removed resource lifetimes.
  {
    World owner(1700000000000000000LL);
    auto entity = add(owner, "scout", "io_owner");
    entity->io = std::make_shared<EntityIO>();
    std::weak_ptr<EntityIO> attachment = entity->io;
    auto frame = owner.capture();
    auto removal = request(owner, Op::Remove, {entity->id, entity->generation});
    execute(owner, removal);
    assert(removal->result.success);
    entity.reset();
    assert(!attachment.expired()); // Original retired command owns cleanup.
    removal->retired.reset();
    assert(attachment.expired() && frame.states.front().entity.expired());
  }
  // Output deadlines stay on the epoch grid even when dt does not divide the
  // output period. This is the original plant's 3 ms / 10 ms schedule.
  World rate_grid(1700000000000000000LL,3000000,10000000);
  Frame output;std::vector<int64_t> stamps;
  for(int k=0;k<20;++k){rate_grid.advance();if(rate_grid.take_frame(output))stamps.push_back(output.stamp-rate_grid.epoch);}
  assert((stamps==std::vector<int64_t>{12000000,21000000,30000000,42000000,51000000,60000000}));
  // Unsafe intervals are rejected before any model can be partially stepped.
  bool unsafe = false;
  try { World invalid(1700000000000000000LL, 201000000); }
  catch (const std::invalid_argument &) { unsafe = true; }
  assert(unsafe);
  World fine_output(1700000000000000000LL,10000000,3000000);
  fine_output.advance();Frame fine;assert(fine_output.take_frame(fine));assert(fine.stamp==fine_output.epoch+10000000);
  // The shared output pool is bounded and immutable while any consumer holds
  // a frame. Saturation skips observations, never stalls model integration.
  {
    World pooled(1700000000000000000LL, 2000000, 2000000);
    auto first = add(pooled, "mecanum", "pooled");
    const auto initial_growth = pooled.metrics.frame_array_grows.load();
    assert(initial_growth == World::frame_pool_size);
    std::vector<std::shared_ptr<const Frame>> held;
    for (size_t i = 0; i < World::frame_pool_size; ++i) {
      pooled.advance();
      std::shared_ptr<const Frame> frame;
      assert(pooled.take_frame(frame));
      assert(frame->geometry_revision > 0 && frame->states.size() == 1);
      for (const auto &earlier : held) {
        assert(earlier.get() != frame.get());
        assert(earlier->geometry_revision == frame->geometry_revision);
      }
      held.push_back(std::move(frame));
    }
    const auto oldest_stamp = held.front()->stamp;
    const auto *const reusable = held.front().get();
    const auto old_geometry = held.front()->geometry_revision;
    const auto misses = pooled.metrics.output_misses.load();
    pooled.advance();
    std::shared_ptr<const Frame> resumed;
    assert(!pooled.take_frame(resumed));
    assert(pooled.metrics.output_misses == misses + 1);
    assert(pooled.metrics.steps == World::frame_pool_size + 1);
    for (size_t i = 0; i < held.size(); ++i) {
      assert(held[i]->stamp == oldest_stamp + int64_t(i) * pooled.dt);
      assert(held[i]->states.front().key.id == first->id);
      assert(held[i]->states.front().position == first->config.initial);
      assert(!held[i]->states.front().enabled);
    }
    held.front().reset();
    pooled.advance();
    assert(pooled.take_frame(resumed) && resumed.get() == reusable);
    assert(resumed->stamp == oldest_stamp + int64_t(World::frame_pool_size + 1) * pooled.dt);
    assert(resumed->geometry_revision == old_geometry);
    held.clear();
    for (size_t i = 0; i < World::frame_pool_size * 2; ++i) {
      pooled.advance();
      assert(pooled.take_frame(resumed));
    }
    assert(pooled.metrics.frame_array_grows == initial_growth);

    const auto retained = resumed;
    add(pooled, "scout", "second");
    pooled.advance();
    assert(pooled.take_frame(resumed));
    assert(resumed->states.size() == 2 && retained->states.size() == 1);
    assert(resumed->geometry_revision > retained->geometry_revision);
    const auto enabled_geometry = resumed->geometry_revision;
    start(pooled, first);
    pooled.advance();
    assert(pooled.take_frame(resumed));
    assert(resumed->geometry_revision > enabled_geometry);
    const auto stationary_geometry = resumed->geometry_revision;
    pooled.advance();
    assert(pooled.take_frame(resumed));
    assert(resumed->geometry_revision == stationary_geometry);
    auto motion = request(pooled, Op::Velocity, {first->id, first->generation});
    motion->velocity = {1, 0, 0};
    execute(pooled, motion);
    pooled.advance();
    assert(pooled.take_frame(resumed));
    assert(resumed->geometry_revision > stationary_geometry);
    assert(retained->states.front().key.generation == 0);
    assert(retained->states.front().position == first->config.initial);
  }
  bool overflow=false;try{World invalid(INT64_MAX-1000000,1000000,10000000);}catch(const std::invalid_argument&){overflow=true;}assert(overflow);
  World paced(1700000000000000000LL, 10000000, 10000000);
  const auto wall_start = Clock::now();
  paced.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(3));
  assert(paced.metrics.steps == 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  const auto completed = paced.metrics.steps.load();
  const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
                           Clock::now() - wall_start)
                           .count();
  assert(completed > 0 && paced.metrics.sim_ns.load() - paced.epoch <= elapsed);
  paced.stop();
  World w(1700000000000000000LL);
  std::vector<std::shared_ptr<Entity>> entities;
  for (const char *kind : {"fs150", "scout", "mecanum"}) {
    auto e = add(w, kind, kind);
    entities.push_back(e);
    start(w, e);
    Key old{e->id, e->generation};
    auto stale =
        request(w, e->config.kind == Kind::FS150 ? Op::Arm : Op::Velocity, old);
    stale->arm = true;
    stale->velocity = {1, 1, 1};
    stale->at = w.metrics.sim_ns + 100000000;
    // Pending old-generation command must not affect reset entity.
    if (stale->op == Op::Arm) {
      stale->op = Op::Pva;
      stale->pva.coordinate_frame = 1;
      stale->pva.type_mask = 3527;
      stale->pva.velocity[0] = 1;
    }
    w.submit(stale);
    auto reset = request(w, Op::Reset, old);
    reset->prepared =
        std::make_unique<Prepared>(Prepared{e, prepare_model(e->config)});
    execute(w, reset);
    assert(e->generation == old.generation + 1);
    for (int k = 0; k < 110; ++k) {
      w.boundary();
      w.advance();
    }
    assert(stale->phase == 2 && !stale->result.success &&
           stale->result.reason == 1);
    auto expired = request(w, Op::Arm, {e->id, e->generation});
    expired->arm = true;
    expired->deadline = Clock::now() - std::chrono::milliseconds(1);
    w.submit(expired);
    assert(!World::wait(expired));
    w.boundary();
    assert(expired->phase == 3);
    auto stop = request(w, Op::Provider, {e->id, e->generation});
    stop->action = 2;
    execute(w, stop);
    assert(!e->enabled);
    auto oldstop = request(w, Op::Provider, old);
    oldstop->action = 2;
    execute(w, oldstop);
    assert(!oldstop->result.success);
    start(w, e);
  }
  // Dense swap after removing first of the same kind, then command survivor.
  auto a = add(w, "mecanum", "a"), b = add(w, "mecanum", "b");
  start(w, a);
  start(w, b);
  auto remove = request(w, Op::Remove, {a->id, a->generation});
  execute(w, remove);
  assert(remove->result.applied && remove->result.success && !remove->result.enabled);
  assert(!a->alive && !a->enabled);
  auto replacement = add(w, "mecanum", "a");
  assert(replacement->id != a->id);
  start(w, replacement);
  auto old = request(w, Op::Velocity, {a->id, a->generation});
  old->velocity = {1, 0, 0};
  execute(w, old);
  assert(!old->result.success);
  auto move = request(w, Op::Velocity, {b->id, b->generation});
  move->velocity = {1, 0, 0};
  execute(w, move);
  w.advance();
  for (auto &s : w.capture().states) {
    if (s.key.id == b->id)
      assert(s.velocity.norm() > .9);
    if (s.key.id == replacement->id)
      assert(s.velocity.norm() == 0);
  }
  // Caller can be slow/hold its synchronization lock; world completion cannot
  // wait.
  auto held = request(w, Op::Arm, {entities[0]->id, entities[0]->generation});
  held->arm = true;
  std::unique_lock<std::mutex> held_lock(held->mutex);
  execute(w, held);
  assert(held->result.success);
  held_lock.unlock();
  // The deterministic World has already bound HOLD to this thread; native management uses a new World.
  World paused_world(w.metrics.sim_ns);
  auto pause = request(paused_world, Op::Pause);
  paused_world.submit(pause); paused_world.apply_commands();
  const auto before = paused_world.metrics.steps.load();
  paused_world.start();
  auto paused_add = request(paused_world, Op::Add);
  auto pe = std::make_shared<Entity>(
      parse_entity({{"name", "paused"}, {"kind", "scout"}}));
  paused_add->prepared =
      std::make_unique<Prepared>(Prepared{pe, prepare_model(pe->config)});
  paused_world.submit(paused_add);
  assert(World::wait(paused_add));
  assert(paused_world.metrics.steps == before);
  auto step = request(paused_world, Op::Step);
  step->steps = 7;
  paused_world.submit(step);
  assert(World::wait(step));
  assert(paused_world.metrics.steps == before + 7);
  paused_world.stop();
  // One flat waiter per Arm/Mode, the same submit-then-wait a service
  // callback uses. apply_commands drains the inbox on the world thread.
  uint64_t fleet_applied_step_span = 0;
  int64_t fleet_queue_spread_ns = 0, fleet_apply_spread_ns = 0,
          fleet_response_spread_ns = 0, fleet_queue_to_apply_ns = 0,
          fleet_queue_to_response_ns = 0;
  {
    World fleet(1700000000000000000LL);
    fleet.start();
    constexpr int count = 100;
    std::vector<std::shared_ptr<Entity>> entities;
    entities.reserve(count);
    for (int i = 0; i < count; ++i) {
      auto entity = std::make_shared<Entity>(parse_entity(
          {{"name", "fleet_" + std::to_string(i)}, {"kind", "fs150"}}));
      auto added = request(fleet, Op::Add);
      added->prepared =
          std::make_unique<Prepared>(Prepared{entity, prepare_model(entity->config)});
      fleet.submit(added);
      assert(World::wait(added) && added->result.success);
      auto enabled = request(fleet, Op::Provider, {entity->id, entity->generation});
      enabled->action = 1;
      enabled->prepared =
          std::make_unique<Prepared>(Prepared{entity, prepare_model(entity->config)});
      fleet.submit(enabled);
      assert(World::wait(enabled) && enabled->result.success);
      entities.push_back(std::move(entity));
    }
    std::vector<Ticket> calls(count);
    for (int i = 0; i < count; ++i) {
      const bool arm = (i % 2) == 0;
      calls[i] = request(fleet, arm ? Op::Arm : Op::Mode,
                         {entities[i]->id, entities[i]->generation});
      if (arm)
        calls[i]->arm = true;
      else
        calls[i]->mode = "POSCTL";
    }
    std::atomic<int> armed_clients{0};
    std::atomic<bool> release{false};
    std::vector<std::thread> clients;
    clients.reserve(count);
    for (int i = 0; i < count; ++i)
      clients.emplace_back([&, i] {
        if (armed_clients.fetch_add(1) + 1 == count)
          release.store(true, std::memory_order_release);
        while (!release.load(std::memory_order_acquire))
          std::this_thread::yield();
        fleet.submit(calls[i]);
        assert(World::wait(calls[i]));
      });
    for (auto &client : clients)
      client.join();
    uint64_t min_step = UINT64_MAX, max_step = 0;
    auto min_queued = Clock::time_point::max();
    auto max_queued = Clock::time_point::min();
    auto min_applied = Clock::time_point::max();
    auto max_applied = Clock::time_point::min();
    auto min_feedback = Clock::time_point::max();
    auto max_feedback = Clock::time_point::min();
    for (const auto &ticket : calls) {
      assert(ticket->phase == 2 && ticket->result.success);
      assert(ticket->queued_at <= ticket->applied_at);
      assert(ticket->applied_at <= ticket->feedback_at);
      min_step = std::min(min_step, ticket->result.step);
      max_step = std::max(max_step, ticket->result.step);
      min_queued = std::min(min_queued, ticket->queued_at);
      max_queued = std::max(max_queued, ticket->queued_at);
      min_applied = std::min(min_applied, ticket->applied_at);
      max_applied = std::max(max_applied, ticket->applied_at);
      min_feedback = std::min(min_feedback, ticket->feedback_at);
      max_feedback = std::max(max_feedback, ticket->feedback_at);
    }
    const auto elapsed_ns = [](Clock::time_point from, Clock::time_point to) {
      return std::chrono::duration_cast<std::chrono::nanoseconds>(to - from).count();
    };
    fleet_applied_step_span = max_step - min_step;
    fleet_queue_spread_ns = elapsed_ns(min_queued, max_queued);
    fleet_apply_spread_ns = elapsed_ns(min_applied, max_applied);
    fleet_response_spread_ns = elapsed_ns(min_feedback, max_feedback);
    fleet_queue_to_apply_ns = elapsed_ns(min_queued, max_applied);
    fleet_queue_to_response_ns = elapsed_ns(min_queued, max_feedback);
    // A boundary may split the burst. One call per step would span ~100.
    assert(fleet_applied_step_span + 1 < static_cast<uint64_t>(count) / 2);
    fleet.stop();
  }
  // Real immutable-scene CPU scans, mount transform, generation and bounded
  // pending.
  Sensors sensors(sceneFixture({{"obstacles", Json::array({{{"type", "box"},
                                               {"position", {4, 0, 0}},
                                               {"size", {1, 2, 2}}}})}}),
                  2);
  const Json sensor_fixture = {{"name", "sensor"},
                                             {"kind", "scout"},
                                             {"sensor",
                                              {{"backend", "cpu"},
                                               {"h_fov_deg", 90},
                                               {"v_fov_deg", 30},
                                               {"h_res", 1000},
                                               {"v_res", 200},
                                               {"rate_hz", 1000},
                                               {"translation", {1, 0, 0}}}}};
  auto se = std::make_shared<Entity>(parse_entity(sensor_fixture));
  se->id = 9;
  se->alive = true;
  se->enabled = true;
  se->generation = 1;
  se->sensor = sensors.prepare(se, sensor_fixture.at("sensor"));
  State s;
  s.entity = se;
  s.key = {9, 1};
  s.enabled = true;
  for (int k = 0; k < 100; ++k) {
    s.stamp = 1700000000000000000LL + k * 1000000;
    sensors.submit(s);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  Sample sampled;
  std::vector<uint8_t> cloud;
  assert(Sensors::take(se->sensor, sampled, cloud));
  assert(sampled.position.x() == 1);
  assert(sampled.scene_version == 1);
  assert(!cloud.empty());
  assert(se->sensor->misses > 0);
  s.stamp += 1000000;
  sensors.submit(s);
  ++se->generation;
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  assert(!Sensors::take(se->sensor, sampled, cloud));
  auto sensor_status = sensors.status();
  sensors.stop();
  std::cout << Json({{"result", "PASS"},
                     {"checks",
                      {"retired_ros_config", "entity_io_retirement", "three_kinds", "reset_old_generation", "timeout_cancel",
                       "swap_remove", "old_id", "slow_waiter",
                       "paused_management", "step_exact", "fleet_arm_mode_timing", "cpu_scan",
                       "mount_pose", "pending_replacement", "stale_scan"}},
                     {"fleet_calls", 100},
                     {"applied_step_span", fleet_applied_step_span},
                     {"queue_spread_ns", fleet_queue_spread_ns},
                     {"apply_spread_ns", fleet_apply_spread_ns},
                     {"response_spread_ns", fleet_response_spread_ns},
                     {"queue_to_apply_ns", fleet_queue_to_apply_ns},
                     {"queue_to_response_ns", fleet_queue_to_response_ns},
                     {"sensors", sensor_status}})
                   .dump()
            << '\n';
}
