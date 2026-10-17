#include "core/world.hpp"
#include <atomic>
#include <cassert>
#include <chrono>
#include <iostream>
#include <thread>

using namespace xsim;

namespace {
constexpr int64_t epoch = 1700000000000000000LL;

Ticket command(Op op, Key key = {}) {
  auto result = std::make_shared<Command>();
  result->op = op;
  result->key = key;
  return result;
}

std::shared_ptr<Entity> add_enabled(World &world, Kind kind, const char *name) {
  Config config;
  config.kind = kind;
  config.name = name;
  auto entity = std::make_shared<Entity>(config);
  auto add = command(Op::Add);
  add->prepared = std::make_unique<Prepared>(Prepared{entity, prepare_model(config)});
  world.submit(add);
  world.apply_commands();
  assert(add->phase == 2 && add->result.success);
  auto enable = command(Op::SetEnabled, {entity->id, entity->generation});
  enable->enabled = true;
  world.submit(enable);
  world.apply_commands();
  assert(enable->phase == 2 && enable->result.success);
  return entity;
}

template <class Predicate> void eventually(Predicate ready) {
  const auto deadline = Clock::now() + std::chrono::seconds(2);
  while (!ready() && Clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  assert(ready());
}

void unchanged_ticket_and_time_contract() {
  World world(epoch);
  auto entity = add_enabled(world, Kind::Mecanum, "commands");
  const Key key{entity->id, entity->generation};
  std::vector<Ticket> burst;
  for (unsigned i = 0; i < 100; ++i) {
    auto input = command(Op::Velocity, key);
    input->velocity = {double(i + 1) / 100.0, 0, 0};
    world.submit(input);
    burst.push_back(std::move(input));
  }
  assert(world.metrics.input_misses == 99);
  for (size_t i = 0; i + 1 < burst.size(); ++i) assert(burst[i]->phase == 3);
  assert(burst.back()->phase == 0 && world.metrics.steps == 0);
  world.apply_commands();
  assert(burst.back()->phase == 2 && burst.back()->result.success);
  world.advance();
  assert(world.capture().states.front().velocity.x() == 1.0);

  auto future = command(Op::Velocity, key);
  future->at = world.metrics.sim_ns + 2 * world.dt;
  future->velocity = {.5, 0, 0};
  world.submit(future);
  world.apply_commands();
  assert(future->phase == 0);
  world.advance();
  world.apply_commands();
  assert(future->phase == 0);
  world.advance();
  world.apply_commands();
  assert(future->phase == 2 && future->result.success);
  assert(future->result.stamp == future->at);

  auto first = command(Op::Velocity, key);
  auto pause = command(Op::Pause);
  auto second = command(Op::Velocity, key);
  first->velocity = {.25, 0, 0};
  second->velocity = {.75, 0, 0};
  world.submit(first);
  world.submit(pause);
  world.submit(second);
  world.apply_commands();
  assert(first->phase == 2 && second->phase == 2 && pause->result.success);
  assert(world.metrics.input_misses == 99); // No coalescing through a discrete command.
}

void paused_input_management_and_hold() {
  World world(epoch);
  auto entity = add_enabled(world, Kind::Mecanum, "paused");
  world.metrics.paused = true;
  world.start();
  auto input = command(Op::Velocity, {entity->id, entity->generation});
  input->velocity = {1, 0, 0};
  input->received_ns = world.hold().now();
  world.submit(input);
  assert(World::wait(input) && input->result.success);
  assert(world.metrics.steps == 0 && world.metrics.sim_ns == epoch);

  const auto engaged = world.hold().domain().engage({"paused"}).results();
  assert(engaged.size() == 1 && engaged.front().state->held);
  eventually([&] {
    const auto state = world.hold().domain().state({"paused"});
    return state.robots.size() == 1 &&
           state.robots.front().stage != xgc2::chassis_hold::Stage::gated;
  });
  assert(world.metrics.steps == 0); // HOLD wakes the paused owner without integrating.

  auto steps = command(Op::Step);
  steps->steps = 7;
  world.submit(steps);
  assert(World::wait(steps) && steps->result.success && steps->result.completed_steps == 7);
  assert(world.metrics.paused && world.metrics.steps == 7);
  assert(world.metrics.sim_ns == epoch + 7 * world.dt);

  const auto generation = entity->generation.load();
  auto reset = command(Op::Reset, {entity->id, generation});
  reset->prepared = std::make_unique<Prepared>(Prepared{entity, prepare_model(entity->config)});
  world.submit(reset);
  assert(World::wait(reset) && reset->result.success);
  assert(entity->generation == generation + 1 && world.metrics.steps == 7);
  auto stale = command(Op::Velocity, {entity->id, generation});
  world.submit(stale);
  assert(World::wait(stale) && !stale->result.success && stale->result.reason == 1);

  auto resume = command(Op::Resume);
  world.submit(resume);
  assert(World::wait(resume) && resume->result.success && !world.metrics.paused);
  auto repeated = command(Op::Resume);
  world.submit(repeated);
  assert(World::wait(repeated) && repeated->result.success);
  world.stop();
}

void pause_arrival_race() {
  World world(epoch);
  world.start();
  for (unsigned cycle = 0; cycle < 16; ++cycle) {
    auto pause = command(Op::Pause);
    auto expired = command(Op::Velocity);
    expired->deadline = Clock::now() - std::chrono::seconds(1);
    std::atomic<bool> completed{false};
    expired->notification_context = &completed;
    expired->notification = [](void *context) noexcept {
      static_cast<std::atomic<bool> *>(context)->store(true, std::memory_order_release);
    };
    std::atomic<bool> go{false};
    std::thread producer([&] {
      while (!go.load(std::memory_order_acquire)) std::this_thread::yield();
      world.submit(expired);
    });
    go.store(true, std::memory_order_release);
    world.submit(pause);
    producer.join();
    assert(World::wait(pause) && pause->result.success);
    // Observe actual owner completion, not World::wait's caller-side cancellation.
    eventually([&] { return completed.load(std::memory_order_acquire); });
    assert(expired->phase == 3 && world.metrics.paused);
    auto resume = command(Op::Resume);
    world.submit(resume);
    assert(World::wait(resume) && resume->result.success);
  }
  world.stop();
}

void zero_integration_cost_is_retained() {
  PhysicsClock clock(2000000, 10000000);
  clock.rebase(0, 0);
  // An early management boundary can spend owner time without integrating.
  // Charge it to the same existing load window, not to sleep or paused time.
  for (int64_t window = 1; window <= 2; ++window) {
    clock.observe(0, 90000000);
    clock.observe(100000000, 10000000);
    clock.schedule(window * 100000000);
  }
  assert(clock.period() == 2500000 && clock.degradations() == 1);
  clock.rebase(10000000000LL, 12345);
  assert(clock.target(10000000000LL) == 12345); // Resume clears old window debt/cost.
}
} // namespace

int main() {
  unchanged_ticket_and_time_contract();
  paused_input_management_and_hold();
  pause_arrival_race();
  zero_integration_cost_is_retained();
  std::cout << "PASS: continuous tickets, paused wakes, HOLD, exact Step, Reset and clock cost\n";
}
