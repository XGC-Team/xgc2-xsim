#include "core/world.hpp"
#include <cassert>
#include <iostream>

using namespace xsim;

namespace {
Ticket command(Op op, Key key = {}) {
  auto result = std::make_shared<Command>();
  result->op = op;
  result->key = key;
  return result;
}
void execute(World &world, const Ticket &request) {
  world.submit(request);
  world.apply_commands();
  assert(request->phase == 2);
}
std::shared_ptr<Entity> add(World &world, Kind kind, const std::string &name) {
  Config config;
  config.name = name;
  config.kind = kind;
  config.initial = {2, -1, kind == Kind::FS150 ? 1.0 : 0.0};
  config.yaw = .35;
  auto entity = std::make_shared<Entity>(config);
  auto request = command(Op::Add);
  request->capture_state = true;
  request->prepared = std::make_unique<Prepared>(Prepared{entity, prepare_model(config)});
  execute(world, request);
  assert(request->result.success && request->result.has_state);
  assert(request->result.state.key.id == entity->id);
  assert(!request->result.state.enabled);
  return entity;
}
Ticket enabled(World &world, const std::shared_ptr<Entity> &entity, bool value) {
  auto request = command(Op::SetEnabled, {entity->id, entity->generation});
  request->enabled = value;
  request->capture_state = true;
  execute(world, request);
  assert(request->result.success && request->result.has_state);
  assert(request->result.state.enabled == value);
  return request;
}
State entity_state(World &world, uint64_t id) {
  for (const auto &state : world.capture().states)
    if (state.key.id == id) return state;
  assert(false);
  return {};
}

void toggle_without_reset(Kind kind, const char *name) {
  World world(1700000000000000000LL);
  auto entity = add(world, kind, name);
  const auto generation = entity->generation.load();
  auto first = enabled(world, entity, true);
  assert(entity->generation == generation);
  if (kind != Kind::FS150) {
    const auto released = world.hold().domain().release(world.hold().instance_id(), {{entity->public_id, 1}});
    world.boundary(); // this deterministic fixture never starts a world thread
    assert(released.done() && released.results()[0].outcome == xgc2::chassis_hold::Outcome::released);
    auto move = command(Op::Velocity, {entity->id, entity->generation});
    move->velocity = {1, .2, .1};
    move->received_ns = world.hold().now();
    execute(world, move);
    assert(move->result.success && !move->result.has_state);
  }
  for (unsigned i = 0; i < 50; ++i) world.advance();
  const auto moved = entity_state(world, entity->id);
  assert((moved.position - first->result.state.position).norm() > 1e-4);
  auto stopped = enabled(world, entity, false);
  auto restarted = enabled(world, entity, true);
  assert(entity->generation == generation);
  assert((stopped->result.state.position - moved.position).norm() < 1e-12);
  assert((restarted->result.state.position - moved.position).norm() < 1e-12);
  // Retained command results do not alias a later live/output snapshot.
  assert(!stopped->result.state.enabled && restarted->result.state.enabled);
  assert(first->result.state.enabled);
  assert((first->result.state.position - entity->config.initial).norm() < 1e-12);

  auto future = command(kind == Kind::FS150 ? Op::Pva : Op::Velocity,
                        {entity->id, entity->generation});
  future->at = world.metrics.sim_ns + 1000000000;
  future->velocity = {1, 0, 0};
  world.submit(future);
  auto stop = command(Op::SetEnabled, {entity->id, entity->generation});
  stop->enabled = false;
  world.submit(stop);
  auto reset = command(Op::Reset, {entity->id, entity->generation});
  reset->capture_state = true;
  reset->prepared = std::make_unique<Prepared>(Prepared{entity, prepare_model(entity->config)});
  world.submit(reset);
  world.apply_commands();
  assert(future->phase == 3); // Only pending controls are cancelled.
  assert(stop->result.success && reset->result.success && reset->result.has_state);
  assert(entity->generation == generation + 1);
  assert((reset->result.state.position - entity->config.initial).norm() < 1e-12);

  // Legacy ROS Provider start remains a reset operation.
  auto legacy = command(Op::Provider, {entity->id, entity->generation});
  legacy->action = 1;
  legacy->prepared = std::make_unique<Prepared>(Prepared{entity, prepare_model(entity->config)});
  execute(world, legacy);
  assert(legacy->result.success && entity->generation == generation + 2);

  auto removed = command(Op::Remove, {entity->id, entity->generation});
  removed->capture_state = true;
  execute(world, removed);
  assert(removed->result.success && removed->result.has_state);
  assert(removed->result.state.key.id == entity->id);
  assert(!removed->result.state.enabled && !entity->alive);
  assert(world.capture().states.empty());
  reset->prepared.reset();
  legacy->prepared.reset();
  entity.reset();
  removed->retired.reset();
  assert(removed->result.state.entity.expired());
}

void batch_and_ready() {
  World world(1700000000000000000LL);
  auto first = add(world, Kind::Scout, "first");
  auto second = add(world, Kind::Mecanum, "second");
  auto insufficient = command(Op::Pause);
  insufficient->capture_states = true;
  execute(world, insufficient);
  assert(!insufficient->result.success && !world.metrics.paused);

  auto pause = command(Op::Pause);
  pause->capture_states = true;
  pause->states.reserve(256);
  const auto *storage = pause->states.data();
  execute(world, pause);
  assert(pause->result.success && pause->result.paused);
  assert(pause->states.size() == 2 && pause->states.data() == storage);
  assert(pause->states.front().stamp == pause->result.stamp);
  auto enable = command(Op::SetEnabled, {first->id, first->generation});
  enable->enabled = true;
  enable->capture_state = true;
  world.submit(enable);
  world.start();
  // The first boundary and emitted frame, not running_/a clock sample, fence start.
  assert(enable->phase == 2 && enable->result.success);
  Frame frame;
  assert(world.take_frame(frame) && frame.states.size() == 2);
  bool found = false;
  for (const auto &state : frame.states)
    if (state.key.id == first->id) { assert(state.enabled); found = true; }
  assert(found);
  for (const auto &state : pause->states) assert(!state.enabled);
  auto step = command(Op::Step);
  step->steps = 3;
  step->capture_states = true;
  step->states.reserve(256);
  world.submit(step);
  assert(World::wait(step) && step->result.success && step->result.paused);
  assert(step->result.step == 3 && step->states.size() == 2);
  assert(step->result.completed_steps == 3 && !step->result.interrupted);
  for (const auto &state : step->states) assert(state.stamp == step->result.stamp);
  auto reset = command(Op::Reset);
  reset->action = 1;
  reset->capture_states = true;
  reset->states.reserve(256);
  for (const auto &entity : {first, second})
    reset->resets.emplace_back(Key{entity->id, entity->generation}, prepare_model(entity->config));
  world.submit(reset);
  assert(World::wait(reset) && reset->result.success && reset->states.size() == 2);
  for (const auto &state : reset->states) assert(state.stamp == reset->result.stamp);
  for (const auto &state : step->states) assert(state.key.generation == 0);
  world.stop();

  // Existing pending cancellation remains available after stopping.
  auto pending = command(Op::Velocity, {second->id, second->generation});
  world.submit(pending);
  world.stop();
  assert(pending->phase == 3);
  World fresh(world.epoch);
  assert(fresh.hold().instance_id() != world.hold().instance_id());
  fresh.start();
  fresh.stop();
  bool rejected = false;
  try { world.start(); } catch (const std::logic_error &) { rejected = true; }
  assert(rejected); // a stopped production World must never create a second tick thread
}

void stop_partial_step() {
  World world(1700000000000000000LL);
  auto entity = add(world, Kind::Scout, "partial-step");
  execute(world, command(Op::Pause));

  auto partial = command(Op::Step);
  partial->steps = 100;
  partial->capture_states = true;
  partial->states.reserve(256);
  const auto *storage = partial->states.data();
  world.submit(partial);
  world.apply_commands();
  assert(partial->phase == 1);
  world.advance();
  world.advance();
  const auto stopped_stamp = world.metrics.sim_ns.load();
  world.stop();
  assert(partial->phase == 4 && partial->result.interrupted);
  assert(!partial->result.success && partial->result.applied);
  assert(partial->result.completed_steps == 2 && partial->result.step == 2);
  assert(partial->result.stamp == stopped_stamp && partial->result.paused);
  assert(partial->states.data() == storage && partial->states.size() == 1);
  assert(partial->states.front().stamp == stopped_stamp);
  assert(partial->states.front().key.id == entity->id);
  assert(!World::wait(partial));

  // Resume succeeds after Stop cancels outstanding step work. A new World starts without that work.
  auto resume = command(Op::Resume);
  execute(world, resume);
  assert(resume->result.success);
  execute(world, command(Op::Pause));
  World fresh(stopped_stamp);
  fresh.metrics.paused = true;
  fresh.start();
  fresh.stop();
  assert(fresh.metrics.sim_ns == stopped_stamp && fresh.metrics.steps == 0);
  assert(world.metrics.sim_ns == stopped_stamp && world.metrics.steps == 2);
  assert(partial->result.stamp == stopped_stamp && partial->result.completed_steps == 2);

  auto unadvanced = command(Op::Step);
  unadvanced->steps = 100;
  unadvanced->capture_states = true;
  unadvanced->states.reserve(256);
  world.submit(unadvanced);
  world.apply_commands();
  assert(unadvanced->phase == 1);
  world.stop();
  assert(unadvanced->phase == 4 && unadvanced->result.interrupted);
  assert(!unadvanced->result.success && !unadvanced->result.applied);
  assert(unadvanced->result.completed_steps == 0 && unadvanced->result.step == 2);
  assert(unadvanced->result.stamp == stopped_stamp && unadvanced->result.paused);
  assert(unadvanced->states.size() == 1);

  auto queued = command(Op::Step);
  world.submit(queued);
  world.stop();
  assert(queued->phase == 3 && !queued->result.interrupted);
}
} // namespace

int main() {
  toggle_without_reset(Kind::FS150, "flight");
  toggle_without_reset(Kind::Scout, "scout");
  toggle_without_reset(Kind::Mecanum, "mecanum");
  batch_and_ready();
  stop_partial_step();
  std::cout << "world management snapshots, non-reset enable, interrupted steps and native startup passed\n";
}
