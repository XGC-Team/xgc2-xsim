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
    auto move = command(Op::Velocity, {entity->id, entity->generation});
    move->velocity = {1, .2, .1};
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

void fcu_reboot() {
  World world(1700000000000000000LL);
  auto first = add(world, Kind::FS150, "reboot");
  auto second = add(world, Kind::FS150, "independent");
  enabled(world, first, true);
  enabled(world, second, true);
  for (unsigned i = 0; i != 20; ++i) world.advance();
  const Key key{first->id, first->generation};
  const auto input = [&](Key owner) {
    auto request = command(Op::Pva, owner);
    request->pva.coordinate_frame = 1;
    request->pva.type_mask = 3527;
    request->pva.velocity[0] = 1.0;
    return request;
  };
  execute(world, input(key));
  auto offboard = command(Op::Mode, key);
  offboard->mode = "OFFBOARD";
  execute(world, offboard);
  assert(offboard->result.success);
  auto future = input(key);
  future->at = world.metrics.sim_ns + 10000000;
  world.submit(future);
  auto other_future = input({second->id, second->generation});
  other_future->at = future->at;
  world.submit(other_future);
  // Cover both an already-deferred mailbox value and its new unread value.
  world.submit_control(*first, *future);
  world.boundary();
  world.submit_control(*first, *future);
  const auto before = entity_state(world, first->id);
  const auto other_before = entity_state(world, second->id);
  const auto stamp = world.metrics.sim_ns.load();
  const auto steps = world.metrics.steps.load();
  auto reboot = command(Op::Reboot, key);
  reboot->capture_state = true;
  execute(world, reboot);
  assert(reboot->result.success && reboot->result.has_state && reboot->result.enabled);
  const auto after = reboot->result.state;
  assert(after.key.id == key.id && after.key.generation == key.generation);
  assert(first->alive && first->enabled && first->generation == key.generation);
  assert(after.position == before.position && after.velocity == before.velocity);
  assert(after.orientation.coeffs() == before.orientation.coeffs());
  assert(after.omega == before.omega && after.rotors == before.rotors);
  assert(after.control.allocation.target_rotor_speed.isZero());
  assert(!after.armed && std::string(after.mode.data()) == "POSCTL");
  assert(world.metrics.sim_ns == stamp && world.metrics.steps == steps);
  const auto other_after = entity_state(world, second->id);
  assert(other_after.position == other_before.position && other_after.velocity == other_before.velocity);
  assert(other_after.key.generation == other_before.key.generation);
  assert(future->phase == 3 && other_future->phase == 0);
  for (unsigned i = 0; i != 6; ++i) { world.advance(); world.boundary(); }
  assert(other_future->phase == 2 && other_future->result.success);
  auto mode = command(Op::Mode, key);
  mode->mode = "OFFBOARD";
  execute(world, mode);
  assert(!mode->result.success); // Neither old mailbox nor deferred request survived.
  execute(world, input(key));
  mode = command(Op::Mode, key);
  mode->mode = "OFFBOARD";
  execute(world, mode);
  assert(mode->result.success);
  auto arm = command(Op::Arm, key);
  arm->arm = true;
  execute(world, arm);
  assert(arm->result.success);
  auto pending = input(key);
  pending->at = world.metrics.sim_ns + 10000000;
  world.submit(pending);
  const auto armed_before = entity_state(world, first->id);
  auto refused = command(Op::Reboot, key);
  execute(world, refused);
  assert(!refused->result.success && refused->result.reason == 2);
  assert(pending->phase == 0); // Refusal does not discard the armed FCU's controls.
  const auto armed_after = entity_state(world, first->id);
  assert(armed_after.armed && armed_after.mode == armed_before.mode);
  assert(armed_after.position == armed_before.position && armed_after.velocity == armed_before.velocity);
  assert(armed_after.control.allocation.motor_commands == armed_before.control.allocation.motor_commands);
  auto removal = command(Op::Remove, key);
  execute(world, removal);
  assert(removal->result.success && !first->alive && world.capture().states.size() == 1);
  first.reset();
  removal->retired.reset();
  assert(after.entity.expired());
}
} // namespace

int main() {
  toggle_without_reset(Kind::FS150, "flight");
  toggle_without_reset(Kind::Scout, "scout");
  toggle_without_reset(Kind::Mecanum, "mecanum");
  batch_and_ready();
  stop_partial_step();
  fcu_reboot();
  std::cout << "world management snapshots, non-reset enable, interrupted steps and native startup passed\n";
}
