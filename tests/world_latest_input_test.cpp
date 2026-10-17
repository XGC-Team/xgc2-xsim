#include "core/world.hpp"
#include <cassert>
#include <iostream>

using namespace xsim;
namespace {
constexpr int64_t epoch = 1700000000000000000LL;
Ticket command(Op op, const std::shared_ptr<Entity> &entity = {}) {
  auto request = std::make_shared<Command>();
  request->op = op;
  if (entity) request->key = {entity->id, entity->generation};
  return request;
}
void execute(World &world, const Ticket &request) {
  world.submit(request);
  world.apply_commands();
  assert(request->phase == 2 && request->result.success);
}
std::shared_ptr<Entity> add(World &world, const char *name) {
  Config config;
  config.name = name;
  config.kind = Kind::Mecanum;
  auto entity = std::make_shared<Entity>(config);
  auto request = command(Op::Add);
  request->prepared = std::make_unique<Prepared>(Prepared{entity, prepare_model(config)});
  execute(world, request);
  auto enable = command(Op::SetEnabled, entity);
  enable->enabled = true;
  execute(world, enable);
  return entity;
}
ControlSample velocity(World &world, const std::shared_ptr<Entity> &entity, double x) {
  ControlSample sample;
  sample.op = Op::Velocity;
  sample.key = {entity->id, entity->generation};
  sample.received_ns = world.hold().now();
  sample.velocity = {x, 0, 0};
  return sample;
}
void latest_and_lifecycle() {
  World world(epoch);
  auto entity = add(world, "latest");
  for (unsigned i = 1; i <= 100; ++i) {
    auto sample = velocity(world, entity, double(i) / 100);
    world.submit_control(*entity, sample);
  }
  assert(world.metrics.steps == 0 && world.metrics.input_misses == 99);
  world.boundary();
  world.advance();
  assert(world.capture().states.front().velocity.x() == 1);

  auto future = velocity(world, entity, .5);
  future.at = world.metrics.sim_ns + 10 * world.dt;
  world.submit_control(*entity, future);
  world.boundary();
  // A future latest value cannot prevent a newer current value being used.
  auto current = velocity(world, entity, .25);
  world.submit_control(*entity, current);
  world.boundary();
  world.advance();
  assert(world.capture().states.front().velocity.x() == .25);

  auto pending = velocity(world, entity, .75);
  pending.at = world.metrics.sim_ns + world.dt;
  world.submit_control(*entity, pending);
  world.boundary();
  auto disable = command(Op::SetEnabled, entity);
  execute(world, disable);
  auto enable = command(Op::SetEnabled, entity);
  enable->enabled = true;
  execute(world, enable);
  world.advance();
  world.boundary();
  world.advance();
  assert(world.capture().states.front().velocity.x() == 0);

  auto stale = velocity(world, entity, .9);
  auto reset = command(Op::Reset, entity);
  reset->prepared = std::make_unique<Prepared>(Prepared{entity, prepare_model(entity->config)});
  execute(world, reset);
  world.submit_control(*entity, stale);
  world.boundary();
  world.advance();
  assert(world.capture().states.front().velocity.x() == 0);

  const auto held = world.hold().domain().engage({entity->public_id}).results();
  assert(held.size() == 1 && held.front().state->held);
  auto refused = velocity(world, entity, 1);
  world.submit_control(*entity, refused);
  world.boundary();
  world.advance();
  assert(world.capture().states.front().velocity.x() == 0);
  assert(world.metrics.hold_refused > 0);
}
void paused_delivery() {
  World world(epoch);
  auto entity = add(world, "paused-latest");
  world.metrics.paused = true;
  world.start();
  Frame frame;
  assert(world.take_frame(frame));
  const auto revision = frame.revision;
  auto sample = velocity(world, entity, .5);
  world.submit_control(*entity, sample);
  const auto deadline = Clock::now() + std::chrono::seconds(2);
  bool applied = false;
  while (Clock::now() < deadline && !applied) {
    if (world.take_frame(frame)) applied = frame.revision > revision;
    else std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  assert(applied && world.metrics.steps == 0 && frame.stamp == epoch);
  auto step = command(Op::Step);
  step->steps = 7;
  world.submit(step);
  assert(World::wait(step) && step->result.success && step->result.completed_steps == 7);
  assert(world.metrics.steps == 7 && world.metrics.sim_ns == epoch + 7 * world.dt);
  world.stop();
}
}
int main() {
  latest_and_lifecycle();
  paused_delivery();
  std::cout << "latest world inputs: replacement, generation/disable/HOLD and paused delivery PASS\n";
}
