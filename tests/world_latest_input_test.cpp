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
std::shared_ptr<Entity> add(World &world, const char *name, Kind kind = Kind::Mecanum) {
  Config config;
  config.name = name;
  config.kind = kind;
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
void deferred_order_and_lifetime() {
  World world(epoch);
  auto entity = add(world, "scheduled");
  const auto publish_at = [&](double x, int64_t stamp) {
    auto sample = velocity(world, entity, x);
    sample.at = stamp;
    world.submit_control(*entity, sample);
    world.boundary();
  };
  const auto check_step = [&](double expected) {
    world.boundary();
    world.advance();
    assert(world.capture().states.front().velocity.x() == expected);
  };
  const int64_t tick = world.dt;
  // A burst from two logical producers uses the same serialized input mailbox.
  // The earlier deadline survives later ones. Only the latest later value survives.
  publish_at(.25, epoch + 4*tick);
  publish_at(.5, epoch + 8*tick);
  publish_at(.75, epoch + 10*tick);
  for (int i = 0; i < 4; ++i) check_step(0);
  for (int i = 4; i < 10; ++i) check_step(.25);
  check_step(.75);

  auto t = world.metrics.sim_ns.load();
  publish_at(.25, t + 4*tick);
  publish_at(.5, t + 8*tick);
  publish_at(.375, t); // Newer due input supersedes both older future values.
  for (int i = 0; i <= 8; ++i) check_step(.375);

  t = world.metrics.sim_ns;
  publish_at(.25, t + 4*tick);
  publish_at(.5, t + 8*tick);
  publish_at(.875, t + 4*tick); // Same deadline, last publication wins.
  for (int i = 0; i < 4; ++i) check_step(.375);
  for (int i = 4; i <= 8; ++i) check_step(.875);

  t = world.metrics.sim_ns;
  publish_at(.25, t + 4*tick);
  publish_at(.5, t + 8*tick);
  auto disable = command(Op::SetEnabled, entity);
  execute(world, disable);
  auto enable = command(Op::SetEnabled, entity);
  enable->enabled = true;
  execute(world, enable);
  for (int i = 0; i <= 8; ++i) check_step(0);

  t = world.metrics.sim_ns;
  publish_at(.25, t + 4*tick);
  publish_at(.5, t + 8*tick);
  const auto previous_generation = entity->generation.load();
  auto reset = command(Op::Reset, entity);
  reset->prepared = std::make_unique<Prepared>(Prepared{entity, prepare_model(entity->config)});
  execute(world, reset);
  assert(entity->generation != previous_generation);
  for (int i = 0; i <= 8; ++i) check_step(0);

  auto expired = velocity(world, entity, .75);
  expired.at = world.metrics.sim_ns + tick;
  expired.deadline = Clock::now() - std::chrono::seconds(1);
  world.submit_control(*entity, expired);
  check_step(0);
  check_step(0);
}
// Run the real model and its OFFBOARD stream watchdog under a deterministic
// model clock; publishing a future position target is not an FCU receipt.
void periodic_flight_inputs(int64_t lead_ns, uint16_t mask, bool interleave_attitude = false) {
  World world(epoch);
  auto entity = add(world, "periodic-flight", Kind::FS150);
  ControlSample pva;
  pva.op = Op::Pva;
  pva.key = {entity->id, entity->generation};
  pva.pva.coordinate_frame = 1;
  pva.pva.type_mask = mask;
  pva.pva.position[2] = 1;
  world.submit_control(*entity, pva);
  world.boundary();
  auto mode = command(Op::Mode, entity);
  mode->mode = "OFFBOARD";
  execute(world, mode);
  int64_t next = 0;
  int publications = 0;
  for (int i = 0; i < 1500; ++i) {
    const int64_t elapsed = world.metrics.sim_ns - epoch;
    if (elapsed >= next) {
      // Independent PVA/attitude topics share the serialized ROS input queue.
      pva.op = interleave_attitude && publications % 2 ? Op::Attitude : Op::Pva;
      pva.attitude.type_mask = 7;
      pva.attitude.q = Eigen::Quaterniond::Identity();
      pva.attitude.thrust = 0.5;
      pva.at = epoch + elapsed + lead_ns;
      world.submit_control(*entity, pva);
      next = ++publications * 1000000000LL / 30;
    }
    world.boundary();
    world.advance();
  }
  const std::string final_mode(world.capture().states.front().mode.data());
  std::cout << "30 Hz, lead_ns=" << lead_ns << ", mask=" << mask
            << ", mixed_topics=" << interleave_attitude << ", publishes=" << publications << ", mode=" << final_mode << std::endl;
  assert(final_mode == "OFFBOARD");
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
  deferred_order_and_lifetime();
  periodic_flight_inputs(0, 4088);
  periodic_flight_inputs(35000000, 4088);
  periodic_flight_inputs(50000000, 3520);
  periodic_flight_inputs(50000000, 3520, true);
  paused_delivery();
  std::cout << "latest world inputs: replacement, generation/disable/HOLD and paused delivery PASS\n";
}
