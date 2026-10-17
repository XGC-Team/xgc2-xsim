#include "core/world.hpp"
#include <xgc2_math/control/delayed_planar_velocity.hpp>
#include <cassert>
#include <exception>
#include <iostream>

using namespace xsim;
namespace {
void execute(World &world, const Ticket &request) {
  world.submit(request);
  world.apply_commands();
  assert(request->phase == 2 && request->result.success);
}
void submit_velocity(World &world, Entity &entity, double forward) {
  ControlSample sample;
  sample.op = Op::Velocity;
  sample.key = {entity.id, entity.generation};
  sample.received_ns = world.hold().now();
  sample.velocity = {forward, 0, 0};
  world.submit_control(entity, sample);
}
} // namespace

int main() {
  constexpr int64_t epoch = 1700000000000000000LL;
  constexpr auto step = PhysicsClock::minimum_step;
  World world(epoch, step);
  Config config;
  config.name = "latest-scout-exception";
  config.kind = Kind::Scout;
  auto entity = std::make_shared<Entity>(config);
  auto add = std::make_shared<Command>();
  add->op = Op::Add;
  add->prepared = std::make_unique<Prepared>(Prepared{entity, prepare_model(config)});
  execute(world, add);
  auto enable = std::make_shared<Command>();
  enable->op = Op::SetEnabled;
  enable->key = {entity->id, entity->generation};
  enable->enabled = true;
  execute(world, enable);

  // Distinct 1 us simulation timestamps fill the real Scout delay queue before
  // its first command can be released. Bound the fixture by the public delay,
  // without copying or exposing the Math implementation's private capacity.
  const xgc2_math::DelayedPlanarVelocityParameters response;
  const auto before_first_release = int64_t(response.delay_s * 1e9) / step;
  assert(before_first_release > 1);
  bool rejected = false;
  for (int64_t i = 0; i < before_first_release; ++i) {
    submit_velocity(world, *entity, .5);
    try {
      world.boundary();
    } catch (const std::exception &error) {
      std::cerr << "latest input escaped the World exception boundary: "
                << error.what() << '\n';
      return 1;
    }
    if (world.metrics.input_misses != 0) {
      assert(i > 0 && world.metrics.input_misses == 1);
      rejected = true;
      break;
    }
    world.advance();
  }
  assert(rejected);
  // The failed sample was consumed once, rather than retried each boundary.
  world.boundary();
  assert(world.metrics.input_misses == 1);

  // Releasing queued targets makes room; the same World still accepts and
  // evolves a subsequent latest input after the model exception.
  world.advance(int64_t(response.delay_s * 1e9));
  submit_velocity(world, *entity, .25);
  world.boundary();
  assert(world.metrics.input_misses == 1);
  world.advance(10000000);
  const auto frame = world.capture();
  assert(frame.states.size() == 1);
  assert(frame.states.front().position.allFinite());
  assert(frame.states.front().velocity.allFinite());
  assert(frame.states.front().velocity.x() > 0);
  std::cout << "latest Scout model exception is isolated and later inputs recover PASS\n";
}
