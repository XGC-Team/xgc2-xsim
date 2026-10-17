#include "core/world.hpp"
#include <cassert>
#include <iostream>

using namespace xsim;
int main() {
  World world(1700000000000000000LL);
  Config config;
  config.name = "pause-arrival";
  config.kind = Kind::Mecanum;
  auto entity = std::make_shared<Entity>(config);
  auto add = std::make_shared<Command>();
  add->op = Op::Add;
  add->prepared = std::make_unique<Prepared>(Prepared{entity, prepare_model(config)});
  world.submit(add);
  world.apply_commands();
  assert(add->result.success);
  auto enable = std::make_shared<Command>();
  enable->op = Op::SetEnabled;
  enable->key = {entity->id, entity->generation};
  enable->enabled = true;
  world.submit(enable);
  world.apply_commands();
  assert(enable->result.success);
  world.start();
  Frame frame;
  assert(world.take_frame(frame));
  auto expected = frame.revision;
  for (unsigned cycle = 0; cycle < 32; ++cycle) {
    ControlSample sample;
    sample.op = Op::Velocity;
    sample.key = {entity->id, entity->generation};
    sample.received_ns = world.hold().now();
    sample.velocity = {.5, 0, 0};
    std::atomic<bool> go{false};
    std::thread producer([&] {
      while (!go.load(std::memory_order_acquire)) std::this_thread::yield();
      world.submit_control(*entity, sample);
    });
    auto pause = std::make_shared<Command>();
    pause->op = Op::Pause;
    go.store(true, std::memory_order_release);
    world.submit(pause);
    producer.join();
    assert(World::wait(pause) && pause->result.success);
    expected += 2; // Both independent inputs were actually applied.
    const auto deadline = Clock::now() + std::chrono::seconds(2);
    bool observed = false;
    while (Clock::now() < deadline && !observed) {
      if (world.take_frame(frame)) observed = frame.revision >= expected;
      else std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    assert(observed && world.metrics.paused && frame.stamp == pause->result.stamp);
    auto resume = std::make_shared<Command>();
    resume->op = Op::Resume;
    world.submit(resume);
    assert(World::wait(resume) && resume->result.success);
    ++expected;
  }
  world.stop();
  std::cout << "latest arrival during Pause never strands the paused owner PASS\n";
}
