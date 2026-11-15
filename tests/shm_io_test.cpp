#include "io/shm/shm_io.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>

using namespace xsim;

namespace {

std::shared_ptr<Entity> spawn_fs150(World &world, RuntimeIO &io,
                                    const std::string &name,
                                    const Eigen::Vector3d &pos) {
  Config cfg;
  cfg.name = name;
  cfg.kind = Kind::FS150;
  cfg.initial = pos;
  cfg.yaw = 0.0;
  cfg.ground_z = 0.0;
  auto entity = std::make_shared<Entity>(cfg);
  auto add = std::make_shared<Command>();
  add->op = Op::Add;
  add->prepared = std::make_unique<Prepared>(Prepared{entity, prepare_model(cfg)});
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

  io.attach_entity(entity, Json::object());
  return entity;
}

} // namespace

int main() {
  const std::string shm_name = "xsim_test_" + std::to_string(::getpid());
  World world(1700000000000000000LL, 2000000, 8000000);
  Sensors sensors(Json::object(), 1);
  Json cfg{{"shm_name", shm_name}, {"shm_capacity", 128}};
  RuntimeIO io = make_shm_io(cfg, world, sensors);

  std::atomic<uint64_t> wakes{0};
  io.bind_input_wake([&] { wakes.fetch_add(1, std::memory_order_relaxed); });
  io.start_outputs();

  constexpr int kNumUavs = 32;
  std::vector<std::shared_ptr<Entity>> uavs;
  for (int i = 0; i < kNumUavs; ++i) {
    uavs.push_back(spawn_fs150(world, io, "uav" + std::to_string(i),
                               Eigen::Vector3d(double(i) * 1.5, 0.0, 1.0)));
  }

  auto rates = std::make_shared<TelemetryRates>();
  auto f0 = std::make_shared<const Frame>(world.capture());
  io.publish_entities(f0, rates);

  ShmClientView client(shm_name, false);
  assert(client.active_count() == kNumUavs);
  assert(client.find_slot("uav0") == 0);
  assert(client.find_slot("uav31") == 31);

  ShmEntityStateSnapshot snap{};
  assert(client.read_state(5, snap));
  assert(std::abs(snap.position[0] - 7.5) < 1e-6);
  assert(std::abs(snap.position[2] - 1.0) < 1e-6);
  assert(snap.enabled == 1);
  assert(snap.armed == 0);

  // Arm, stream PVA hover/climb setpoint, and switch to OFFBOARD via SHM!
  for (int i = 0; i < kNumUavs; ++i) {
    assert(client.publish_arm(i, true));
  }
  assert(io.inputs_pending());
  io.poll_inputs();
  world.boundary();

  for (int i = 0; i < kNumUavs; ++i) {
    RawPositionTarget pva{};
    pva.coordinate_frame = 1;
    pva.type_mask = 3072; // p + v + a finite, yaw/yaw_rate ignored
    pva.position[0] = double(i) * 1.5;
    pva.position[1] = 0.0;
    pva.position[2] = 2.5;
    pva.velocity[0] = 0.0;
    pva.velocity[1] = 0.0;
    pva.velocity[2] = 0.5;
    pva.acceleration[0] = 0.0;
    pva.acceleration[1] = 0.0;
    pva.acceleration[2] = 2.0;
    assert(client.publish_pva(i, pva));
  }
  io.poll_inputs();
  world.boundary();

  for (int i = 0; i < kNumUavs; ++i) {
    assert(client.publish_mode(i, "OFFBOARD"));
  }
  io.poll_inputs();
  world.boundary();

  // Step 0.5 s (250 * 2 ms) of 6-DoF + PX4 simulation.
  for (int step = 0; step < 250; ++step) {
    world.advance();
    world.boundary();
  }
  auto f1 = std::make_shared<const Frame>(world.capture());
  io.publish_entities(f1, rates);

  for (int i = 0; i < kNumUavs; ++i) {
    ShmEntityStateSnapshot s{};
    assert(client.read_state(i, s));
    assert(s.armed == 1);
    assert(std::string(s.mode) == "OFFBOARD");
    assert(s.position[2] > 1.05);
  }

  io.stop_outputs();
  sensors.stop();
  std::cout << "PASS xsim_shm_io_test (" << kNumUavs << " UAVs armed + OFFBOARD climb via lock-free SHM)\n";
  return 0;
}
