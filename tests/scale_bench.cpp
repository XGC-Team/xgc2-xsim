#include "io/shm/shm_io.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>

using namespace xsim;

namespace {

void run_bench(int count) {
  const std::string shm_name = "xsim_bench_" + std::to_string(::getpid()) + "_" + std::to_string(count);
  World world(1700000000000000000LL, 2000000, 8000000);
  Sensors sensors(Json::object(), 2);
  RuntimeIO io = make_shm_io(Json{{"shm_name", shm_name}, {"shm_capacity", uint32_t(count + 64)}},
                             world, sensors);

  std::vector<std::shared_ptr<Entity>> entities;
  entities.reserve(count);
  const int side = std::max(1, int(std::ceil(std::sqrt(double(count)))));
  for (int i = 0; i < count; ++i) {
    Config cfg;
    cfg.name = "uav" + std::to_string(i);
    cfg.kind = Kind::FS150;
    cfg.initial = {double(i % side) * 2.0, double(i / side) * 2.0, 2.0};
    cfg.yaw = 0.0;
    cfg.ground_z = 0.0;
    auto e = std::make_shared<Entity>(cfg);
    auto add = std::make_shared<Command>();
    add->op = Op::Add;
    add->prepared = std::make_unique<Prepared>(Prepared{e, prepare_model(cfg)});
    world.submit(add);
    world.apply_commands();

    auto en = std::make_shared<Command>();
    en->op = Op::SetEnabled;
    en->key = {e->id, e->generation};
    en->enabled = true;
    world.submit(en);
    world.apply_commands();

    auto arm = std::make_shared<Command>();
    arm->op = Op::Arm;
    arm->key = {e->id, e->generation};
    arm->arm = true;
    world.submit(arm);
    world.apply_commands();

    ControlSample c;
    c.op = Op::Pva;
    c.key = {e->id, e->generation};
    c.pva.coordinate_frame = 1;
    c.pva.type_mask = 3072;
    c.pva.position[0] = cfg.initial.x();
    c.pva.position[1] = cfg.initial.y();
    c.pva.position[2] = 2.0;
    c.pva.velocity[0] = 0.0;
    c.pva.velocity[1] = 0.0;
    c.pva.velocity[2] = 0.0;
    c.pva.acceleration[0] = 0.0;
    c.pva.acceleration[1] = 0.0;
    c.pva.acceleration[2] = 0.0;
    world.submit_control(*e, c);
    world.boundary();

    auto mode = std::make_shared<Command>();
    mode->op = Op::Mode;
    mode->key = {e->id, e->generation};
    mode->mode = "OFFBOARD";
    world.submit(mode);
    world.apply_commands();

    io.attach_entity(e, Json::object());
    entities.push_back(std::move(e));
  }

  ShmClientView client(shm_name, false);
  auto rates = std::make_shared<TelemetryRates>();

  // Warmup 200 steps (0.4 s)
  for (int s = 0; s < 200; ++s) {
    world.advance();
    world.boundary();
    std::shared_ptr<const Frame> frame;
    if (world.take_frame(frame))
      io.publish_entities(frame, rates);
  }

  // Benchmark 1,000 steps (2.0 s simulated @ 500 Hz, 100 Hz SHM PVA commands + 125 Hz SHM state output)
  constexpr int kSteps = 1000;
  std::vector<double> step_us;
  step_us.reserve(kSteps);
  double total_shm_cmd_us = 0.0;
  double total_shm_out_us = 0.0;
  int cmd_batches = 0, out_frames = 0;

  const auto wall_t0 = std::chrono::steady_clock::now();
  for (int s = 0; s < kSteps; ++s) {
    if (s % 5 == 0) {
      const auto ct0 = std::chrono::steady_clock::now();
      for (int i = 0; i < count; ++i) {
        RawPositionTarget pva{};
        pva.coordinate_frame = 1;
        pva.type_mask = 3072;
        pva.position[0] = double(i % side) * 2.0 + 0.2 * std::sin(0.01 * s);
        pva.position[1] = double(i / side) * 2.0;
        pva.position[2] = 2.0;
        pva.velocity[0] = 1.0 * std::cos(0.01 * s);
        pva.velocity[1] = 0.0;
        pva.velocity[2] = 0.0;
        pva.acceleration[0] = 0.0;
        pva.acceleration[1] = 0.0;
        pva.acceleration[2] = 0.0;
        client.publish_pva(uint32_t(i), pva);
      }
      io.poll_inputs();
      const auto ct1 = std::chrono::steady_clock::now();
      total_shm_cmd_us += std::chrono::duration<double, std::micro>(ct1 - ct0).count();
      ++cmd_batches;
    }
    const auto st0 = std::chrono::steady_clock::now();
    world.boundary();
    world.advance();
    const auto st1 = std::chrono::steady_clock::now();
    step_us.push_back(std::chrono::duration<double, std::micro>(st1 - st0).count());

    std::shared_ptr<const Frame> frame;
    if (world.take_frame(frame)) {
      const auto ot0 = std::chrono::steady_clock::now();
      io.publish_entities(frame, rates);
      const auto ot1 = std::chrono::steady_clock::now();
      total_shm_out_us += std::chrono::duration<double, std::micro>(ot1 - ot0).count();
      ++out_frames;
    }
  }
  const auto wall_t1 = std::chrono::steady_clock::now();
  const double wall_ms = std::chrono::duration<double, std::milli>(wall_t1 - wall_t0).count();

  std::sort(step_us.begin(), step_us.end());
  double sum = 0.0;
  for (double v : step_us) sum += v;
  const double mean_us = sum / step_us.size();
  const double p95_us = step_us[size_t(0.95 * (step_us.size() - 1))];
  const double max_us = step_us.back();
  const double rtf = 2000.0 / wall_ms;

  std::cout << std::fixed << std::setprecision(1)
            << "D=" << std::setw(4) << count
            << " | 2ms_step mean=" << std::setw(6) << mean_us << " us"
            << " p95=" << std::setw(6) << p95_us << " us"
            << " max=" << std::setw(7) << max_us << " us"
            << " | shm_cmd(100Hz)=" << std::setw(6) << (total_shm_cmd_us / std::max(1, cmd_batches)) << " us"
            << " shm_out(125Hz)=" << std::setw(6) << (total_shm_out_us / std::max(1, out_frames)) << " us"
            << " | 2.0s_wall=" << std::setw(6) << wall_ms << " ms"
            << " (" << std::setprecision(2) << rtf << "x real-time)\n";
  sensors.stop();
}

} // namespace

int main() {
  for (int count : {100, 500, 1000}) {
    run_bench(count);
  }
  return 0;
}
