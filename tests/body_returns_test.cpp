#include "systems/sensors.hpp"
#include <cstring>
#include <iostream>
#include <stdexcept>

using namespace xsim;
namespace {
void require(bool valid, const char* message) { if (!valid) throw std::runtime_error(message); }
void execute(World& world, const Ticket& command) {
  world.submit(command); world.apply_commands();
  require(command->phase == 2 && command->result.success, "fixture world command failed");
}
std::shared_ptr<Entity> add(World& world, const char* name, const Eigen::Vector3d& position, bool body_returns, double yaw = 0) {
  Config config; config.name = name; config.kind = Kind::Mecanum;
  config.initial = position; config.body_returns = body_returns;
  config.yaw = yaw;
  auto entity = std::make_shared<Entity>(config);
  auto command = std::make_shared<Command>(); command->op = Op::Add;
  command->prepared = std::make_unique<Prepared>(Prepared{entity, prepare_model(config)});
  execute(world, command);
  command = std::make_shared<Command>(); command->op = Op::SetEnabled;
  command->key = {entity->id, entity->generation}; command->enabled = true;
  execute(world, command);
  return entity;
}
std::vector<uint8_t> observe(Sensors& sensors, World& world, const std::shared_ptr<Entity>& scanner,
                             Frame* observed = nullptr) {
  const auto requested_stamp = world.capture().stamp;
  std::map<int64_t, Frame> submitted;
  Sample sample; std::vector<uint8_t> cloud;
  const auto deadline = Clock::now() + std::chrono::seconds(5);
  for (;;) {
    require(Clock::now() < deadline, "native body observation timeout");
    auto frame = world.capture();
    submitted.emplace(frame.stamp, frame);
    sensors.submit_frame(frame);
    if (Sensors::take(scanner->sensor, sample, cloud) && sample.stamp >= requested_stamp) {
      const auto found = submitted.find(sample.stamp);
      require(found != submitted.end(), "sample has no matching submitted body snapshot");
      if (observed) *observed = found->second;
      break;
    }
    world.advance(8000000);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  require(!scanner->sensor->errors, "native sensing error");
  return cloud;
}
bool hit(const std::vector<uint8_t>& cloud, uint64_t target) {
  require(cloud.size() % 16 == 0, "tagged point layout");
  for (std::size_t at = 12; at < cloud.size(); at += 16) {
    int32_t id; std::memcpy(&id, cloud.data() + at, 4);
    if (id == int32_t(target)) return true;
  }
  return false;
}
void move(World& world, const std::shared_ptr<Entity>& target) {
  auto command = std::make_shared<Command>(); command->op = Op::Velocity;
  command->key = {target->id, target->generation}; command->velocity = {0, 1, 0};
  command->received_ns = world.hold().now(); execute(world, command);
  // Actual native model integration, independent of body-return selection.
  for (int i = 0; i < 250; ++i) world.advance(2000000);
}
Eigen::Vector3d position(const Frame& frame, uint64_t id) {
  for (const auto& state : frame.states) if (state.key.id == id) return state.position;
  throw std::runtime_error("missing native body state");
}
Eigen::Vector3d position(World& world, uint64_t id) { return position(world.capture(), id); }
}
int main(int argc, char** argv) {
  require(argc == 2, "pass cpu, gpu or auto");
  const std::string backend = argv[1];
  Sensors sensors(Json{{"obstacles", Json::array({{{"type", "box"}, {"position", {6, 0, .5}},
      {"size", {1, 20, 20}}}})}});
  for (const char* mode : {"raycast", "penetrating"}) {
    for (int selection = 0; selection < 4; ++selection) {
      World world(1700000000000000000LL);
      const double axis = selection == 3 ? -1 : 1;
      auto scanner = add(world, "scanner", {0, 0, .5}, false, selection == 3 ? 3.141592653589793 : 0);
      const bool visible = selection != 0;
      auto front = add(world, "front", {axis * 3, 0, .5}, visible);
      auto near = add(world, "near", {axis * 1.5, 1, .5}, selection == 1);
      auto behind = add(world, "behind", {axis * 9, 0, .5}, visible);
      require(!front->sensor && !near->sensor && !behind->sensor, "targets must have disabled sensors");
      scanner->sensor = sensors.prepare(scanner, {{"backend", backend}, {"mode", mode}, {"rate_hz", 4},
          {"range", 20}, {"min_range", .2}, {"h_res", 360}, {"v_res", 32},
          {"h_fov_deg", 90}, {"v_fov_deg", 60}, {"surface_spacing", .1}, {"world_bodies", true}});
      if (backend != "cpu") require(scanner->sensor->gpu && !scanner->sensor->cpu, "real GPU path required");
      auto first = observe(sensors, world, scanner);
      require(hit(first, front->id) == visible, "front target bodyReturns flag ignored");
      require(hit(first, near->id) == (selection == 1), "mixed target bodyReturns flag ignored");
      require(hit(first, behind->id) == (visible && std::string(mode) == "penetrating"), "static/body occlusion mismatch");
      require(!hit(first, scanner->id), "scanner must exclude itself");
      if (selection == 3 && scanner->sensor->gpu)
        require(scanner->sensor->gpu_static_points.empty(), "body-only fixture must have an empty GPU static result");
      const auto computed = scanner->sensor->computed_scans.load();
      const auto start = position(world, front->id);
      move(world, front);
      const auto moved = position(world, front->id);
      require((moved - start).norm() > .01, "bodyReturns must not disable native dynamics");
      Frame sampled;
      auto second = observe(sensors, world, scanner, &sampled);
      const auto sampled_position = position(sampled, front->id);
      if (!visible) {
        require(first == second && scanner->sensor->computed_scans == computed,
                "default-off neighbour motion invalidated static observation cache");
      } else {
        require(first != second && scanner->sensor->computed_scans > computed, "visible dynamic body retained stale observation");
        bool relocated = false;
        for (std::size_t at = 0; at < second.size(); at += 16) {
          int32_t id; std::memcpy(&id, second.data() + at + 12, 4);
          if (id != int32_t(front->id)) continue;
          float xyz[3]; std::memcpy(xyz, second.data() + at, 12);
          relocated |= std::abs((Eigen::Vector3d(xyz[0], xyz[1], xyz[2]) - sampled_position).norm() - .3) < 1e-5;
        }
        require(relocated, "dynamic return is not on current native sphere");
      }
      require(scanner->sensor->effective_period_ns == scanner->sensor->period, "body observation reduced requested rate");
      if (selection == 2) {
        // Raw sensor rendering uses its ordinary untagged layout, independently
        // of the target visibility query. The scanner itself has bodyReturns off.
        scanner->sensor = sensors.prepare(scanner, {{"backend", backend}, {"mode", mode}, {"rate_hz", 4},
            {"range", 20}, {"min_range", .2}, {"h_res", 360}, {"v_res", 32},
            {"h_fov_deg", 90}, {"v_fov_deg", 60}});
        require(!scanner->sensor->with_bodies, "raw cloud tagging must default off");
        Frame raw_frame;
        const auto raw = observe(sensors, world, scanner, &raw_frame);
        const auto target = position(raw_frame, front->id);
        const std::size_t stride = scanner->sensor->gpu ? 16 : 12;
        bool raw_hit = false;
        for (std::size_t at = 0; at < raw.size(); at += stride) {
          float xyz[3]; std::memcpy(xyz, raw.data() + at, 12);
          raw_hit |= std::abs((Eigen::Vector3d(xyz[0], xyz[1], xyz[2]) - target).norm() - .3) < 1e-5;
        }
        require(raw_hit, "target query was coupled to raw point-cloud tagging");
      }
      std::cout << "PASS " << backend << " effective=" << (scanner->sensor->gpu ? "gpu" : "cpu")
                << " mode=" << mode << " selection=" << selection << " dynamic+cache+occlusion\n";
    }
  }
  sensors.stop();
}
