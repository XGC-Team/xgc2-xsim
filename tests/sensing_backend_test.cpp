#include "systems/sensors.hpp"
#include <cassert>
#include <cstring>
#include <iostream>
#include <thread>
#include <algorithm>
#include <sys/resource.h>
using namespace xsim;

namespace {
void applied(World& world, const Ticket& command) {
  world.submit(command); world.apply_commands();
  if (command->phase != 2 || !command->result.success)
    throw std::runtime_error("native benchmark setup command failed");
}
Ticket input(const std::shared_ptr<Entity>& entity, Op op) {
  auto command = std::make_shared<Command>();
  command->op = op; command->key = {entity->id, entity->generation};
  return command;
}
Ticket velocity_target(const std::shared_ptr<Entity>& entity) {
  auto command = input(entity, Op::Pva);
  command->pva.coordinate_frame = 1;
  command->pva.type_mask = 1 | 2 | 4 | 64 | 128 | 256 | 1024 | 2048;
  command->pva.velocity[0] = .5;
  return command;
}
double cpu_seconds() {
  rusage usage{}; getrusage(RUSAGE_SELF, &usage);
  return usage.ru_utime.tv_sec + usage.ru_stime.tv_sec +
      (usage.ru_utime.tv_usec + usage.ru_stime.tv_usec) * 1e-6;
}
// Native entities and World integration, without ROS or platform asset records.
// Fixed scene/grid/rates make baseline and candidate use identical moving work.
int benchmark(int argc, char** argv) {
  if (argc != 6 && argc != 7) throw std::invalid_argument("--benchmark cpu|gpu|auto count seconds workers [scene-sensor.json]");
  const std::string backend = argv[2];
  const int count = std::stoi(argv[3]), seconds = std::stoi(argv[4]), workers = std::stoi(argv[5]);
  if (count < 1 || count > 1000 || seconds < 1 || seconds > 60 || workers < 1 || workers > 4)
    throw std::invalid_argument("bounded benchmark arguments required");
  Json scene = {{"obstacles", Json::array({{{"type", "box"}, {"position", {0, 0, -.5}}, {"size", {64, 64, 1}}}})}};
  Json sensor = {{"backend", backend}, {"mode", "raycast"}, {"rate_hz", 10},
      {"h_res", 72}, {"v_res", 16}, {"v_fov_deg", 90}, {"h_fov_deg", 360},
      {"range", 20}, {"translation", {0, 0, 1}}, {"noise_std", 0}};
  if (argc == 7) {
    const auto authored = load_json_object(argv[6]);
    scene = authored.at("scene"); sensor = authored.at("sensor");
    if (!scene.is_object() || !sensor.is_object()) throw std::invalid_argument("scene and sensor must be objects");
    sensor["backend"] = backend;
  }
  Sensors sensors(scene, unsigned(workers));
  World world(1700000000000000000LL);
  std::vector<std::shared_ptr<Entity>> entities;
  for (int i = 0; i < count; ++i) {
    Config config; config.name = "native" + std::to_string(i); config.kind = Kind::FS150;
    config.initial = {double(i % 10) * 2 - 9, double(i / 10) * 2 - 9, 1};
    auto entity = std::make_shared<Entity>(config);
    auto add = std::make_shared<Command>(); add->op = Op::Add;
    add->prepared = std::make_unique<Prepared>(Prepared{entity, prepare_model(config)});
    applied(world, add);
    entity->sensor = sensors.prepare(entity, sensor);
    auto enabled = input(entity, Op::SetEnabled); enabled->enabled = true; applied(world, enabled);
    applied(world, velocity_target(entity));
    entities.push_back(std::move(entity));
  }
  world.advance(2000000);
  for (const auto& entity : entities) {
    auto arm = input(entity, Op::Arm); arm->arm = true; applied(world, arm);
    auto mode = input(entity, Op::Mode); mode->mode = "OFFBOARD"; applied(world, mode);
  }
  world.start();
  const auto warm_end = Clock::now() + std::chrono::seconds(2);
  const auto end = warm_end + std::chrono::seconds(seconds);
  auto next_input = Clock::now();
  bool measured = false;
  int64_t start_sim = 0;
  double start_cpu = 0;
  Clock::time_point start_wall;
  std::vector<Eigen::Vector3d> initial(size_t(count), Eigen::Vector3d::Zero());
  std::vector<uint64_t> clouds(size_t(count), 0), computed(size_t(count), 0), hits(size_t(count), 0), grows(size_t(count), 0);
  std::vector<double> ages; ages.reserve(size_t(count) * size_t(seconds) * 20);
  std::shared_ptr<const Frame> last;
  uint64_t bytes = 0, empty = 0;
  while (Clock::now() < end) {
    if (Clock::now() >= next_input) {
      for (const auto& entity : entities) world.submit(velocity_target(entity));
      next_input = Clock::now() + std::chrono::milliseconds(100);
    }
    std::shared_ptr<const Frame> frame;
    if (world.take_frame(frame)) { sensors.submit_frame(frame); last = std::move(frame); }
    if (!measured && Clock::now() >= warm_end && last) {
      measured = true; start_sim = world.metrics.sim_ns; start_cpu = cpu_seconds(); start_wall = Clock::now();
      for (const auto& state : last->states) initial.at(size_t(state.key.id - 1)) = state.position;
      for (size_t i = 0; i < entities.size(); ++i) {
        computed[i] = entities[i]->sensor->computed_scans; hits[i] = entities[i]->sensor->cache_hits; grows[i] = entities[i]->sensor->payload_grows;
      }
    }
    for (size_t i = 0; i < entities.size(); ++i) {
      Sample sample; std::shared_ptr<const SensorPayload> payload;
      if (Sensors::take_shared(entities[i]->sensor, sample, payload) && measured) {
        ++clouds[i]; bytes += payload->data.size(); empty += payload->data.empty();
        ages.push_back(std::max<int64_t>(0, world.metrics.sim_ns.load() - sample.stamp) * 1e-6);
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  const double wall = std::chrono::duration<double>(Clock::now() - start_wall).count();
  const double cpu = cpu_seconds() - start_cpu;
  const double rtf = (world.metrics.sim_ns.load() - start_sim) * 1e-9 / wall;
  world.stop(); sensors.stop();
  std::sort(ages.begin(), ages.end());
  double min_move = std::numeric_limits<double>::infinity();
  for (const auto& state : last->states) min_move = std::min(min_move, (state.position - initial.at(size_t(state.key.id - 1))).norm());
  uint64_t scans = 0, cache_hits = 0, payload_grows = 0;
  for (size_t i = 0; i < entities.size(); ++i) {
    scans += entities[i]->sensor->computed_scans - computed[i]; cache_hits += entities[i]->sensor->cache_hits - hits[i]; payload_grows += entities[i]->sensor->payload_grows - grows[i];
  }
  rusage usage{}; getrusage(RUSAGE_SELF, &usage);
  std::cout << Json({{"mode", "native-moving-fs150"}, {"count", count}, {"workers", workers},
      {"scene", scene}, {"sensor_config", sensor}, {"warmup_seconds", 2}, {"duration_seconds", wall},
      {"rtf", rtf}, {"cpu_cores", cpu / wall}, {"max_rss_kib", usage.ru_maxrss},
      {"min_displacement_m", min_move}, {"min_cloud_hz", *std::min_element(clouds.begin(), clouds.end()) / wall},
      {"max_cloud_hz", *std::max_element(clouds.begin(), clouds.end()) / wall}, {"point_bytes", bytes}, {"empty_clouds", empty},
      {"cloud_age_p95_ms", ages.empty() ? 0 : ages[size_t(.95 * (ages.size() - 1))]}, {"cloud_age_max_ms", ages.empty() ? 0 : ages.back()},
      {"computed_scans", scans}, {"cache_hits", cache_hits}, {"payload_grows", payload_grows}, {"sensors", sensors.status()}}).dump() << '\n';
  return measured && min_move > .05 && scans > 0 && empty == 0 && std::all_of(clouds.begin(), clouds.end(), [](uint64_t n) { return n > 0; }) ? 0 : 2;
}
}
int main(int argc, char** argv) {
  if (argc > 1 && std::string(argv[1]) == "--benchmark") return benchmark(argc, argv);
  for (const auto backend : {"cpu", "gpu", "auto"}) for (const bool through : {false, true}) {
    if(argc==2 && std::string(backend)!=argv[1]) continue;
    Sensors sensors(Json{{"obstacles", Json::array({
      {{"type","box"},{"position",{5,0,0}},{"size",{1,8,8}}},
      {{"type","box"},{"position",{9,0,0}},{"size",{1,2,2}}}})}});
    std::vector<std::shared_ptr<Entity>> robots;
    for (int id=1; id<=20; ++id) {
      auto e=std::make_shared<Entity>(parse_entity({{"name","uav"+std::to_string(id)},{"kind","scout"}}));
      e->id=id;e->alive=e->enabled=true;e->generation=1;
      e->sensor=sensors.prepare(e,{{"backend",backend},{"mode",through?"penetrating":"raycast"},
        {"range",20},{"min_range",.2},{"h_res",120},{"v_res",120},
        {"h_fov_deg",60},{"v_fov_deg",60},{"surface_spacing",id==20?.2:.1}});
      robots.push_back(e);
    }
    int64_t stamp=1000000000LL;
    for (int round=0;round<3;++round) {
      std::vector<bool> received(robots.size(),false);
      size_t remaining=robots.size();
      const auto end=Clock::now()+std::chrono::seconds(10);
      while (remaining) {
        for(size_t i=0;i<robots.size();++i) if(!received[i]) {
          const auto&e=robots[i];State p;p.entity=e;p.key={e->id,e->generation};p.enabled=true;p.stamp=stamp;
          p.position={round==2?.05:0,0,0};sensors.submit(p);
          Sample sample;std::vector<uint8_t> data;
          if(!Sensors::take(e->sensor,sample,data)) continue;
          if(sample.position.x() != (round==2?.05:0)) continue;
          const size_t stride=e->sensor->gpu?16:12;assert(!data.empty() && data.size()%stride==0);
          size_t behind=0;for(size_t at=0;at<data.size();at+=stride){float x;std::memcpy(&x,data.data()+at,4);if(x>7)++behind;}
          assert((behind>0)==through);received[i]=true;--remaining;
        }
        if(Clock::now()>=end){std::cerr<<backend<<" round="<<round<<" "<<sensors.status().dump()<<std::endl;return 2;}
        stamp+=20000000LL;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      stamp+=1000000000LL;
    }
    for(const auto&e:robots) assert(e->sensor->cache_hits>=1 && e->sensor->computed_scans==2);
    std::cout<<"PASS XSim "<<backend<<" effective="<<(robots[0]->sensor->gpu?"gpu":"cpu")<<" through="<<through<<" 20 robots, two shared geometry policies, pose cache\n";
  }
}
