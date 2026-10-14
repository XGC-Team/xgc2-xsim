// The sensing pipeline against the xgc2_math::scene geometry it consumes: every payload the worker threads
// produce must equal what the scene API returns for the same scene, sensor model, pose and scan index, in the
// wire layout the ROS boundary publishes.
#include "systems/sensors.hpp"
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include <thread>
#include <xgc2_math/scene.hpp>

using namespace xsim;
namespace scene = xgc2_math::scene;
namespace {
constexpr int64_t epoch = 1700000000000000000LL;
constexpr int64_t period = 100000000;
using Bytes = std::vector<uint8_t>;

std::shared_ptr<Entity> entity(uint64_t id, const std::string &name, bool body_returns = false) {
  auto e = std::make_shared<Entity>(parse_entity({{"name", name}, {"kind", "scout"}, {"body_returns", body_returns}}));
  e->id = id;
  e->alive = e->enabled = true;
  e->generation = 1;
  return e;
}
State state(const std::shared_ptr<Entity> &e, int64_t stamp, const Eigen::Vector3d &position,
            const Eigen::Quaterniond &orientation = Eigen::Quaterniond::Identity()) {
  State s;
  s.entity = e;
  s.key = {e->id, e->generation};
  s.enabled = true;
  s.body_returns = e->config.body_returns;
  s.stamp = stamp;
  s.position = position;
  s.orientation = orientation;
  return s;
}
Eigen::Quaterniond yaw(double radians) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(radians, Eigen::Vector3d::UnitZ()));
}
// The [w,x,y,z] order of the scene's obstacles and sensor mounts, and back.
Json wxyz(const Eigen::Quaterniond &q) { return Json::array({q.w(), q.x(), q.y(), q.z()}); }
Eigen::Quaterniond mounted(const Eigen::Quaterniond &q) {
  const Json j = wxyz(q);
  return Eigen::Quaterniond(j[0].get<double>(), j[1].get<double>(), j[2].get<double>(), j[3].get<double>())
      .normalized();
}
Json vector(const Eigen::Vector3d &v) { return Json::array({v.x(), v.y(), v.z()}); }

// The world every case scans: one obstacle of each kind.
const std::vector<Eigen::Vector3d> cube = {{-.5, -.5, -.5}, {.5, -.5, -.5}, {-.5, .5, -.5}, {.5, .5, -.5},
                                           {-.5, -.5, .5}, {.5, -.5, .5}, {-.5, .5, .5}, {.5, .5, .5}};
Json vertices() {
  Json list = Json::array();
  for (const auto &v : cube) list.push_back(vector(v));
  return list;
}
Json world(const Json &extra = Json::object()) {
  Json j = {{"obstacles", Json::array({
      {{"type", "box"}, {"position", {6, 0, .5}}, {"size", {1, 6, 3}}, {"orientation", wxyz(yaw(.35))}},
      {{"type", "sphere"}, {"position", {0, 6, 1}}, {"radius", 1.5}},
      {{"type", "cylinder"}, {"position", {-5, -1, 1}}, {"radius", .8}, {"height", 3}},
      {{"type", "capsule"}, {"position", {1, -6, 1}}, {"radius", .6}, {"height", 2}},
      {{"type", "convex"}, {"position", {-4, 5, 1}}, {"vertices", vertices()}}})}};
  for (auto it = extra.begin(); it != extra.end(); ++it) j[it.key()] = it.value();
  return j;
}
std::vector<scene::Obstacle> obstacles() {
  using O = scene::Obstacle;
  return {O::box({6, 0, .5}, {1, 6, 3}, mounted(yaw(.35))), O::sphere({0, 6, 1}, 1.5),
          O::cylinder({-5, -1, 1}, .8, 3), O::capsule({1, -6, 1}, .6, 2), O::convexHull(cube, {-4, 5, 1})};
}

Sample receive(const std::shared_ptr<Sensor> &sensor, Bytes &cloud, Bytes &beams) {
  const auto deadline = Clock::now() + std::chrono::seconds(5);
  Sample sample;
  while (!Sensors::take(sensor, sample, cloud, &beams)) {
    assert(Clock::now() < deadline);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return sample;
}
// A private sensor of the scene API over its own compile of the world, which is what a worker's sensor must equal.
std::unique_ptr<scene::WorldLidar> reference(const scene::SensorConfig &config, double spacing = .1,
                                             bool buried = false) {
  auto lidar = std::make_unique<scene::WorldLidar>(config);
  lidar->setScene(std::make_shared<scene::LidarScene>(obstacles(), spacing, buried));
  return lidar;
}
float number(const Bytes &data, size_t at) {
  float value;
  std::memcpy(&value, data.data() + at, sizeof value);
  return value;
}
int32_t integer(const Bytes &data, size_t at) {
  int32_t value;
  std::memcpy(&value, data.data() + at, sizeof value);
  return value;
}

// A mounted raycast sensor on a moving robot returns the scene API's cloud for the mounted pose, and a noisy sensor
// follows its own noise stream over its scans.
void raycast_and_noise() {
  Sensors sensors(world(), 2);
  auto robot = entity(1, "robot");
  const Eigen::Vector3d translation(.2, -.1, .3);
  const auto rotation = mounted(yaw(-.4));
  robot->sensor = sensors.prepare(robot, {{"h_res", 72}, {"v_res", 8}, {"h_fov_deg", 120}, {"v_fov_deg", 20},
      {"range", 15}, {"min_range", .3}, {"translation", vector(translation)}, {"rotation", wxyz(yaw(-.4))},
      {"rate_hz", 10}});
  const Json noisy_config = {{"h_res", 90}, {"v_res", 4}, {"range", 12}, {"noise_std", .05}, {"seed", 7},
                             {"rate_hz", 10}};
  auto noisy = entity(2, "noisy");
  noisy->sensor = sensors.prepare(noisy, noisy_config);
  scene::SensorConfig plain;
  plain.h_res = 72;
  plain.v_res = 8;
  plain.h_fov_deg = 120;
  plain.v_fov_deg = 20;
  plain.range = 15;
  plain.min_range = .3;
  scene::SensorConfig noise;
  noise.h_res = 90;
  noise.v_res = 4;
  noise.range = 12;
  noise.noise_std = .05;
  noise.seed = 7;
  auto expected = reference(plain);
  auto noise_expected = reference(noise);

  Bytes cloud, beams, want;
  for (int k = 0; k < 4; ++k) {
    const auto stamp = epoch + k * period;
    const auto pose = state(robot, stamp, {1. + .5 * k, 2. - .25 * k, .5}, yaw(.3 + .1 * k));
    const auto still = state(noisy, stamp, {-1, 1, .5});
    Frame frame;
    frame.stamp = stamp;
    frame.states = {pose, still};
    sensors.submit_frame(frame);
    auto sample = receive(robot->sensor, cloud, beams);
    const Eigen::Vector3d position = pose.position + pose.orientation * translation;
    const Eigen::Quaterniond attitude = pose.orientation * rotation;
    assert(sample.stamp == stamp && sample.key.id == 1);
    assert(sample.position == position && sample.orientation.coeffs() == attitude.coeffs());
    expected->scanInto(position, attitude, {}, false, &want);
    assert(!want.empty() && cloud == want);
    assert(beams.empty());
    receive(noisy->sensor, cloud, beams);
    noise_expected->scanInto(still.position, still.orientation, {}, false, &want);
    assert(!want.empty() && cloud == want);
  }
  // The noise of two sensors with one seed starts alike and is not shared; another seed is another stream.
  auto twin = entity(3, "twin"), other_seed = entity(4, "other_seed");
  twin->sensor = sensors.prepare(twin, noisy_config);
  Json different = noisy_config;
  different["seed"] = 8;
  other_seed->sensor = sensors.prepare(other_seed, different);
  Frame frame;
  frame.stamp = epoch + 10 * period;
  frame.states = {state(twin, frame.stamp, {-1, 1, .5}), state(other_seed, frame.stamp, {-1, 1, .5})};
  sensors.submit_frame(frame);
  Bytes first, second;
  receive(twin->sensor, first, beams);
  receive(other_seed->sensor, second, beams);
  reference(noise)->scanInto({-1, 1, .5}, Eigen::Quaterniond::Identity(), {}, false, &want);
  assert(first == want && second != want);
  sensors.stop();
}

// The beam records and the point cloud of a sensor that observes the other robots: the layout of the ROS messages
// (x y z dx dy dz range hit vehicle_id, 36 bytes; points x y z vehicle_id, 16 bytes).
void beams_and_bodies() {
  Sensors sensors(world(), 2);
  auto source = entity(1, "source"), body = entity(2, "body", true);
  source->sensor = sensors.prepare(source, {{"h_res", 48}, {"v_res", 6}, {"h_fov_deg", 90}, {"v_fov_deg", 20},
      {"range", 14}, {"world_bodies", true}, {"publish_beams", true}, {"rate_hz", 10}});
  scene::SensorConfig c;
  c.h_res = 48;
  c.v_res = 6;
  c.h_fov_deg = 90;
  c.v_fov_deg = 20;
  c.range = 14;
  auto lidar = reference(c);
  Frame frame;
  frame.stamp = epoch;
  frame.states = {state(source, epoch, {0, 0, .5}), state(body, epoch, {3, .2, .5})};
  sensors.submit_frame(frame);
  Bytes cloud, beams;
  receive(source->sensor, cloud, beams);
  scene::BeamScan scan;
  lidar->scanBeamsInto({0, 0, .5}, Eigen::Quaterniond::Identity(), {{2, {3, .2, .5}, .3}}, &scan);
  assert(scan.size() == 48 * 6 && beams.size() == scan.size() * 36);
  size_t hits = 0, on_body = 0;
  for (size_t i = 0; i < scan.size(); ++i) {
    const size_t at = i * 36;
    for (int k = 0; k < 3; ++k) {
      assert(number(beams, at + 4 * size_t(k)) == float(scan.origin[k]));
      assert(number(beams, at + 12 + 4 * size_t(k)) == float(scan.direction[i][k]));
    }
    assert(number(beams, at + 24) == float(scan.range[i]));
    assert(number(beams, at + 28) == (scan.hit[i] ? 1.f : 0.f));
    assert(integer(beams, at + 32) == scan.vehicle_id[i]);
    if (!scan.hit[i]) continue;
    const auto point = scan.point(i);
    const size_t out = hits * 16;
    assert(out + 16 <= cloud.size());
    for (int k = 0; k < 3; ++k) assert(number(cloud, out + 4 * size_t(k)) == float(point[k]));
    assert(integer(cloud, out + 12) == scan.vehicle_id[i]);
    on_body += scan.vehicle_id[i] == 2;
    ++hits;
  }
  // The other robot is seen, the sensor's own body is not an obstacle to itself, and every hit is accounted for.
  assert(hits > 0 && on_body > 0 && cloud.size() == hits * 16);
  sensors.stop();
}

// Penetrating sensors sample the scene at their own spacing and buried policy; depth sensors are pinhole cameras.
void penetrating_and_depth() {
  Sensors sensors(world({{"surface_spacing", .1}, {"keep_buried", false}}), 2);
  struct Case {
    std::string name;
    Json config;
    scene::SensorConfig model;
    double spacing;
    bool buried;
  };
  std::vector<Case> cases;
  {
    Case c{"world_policy", {{"mode", "penetrating"}, {"range", 9}, {"rate_hz", 10}}, {}, .1, false};
    c.model.mode = scene::SensorConfig::kPenetrating;
    c.model.range = 9;
    c.model.surface_spacing = .1;
    cases.push_back(c);
  }
  {
    Case c{"own_policy", {{"mode", "penetrating"}, {"range", 9}, {"surface_spacing", .25}, {"keep_buried", true},
                          {"rate_hz", 10}}, {}, .25, true};
    c.model.mode = scene::SensorConfig::kPenetrating;
    c.model.range = 9;
    c.model.surface_spacing = .25;
    c.model.penetrating_keep_buried = true;
    cases.push_back(c);
  }
  {
    Case c{"heading_crop", {{"mode", "penetrating"}, {"range", 12}, {"heading_crop", true}, {"heading_cos_min", .5},
                            {"vertical_slab_tan", .4}, {"rate_hz", 10}}, {}, .1, false};
    c.model.mode = scene::SensorConfig::kPenetrating;
    c.model.range = 12;
    c.model.surface_spacing = .1;
    c.model.penetrating_heading_crop = true;
    c.model.heading_cos_min = .5;
    c.model.vertical_slab_tan = .4;
    cases.push_back(c);
  }
  {
    Case c{"depth", {{"mode", "depth"}, {"width", 40}, {"height", 30}, {"h_fov_deg", 90}, {"range", 16},
                     {"rate_hz", 10}}, {}, .1, false};
    c.model.mode = scene::SensorConfig::kDepthFrustum;
    c.model.width = 40;
    c.model.height = 30;
    c.model.h_fov_deg = 90;
    c.model.range = 16;
    cases.push_back(c);
  }
  uint64_t id = 10;
  for (auto &c : cases) {
    auto robot = entity(id++, c.name);
    robot->sensor = sensors.prepare(robot, c.config);
    const auto pose = state(robot, epoch, {-1, .5, .5}, yaw(.2));
    sensors.submit(pose);
    Bytes cloud, beams, want;
    receive(robot->sensor, cloud, beams);
    reference(c.model, c.spacing, c.buried)->scanInto(pose.position, pose.orientation, {}, false, &want);
    assert(!want.empty() && cloud == want);
    assert(beams.empty());
  }
  // A penetrating model has no beams, whatever the request.
  bool refused = false;
  try {
    sensors.prepare(entity(30, "beamless"), {{"mode", "penetrating"}, {"publish_beams", true}});
  } catch (const std::invalid_argument &) { refused = true; }
  assert(refused);
  sensors.stop();
}

// The scene document form (obstacles made of placed parts) and the plain obstacle list describe one world.
void scene_forms() {
  const auto part = [](const Json &position, const Json &geometry) {
    return Json{{"pose", {{"position", position}}}, {"geometry", geometry}};
  };
  const Json document = {{"document", {{"obstacles", Json::array({
      {{"id", "wall"}, {"pose", {{"position", {5, 0, 0}}}},
       {"parts", Json::array({part({1, 0, .5}, {{"type", "box"}, {"size", {1, 6, 3}}})})}},
      {{"id", "pillar"}, {"pose", {{"position", {-5, -1, 0}}}},
       {"parts", Json::array({
           part({0, 0, 1}, {{"type", "cylinder"}, {"radius", .8}, {"height", 3}}),
           part({0, 3, 1}, {{"type", "sphere"}, {"radius", 1.5}}),
           part({0, -3, 1}, {{"type", "capsule"}, {"radius", .6}, {"height", 2}}),
           part({5, 0, 1}, {{"type", "convex"}, {"vertices", vertices()}})})}}})}}}};
  const Json plain = {{"obstacles", Json::array({
      {{"type", "box"}, {"position", {6, 0, .5}}, {"size", {1, 6, 3}}},
      {{"type", "cylinder"}, {"position", {-5, -1, 1}}, {"radius", .8}, {"height", 3}},
      {{"type", "sphere"}, {"position", {-5, 2, 1}}, {"radius", 1.5}},
      {{"type", "capsule"}, {"position", {-5, -4, 1}}, {"radius", .6}, {"height", 2}},
      {{"type", "convex"}, {"position", {0, -1, 1}}, {"vertices", vertices()}}})}};
  Sensors from_document(document, 1), from_list(plain, 1);
  assert(!from_list.reference_cloud().empty());
  assert(from_document.reference_cloud() == from_list.reference_cloud());
  auto a = entity(1, "a"), b = entity(2, "b");
  const Json config = {{"h_res", 120}, {"v_res", 6}, {"range", 18}, {"rate_hz", 10}};
  a->sensor = from_document.prepare(a, config);
  b->sensor = from_list.prepare(b, config);
  from_document.submit(state(a, epoch, {0, 0, .8}));
  from_list.submit(state(b, epoch, {0, 0, .8}));
  Bytes first, second, beams;
  receive(a->sensor, first, beams);
  receive(b->sensor, second, beams);
  assert(!first.empty() && first == second);
  from_document.stop();
  from_list.stop();
  // A world the scene API cannot compile is refused with the construction's own error.
  for (const Json &bad : {Json{{"obstacles", Json::array({{{"type", "torus"}, {"position", {0, 0, 0}}}})}},
                          Json{{"obstacles", Json::array({{{"type", "box"}, {"position", {0, 0, 0}},
                                                           {"size", {1, 0, 1}}}})}}}) {
    bool refused = false;
    try {
      Sensors rejected(bad, 1);
    } catch (const std::invalid_argument &) { refused = true; }
    assert(refused);
  }
}

// The latched reference cloud is the sampled surface of the scene at the world's spacing and buried policy.
void reference_cloud() {
  for (const auto &policy : {std::make_pair(.1, false), std::make_pair(.25, true)}) {
    Sensors sensors(world({{"surface_spacing", policy.first}, {"keep_buried", policy.second}}), 1);
    const scene::LidarScene expected(obstacles(), policy.first, policy.second);
    const auto cloud = sensors.reference_cloud();
    assert(!expected.samples().empty() && cloud.size() == expected.samples().size() * 12);
    for (size_t i = 0; i < expected.samples().size(); ++i)
      for (size_t k = 0; k < 3; ++k) assert(number(cloud, i * 12 + 4 * k) == float(expected.samples()[i][long(k)]));
    sensors.stop();
  }
}

// Sensors of every model on three workers over several frames: each returns exactly the clouds that a private
// sensor of the scene API returns for the same poses, so the worker threads share the scene and nothing else.
void fleet_on_workers() {
  constexpr int members = 9;
  Sensors sensors(world(), 3);
  struct Member {
    std::shared_ptr<Entity> robot;
    std::unique_ptr<scene::WorldLidar> lidar;
    Bytes cloud, beams, want;
  };
  std::vector<Member> fleet(members);
  for (int i = 0; i < members; ++i) {
    auto &m = fleet[size_t(i)];
    m.robot = entity(100 + uint64_t(i), "fleet" + std::to_string(i));
    scene::SensorConfig model;
    Json config = {{"rate_hz", 10}};
    if (i % 3 == 0) {
      model.h_res = 90 + i;
      model.v_res = 5;
      model.range = 14;
      model.noise_std = i ? .02 : 0;
      model.seed = unsigned(i);
      config.update(Json{{"h_res", model.h_res}, {"v_res", 5}, {"range", 14}, {"noise_std", model.noise_std},
                         {"seed", i}});
    } else if (i % 3 == 1) {
      model.mode = scene::SensorConfig::kPenetrating;
      model.range = 7 + i;
      config.update(Json{{"mode", "penetrating"}, {"range", model.range}});
    } else {
      model.mode = scene::SensorConfig::kDepthFrustum;
      model.width = 24;
      model.height = 16;
      model.h_fov_deg = 80;
      model.range = 13;
      config.update(Json{{"mode", "depth"}, {"width", 24}, {"height", 16}, {"h_fov_deg", 80}, {"range", 13}});
    }
    m.robot->sensor = sensors.prepare(m.robot, config);
    m.lidar = reference(model);
  }
  for (int k = 0; k < 6; ++k) {
    Frame frame;
    frame.stamp = epoch + k * period;
    for (int i = 0; i < members; ++i)
      frame.states.push_back(state(fleet[size_t(i)].robot, frame.stamp,
                                   {-2. + .6 * i + .1 * k, .3 * i - 1, .5}, yaw(.2 * i + .05 * k)));
    sensors.submit_frame(frame);
    for (int i = 0; i < members; ++i) {
      auto &m = fleet[size_t(i)];
      const auto &pose = frame.states[size_t(i)];
      receive(m.robot->sensor, m.cloud, m.beams);
      m.lidar->scanInto(pose.position, pose.orientation, {}, false, &m.want);
      assert(!m.want.empty() && m.cloud == m.want);
    }
  }
  for (const auto &m : fleet) assert(m.robot->sensor->errors == 0 && m.robot->sensor->misses == 0);
  sensors.stop();
}
} // namespace

int main() {
  raycast_and_noise();
  beams_and_bodies();
  penetrating_and_depth();
  scene_forms();
  reference_cloud();
  fleet_on_workers();
  std::cout << Json({{"result", "PASS"}, {"checks", {"mounted_raycast_equals_scene_api", "noise_stream_per_sensor",
      "beam_and_point_layout", "own_body_excluded", "penetrating_policy_scene", "depth_camera",
      "scene_document_equals_list", "reference_cloud", "fleet_on_shared_scene"}}}).dump() << '\n';
}
