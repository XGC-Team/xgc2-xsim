#include "config.hpp"
#include <limits>
#include <set>

namespace xsim {
namespace {
void require(bool valid, const std::string &message) {
  if (!valid) throw std::invalid_argument("experiment: " + message);
}
std::string text(const Json &j, const char *key) { return j.at(key).get<std::string>(); }
bool ros_alpha(char c) { return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z'); }
bool ros_tail(char c) { return ros_alpha(c) || (c >= '0' && c <= '9') || c == '_'; }
// Linear scanning also bounds stack use for documents near the input byte limit.
bool ros_name(const std::string &value, bool allow_segments) {
  if (value.size() < 2 || value[0] != '/' || !ros_alpha(value[1])) return false;
  for (std::size_t i = 2; i < value.size(); ++i) {
    if (value[i] == '/' && allow_segments) {
      if (++i == value.size() || !ros_alpha(value[i])) return false;
    } else if (!ros_tail(value[i])) return false;
  }
  return true;
}
bool simulated_robot(const Json &robot, const std::string &run_mode) {
  if (run_mode != "hybrid") return true;
  const auto source = text(robot.at("binding"), "hybridSource");
  require(source == "simulation" || source == "physical", "invalid Hybrid source");
  return source == "simulation";
}
double number(const Json &v) {
  require(v.is_number(), "expected a finite number");
  const auto n = v.get<double>();
  require(std::isfinite(n), "expected a finite number");
  return n;
}
int64_t timing(const Json &parameters, const char *key, int64_t fallback) {
  if (!parameters.contains(key)) return fallback * 1000000;
  const auto &v = parameters.at(key);
  require(v.is_number_integer(), std::string(key) + " must be whole milliseconds");
  require(!v.is_number_unsigned() || v.get<uint64_t>() <= uint64_t(INT64_MAX), "timing overflow");
  const auto ms = v.get<int64_t>();
  require(ms > 0 && ms <= INT64_MAX / 1000000, "timing must be positive and representable in nanoseconds");
  require(std::string(key) != "modelStepMs" || ms <= 10, "modelStepMs exceeds 10ms");
  return ms * 1000000;
}
bool reference_requested(const Json &topics) {
  require(topics.is_array(), "visualizationTopics declarations are required");
  for (const auto &topic : topics) {
    require(topic.is_object(), "visualizationTopics entries must be objects");
    if (topic.value("topic", std::string{}) == "/xgc/scene/reference_cloud" &&
        topic.value("messageType", std::string{}) == "sensor_msgs/PointCloud2" &&
        topic.value("role", std::string{}) == "semantic" &&
        (topic.value("visible3d", false) || topic.value("visibleAr", false))) return true;
  }
  return false;
}

Json sensor(const Json &authored, const std::string &ns, const std::string &default_backend) {
  require(authored.is_object(), "binding.simulationSensors must be an object");
  // Core omits zero-valued binding.simulationSensors; omission is no sensor opt-in.
  if (authored.empty()) return nullptr;
  require(authored.contains("simpleLidar"), "binding.simulationSensors.simpleLidar is required");
  require(authored.size() == 1, "unknown authored simulation sensor field");
  const auto &raw = authored.at("simpleLidar");
  Json lidar = raw;
  if (raw.is_boolean()) lidar = Json{{"enabled", raw.get<bool>()}};
  require(lidar.is_object(), "authored simpleLidar must be boolean or object");
  const std::set<std::string> allowed{"enabled", "mode", "preset", "acceleration", "rateHz", "rangeMeters", "hFovDeg", "vFovDeg", "hRes", "vRes", "publishBeams", "bodyReturns"};
  for (auto i = lidar.begin(); i != lidar.end(); ++i) require(allowed.count(i.key()) != 0, "unknown authored lidar field: " + i.key());
  const auto acceleration = lidar.value("acceleration", std::string{});
  require(acceleration.empty() || acceleration == "auto" || acceleration == "cpu" || acceleration == "gpu", "unsupported authored acceleration");
  const auto backend = acceleration.empty() ? default_backend : acceleration;
  const auto enabled = lidar.value("enabled", false);
  (void)lidar.value("publishBeams", false);
  (void)lidar.value("bodyReturns", false);
  for (const auto &bound : {std::pair<const char *, double>{"rateHz", 100}, {"rangeMeters", 200}, {"hFovDeg", 360}, {"vFovDeg", 180}})
    if (lidar.contains(bound.first)) {
      const auto value = number(lidar.at(bound.first));
      require(value >= 0 && value <= bound.second, "authored lidar numeric bound exceeded");
    }
  for (const char *key : {"hRes", "vRes"})
    if (lidar.contains(key)) {
      require(lidar.at(key).is_number_integer(), "authored lidar resolution must be integer");
      require(!lidar.at(key).is_number_unsigned() || lidar.at(key).get<uint64_t>() <= 4096, "lidar resolution bound exceeded");
      const auto value = lidar.at(key).get<int64_t>();
      require(value >= 0 && value <= 4096 && (lidar.value("mode", std::string{}) == "penetrating" || value != 1), "authored lidar resolution bound exceeded");
    }
  const auto mode_authored = lidar.value("mode", std::string{});
  auto preset = lidar.value("preset", std::string{});
  require(mode_authored.empty() || mode_authored == "raycast" || mode_authored == "penetrating" || mode_authored == "depth_frustum", "unsupported lidar mode");
  require(preset.empty() || preset == "bridge_equivalent" || preset == "zju_cpu_crop" || preset == "zju_cpu_crop_ego_v2", "unsupported lidar preset");
  if (!enabled) return nullptr;
  std::string mode = "raycast";
  double rate = 10, range = 20, hfov = 360, vfov = 30, heading_cos = 0, slab = 0;
  int hres = 360, vres = backend == "cpu" ? 32 : 30;
  bool buried = false, crop = false;
  if (!preset.empty()) {
    mode = "penetrating"; vfov = 180; buried = true; range = 5;
    if (preset == "bridge_equivalent") range = 8;
    else { crop = true; slab = 0.5773502691896258; if (preset == "zju_cpu_crop") heading_cos = 0.5; }
  }
  if (!mode_authored.empty()) mode = mode_authored;
  if (mode == "depth_frustum") { hfov = 90; vfov = 60; }
  for (auto pair : {std::pair<const char *, double *>{"rateHz", &rate}, {"rangeMeters", &range}, {"hFovDeg", &hfov}, {"vFovDeg", &vfov}})
    if (lidar.contains(pair.first)) { const auto v = number(lidar.at(pair.first)); if (v > 0) *pair.second = v; }
  for (auto pair : {std::pair<const char *, int *>{"hRes", &hres}, {"vRes", &vres}})
    if (lidar.contains(pair.first)) {
      require(lidar.at(pair.first).is_number_integer(), "lidar resolution must be integer");
      const auto v = lidar.at(pair.first).get<int64_t>();
      require(v >= 0 && v <= 4096, "lidar resolution exceeds authored bounds");
      if (v > 0) *pair.second = static_cast<int>(v);
    }
  const bool beams = lidar.value("publishBeams", false);
  require(!(beams && mode == "penetrating"), "penetrating has no rays for beams");
  require(!(beams && backend == "gpu"), "GPU point observations do not produce beams");
  if (mode == "depth_frustum") mode = "depth";
  Json result{{"backend", backend}, {"mode", mode}, {"topic", ns + "/simple_lidar/points"}, {"frame", "world"},
    {"rate_hz", rate}, {"range", range}, {"h_fov_deg", hfov}, {"v_fov_deg", vfov}, {"h_res", hres}, {"v_res", vres},
    {"surface_spacing", 0.1}, {"keep_buried", buried}, {"heading_crop", crop}, {"heading_cos_min", heading_cos},
    {"vertical_slab_tan", slab}, {"publish_beams", beams}, {"world_bodies", false}};
  if (backend != "cpu") { result["min_range"] = 0.1; result["point_cover_spacing"] = 0.1; }
  return result;
}
} // namespace
Json experiment_config(const Json &input, int64_t epoch_ns) {
  require(input.is_object(), "frozen Session must be one object");
  for (const char *key : {"context", "settings", "instanceId", "epochNs"})
    require(!input.contains(key), std::string("retired Session envelope field: ") + key);
  require(!text(input, "sessionId").empty(), "frozen sessionId is required");
  require(epoch_ns > 0, "native startup epoch must be positive");
  const auto run_mode = text(input, "runMode");
  require(run_mode == "simulation" || run_mode == "hybrid", "unsupported runMode");
  require(text(input.at("scene"), "simulator") == "xsim", "scene simulator must be xsim");
  require(text(input.at("simulation"), "placement") == "core", "xsim must be centralized on Core");
  const auto &robots = input.at("robots");
  require(robots.is_array() && robots.size() <= 256, "robots must be the frozen roster");
  auto parameters = input.at("scene").value("parameters", Json::object());
  require(parameters.is_object(), "scene parameters must be an object");
  // Robot projection already lowered Scene scope into the frozen slot bindings.
  parameters.erase("lidar");
  for (auto i = parameters.begin(); i != parameters.end(); ++i)
    require(i.key() == "modelStepMs" || i.key() == "outputPeriodMs" || i.key() == "commandPollMs" ||
      i.key() == "pointCloudBackend" || i.key() == "fcuParameters" || i.key() == "generation" || i.key() == "frozen", "unknown scene parameter: " + i.key());
  if (parameters.contains("frozen")) require(parameters.at("frozen").is_boolean(), "scene frozen must be a boolean");
  if (parameters.contains("generation")) {
    require(parameters.at("generation").is_object(), "generation parameters must be a numeric object");
    for (const auto &v : parameters.at("generation")) (void)number(v);
  }
  const auto step = timing(parameters, "modelStepMs", 2);
  const auto output = timing(parameters, "outputPeriodMs", 8);
  const auto poll = timing(parameters, "commandPollMs", 5);
  require(step <= INT64_MAX - epoch_ns && output <= INT64_MAX - epoch_ns, "native epoch plus timing overflows");
  const auto backend = parameters.value("pointCloudBackend", std::string("auto"));
  require(backend == "auto" || backend == "cpu" || backend == "gpu", "pointCloudBackend must be auto, cpu or gpu");
  const auto fcu = parameters.value("fcuParameters", Json::object());
  require(fcu.is_object(), "fcuParameters must be a numeric object");
  FlightControllerParameters fcu_validator;
  for (auto i = fcu.begin(); i != fcu.end(); ++i) require(fcu_validator.set(i.key(), number(i.value())), "invalid FCU parameter: " + i.key());
  if (fcu.contains("MPC_THR_MIN") && fcu.contains("MPC_THR_MAX"))
    require(number(fcu.at("MPC_THR_MIN")) <= number(fcu.at("MPC_THR_MAX")), "FCU MPC_THR_MIN exceeds MPC_THR_MAX");
  Json config{{"epoch_ns", epoch_ns}, {"model_step_ns", step}, {"output_period_ns", output},
    {"input_poll_ns", poll}, {"publish_clock", true}, {"scene", Json::object()}, {"entities", Json::array()}};
  std::set<std::string> names;
  for (const auto &robot : robots) {
    if (!simulated_robot(robot, run_mode)) continue;
    const auto ns = text(robot, "namespace");
    const auto &binding = robot.at("binding");
    require(text(binding, "namespace") == ns, "binding namespace differs from frozen robot namespace");
    require(text(binding.at("executionHost"), "kind") == "station", "xsim robot must execute on the station");
    require(ros_name(ns, false), "namespace must be one absolute ROS segment");
    const auto body = ns.substr(1);
    require(names.insert(body).second, "duplicate entity namespace");
    const auto kind = text(robot, "kind");
    std::string model;
    if (kind == "px4_multirotor") {
      const auto &px4 = robot.at("asset").at("px4");
      require(text(px4, "modelId") == "fs150", "xsim models FS150 only");
      require(!binding.value("px4", Json::object()).value("imageSimulationEnabled", false), "xsim has no camera producer");
      model = "fs150";
    } else if (kind == "scout_mini") {
      const auto scout = binding.value("scout", Json::object());
      require(!scout.value("lidarSimulationEnabled", false), "xsim has no 2D scan producer");
      require(!scout.value("imageSimulationEnabled", false), "xsim has no camera/depth producer");
      model = "scout";
    } else if (kind == "mecanum_ugv") model = "mecanum";
    else throw std::invalid_argument("experiment: unsupported robot kind: " + kind);
    const auto &pose = binding.at("initialPose");
    Json position = Json::array({number(pose.at("x")), number(pose.at("y")), number(pose.at("z"))});
    const auto yaw = number(pose.at("yaw"));
    uint32_t seed = 2166136261u;
    for (unsigned char byte : body) { seed ^= byte; seed *= 16777619u; }
    const auto &profile = robot.at("profile").at("parameters");
    const auto pose_topic = profile.value("localization_pose_topic", std::string{});
    const auto twist_topic = profile.value("localization_twist_topic", std::string{});
    const auto resolved_pose = pose_topic.empty() ? ns + "/pose" : pose_topic;
    const auto resolved_twist = twist_topic.empty() ? ns + "/twist" : twist_topic;
    require(ros_name(resolved_pose, true) && ros_name(resolved_twist, true), "invalid localization topic");
    Json entity{{"name", body}, {"kind", model}, {"position", position}, {"yaw", yaw}, {"ros", Json{
      {"mocap_noise", Json::array({1e-7, 1e-7, 1e-7})}, {"mocap_seed", seed & 0x7fffffffu},
      {"localization_pose_topic", resolved_pose}, {"localization_twist_topic", resolved_twist}}}};
    if (model == "fs150" && !fcu.empty()) entity["fcu_parameters"] = fcu;
    const auto authored_sensors = binding.value("simulationSensors", Json::object());
    const auto authored_lidar = authored_sensors.value("simpleLidar", Json(false));
    entity["body_returns"] = authored_lidar.is_object() ? authored_lidar.value("bodyReturns", false) : false;
    const auto desired_sensor = sensor(authored_sensors, ns, backend);
    if (!desired_sensor.is_null()) entity["sensor"] = desired_sensor;
    (void)parse_entity(entity);
    config["entities"].push_back(std::move(entity));
  }
  if (reference_requested(input.at("visualizationTopics"))) config["reference_cloud_topic"] = "/xgc/scene/reference_cloud";
  return config;
}
Json experiment_robot_bindings(const Json &input) {
  require(input.is_object(), "frozen Session must be one object");
  const auto run_mode = text(input, "runMode");
  require(run_mode == "simulation" || run_mode == "hybrid", "unsupported runMode");
  const auto &robots = input.at("robots");
  require(robots.is_array() && robots.size() <= 256, "robots must be the frozen roster");
  Json bindings = Json::object();
  std::set<std::string> ids;
  for (const auto &robot : robots) {
    if (!simulated_robot(robot, run_mode)) continue;
    const auto ns = text(robot, "namespace");
    require(ros_name(ns, false), "namespace must be one absolute ROS segment");
    const auto id = text(robot, "slotId");
    require(id == text(robot.at("binding"), "id"), "binding ID differs from frozen slotId");
    require(!id.empty() && id.size() <= 128 &&
      id.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.:-") == std::string::npos,
      "robot ID requires 1..128 ASCII letters, digits or . _ : -");
    require(ids.insert(id).second, "duplicate robot ID");
    require(bindings.emplace(ns.substr(1), id).second, "duplicate entity namespace");
  }
  return bindings;
}
} // namespace xsim
