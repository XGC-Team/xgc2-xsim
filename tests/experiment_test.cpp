#include "io/config.hpp"
#include <cassert>
#include <iostream>
#include <fstream>

namespace {
constexpr int64_t startup_epoch = 1791271800123456789;
xsim::Json config_for(const xsim::Json &input) { return xsim::experiment_config(input, startup_epoch); }
void slot_id(xsim::Json &robot, const xsim::Json &id) { robot["slotId"] = id; robot["binding"]["id"] = id; }
}

int main(int argc, char **argv) {
  assert(argc == 2);
  const std::string root = argv[1];
  const auto cases = xsim::load_json_object(root + "/manifest.json").at("cases");
  for (const auto &entry : cases) {
    const auto name = entry.at("name").get<std::string>();
    xsim::Json input;
    const auto input_path = root + "/" + entry.at("input").get<std::string>();
    // A missing fixture is a test setup failure, never an expected rejection.
    if (!std::ifstream(input_path).good()) {
      std::cerr << name << " missing fixture: " << input_path << '\n';
      return 1;
    }
    try { input = xsim::load_json_object(input_path); }
    catch (const std::exception &) { if (entry.contains("expectedError")) continue; throw; }
    if (entry.contains("expectedConfig")) {
      const auto expected = xsim::load_json_object(root + "/" + entry.at("expectedConfig").get<std::string>());
      const auto actual = config_for(input);
      if (expected != actual) {
        std::cerr << name << "\nexpected " << expected.dump(2) << "\nactual " << actual.dump(2) << '\n';
        return 1;
      }
    } else {
      bool rejected = false;
      try { (void)config_for(input); } catch (const std::exception &) { rejected = true; }
      if (!rejected) { std::cerr << name << " was accepted\n"; return 1; }
    }
  }
  auto original = xsim::load_json_object(root + "/default-mixed.input.json");
  for (const char *scope : {"global", "individual"}) {
    auto authored = original;
    authored["scene"]["parameters"]["lidar"] = {
      {"scope", scope}, {"settings", {{"enabled", true}, {"acceleration", "gpu"}}}};
    // The frozen per-slot binding is authoritative; this is authoring metadata.
    assert(config_for(authored) == config_for(original));
  }
  for (const auto *backend : {"cpu", "gpu"}) {
    auto selected = original;
    selected["scene"]["parameters"]["pointCloudBackend"] = std::string(backend) == "cpu" ? "gpu" : "cpu";
    selected["robots"][0]["binding"]["simulationSensors"]["simpleLidar"] = {
      {"enabled", true}, {"acceleration", backend}, {"hFovDeg", 360}, {"vFovDeg", 90}, {"hRes", 360}, {"vRes", 90}};
    assert(config_for(selected)["entities"][0]["sensor"]["backend"] == backend);
  }
  for (const bool locked : {false, true}) {
    auto authored = original;
    authored["scene"]["parameters"]["frozen"] = locked;
    assert(config_for(authored) == config_for(original));
  }
  auto invalid_lock = original;
  invalid_lock["scene"]["parameters"]["frozen"] = "false";
  bool refused_lock = false;
  try { (void)config_for(invalid_lock); } catch (const std::exception &) { refused_lock = true; }
  assert(refused_lock);

  for (const char *key : {"sessionId", "runMode", "scene", "simulation", "robots", "visualizationTopics"}) {
    auto modified = original; modified.erase(key);
    bool rejected = false;
    try { (void)config_for(modified); } catch (const std::exception &) { rejected = true; }
    assert(rejected);
  }
  for (const char *key : {"context", "settings", "instanceId", "epochNs"}) {
    auto modified = original; modified[key] = xsim::Json::object();
    bool refused = false;
    try { (void)config_for(modified); } catch (const std::exception &) { refused = true; }
    assert(refused);
  }
  for (const auto epoch : {int64_t(0), int64_t(-1), INT64_MAX}) {
    bool refused = false;
    try { (void)xsim::experiment_config(original, epoch); } catch (const std::exception &) { refused = true; }
    assert(refused);
  }
  auto missing_binding = original; missing_binding["robots"][0].erase("binding");
  bool rejected = false;
  try { (void)config_for(missing_binding); } catch (const std::exception &) { rejected = true; }
  assert(rejected);
  auto omitted_selection = original;
  omitted_selection["robots"][0]["binding"].erase("simulationSensors");
  const auto no_sensor = config_for(omitted_selection);
  assert(!no_sensor["entities"][0].contains("sensor"));
  auto mismatched_namespace = original;
  mismatched_namespace["robots"][0]["binding"]["namespace"] = "/another";
  rejected = false;
  try { (void)config_for(mismatched_namespace); } catch (const std::exception &) { rejected = true; }
  assert(rejected);
  auto long_names = original;
  const std::string large_segment(100000, 'a');
  long_names["robots"][0]["namespace"] = "/" + large_segment;
  long_names["robots"][0]["binding"]["namespace"] = "/" + large_segment;
  long_names["robots"][0]["profile"]["parameters"]["localization_pose_topic"] = "/" + large_segment + "/pose";
  const auto large_config = config_for(long_names);
  assert(large_config["entities"][0]["name"] == large_segment);
  for (const char *invalid : {"/", "/1bad", "/valid/", "/valid//pose", "/valid/1pose", "/valid/pose-1"}) {
    auto modified = original; modified["robots"][0]["profile"]["parameters"]["localization_pose_topic"] = invalid;
    bool refused = false;
    try { (void)config_for(modified); } catch (const std::exception &) { refused = true; }
    assert(refused);
  }
  const auto frozen_before = original.dump();
  const auto config_before = config_for(original).dump();
  const xsim::Json expected_bindings{{"ugv1", "scout-1"}, {"uav1", "px4-1"},
    {"uav2", "px4-2"}, {"mecanum1", "mecanum-1"}};
  assert(xsim::experiment_robot_bindings(original) == expected_bindings);
  assert(original.dump() == frozen_before);
  assert(config_for(original).dump() == config_before);

  auto explicit_identity = original;
  slot_id(explicit_identity["robots"][0], "Robot:Case_01.rev-2");
  const auto identities = xsim::experiment_robot_bindings(explicit_identity);
  assert(identities.at("ugv1") == "Robot:Case_01.rev-2");
  assert(!identities.contains("Robot:Case_01.rev-2"));
  assert(config_for(explicit_identity).dump() == config_before);
  slot_id(explicit_identity["robots"][0], std::string(128, 'a'));
  assert(xsim::experiment_robot_bindings(explicit_identity).at("ugv1") == std::string(128, 'a'));

  const auto reject_binding = [](const xsim::Json &input) {
    bool refused = false;
    try { (void)xsim::experiment_robot_bindings(input); }
    catch (const std::exception &) { refused = true; }
    assert(refused);
  };
  for (const auto &invalid : xsim::Json::array({nullptr, 42, "", "robot/name", "robot name", "robot\nname", "机器人", std::string(129, 'a')})) {
    auto modified = original; slot_id(modified["robots"][0], invalid);
    reject_binding(modified);
  }
  auto missing_id = original; missing_id["robots"][0].erase("slotId");
  reject_binding(missing_id);
  auto mismatched_id = original; mismatched_id["robots"][0]["binding"]["id"] = "another-slot";
  reject_binding(mismatched_id);
  auto duplicate_id = original; slot_id(duplicate_id["robots"][1], duplicate_id["robots"][0]["slotId"]);
  reject_binding(duplicate_id);
  auto duplicate_name = original; duplicate_name["robots"][1]["namespace"] = duplicate_name["robots"][0]["namespace"];
  reject_binding(duplicate_name);
  for (const char *invalid : {"", "ugv1", "/1bad", "/two/segments"}) {
    auto modified = original; modified["robots"][0]["namespace"] = invalid;
    reject_binding(modified);
  }
  auto hybrid = xsim::load_json_object(root + "/hybrid-filter-physical-sensor.input.json");
  const xsim::Json hybrid_bindings{{"ugv1", "scout-1"}, {"mecanum1", "mecanum-1"}};
  assert(xsim::experiment_robot_bindings(hybrid) == hybrid_bindings);
  // Physical members are outside this world's identity map and are not repaired.
  hybrid["robots"][1].erase("slotId");
  hybrid["robots"][1].erase("namespace");
  assert(xsim::experiment_robot_bindings(hybrid) == hybrid_bindings);
  auto invalid_source = hybrid; invalid_source["robots"][0]["binding"]["hybridSource"] = "unknown";
  reject_binding(invalid_source);
  auto invalid_mode = original; invalid_mode["runMode"] = "unknown";
  reject_binding(invalid_mode);
  const auto simulation_physical = xsim::load_json_object(root + "/simulation-retains-physical-hybrid-flags.input.json");
  assert(xsim::experiment_robot_bindings(simulation_physical) == expected_bindings);
  auto empty = original; empty["robots"] = xsim::Json::array();
  assert(xsim::experiment_robot_bindings(empty) == xsim::Json::object());

  // Fig8B has seven authored FS150 slots. Runtime binding owns execution and
  // ground spawn; asset simulationSetup is metadata, not runtime placement.
  auto fig8b = original;
  fig8b["robots"] = xsim::Json::array();
  const xsim::Json spawns = {{-31, -.5, 0}, {-31, .5, 0}, {-32, -1, 0},
    {-32, 0, 0}, {-32, 1, 0}, {-33, -.5, 0}, {-33, .5, 0}};
  for (size_t i = 0; i < spawns.size(); ++i) {
    auto robot = original["robots"][1];
    const auto name = "uav" + std::to_string(i + 1);
    const auto ns = "/" + name;
    slot_id(robot, "fig8b-" + name);
    robot["assetId"] = "fig8b-fs150-" + std::to_string(i + 1);
    robot["assetCommitId"] = "frozen-" + name;
    robot["name"] = robot["asset"]["name"] = name;
    robot["namespace"] = robot["binding"]["namespace"] = ns;
    robot["binding"]["initialPose"] = {{"x", spawns[i][0]}, {"y", spawns[i][1]}, {"z", spawns[i][2]}, {"yaw", 0}};
    robot["binding"].erase("simulationSensors");
    robot["asset"]["px4"]["simulationSetup"] = "onboard";
    robot["profile"]["parameters"]["localization_pose_topic"] = "/xgc/act1/" + name + "/pose";
    robot["profile"]["parameters"]["localization_twist_topic"] = "/xgc/act1/" + name + "/twist";
    fig8b["robots"].push_back(robot);
  }
  const auto seven = config_for(fig8b);
  const auto seven_bindings = xsim::experiment_robot_bindings(fig8b);
  assert(seven["entities"].size() == 7 && seven_bindings.size() == 7);
  assert(seven["epoch_ns"] == startup_epoch);
  for (size_t i = 0; i < spawns.size(); ++i) {
    const auto name = "uav" + std::to_string(i + 1);
    const auto &entity = seven["entities"][i];
    assert(entity["name"] == name && entity["kind"] == "fs150");
    assert(entity["position"] == spawns[i] && entity["yaw"] == 0);
    assert(entity["ros"]["localization_pose_topic"] == "/xgc/act1/" + name + "/pose");
    assert(seven_bindings[name] == "fig8b-" + name);
    assert(!entity.contains("sensor"));
  }
  std::cout << cases.size() << " current frozen Session fixtures and strict native-input negative controls passed\n";
  std::cout << "frozen public robot identities, hybrid filtering and unchanged scientific config passed\n";
}
