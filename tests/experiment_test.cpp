#include "io/config.hpp"
#include <cassert>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <unistd.h>

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
  // Target visibility survives lowering even when that target has no sensor.
  for (const bool visible : {false, true}) {
    auto selected = original;
    selected["robots"][0]["binding"]["simulationSensors"]["simpleLidar"] = {
      {"enabled", false}, {"bodyReturns", visible}};
    const auto lowered = config_for(selected).at("entities")[0];
    assert(lowered.at("body_returns") == visible && !lowered.contains("sensor"));
    assert(xsim::parse_entity(lowered).body_returns == visible);
  }
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
  {
    // A saved point cloud is a frozen asset fact (sceneAsset.pointCloud). It, and nothing next to
    // the scene file, moves the cloud to the scene adapter: no entity sensor and no reference topic.
    auto sensed = xsim::load_json_object(root + "/gpu-depth_frustum.input.json");
    sensed["visualizationTopics"] = xsim::Json::array({{{"topic", "/xgc/scene/reference_cloud"},
      {"messageType", "sensor_msgs/PointCloud2"}, {"role", "semantic"}, {"visible3d", true}}});
    const auto both = config_for(sensed);
    assert(both.contains("reference_cloud_topic") && both.at("entities").at(0).contains("sensor"));
    auto saved = sensed;
    saved["sceneAsset"] = {{"pointCloud", {{"file", "original-volume.xyz.f32"}, {"sha256", "0"},
      {"format", "xyz-f32-le"}, {"frame", "world"}}}};
    const auto moved = config_for(saved);
    assert(!moved.contains("reference_cloud_topic"));
    for (const auto &entity : moved.at("entities")) assert(!entity.contains("sensor"));
    auto reduced = both;
    reduced.erase("reference_cloud_topic");
    for (auto &entity : reduced["entities"]) entity.erase("sensor");
    assert(moved == reduced);
    // The sensor selection is still validated; only its publisher moved.
    auto invalid = saved;
    invalid["robots"][0]["binding"]["simulationSensors"]["simpleLidar"]["hRes"] = 4097;
    bool refused_sensor = false;
    try { (void)config_for(invalid); } catch (const std::exception &) { refused_sensor = true; }
    assert(refused_sensor);
    auto not_object = saved;
    not_object["sceneAsset"]["pointCloud"] = "original-volume.xyz.f32";
    bool refused_cloud = false;
    try { (void)config_for(not_object); } catch (const std::exception &) { refused_cloud = true; }
    assert(refused_cloud);
    // No manifest beside the scene file is read any more.
    char manifest_pattern[] = "/tmp/xsim-experiment-manifest-XXXXXX";
    assert(mkdtemp(manifest_pattern));
    const std::filesystem::path manifest_dir(manifest_pattern);
    std::ofstream(manifest_dir / "manifest.json") << R"({"pointCloud":{"file":"original-volume.xyz.f32"}})";
    std::ofstream(manifest_dir / "scene.json") << R"({"schema":"xgc2.scene.compiled.v1","id":"yard","frame":"world","geometries":[],"models":{},"obstacles":[]})";
    auto beside = sensed;
    beside["sceneAsset"] = {{"sceneFile", (manifest_dir / "scene.json").string()}};
    const auto ignored = xsim::resolve_scene_file(config_for(beside), "");
    assert(ignored.contains("reference_cloud_topic") && ignored.at("entities").at(0).contains("sensor"));
    std::filesystem::remove_all(manifest_dir);
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

  for (const char *key : {"sessionId", "runMode", "scene", "robots", "visualizationTopics"}) {
    auto modified = original; modified.erase(key);
    bool rejected = false;
    try { (void)config_for(modified); } catch (const std::exception &) { rejected = true; }
    assert(rejected);
  }
  assert(!original.at("simulation").contains("placement"));
  auto leftover_placement = original;
  leftover_placement["simulation"]["placement"] = "agent-world";
  assert(config_for(leftover_placement) == config_for(original));
  auto absent_simulation = original;
  absent_simulation.erase("simulation");
  assert(config_for(absent_simulation) == config_for(original));
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

  // Fig8B has seven model slots. Runtime binding owns execution and ground
  // spawn; the selected model is frozen in kind and derived.modelId.
  auto fig8b = original;
  fig8b["robots"] = xsim::Json::array();
  const xsim::Json spawns = {{-31, -.5, 0}, {-31, .5, 0}, {-32, -1, 0},
    {-32, 0, 0}, {-32, 1, 0}, {-33, -.5, 0}, {-33, .5, 0}};
  for (size_t i = 0; i < spawns.size(); ++i) {
    auto robot = original["robots"][1];
    const auto name = "uav" + std::to_string(i + 1);
    const auto ns = "/" + name;
    slot_id(robot, "fig8b-" + name);
    robot.erase("asset");
    robot.erase("assetId");
    robot.erase("assetCommitId");
    robot.erase("derived");
    robot["kind"] = "px4_multirotor";
    robot["derived"] = {{"modelId", "fs150"}};
    robot["name"] = name;
    robot["namespace"] = robot["binding"]["namespace"] = ns;
    robot["binding"]["initialPose"] = {{"x", spawns[i][0]}, {"y", spawns[i][1]}, {"z", spawns[i][2]}, {"yaw", 0}};
    robot["binding"].erase("simulationSensors");
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
  auto none = original;
  none["robots"] = xsim::Json::array();
  assert(config_for(none).at("entities").empty());

  auto fleet = original;
  fleet["robots"] = xsim::Json::array();
  for (int i = 0; i < 100; ++i) {
    const auto name = "uav" + std::to_string(i);
    const auto ns = "/" + name;
    xsim::Json robot{{"slotId", "model-" + name}, {"name", name}, {"namespace", ns},
      {"kind", "px4_multirotor"}, {"derived", {{"modelId", "fs150"}}},
      {"profile", {{"parameters", {{"localization_pose_topic", ns + "/pose"}, {"localization_twist_topic", ns + "/twist"}}}}},
      {"binding", {{"id", "model-" + name}, {"namespace", ns}, {"hybridSource", "simulation"},
        {"applyModelDynamics", true},
        {"initialPose", {{"x", 0.0}, {"y", static_cast<double>(i)}, {"z", 0.0}, {"yaw", 0.0}}},
        {"executionHost", {{"kind", "station"}}}}}};
    assert(!robot.contains("asset") && !robot.contains("assetId") && !robot.contains("assetCommitId") &&
      !robot.contains("physicalBinding") && !robot.contains("model"));
    fleet["robots"].push_back(std::move(robot));
  }
  const auto hundred = config_for(fleet);
  assert(hundred.at("entities").size() == 100);
  assert(xsim::experiment_robot_bindings(fleet).size() == 100);
  for (int i = 0; i < 100; ++i) {
    const auto &entity = hundred.at("entities").at(i);
    assert(entity.at("kind") == "fs150");
    assert(entity.at("name") == "uav" + std::to_string(i));
    assert(entity.at("position").at(1).get<double>() == i);
  }

  const char *selected_kind[] = {"px4_multirotor", "scout_mini", "mecanum_ugv"};
  const char *selected_model[] = {"fs150", "scout-mini", "mecanum-ugv"};
  const char *selected_entity[] = {"fs150", "scout", "mecanum"};
  auto selected = original;
  selected["robots"] = xsim::Json::array();
  for (int i = 0; i < 3; ++i) {
    const auto name = std::string("slot") + std::to_string(i);
    const auto ns = "/" + name;
    xsim::Json robot{{"slotId", name}, {"name", name}, {"namespace", ns},
      {"kind", selected_kind[i]}, {"derived", {{"modelId", selected_model[i]}}},
      {"profile", {{"parameters", {{"localization_pose_topic", ns + "/pose"}, {"localization_twist_topic", ns + "/twist"}}}}},
      {"binding", {{"id", name}, {"namespace", ns}, {"hybridSource", "simulation"},
        {"applyModelDynamics", true},
        {"initialPose", {{"x", 0.0}, {"y", 0.0}, {"z", 0.0}, {"yaw", 0.0}}},
        {"executionHost", {{"kind", "station"}}}}}};
    if (std::string(selected_kind[i]) == "scout_mini")
      robot["binding"]["scout"] = {{"lidarSimulationEnabled", false}, {"imageSimulationEnabled", false}};
    selected["robots"].push_back(std::move(robot));
  }
  const auto selected_config = config_for(selected);
  for (int i = 0; i < 3; ++i) assert(selected_config.at("entities").at(i).at("kind") == selected_entity[i]);
  auto unknown_model = selected;
  unknown_model["robots"][0]["derived"]["modelId"] = "mocap_rotor";
  unknown_model["robots"][0]["physicalBinding"] = {{"asset", {{"px4", {{"modelId", "fs150"}}}}}};
  bool refused_model = false;
  try { (void)config_for(unknown_model); } catch (const std::exception &) { refused_model = true; }
  assert(refused_model);
  auto asset_fallback = original;
  asset_fallback["robots"][1].erase("derived");
  asset_fallback["robots"][1]["physicalBinding"]["asset"]["px4"]["modelId"] = "fs150";
  bool refused_fallback = false;
  try { (void)config_for(asset_fallback); } catch (const std::exception &) { refused_fallback = true; }
  assert(refused_fallback);

  auto tracker = selected;
  tracker["scene"]["parameters"]["fcuParameters"] = {{"MC_ROLL_P", 6.5}};
  tracker["robots"][0]["binding"]["applyModelDynamics"] = false;
  const auto tracked = config_for(tracker);
  assert(tracked.at("entities").size() == 2);
  assert(tracked.at("entities").at(0).at("name") == "slot1");
  assert(tracked.at("entities").at(1).at("name") == "slot2");
  const auto tracked_bindings = xsim::experiment_robot_bindings(tracker);
  assert(tracked_bindings.size() == 2);
  assert(!tracked_bindings.contains("slot0"));
  assert(!tracker["robots"][0].contains("asset"));
  tracker["robots"][0]["binding"]["applyModelDynamics"] = true;
  assert(config_for(tracker).at("entities").at(0).contains("fcu_parameters"));
  tracker["robots"][0]["binding"]["applyModelDynamics"] = false;
  for (const char *key : {"physical_fcu_url", "simulation_fcu_url", "management_ip"}) {
    tracker["robots"][0]["profile"]["parameters"][key] = "dropped";
    bool refused_address = false;
    try { (void)config_for(tracker); } catch (const std::exception &) { refused_address = true; }
    assert(refused_address);
    tracker["robots"][0]["profile"]["parameters"].erase(key);
  }

  auto physical = hybrid;
  for (auto &robot : physical["robots"]) {
    if (robot.at("binding").at("hybridSource") == "physical") {
      // A physically sourced robot is filtered before simulation geometry is read.
      robot["kind"] = "not-a-simulated-kind";
      robot["derived"] = {{"modelId", "not-a-simulated-model"}};
    } else {
      // The optional attachment never overrides the common model selection.
      robot["physicalBinding"] = {{"assetId", "not-a-model-source"}, {"assetCommitId", "not-a-model-source"},
        {"asset", {{"px4", {{"modelId", "mocap_rotor"}}}}}};
    }
  }
  assert(config_for(physical) == config_for(hybrid));
  assert(xsim::experiment_robot_bindings(physical) == hybrid_bindings);

  const auto type1 = config_for(original);
  assert(type1.at("publish_clock") == true);
  assert(type1.at("epoch_ns") == startup_epoch);
  assert(!type1.contains("use_sim_time") && !type1.contains("ros_node_name") &&
         !type1.contains("phase_t0_ns") && !type1.contains("slotId"));
  assert(xsim::experiment_config(original, startup_epoch, "") == type1);
  auto onboard = original;
  onboard["robots"][0]["binding"]["executionHost"] = {{"kind", "agent"}, {"agentId", "agent-1"}};
  onboard["robots"][0]["binding"]["simulationSensors"]["simpleLidar"] = {
    {"enabled", false}, {"bodyReturns", true}};
  bool station_required = false;
  try { (void)config_for(onboard); } catch (const std::exception &) { station_required = true; }
  assert(station_required);
  auto no_world = onboard;
  no_world["deployment"] = xsim::Json::object();
  assert(!no_world.at("deployment").contains("worldHost"));
  const auto self = xsim::experiment_config(onboard, startup_epoch, "scout-1");
  const auto without_world = xsim::experiment_config(no_world, startup_epoch, "scout-1");
  assert(without_world == self);
  assert(!without_world.contains("worldHost") && !without_world.contains("placement"));
  no_world.erase("deployment");
  assert(xsim::experiment_config(no_world, startup_epoch, "scout-1") == self);
  assert(self.at("entities").size() == 1);
  assert(self.at("entities").at(0).at("name") == "ugv1");
  assert(self.at("entities").at(0).at("body_returns") == true);
  assert(self.at("publish_clock") == false);
  assert(self.at("use_sim_time") == false);
  assert(self.at("ros_node_name") == "ugv1");
  assert(self.at("epoch_ns") == startup_epoch);
  assert(!self.contains("phase_t0_ns") && !self.contains("slotId"));
  const auto self_bindings = xsim::experiment_robot_bindings(onboard, "scout-1");
  assert(self_bindings.size() == 1);
  assert(self_bindings.at("ugv1") == "scout-1");
  const int64_t phase = 42;
  const auto phased = xsim::experiment_config(onboard, startup_epoch, "scout-1", &phase);
  assert(phased.at("epoch_ns") == startup_epoch && phased.at("phase_t0_ns") == phase);
  slot_id(onboard["robots"][0], "Robot:Case_01.rev-2");
  const auto named = xsim::experiment_config(onboard, startup_epoch, "Robot:Case_01.rev-2");
  assert(named.at("ros_node_name") == "ugv1");
  assert(named.at("entities").at(0).at("name") == "ugv1");
  const auto reject_slot = [](const xsim::Json &input, const std::string &id) {
    bool refused = false;
    try { (void)xsim::experiment_config(input, startup_epoch, id); }
    catch (const std::exception &) { refused = true; }
    assert(refused);
    refused = false;
    try { (void)xsim::experiment_robot_bindings(input, id); }
    catch (const std::exception &) { refused = true; }
    assert(refused);
  };
  reject_slot(original, "missing-slot");
  const auto hybrid_roster = xsim::load_json_object(root + "/hybrid-filter-physical-sensor.input.json");
  reject_slot(hybrid_roster, "px4-1");
  auto tracker_slot = original;
  tracker_slot["robots"][0]["binding"]["applyModelDynamics"] = false;
  reject_slot(tracker_slot, "scout-1");
  auto duplicate_slot = original;
  slot_id(duplicate_slot["robots"][1], "scout-1");
  reject_slot(duplicate_slot, "scout-1");

  // Explicit HIL cannot turn a fleet, simulated slot or station into a physical plant.
  auto hil_input = original;
  hil_input["experimentType"] = "onboard";
  auto px4 = original.at("robots").at(1);
  assert(px4.at("kind") == "px4_multirotor");
  px4["targetId"] = "onboard";
  px4["binding"].erase("executionHost");
  px4["binding"]["applyModelDynamics"] = false;
  px4["binding"]["hybridSource"] = "physical";
  hil_input["robots"] = xsim::Json::array({px4});
  hil_input["runMode"] = "physical";
  const auto hil_id = px4.at("slotId").get<std::string>();
  const auto before_hil = hil_input;
  const auto hil_world = xsim::experiment_config(hil_input, startup_epoch, hil_id, nullptr, true);
  assert(hil_world.at("entities").size() == 1 && hil_world.at("use_sim_time") == false);
  assert(hil_world.at("publish_clock") == false);
  assert(xsim::experiment_robot_bindings(hil_input, hil_id, true).size() == 1);
  assert(hil_input == before_hil);
  hil_input["runMode"] = "hybrid";
  assert(xsim::experiment_config(hil_input, startup_epoch, hil_id, nullptr, true) == hil_world);
  hil_input["scene"]["simulator"] = "gazebo";
  hil_input["scene"]["parameters"] = {{"maxStepSize", .004}, {"realTimeUpdateRate", 250}};
  assert(xsim::experiment_config(hil_input, startup_epoch, hil_id, nullptr, true) == hil_world);
  const auto reject_hil = [](const xsim::Json &input, const std::string &id) {
    bool refused = false;
    try { (void)xsim::experiment_config(input, startup_epoch, id, nullptr, true); }
    catch (const std::exception &) { refused = true; }
    assert(refused);
    refused = false;
    try { (void)xsim::experiment_robot_bindings(input, id, true); }
    catch (const std::exception &) { refused = true; }
    assert(refused);
  };
  reject_hil(hil_input, "");
  reject_hil(hil_input, "missing");
  hil_input["robots"][0]["binding"]["hybridSource"] = "simulation";
  reject_hil(hil_input, hil_id);
  hil_input["runMode"] = "simulation";
  reject_hil(hil_input, hil_id);
  hil_input["runMode"] = "physical";
  hil_input["robots"][0]["targetId"] = "local";
  reject_hil(hil_input, hil_id);

  char pattern[] = "/tmp/xsim-experiment-ground-XXXXXX";
  assert(mkdtemp(pattern));
  const std::filesystem::path ground_dir(pattern);
  struct GroundCleanup { std::filesystem::path root; ~GroundCleanup() { std::filesystem::remove_all(root); } } ground_cleanup{ground_dir};
  const auto write_ground = [&](const char *name, const char *body) {
    const auto path = ground_dir / name;
    std::ofstream(path) << body;
    return path.string();
  };
  const auto plane_path = write_ground("plane.json",
      R"({"schema":"xgc2.scene.compiled.v1","id":"yard","frame":"world","ground":{"mode":"default_plane","z":1.5},"geometries":[{"type":"not-a-shape"}],"models":{},"obstacles":[]})");
  auto session = original;
  session["sceneAsset"] = {{"sceneFile", plane_path}};
  const auto lowered = config_for(session);
  assert(!lowered.at("entities").empty());
  for (const auto &entity : lowered.at("entities")) assert(!entity.contains("ground_z"));
  const auto applied = xsim::resolve_scene_file(lowered, "");
  for (const auto &entity : applied.at("entities")) {
    assert(entity.at("ground_z") == 1.5);
    assert(xsim::parse_entity(entity).ground_z == 1.5);
  }
  assert(applied.at("scene").at("document").at("geometries").at(0).at("type") == "not-a-shape");
  for (const char *kind : {"none", "model"}) {
    const auto path = write_ground(kind, kind == std::string("none")
        ? R"({"schema":"xgc2.scene.compiled.v1","id":"yard","frame":"world","ground":{"mode":"none"},"geometries":[],"models":{},"obstacles":[]})"
        : R"({"schema":"xgc2.scene.compiled.v1","id":"yard","frame":"world","ground":{"mode":"model","model":"terrain"},"geometries":[],"models":{},"obstacles":[]})");
    auto moving = original;
    moving["sceneAsset"] = {{"sceneFile", path}};
    bool refused = false;
    try { (void)xsim::resolve_scene_file(config_for(moving), ""); }
    catch (const std::exception &error) {
      refused = std::string(error.what()).find(std::string("planar contact does not support ground ") + kind) != std::string::npos;
    }
    assert(refused);
  }

  std::cout << cases.size() << " current frozen Session fixtures and strict native-input negative controls passed\n";
  std::cout << "frozen public robot identities, hybrid filtering and unchanged scientific config passed\n";
}
