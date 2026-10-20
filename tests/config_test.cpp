#include "io/config.hpp"
#include "sensors/scene_document.hpp"
#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <sys/stat.h>
#include <unistd.h>

namespace {
template <class Function> void rejects(Function function, const std::string &message = "") {
  bool rejected = false;
  try { function(); }
  catch (const std::exception &error) {
    rejected = true;
    if (!message.empty()) assert(std::string(error.what()).find(message) != std::string::npos);
  }
  assert(rejected);
}
}

int main() {
  char pattern[] = "/tmp/xsim-config-test-XXXXXX";
  const char *created = mkdtemp(pattern);
  assert(created);
  const std::filesystem::path root(created);
  struct Remove { std::filesystem::path root; ~Remove() { std::filesystem::remove_all(root); } } remove{root};
  const auto configuration = (root / "world.json").string();
  const auto scene = (root / "scene.yaml").string();
  const auto other = (root / "other.yaml").string();
  std::ofstream(scene) << R"({"name":"original-scene","enabled":true,"origin":[1,2.5,-3],"objects":[{"id":"1","name":"false","label":"42","vertices":[[0,0,0],[1,2,3]]}]})";
  std::ofstream(other) << R"({"name":"other-scene"})";
  xsim::Json original{{"instance_id", "frozen-instance"}, {"epoch_ns", int64_t(1791250000000000000)},
                      {"entities", xsim::Json::array()}, {"scene", {{"keep", "source-value"}}}};
  auto write = [&](const xsim::Json &value) { std::ofstream(configuration) << value.dump(); };
  write(original);
  assert(xsim::load_config(configuration) == original);
  assert(xsim::load_config(configuration, "") == original);
  auto from_flag = xsim::load_config(configuration, scene);
  auto configured = original;
  configured["scene_file"] = scene;
  write(configured);
  const auto from_json = xsim::load_config(configuration);
  assert(from_flag == from_json);
  assert(xsim::load_config(configuration, scene) == from_json);
  assert(xsim::load_config(configuration, "") == from_json);
  assert(from_flag.at("epoch_ns") == original.at("epoch_ns"));
  assert(from_flag.at("instance_id") == original.at("instance_id"));
  assert(from_flag.at("entities") == original.at("entities"));
  assert(from_flag.at("scene").at("keep") == "source-value");
  auto reference = original;
  reference["scene_file"] = scene;
  reference["reference_cloud_topic"] = "/xgc/scene/reference_cloud";
  assert(xsim::resolve_scene_file(reference, scene).contains("reference_cloud_topic"));
  // A manifest beside the scene is not read: the saved cloud is the frozen sceneAsset.pointCloud
  // fact that experiment_config applies (see experiment_test).
  std::ofstream(root / "manifest.json") << R"({"pointCloud":{"file":"original-volume.xyz.f32"}})";
  reference["entities"].push_back({{"name", "uav1"}, {"sensor", {{"backend", "gpu"}}}});
  const auto beside_manifest = xsim::resolve_scene_file(reference, scene);
  assert(beside_manifest.contains("reference_cloud_topic"));
  assert(beside_manifest.at("entities").at(0).contains("sensor"));
  std::filesystem::remove(root / "manifest.json");
  const auto &document = from_flag.at("scene").at("document");
  assert(document.at("name") == "original-scene");
  assert(document.at("enabled") == true);
  assert(document.at("origin").at(1) == 2.5);
  assert(document.at("objects").at(0).at("id").get<std::string>() == "1");
  assert(document.at("objects").at(0).at("name").get<std::string>() == "false");
  assert(document.at("objects").at(0).at("label").get<std::string>() == "42");
  const auto overridden = xsim::load_config(configuration, other);
  assert(overridden.at("scene_file") == other);
  assert(overridden.at("scene").at("document").at("name") == "other-scene");
  assert(overridden.at("epoch_ns") == original.at("epoch_ns"));
  assert(xsim::load_json_object(configuration).at("scene_file") == scene);
  configured["scene_file"] = (root / "missing-from-config.yaml").string();
  write(configured);
  const auto once = xsim::load_config(configuration, other);
  assert(once.at("scene_file") == other);
  assert(once.at("scene").at("document").at("name") == "other-scene");
  rejects([&] { xsim::load_config(configuration, ""); });
  configured["scene_file"] = "";
  write(configured);
  assert(xsim::load_config(configuration, scene) == from_json);
  const auto kept = xsim::load_config(configuration, "");
  assert(kept.at("scene_file") == "");
  assert(!kept.at("scene").contains("document"));
  rejects([&] { xsim::load_config(configuration, (root / "missing.yaml").string()); });
  std::ofstream(other) << "{";
  rejects([&] { xsim::load_config(configuration, other); });
  configured["scene_file"] = 5;
  write(configured);
  rejects([&] { xsim::load_config(configuration); }, "scene_file must be a string");
  const auto typed = xsim::load_config(configuration, scene);
  assert(typed.at("scene_file") == scene);
  assert(typed.at("scene").at("document").at("name") == "original-scene");
  assert(configured.at("epoch_ns") == original.at("epoch_ns"));
  // Native fields retain their original types/values; only the JSON boundary
  // is stricter, not a second limited Core-side product DTO.
  auto native = original;
  native["native_extension"] = {{"weights", {1, 2.5, -3}}, {"label", "source"}};
  write(native);
  assert(xsim::load_config(configuration) == native);
  for (const auto &text : {"{} {}", "{} trailing", "{\"epoch_ns\":1,\"epoch_ns\":2}",
                          "{\"outer\":[{\"x\":1,\"x\":2}]}", "[]", "{\"x\":NaN}"}) {
    std::ofstream(configuration) << text;
    rejects([&] { xsim::load_config(configuration); });
  }
  std::ofstream(configuration) << "{\"a\":{\"x\":1},\"b\":{\"x\":2}} \n\t";
  assert(xsim::load_config(configuration).at("b").at("x") == 2);
  xsim::Json boundary{{"padding", ""}};
  boundary["padding"] = std::string(1024 * 1024 - boundary.dump().size(), 'x');
  assert(boundary.dump().size() == 1024 * 1024);
  write(boundary);
  assert(xsim::load_config(configuration) == boundary);
  std::ofstream(configuration) << std::string(1024 * 1024 + 1, ' ');
  rejects([&] { xsim::load_config(configuration); }, "1 MiB");
  std::ofstream(configuration) << "";
  rejects([&] { xsim::load_config(configuration); }, "nonempty regular");
  rejects([&] { xsim::load_config(root.string()); }, "regular");
  const auto fifo = (root / "fifo").string();
  assert(mkfifo(fifo.c_str(), 0600) == 0);
  rejects([&] { xsim::load_config(fifo); }, "regular");
  const auto link = (root / "linked.json").string();
  assert(symlink(configuration.c_str(), link.c_str()) == 0);
  rejects([&] { xsim::load_config(link); }, "symbolic links");
  const auto write_compiled = [&](const xsim::Json *ground) {
    xsim::Json document{{"schema", "xgc2.scene.compiled.v1"}, {"id", "ground"}, {"frame", "world"},
                        {"geometries", xsim::Json::array({{{"type", "not-a-shape"}}})},
                        {"models", xsim::Json::object()}, {"obstacles", xsim::Json::array()}};
    if (ground) document["ground"] = *ground;
    const auto path = (root / "compiled-ground.json").string();
    std::ofstream(path) << document.dump();
    return path;
  };
  xsim::Json robots = original;
  robots["entities"] = xsim::Json::array({
      {{"name", "a"}, {"kind", "scout"}, {"ground_z", 9.0}},
      {{"name", "b"}, {"kind", "mecanum"}, {"ground_z", 9.0}}});
  const xsim::Json plane{{"mode", "default_plane"}, {"z", 1.5}};
  const auto wired = xsim::resolve_scene_file(robots, write_compiled(&plane));
  assert(wired.at("scene").at("document").at("geometries").at(0).at("type") == "not-a-shape");
  for (const auto &entity : wired.at("entities")) {
    assert(entity.at("ground_z") == 1.5);
    assert(xsim::parse_entity(entity).ground_z == 1.5);
  }
  const auto omitted = xsim::resolve_scene_file(robots, write_compiled(nullptr));
  for (const auto &entity : omitted.at("entities")) assert(entity.at("ground_z") == 0.0);
  xsim::Json authored = robots;
  const auto kept_plane = xsim::resolve_scene_file(authored, scene);
  assert(kept_plane.at("entities").at(0).at("ground_z") == 9.0);
  const xsim::Json bare_none{{"mode", "none"}};
  const xsim::Json bare_model{{"mode", "model"}, {"model", "terrain"},
                              {"pose", {{"position", {0, 0, 4}}}}};
  rejects([&] { xsim::resolve_scene_file(robots, write_compiled(&bare_none)); },
          "planar contact does not support ground none");
  rejects([&] { xsim::resolve_scene_file(robots, write_compiled(&bare_model)); },
          "planar contact does not support ground model");
  const auto lidar_only = xsim::resolve_scene_file(original, write_compiled(&bare_model));
  assert(lidar_only.at("entities").empty());
  assert(lidar_only.at("scene").at("document").at("ground").at("mode") == "model");
  const auto integers = root / "compiled-integers.json";
  std::ofstream(integers) << R"({"schema":"xgc2.scene.compiled.v1","id":"integers","frame":"world","ground":{"mode":"default_plane","z":0},"geometries":[{"type":"box","size":[1,1,1]},{"type":"mesh","vertices":[[0,0,0],[1,0,0],[0,1,0],[0,0,1]],"triangles":[0,2,1,0,1,3,0,3,2,1,2,3]}],"models":{"crate":{"parts":[{"id":"body","geometryIndex":0,"pose":{"position":[0,0,0],"orientation":[0,0,0,1]}}]},"tet":{"parts":[{"id":"shell","geometryIndex":1,"pose":{"position":[0,0,0],"orientation":[0,0,0,1]}}]}},"obstacles":[{"id":"left","name":"Left","model":"tet","pose":{"position":[0,0,0],"orientation":[0,0,0,1]},"motion":{"type":"hold"}},{"id":"right","name":"Right","model":"tet","pose":{"position":[3,0,0],"orientation":[0,0,0,1]},"motion":{"type":"hold"}},{"id":"box","name":"Box","model":"crate","pose":{"position":[0,2,0],"orientation":[0,0,0,1]},"motion":{"type":"hold"}}]})";
  const auto loaded = xsim::resolve_scene_file(original, integers.string());
  const auto &integer_document = loaded.at("scene").at("document");
  assert(integer_document.at("geometries").size() == 2);
  assert(integer_document.at("obstacles").size() == 3);
  const auto &box_index = integer_document.at("models").at("crate").at("parts").at(0).at("geometryIndex");
  const auto &mesh_index = integer_document.at("models").at("tet").at("parts").at(0).at("geometryIndex");
  assert(box_index.is_number_integer() && box_index.get<std::uint32_t>() == 0);
  assert(mesh_index.is_number_integer() && mesh_index.get<std::uint32_t>() == 1);
  for (const auto &index : integer_document.at("geometries").at(1).at("triangles"))
    assert(index.is_number_integer());
  const auto shapes = xsim::sceneDocumentObstacles(integer_document);
  assert(shapes.size() == 3);
  assert(shapes[0].meshData && shapes[0].meshData.get() == shapes[1].meshData.get());
  assert(shapes[0].meshData.use_count() >= 2);
  assert(shapes[2].type == xgc2_math::scene::Obstacle::kBox);
  const auto asset_dir = root / "frozen-asset";
  const auto compiled_dir = root / "compiled-scenes" / "session";
  std::filesystem::create_directories(asset_dir);
  std::filesystem::create_directories(compiled_dir);
  const auto frozen_source = (asset_dir / "scene.yaml").string();
  const auto compiled = (compiled_dir / "scene.compiled.json").string();
  std::ofstream(frozen_source) << "name: [source-geometry-must-not-load\n";
  std::ofstream(compiled) << R"({"schema":"xgc2.scene.compiled.v1","name":"compiled-only","geometries":[]})";
  std::ofstream(compiled_dir / "manifest.json") << R"({"pointCloud":{"file":"decoy.xyz.f32"}})";
  std::ofstream(asset_dir / "manifest.json") << R"({"pointCloud":{"file":"original-volume.xyz.f32"}})";
  xsim::Json split = original;
  split["scene_file"] = frozen_source;
  split["reference_cloud_topic"] = "/xgc/scene/reference_cloud";
  split["entities"] = xsim::Json::array({{{"name", "uav1"}, {"sensor", {{"backend", "gpu"}}}}});
  const auto owned = xsim::resolve_scene_file(split, compiled);
  assert(owned.at("scene_file") == compiled);
  assert(owned.at("scene").at("document").at("name") == "compiled-only");
  // Neither the frozen source's manifest nor the compiled file's is consulted.
  assert(owned.contains("reference_cloud_topic"));
  assert(owned.at("entities").at(0).contains("sensor"));
  auto unfrozen = split;
  unfrozen.erase("scene_file");
  const auto no_source = xsim::resolve_scene_file(unfrozen, compiled);
  assert(no_source.contains("reference_cloud_topic"));
  assert(no_source.at("entities").at(0).contains("sensor"));
  unfrozen["scene_file"] = "";
  const auto empty_source = xsim::resolve_scene_file(unfrozen, compiled);
  assert(empty_source.contains("reference_cloud_topic"));
  assert(empty_source.at("entities").at(0).contains("sensor"));
  std::cout << "scene flag/config parity, empty flag, host override, parser errors and frozen epoch passed\n";
}
