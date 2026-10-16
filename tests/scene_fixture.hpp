#pragma once
#include <nlohmann/json.hpp>

// Test stand-in for the Core cold compiler. Legacy obstacle lists become one
// compiled.v1 document: each shape is one geometry, one model, one rigid instance.
inline nlohmann::json sceneFixture(nlohmann::json configuration) {
  using Json = nlohmann::json;
  if (!configuration.contains("obstacles")) return configuration;
  Json geometries = Json::array();
  Json models = Json::object();
  Json obstacles = Json::array();
  for (auto geometry : configuration.at("obstacles")) {
    Json pose{{"position", geometry.at("position")}};
    geometry.erase("position");
    if (geometry.contains("orientation")) {
      const auto q = geometry.at("orientation"); // legacy test vectors are wxyz
      pose["orientation"] = {q[1], q[2], q[3], q[0]};
      geometry.erase("orientation");
    }
    const auto index = geometries.size();
    const auto name = "fixture-" + std::to_string(index);
    geometries.push_back(std::move(geometry));
    models[name] = {{"parts", Json::array({{{"id", "body"},
        {"pose", {{"position", {0, 0, 0}}, {"orientation", {0, 0, 0, 1}}}},
        {"geometryIndex", index}, {"color", {1.0, 0.75, 0.0, 1.0}}}})}};
    obstacles.push_back({{"id", name}, {"name", name}, {"model", name}, {"pose", pose},
        {"motion", {{"type", "hold"}}}});
  }
  configuration.erase("obstacles");
  configuration["document"] = {{"schema", "xgc2.scene.compiled.v1"}, {"id", "fixture"}, {"frame", "world"},
      {"ground", {{"mode", "default_plane"}, {"z", 0}}}, {"geometries", std::move(geometries)},
      {"models", std::move(models)}, {"obstacles", std::move(obstacles)}};
  return configuration;
}
