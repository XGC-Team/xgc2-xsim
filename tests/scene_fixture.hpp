#pragma once
#include <nlohmann/json.hpp>

// Test builder only. Every sensor is constructed through the production document entry.
inline nlohmann::json sceneFixture(nlohmann::json configuration) {
  using Json = nlohmann::json;
  if (!configuration.contains("obstacles")) return configuration;
  Json obstacles = Json::array();
  for (auto geometry : configuration.at("obstacles")) {
    Json pose{{"position", geometry.at("position")}};
    geometry.erase("position");
    if (geometry.contains("orientation")) {
      const auto q = geometry.at("orientation"); // legacy test vectors are wxyz
      pose["orientation"] = {q[1], q[2], q[3], q[0]};
      geometry.erase("orientation");
    }
    obstacles.push_back({{"id", "fixture-" + std::to_string(obstacles.size())}, {"pose", pose},
      {"parts", Json::array({{{"id", "body"}, {"geometry", geometry}}})}});
  }
  configuration.erase("obstacles");
  configuration["document"] = {{"schema", "xgc2.scene.v1"}, {"id", "fixture"},
      {"frame", "world"}, {"obstacles", obstacles}};
  return configuration;
}
