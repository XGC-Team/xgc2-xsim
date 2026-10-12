#pragma once
#include "core/entity.hpp"
#include <nlohmann/json.hpp>
namespace xsim {
using Json = nlohmann::json;
Config parse_entity(const Json &);
Json entity_artifact(const Json &);
Json load_manifest(const std::string &path);
// Call after experiment_config validates the frozen Session. Checks identities and
// returns internal entity name -> exact public ID from the simulated roster.
Json experiment_robot_bindings(const Json &input);
Json parse_json_object(const std::string &contents);
Json load_json_object(const std::string &path);
Json resolve_scene_file(Json config, const std::string &scene_file);
// epoch_ns is sampled once by the native process, never supplied by Session authoring.
Json experiment_config(const Json &input, int64_t epoch_ns);
Json load_config(const std::string &path, const std::string &scene_file = "");
} // namespace xsim
