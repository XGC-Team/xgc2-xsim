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
// Empty slot_id keeps every simulated plant. A non-empty slot_id returns that one body.
Json experiment_robot_bindings(const Json &input, const std::string &slot_id = {});
Json parse_json_object(const std::string &contents);
Json load_json_object(const std::string &path);
// Non-empty scene_file replaces config["scene_file"] and that file is loaded once.
// Empty keeps the frozen path. The argument is by value, so the caller's JSON is unchanged.
// A compiled default_plane writes its z onto each entity once. model/none with entities
// is unsupported planar contact. Geometry is not decoded here.
Json resolve_scene_file(Json config, const std::string &scene_file);
// epoch_ns is sampled once by the native process, never supplied by Session authoring.
// slot_id is the process --slot value, not a Session field. Empty keeps Type1.
// phase_t0_ns, when non-null, is an independent phase count and is not epoch_ns.
Json experiment_config(const Json &input, int64_t epoch_ns, const std::string &slot_id = {},
                       const int64_t *phase_t0_ns = nullptr);
Json load_config(const std::string &path, const std::string &scene_file = "");
} // namespace xsim
