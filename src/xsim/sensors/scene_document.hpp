#pragma once

#include <nlohmann/json.hpp>
#include <xgc2_math/geometry/geometry_json.hpp>
#include <xgc2_math/scene/mesh_bvh.hpp>
#include <xgc2_math/scene/scene_conversion.hpp>

#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace xsim {
// One rigid part of a moving obstacle. `local_pose` stays in the model frame.
// The world pose is composed only when an external SceneState arrives.
struct DynamicPart {
    std::size_t obstacle_index = 0;
    xgc2_math::Pose3 local_pose;
};
struct DynamicObstacle {
    std::string id;
    xgc2_math::Pose3 pose;
    std::vector<DynamicPart> parts;
};

// Cold numeric scene. Geometries are decoded once; instances are rigid poses.
// Mesh instances share one TriangleMeshPtr. No SDF/glTF IO and no runtime scale.
// Hold obstacles stay in `obstacles` only. twist/ping_pong/circle also keep a
// part index. There is no motion interpreter here.
struct CompiledScene {
    enum class GroundMode { DefaultPlane, Model, None };
    std::vector<xgc2_math::scene::Obstacle> obstacles;
    std::vector<DynamicObstacle> dynamic_obstacles;
    GroundMode ground_mode = GroundMode::DefaultPlane;
    bool planar_ground = true;
    double ground_z = 0;
};

inline CompiledScene compiledSceneDocument(const nlohmann::json& document) {
    using Json = nlohmann::json;
    if (!document.is_object() || document.value("schema", std::string{}) != "xgc2.scene.compiled.v1")
        throw std::invalid_argument("authored scene document is not a runtime input");
    if (!document.at("id").is_string() || document.at("id").get<std::string>().empty())
        throw std::invalid_argument("compiled scene id must be a string");
    if (document.at("frame") != "world")
        throw std::invalid_argument("compiled scene frame must be world");
    if (!document.at("geometries").is_array() || !document.at("models").is_object() ||
        !document.at("obstacles").is_array())
        throw std::invalid_argument("compiled scene needs geometries, models and obstacles");

    const auto v3 = [](const Json& value) -> Eigen::Vector3d {
        if (!value.is_array() || value.size() != 3) throw std::invalid_argument("scene vector needs 3 values");
        Eigen::Vector3d out(value.at(0).get<double>(), value.at(1).get<double>(), value.at(2).get<double>());
        if (!out.allFinite()) throw std::invalid_argument("nonfinite scene vector");
        return out;
    };
    const auto pose_of = [&](const Json& value) {
        xgc2_math::Pose3 result;
        result.position = v3(value.value("position", Json::array({0.0, 0.0, 0.0})));
        const auto q = value.value("orientation", Json::array({0.0, 0.0, 0.0, 1.0}));
        if (!q.is_array() || q.size() != 4) throw std::invalid_argument("scene quaternion needs xyzw");
        result.orientation = Eigen::Quaterniond(q[3].get<double>(), q[0].get<double>(), q[1].get<double>(),
                                                q[2].get<double>());
        if (!result.orientation.coeffs().allFinite()) throw std::invalid_argument("nonfinite scene quaternion");
        return result;
    };

    // One local Obstacle per geometry. Instances copy this value and replace the pose.
    struct GeometryTemplate {
        xgc2_math::scene::Obstacle local;
    };
    std::vector<GeometryTemplate> geometries;
    geometries.reserve(document.at("geometries").size());
    for (const auto& item : document.at("geometries")) {
        GeometryTemplate entry;
        if (item.value("type", std::string{}) == "mesh") {
            const auto payload = xgc2_math::geometry::geometryFromJson(item);
            const auto decoded = xgc2_math::geometry::decodeGeometry(payload);
            if (decoded.type != xgc2_math::geometry::GeometryType::Mesh)
                throw std::invalid_argument("mesh geometry did not decode as a mesh");
            entry.local = xgc2_math::scene::Obstacle::mesh(xgc2_math::scene::prepareTriangleMesh(
                xgc2_math::geometry::preparation_detail::eigenVertices(decoded), decoded.triangles));
            xgc2_math::scene::validateObstacle(entry.local);
        } else {
            entry.local = xgc2_math::scene::obstacleFromGeometry(xgc2_math::geometry::geometryFromJson(item));
        }
        geometries.push_back(std::move(entry));
    }

    struct PartTemplate {
        xgc2_math::Pose3 pose;
        std::uint32_t geometry_index = 0;
    };
    struct ModelTemplate {
        std::vector<PartTemplate> parts;
        bool unparsed_notes = false;
        bool unparsed_invalid = false;
    };
    std::map<std::string, ModelTemplate> models;
    for (auto it = document.at("models").begin(); it != document.at("models").end(); ++it) {
        const auto& model = it.value();
        if (!model.is_object() || model.contains("scale"))
            throw std::invalid_argument("compiled model '" + it.key() + "' has no runtime scale");
        ModelTemplate stored;
        if (model.contains("resource")) {
            const auto& resource = model.at("resource");
            if (!resource.is_object()) throw std::invalid_argument("compiled resource must be an object");
            const auto format = resource.at("format").get<std::string>();
            if (format != "sdformat-model" && format != "gltf" && format != "mesh")
                throw std::invalid_argument("compiled resource format is not numeric");
            if (!resource.at("file").is_string() || resource.at("file").get<std::string>().empty())
                throw std::invalid_argument("compiled resource file must be a string");
        }
        const auto parts = model.value("parts", Json::array());
        if (!parts.is_array()) throw std::invalid_argument("compiled model parts must be an array");
        for (const auto& part : parts) {
            if (!part.is_object() || part.contains("geometry") || part.contains("scale"))
                throw std::invalid_argument("compiled part references geometryIndex");
            if (!part.at("id").is_string() || part.at("id").get<std::string>().empty())
                throw std::invalid_argument("compiled part id must be a string");
            if (part.contains("color")) {
                const auto& color = part.at("color");
                if (!color.is_array() || color.size() != 4) throw std::invalid_argument("part color needs 4 numbers");
                for (const auto& channel : color)
                    if (!std::isfinite(channel.get<double>())) throw std::invalid_argument("nonfinite part color");
            }
            const auto& index_json = part.at("geometryIndex");
            if (!index_json.is_number_integer() || index_json.get<std::int64_t>() < 0 ||
                index_json.get<std::int64_t>() > static_cast<std::int64_t>(UINT32_MAX))
                throw std::invalid_argument("geometryIndex must be a uint32");
            const auto index = index_json.get<std::uint32_t>();
            if (index >= geometries.size()) throw std::invalid_argument("geometryIndex is out of range");
            stored.parts.push_back(PartTemplate{pose_of(part.at("pose")), index});
        }
        if (model.contains("unparsed")) {
            const auto& notes = model.at("unparsed");
            if (!notes.is_array()) stored.unparsed_invalid = true;
            else {
                for (const auto& note : notes)
                    if (!note.is_string()) stored.unparsed_invalid = true;
                stored.unparsed_notes = !notes.empty() && !stored.unparsed_invalid;
            }
        }
        models.emplace(it.key(), std::move(stored));
    }

    CompiledScene result;
    const auto append_model = [&](const std::string& name, const xgc2_math::Pose3& parent, const std::string& where,
                                  std::vector<DynamicPart>* dynamic_parts) {
        const auto found = models.find(name);
        if (found == models.end()) throw std::invalid_argument(where + " model '" + name + "' is missing");
        if (found->second.unparsed_invalid)
            throw std::invalid_argument("compiled unparsed annotations must be strings");
        if (found->second.unparsed_notes)
            throw std::invalid_argument("static LidarScene does not support unparsed notes on model '" + name + "'");
        const auto orientation = xgc2_math::normalizedQuaternion(parent.orientation);
        for (const auto& part : found->second.parts) {
            const auto& geometry = geometries.at(part.geometry_index);
            const auto local = xgc2_math::normalizedQuaternion(part.pose.orientation);
            auto shape = geometry.local;
            shape.position = parent.position + orientation * part.pose.position;
            shape.orientation = orientation * local;
            if (dynamic_parts)
                dynamic_parts->push_back(DynamicPart{result.obstacles.size(), part.pose});
            result.obstacles.push_back(std::move(shape));
        }
    };

    std::set<std::string> obstacle_ids;
    for (const auto& obstacle : document.at("obstacles")) {
        if (!obstacle.is_object() || obstacle.contains("parts") || obstacle.contains("geometry") ||
            obstacle.contains("scale"))
            throw std::invalid_argument("compiled obstacle has no inline geometry");
        const auto id = obstacle.at("id").get<std::string>();
        if (id.empty() || !obstacle.at("name").is_string())
            throw std::invalid_argument("compiled obstacle needs id and name");
        if (!obstacle_ids.insert(id).second)
            throw std::invalid_argument("compiled obstacle id is duplicated");
        if (!obstacle.contains("motion") || !obstacle.at("motion").is_object() ||
            !obstacle.at("motion").contains("type") || !obstacle.at("motion").at("type").is_string())
            throw std::invalid_argument(
                "compiled LidarScene accepts only hold, constant_twist, ping_pong or circle motion");
        const auto motion = obstacle.at("motion").at("type").get<std::string>();
        if (motion != "hold" && motion != "constant_twist" && motion != "ping_pong" && motion != "circle")
            throw std::invalid_argument(
                "compiled LidarScene accepts only hold, constant_twist, ping_pong or circle motion");
        const auto parent = pose_of(obstacle.at("pose"));
        std::vector<DynamicPart> parts;
        try {
            append_model(obstacle.at("model").get<std::string>(), parent, "obstacle '" + id + "'",
                         motion == "hold" ? nullptr : &parts);
        } catch (const std::exception& error) {
            throw std::invalid_argument("obstacle '" + id + "': " + error.what());
        }
        if (motion != "hold") {
            DynamicObstacle moving;
            moving.id = id;
            moving.pose = parent;
            moving.pose.orientation = xgc2_math::normalizedQuaternion(parent.orientation);
            moving.parts = std::move(parts);
            result.dynamic_obstacles.push_back(std::move(moving));
        }
    }

    const auto ground = document.value("ground", Json::object());
    if (!ground.is_object()) throw std::invalid_argument("compiled ground must be an object");
    const auto mode = ground.value("mode", std::string("default_plane"));
    if (mode == "default_plane") {
        if (ground.contains("model")) throw std::invalid_argument("default_plane ground has no model");
        result.ground_mode = CompiledScene::GroundMode::DefaultPlane;
        result.planar_ground = true;
        result.ground_z = ground.value("z", 0.0);
        if (!std::isfinite(result.ground_z)) throw std::invalid_argument("nonfinite ground z");
    } else if (mode == "none") {
        if (ground.contains("z") || ground.contains("model"))
            throw std::invalid_argument("ground none is not a plane");
        result.ground_mode = CompiledScene::GroundMode::None;
        result.planar_ground = false;
        result.ground_z = 0;
    } else if (mode == "model") {
        if (ground.contains("z")) throw std::invalid_argument("ground model is not a sampled plane");
        result.ground_mode = CompiledScene::GroundMode::Model;
        result.planar_ground = false;
        result.ground_z = 0;
        append_model(ground.at("model").get<std::string>(), pose_of(ground.at("pose")), "ground", nullptr);
    } else {
        throw std::invalid_argument("unknown compiled ground mode");
    }
    return result;
}

inline std::vector<xgc2_math::scene::Obstacle> sceneDocumentObstacles(const nlohmann::json& document) {
    return compiledSceneDocument(document).obstacles;
}
} // namespace xsim
