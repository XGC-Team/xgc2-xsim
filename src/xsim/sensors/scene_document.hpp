#pragma once

#include <nlohmann/json.hpp>
#include <xgc2_math/scene/scene_conversion.hpp>

namespace xsim {
// Authored scene document -> named Math parameters. Pose quaternions in the
// document are xyzw; Math receives the composed rigid obstacle/part frames.
inline std::vector<xgc2_math::scene::Obstacle> sceneDocumentObstacles(const nlohmann::json& document) {
    using Json = nlohmann::json;
    const auto v3 = [](const Json& value) -> Eigen::Vector3d {
        if (!value.is_array() || value.size() != 3) throw std::invalid_argument("scene vector needs 3 values");
        return {value.at(0).get<double>(), value.at(1).get<double>(), value.at(2).get<double>()};
    };
    const auto pose = [&](const Json& value) {
        xgc2_math::Pose3 result;
        result.position = v3(value.value("position", Json::array({0, 0, 0})));
        const auto q = value.value("orientation", Json::array({0, 0, 0, 1}));
        if (!q.is_array() || q.size() != 4) throw std::invalid_argument("scene quaternion needs xyzw");
        result.orientation = Eigen::Quaterniond(q[3].get<double>(), q[0].get<double>(),
                                                q[1].get<double>(), q[2].get<double>());
        return result;
    };
    std::vector<xgc2_math::scene::SceneObstacleDescription> descriptions;
    for (const auto& obstacle : document.at("obstacles")) {
        xgc2_math::scene::SceneObstacleDescription description;
        description.id = obstacle.at("id");
        description.pose = pose(obstacle.value("pose", Json::object()));
        for (const auto& part : obstacle.at("parts")) {
            const auto& geometry = part.at("geometry");
            xgc2_math::scene::ScenePartDescription converted;
            converted.type = geometry.at("type");
            converted.pose = pose(part.value("pose", Json::object()));
            if (geometry.contains("size")) converted.size = v3(geometry.at("size"));
            converted.radius = geometry.value("radius", 0.0);
            converted.height = geometry.value("height", 0.0);
            if (converted.type == "ring") {
                const auto& radii = geometry.at("radii");
                const auto& tube = geometry.at("tubeRadius");
                if (!radii.is_array() || radii.size() != 2 || !radii[0].is_number() ||
                    !radii[1].is_number() || !tube.is_number())
                    throw std::invalid_argument("ring needs two numeric radii and numeric tubeRadius");
                converted.radii = {radii[0].get<double>(), radii[1].get<double>()};
                converted.tubeRadius = tube.get<double>();
            }
            if (geometry.contains("vertices"))
                for (const auto& vertex : geometry.at("vertices")) converted.vertices.push_back(v3(vertex));
            description.parts.push_back(std::move(converted));
        }
        descriptions.push_back(std::move(description));
    }
    return xgc2_math::scene::toObstacles(descriptions);
}
} // namespace xsim
