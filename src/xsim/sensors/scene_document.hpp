#pragma once

#include <nlohmann/json.hpp>
#include <xgc2_math/scene/scene_conversion.hpp>
#include <xgc2_math/geometry/geometry_json.hpp>

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
    std::vector<xgc2_math::scene::Obstacle> shapes;
    for (const auto& obstacle : document.at("obstacles")) {
        const auto parent = pose(obstacle.value("pose", Json::object()));
        for (const auto& part : obstacle.at("parts")) {
            try {
                const auto local = pose(part.value("pose", Json::object()));
                const auto orientation = xgc2_math::normalizedQuaternion(parent.orientation);
                shapes.push_back(xgc2_math::scene::obstacleFromGeometry(
                    xgc2_math::geometry::geometryFromJson(part.at("geometry")),
                    parent.position + orientation * local.position,
                    orientation * xgc2_math::normalizedQuaternion(local.orientation)));
            } catch (const std::exception& error) {
                throw std::invalid_argument("obstacle '" + obstacle.value("id", std::string{}) + "': " + error.what());
            }
        }
    }
    return shapes;
}
} // namespace xsim
