#include "sensors/scene_document.hpp"
#include <xgc2_math/scene/lidar_scene.hpp>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
nlohmann::json pose(const nlohmann::json& position, const nlohmann::json& orientation = {0, 0, 0, 1}) {
    return {{"position", position}, {"orientation", orientation}};
}
nlohmann::json part(const char* id, int index, const nlohmann::json& position, const nlohmann::json& orientation) {
    return {{"id", id}, {"pose", pose(position, orientation)}, {"geometryIndex", index}};
}
nlohmann::json instance(const char* id, const char* name, const char* model, const nlohmann::json& position,
                        const nlohmann::json& motion) {
    return {{"id", id}, {"name", name}, {"model", model}, {"pose", pose(position)}, {"motion", motion}};
}
nlohmann::json document_with(const nlohmann::json& geometry, const nlohmann::json& child_orientation,
                             const Eigen::Quaterniond& parent_orientation) {
    using nlohmann::json;
    const json parent_q = {parent_orientation.x(), parent_orientation.y(), parent_orientation.z(), parent_orientation.w()};
    json document = {{"schema", "xgc2.scene.compiled.v1"}, {"id", "ring"}, {"frame", "world"},
                     {"ground", {{"mode", "default_plane"}, {"z", 0}}}, {"geometries", json::array({geometry})},
                     {"models", json::object()}, {"obstacles", json::array()}};
    document["models"]["ring"] = {{"parts", json::array({part("body", 0, {.1, .2, .3}, child_orientation)})}};
    document["obstacles"].push_back({{"id", "ring"}, {"name", "Ring"}, {"model", "ring"},
                                     {"pose", pose({2, 3, 4}, parent_q)}, {"motion", {{"type", "hold"}}}});
    return document;
}
void require_throw(const nlohmann::json& document, const char* label, const char* snippet = nullptr) {
    bool rejected = false;
    try { xsim::compiledSceneDocument(document); }
    catch (const std::invalid_argument& error) {
        rejected = !snippet || std::string(error.what()).find(snippet) != std::string::npos;
    }
    if (!rejected) throw std::runtime_error(label);
}
}

int main() {
    using nlohmann::json;
    const Eigen::Quaterniond parent(Eigen::AngleAxisd(.6, Eigen::Vector3d::UnitY()));
    const Eigen::Quaterniond child(Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitZ()));
    const json child_q = {child.x(), child.y(), child.z(), child.w()};
    json document = document_with({{"type", "ring"}, {"radii", {1., 1.2}}, {"tubeRadius", .05}}, child_q, parent);
    const auto shapes = xsim::sceneDocumentObstacles(document);
    if (shapes.size() != 1 || shapes[0].type != xgc2_math::scene::Obstacle::kRing ||
        shapes[0].radii != Eigen::Vector2d(1, 1.2) || shapes[0].tubeRadius != .05 ||
        (shapes[0].position - (Eigen::Vector3d(2, 3, 4) + parent * Eigen::Vector3d(.1, .2, .3))).norm() > 1e-12 ||
        shapes[0].orientation.angularDistance(parent * child) > 1e-12)
        throw std::runtime_error("typed document ring/poses");
    for (const auto invalid : {json::array({true, 1.2}), json::array({1.}), json::array({1., 1.2, 1.3}),
                               json::array({0., 1.2})}) {
        auto bad = document;
        bad["geometries"][0]["radii"] = invalid;
        bool rejected = false;
        try { xsim::sceneDocumentObstacles(bad); } catch (const std::exception&) { rejected = true; }
        if (!rejected) throw std::runtime_error("invalid ring accepted: " + invalid.dump());
    }
    auto typed = document;
    typed["geometries"][0] = {{"type", "ellipsoid"}, {"semiaxes", {1, 2, 3}}};
    const auto ellipsoid = xsim::sceneDocumentObstacles(typed);
    if (ellipsoid.size() != 1 || ellipsoid[0].type != xgc2_math::scene::Obstacle::kEllipsoid ||
        ellipsoid[0].size != Eigen::Vector3d(1, 2, 3))
        throw std::runtime_error("ellipsoid retains analytic semiaxes");
    typed["geometries"][0] = {{"type", "halfspaces"}, {"halfspaces", json::array({
        json::array({1, 0, 0, -1}), json::array({-1, 0, 0, -1}), json::array({0, 1, 0, -1}),
        json::array({0, -1, 0, -1}), json::array({0, 0, 1, -1}), json::array({0, 0, -1, -1})})}};
    const auto bounded = xsim::sceneDocumentObstacles(typed);
    if (bounded.size() != 1 || bounded[0].type != xgc2_math::scene::Obstacle::kConvex || bounded[0].vertices.size() != 8)
        throw std::runtime_error("bounded numeric H input");
    typed["geometries"][0] = {{"type", "convex"}, {"vertices", json::array({
        json::array({0, 0, 0}), json::array({1, 0, 0}), json::array({0, 1, 0}), json::array({0, 0, 1})})}};
    const auto temporary = xsim::sceneDocumentObstacles(typed);
    if (temporary.size() != 1 || temporary[0].vertices.size() != 4)
        throw std::runtime_error("temporary unregistered hull");

    const json mesh = {{"type", "mesh"},
                       {"vertices", json::array({json::array({0, 0, 0}), json::array({1, 0, 0}), json::array({0, 1, 0}),
                                                 json::array({0, 0, 1})})},
                       {"triangles", json::array({0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3})}};
    json shared = {{"schema", "xgc2.scene.compiled.v1"}, {"id", "share"}, {"frame", "world"},
                   {"geometries", json::array({mesh})}, {"models", json::object()}, {"obstacles", json::array()}};
    shared["models"]["tet"] = {{"parts", json::array({part("body", 0, {0, 0, 0}, {0, 0, 0, 1})})}};
    shared["obstacles"].push_back(instance("a", "A", "tet", {0, 0, 0}, {{"type", "hold"}}));
    shared["obstacles"].push_back(instance("b", "B", "tet", {3, 1, 0}, {{"type", "hold"}}));
    const auto instances = xsim::compiledSceneDocument(shared);
    if (instances.obstacles.size() != 2 || instances.obstacles[0].meshData.get() != instances.obstacles[1].meshData.get() ||
        instances.obstacles[0].meshData.use_count() < 2 ||
        (instances.obstacles[0].position - instances.obstacles[1].position).norm() < 1 ||
        (instances.obstacles[1].position - Eigen::Vector3d(3, 1, 0)).norm() > 1e-12)
        throw std::runtime_error("two hold instances must share one prepared mesh");
    xgc2_math::scene::LidarScene(instances.obstacles, 0.5, false);

    auto hole = shared;
    hole["geometries"][0]["triangles"] = json::array({0, 2, 1, 0, 1, 3, 0, 3, 2});
    hole["obstacles"].erase(1);
    require_throw(hole, "open mesh accepted");
    auto nested = shared;
    nested["geometries"][0]["triangles"] = json::array({json::array({0, 2, 1}), json::array({0, 1, 3}),
                                                       json::array({0, 3, 2}), json::array({1, 2, 3})});
    require_throw(nested, "nested triangles accepted");
    require_throw({{"schema", "xgc2.scene.v1"}, {"obstacles", json::array()}}, "authored document accepted");

    const json box = {{"type", "box"}, {"size", {1, 1, 1}}};
    const auto identity = part("body", 0, {0, 0, 0}, {0, 0, 0, 1});
    const auto slab = part("slab", 0, {0, 0, 0}, {0, 0, 0, 1});
    json base = {{"schema", "xgc2.scene.compiled.v1"}, {"id", "ground"}, {"frame", "world"},
                 {"geometries", json::array({box})}, {"models", json::object()}, {"obstacles", json::array()}};
    base["models"]["body"] = {{"parts", json::array({identity})}};
    base["models"]["terrain"] = {{"parts", json::array({slab})}};
    base["obstacles"].push_back(instance("only", "Only", "body", {2, 0, 1}, {{"type", "hold"}}));
    auto plane_doc = base;
    plane_doc["ground"] = {{"mode", "default_plane"}, {"z", 1.25}};
    const auto plane = xsim::compiledSceneDocument(plane_doc);
    if (!plane.planar_ground || plane.ground_z != 1.25 || plane.obstacles.size() != 1 ||
        plane.ground_mode != xsim::CompiledScene::GroundMode::DefaultPlane)
        throw std::runtime_error("default plane must stay a scalar and add no solid");
    const auto omitted = xsim::compiledSceneDocument(base);
    if (!omitted.planar_ground || omitted.ground_z != 0 || omitted.obstacles.size() != 1)
        throw std::runtime_error("omitted ground must keep the default plane");
    auto none_doc = base;
    none_doc["ground"] = {{"mode", "none"}};
    const auto none = xsim::compiledSceneDocument(none_doc);
    if (none.planar_ground || none.ground_z != 0 || none.obstacles.size() != 1 ||
        none.ground_mode != xsim::CompiledScene::GroundMode::None)
        throw std::runtime_error("ground none must not invent a plane");
    auto terrain = base;
    terrain["obstacles"] = json::array();
    terrain["ground"] = {{"mode", "model"}, {"model", "terrain"}, {"pose", pose({0, 0, 4})}};
    const auto sampled = xsim::compiledSceneDocument(terrain);
    if (sampled.planar_ground || sampled.ground_z != 0 || sampled.obstacles.size() != 1 ||
        sampled.obstacles[0].type != xgc2_math::scene::Obstacle::kBox ||
        (sampled.obstacles[0].position - Eigen::Vector3d(0, 0, 4)).norm() > 1e-12)
        throw std::runtime_error("ground model must not be sampled into a plane");
    auto visual = base;
    visual["ground"] = {{"mode", "none"}};
    visual["models"] = json::object();
    visual["models"]["picture"] = {{"resource", {{"format", "sdformat-model"}, {"file", "/tmp/xsim-missing-model.sdf"}}}};
    visual["obstacles"][0]["model"] = "picture";
    const auto unloaded = xsim::compiledSceneDocument(visual);
    if (!unloaded.obstacles.empty()) throw std::runtime_error("resource file was turned into collision");

    json analytic = {{"schema", "xgc2.scene.compiled.v1"}, {"id", "template"}, {"frame", "world"},
                     {"geometries", json::array({{{"type", "halfspaces"}, {"halfspaces", json::array({
                         json::array({1, 0, 0, -1}), json::array({-1, 0, 0, -1}), json::array({0, 1, 0, -1}),
                         json::array({0, -1, 0, -1}), json::array({0, 0, 1, -1}), json::array({0, 0, -1, -1})})}}})},
                     {"models", json::object()}, {"obstacles", json::array()}};
    analytic["models"]["hull"] = {{"parts", json::array({part("body", 0, {0, 0, 0}, {0, 0, 0, 1})})}};
    analytic["obstacles"].push_back(instance("left", "Left", "hull", {0, 0, 0}, {{"type", "hold"}}));
    analytic["obstacles"].push_back(instance("right", "Right", "hull", {4, 0, 0}, {{"type", "hold"}}));
    const auto copies = xsim::compiledSceneDocument(analytic);
    if (copies.obstacles.size() != 2 || copies.obstacles[0].type != xgc2_math::scene::Obstacle::kConvex ||
        copies.obstacles[0].vertices.size() != 8 || copies.obstacles[0].vertices != copies.obstacles[1].vertices ||
        copies.obstacles[0].vertices.data() == copies.obstacles[1].vertices.data() ||
        (copies.obstacles[1].position - Eigen::Vector3d(4, 0, 0)).norm() > 1e-12)
        throw std::runtime_error("analytic instances must copy one prepared hull");
    if (!plane.dynamic_obstacles.empty())
        throw std::runtime_error("hold document must not invent a dynamic index");
    for (const char* kind : {"constant_twist", "ping_pong", "circle"}) {
        auto moving = base;
        moving["obstacles"][0]["motion"] = {{"type", kind}};
        moving["models"]["body"]["parts"][0]["pose"]["position"] = {0.0, 0.25, 0.0};
        moving["epoch"] = "doc-epoch";
        const auto indexed = xsim::compiledSceneDocument(moving);
        if (indexed.dynamic_obstacles.size() != 1 ||
            indexed.dynamic_obstacles[0].id != "only" || indexed.dynamic_obstacles[0].parts.size() != 1 ||
            indexed.dynamic_obstacles[0].parts[0].obstacle_index != 0 ||
            (indexed.dynamic_obstacles[0].parts[0].local_pose.position - Eigen::Vector3d(0, 0.25, 0)).norm() > 1e-12 ||
            indexed.obstacles.size() != 1)
            throw std::runtime_error(std::string(kind) + " did not keep a part-local index");
    }
    auto mixed = base;
    mixed["obstacles"].push_back(instance("still", "Still", "body", {8, 0, 0}, {{"type", "hold"}}));
    mixed["obstacles"][0]["motion"] = {{"type", "ping_pong"}};
    const auto mixed_scene = xsim::compiledSceneDocument(mixed);
    if (mixed_scene.obstacles.size() != 2 || mixed_scene.dynamic_obstacles.size() != 1 ||
        mixed_scene.dynamic_obstacles[0].id != "only" || mixed_scene.dynamic_obstacles[0].parts[0].obstacle_index != 0)
        throw std::runtime_error("hold obstacle must stay out of the dynamic index");
    for (const char* kind : {"waypoints", "orbit"}) {
        auto moving = base;
        moving["obstacles"][0]["motion"] = {{"type", kind}};
        require_throw(moving, kind);
    }
    auto duplicated = base;
    duplicated["obstacles"].push_back(instance("only", "Again", "body", {1, 0, 0}, {{"type", "hold"}}));
    require_throw(duplicated, "duplicate obstacle id");
    auto omitted_type = base;
    omitted_type["obstacles"][0]["motion"] = json::object();
    require_throw(omitted_type, "omitted motion type accepted as hold");
    auto omitted_motion = base;
    omitted_motion["obstacles"][0].erase("motion");
    require_throw(omitted_motion, "omitted motion accepted as hold");

    const json notes = json::array({"joint:j", "plugin:motion", "actor:a", "light:lamp"});
    auto empty_notes = base;
    empty_notes["models"]["body"]["unparsed"] = json::array();
    const auto empty_ok = xsim::compiledSceneDocument(empty_notes);
    if (empty_ok.obstacles.size() != 1) throw std::runtime_error("empty unparsed rejected");
    const auto absent_ok = xsim::compiledSceneDocument(base);
    if (absent_ok.obstacles.size() != 1 || !absent_ok.planar_ground)
        throw std::runtime_error("absent unparsed rejected");
    auto referenced = base;
    referenced["models"]["body"]["unparsed"] = notes;
    require_throw(referenced, "referenced unparsed notes accepted", "does not support unparsed notes");
    auto unused = base;
    unused["models"]["parked"] = {{"parts", json::array({part("idle", 0, {0, 0, 0}, {0, 0, 0, 1})})},
                                  {"unparsed", notes}};
    const auto unused_ok = xsim::compiledSceneDocument(unused);
    if (unused_ok.obstacles.size() != 1 || unused_ok.obstacles[0].type != xgc2_math::scene::Obstacle::kBox)
        throw std::runtime_error("unused unparsed notes blocked the static scene");
    auto ground_notes = base;
    ground_notes["obstacles"] = json::array();
    ground_notes["models"]["terrain"]["unparsed"] = notes;
    ground_notes["ground"] = {{"mode", "model"}, {"model", "terrain"}, {"pose", pose({0, 0, 4})}};
    require_throw(ground_notes, "ground unparsed notes accepted", "does not support unparsed notes");
    auto ground_empty = ground_notes;
    ground_empty["models"]["terrain"]["unparsed"] = json::array();
    const auto ground_empty_ok = xsim::compiledSceneDocument(ground_empty);
    if (ground_empty_ok.planar_ground || ground_empty_ok.obstacles.size() != 1)
        throw std::runtime_error("empty ground unparsed rejected");

    std::cout << "PASS compiled.v1 reader, shared mesh, hole, ground\n";
}
