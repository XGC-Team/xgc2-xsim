#include "sensors/scene_document.hpp"
#include <iostream>
#include <stdexcept>
int main() {
    using nlohmann::json;
    const Eigen::Quaterniond parent(Eigen::AngleAxisd(.6, Eigen::Vector3d::UnitY()));
    const Eigen::Quaterniond child(Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitZ()));
    const auto q=[](const Eigen::Quaterniond& r){return json::array({r.x(),r.y(),r.z(),r.w()});};
    json document={{"obstacles", json::array({{{"id","ring"}, {"pose",{{"position",{2,3,4}},{"orientation",q(parent)}}},
        {"parts",json::array({{{"pose",{{"position",{.1,.2,.3}},{"orientation",q(child)}}},
            {"geometry",{{"type","ring"},{"radii",{1.,1.2}},{"tubeRadius",.05}}}}})}}})}};
    const auto shapes=xsim::sceneDocumentObstacles(document);
    if(shapes.size()!=1 || shapes[0].type!=xgc2_math::scene::Obstacle::kRing ||
       shapes[0].radii!=Eigen::Vector2d(1,1.2) || shapes[0].tubeRadius!=.05 ||
       (shapes[0].position-(Eigen::Vector3d(2,3,4)+parent*Eigen::Vector3d(.1,.2,.3))).norm()>1e-12 ||
       shapes[0].orientation.angularDistance(parent*child)>1e-12)throw std::runtime_error("typed document ring/poses");
    for(const auto invalid : {json::array({true,1.2}),json::array({1.}),json::array({1.,1.2,1.3}),json::array({0.,1.2})}) {
        auto bad=document;bad["obstacles"][0]["parts"][0]["geometry"]["radii"]=invalid;
        bool rejected=false;try{xsim::sceneDocumentObstacles(bad);}catch(const std::exception&){rejected=true;}
        if(!rejected)throw std::runtime_error("invalid ring accepted: " + invalid.dump());
    }
    auto typed = document;
    auto& shape = typed["obstacles"][0]["parts"][0]["geometry"];
    shape = {{"type", "ellipsoid"}, {"semiaxes", {1,2,3}}};
    const auto ellipsoid = xsim::sceneDocumentObstacles(typed);
    if (ellipsoid.size()!=1 || ellipsoid[0].type!=xgc2_math::scene::Obstacle::kEllipsoid ||
        ellipsoid[0].size!=Eigen::Vector3d(1,2,3)) throw std::runtime_error("ellipsoid retains analytic semiaxes");
    shape = {{"type", "halfspaces"}, {"halfspaces", {{1,0,0,-1},{-1,0,0,-1},{0,1,0,-1},{0,-1,0,-1},{0,0,1,-1},{0,0,-1,-1}}}};
    const auto bounded = xsim::sceneDocumentObstacles(typed);
    if(bounded.size()!=1 || bounded[0].type!=xgc2_math::scene::Obstacle::kConvex || bounded[0].vertices.size()!=8)
        throw std::runtime_error("bounded numeric H input");
    shape = {{"type", "convex"}, {"vertices", {{0,0,0},{1,0,0},{0,1,0},{0,0,1}}}};
    const auto temporary = xsim::sceneDocumentObstacles(typed);
    if(temporary.size()!=1 || temporary[0].vertices.size()!=4) throw std::runtime_error("temporary unregistered hull");
    std::cout<<"PASS XSim exact production document mapping, named asymmetric radii, composed poses, invalid types\n";
}
