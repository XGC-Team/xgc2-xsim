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
        if(!rejected)throw std::runtime_error("invalid ring accepted");
    }
    std::cout<<"PASS XSim exact production document mapping, named asymmetric radii, composed poses, invalid types\n";
}
