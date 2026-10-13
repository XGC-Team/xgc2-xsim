#include "systems/sensors.hpp"
#include <cassert>
#include <cstring>
#include <iostream>
#include <thread>
using namespace xsim;
int main(int argc, char** argv) {
  for (const auto backend : {"cpu", "gpu", "auto"}) for (const bool through : {false, true}) {
    if(argc==2 && std::string(backend)!=argv[1]) continue;
    Sensors sensors(Json{{"obstacles", Json::array({
      {{"type","box"},{"position",{5,0,0}},{"size",{1,8,8}}},
      {{"type","box"},{"position",{9,0,0}},{"size",{1,2,2}}}})}});
    std::vector<std::shared_ptr<Entity>> robots;
    for (int id=1; id<=20; ++id) {
      auto e=std::make_shared<Entity>(parse_entity({{"name","uav"+std::to_string(id)},{"kind","scout"}}));
      e->id=id;e->alive=e->enabled=true;e->generation=1;
      e->sensor=sensors.prepare(e,{{"backend",backend},{"mode",through?"penetrating":"raycast"},
        {"range",20},{"min_range",.2},{"h_res",120},{"v_res",120},
        {"h_fov_deg",60},{"v_fov_deg",60},{"surface_spacing",id==20?.2:.1}});
      robots.push_back(e);
    }
    int64_t stamp=1000000000LL;
    for (int round=0;round<3;++round) {
      std::vector<bool> received(robots.size(),false);
      size_t remaining=robots.size();
      const auto end=Clock::now()+std::chrono::seconds(10);
      while (remaining) {
        for(size_t i=0;i<robots.size();++i) if(!received[i]) {
          const auto&e=robots[i];State p;p.entity=e;p.key={e->id,e->generation};p.enabled=true;p.stamp=stamp;
          p.position={round==2?.05:0,0,0};sensors.submit(p);
          Sample sample;std::vector<uint8_t> data;
          if(!Sensors::take(e->sensor,sample,data)) continue;
          if(sample.position.x() != (round==2?.05:0)) continue;
          const size_t stride=e->sensor->gpu?16:12;assert(!data.empty() && data.size()%stride==0);
          size_t behind=0;for(size_t at=0;at<data.size();at+=stride){float x;std::memcpy(&x,data.data()+at,4);if(x>7)++behind;}
          assert((behind>0)==through);received[i]=true;--remaining;
        }
        if(Clock::now()>=end){std::cerr<<backend<<" round="<<round<<" "<<sensors.status().dump()<<std::endl;return 2;}
        stamp+=20000000LL;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      stamp+=1000000000LL;
    }
    for(const auto&e:robots) assert(e->sensor->cache_hits>=1 && e->sensor->computed_scans==2);
    std::cout<<"PASS XSim "<<backend<<" effective="<<(robots[0]->sensor->gpu?"gpu":"cpu")<<" through="<<through<<" 20 robots, two shared geometry policies, pose cache\n";
  }
}
