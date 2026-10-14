#include <xgc2_sensing/gpu.h>
#include <xgc2_math/scene.hpp>
#include <cassert>
#include <chrono>
#include <cstring>
#include <iostream>
#include <vector>
int main(){
  using namespace xgc2_math::scene;
  const std::vector<Obstacle> bodies={Obstacle::box({5,0,0},{1,8,8}),Obstacle::box({9,0,0},{1,2,2})};
  auto geometry=std::make_shared<LidarScene>(bodies,.1,false);
  SensorConfig c;c.range=20;c.min_range=.2;c.h_res=120;c.v_res=120;c.h_fov_deg=60;c.v_fov_deg=60;
  WorldLidar cpu(c);cpu.setScene(geometry);
  WorldLidar source(SensorConfig{});source.setScene(geometry);
  std::vector<float> map;for(const auto&p:source.globalMap(.1))for(int k=0;k<3;++k)map.push_back(float(p[k]));
  const auto*api=xgc2_sensing_gpu_entry(XGC2_SENSING_GPU_ABI);
  xgc2_sensing_gpu_projection config{20,.2,60,60,120,120,.1,10,0,0,-1,0};char error[512]={};
  void*renderer=api->create(map.data(),map.size()/3,&config,error,sizeof(error));if(!renderer){std::cerr<<error;return 1;}
  const double position[3]={0,0,0},orientation[4]={1,0,0,0};
  std::vector<unsigned char> bytes;bytes.reserve(map.size()/3*16);
  auto grow=[](void*ctx,size_t size)->void*{auto&v=*static_cast<std::vector<unsigned char>*>(ctx);v.resize(size);return v.data();};
  double worst=0;size_t counts[2]={};
  for (int grid=0;grid<2;++grid) {
  if (grid) {
    config.h_fov_deg=c.h_fov_deg=360;config.v_fov_deg=c.v_fov_deg=180;
    config.h_res=c.h_res=360;config.v_res=c.v_res=32;
  }
  for(int mode=0;mode<2;++mode){config.penetrating=mode;size_t behind=0;
    for(int repeat=0;repeat<20;++repeat){auto begin=std::chrono::steady_clock::now();
      if(api->scan(renderer,position,orientation,1+repeat*.1,&config,grow,&bytes,error,sizeof(error))){std::cerr<<error;return 2;}
      worst=std::max(worst,std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count());
    }
    double highest=-1,lowest=1;
    counts[mode]=bytes.size()/16;for(size_t i=0;i<bytes.size();i+=16){
      float p[4];std::memcpy(p,bytes.data()+i,16);if(p[0]>7)++behind;
      const double elevation=std::atan2(p[2],std::hypot(p[0],p[1]));
      highest=std::max(highest,elevation);lowest=std::min(lowest,elevation);
    }
    assert(counts[mode]>0);if(mode==0)assert(behind==0);else assert(behind>0);
    // A 360x32 grid still covers the authored vertical FoV. Reusing the
    // horizontal one-degree step would incorrectly compress it to 32 degrees.
    if(grid && mode==0){assert(highest>.5);assert(lowest<-.5);}
    c.mode=mode?SensorConfig::kPenetrating:SensorConfig::kRaycast;WorldLidar native(c);native.setScene(geometry);
    auto scan=native.scan(Eigen::Vector3d::Zero(),Eigen::Quaterniond::Identity());size_t cpu_behind=0;
    for(const auto&p:scan)if(p.x()>7)++cpu_behind;if(mode==0)assert(cpu_behind==0);else assert(cpu_behind>0);
  }
  }
  api->destroy(renderer);std::cout<<"PASS CPU/GPU occlusion and penetration; GPU counts "<<counts[0]<<"/"<<counts[1]<<", worst scan "<<worst<<" ms\n";
}
