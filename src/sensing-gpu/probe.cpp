#include <xgc2_sensing/loader.hpp>
int main(){try{
  auto& api=xgc2_sensing::gpuLibrary();
  const float point[3]={1,0,0};
  xgc2_sensing_gpu_projection p{2,.2,360,180,4,2,.1,1,1,0,-1,0};
  char error[512]={};void* ctx=api.create(point,1,&p,error,sizeof(error));
  if(!ctx)return 1;api.destroy(ctx);return 0;
}catch(...){return 1;}}
