#pragma once
// All methods run on the existing renderer's GL owner thread. XYZ storage is
// uploaded once; the feedback buffer and CPU readback capacity survive scans.
class PointFilter {
  GLuint program_ = 0, vao_ = 0, input_ = 0, output_ = 0, indices_ = 0;
  GLsizei count_ = 0;
  std::vector<float> readback_;
public:
  ~PointFilter() {
    if (program_) glDeleteProgram(program_);
    if (vao_) glDeleteVertexArrays(1,&vao_);
    if (indices_) glDeleteBuffers(1,&indices_);
    if (output_) glDeleteBuffers(1,&output_);
  }
  void load(GLuint shared_input, std::size_t point_count) {
    static const char* code = R"GLSL(#version 330 core
layout(location=0) in vec3 worldPoint;
uniform mat3 worldToSensor;
uniform vec3 origin;
uniform vec4 bounds; // near, far, horizontal half-FOV, vertical half-FOV
uniform vec3 crop; // enabled, forward cosine threshold, fixed world Z/range
out vec4 selected;
void main() {
  vec3 delta=worldPoint-origin;
  vec3 local=worldToSensor*delta;
  float range=length(delta);
  bool keep=range>=bounds.x && range<=bounds.y;
  keep=keep && abs(atan(local.y,local.x))<=bounds.z;
  keep=keep && abs(atan(local.z,length(local.xy)))<=bounds.w;
  if(crop.x>0.0 && range>0.0) {
    keep=keep && local.x/range>=crop.y;
    if(crop.z>0.0) keep=keep && abs(delta.z)<=crop.z*range;
  }
  selected=vec4(worldPoint,keep?1.0:0.0);
  gl_Position=vec4(0.0);
})GLSL";
    GLuint shader=glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(shader,1,&code,nullptr);glCompileShader(shader);
    GLint ok=0;glGetShaderiv(shader,GL_COMPILE_STATUS,&ok);
    if(!ok){char log[1024];glGetShaderInfoLog(shader,sizeof(log),nullptr,log);glDeleteShader(shader);throw std::runtime_error(log);}
    program_=glCreateProgram();glAttachShader(program_,shader);
    const char* varying="selected";glTransformFeedbackVaryings(program_,1,&varying,GL_INTERLEAVED_ATTRIBS);
    glLinkProgram(program_);glDeleteShader(shader);glGetProgramiv(program_,GL_LINK_STATUS,&ok);
    if(!ok) throw std::runtime_error("GPU point filter linking failed");
    count_=static_cast<GLsizei>(point_count);
    glGenVertexArrays(1,&vao_);glBindVertexArray(vao_);
    input_=shared_input;glBindBuffer(GL_ARRAY_BUFFER,input_);
    glEnableVertexAttribArray(0);glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,6*sizeof(float),nullptr);
    glGenBuffers(1,&output_);glBindBuffer(GL_TRANSFORM_FEEDBACK_BUFFER,output_);
    glBufferData(GL_TRANSFORM_FEEDBACK_BUFFER,count_*4*sizeof(float),nullptr,GL_STREAM_READ);
    readback_.reserve(static_cast<size_t>(count_)*4);
    glGenBuffers(1,&indices_);glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,indices_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,count_*sizeof(unsigned int),nullptr,GL_STREAM_DRAW);
    glBindVertexArray(0);
  }
  void scan(const Eigen::Vector3d& origin,const Eigen::Quaterniond& rotation,
            const xgc2_sensing_gpu_projection& p,const std::vector<int>& candidates,pcl::PointCloud<pcl::PointXYZI>& result) {
    glUseProgram(program_);glBindVertexArray(vao_);
    const Eigen::Matrix3f transform=rotation.normalized().conjugate().toRotationMatrix().cast<float>();
    glUniformMatrix3fv(glGetUniformLocation(program_,"worldToSensor"),1,GL_FALSE,transform.data());
    glUniform3f(glGetUniformLocation(program_,"origin"),origin.x(),origin.y(),origin.z());
    glUniform4f(glGetUniformLocation(program_,"bounds"),p.min_range_m,p.range_m,
                p.h_fov_deg*M_PI/360.0,p.v_fov_deg*M_PI/360.0);
    glUniform3f(glGetUniformLocation(program_,"crop"),p.heading_crop,p.heading_cos_min,p.vertical_slab_tan);
    glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER,0,output_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,indices_);
    glBufferSubData(GL_ELEMENT_ARRAY_BUFFER,0,candidates.size()*sizeof(unsigned int),candidates.data());
    glEnable(GL_RASTERIZER_DISCARD);glBeginTransformFeedback(GL_POINTS);
    glDrawElements(GL_POINTS,candidates.size(),GL_UNSIGNED_INT,nullptr);glEndTransformFeedback();glDisable(GL_RASTERIZER_DISCARD);
    readback_.resize(candidates.size()*4);
    glBindBuffer(GL_TRANSFORM_FEEDBACK_BUFFER,output_);
    glGetBufferSubData(GL_TRANSFORM_FEEDBACK_BUFFER,0,readback_.size()*sizeof(float),readback_.data());
    glBindVertexArray(0);glUseProgram(0);
    result.clear();
    for(size_t i=0;i<readback_.size();i+=4) if(readback_[i+3]!=0.0f) {
      pcl::PointXYZI point;point.x=readback_[i];point.y=readback_[i+1];point.z=readback_[i+2];point.intensity=0;
      result.push_back(point);
    }
  }
};
