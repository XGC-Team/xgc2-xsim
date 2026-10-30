// SPDX-License-Identifier: GPL-2.0-only
#include <xgc2_sensing/gpu.h>
#include <xgc2_math/scene/sensor_metadata.hpp>
#include <opengl_sim.hpp>
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <memory>
#include <omp.h>
#include <stdexcept>
#include <tuple>

namespace {
#include "point_filter.hpp"
xgc2_math::scene::SensorMetadata metadata(const xgc2_sensing_gpu_projection *p) {
  if (!p) throw std::invalid_argument("GPU projection required");
  xgc2_math::scene::SensorMetadata m;
  m.backend = "gpu";
  m.observation_model = "lidar_scan";
  m.range_m = p->range_m;
  m.min_range_m = p->min_range_m;
  m.h_fov_deg = p->h_fov_deg;
  m.v_fov_deg = p->v_fov_deg;
  m.h_res = p->h_res;
  m.v_res = p->v_res;
  m.point_cover_spacing_m = p->point_cover_spacing_m;
  m.publish_rate_hz = p->publish_rate_hz;
  m.frame_id = "world";
  m.stamp_policy = "pose";
  m.pose_type = "geometry_msgs/PoseStamped";
  if (!p->penetrating) xgc2_math::scene::validateGpuSensorMetadata(m);
  else if (!std::isfinite(m.range_m) || m.range_m <= m.min_range_m || m.min_range_m < 0
           || m.h_fov_deg <= 0 || m.h_fov_deg > 360 || m.v_fov_deg <= 0 || m.v_fov_deg > 180)
    throw std::invalid_argument("Invalid penetrating sensor bounds");
  return m;
}
void error_text(char *error, size_t size, const char *message) noexcept {
  if (error && size) std::snprintf(error, size, "%s", message);
}

class Renderer {
public:
  std::unique_ptr<opengl_pointcloud_render> renderer{new opengl_pointcloud_render};
  pcl::PointCloud<pcl::PointXYZI>::Ptr rendered{new pcl::PointCloud<pcl::PointXYZI>};
  unsigned framebuffer = 0, color_buffer = 0, depth_buffer = 0, depth_format = 0;
  int buffer_width = 0, buffer_height = 0;
  pcl::KdTreeFLANN<PointType> index{false};
  std::vector<int> candidates;
  std::vector<float> distances;
  std::unique_ptr<PointFilter> filter;

  xgc2_sensing_gpu_projection last_projection{};
  xgc2_math::scene::SensorMetadata last_metadata;
  bool configured = false;
  const xgc2_math::scene::SensorMetadata& configuration(const xgc2_sensing_gpu_projection* p) {
    if (!p) throw std::invalid_argument("GPU projection required");
    const auto key=[](const auto& v){return std::tie(v.range_m,v.min_range_m,v.h_fov_deg,v.v_fov_deg,
      v.h_res,v.v_res,v.point_cover_spacing_m,v.publish_rate_hz,v.penetrating,v.heading_crop,
      v.heading_cos_min,v.vertical_slab_tan);};
    if (!configured || key(*p) != key(last_projection)) {
      last_metadata=metadata(p);last_projection=*p;configured=true;
    }
    return last_metadata;
  }
  ~Renderer() {
    // A failed GL initialization must unwind without touching GLFW: the owner
    // context may already be terminated and there is nothing to release.
    if (framebuffer || color_buffer || depth_buffer) {
      renderer->activate();
      // Release projection buffers before the renderer destroys its owner context.
      if (framebuffer) glDeleteFramebuffers(1, &framebuffer);
      if (color_buffer) glDeleteRenderbuffers(1, &color_buffer);
      if (depth_buffer) glDeleteRenderbuffers(1, &depth_buffer);
    }
  }
  void projection(const xgc2_math::scene::SensorMetadata &m) {
    renderer->setParameters(m.h_res, m.v_res, 250.0f, 250.0f,
        static_cast<float>(m.point_cover_spacing_m),
        static_cast<float>(m.h_fov_deg), static_cast<float>(m.v_fov_deg),
        static_cast<float>(m.min_range_m), static_cast<float>(m.range_m),
        static_cast<int>(m.publish_rate_hz), 0, 0, 0);
  }
  void load(const float *xyz, size_t count, const xgc2_math::scene::SensorMetadata &m) {
    projection(m);
    pcl::PointCloud<PointType> cloud;
    cloud.resize(count);
    for (size_t i = 0; i < count; ++i) {
      auto &point = cloud[i];
      point.x = xyz[3 * i]; point.y = xyz[3 * i + 1]; point.z = xyz[3 * i + 2];
      point.intensity = 0;
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
        throw std::invalid_argument("GPU static map must be finite");
    }
    if (!renderer->read_pointcloud_frommemory(cloud))
      throw std::runtime_error("original GPU context/static upload initialization failed; no CPU fallback");
    index.setInputCloud(cloud.makeShared());
    candidates.reserve(count);distances.reserve(count);rendered->points.reserve(count);
    const char *vendor = reinterpret_cast<const char *>(glGetString(GL_VENDOR));
    const char *name = reinterpret_cast<const char *>(glGetString(GL_RENDERER));
    const char *version = reinterpret_cast<const char *>(glGetString(GL_VERSION));
    if (!vendor || !name || !version) throw std::runtime_error("actual GL identity missing");
    std::string check(name);
    std::transform(check.begin(), check.end(), check.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    for (const auto *software : {"llvmpipe", "softpipe", "swrast", "software"})
      if (check.find(software) != std::string::npos)
        throw std::runtime_error("software GL is not admitted as GPU; no CPU fallback");
    std::cout << "GL_VENDOR=" << vendor << "\nGL_RENDERER=" << name << "\nGL_VERSION=" << version << std::endl;
  }
  const pcl::PointCloud<pcl::PointXYZI> &scan(const Eigen::Vector3d &p,
                                          const Eigen::Quaterniond &q, double stamp) {
    renderer->render_pointcloud(rendered, p.cast<float>(), q.cast<float>(), stamp);
    return *rendered;
  }
  const pcl::PointCloud<pcl::PointXYZI> &scan(const Eigen::Vector3d &p,
      const Eigen::Quaterniond &q, double stamp, const xgc2_math::scene::SensorMetadata &m) {
    projection(m);
    if (!glfwGetCurrentContext()) throw std::runtime_error("GPU owner context missing");
    // Preserve native depth precision and rendering math while avoiding the
    // asynchronous X11 drawable resize when sensors switch projections.
    if (!framebuffer) {
      GLint bits = 0, component = 0;
      glGetFramebufferAttachmentParameteriv(
          GL_FRAMEBUFFER, GL_DEPTH, GL_FRAMEBUFFER_ATTACHMENT_DEPTH_SIZE, &bits);
      glGetFramebufferAttachmentParameteriv(
          GL_FRAMEBUFFER, GL_DEPTH, GL_FRAMEBUFFER_ATTACHMENT_COMPONENT_TYPE, &component);
      depth_format = bits == 16 ? GL_DEPTH_COMPONENT16 : bits == 24 ? GL_DEPTH_COMPONENT24
          : component == GL_FLOAT ? GL_DEPTH_COMPONENT32F : GL_DEPTH_COMPONENT32;
      if (bits != 16 && bits != 24 && bits != 32)
        throw std::runtime_error("unsupported native depth precision: " + std::to_string(bits));
      glGenFramebuffers(1, &framebuffer);
      glGenRenderbuffers(1, &color_buffer);
      glGenRenderbuffers(1, &depth_buffer);
    }
    renderer->setOffscreen();
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
    if (buffer_width != m.h_res || buffer_height != m.v_res) {
      const auto require_size = [&] {
        GLint width = 0, height = 0;
        glGetRenderbufferParameteriv(GL_RENDERBUFFER, GL_RENDERBUFFER_WIDTH, &width);
        glGetRenderbufferParameteriv(GL_RENDERBUFFER, GL_RENDERBUFFER_HEIGHT, &height);
        if (width != m.h_res || height != m.v_res)
          throw std::runtime_error("GPU projection buffer allocation unavailable for requested resolution");
      };
      glBindRenderbuffer(GL_RENDERBUFFER, color_buffer);
      glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, m.h_res, m.v_res);
      require_size();
      glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, color_buffer);
      glBindRenderbuffer(GL_RENDERBUFFER, depth_buffer);
      glRenderbufferStorage(GL_RENDERBUFFER, depth_format, m.h_res, m.v_res);
      require_size();
      glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_buffer);
      if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        throw std::runtime_error("GPU projection framebuffer allocation failed");
      buffer_width = m.h_res; buffer_height = m.v_res;
    }
    glDrawBuffer(GL_COLOR_ATTACHMENT0);
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glViewport(0, 0, m.h_res, m.v_res);
    return scan(p, q, stamp);
  }
};

void *create(const float *xyz, size_t count, const xgc2_sensing_gpu_projection *first,
             char *error, size_t error_size) noexcept {
  error_text(error, error_size, "");
  try {
    omp_set_dynamic(0);
    omp_set_num_threads(1);
    const auto m = metadata(first);
    if (!xyz || !count || count > SIZE_MAX / (3 * sizeof(float)))
      throw std::invalid_argument("GPU requires one nonempty static cloud");
    auto renderer = std::make_unique<Renderer>();
    renderer->load(xyz, count, m);
    return renderer.release();
  } catch (const std::exception &e) { error_text(error, error_size, e.what()); }
    catch (...) { error_text(error, error_size, "GPU creation failed"); }
  return nullptr;
}
int scan(void *renderer, const double position[3], const double orientation[4], double stamp,
         const xgc2_sensing_gpu_projection *projection, xgc2_sensing_gpu_grow grow, void *context,
         char *error, size_t error_size) noexcept {
  error_text(error, error_size, "");
  try {
    if (!renderer || !position || !orientation || !grow || !std::isfinite(stamp))
      throw std::invalid_argument("GPU scan requires renderer, pose, stamp and storage");
    const Eigen::Vector3d p(position[0], position[1], position[2]);
    const Eigen::Quaterniond q(orientation[0], orientation[1], orientation[2], orientation[3]);
    if (!p.allFinite() || !q.coeffs().allFinite() || q.norm() < 1e-12)
      throw std::invalid_argument("GPU scan pose invalid");
    auto* owner = static_cast<Renderer*>(renderer);
    const auto& m = owner->configuration(projection);
    owner->renderer->activate();
    PointType center;center.x=p.x();center.y=p.y();center.z=p.z();
    owner->index.radiusSearch(center,m.range_m,owner->candidates,owner->distances);
    // Opaque rasterization preserves source order for deterministic depth ties.
    // Penetrating point sets do not need the two O(N log N) distance/index sorts.
    if (!projection->penetrating) std::sort(owner->candidates.begin(),owner->candidates.end());
    if(projection->penetrating) {
      if(!owner->filter) { owner->filter.reset(new PointFilter); owner->filter->load(owner->renderer->pointBuffer(),owner->index.getInputCloud()->size()); }
      owner->filter->scan(p,q,*projection,owner->candidates,*owner->rendered);
    } else { owner->renderer->selectPoints(owner->candidates); owner->scan(p, q, stamp, m); }
    const auto &points = *owner->rendered;
    if (points.size() > SIZE_MAX / 16) throw std::length_error("GPU scan payload too large");
    auto *bytes = static_cast<unsigned char *>(grow(context, points.size() * 16));
    if (!bytes && !points.empty()) throw std::runtime_error("GPU scan storage unavailable");
    for (size_t i = 0; i < points.size(); ++i) {
      const auto &p = points[i];
      const float xyzi[4] = {p.x, p.y, p.z, p.intensity};
      std::memcpy(bytes + i * 16, xyzi, 16);
    }
    return 0;
  } catch (const std::exception &e) { error_text(error, error_size, e.what()); }
    catch (...) { error_text(error, error_size, "GPU scan failed"); }
  return -1;
}
void destroy(void *renderer) noexcept { delete static_cast<Renderer *>(renderer); }
const xgc2_sensing_gpu_api api = {XGC2_SENSING_GPU_ABI, sizeof(xgc2_sensing_gpu_api), create, scan, destroy};
}

extern "C" __attribute__((visibility("default")))
const xgc2_sensing_gpu_api *xgc2_sensing_gpu_entry(uint32_t abi) {
  return abi == XGC2_SENSING_GPU_ABI ? &api : nullptr;
}
