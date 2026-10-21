#include <xgc2_math/scene/sensor_model.hpp>

#include <Eigen/Core>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

// Float32 emulation of the point_filter.hpp shader compared with the real CPU
// xgc2_math::scene::HeadingCrop::contains. This is not a GPU execution.

struct Vec3 { float x, y, z; };

static float length3(Vec3 v) {
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

struct Crop {
  float near_m, sensor_range, h_half, v_half;
  float enabled, cos_min, slab_tan;
};

// Identity sensor frame. bounds.y is the fixed sensor range used by the slab.
static bool shader_keep(Vec3 world, Crop c) {
  Vec3 delta{world.x, world.y, world.z};
  Vec3 local = delta;
  float range = length3(delta);
  bool keep = range >= c.near_m && range <= c.sensor_range;
  keep = keep && std::abs(std::atan2(local.y, local.x)) <= c.h_half;
  keep = keep && std::abs(std::atan2(local.z, length3({local.x, local.y, 0.f}))) <= c.v_half;
  if (c.enabled > 0.f && range > 0.f) {
    keep = keep && local.x / range >= c.cos_min;
    if (c.slab_tan > 0.f) keep = keep && std::abs(delta.z) <= c.slab_tan * c.sensor_range;
  }
  return keep;
}

// Removed per-point slab, kept only as the F2 counterexample.
static bool per_point_slab_keep(Vec3 world, Crop c) {
  Vec3 delta{world.x, world.y, world.z};
  Vec3 local = delta;
  float range = length3(delta);
  bool keep = range >= c.near_m && range <= c.sensor_range;
  keep = keep && std::abs(std::atan2(local.y, local.x)) <= c.h_half;
  keep = keep && std::abs(std::atan2(local.z, length3({local.x, local.y, 0.f}))) <= c.v_half;
  if (c.enabled > 0.f && range > 0.f) {
    keep = keep && local.x / range >= c.cos_min;
    if (c.slab_tan > 0.f) keep = keep && std::abs(delta.z) <= c.slab_tan * range;
  }
  return keep;
}

static bool cpu_heading_crop(double x, double y, double z, bool enabled, double cos_min, double slab_tan,
                             double sensor_range) {
  const xgc2_math::scene::HeadingCrop crop(enabled, cos_min, slab_tan, sensor_range);
  return crop.contains(Eigen::Vector3d(x, y, z), Eigen::Vector3d(1.0, 0.0, 0.0));
}

static void expect(bool condition, const char *name) {
  if (!condition) {
    std::cerr << "crop parity failed: " << name << '\n';
    std::exit(1);
  }
}

static std::string shader_source() {
  std::ifstream in(POINT_FILTER_HPP);
  if (!in) {
    std::cerr << "cannot read " << POINT_FILTER_HPP << '\n';
    std::exit(1);
  }
  std::ostringstream buffer;
  buffer << in.rdbuf();
  return buffer.str();
}

int main() {
  std::cout << "real HeadingCrop::contains vs float32 shader emulation; GPU not executed\n";
  const std::string source = shader_source();
  expect(source.find("abs(delta.z)<=crop.z*bounds.y") != std::string::npos, "fixed slab uses bounds.y");
  expect(source.find("crop.z*range") == std::string::npos, "per-point slab term is gone");
  expect(source.find("range>=bounds.x && range<=bounds.y") != std::string::npos, "range gate unchanged");
  expect(source.find("local.x/range>=crop.y") != std::string::npos, "forward gate unchanged");

  const float pi = std::acos(-1.f);
  Crop wide{0.1f, 5.f, pi, pi / 2.f, 1.f, 0.f, 0.5f};
  const Vec3 near{1.f, 0.f, 2.f};
  expect(shader_keep(near, wide), "near point kept by fixed slab");
  expect(!per_point_slab_keep(near, wide), "near point was dropped by per-point length");
  expect(shader_keep(near, wide) == cpu_heading_crop(1, 0, 2, true, 0, 0.5, 5), "near matches HeadingCrop");

  const Vec3 far{4.f, 0.f, 0.5f};
  expect(shader_keep(far, wide), "far point inside the fixed slab");
  expect(per_point_slab_keep(far, wide), "far point was not a per-point reject");
  expect(shader_keep(far, wide) == cpu_heading_crop(4, 0, 0.5, true, 0, 0.5, 5), "far matches HeadingCrop");

  Crop edge{0.1f, 4.f, pi, pi / 2.f, 1.f, 0.f, 0.5f};
  const Vec3 on_slab{0.f, 0.f, 2.f};
  expect(shader_keep(on_slab, edge), "boundary |z|==tan*sensor range is kept");
  expect(!per_point_slab_keep(on_slab, edge), "boundary was outside the per-point slab");
  expect(shader_keep(on_slab, edge) == cpu_heading_crop(0, 0, 2, true, 0, 0.5, 4), "boundary matches HeadingCrop");
  const Vec3 past{0.f, 0.f, 2.25f};
  expect(!shader_keep(past, edge), "just outside the fixed slab is dropped");
  expect(shader_keep(past, edge) == cpu_heading_crop(0, 0, 2.25, true, 0, 0.5, 4), "outside matches HeadingCrop");

  edge.enabled = 0.f;
  expect(cpu_heading_crop(0, 0, 2.25, false, 0, 0.5, 4), "disabled HeadingCrop keeps the offset");
  expect(shader_keep(past, edge), "crop off does not apply the slab");
  expect(shader_keep(Vec3{6.f, 0.f, 0.f}, edge) == false, "range gate still drops a far point");
  return 0;
}
