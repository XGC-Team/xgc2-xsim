#pragma once

#include "rigid_body.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace xgc_lightweight {

struct GroundContactResult {
  bool valid{true};
  bool constrained{false};
  double normal_impulse{0.0}; // N s, sum of unilateral inelastic impulses
  Eigen::Vector3d base_acceleration_world{Eigen::Vector3d::Zero()};
};

// Declared FS150 support. iris.sdf base_link_inertia_collision is one box,
// size 0.47 0.47 0.066, pose identity in base_link (measurement origin, FLU).
// Its bottom face is the unique lowest collision surface (z = -0.033).
// Rotor cylinders bottom at z = +0.0205 and are not contacts. Box side faces
// are not contacts. This is that one face, not a collision engine.
//
// world_ground_z is the Experiment/world horizontal plane, not a measurement
// height. Corner world z is tested against that plane. A level body therefore
// rests with its base origin 0.033 m above the plane (half the 0.066 m box).
// The old settled pose base z = ground_z put this face 3.3 cm underground.
// A spawn below the plane is lifted by a vertical translation. Orientation
// is not snapped and velocity is not cleared.
//
// Friction is the SDFormat 1.6 default of the empty <friction><ode/> element:
// mu = mu2 = 1, slip1 = slip2 = 0, fdir1 unset. Isotropic Coulomb, no viscous
// slip and no contact kp/kd. SDF min_depth 0.001 is a contact-correction
// allowance, not a physical slab. Using it as a 1 mm solver tolerance — a
// corner within 1 mm of the plane joins the same velocity constraint — is an
// approximation of that allowance. Velocity is zeroed without rotating the
// body, so a face caught inside the band can rest with a tilt of about
// min_depth / 0.47 m. A corner inside that band is not a contact when it is
// on or above the plane, its normal speed is separating, and the normal
// impulse is zero. Armed Land uses that flag and does not disarm on the way
// through the band. max_vel 0 keeps restitution at 0. Friction impulses use
// the fixed world XY axes of this horizontal plane (fdir1 is unset). A
// slip-following basis diverges on impact. The 32 passes are a solver bound,
// not a damping gain. Each pass solves one corner's normal impulse together
// with that friction: the normal row of the point-mass matrix includes the
// tangent impulse, and the tangent is then projected onto μ times the normal.
// If that projection still leaves the corner moving into the plane, only a
// further nonnegative normal impulse is applied. Friction inside the cone is
// not scaled down again, because that coupling would reopen the inward speed.
struct Fs150SupportFace {
  static constexpr double kMu = 1.0;
  static constexpr double kMinDepth = 0.001; // solver tolerance, not a slab
  static constexpr double kBoxX = 0.47;
  static constexpr double kBoxY = 0.47;
  static constexpr double kBoxZ = 0.066;
  static constexpr double kFaceZ = -0.5 * kBoxZ; // -0.033 m, body frame
  static constexpr double kLevelBaseHeight = -kFaceZ; // 0.033 m above ground_z
  static constexpr int kCorners = 4;
  static constexpr int kPasses = 32;

  static Eigen::Vector3d corner(int index) {
    const double sx = (index & 1) ? 0.5 : -0.5;
    const double sy = (index & 2) ? 0.5 : -0.5;
    return {sx * kBoxX, sy * kBoxY, kFaceZ};
  }
};

// Lowest support corner relative to the world plane. Zero means the face is
// on ground_z. Landed uses this clearance, not base height.
inline double fs150SupportClearance(const RigidBodyModel &body, double world_ground_z) {
  const auto state = body.state();
  const Eigen::Vector3d &com = body.parameters().body_origin_to_com;
  double lowest = std::numeric_limits<double>::infinity();
  for (int i = 0; i < Fs150SupportFace::kCorners; ++i) {
    const Eigen::Vector3d lever = Fs150SupportFace::corner(i) - com;
    lowest = std::min(lowest, (state.position + state.orientation * lever).z());
  }
  return lowest - world_ground_z;
}

namespace support_detail {
inline double inverse_mass(const RigidBodyModel &body, const Eigen::Quaterniond &orientation,
                           const Eigen::Vector3d &r_body, const Eigen::Vector3d &direction) {
  const Eigen::Vector3d normal_body = orientation.conjugate() * direction;
  const Eigen::Vector3d lever = r_body.cross(normal_body);
  return 1.0 / body.parameters().mass + lever.dot(body.inverse_inertia() * lever);
}

inline void apply_impulse(RigidBodyState &state, const RigidBodyModel &body,
                          const Eigen::Vector3d &r_body, const Eigen::Vector3d &direction,
                          double impulse) {
  if (impulse == 0.0)
    return;
  const Eigen::Vector3d normal_body = state.orientation.conjugate() * direction;
  const Eigen::Vector3d lever = r_body.cross(normal_body);
  state.velocity += direction * (impulse / body.parameters().mass);
  state.angular_velocity += body.inverse_inertia() * lever * impulse;
}

// World impulse at the contact to the change in that point's world velocity.
// Orientation is constant while impulses are resolved.
inline Eigen::Matrix3d point_delassus(const RigidBodyModel &body,
                                      const Eigen::Quaterniond &orientation,
                                      const Eigen::Vector3d &r_body) {
  Eigen::Matrix3d metric = Eigen::Matrix3d::Zero();
  const double inv_mass = 1.0 / body.parameters().mass;
  for (int axis = 0; axis < 3; ++axis) {
    const Eigen::Vector3d direction = Eigen::Vector3d::Unit(axis);
    const Eigen::Vector3d body_direction = orientation.conjugate() * direction;
    const Eigen::Vector3d angular =
        body.inverse_inertia() * r_body.cross(body_direction);
    metric.col(axis) = direction * inv_mass + orientation * angular.cross(r_body);
  }
  return 0.5 * (metric + metric.transpose());
}
} // namespace support_detail

inline GroundContactResult resolveFlatGroundContact(RigidBodyModel &body,
                                                    double world_ground_z,
                                                    double dt = 0.0) {
  using support_detail::apply_impulse;
  using support_detail::inverse_mass;
  GroundContactResult result;
  result.base_acceleration_world = body.base_acceleration();
  const auto &parameters = body.parameters();
  if (!std::isfinite(world_ground_z) || !std::isfinite(dt) || dt < 0.0)
    return result.valid = false, result;

  const double plane = world_ground_z;
  const Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  RigidBodyState state = body.state();
  const Eigen::Vector3d base_velocity_before = body.base_velocity();
  std::array<Eigen::Vector3d, Fs150SupportFace::kCorners> lever{};
  std::array<double, Fs150SupportFace::kCorners> lambda{};
  // Fixed world axes. The plane is horizontal, so Coulomb friction lives in
  // XY. A slip-aligned basis changes every pass and the stored impulse then
  // points with the velocity, which diverges on impact.
  std::array<double, Fs150SupportFace::kCorners> friction_x{};
  std::array<double, Fs150SupportFace::kCorners> friction_y{};
  std::array<bool, Fs150SupportFace::kCorners> active{};

  auto place = [&](int index) {
    return (state.position + state.orientation * lever[index]).eval();
  };
  for (int i = 0; i < Fs150SupportFace::kCorners; ++i)
    lever[i] = Fs150SupportFace::corner(i) - parameters.body_origin_to_com;

  double lowest = place(0).z();
  for (int i = 1; i < Fs150SupportFace::kCorners; ++i)
    lowest = std::min(lowest, place(i).z());
  if (!std::isfinite(lowest))
    return result.valid = false, result;
  if (lowest > plane + Fs150SupportFace::kMinDepth)
    return result;
  if (lowest < plane)
    state.position.z() += plane - lowest;

  bool any = false;
  for (int i = 0; i < Fs150SupportFace::kCorners; ++i) {
    active[i] = place(i).z() <= plane + Fs150SupportFace::kMinDepth;
    any = any || active[i];
  }
  if (!any)
    return result;

  auto point_velocity = [&](int index) {
    return (state.velocity + state.orientation * state.angular_velocity.cross(lever[index])).eval();
  };
  // Numerical slack so a resting corner at 1e-16 m/s still carries weight.
  // Not a physical speed.
  constexpr double kBoundSpeed = 1e-9;

  std::array<Eigen::Matrix3d, Fs150SupportFace::kCorners> metric{};
  std::array<Eigen::LDLT<Eigen::Matrix3d>, Fs150SupportFace::kCorners> factor;
  for (int i = 0; i < Fs150SupportFace::kCorners; ++i) {
    if (!active[i])
      continue;
    metric[i] = support_detail::point_delassus(body, state.orientation, lever[i]);
    if (!metric[i].allFinite() || metric[i](2, 2) <= 0.0)
      return result.valid = false, result;
    factor[i].compute(metric[i]);
    if (factor[i].info() != Eigen::Success)
      return result.valid = false, result;
  }

  for (int pass = 0; pass < Fs150SupportFace::kPasses; ++pass) {
    for (int i = 0; i < Fs150SupportFace::kCorners; ++i) {
      if (!active[i])
        continue;
      const Eigen::Matrix3d &mass = metric[i];
      const Eigen::Vector3d velocity = point_velocity(i);
      const Eigen::Vector3d previous(friction_x[i], friction_y[i], lambda[i]);
      const Eigen::Vector3d free_velocity = velocity - mass * previous;
      // Speed with this corner's impulse removed. A positive value is
      // separation: friction must not be stripped afterwards just because the
      // tangent impulse itself made the normal speed positive.
      Eigen::Vector3d impulse = Eigen::Vector3d::Zero();
      if (free_velocity.z() <= kBoundSpeed) {
        impulse = -factor[i].solve(free_velocity);
        if (!impulse.allFinite())
          return result.valid = false, result;
        const double tangential = impulse.head<2>().norm();
        const bool inside_cone =
            impulse.z() >= 0.0 && tangential <= Fs150SupportFace::kMu * impulse.z();
        if (!inside_cone) {
          Eigen::Vector2d tangent_dir = impulse.head<2>();
          if (tangent_dir.norm() <= 1e-12)
            tangent_dir = -free_velocity.head<2>();
          if (tangent_dir.norm() <= kBoundSpeed) {
            impulse = free_velocity.z() < 0.0
                          ? Eigen::Vector3d(0.0, 0.0, -free_velocity.z() / mass(2, 2))
                          : Eigen::Vector3d::Zero();
          } else {
            tangent_dir.normalize();
            const Eigen::Vector3d ray(Fs150SupportFace::kMu * tangent_dir.x(),
                                      Fs150SupportFace::kMu * tangent_dir.y(), 1.0);
            const double denominator = (mass * ray).z();
            if (free_velocity.z() < 0.0 && denominator > 1e-8)
              impulse = (-free_velocity.z() / denominator) * ray;
            else if (free_velocity.z() < 0.0)
              impulse = {0.0, 0.0, -free_velocity.z() / mass(2, 2)};
            else
              impulse.setZero();
          }
          if (!impulse.allFinite())
            return result.valid = false, result;
        }
      }
      if (impulse.z() < 0.0)
        impulse.setZero();
      else {
        const double limit = Fs150SupportFace::kMu * impulse.z();
        const double held_tangent = std::hypot(impulse.x(), impulse.y());
        if (held_tangent > limit) {
          const double scale = limit / held_tangent;
          impulse.x() *= scale;
          impulse.y() *= scale;
        }
      }
      const Eigen::Vector3d delta = impulse - previous;
      apply_impulse(state, body, lever[i], Eigen::Vector3d::UnitX(), delta.x());
      apply_impulse(state, body, lever[i], Eigen::Vector3d::UnitY(), delta.y());
      apply_impulse(state, body, lever[i], normal, delta.z());
      friction_x[i] = impulse.x();
      friction_y[i] = impulse.y();
      lambda[i] = impulse.z();
      // Cone projection changes the normal speed. Close any remaining inward
      // speed with normal impulse only; do not shrink the friction again.
      const double opening = point_velocity(i).z();
      if (opening < -kBoundSpeed) {
        const double normal_mass = inverse_mass(body, state.orientation, lever[i], normal);
        if (!std::isfinite(normal_mass) || normal_mass <= 0.0)
          return result.valid = false, result;
        const double extra = -opening / normal_mass;
        apply_impulse(state, body, lever[i], normal, extra);
        lambda[i] += extra;
      }
    }
  }

  for (int i = 0; i < Fs150SupportFace::kCorners; ++i) {
    if (!active[i])
      continue;
    const double opening = point_velocity(i).z();
    if (opening >= -kBoundSpeed)
      continue;
    const double normal_mass = inverse_mass(body, state.orientation, lever[i], normal);
    if (!std::isfinite(normal_mass) || normal_mass <= 0.0)
      return result.valid = false, result;
    const double extra = -opening / normal_mass;
    apply_impulse(state, body, lever[i], normal, extra);
    lambda[i] += extra;
  }

  if (!state.position.allFinite() || !state.velocity.allFinite() ||
      !state.angular_velocity.allFinite())
    return result.valid = false, result;
  body.set_state(state);
  // Band membership alone is not contact. Separating corners above or on the
  // plane, with no normal impulse, must not report constrained: FlightModel
  // disarms armed Land on that flag. Resting speed and a real impulse still
  // hold. Penetration was already lifted above, before this decision.
  bool held = false;
  for (int i = 0; i < Fs150SupportFace::kCorners; ++i) {
    result.normal_impulse += lambda[i];
    if (active[i] && (lambda[i] > 0.0 || point_velocity(i).z() <= kBoundSpeed))
      held = true;
  }
  result.constrained = held;
  result.base_acceleration_world = body.base_acceleration();
  if (dt > 0.0) {
    result.base_acceleration_world += (body.base_velocity() - base_velocity_before) / dt;
    return result;
  }
  if (!held)
    return result;

  // No elapsed interval: report the support force that cancels inward
  // acceleration. Velocity already holds the inelastic impulse above.
  Eigen::Vector3d linear = body.acceleration_world();
  Eigen::Vector3d angular = body.angular_acceleration_body();
  const Eigen::Vector3d &inertia_lever = parameters.body_origin_to_com;
  std::array<double, Fs150SupportFace::kCorners> force{};
  auto point_acceleration = [&](int index) {
    const Eigen::Vector3d &w = state.angular_velocity;
    return (linear + state.orientation *
                         (angular.cross(lever[index]) + w.cross(w.cross(lever[index]))))
        .eval();
  };
  auto apply_force = [&](int index, const Eigen::Vector3d &direction, double magnitude) {
    if (magnitude == 0.0)
      return;
    const Eigen::Vector3d normal_body = state.orientation.conjugate() * direction;
    const Eigen::Vector3d arm = lever[index].cross(normal_body);
    linear += direction * (magnitude / parameters.mass);
    angular += body.inverse_inertia() * arm * magnitude;
  };
  for (int pass = 0; pass < Fs150SupportFace::kPasses; ++pass) {
    for (int i = 0; i < Fs150SupportFace::kCorners; ++i) {
      if (!active[i] || point_velocity(i).z() > kBoundSpeed)
        continue;
      const double inward = point_acceleration(i).z();
      const double normal_mass = inverse_mass(body, state.orientation, lever[i], normal);
      const double next = std::max(0.0, force[i] - inward / normal_mass);
      apply_force(i, normal, next - force[i]);
      force[i] = next;
    }
  }
  const Eigen::Vector3d &w = state.angular_velocity;
  result.base_acceleration_world =
      linear - state.orientation * (angular.cross(inertia_lever) + w.cross(w.cross(inertia_lever)));
  return result;
}

} // namespace xgc_lightweight
