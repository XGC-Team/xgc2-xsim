#pragma once

#include "fs150_native_flight_model.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <optional>
#include <vector>

namespace xgc_lightweight {

struct RigidBodyParameters {
  // The asset owner generates every machine value from the declared SDF.
  double mass{fs150_native_asset::mass};
  Eigen::Matrix3d inertia{Eigen::Matrix3d::Zero()};
  std::array<Eigen::Vector3d, 4> rotor_position{};
  Eigen::Vector4d rotor_yaw_sign{Eigen::Vector4d::Zero()};
  double thrust_coefficient{fs150_native_asset::rotors[0].kf};
  double moment_ratio{fs150_native_asset::rotors[0].moment_constant};
  double motor_tau_up{fs150_native_asset::rotors[0].tau_up};
  double motor_tau_down{fs150_native_asset::rotors[0].tau_down};
  double max_rotor_speed{fs150_native_asset::rotors[0].max_rot_velocity};
  double gravity{9.8066}; // SI gravity magnitude, not a machine calibration
  Eigen::Vector3d body_origin_to_com{Eigen::Vector3d::Zero()};

  RigidBodyParameters() {
    for (int i = 0; i < 3; ++i) {
      body_origin_to_com[i] = fs150_native_asset::center_of_mass[i];
      for (int j = 0; j < 3; ++j) inertia(i, j) = fs150_native_asset::inertia[i][j];
    }
    for (int i = 0; i < 4; ++i) {
      const auto &rotor = fs150_native_asset::rotors[i];
      for (int j = 0; j < 3; ++j) rotor_position[i][j] = rotor.position[j];
      rotor_yaw_sign[i] = rotor.reaction_torque_sign;
      if (rotor.motor_number != i || rotor.kf != thrust_coefficient ||
          rotor.moment_constant != moment_ratio || rotor.tau_up != motor_tau_up ||
          rotor.tau_down != motor_tau_down || rotor.max_rot_velocity != max_rotor_speed)
        throw std::invalid_argument("FS150 asset needs homogeneous motor physics");
    }
  }
};

// Provenance (paths relative to devops/products/ros1/simulator/gazebo-sim):
// fs150-sitl/scripts/render_fs150_indoor_sdf.py and
// fs150-sitl/models/fs150/iris.sdf.
// Base: .275 kg, J = Iris J*(.275/1.5)*.35; IMU: .015 kg, J=1e-5 I.
// Four .005 kg rotors: J=diag(9.75e-7,.000273104,.000274004)*.35,
// at the stated base-frame offsets. Parallel-axis aggregation about the
// total COM gives the inertia above (off-diagonals cancel), using stored SDF
// base inertia (.00186885417,.00186885417,.00354360417), rounded from scaling.
// GazeboMotorModel uses body yaw torque -turning_direction*force*momentConstant
// with CCW=+1, CW=-1: rotor 0/1 CCW therefore have NEGATIVE reaction torque.
// Verified against PX4 v1.12.0's Tools/sitl_gazebo gitlink
// 822050a7ab6fd87972e59f16312f451bce217a56, include/gazebo_motor_model.h:43-44
// and src/gazebo_motor_model.cpp:243-246 (PX4/PX4-SITL_gazebo on GitHub).
// This is the renderer's Iris-equivalent model, NOT real-airframe calibration.
// Rotor spin gyroscopic effects, air drag, rolling moments, wind and contacts
// are deliberately absent. The FCU wrapper owns ground/contact behaviour.
inline RigidBodyParameters fs150EquivalentParameters() { return {}; }

struct RigidBodyState {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()}; // COM, world ENU, m
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()}; // COM, world ENU, m/s
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()}; // FLU->ENU
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()}; // body, rad/s
  Eigen::Vector4d rotor_speed{Eigen::Vector4d::Zero()}; // nonnegative rad/s
};

// World-owned columns. Standalone numerical fixtures retain local storage;
// a bound model has no private physical state. Dense moves rebind its index.
struct BodyColumns {
  std::vector<Eigen::Vector3d> position, velocity, angular_velocity;
  std::vector<Eigen::Quaterniond> orientation;
  std::vector<Eigen::Vector4d> rotor_speed;
  size_t append(const RigidBodyState &s) {
    const auto i = position.size();
    position.push_back(s.position); velocity.push_back(s.velocity);
    orientation.push_back(s.orientation); angular_velocity.push_back(s.angular_velocity);
    rotor_speed.push_back(s.rotor_speed); return i;
  }
  RigidBodyState get(size_t i) const {
    return {position[i], velocity[i], orientation[i], angular_velocity[i], rotor_speed[i]};
  }
  void set(size_t i, const RigidBodyState &s) {
    position[i]=s.position; velocity[i]=s.velocity; orientation[i]=s.orientation;
    angular_velocity[i]=s.angular_velocity; rotor_speed[i]=s.rotor_speed;
  }
  void erase(size_t i) {
    set(i, get(position.size()-1));
    position.pop_back(); velocity.pop_back(); orientation.pop_back();
    angular_velocity.pop_back(); rotor_speed.pop_back();
  }
};

// Pure 6DoF plant. A command holds target rotor speeds for dt; attitude evolves
// ONLY through torque and qdot = .5*q*(0,omega_body). Motors follow their exact
// first-order response, sampled at every RK4 stage. Rigid-body state uses RK4
// with substeps <=2 ms (and <=.1 rad of rotation at the initial angular rate).
class RigidBodyModel {
public:
  explicit RigidBodyModel(
      const Eigen::Vector3d &initial_base_position, double yaw = 0.0,
      const RigidBodyParameters &params = fs150EquivalentParameters())
      : parameters_(params) {
    validate_parameters(parameters_);
    if (!initial_base_position.allFinite() || !std::isfinite(yaw))
      throw std::invalid_argument("nonfinite initial rigid-body pose");
    inverse_inertia_ = parameters_.inertia.llt().solve(Eigen::Matrix3d::Identity());
    if (!inverse_inertia_.allFinite())
      throw std::invalid_argument("inertia inverse is not representable");
    auto &state_ = *local_;
    state_.orientation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    state_.position = initial_base_position +
                      state_.orientation * parameters_.body_origin_to_com;
    if (!state_.position.allFinite())
      throw std::invalid_argument("overflow in initial COM position");
  }

  RigidBodyState state() const { return columns_ ? columns_->get(index_) : *local_; }
  void bind(BodyColumns &columns, size_t index) {
    const auto s = state(); columns_ = &columns; index_ = index;
    columns_->set(index_, s); local_.reset();
  }
  void rebind(size_t index) { index_ = index; }
  void assign_state(const RigidBodyState &s) {
    if (columns_) columns_->set(index_, s); else local_ = s;
  }
  const RigidBodyParameters &parameters() const { return parameters_; }
  const Eigen::Matrix3d &inverse_inertia() const { return inverse_inertia_; }

  // Controlled initialization/reset entry. Validates before mutation and
  // normalizes a finite nonzero quaternion; COM coordinates are not shifted.
  void set_state(const RigidBodyState &state) {
    validate_state(state);
    RigidBodyState next = state;
    next.orientation.normalize();
    assign_state(next);
  }

  // dt=0 is a validated no-op. Reject negative/nonfinite dt and pauses >.1 s;
  // callers catching up a longer elapsed time must supply bounded chunks.
  // Finite commands saturate to [0,max_rotor_speed]. Invalid input or numeric
  // overflow leaves the previous state intact (no partial step publication).
  // Limit each call to 10000 substeps so extreme finite rates cannot hang it.
  void step(const Eigen::Vector4d &target_rotor_speed, double dt) {
    if (!target_rotor_speed.allFinite() || !std::isfinite(dt) || dt < 0.0 ||
        dt > 0.1)
      throw std::invalid_argument("invalid rotor command or dt (expected [0,.1])");
    if (dt == 0.0)
      return;
    const Eigen::Vector4d target = target_rotor_speed.cwiseMax(0.0).cwiseMin(
        parameters_.max_rotor_speed);
    RigidBodyState next = state();
    double remaining = dt;
    unsigned int substeps = 0;
    while (remaining > 0.0) {
      if (++substeps > 10000)
        throw std::overflow_error("rigid-body step exceeds substep budget");
      const double rate = next.angular_velocity.stableNorm();
      // An unresolvable extreme state must fail rather than loop forever.
      const double h = std::min({remaining, 0.002, 0.1 / std::max(1.0, rate)});
      if (!std::isfinite(h) || h <= 0.0 || remaining - h == remaining)
        throw std::overflow_error("rigid-body step cannot resolve angular rate");
      rk4(next, target, h);
      validate_state(next);
      remaining = h == remaining ? 0.0 : remaining - h;
    }
    assign_state(next);
  }

  // Instantaneous endpoint accelerations, including gravity in world ENU.
  Eigen::Vector3d acceleration_world() const {
    return linear_acceleration(state());
  }
  Eigen::Vector3d angular_acceleration_body() const {
    return angular_acceleration(state());
  }
  Eigen::Vector3d base_position() const {
    const auto state_ = state();
    return state_.position - state_.orientation * parameters_.body_origin_to_com;
  }
  Eigen::Vector3d base_velocity() const {
    const auto state_ = state();
    return state_.velocity - state_.orientation *
        state_.angular_velocity.cross(parameters_.body_origin_to_com);
  }
  Eigen::Vector3d base_acceleration() const {
    const auto state_ = state();
    const Eigen::Vector3d &r = parameters_.body_origin_to_com;
    const Eigen::Vector3d &w = state_.angular_velocity;
    return acceleration_world() - state_.orientation *
        (angular_acceleration_body().cross(r) + w.cross(w.cross(r)));
  }

private:
  static void validate_parameters(const RigidBodyParameters &p) {
    const auto positive = [](double x) { return std::isfinite(x) && x > 0.0; };
    if (!positive(p.mass) || !positive(p.thrust_coefficient) ||
        !positive(p.motor_tau_up) || !positive(p.motor_tau_down) ||
        !positive(p.max_rotor_speed) || !std::isfinite(p.gravity) ||
        p.gravity < 0.0 || !std::isfinite(p.moment_ratio) || p.moment_ratio < 0.0 ||
        !p.inertia.allFinite() || !p.body_origin_to_com.allFinite() ||
        !p.rotor_yaw_sign.allFinite())
      throw std::invalid_argument("invalid rigid-body parameters");
    if (!p.inertia.isApprox(p.inertia.transpose(), 1e-12) ||
        Eigen::LLT<Eigen::Matrix3d>(p.inertia).info() != Eigen::Success)
      throw std::invalid_argument("inertia must be symmetric positive definite");
    for (int i = 0; i < 4; ++i)
      if (!p.rotor_position[i].allFinite() || std::abs(p.rotor_yaw_sign[i]) != 1.0)
        throw std::invalid_argument("invalid rotor position or reaction sign");
    const double max_force = p.thrust_coefficient * p.max_rotor_speed *
                             p.max_rotor_speed;
    if (!std::isfinite(max_force) || !std::isfinite(4.0 * max_force / p.mass))
      throw std::invalid_argument("unrepresentable rotor thrust");
  }

  void validate_state(const RigidBodyState &s) const {
    const double qnorm = s.orientation.norm();
    if (!s.position.allFinite() || !s.velocity.allFinite() ||
        !s.orientation.coeffs().allFinite() || !std::isfinite(qnorm) ||
        qnorm < 1e-12 || !s.angular_velocity.allFinite() ||
        !s.rotor_speed.allFinite() || (s.rotor_speed.array() < 0.0).any() ||
        (s.rotor_speed.array() > parameters_.max_rotor_speed).any())
      throw std::invalid_argument("invalid rigid-body state");
  }

  Eigen::Vector4d motor_at(const Eigen::Vector4d &initial,
                         const Eigen::Vector4d &target, double t) const {
    Eigen::Vector4d speed;
    std::optional<double> response_up, response_down;
    for (int i = 0; i < 4; ++i) {
      const bool increasing = target[i] > initial[i];
      const double tau = increasing ? parameters_.motor_tau_up
                                    : parameters_.motor_tau_down;
      auto &response = increasing ? response_up : response_down;
      if (!response)
        response = std::expm1(-t / tau);
      // expm1 avoids cancellation for tiny dt. This is a convex interpolation.
      speed[i] = initial[i] + (target[i] - initial[i]) * (-*response);
      speed[i] = std::clamp(speed[i], 0.0, parameters_.max_rotor_speed);
    }
    return speed;
  }

  Eigen::Vector4d forces(const RigidBodyState &s) const {
    return parameters_.thrust_coefficient * s.rotor_speed.array().square().matrix();
  }
  Eigen::Vector3d linear_acceleration(const RigidBodyState &s) const {
    return Eigen::Vector3d(0.0, 0.0, -parameters_.gravity) +
           s.orientation.normalized() *
               Eigen::Vector3d(0.0, 0.0, forces(s).sum() / parameters_.mass);
  }
  Eigen::Vector3d angular_acceleration(const RigidBodyState &s) const {
    const Eigen::Vector4d f = forces(s);
    Eigen::Vector3d torque = Eigen::Vector3d::Zero();
    for (int i = 0; i < 4; ++i)
      torque += parameters_.rotor_position[i].cross(Eigen::Vector3d(0, 0, f[i])) +
                Eigen::Vector3d(0, 0, parameters_.rotor_yaw_sign[i] *
                                           parameters_.moment_ratio * f[i]);
    return inverse_inertia_ *
           (torque - s.angular_velocity.cross(parameters_.inertia *
                                             s.angular_velocity));
  }

  struct Derivative {
    Eigen::Vector3d position, velocity, angular_velocity;
    Eigen::Vector4d quaternion; // Eigen coefficient order x,y,z,w
  };
  Derivative derivative(const RigidBodyState &s) const {
    const Eigen::Quaterniond spin(0.0, s.angular_velocity.x(),
                                 s.angular_velocity.y(), s.angular_velocity.z());
    return {s.velocity, linear_acceleration(s), angular_acceleration(s),
            0.5 * (s.orientation * spin).coeffs()};
  }
  static RigidBodyState stage(const RigidBodyState &s, const Derivative &d,
                             double t, const Eigen::Vector4d &motor) {
    RigidBodyState result = s;
    result.position += t * d.position;
    result.velocity += t * d.velocity;
    result.angular_velocity += t * d.angular_velocity;
    result.orientation.coeffs() += t * d.quaternion;
    result.rotor_speed = motor;
    return result;
  }
  void rk4(RigidBodyState &s, const Eigen::Vector4d &target, double h) const {
    const Eigen::Vector4d middle = motor_at(s.rotor_speed, target, 0.5 * h);
    const Eigen::Vector4d end = motor_at(s.rotor_speed, target, h);
    const Derivative k1 = derivative(s);
    const Derivative k2 = derivative(stage(s, k1, 0.5 * h, middle));
    const Derivative k3 = derivative(stage(s, k2, 0.5 * h, middle));
    const Derivative k4 = derivative(stage(s, k3, h, end));
    s.position += (h / 6.0) * (k1.position + 2.0 * k2.position +
                               2.0 * k3.position + k4.position);
    s.velocity += (h / 6.0) * (k1.velocity + 2.0 * k2.velocity +
                               2.0 * k3.velocity + k4.velocity);
    s.angular_velocity += (h / 6.0) *
        (k1.angular_velocity + 2.0 * k2.angular_velocity +
         2.0 * k3.angular_velocity + k4.angular_velocity);
    s.orientation.coeffs() += (h / 6.0) *
        (k1.quaternion + 2.0 * k2.quaternion + 2.0 * k3.quaternion + k4.quaternion);
    const double qnorm = s.orientation.norm();
    if (!std::isfinite(qnorm) || qnorm < 1e-12)
      throw std::overflow_error("rigid-body quaternion overflow");
    s.orientation.normalize();
    s.rotor_speed = end;
  }

  RigidBodyParameters parameters_;
  Eigen::Matrix3d inverse_inertia_;
  std::optional<RigidBodyState> local_{RigidBodyState{}};
  BodyColumns *columns_{nullptr};
  size_t index_{0};
};

} // namespace xgc_lightweight
