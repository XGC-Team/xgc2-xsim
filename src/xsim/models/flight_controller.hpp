#pragma once

#include "rigid_body.hpp"
#include "fs150_native_flight_model.hpp"
#include "px4_setpoint.hpp"
#include "px4_v1_12_3/include/px4_reference.hpp"

#include <Eigen/LU>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string_view>

namespace xgc_lightweight {

struct ParameterDefinition {
  const char *name;
  const char *unit;
  double px4_default;
  double fs_actual;
  double minimum;
  double maximum;
  bool integer;
  const char *declaration_source;
};

struct ParameterSnapshot {
  const char *name;
  const char *unit;
  double px4_default;
  double fs_actual;
  double effective;
  double minimum;
  double maximum;
  bool integer;
  const char *declaration_source;
  const char *effective_source;
  bool startup_override;
};

// Single startup parameter source. Defaults are pinned PX4 v1.12.3
// 2e8918da66af37922ededee1cc2d2efffec4cfb2 parameter declarations; FS values
// are the active config/generated/fs150-sitl.params overlay; keys not
// present there retain the pinned defaults (the full hardware export is provenance only). No simulation gain table or hover-to-force compensation exists.
class FlightControllerParameters {
public:
  static constexpr size_t kCount = 46;
  static double fs_actual(std::string_view name) {
    for (const auto &entry : fs150_native_asset::fcu_overrides)
      if (name == entry.name)
        return entry.value;
    return std::numeric_limits<double>::quiet_NaN();
  }
  static const std::array<ParameterDefinition, kCount> &definitions() {
    static const std::array<ParameterDefinition, kCount> rows{{
      {"MC_ROLL_P", "1/s", 6.5, fs_actual("MC_ROLL_P"), 0, 12, false, "src/modules/mc_att_control/mc_att_control_params.c"},
      {"MC_PITCH_P", "1/s", 6.5, fs_actual("MC_PITCH_P"), 0, 12, false, "src/modules/mc_att_control/mc_att_control_params.c"},
      {"MC_YAW_P", "1/s", 2.7999999999999998, fs_actual("MC_YAW_P"), 0, 5, false, "src/modules/mc_att_control/mc_att_control_params.c"},
      {"MC_YAW_WEIGHT", "normalized", 0.40000000000000002, fs_actual("MC_YAW_WEIGHT"), 0, 1, false, "src/modules/mc_att_control/mc_att_control_params.c"},
      {"MC_ROLLRATE_K", "normalized", 1, fs_actual("MC_ROLLRATE_K"), 0.01, 5, false, "src/modules/mc_rate_control/mc_rate_control_params.c"},
      {"MC_ROLLRATE_P", "s/rad", 0.14999999999999999, fs_actual("MC_ROLLRATE_P"), 0.01, 0.5, false, "src/modules/mc_rate_control/mc_rate_control_params.c"},
      {"MC_ROLLRATE_I", "1/rad", 0.20000000000000001, fs_actual("MC_ROLLRATE_I"), 0, std::numeric_limits<double>::infinity(), false, "src/modules/mc_rate_control/mc_rate_control_params.c"},
      {"MC_ROLLRATE_D", "s^2/rad", 0.0030000000000000001, fs_actual("MC_ROLLRATE_D"), 0, 0.01, false, "src/modules/mc_rate_control/mc_rate_control_params.c"},
      {"MC_ROLLRATE_FF", "s/rad", 0, fs_actual("MC_ROLLRATE_FF"), 0, std::numeric_limits<double>::infinity(), false, "src/modules/mc_rate_control/mc_rate_control_params.c"},
      {"MC_RR_INT_LIM", "normalized", 0.29999999999999999, fs_actual("MC_RR_INT_LIM"), 0, std::numeric_limits<double>::infinity(), false, "src/modules/mc_rate_control/mc_rate_control_params.c"},
      {"MC_ROLLRATE_MAX", "deg/s", 220, fs_actual("MC_ROLLRATE_MAX"), 0, 1800, false, "src/modules/mc_att_control/mc_att_control_params.c"},
      {"MC_PITCHRATE_K", "normalized", 1, fs_actual("MC_PITCHRATE_K"), 0.01, 5, false, "src/modules/mc_rate_control/mc_rate_control_params.c"},
      {"MC_PITCHRATE_P", "s/rad", 0.14999999999999999, fs_actual("MC_PITCHRATE_P"), 0.01, 0.59999999999999998, false, "src/modules/mc_rate_control/mc_rate_control_params.c"},
      {"MC_PITCHRATE_I", "1/rad", 0.20000000000000001, fs_actual("MC_PITCHRATE_I"), 0, std::numeric_limits<double>::infinity(), false, "src/modules/mc_rate_control/mc_rate_control_params.c"},
      {"MC_PITCHRATE_D", "s^2/rad", 0.0030000000000000001, fs_actual("MC_PITCHRATE_D"), 0, std::numeric_limits<double>::infinity(), false, "src/modules/mc_rate_control/mc_rate_control_params.c"},
      {"MC_PITCHRATE_FF", "s/rad", 0, fs_actual("MC_PITCHRATE_FF"), 0, std::numeric_limits<double>::infinity(), false, "src/modules/mc_rate_control/mc_rate_control_params.c"},
      {"MC_PR_INT_LIM", "normalized", 0.29999999999999999, fs_actual("MC_PR_INT_LIM"), 0, std::numeric_limits<double>::infinity(), false, "src/modules/mc_rate_control/mc_rate_control_params.c"},
      {"MC_PITCHRATE_MAX", "deg/s", 220, fs_actual("MC_PITCHRATE_MAX"), 0, 1800, false, "src/modules/mc_att_control/mc_att_control_params.c"},
      {"MC_YAWRATE_K", "normalized", 1, fs_actual("MC_YAWRATE_K"), 0, 5, false, "src/modules/mc_rate_control/mc_rate_control_params.c"},
      {"MC_YAWRATE_P", "s/rad", 0.20000000000000001, fs_actual("MC_YAWRATE_P"), 0, 0.59999999999999998, false, "src/modules/mc_rate_control/mc_rate_control_params.c"},
      {"MC_YAWRATE_I", "1/rad", 0.10000000000000001, fs_actual("MC_YAWRATE_I"), 0, std::numeric_limits<double>::infinity(), false, "src/modules/mc_rate_control/mc_rate_control_params.c"},
      {"MC_YAWRATE_D", "s^2/rad", 0, fs_actual("MC_YAWRATE_D"), 0, std::numeric_limits<double>::infinity(), false, "src/modules/mc_rate_control/mc_rate_control_params.c"},
      {"MC_YAWRATE_FF", "s/rad", 0, fs_actual("MC_YAWRATE_FF"), 0, std::numeric_limits<double>::infinity(), false, "src/modules/mc_rate_control/mc_rate_control_params.c"},
      {"MC_YR_INT_LIM", "normalized", 0.29999999999999999, fs_actual("MC_YR_INT_LIM"), 0, std::numeric_limits<double>::infinity(), false, "src/modules/mc_rate_control/mc_rate_control_params.c"},
      {"MC_YAWRATE_MAX", "deg/s", 200, fs_actual("MC_YAWRATE_MAX"), 0, 1800, false, "src/modules/mc_att_control/mc_att_control_params.c"},
      {"MPC_XY_P", "1/s", 0.94999999999999996, fs_actual("MPC_XY_P"), 0, 2, false, "src/modules/mc_pos_control/mc_pos_control_params.c"},
      {"MPC_Z_P", "1/s", 1, fs_actual("MPC_Z_P"), 0, 1.5, false, "src/modules/mc_pos_control/mc_pos_control_params.c"},
      {"MPC_XY_VEL_P_ACC", "1/s", 1.8, fs_actual("MPC_XY_VEL_P_ACC"), 1.2, 5, false, "src/modules/mc_pos_control/mc_pos_control_params.c"},
      {"MPC_XY_VEL_I_ACC", "1/s^2", 0.40000000000000002, fs_actual("MPC_XY_VEL_I_ACC"), 0, 60, false, "src/modules/mc_pos_control/mc_pos_control_params.c"},
      {"MPC_XY_VEL_D_ACC", "dimensionless", 0.20000000000000001, fs_actual("MPC_XY_VEL_D_ACC"), 0.10000000000000001, 2, false, "src/modules/mc_pos_control/mc_pos_control_params.c"},
      {"MPC_Z_VEL_P_ACC", "1/s", 4, fs_actual("MPC_Z_VEL_P_ACC"), 2, 15, false, "src/modules/mc_pos_control/mc_pos_control_params.c"},
      {"MPC_Z_VEL_I_ACC", "1/s^2", 2, fs_actual("MPC_Z_VEL_I_ACC"), 0.20000000000000001, 3, false, "src/modules/mc_pos_control/mc_pos_control_params.c"},
      {"MPC_Z_VEL_D_ACC", "dimensionless", 0, fs_actual("MPC_Z_VEL_D_ACC"), 0, 2, false, "src/modules/mc_pos_control/mc_pos_control_params.c"},
      {"MPC_XY_VEL_MAX", "m/s", 12, fs_actual("MPC_XY_VEL_MAX"), 0, 20, false, "src/modules/mc_pos_control/mc_pos_control_params.c"},
      {"MPC_Z_VEL_MAX_UP", "m/s", 3, fs_actual("MPC_Z_VEL_MAX_UP"), 0.5, 8, false, "src/modules/mc_pos_control/mc_pos_control_params.c"},
      {"MPC_Z_VEL_MAX_DN", "m/s", 1, fs_actual("MPC_Z_VEL_MAX_DN"), 0.5, 4, false, "src/modules/mc_pos_control/mc_pos_control_params.c"},
      {"MPC_THR_MIN", "norm", 0.12, fs_actual("MPC_THR_MIN"), 0.050000000000000003, 1, false, "src/modules/mc_pos_control/mc_pos_control_params.c"},
      {"MPC_THR_MAX", "norm", 1, fs_actual("MPC_THR_MAX"), 0, 1, false, "src/modules/mc_pos_control/mc_pos_control_params.c"},
      {"MPC_THR_HOVER", "norm", 0.5, fs_actual("MPC_THR_HOVER"), 0.10000000000000001, 0.80000000000000004, false, "src/modules/mc_pos_control/mc_pos_control_params.c"},
      {"MPC_TILTMAX_AIR", "deg", 45, fs_actual("MPC_TILTMAX_AIR"), 20, 89, false, "src/modules/mc_pos_control/mc_pos_control_params.c"},
      {"MPC_TILTMAX_LND", "deg", 12, fs_actual("MPC_TILTMAX_LND"), 10, 89, false, "src/modules/mc_pos_control/mc_pos_control_params.c"},
      {"MPC_LAND_SPEED", "m/s", 0.69999999999999996, fs_actual("MPC_LAND_SPEED"), 0.59999999999999998, std::numeric_limits<double>::infinity(), false, "src/modules/mc_pos_control/mc_pos_control_params.c"},
      {"MC_AIRMODE", "enum", 0, fs_actual("MC_AIRMODE"), 0, 2, true, "src/modules/mc_att_control/mc_att_control_params.c"},
      {"THR_MDL_FAC", "normalized", 0, fs_actual("THR_MDL_FAC"), 0, 1, false, "src/lib/mixer_module/motor_params.c"},
      {"MOT_SLEW_MAX", "s/(1000*PWM)", 0, fs_actual("MOT_SLEW_MAX"), 0, std::numeric_limits<double>::infinity(), false, "src/lib/mixer_module/motor_params.c"},
      {"COM_OF_LOSS_T", "s", 1, fs_actual("COM_OF_LOSS_T"), 0, 60, false, "src/modules/commander/commander_params.c"},
    }};
    return rows;
  }
  FlightControllerParameters() {
    const auto &rows = definitions();
    for (size_t i = 0; i != kCount; ++i)
      values_[i] = static_cast<float>(std::isfinite(rows[i].fs_actual)
                                         ? rows[i].fs_actual
                                         : rows[i].px4_default);
  }
  bool set(std::string_view name, double value) noexcept {
    const auto &rows = definitions();
    for (size_t i = 0; i != kCount; ++i) {
      if (name != rows[i].name)
        continue;
      if (overridden_[i] || !std::isfinite(value) ||
          !std::isfinite(static_cast<float>(value)) || value < rows[i].minimum ||
          value > rows[i].maximum || (rows[i].integer && std::trunc(value) != value))
        return false;
      values_[i] = static_cast<float>(value);
      overridden_[i] = true;
      return true;
    }
    return false;
  }
  float get(std::string_view name) const {
    const auto &rows = definitions();
    for (size_t i = 0; i != kCount; ++i)
      if (name == rows[i].name)
        return values_[i];
    throw std::invalid_argument("unknown PX4 FCU parameter");
  }
  std::array<ParameterSnapshot, kCount> snapshot() const {
    std::array<ParameterSnapshot, kCount> result{};
    const auto &rows = definitions();
    for (size_t i = 0; i != kCount; ++i) {
      const auto &r = rows[i];
      result[i] = {r.name, r.unit, r.px4_default, r.fs_actual, values_[i],
                   r.minimum, r.maximum, r.integer, r.declaration_source,
                   overridden_[i] ? "startup fcu_parameters"
                       : std::isfinite(r.fs_actual)
                             ? "fs150-sitl/config/generated/fs150-sitl.params"
                             : "PX4 v1.12.3 parameter default",
                   overridden_[i]};
    }
    return result;
  }

private:
  std::array<float, kCount> values_{};
  std::array<bool, kCount> overridden_{};
};

// The only internal frame bridge. Receiver values have already completed
// the MAVROS path and are ENU/FLU; do not apply that frontend again here.
namespace Px4Frame {
inline matrix::Vector3f world(const Eigen::Vector3d &enu) {
  return {static_cast<float>(enu.y()), static_cast<float>(enu.x()),
          static_cast<float>(-enu.z())};
}
inline Eigen::Vector3d world(const matrix::Vector3f &ned) {
  return {ned(1), ned(0), -ned(2)};
}
inline matrix::Vector3f body(const Eigen::Vector3d &flu) {
  return {static_cast<float>(flu.x()), static_cast<float>(-flu.y()),
          static_cast<float>(-flu.z())};
}
inline Eigen::Vector3d body(const matrix::Vector3f &frd) {
  return {frd(0), -frd(1), -frd(2)};
}
inline const Eigen::Quaterniond &world_rotation() {
  static const Eigen::Quaterniond q{0.0, std::sqrt(0.5), std::sqrt(0.5), 0.0};
  return q;
}
inline const Eigen::Quaterniond &body_rotation() {
  static const Eigen::Quaterniond q{0.0, 1.0, 0.0, 0.0};
  return q;
}
inline matrix::Quatf attitude(const Eigen::Quaterniond &flu_enu) {
  const auto q = (world_rotation() * flu_enu * body_rotation()).normalized();
  return {static_cast<float>(q.w()), static_cast<float>(q.x()),
          static_cast<float>(q.y()), static_cast<float>(q.z())};
}
inline Eigen::Quaterniond attitude(const matrix::Quatf &frd_ned) {
  const Eigen::Quaterniond q(frd_ned(0), frd_ned(1), frd_ned(2), frd_ned(3));
  return (world_rotation().conjugate() * q * body_rotation().conjugate()).normalized();
}
inline float heading(double enu) {
  constexpr double kPi = 3.141592653589793238462643383279502884;
  return static_cast<float>(std::remainder(0.5 * kPi - enu, 2.0 * kPi));
}
inline float radians(float degrees) {
  constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;
  return degrees * kDegToRad;
}
inline vehicle_local_position_setpoint_s setpoint(const MaskedPva &value) {
  vehicle_local_position_setpoint_s out{};
  const auto p = world(value.position), v = world(value.velocity), a = world(value.acceleration);
  out.x = p(0); out.y = p(1); out.z = p(2);
  out.vx = v(0); out.vy = v(1); out.vz = v(2);
  a.copyTo(out.acceleration);
  out.yaw = heading(value.yaw);
  out.yawspeed = static_cast<float>(-value.yaw_rate);
  return out;
}
} // namespace Px4Frame

struct FlightFeedback {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()}; // base, ENU
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()}; // base, ENU
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()}; // actual base, ENU
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()}; // FLU->ENU
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()}; // FLU
  Eigen::Vector3d angular_acceleration{Eigen::Vector3d::Zero()}; // FLU
};

struct RotorAllocation {
  Eigen::Vector4d motor_commands{Eigen::Vector4d::Zero()}; // [0,1], after THR model
  Eigen::Vector4d target_rotor_speed{Eigen::Vector4d::Zero()};
  Eigen::Vector4d allocated_wrench{Eigen::Vector4d::Zero()}; // target T, tau FLU (N, N m)
  uint16_t saturation_status{0}; // original MultirotorMixer bit layout
  bool collective_saturated{false}; // additionally records out-of-range raw input
  bool saturated() const {
    MultirotorMixer::saturation_status status{};
    status.value = saturation_status;
    return collective_saturated || status.flags.motor_pos || status.flags.motor_neg ||
           status.flags.roll_pos || status.flags.roll_neg || status.flags.pitch_pos ||
           status.flags.pitch_neg || status.flags.yaw_pos || status.flags.yaw_neg ||
           status.flags.thrust_pos || status.flags.thrust_neg;
  }
};

struct FlightControlOutput {
  Eigen::Vector3d acceleration_command{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond desired_orientation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d desired_body_rate{Eigen::Vector3d::Zero()};
  Eigen::Vector3d commanded_body_rate{Eigen::Vector3d::Zero()};
  Eigen::Vector3d normalized_torque_frd{Eigen::Vector3d::Zero()}; // NOT N m
  // MAVLink ATTITUDE_TARGET v1.12.3 reports the FCU collective setpoint,
  // independently of mixer desaturation/airmode/curve or physical motor lag.
  double normalized_thrust{0.0};
  double allocated_normalized_thrust{0.0}; // diagnostic post-mixer thrust-model mean
  uint32_t type_mask{7};
  bool position_control_valid{true}; // separate from receiver heartbeat acceptance
  RotorAllocation allocation;
};

class FlightController {
public:
  // Generated normalized 4x rows at the pinned source, in FS rotor0..3 order.
  // This is the upstream geometry, not a new inverse allocator for FS inertia.
  static const std::array<MultirotorMixer::Rotor, 4> &rotors() {
    const auto geometry = static_cast<unsigned>(MultirotorGeometry::QUAD_X);
    if (_config_rotor_count[geometry] != 4 ||
        std::string_view(_config_key[geometry]) != "4x")
      throw std::runtime_error("fixed PX4 quad_x geometry mismatch");
    static const std::array<MultirotorMixer::Rotor, 4> values{{
        _config_quad_x[0], _config_quad_x[1], _config_quad_x[2], _config_quad_x[3]}};
    return values;
  }

  explicit FlightController(const RigidBodyParameters &plant,
                            const FlightControllerParameters &parameters = {})
      : parameters_(parameters),
        tilt_air_(parameters.get("MPC_TILTMAX_AIR")),
        tilt_land_(parameters.get("MPC_TILTMAX_LND")),
        motor_slew_max_(parameters.get("MOT_SLEW_MAX")),
        thrust_model_factor_(parameters.get("THR_MDL_FAC")),
        controls_(std::make_unique<std::array<float, 4>>()),
        mixer_(std::make_unique<MultirotorMixer>(&control_callback,
            reinterpret_cast<uintptr_t>(controls_.get()), rotors().data(), 4)) {
    if (parameters.get("MPC_THR_MIN") > parameters.get("MPC_THR_MAX") ||
        plant.max_rotor_speed < kMotorArmedZero + kMotorScaling)
      throw std::invalid_argument("invalid thrust limits or FS motor command range");
    if (&plant == &fs150EquivalentParameters() ||
        plant == fs150EquivalentParameters()) {
      plant_ = &fs150EquivalentParameters();
      allocation_ = &canonical_allocation();
    } else {
      custom_plant_ = std::make_unique<CustomPlant>(plant, build_allocation(plant));
      plant_ = &custom_plant_->plant;
      allocation_ = &custom_plant_->allocation;
    }
    const auto p = [&parameters](const char *name) { return parameters.get(name); };
    attitude_.setProportionalGain({p("MC_ROLL_P"), p("MC_PITCH_P"), p("MC_YAW_P")},
                                  p("MC_YAW_WEIGHT"));
    attitude_.setRateLimit({Px4Frame::radians(p("MC_ROLLRATE_MAX")),
                            Px4Frame::radians(p("MC_PITCHRATE_MAX")),
                            Px4Frame::radians(p("MC_YAWRATE_MAX"))});
    const matrix::Vector3f k(p("MC_ROLLRATE_K"), p("MC_PITCHRATE_K"), p("MC_YAWRATE_K"));
    rate_.setGains(k.emult(matrix::Vector3f(p("MC_ROLLRATE_P"), p("MC_PITCHRATE_P"), p("MC_YAWRATE_P"))),
                   k.emult(matrix::Vector3f(p("MC_ROLLRATE_I"), p("MC_PITCHRATE_I"), p("MC_YAWRATE_I"))),
                   k.emult(matrix::Vector3f(p("MC_ROLLRATE_D"), p("MC_PITCHRATE_D"), p("MC_YAWRATE_D"))));
    rate_.setFeedForwardGain({p("MC_ROLLRATE_FF"), p("MC_PITCHRATE_FF"), p("MC_YAWRATE_FF")});
    rate_.setIntegratorLimit({p("MC_RR_INT_LIM"), p("MC_PR_INT_LIM"), p("MC_YR_INT_LIM")});
    position_.setPositionGains({p("MPC_XY_P"), p("MPC_XY_P"), p("MPC_Z_P")});
    position_.setVelocityGains({p("MPC_XY_VEL_P_ACC"), p("MPC_XY_VEL_P_ACC"), p("MPC_Z_VEL_P_ACC")},
                              {p("MPC_XY_VEL_I_ACC"), p("MPC_XY_VEL_I_ACC"), p("MPC_Z_VEL_I_ACC")},
                              {p("MPC_XY_VEL_D_ACC"), p("MPC_XY_VEL_D_ACC"), p("MPC_Z_VEL_D_ACC")});
    position_.setVelocityLimits(p("MPC_XY_VEL_MAX"), p("MPC_Z_VEL_MAX_UP"), p("MPC_Z_VEL_MAX_DN"));
    position_.setThrustLimits(p("MPC_THR_MIN"), p("MPC_THR_MAX"));
    position_.setHoverThrust(p("MPC_THR_HOVER"));
    mixer_->set_airmode(static_cast<Mixer::Airmode>(int(p("MC_AIRMODE"))));
    mixer_->set_thrust_factor(p("THR_MDL_FAC"));
  }
  FlightController(const FlightController &) = delete;
  FlightController &operator=(const FlightController &) = delete;
  FlightController(FlightController &&) = default;
  FlightController &operator=(FlightController &&) = default;
  const FlightControllerParameters &parameters() const { return parameters_; }
  void reset() {
    position_.resetIntegral();
    rate_.resetIntegral();
  }
  rate_ctrl_status_s rate_status() {
    rate_ctrl_status_s result{};
    rate_.getRateControlStatus(result);
    return result;
  }

  FlightControlOutput command_pva(const FlightFeedback &feedback,
                                 const MaskedPva &setpoint, double dt,
                                 bool landed, bool landing = false) {
    validate_dt(dt);
    const auto q = Px4Frame::attitude(feedback.orientation);
    PositionControlStates states{};
    states.position = Px4Frame::world(feedback.position);
    states.velocity = Px4Frame::world(feedback.velocity);
    states.acceleration = Px4Frame::world(feedback.acceleration);
    states.yaw = matrix::Eulerf(q).psi();
    position_.setState(states);
    position_.setTiltLimit(Px4Frame::radians(landing ? tilt_land_ : tilt_air_));
    position_.setInputSetpoint(Px4Frame::setpoint(setpoint));
    const bool valid = position_.update(static_cast<float>(dt));
    if (!valid) {
      // Same source fallback mechanism as MulticopterPositionControl.cpp:
      // feed a finite velocity stop into PositionControl, then update again.
      // This never rewrites the receiver's heartbeat decision.
      MaskedPva stop;
      stop.velocity.setZero();
      position_.setInputSetpoint(Px4Frame::setpoint(stop));
      position_.update(static_cast<float>(dt));
    }
    vehicle_attitude_setpoint_s target{};
    vehicle_local_position_setpoint_s executed{};
    position_.getAttitudeSetpoint(target);
    position_.getLocalPositionSetpoint(executed);
    const matrix::Quatf desired(target.q_d);
    attitude_.setAttitudeSetpoint(desired, target.yaw_sp_move_rate);
    FlightControlOutput result;
    result.position_control_valid = valid;
    result.desired_orientation = Px4Frame::attitude(desired);
    result.acceleration_command = Px4Frame::world(matrix::Vector3f(executed.acceleration));
    const auto rates = attitude_.update(q);
    result.desired_body_rate = Px4Frame::body(rates);
    finish(feedback, result, rates, -target.thrust_body[2], dt, landed);
    return result;
  }

  FlightControlOutput command_rate(const FlightFeedback &feedback,
                                  const Eigen::Vector3d &body_rate,
                                  double normalized_thrust, double dt, bool landed) {
    if (!body_rate.allFinite() || !std::isfinite(normalized_thrust))
      throw std::invalid_argument("invalid raw rate setpoint");
    validate_dt(dt);
    FlightControlOutput result;
    result.type_mask = 128;
    result.desired_orientation = feedback.orientation; // ignored by raw 128
    result.commanded_body_rate = result.desired_body_rate = body_rate;
    finish(feedback, result, Px4Frame::body(body_rate), normalized_thrust, dt, landed);
    return result;
  }
  FlightControlOutput command_attitude(
      const FlightFeedback &feedback, const Eigen::Quaterniond &q,
      double normalized_thrust, const Eigen::Vector3d &body_rate_ff,
      uint32_t type_mask, double dt, bool landed) {
    if ((type_mask != 0 && type_mask != 7) || !q.coeffs().allFinite() ||
        !std::isfinite(q.norm()) || q.norm() < 1e-12 ||
        (type_mask == 0 && !body_rate_ff.allFinite()) || !std::isfinite(normalized_thrust))
      throw std::invalid_argument("invalid raw attitude setpoint");
    validate_dt(dt);
    FlightControlOutput result;
    result.type_mask = type_mask;
    result.desired_orientation = q.normalized();
    const auto actual = Px4Frame::attitude(feedback.orientation);
    attitude_.setAttitudeSetpoint(Px4Frame::attitude(result.desired_orientation), 0.f);
    auto rates = attitude_.update(actual);
    if (type_mask == 0) {
      result.commanded_body_rate = body_rate_ff;
      rates += Px4Frame::body(body_rate_ff);
    }
    result.desired_body_rate = Px4Frame::body(rates);
    finish(feedback, result, rates, normalized_thrust, dt, landed);
    return result;
  }
  FlightControlOutput disarmed(double dt) {
    reset();
    FlightControlOutput result;
    result.allocation = mix(matrix::Vector3f{}, 0.0, dt, false);
    return result;
  }

  // FS renderer iris.sdf motor channel affine calibration, confirmed by the
  // pinned GazeboMavlinkInterface::handle_actuator_controls: 100+1000*m
  // while armed, 0 when disarmed. The rigid kernel owns motor lag and kf*w^2.
  static constexpr double kMotorScaling = fs150_native_asset::rotors[0].input_scaling;
  static constexpr double kMotorArmedZero = fs150_native_asset::rotors[0].zero_position_armed;
  Eigen::Vector4d wrench_for_motor_commands(const Eigen::Vector4d &motor,
                                           bool armed = true) const {
    Eigen::Vector4d force = Eigen::Vector4d::Zero();
    if (armed)
      for (int i = 0; i != 4; ++i) {
        const auto &channel = fs150_native_asset::rotors[i];
        const double omega = channel.zero_position_armed + channel.input_scaling *
                             (motor[i] + channel.input_offset);
        force[i] = plant_->thrust_coefficient * omega * omega;
      }
    return allocation_->physical_matrix * force;
  }
  Eigen::Vector4d normalized_controls_for_wrench(const Eigen::Vector4d &wrench) const {
    const auto force = allocation_->physical_lu.solve(wrench).eval();
    Eigen::Vector4d thrust_model;
    const double alpha = thrust_model_factor_;
    for (int i = 0; i != 4; ++i) {
      const auto &channel = fs150_native_asset::rotors[i];
      const double motor = (std::sqrt(force[i] / plant_->thrust_coefficient) -
                            channel.zero_position_armed) / channel.input_scaling - channel.input_offset;
      if (!std::isfinite(motor) || motor < -1e-10 || motor > 1.0 + 1e-10)
        throw std::invalid_argument("wrench outside unsaturated FS motor domain");
      thrust_model[i] = (1.0 - alpha) * motor + alpha * motor * motor;
    }
    // Only an unsaturated target has this unique reverse mapping. Mixer
    // desaturation intentionally loses input information; no inverse is claimed.
    return allocation_->normalized_lu.solve(thrust_model);
  }

private:
  struct AllocationMatrices {
    Eigen::Matrix4d physical_matrix{Eigen::Matrix4d::Zero()};
    Eigen::Matrix4d normalized_matrix{Eigen::Matrix4d::Zero()};
    Eigen::FullPivLU<Eigen::Matrix4d> physical_lu;
    Eigen::FullPivLU<Eigen::Matrix4d> normalized_lu;
  };
  struct CustomPlant {
    RigidBodyParameters plant;
    AllocationMatrices allocation;
    CustomPlant(const RigidBodyParameters &p, AllocationMatrices &&a)
        : plant(p), allocation(std::move(a)) {}
  };
  static AllocationMatrices build_allocation(const RigidBodyParameters &plant) {
    AllocationMatrices out;
    for (int i = 0; i != 4; ++i) {
      out.physical_matrix.col(i) << 1.0, plant.rotor_position[i].y(),
          -plant.rotor_position[i].x(), plant.moment_ratio * plant.rotor_yaw_sign[i];
      const auto &r = rotors()[i];
      out.normalized_matrix.row(i) << r.roll_scale, r.pitch_scale, r.yaw_scale, r.thrust_scale;
    }
    out.physical_lu.compute(out.physical_matrix);
    if (!out.physical_lu.isInvertible())
      throw std::invalid_argument("singular physical rotor geometry");
    out.normalized_lu.compute(out.normalized_matrix);
    return out;
  }
  static const AllocationMatrices &canonical_allocation() {
    static const AllocationMatrices value = build_allocation(fs150EquivalentParameters());
    return value;
  }

  // Unit-only access to replay the upstream component fixture schema. The
  // runtime still obtains saturation solely from its previous real mixer.
  friend struct FlightControllerReplay;
  static void validate_dt(double dt) {
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.02)
      throw std::invalid_argument("FCU dt must be in (0,.02]");
  }
  static int control_callback(uintptr_t handle, uint8_t group, uint8_t index, float &input) {
    if (group != 0 || index >= 4)
      return -1;
    input = (*reinterpret_cast<std::array<float, 4> *>(handle))[index];
    return 0;
  }
  RotorAllocation mix(const matrix::Vector3f &torque, double thrust,
                      double dt, bool armed) {
    *controls_ = {torque(0), torque(1), torque(2), static_cast<float>(thrust)};
    const float slew = motor_slew_max_;
    mixer_->set_max_delta_out_once(slew > 0.f ? 2.f * static_cast<float>(dt) / slew : 0.f);
    float outputs[4]{};
    if (mixer_->mix(outputs, 4) != 4)
      throw std::runtime_error("PX4 mixer failed");
    saturation_.value = mixer_->get_saturation_status();
    RotorAllocation result;
    result.saturation_status = saturation_.value;
    result.collective_saturated = thrust < 0.0 || thrust > 1.0 ||
                                 saturation_.flags.thrust_pos || saturation_.flags.thrust_neg;
    for (int i = 0; i != 4; ++i) {
      result.motor_commands[i] = armed ? (outputs[i] + 1.0) * 0.5 : 0.0;
      const auto &channel = fs150_native_asset::rotors[i];
      result.target_rotor_speed[i] = armed
          ? channel.zero_position_armed + channel.input_scaling *
                (result.motor_commands[i] + channel.input_offset)
          : channel.zero_position_disarmed;
    }
    result.allocated_wrench = wrench_for_motor_commands(result.motor_commands, armed);
    return result;
  }
  void finish(const FlightFeedback &feedback, FlightControlOutput &result,
              const matrix::Vector3f &rates, double thrust, double dt, bool landed) {
    result.normalized_thrust = std::clamp(thrust, 0.0, 1.0);
    result.commanded_body_rate = result.desired_body_rate; // actual FCU rate setpoint
    rate_.setSaturationStatus(saturation_); // previous actual mixer feedback
    auto torque = rate_.update(Px4Frame::body(feedback.angular_velocity), rates,
                               Px4Frame::body(feedback.angular_acceleration),
                               static_cast<float>(dt), landed);
    // Same finite-output guard as MulticopterRateControl's actuator publisher.
    for (int i = 0; i != 3; ++i)
      if (!std::isfinite(torque(i)))
        torque(i) = 0.f;
    result.normalized_torque_frd = {torque(0), torque(1), torque(2)};
    result.allocation = mix(torque, thrust, dt, true);
    const double alpha = thrust_model_factor_;
    for (int i = 0; i != 4; ++i) {
      const double m = result.allocation.motor_commands[i];
      result.allocated_normalized_thrust += ((1.0 - alpha) * m + alpha * m * m) / 4.0;
    }
  }

  std::unique_ptr<CustomPlant> custom_plant_;
  const RigidBodyParameters *plant_{nullptr};
  const AllocationMatrices *allocation_{nullptr};
  FlightControllerParameters parameters_;
  // Parameters are immutable after construction; reset/move retains the values.
  float tilt_air_, tilt_land_, motor_slew_max_, thrust_model_factor_;
  AttitudeControl attitude_;
  RateControl rate_;
  PositionControl position_;
  // The mixer callback storage stays stable when World moves dense models.
  std::unique_ptr<std::array<float, 4>> controls_;
  std::unique_ptr<MultirotorMixer> mixer_;
  MultirotorMixer::saturation_status saturation_{};
};

} // namespace xgc_lightweight
