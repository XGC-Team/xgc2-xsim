#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include "flight_controller.hpp"
#include "ground_contact.hpp"

#include <xgc2_math/control/delayed_planar_velocity.hpp>
#include <xgc2_math/geometry/kinematics.hpp>

// The numerical plant has no ROS, transport or scheduler. Native and ROS
// boundaries supply commands and elapsed model time; flight embeds an FCU loop.
namespace xgc_lightweight {

using FlightSetpoint = MaskedPva;

struct FlightAttitudeSetpoint {
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d body_rate{Eigen::Vector3d::Zero()};
  double thrust{0.0};
  uint32_t type_mask{128};
};

enum class FlightMode { Hold, Offboard, Land };
enum class FlightEvent { None, OffboardLost, Landed };

// FCU service/stream facade around the fixed PX4 pure control classes.
// No EKF, GPS/RC/preflight, uORB, full Commander or firmware is simulated.
class FlightModel {
public:
  static constexpr double kLandedHeight = 0.01;
  static constexpr double kMaxStep = 0.02;

  explicit FlightModel(const Eigen::Vector3d &position, double yaw = 0.0,
                       const FlightControllerParameters &parameters = {},
                       double world_ground_z = 0.0)
      : body_(position, yaw), controller_(body_.parameters(), parameters),
        ground_z_(world_ground_z), initial_position_(position), initial_yaw_(yaw),
        land_speed_(parameters.get("MPC_LAND_SPEED")),
        offboard_loss_time_(parameters.get("COM_OF_LOSS_T")) {
    if (!std::isfinite(world_ground_z))
      throw std::invalid_argument("flight model: invalid world ground");
    constrain_ground(0.0);
  }
  // World-thread lifecycle operation: one slot only. This recreates the
  // noncopyable mixer and clears physical/control/request histories; no Host
  // world clock or other robot is represented or modified by this object.
  void resetToInitial() {
    FlightModel fresh(initial_position_, initial_yaw_, parameters(), ground_z_);
    *this = std::move(fresh);
  }
  void bind(BodyColumns &columns, size_t index) { body_.bind(columns, index); }
  void rebind(size_t index) { body_.rebind(index); }
  RigidBodyState body_state() const { return body_.state(); }
  bool setpoint(const MaskedPva &value) {
    const auto any = [](const Eigen::Vector3d &v) {
      return std::isfinite(v.x()) || std::isfinite(v.y()) || std::isfinite(v.z());
    };
    if (!value.receiver_valid ||
        !(any(value.position) || any(value.velocity) || any(value.acceleration)))
      return false;
    // The receiver heartbeat is independent of PositionControl's XY/per-axis
    // validity. An accepted incomplete setpoint is still a fresh stream.
    setpoint_ = value;
    attitude_control_ = false;
    received_since_step_ = true;
    return true;
  }
  bool attitude_setpoint(const FlightAttitudeSetpoint &value) {
    if (value.type_mask != 0 && value.type_mask != 7 && value.type_mask != 128)
      return false;
    if (!std::isfinite(value.thrust) ||
        (value.type_mask != 7 && !value.body_rate.allFinite()))
      return false;
    if (value.type_mask != 128) {
      const double norm = value.q.norm();
      if (!value.q.coeffs().allFinite() || !std::isfinite(norm) || norm < 1e-12)
        return false;
    }
    attitude_setpoint_ = value;
    if (value.type_mask != 128)
      attitude_setpoint_.q.normalize();
    attitude_control_ = true;
    received_since_step_ = true;
    return true;
  }
  bool request_arm(bool value, bool force = false) {
    if (!value && armed_ && !landed() && !force)
      return false;
    if (value != armed_)
      controller_.reset();
    armed_ = value;
    return true;
  }
  bool request_mode(FlightMode value) {
    if (value == FlightMode::Offboard && !stream_available())
      return false;
    mode_ = value;
    return true;
  }
  bool armed() const { return armed_; }
  FlightMode mode() const { return mode_; }
  // 1 cm is the gap from the support face to the world plane. A level body
  // rests with its base 0.033 m above ground_z, so base-versus-plane is not
  // this test.
  bool landed() const { return fs150SupportClearance(body_, ground_z_) <= kLandedHeight; }
  xgc2_math::TranslationalState state() const { return {body_.base_position(), body_.base_velocity()}; }
  const Eigen::Vector3d &acceleration() const { return acceleration_; }
  Eigen::Quaterniond orientation() const { return body_.state().orientation; }
  Eigen::Vector3d angular_velocity_body() const { return body_.state().angular_velocity; }
  Eigen::Vector4d rotor_speed() const { return body_.state().rotor_speed; }
  Eigen::Vector3d specific_force_body() const {
    return orientation().conjugate() *
           (acceleration_ - Eigen::Vector3d(0.0, 0.0, -body_.parameters().gravity));
  }
  double yaw() const {
    const auto r = orientation().toRotationMatrix();
    return std::atan2(r(1, 0), r(0, 0));
  }
  double yaw_rate() const { return yaw_rate_; }
  const FlightControlOutput &control_output() const { return control_output_; }
  const FlightControllerParameters &parameters() const { return controller_.parameters(); }

  // Inactive provider physics: keep the last motor target, without running
  // FCU control/integrals/stream hysteresis or changing armed/mode state.
  // Next world-thread start reconstructs this slot via resetToInitial().
  void stepPhysicsOnly(double dt) {
    validate_dt(dt);
    if (dt == 0.0)
      return;
    const double yaw_before = yaw();
    body_.step(control_output_.allocation.target_rotor_speed, dt);
    constrain_ground(dt);
    yaw_rate_ = xgc2_math::normalizeAngle(yaw() - yaw_before) / dt;
  }

  // One control update consumes the caller's complete elapsed interval.
  // The rigid-body plant alone subdivides for numerical integration safety.
  FlightEvent step(double dt) {
    validate_dt(dt);
    if (dt == 0.0)
      return FlightEvent::None;
    FlightEvent event = FlightEvent::None;
    const double yaw_before = yaw();
    // Commander v1.12.3 sets the true->false offboard-available hysteresis
    // directly to COM_OF_LOSS_T. Valid first input becomes available at once;
    // there is no one-second warmup or a second invented stream timeout.
    update_stream(dt);
    if (mode_ == FlightMode::Offboard && !offboard_available_) {
      mode_ = FlightMode::Hold;
      event = FlightEvent::OffboardLost;
    }
    const auto feedback = current_feedback();
    if (!armed_) {
      control_output_ = controller_.disarmed(dt);
    } else if (mode_ == FlightMode::Hold && landed()) {
      // Keep armed idle while waiting on the initial ground for a command.
      // This is the narrow service facade's ground gate, not a simulated
      // Commander/Takeoff state machine. Attitude and rate functions remain
      // the fixed source functions; no physical attitude is reset.
      control_output_ = controller_.command_attitude(
          feedback, orientation(), 0.0, Eigen::Vector3d::Zero(), 7, dt, true);
    } else if (mode_ == FlightMode::Offboard && attitude_control_) {
      const auto &raw = attitude_setpoint_;
      if (raw.type_mask == 128) {
        control_output_ = controller_.command_rate(feedback, raw.body_rate,
                                                   raw.thrust, dt, landed());
        control_output_.desired_orientation = raw.q; // ignored target payload
      } else {
        control_output_ = controller_.command_attitude(
            feedback, raw.q, raw.thrust, raw.body_rate, raw.type_mask, dt, landed());
      }
    } else {
      MaskedPva command;
      if (mode_ == FlightMode::Offboard) {
        command = setpoint_;
      } else {
        command.velocity.setZero();
        if (mode_ == FlightMode::Land)
          command.velocity.z() = -land_speed_;
      }
      control_output_ = controller_.command_pva(feedback, command, dt,
                                                landed(), mode_ == FlightMode::Land);
    }
    body_.step(control_output_.allocation.target_rotor_speed, dt);
    if (constrain_ground(dt)) {
      if (armed_ && mode_ == FlightMode::Land) {
        armed_ = false;
        controller_.reset();
        event = FlightEvent::Landed;
      }
    }
    yaw_rate_ = xgc2_math::normalizeAngle(yaw() - yaw_before) / dt;
    return event;
  }

private:
  static void validate_dt(double dt) {
    if (!std::isfinite(dt) || dt < 0.0 || dt > kMaxStep)
      throw std::invalid_argument("flight model: dt must be in [0,.02]");
  }
  bool constrain_ground(double dt) {
    const Eigen::Vector3d omega_before = body_.state().angular_velocity;
    const auto result = resolveFlatGroundContact(body_, ground_z_, dt);
    acceleration_ = result.base_acceleration_world;
    angular_acceleration_body_ = body_.angular_acceleration_body();
    if (result.valid && dt > 0.0)
      angular_acceleration_body_ += (body_.state().angular_velocity - omega_before) / dt;
    return result.valid && result.constrained;
  }
  FlightFeedback current_feedback() const {
    return {body_.base_position(), body_.base_velocity(), acceleration_, orientation(),
            angular_velocity_body(), angular_acceleration_body_};
  }
  bool stream_available() const {
    return received_since_step_ || offboard_available_;
  }
  void update_stream(double dt) {
    if (received_since_step_) {
      // Same true->false-only hysteresis as Commander::offboard_control_update.
      // No warmup on first valid receiver event. Loss delay begins at the
      // first FCU iteration without a new valid event, not at its last stamp.
      offboard_available_ = true;
      received_since_step_ = false;
      loss_pending_ = false;
      loss_age_ = 0.0;
    } else if (offboard_available_) {
      if (!loss_pending_) {
        loss_pending_ = true;
        loss_age_ = 0.0;
      }
      if (loss_age_ >= offboard_loss_time_ - 1e-9)
        offboard_available_ = false;
      loss_age_ += dt;
    }
  }

  RigidBodyModel body_;
  FlightController controller_;
  double ground_z_;
  Eigen::Vector3d initial_position_;
  double initial_yaw_;
  double land_speed_, offboard_loss_time_;
  MaskedPva setpoint_;
  FlightAttitudeSetpoint attitude_setpoint_;
  FlightControlOutput control_output_;
  Eigen::Vector3d acceleration_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_acceleration_body_{Eigen::Vector3d::Zero()};
  double loss_age_{0.0};
  double yaw_rate_{0.0};
  FlightMode mode_{FlightMode::Hold};
  bool received_since_step_{false};
  bool offboard_available_{false};
  bool loss_pending_{false};
  bool attitude_control_{false};
  bool armed_{false};
};

struct PlanarColumns {
  std::vector<Eigen::Vector2d> position;
  std::vector<double> yaw;
  size_t append(xgc2_math::Pose2 p) {
    size_t i=position.size(); position.push_back(p.position); yaw.push_back(p.yaw); return i;
  }
  xgc2_math::Pose2 get(size_t i) const { return {position[i], yaw[i]}; }
  void set(size_t i, xgc2_math::Pose2 p) { position[i]=p.position; yaw[i]=p.yaw; }
  void erase(size_t i) { set(i, get(position.size()-1)); position.pop_back(); yaw.pop_back(); }
};

class ScoutModel {
public:
  explicit ScoutModel(
      const xgc2_math::Pose2 &pose,
      const xgc2_math::DelayedPlanarVelocityParameters &response = {})
      : local_pose_(pose), response_(response) {}

  void command(double time, double forward, double yaw_rate) {
    // Same defaults as the accepted Gazebo unicycle plant; not wheel physics.
    response_.command(time, {std::clamp(forward, -1.5, 1.5),
                             std::clamp(yaw_rate, -1.0, 1.0)});
  }
  void advance(double end) {
    const double dt = end - response_.time();
    const auto before = response_.velocity();
    const auto middle = response_.advance(response_.time() + 0.5 * dt);
    set_pose(xgc2_math::stepBodyVelocity(pose(), {middle.linear_m_s, 0.0},
                                        middle.yaw_rad_s, dt));
    const auto after = response_.advance(end);
    if (dt > 0.0) {
      // The acceleration is from the realised actuator response, not its
      // command. The body frame turns as well as changing forward speed.
      acceleration_body_ = {(after.linear_m_s - before.linear_m_s) / dt,
                            after.yaw_rad_s * after.linear_m_s};
    }
  }
  xgc2_math::Pose2 pose() const { return columns_ ? columns_->get(index_) : *local_pose_; }
  void set_pose(xgc2_math::Pose2 p) { if(columns_) columns_->set(index_,p); else local_pose_=p; }
  void bind(PlanarColumns &columns, size_t i) {
    const auto p=pose(); columns_=&columns; index_=i; columns.set(i,p); local_pose_.reset();
  }
  void rebind(size_t i) { index_=i; }
  xgc2_math::PlanarVelocity velocity() const { return response_.velocity(); }
  Eigen::Vector3d specific_force_body() const {
    return {acceleration_body_.x(), acceleration_body_.y(), 9.8066};
  }

private:
  std::optional<xgc2_math::Pose2> local_pose_;
  PlanarColumns *columns_{nullptr};
  size_t index_{0};
  xgc2_math::DelayedPlanarVelocity response_;
  Eigen::Vector2d acceleration_body_{Eigen::Vector2d::Zero()};
};

class MecanumModel {
public:
  explicit MecanumModel(const xgc2_math::Pose2 &pose) : local_pose_(pose) {}

  void command(double forward, double left, double yaw_rate) {
    // Defaults of ugv_sim_single.launch, not unverified MCU/TEB limits.
    body_velocity_ = {std::clamp(forward, -1.5, 1.5),
                      std::clamp(left, -1.5, 1.5)};
    yaw_rate_ = std::clamp(yaw_rate, -1.5707963267948966, 1.5707963267948966);
  }
  void step(double dt) {
    set_pose(xgc2_math::stepBodyVelocity(pose(), body_velocity_, yaw_rate_, dt));
    if (dt > 0.0) {
      // Direct velocity actuation has an ideal jump on command changes. Its
      // finite-step acceleration uses the actual integration interval; a
      // constant body velocity also produces centripetal acceleration.
      acceleration_body_ = (body_velocity_ - integrated_body_velocity_) / dt +
          Eigen::Vector2d(-yaw_rate_ * body_velocity_.y(),
                           yaw_rate_ * body_velocity_.x());
      integrated_body_velocity_ = body_velocity_;
    }
  }
  Eigen::Vector3d specific_force_body() const {
    return {acceleration_body_.x(), acceleration_body_.y(), 9.8066};
  }
  xgc2_math::Pose2 pose() const { return columns_ ? columns_->get(index_) : *local_pose_; }
  void set_pose(xgc2_math::Pose2 p) { if(columns_) columns_->set(index_,p); else local_pose_=p; }
  void bind(PlanarColumns &columns, size_t i) {
    const auto p=pose(); columns_=&columns; index_=i; columns.set(i,p); local_pose_.reset();
  }
  void rebind(size_t i) { index_=i; }
  const Eigen::Vector2d &body_velocity() const { return body_velocity_; }
  double yaw_rate() const { return yaw_rate_; }

private:
  std::optional<xgc2_math::Pose2> local_pose_;
  PlanarColumns *columns_{nullptr};
  size_t index_{0};
  Eigen::Vector2d body_velocity_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d integrated_body_velocity_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d acceleration_body_{Eigen::Vector2d::Zero()};
  double yaw_rate_{0.0};
};

} // namespace xgc_lightweight
