#include "vehicle_model.hpp"
#include "systems/robots.hpp"

#include <cassert>
#include <iostream>
#include <vector>

using namespace xgc_lightweight;

namespace {
MaskedPva position_setpoint(const Eigen::Vector3d &position) {
  MaskedPva value;
  value.position = position;
  value.receiver_valid = true;
  return value;
}
MaskedPva acceleration_setpoint(const Eigen::Vector3d &acceleration) {
  MaskedPva value;
  value.acceleration = acceleration;
  value.receiver_valid = true;
  return value;
}
void take_off(FlightModel &model, const MaskedPva &target) {
  assert(model.setpoint(target));
  assert(model.request_arm(true));
  assert(model.request_mode(FlightMode::Offboard));
}
void fly(FlightModel &model, const MaskedPva &target, int steps, double dt = 0.001) {
  for (int i = 0; i != steps; ++i) {
    assert(model.setpoint(target));
    assert(model.step(dt) == FlightEvent::None);
  }
}
// Long scenarios explicitly schedule their model calls; FlightModel does not.
void advance_ms(FlightModel &model, int milliseconds, bool physics_only = false) {
  for (int i = 0; i != milliseconds; ++i) {
    if (physics_only)
      model.stepPhysicsOnly(0.001);
    else
      model.step(0.001);
  }
}
void fly_raw(FlightModel &model, const FlightAttitudeSetpoint &target, int steps) {
  for (int i = 0; i != steps; ++i) {
    assert(model.attitude_setpoint(target));
    assert(model.step(0.001) == FlightEvent::None);
  }
}
}

int main() {
  // World dense vectors may move their models: mixer callback storage and
  // original integral/motor histories must remain attached to each instance.
  std::vector<FlightModel> models;
  models.reserve(1);
  models.emplace_back(Eigen::Vector3d(0, 0, 0.15), 0.25, FlightControllerParameters{}, 0.15);
  models.emplace_back(Eigen::Vector3d(0, 0, 0));
  auto &model = models[0];
  auto hover = position_setpoint({0.6, -0.4, 2.15});
  assert(model.setpoint(hover));
  advance_ms(model, 100);
  assert((model.state().position - Eigen::Vector3d(0, 0, 0.15)).norm() == 0.0);
  assert(model.angular_velocity_body().norm() == 0.0);
  assert((model.specific_force_body() - Eigen::Vector3d(0, 0, 9.8066)).norm() < 1e-6);
  assert(model.request_arm(true));
  advance_ms(model, 300); // armed hold on initial ground is idle, not a takeoff command
  assert(std::abs(model.state().position.z() - 0.15) < 1e-12);
  assert(model.control_output().normalized_thrust == 0.0);
  assert(model.setpoint(hover));
  assert(model.request_mode(FlightMode::Offboard)); // no one-second warmup
  fly(model, hover, 10);
  assert(model.orientation().angularDistance(model.control_output().desired_orientation) > 0.01);
  assert(model.orientation().angularDistance(Eigen::Quaterniond(Eigen::AngleAxisd(
             0.25, Eigen::Vector3d::UnitZ()))) < 0.02);
  fly(model, hover, 9990);
  const double hover_error = (model.state().position - hover.position).norm();
  assert(hover_error < 0.02);
  assert(model.state().velocity.norm() < 0.02);
  assert(model.control_output().position_control_valid);
  assert(!model.landed());
  assert(!model.request_arm(false));
  assert((model.orientation() * model.specific_force_body() -
          (model.acceleration() + Eigen::Vector3d(0, 0, 9.8066))).norm() < 1e-6);
  const double realized_hover = model.control_output().normalized_thrust;

  // A yaw rate is realized by the original attitude/rate/mixer/motor chain.
  // The instantaneous quaternion never equals a newly commanded yaw target.
  auto turning = hover;
  turning.yaw_rate = 0.5;
  fly(model, turning, 4000);
  assert(std::abs(model.yaw_rate() - 0.5) < 0.03);
  turning.yaw = -1.0;
  turning.yaw_rate = std::numeric_limits<double>::quiet_NaN();
  const double old_yaw = model.yaw();
  fly(model, turning, 1);
  assert(std::abs(xgc2_math::normalizeAngle(model.yaw() - old_yaw)) < 0.002);
  fly(model, turning, 6000);
  assert(std::abs(xgc2_math::normalizeAngle(model.yaw() + 1.0)) < 0.02);

  assert(model.request_mode(FlightMode::Land));
  for (int i = 0; i != 2000; ++i)
    assert(model.step(0.001) == FlightEvent::None);
  assert(std::abs(model.state().velocity.z() + model.parameters().get("MPC_LAND_SPEED")) < 0.08);
  int landed_at = -1;
  double impact = 0.0;
  for (int i = 0; i != 8000 && landed_at < 0; ++i)
    if (model.step(0.001) == FlightEvent::Landed) {
      landed_at = i;
      impact = model.specific_force_body().norm();
    }
  assert(landed_at >= 0 && !model.armed() && model.landed());
  assert(impact > 100.0);
  assert(std::abs(model.state().position.z() - 0.15) < 1e-12);
  assert(model.state().velocity.z() >= -1e-12);
  advance_ms(model, 100);
  assert(std::abs(model.state().position.z() - 0.15) < 1e-12);

  // Decoder reception and PositionControl admissibility are independent.
  PositionTarget wire{};
  wire.coordinate_frame = 1;
  wire.type_mask = 2u | 56u | 448u | 1024u | 2048u;
  wire.position[0] = 1; wire.position[1] = 2; wire.position[2] = 3;
  FlightModel partial({0, 0, 0});
  const auto decoded = decodePositionTarget(wire, partial.orientation(), partial.yaw());
  assert(decoded.receiver_valid && std::isnan(decoded.position.x()) && decoded.position.y() == 2);
  take_off(partial, decoded);
  for (int i = 0; i != 1200; ++i) {
    assert(partial.setpoint(decoded));
    assert(partial.step(0.001) == FlightEvent::None);
    assert(!partial.control_output().position_control_valid);
    assert(partial.mode() == FlightMode::Offboard);
    assert(partial.state().position.allFinite());
  }

  FlightControllerParameters timeout;
  assert(timeout.set("COM_OF_LOSS_T", 0.05));
  FlightModel stream({0, 0, 0}, 0, timeout);
  assert(!stream.request_mode(FlightMode::Offboard));
  take_off(stream, position_setpoint({0, 0, 1}));
  int lost_at = -1;
  for (int i = 0; i != 1000; ++i)
    if (stream.step(0.001) == FlightEvent::OffboardLost) {
      assert(lost_at < 0);
      lost_at = i;
    }
  assert(lost_at == 51); // source false-input hysteresis starts on first missing iteration
  assert(stream.mode() == FlightMode::Hold && stream.armed());
  for (int i = 0; i != 20000; ++i)
    assert(stream.step(0.001) == FlightEvent::None);
  assert(stream.state().velocity.norm() < 0.02);
  FlightControllerParameters immediate;
  assert(immediate.set("COM_OF_LOSS_T", 0));
  FlightModel no_delay({0, 0, 0}, 0, immediate);
  take_off(no_delay, position_setpoint({0, 0, 1}));
  assert(no_delay.step(0.001) == FlightEvent::None);
  assert(no_delay.step(0.001) == FlightEvent::OffboardLost);

  // Explicit, physical-calibration profile for ACC-only/SMC tests. This is
  // an authorized startup override, not FS export or a gain modification.
  const auto physical = fs150EquivalentParameters();
  const double hover_motor =
      (std::sqrt(physical.mass * physical.gravity / (4 * physical.thrust_coefficient)) -
       FlightController::kMotorArmedZero) / FlightController::kMotorScaling;
  FlightControllerParameters calibrated;
  assert(calibrated.set("MPC_THR_HOVER", hover_motor));
  FlightModel raw_model({0, 0, 0}, 0, calibrated);
  const auto altitude = position_setpoint({0, 0, 3});
  take_off(raw_model, altitude);
  fly(raw_model, altitude, 8000);
  FlightAttitudeSetpoint raw;
  raw.q.coeffs().setConstant(std::numeric_limits<double>::quiet_NaN());
  raw.thrust = hover_motor;
  fly_raw(raw_model, raw, 2000);
  assert(raw_model.control_output().type_mask == 128);
  assert(raw_model.orientation().coeffs().allFinite());
  assert(std::abs(raw_model.state().position.z() - 3.0) < 0.03);
  raw.body_rate.y() = 0.25;
  fly_raw(raw_model, raw, 400);
  raw.body_rate.setZero();
  fly_raw(raw_model, raw, 1200);
  const auto biased = raw_model.orientation();
  assert(biased.angularDistance(Eigen::Quaterniond::Identity()) > 0.05);
  fly_raw(raw_model, raw, 500);
  assert(raw_model.orientation().angularDistance(biased) < 0.006);
  assert(raw_model.control_output().desired_body_rate.norm() == 0.0);
  raw.type_mask = 7;
  raw.q = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ());
  raw.body_rate.setConstant(std::numeric_limits<double>::quiet_NaN());
  fly_raw(raw_model, raw, 4000);
  assert(raw_model.orientation().angularDistance(raw.q) < 0.01);
  raw.type_mask = 0;
  raw.q = raw_model.orientation(); raw.body_rate = {0.1, 0.2, 0.3};
  fly_raw(raw_model, raw, 1);
  assert((raw_model.control_output().desired_body_rate - raw.body_rate).norm() < 1e-5);

  const auto recover = position_setpoint(raw_model.state().position + Eigen::Vector3d(0, 0, 0.5));
  fly(raw_model, recover, 20000);
  assert(raw_model.control_output().type_mask == 7);
  std::cout << "raw->PVA recovery error " << (raw_model.state().position - recover.position).norm()
            << " velocity " << raw_model.state().velocity.norm() << "\n";
  assert((raw_model.state().position - recover.position).norm() < 0.03);
  raw.type_mask = 128; raw.body_rate.setZero(); raw.thrust = 0.0;
  fly_raw(raw_model, raw, 120);
  const double idle_force = 4 * physical.thrust_coefficient * 100 * 100 / physical.mass;
  assert(std::abs(raw_model.specific_force_body().norm() - idle_force) < 0.015);
  assert(raw_model.acceleration().z() < -9.0); // armed idle; not zero-speed freefall
  raw.thrust = 2.0;
  fly_raw(raw_model, raw, 100);
  assert(raw_model.control_output().allocation.saturated());
  assert(raw_model.control_output().normalized_thrust <= 1.0000001);
  assert(raw_model.acceleration().z() > 70.0);
  assert(raw_model.state().position.allFinite());
  assert(raw_model.request_mode(FlightMode::Land));
  raw_model.step(0.001);
  assert(raw_model.control_output().type_mask == 7);
  landed_at = -1;
  for (int i = 0; i != 25000 && landed_at < 0; ++i)
    if (raw_model.step(0.001) == FlightEvent::Landed) landed_at = i;
  assert(landed_at >= 0 && !raw_model.armed());

  // ACC-only does not see internal P/V state feedback, but it remains a
  // finite-bandwidth physical actuator, not an ideal double integrator.
  FlightModel acceleration_model({0, 0, 0}, 0, calibrated);
  take_off(acceleration_model, altitude);
  fly(acceleration_model, altitude, 8000);
  auto acc = acceleration_setpoint({2.0, -0.5, 0.2});
  fly(acceleration_model, acc, 1);
  assert((acceleration_model.control_output().acceleration_command - acc.acceleration).norm() < 1e-6);
  assert(acceleration_model.orientation().angularDistance(
             acceleration_model.control_output().desired_orientation) > 0.05);
  assert(std::abs(acceleration_model.acceleration().x()) < 0.1);
  fly(acceleration_model, acc, 1000);
  assert(acceleration_model.state().velocity.x() > 1.0);
  assert(acceleration_model.acceleration().x() > 1.5);

  // A refused raw packet must not replace a valid PVA/raw source or renew
  // the receiver event consumed by the offboard hysteresis.
  FlightModel refused({0, 0, 0}, 0, timeout);
  raw.q.coeffs().setConstant(std::numeric_limits<double>::quiet_NaN());
  raw.type_mask = 128; raw.thrust = hover_motor;
  assert(refused.attitude_setpoint(raw));
  assert(refused.request_arm(true) && refused.request_mode(FlightMode::Offboard));
  advance_ms(refused, 49);
  for (const uint32_t mask : {1u, 2u, 3u, 4u, 5u, 6u, 64u, 135u, 255u}) {
    auto bad = raw; bad.type_mask = mask; bad.thrust = 1.0;
    assert(!refused.attitude_setpoint(bad));
  }
  auto bad = raw; bad.type_mask = 7;
  assert(!refused.attitude_setpoint(bad));
  bad = raw; bad.body_rate.x() = std::numeric_limits<double>::quiet_NaN();
  assert(!refused.attitude_setpoint(bad));
  assert(refused.step(0.001) == FlightEvent::None);
  assert(refused.step(0.001) == FlightEvent::None);
  assert(refused.step(0.001) == FlightEvent::OffboardLost);

  std::cout << "PX4/FS physical facade: hover_error=" << hover_error
            << " actual_hover_target=" << realized_hover
            << " calibrated_profile=" << hover_motor << " impact=" << impact << "\n";

  // The independent world plane does not follow a robot's initial altitude.
  // Reconnect resets one model by move assignment, preserving the world plane
  // while clearing armed/mode/command/motor/PID/mixer histories.
  FlightModel airborne({0.2, -0.3, 2.0}, 0.7, calibrated, 0.0);
  assert(!airborne.landed());
  assert(std::abs(airborne.acceleration().z() + physical.gravity) < 1e-12);
  assert(airborne.specific_force_body().norm() < 1e-12);
  advance_ms(airborne, 100);
  assert(airborne.state().position.z() < 2.0 && airborne.state().position.z() > 1.9);
  airborne = FlightModel({0.4, 0.5, 1.0}, -0.4, calibrated, 0.0);
  assert(!airborne.armed() && airborne.mode() == FlightMode::Hold);
  assert(!airborne.request_mode(FlightMode::Offboard));

  FlightModel stopped_slot({0, 0, 0}, 0, calibrated, 0);
  take_off(stopped_slot, altitude);
  fly(stopped_slot, altitude, 8000);
  auto retained = raw;
  retained.q.coeffs().setConstant(std::numeric_limits<double>::quiet_NaN());
  retained.type_mask = 128; retained.body_rate.setZero(); retained.thrust = 1.0;
  fly_raw(stopped_slot, retained, 1);
  const auto target_before_stop = stopped_slot.control_output();
  const auto pose_before_stop = stopped_slot.state().position;
  advance_ms(stopped_slot, 100, true);
  assert(stopped_slot.armed() && stopped_slot.mode() == FlightMode::Offboard);
  assert((stopped_slot.control_output().allocation.target_rotor_speed -
          target_before_stop.allocation.target_rotor_speed).norm() == 0.0);
  assert(stopped_slot.control_output().normalized_thrust == target_before_stop.normalized_thrust);
  assert((stopped_slot.state().position - pose_before_stop).norm() > 0.05);
  stopped_slot.resetToInitial();
  assert(!stopped_slot.armed() && stopped_slot.mode() == FlightMode::Hold);
  assert(stopped_slot.state().position.norm() == 0.0);
  assert(stopped_slot.control_output().allocation.target_rotor_speed.norm() == 0.0);
  assert((airborne.state().position - Eigen::Vector3d(0.4, 0.5, 1.0)).norm() < 1e-12);
  assert(airborne.state().velocity.norm() == 0.0);
  assert(airborne.angular_velocity_body().norm() == 0.0);
  assert(airborne.control_output().allocation.target_rotor_speed.norm() == 0.0);
  advance_ms(airborne, 600);
  assert(std::abs(airborne.state().position.z()) < 1e-12);
  assert(airborne.landed() && airborne.acceleration().norm() < 1e-8);
  const auto new_target = position_setpoint({0.4, 0.5, 1.0});
  take_off(airborne, new_target);
  fly(airborne, new_target, 10000);
  assert((airborne.state().position - new_target.position).norm() < 0.02);
  airborne.resetToInitial();
  assert(!airborne.armed() && airborne.mode() == FlightMode::Hold);
  assert((airborne.state().position - Eigen::Vector3d(0.4, 0.5, 1.0)).norm() < 1e-12);
  assert(airborne.state().velocity.norm() == 0.0);
  assert(!airborne.request_mode(FlightMode::Offboard));

  // Shared elapsed dt reaches the FCU once, including an overload interval.
  // Compare its public output with one direct controller update and the pure
  // plant; an implicit 1 ms control loop would consume intermediate feedback.
  FlightModel elapsed_model({0, 0, 1}, 0, calibrated);
  FlightAttitudeSetpoint elapsed_raw;
  elapsed_raw.body_rate = {0.3, -0.2, 0.1};
  elapsed_raw.thrust = hover_motor;
  assert(elapsed_model.attitude_setpoint(elapsed_raw));
  assert(elapsed_model.request_arm(true));
  assert(elapsed_model.request_mode(FlightMode::Offboard));
  FlightController direct_controller(physical, calibrated);
  const FlightFeedback initial_feedback{elapsed_model.state().position,
      elapsed_model.state().velocity, elapsed_model.acceleration(),
      elapsed_model.orientation(), elapsed_model.angular_velocity_body(),
      Eigen::Vector3d::Zero()};
  const auto direct = direct_controller.command_rate(initial_feedback,
      elapsed_raw.body_rate, elapsed_raw.thrust, 0.01, false);
  RigidBodyModel direct_body({0, 0, 1}, 0, physical);
  direct_body.step(direct.allocation.target_rotor_speed, 0.01);
  assert(elapsed_model.step(0.01) == FlightEvent::None);
  assert((elapsed_model.control_output().allocation.target_rotor_speed -
          direct.allocation.target_rotor_speed).norm() < 1e-12);
  assert((elapsed_model.state().position - direct_body.base_position()).norm() < 1e-12);
  assert(elapsed_model.orientation().angularDistance(direct_body.state().orientation) < 1e-12);
  const auto before_invalid = elapsed_model.state();
  for (double invalid : {-0.001, 0.020001,
                         std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()}) {
    bool rejected = false;
    try { elapsed_model.step(invalid); }
    catch (const std::invalid_argument &) { rejected = true; }
    assert(rejected);
    assert((elapsed_model.state().position - before_invalid.position).norm() == 0);
  }

  FlightModel variable_hover({0, 0, 0});
  const auto variable_target = position_setpoint({0.3, -0.2, 1.0});
  take_off(variable_hover, variable_target);
  for (int i = 0; i != 500; ++i)
    for (const double dt : {0.002, 0.004, 0.008, 0.01}) {
      assert(variable_hover.setpoint(variable_target));
      assert(variable_hover.step(dt) == FlightEvent::None);
    }
  const double variable_error = (variable_hover.state().position - variable_target.position).norm();
  std::cout << "variable-dt hover error " << variable_error << "\n";
  assert(variable_error < 0.03);
  assert(variable_hover.state().velocity.norm() < 0.03);

  xsim::Config scout_config;
  scout_config.kind = xsim::Kind::Scout;
  xsim::Scout variable_scout(scout_config);
  variable_scout.model.command(variable_scout.age, 1.0, 0.4);
  for (const int64_t dt : {2000000, 6000000, 10000000})
    xsim::step_robot(variable_scout, dt);
  assert(variable_scout.age_ns == 18000000);
  assert(std::abs(variable_scout.age - 0.018) < 1e-15);
  const double response = 1.0 - std::exp(-0.013 / 0.005);
  assert(std::abs(variable_scout.model.velocity().linear_m_s - response) < 1e-12);
  assert(std::abs(variable_scout.model.velocity().yaw_rad_s - 0.4 * response) < 1e-12);
  bool rejected = false;
  try { xsim::step_robot(variable_scout, -1); }
  catch (const std::invalid_argument &) { rejected = true; }
  assert(rejected && variable_scout.age_ns == 18000000);

  ScoutModel scout({{0.0, 0.0}, 0.0});
  scout.command(0.0, 1.0, 0.0);
  scout.advance(0.004);
  assert(scout.pose().position.norm() == 0.0);
  for (int i = 0; i != 996; ++i)
    scout.advance(double(i + 5) * 0.001);
  const double expected = 0.995 - 0.005 * (1.0 - std::exp(-0.995 / 0.005));
  assert(std::abs(scout.pose().position.x() - expected) < 1e-5);
  assert(scout.pose().position.y() == 0.0);
  // Converge after stop with the same delayed velocity response as Gazebo.
  scout.command(1.0, 0.0, 0.0);
  for (int i = 0; i != 100; ++i)
    scout.advance(double(i + 1001) * 0.001);
  assert(scout.velocity().linear_m_s < 1e-7);

  MecanumModel sideways({{0.0, 0.0}, 1.5707963267948966});
  sideways.command(0.0, 1.0, 0.0);
  sideways.step(2.0);
  assert((sideways.pose().position - Eigen::Vector2d(-2.0, 0.0)).norm() <
         1e-12);
  MecanumModel arc({{0.0, 0.0}, 0.0});
  arc.command(1.0, 0.5, 1.0);
  arc.step(6.283185307179586);
  assert(arc.pose().position.norm() < 1e-12);

  // IMU truth is generated at each model's actual integration interval. A
  // resting planar body feels gravity; forward response, braking and turning
  // must change specific force even when ROS samples it at a lower rate.
  ScoutModel imu_scout({{0, 0}, 0.7});
  assert((imu_scout.specific_force_body() - Eigen::Vector3d(0, 0, physical.gravity)).norm() < 1e-12);
  imu_scout.command(0, 0.6, 0.5);
  imu_scout.advance(.004); // delayed command has not reached the plant
  assert(imu_scout.specific_force_body().head<2>().norm() == 0);
  imu_scout.advance(.008); // actual 4 ms interval, 3 ms of actuator response
  const double response_at_8ms = 1 - std::exp(-.003 / .005);
  assert(std::abs(imu_scout.specific_force_body().x() - .6 * response_at_8ms / .004) < 1e-10);
  assert(std::abs(imu_scout.velocity().yaw_rad_s - .5 * response_at_8ms) < 1e-12);
  assert(std::abs(imu_scout.specific_force_body().y() - .3 * response_at_8ms * response_at_8ms) < 1e-12);
  imu_scout.advance(.015); // next interval is 7 ms, not an assumed 2 ms
  const double response_at_15ms = 1 - std::exp(-.01 / .005);
  assert(std::abs(imu_scout.specific_force_body().x() - .6 * (response_at_15ms - response_at_8ms) / .007) < 1e-10);
  imu_scout.command(.015, 0, 0);
  imu_scout.advance(.021);
  assert(imu_scout.specific_force_body().x() < 0);
  imu_scout.advance(.22);
  imu_scout.advance(.23);
  assert(imu_scout.specific_force_body().head<2>().norm() < 1e-6);
  assert(imu_scout.specific_force_body().z() == physical.gravity);

  MecanumModel imu_mecanum({{0, 0}, -0.9});
  assert((imu_mecanum.specific_force_body() - Eigen::Vector3d(0, 0, physical.gravity)).norm() < 1e-12);
  imu_mecanum.command(.4, 0, .6);
  imu_mecanum.step(.002);
  assert(std::abs(imu_mecanum.specific_force_body().x() - 200) < 1e-12);
  imu_mecanum.step(.01);
  assert(std::abs(imu_mecanum.specific_force_body().x()) < 1e-12);
  assert(std::abs(imu_mecanum.specific_force_body().y() - .24) < 1e-12);
  assert(imu_mecanum.yaw_rate() == .6);
  imu_mecanum.command(0, 0, 0);
  imu_mecanum.step(.004);
  assert(std::abs(imu_mecanum.specific_force_body().x() + 100) < 1e-12);
  imu_mecanum.step(.006);
  assert((imu_mecanum.specific_force_body() - Eigen::Vector3d(0, 0, physical.gravity)).norm() < 1e-12);
  std::cout << "lightweight six-DOF FCU and unchanged ground plants: passed\n";
}
