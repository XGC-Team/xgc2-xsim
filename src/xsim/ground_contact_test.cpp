#include "vehicle_model.hpp"

#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace xgc_lightweight;

namespace {
void require(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}

double energy(const RigidBodyState &state, const RigidBodyParameters &parameters) {
  return 0.5 * parameters.mass * state.velocity.squaredNorm() +
         0.5 * state.angular_velocity.dot(parameters.inertia * state.angular_velocity);
}

std::string read_text(const char *path) {
  std::ifstream input(path);
  if (!input)
    throw std::runtime_error("FS150 SDF is not readable");
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

double number_after(const std::string &text, const std::string &key, std::size_t from) {
  const auto at = text.find(key, from);
  require(at != std::string::npos, "SDF field missing");
  return std::stod(text.substr(at + key.size()));
}

void require_declared_support_face() {
#ifndef FS150_IRIS_SDF
  throw std::runtime_error("FS150_IRIS_SDF is not set");
#else
  const std::string text = read_text(FS150_IRIS_SDF);
  const auto box = text.find("base_link_inertia_collision");
  require(box != std::string::npos, "body collision missing");
  const auto box_end = text.find("</collision>", box);
  const std::string collision = text.substr(box, box_end - box);
  require(collision.find("<size>0.47 0.47 0.066</size>") != std::string::npos,
          "support box changed");
  require(collision.find("<pose>0 0 0 0 0 0</pose>") != std::string::npos,
          "support box pose changed");
  require(collision.find("<min_depth>0.001</min_depth>") != std::string::npos,
          "declared min_depth changed");
  const auto friction = collision.find("<friction>");
  const auto friction_end = collision.find("</friction>", friction);
  const std::string ode = collision.substr(friction, friction_end - friction);
  require(ode.find("<mu>") == std::string::npos && ode.find("mu2") == std::string::npos,
          "SDF now declares a numeric friction coefficient");
  require(std::abs(Fs150SupportFace::kFaceZ + 0.033) < 1e-15, "face z is half the box");
  require(Fs150SupportFace::kMu == 1.0 && Fs150SupportFace::kMinDepth == 0.001,
          "solver coefficients drifted from the empty ODE element and min_depth");

  double lowest = 1e9;
  std::size_t link = 0;
  while ((link = text.find("<link ", link)) != std::string::npos) {
    const auto link_end = text.find("<link ", link + 6);
    const std::string body = text.substr(link, (link_end == std::string::npos ? text.size() : link_end) - link);
    const double link_z = number_after(body, "<pose>", 0);
    // pose is "x y z roll pitch yaw"; number_after stops at the first value.
    // Re-read the third value.
    const auto pose_at = body.find("<pose>");
    double x = 0, y = 0, z = 0;
    std::sscanf(body.c_str() + pose_at + 6, "%lf %lf %lf", &x, &y, &z);
    (void)link_z;
    std::size_t collision_at = 0;
    while ((collision_at = body.find("<collision", collision_at)) != std::string::npos) {
      const auto collision_end = body.find("</collision>", collision_at);
      const std::string shape = body.substr(collision_at, collision_end - collision_at);
      double cx = 0, cy = 0, cz = 0;
      const auto cpose = shape.find("<pose>");
      if (cpose != std::string::npos)
        std::sscanf(shape.c_str() + cpose + 6, "%lf %lf %lf", &cx, &cy, &cz);
      double bottom = 1e9;
      const auto size_at = shape.find("<size>");
      const auto length_at = shape.find("<length>");
      if (size_at != std::string::npos) {
        double sx = 0, sy = 0, sz = 0;
        std::sscanf(shape.c_str() + size_at + 6, "%lf %lf %lf", &sx, &sy, &sz);
        bottom = z + cz - 0.5 * sz;
      } else if (length_at != std::string::npos) {
        const double length = std::stod(shape.substr(length_at + 8));
        bottom = z + cz - 0.5 * length;
      }
      lowest = std::min(lowest, bottom);
      collision_at = collision_end;
    }
    link = link + 6;
  }
  require(std::abs(lowest - Fs150SupportFace::kFaceZ) < 1e-12,
          "lowest SDF collision is not the body-box bottom face");
#endif
}

double flight_clearance(const FlightModel &model) {
  RigidBodyModel sample({0, 0, 0});
  sample.set_state(model.body_state());
  return fs150SupportClearance(sample, 0);
}

double tilt_magnitude(const RigidBodyModel &model) {
  const Eigen::Matrix3d rotation = model.state().orientation.toRotationMatrix();
  const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
  const double pitch = std::atan2(-rotation(2, 0), std::hypot(rotation(2, 1), rotation(2, 2)));
  return std::max(std::abs(roll), std::abs(pitch));
}

void settle(RigidBodyModel &model, double seconds, double dt, double ground, double *peak_tilt = nullptr) {
  const auto parameters = model.parameters();
  for (double t = 0; t < seconds; t += dt) {
    const double before = energy(model.state(), parameters);
    model.step(Eigen::Vector4d::Zero(), dt);
    const double fallen = energy(model.state(), parameters);
    const auto contact = resolveFlatGroundContact(model, ground, dt);
    require(contact.valid, "contact step invalid");
    require(energy(model.state(), parameters) <= fallen + 1e-9,
            "contact increased kinetic energy");
    require(before > -1.0, "energy baseline");
    if (peak_tilt != nullptr)
      *peak_tilt = std::max(*peak_tilt, tilt_magnitude(model));
  }
}
} // namespace

int main() {
  try {
    require_declared_support_face();
    const double rest = Fs150SupportFace::kLevelBaseHeight;

    auto parameters = fs150EquivalentParameters();
    parameters.body_origin_to_com = {.2, 0, .3};
    RigidBodyModel tilted({0, 0, 0}, 0, parameters);
    auto state = tilted.state();
    state.orientation = Eigen::AngleAxisd(.6, Eigen::Vector3d::UnitY());
    state.position = state.orientation * parameters.body_origin_to_com +
                     Eigen::Vector3d(0, 0, -.02);
    state.angular_velocity = {0, -20, 0};
    tilted.set_state(state);
    const double before = energy(state, parameters);
    const auto contact = resolveFlatGroundContact(tilted, 0, .001);
    const double after = energy(tilted.state(), parameters);
    require(contact.valid && contact.constrained && contact.normal_impulse > 0,
            "off-centre support impulse");
    double lowest = 1e9;
    for (int i = 0; i < Fs150SupportFace::kCorners; ++i) {
      const Eigen::Vector3d r = Fs150SupportFace::corner(i) - parameters.body_origin_to_com;
      lowest = std::min(lowest, (tilted.state().position + tilted.state().orientation * r).z());
    }
    require(std::abs(lowest) < 1e-9, "lowest corner on the world plane");
    require(std::abs(tilted.base_position().z() - rest) > 1e-4,
            "tilted measurement origin is not the level rest height");
    require((tilted.state().angular_velocity - state.angular_velocity).norm() > 0,
            "contact must change angular momentum");
    require(after <= before + 1e-9, "inelastic support impulse must not add kinetic energy");
    require(tilted.state().orientation.angularDistance(state.orientation) < 1e-12,
            "contact cannot snap attitude");

    RigidBodyModel low({0, 0, 1}), high({0, 0, 2});
    for (int i = 0; i < 400; ++i) {
      low.step(Eigen::Vector4d::Zero(), .01);
      resolveFlatGroundContact(low, 0, .01);
      high.step(Eigen::Vector4d::Zero(), .01);
      resolveFlatGroundContact(high, 0, .01);
    }
    require(std::abs(low.base_position().z() - rest) < 1e-8 &&
                std::abs(high.base_position().z() - rest) < 1e-8,
            "level base rests 0.033 m above the world plane");
    require((low.base_position().head<2>().norm() < 1e-8) &&
                (high.base_position().head<2>().norm() < 1e-8),
            "level drop does not walk");
    const auto resting = resolveFlatGroundContact(high, 0);
    require(std::abs(resting.base_acceleration_world.z()) < 1e-6, "resting acceleration zero");
    require(resting.constrained && high.base_velocity().norm() < 1e-8, "resting velocity zero");

    RigidBodyModel leaving({0, 0, rest});
    auto up = leaving.state();
    up.velocity.z() = 1;
    leaving.set_state(up);
    const auto released = resolveFlatGroundContact(leaving, 0);
    require(!released.constrained && released.normal_impulse == 0 &&
                released.base_acceleration_world.z() < 0,
            "separating support is not glued to ground");

    RigidBodyModel rising({0, 0, rest + 0.5 * Fs150SupportFace::kMinDepth});
    auto climb = rising.state();
    climb.velocity.z() = 0.2;
    rising.set_state(climb);
    const double gap = fs150SupportClearance(rising, 0);
    require(gap > 0.0 && gap <= Fs150SupportFace::kMinDepth,
            "upward case is inside the solver tolerance and above the plane");
    const auto open = resolveFlatGroundContact(rising, 0, 0.002);
    require(!open.constrained && open.normal_impulse == 0.0,
            "separating tolerance band is not a contact");
    require(rising.base_position().z() > rest, "separating band is not pulled onto the plane");

    FlightModel departure({0, 0, rest});
    require(departure.request_arm(true), "arm climb");
    FlightAttitudeSetpoint thrust;
    thrust.type_mask = 128;
    thrust.thrust = 1.0;
    require(departure.attitude_setpoint(thrust) && departure.request_mode(FlightMode::Offboard),
            "climb offboard");
    bool inside = false;
    for (int i = 0; i < 40 && !inside; ++i) {
      require(departure.attitude_setpoint(thrust), "climb thrust");
      require(departure.step(0.001) == FlightEvent::None, "climb step");
      const double clearance = flight_clearance(departure);
      inside = departure.state().velocity.z() > 0.0 && clearance > 0.0 &&
               clearance <= Fs150SupportFace::kMinDepth;
    }
    require(inside, "climb did not cross the 1 mm band with upward speed");
    require(departure.request_mode(FlightMode::Land), "switch to land while climbing");
    require(departure.step(0.001) != FlightEvent::Landed && departure.armed(),
            "armed Land disarmed on separating motion inside the tolerance");

    RigidBodyModel yawed({0, 0, rest});
    auto spinning = yawed.state();
    spinning.angular_velocity = {0, 0, 1};
    yawed.set_state(spinning);
    const double yaw_before = 1.0;
    settle(yawed, 2.0, 0.002, 0);
    require(std::abs(yawed.state().angular_velocity.z()) < 0.05 * yaw_before,
            "ground friction must dissipate yaw");
    require(yawed.state().angular_velocity.head<2>().norm() < 1e-3, "yaw friction stays level");
    require(std::abs(yawed.base_position().z() - rest) < 1e-6, "yaw keeps the 0.033 m base height");

    RigidBodyModel rocked({0, 0, 0.05});
    auto tilt = rocked.state();
    tilt.orientation = Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitX());
    tilt.angular_velocity = tilt.orientation.conjugate() * Eigen::Vector3d(0, 0, 0.8);
    const Eigen::Vector3d com = rocked.parameters().body_origin_to_com;
    double min_corner = 1e9;
    for (int i = 0; i < Fs150SupportFace::kCorners; ++i) {
      const Eigen::Vector3d r = Fs150SupportFace::corner(i) - com;
      min_corner = std::min(min_corner, (tilt.orientation * r).z());
    }
    tilt.position = Eigen::Vector3d(0, 0, 0.02 - min_corner);
    rocked.set_state(tilt);
    double max_tilt = 0.02;
    settle(rocked, 10.0, 0.002, 0, &max_tilt);
    const Eigen::Matrix3d rotation = rocked.state().orientation.toRotationMatrix();
    const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
    const double pitch = std::atan2(-rotation(2, 0), std::hypot(rotation(2, 1), rotation(2, 2)));
    require(std::abs(roll) < 0.05 && std::abs(pitch) < 0.05, "tiny tilt and yaw must not grow to O(1)");
    require(rocked.state().angular_velocity.norm() < 0.05, "support must dissipate the landing rate");
    require(max_tilt < 0.2, "tilt bound");
    require(std::abs(rocked.base_position().z() - rest) < 0.02,
            "settled base stays near ground_z + 0.033");

    // FlightModel integrates the rigid interval first. The endpoint constraint
    // then reports that interval's support acceleration.
    rocked.step(Eigen::Vector4d::Zero(), 0.002);
    const auto imu = resolveFlatGroundContact(rocked, 0, 0.002);
    require(imu.valid, "settled imu sample");
    const Eigen::Vector3d specific = rocked.state().orientation.conjugate() *
                                     (imu.base_acceleration_world - Eigen::Vector3d(0, 0, -rocked.parameters().gravity));
    require(std::abs(specific.z() - rocked.parameters().gravity) < 1e-3, "ground impulse IMU reads support");
    // g * sin(tilt) for a rest tilt inside the 1 mm band: min_depth / box span.
    const double band_specific = rocked.parameters().gravity *
                                 (Fs150SupportFace::kMinDepth / Fs150SupportFace::kBoxX);
    require(specific.head<2>().norm() <= band_specific + 1e-3,
            "settled IMU horizontal force stays inside the 1 mm tolerance");

    // Sol1 residual probe. One active corner has vn = -0.05322056. A normal
    // update followed by a friction projection previously finished at
    // normal_impulse 0 with that inward speed still present.
    RigidBodyModel residual({0, 0, 0});
    auto residual_state = residual.state();
    residual_state.position = {0, 0, 0.44331351646841766};
    residual_state.orientation =
        Eigen::Quaterniond(0.63882995586824864, 0.24611528791343912, -0.48146879243561047,
                           -0.5472763053990658);
    residual_state.velocity = {-0.018354678873946716, -0.038244035458668883,
                               0.043980108415723501};
    residual_state.angular_velocity = {0.29154703221117367, -0.39227990773191773,
                                       0.37438887679789196};
    residual.set_state(residual_state);
    const double residual_before = energy(residual_state, residual.parameters());
    const auto solved = resolveFlatGroundContact(residual, 0.15, 0.002);
    const auto residual_final = residual.state();
    require(solved.valid && solved.constrained && solved.normal_impulse > 1e-4 &&
                solved.normal_impulse < 0.05,
            "single-corner residual impulse");
    const Eigen::Vector3d residual_com = residual.parameters().body_origin_to_com;
    int active_corners = 0;
    double active_vn = 0.0;
    double active_vt = 0.0;
    double active_gap = 0.0;
    for (int i = 0; i < Fs150SupportFace::kCorners; ++i) {
      const Eigen::Vector3d lever = Fs150SupportFace::corner(i) - residual_com;
      const double gap =
          (residual_final.position + residual_final.orientation * lever).z() - 0.15;
      const Eigen::Vector3d point =
          residual_final.velocity +
          residual_final.orientation * residual_final.angular_velocity.cross(lever);
      if (i == 0) {
        active_vn = point.z();
        active_vt = point.head<2>().norm();
        active_gap = gap;
      }
      if (gap <= Fs150SupportFace::kMinDepth)
        ++active_corners;
    }
    require(active_corners == 1, "residual has one active corner");
    require(std::abs(active_gap) < 1e-9, "residual corner stays on the plane");
    require(std::abs(active_vn) < 1e-6, "residual normal speed");
    require(active_vt > 0.05 && active_vt < 0.18, "residual tangential coupling");
    require(residual_final.orientation.angularDistance(residual_state.orientation) < 1e-12,
            "residual attitude");
    require(energy(residual_final, residual.parameters()) <= residual_before + 1e-9,
            "residual energy");
    require(residual_final.velocity.norm() > 0.01 && residual_final.angular_velocity.norm() > 0.2,
            "residual velocity");
    const double residual_after = energy(residual_final, residual.parameters());
    const auto again = resolveFlatGroundContact(residual, 0.15, 0.002);
    const Eigen::Vector3d lever0 = Fs150SupportFace::corner(0) - residual_com;
    const Eigen::Vector3d point_again =
        residual.state().velocity +
        residual.state().orientation * residual.state().angular_velocity.cross(lever0);
    require(again.valid && again.constrained && std::abs(point_again.z()) < 1e-6,
            "residual second solve");
    require(energy(residual.state(), residual.parameters()) <= residual_after + 1e-9,
            "residual second energy");
    require(residual.state().orientation.angularDistance(residual_state.orientation) < 1e-12,
            "residual second attitude");

    std::cout << "PASS support face, canonical ground, friction dissipation, IMU\n";
    std::cout << "  settled roll=" << roll << " pitch=" << pitch
              << " |w|=" << rocked.state().angular_velocity.norm()
              << " yaw_w=" << yawed.state().angular_velocity.z() << '\n';
    std::cout << "  residual vn=" << active_vn << " impulse=" << solved.normal_impulse
              << " vt=" << active_vt << " again_vn=" << point_again.z()
              << " again_impulse=" << again.normal_impulse << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "FAIL " << error.what() << '\n';
    return 1;
  }
}
