#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>
#include <limits>

namespace xgc_lightweight {

// The numbers of a mavros_msgs/PositionTarget as the ROS boundary received them (ENU or body FLU, no
// conversion). Private to xsim: it is handed from the ROS callback to the world thread and never serialised.
struct PositionTarget {
  double position[3]{}, velocity[3]{}, acceleration[3]{};
  double yaw = 0, yaw_rate = 0;
  uint16_t type_mask = 0; // PositionTarget IGNORE_* bits
  uint8_t coordinate_frame = 0;
};

struct MaskedPva {
  Eigen::Vector3d position{Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d velocity{Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  double yaw{std::numeric_limits<double>::quiet_NaN()};
  double yaw_rate{std::numeric_limits<double>::quiet_NaN()};
  bool receiver_valid{false};
};

// Fixed contract, not latest PX4 behaviour:
// PX4/PX4-Autopilot v1.12.3 src/modules/mavlink/mavlink_receiver.cpp:870-993;
// mavlink/mavros 1.20.0 mavros/src/plugins/setpoint_raw.cpp:164-203 and
// mavros/src/lib/ftf_frame_conversions.cpp:75-112.
// Input is a direct copy of ROS PositionTarget numbers (ENU or body FLU).
// Output is world ENU. MAVLink's P/V/A and yaw/rate fields are floats; output doubles
// preserve those receiver values, not the original wire's extra precision.
//
// IMPORTANT: LOCAL_NED(frame 1) numerics round-trip ENU->NED->ENU, but MAVROS
// sends type_mask unchanged. Receiver applies X/Y bits AFTER swapping numbers.
// Consequently a single X/Y ignore bit masks the OTHER ENU component. Do not
// silently "fix" it: PositionControl rejects unpaired XY downstream.
// BODY_NED(frame 8) ignores position, uses heading for velocity, full attitude
// for acceleration, and suppresses an entire V/A group if ANY group bit is set.
// BODY yaw round-trips to wrap(pi/2 + ROS yaw); frame 1 retains ROS yaw.
// Both yaw fields survive; their downstream fallback belongs to PositionControl.
//
// receiver_valid means a receiver heartbeat may be accepted, not that all axes
// are controllable. PositionControl.cpp:91-105/189-207 independently checks XY
// pairing, setpoint/state pairs and finite resulting acceleration/thrust. This
// decoder must not merge those checks into receiver validity or change a timer.
inline MaskedPva decodePositionTarget(const PositionTarget &wire,
                                     const Eigen::Quaterniond &actual_orientation,
                                     double actual_yaw) noexcept {
  MaskedPva out;
  const auto mask = wire.type_mask;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  constexpr double pi = 3.141592653589793238462643383279502884;
  if (wire.coordinate_frame != 1 && wire.coordinate_frame != 8)
    return out; // Includes unsupported LOCAL_OFFSET_NED(7), BODY_OFFSET_NED(9).

  if (wire.coordinate_frame == 1) {
    // MAVROS uses a permutation/diagonal reflection, which does not let an
    // ignored NaN/Inf in one component contaminate another component.
    for (int i = 0; i < 3; ++i) {
      const int native_axis = i < 2 ? 1 - i : i;
      out.position[i] = (mask & (1u << native_axis)) ? nan : static_cast<float>(wire.position[i]);
      out.velocity[i] = (mask & (8u << native_axis)) ? nan : static_cast<float>(wire.velocity[i]);
      out.acceleration[i] = (mask & (64u << native_axis)) ? nan : static_cast<float>(wire.acceleration[i]);
    }
  } else {
    // MAVROS's FLU->FRD rotation is an affine rotation by pi about X, not
    // its special NaN-safe ENU/NED reflection. Preserve that nonfinite-vector
    // propagation for unmasked BODY groups. Masked groups are never evaluated.
    const Eigen::Quaterniond body_flip(Eigen::AngleAxisd(pi, Eigen::Vector3d::UnitX()));
    const Eigen::Matrix3d flu_frd = body_flip.toRotationMatrix();
    if (!(mask & 56u)) {
      const Eigen::Vector3f body = (flu_frd * Eigen::Map<const Eigen::Vector3d>(wire.velocity)).cast<float>();
      const float heading_ned = static_cast<float>(pi / 2.0 - actual_yaw);
      const float c = std::cos(heading_ned), s = std::sin(heading_ned);
      // Receiver yaw-only body FRD->NED, then NaN-safe NED->ENU reflection.
      out.velocity = Eigen::Vector3d(s * body.x() + c * body.y(),
                                    c * body.x() - s * body.y(), -body.z());
    }
    if (!(mask & 448u)) {
      const Eigen::Quaterniond world_flip =
          Eigen::AngleAxisd(pi / 2.0, Eigen::Vector3d::UnitZ()) * body_flip;
      const Eigen::Quaternionf native_orientation = (world_flip * actual_orientation * body_flip).cast<float>();
      const Eigen::Vector3f body = (flu_frd * Eigen::Map<const Eigen::Vector3d>(wire.acceleration)).cast<float>();
      const Eigen::Vector3f native = native_orientation.toRotationMatrix() * body;
      out.acceleration = Eigen::Vector3d(native.y(), native.x(), -native.z());
    }
  }

  if (!(mask & 1024u)) {
    // Execute the quaternion-based MAVROS yaw conversion before the MAVLink
    // float boundary. This also preserves its behaviour for nonfinite/very
    // large ROS angles, where directly adding pi/2 is not numerically equivalent.
    const Eigen::Quaterniond body_flip(Eigen::AngleAxisd(pi, Eigen::Vector3d::UnitX()));
    const Eigen::Quaterniond request(Eigen::AngleAxisd(wire.yaw, Eigen::Vector3d::UnitZ()));
    const Eigen::Quaterniond native = wire.coordinate_frame == 8 ? body_flip * request :
        (Eigen::Quaterniond(Eigen::AngleAxisd(pi / 2.0, Eigen::Vector3d::UnitZ())) * body_flip) * request * body_flip;
    const float yaw_ned = static_cast<float>(std::atan2(
        2.0 * (native.w() * native.z() + native.x() * native.y()),
        1.0 - 2.0 * (native.y() * native.y() + native.z() * native.z())));
    out.yaw = std::isfinite(yaw_ned) ? std::remainder(pi / 2.0 - yaw_ned, 2.0 * pi) : nan;
  }
  if (!(mask & 2048u))
    out.yaw_rate = static_cast<float>(wire.yaw_rate); // ENU->NED->ENU reverses the sign twice.

  const auto any_finite = [](const Eigen::Vector3d &v) {
    return std::isfinite(v.x()) || std::isfinite(v.y()) || std::isfinite(v.z());
  };
  const bool acceleration = any_finite(out.acceleration);
  out.receiver_valid = (any_finite(out.position) || any_finite(out.velocity) || acceleration) &&
                       !(acceleration && (mask & 512u));
  return out;
}

} // namespace xgc_lightweight
