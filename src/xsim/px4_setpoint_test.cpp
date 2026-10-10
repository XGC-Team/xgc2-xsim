#include "px4_setpoint.hpp"

#include <algorithm>
#include <array>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

// Define XGC_PX4_SOURCE_ORACLE and supply px4_source_oracle.hpp on the include
// path to execute the VERBATIM tagged MAVROS local_cb and PX4 receiver methods.
// The companion /tmp evidence package records source URLs, SHA256, extraction
// ranges, minimal message/transport stubs and the reproducible compile command.
// No reference implementation is substituted for those methods.
#ifdef XGC_PX4_SOURCE_ORACLE
#include "px4_source_oracle.hpp"
#endif

using namespace xgc_lightweight;
namespace {
constexpr double pi = 3.141592653589793238462643383279502884;
const double nan = std::numeric_limits<double>::quiet_NaN();
const double inf = std::numeric_limits<double>::infinity();

void require(bool condition, const std::string &message) {
  if (!condition)
    throw std::runtime_error(message);
}
void equal(double a, double b, const std::string &message, bool angle = false) {
  if (std::isnan(a) || std::isnan(b)) {
    require(std::isnan(a) && std::isnan(b), message + " NaN pattern");
  } else if (std::isinf(a) || std::isinf(b)) {
    require(a == b, message + " infinity sign");
  } else {
    const double delta = angle ? std::remainder(a - b, 2 * pi) : a - b;
    require(std::abs(delta) <= 3e-5 * std::max({1.0, std::abs(a), std::abs(b)}),
            message + " actual=" + std::to_string(a) + " oracle=" + std::to_string(b));
  }
}
Eigen::Quaterniond attitude(double yaw) {
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(0.37, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(-0.28, Eigen::Vector3d::UnitX());
}
PositionTarget command() {
  PositionTarget w{};
  w.coordinate_frame = 1;
  const double p[] = {1.25, -2.5, 3.75}, v[] = {0.75, -0.5, 0.25}, a[] = {0.3, -0.4, 0.6};
  std::copy(p, p + 3, w.position);
  std::copy(v, v + 3, w.velocity);
  std::copy(a, a + 3, w.acceleration);
  w.yaw = 0.41;
  w.yaw_rate = -0.23;
  return w;
}
MaskedPva decode(const PositionTarget &w, double yaw = 0.63) {
  return decodePositionTarget(w, attitude(yaw), yaw);
}
void require_nan(const Eigen::Vector3d &v, const std::string &message) {
  require(std::isnan(v.x()) && std::isnan(v.y()) && std::isnan(v.z()), message);
}

void frame_and_heading_examples() {
  auto w = command();
  w.type_mask = 512u | 448u; // FORCE present, but acceleration entirely ignored.
  const auto world = decode(w);
  require(world.receiver_valid, "finite P/V with no finite force accepted");
  for (int i = 0; i < 3; ++i) {
    equal(world.position[i], w.position[i], "frame1 already ENU");
    equal(world.velocity[i], w.velocity[i], "frame1 velocity ENU");
  }
  equal(world.yaw, w.yaw, "frame1 yaw");
  equal(world.yaw_rate, w.yaw_rate, "frame1 yaw rate");

  w.coordinate_frame = 8;
  w.type_mask = 7; // all P ignored; V and A live
  const auto body = decode(w);
  require(body.receiver_valid, "body PVA accepted");
  require_nan(body.position, "BODY position always ignored");
  const Eigen::Vector3d expected_v =
      Eigen::AngleAxisd(0.63, Eigen::Vector3d::UnitZ()) * Eigen::Map<const Eigen::Vector3d>(w.velocity);
  const Eigen::Vector3d expected_a = attitude(0.63) * Eigen::Map<const Eigen::Vector3d>(w.acceleration);
  for (int i = 0; i < 3; ++i) {
    equal(body.velocity[i], expected_v[i], "BODY velocity uses yaw only");
    equal(body.acceleration[i], expected_a[i], "BODY acceleration uses full attitude");
  }
  equal(body.yaw, pi / 2 + w.yaw, "BODY yaw fixed MAVROS chain", true);
  equal(body.yaw_rate, w.yaw_rate, "BODY yaw rate restored independently");
  w.yaw = 2.9;
  equal(decode(w).yaw, std::remainder(pi / 2 + w.yaw, 2 * pi), "BODY yaw wrap", true);
}

void masks_nan_and_receiver_layer() {
  auto w = command();
  // Negative example: raw ROS X-ignore is receiver NED X-ignore, hence ENU Y.
  w.type_mask = 1u | 56u | 448u | 1024u | 2048u;
  const auto one_xy = decode(w);
  require(one_xy.receiver_valid, "unpaired XY must still refresh receiver heartbeat");
  equal(one_xy.position.x(), w.position[0], "ROS X-ignore leaves ENU X alive");
  require(std::isnan(one_xy.position.y()), "ROS X-ignore masks ENU Y after native swap");
  require(std::isfinite(one_xy.position.x()) != std::isfinite(one_xy.position.y()),
          "this valid receiver input fails PositionControl XY pairing separately");

  w.position[0] = w.position[1] = nan;
  w.type_mask = 56u | 448u | 1024u | 2048u;
  const auto z_only = decode(w);
  require(z_only.receiver_valid, "unmasked NaN is absence, finite Z still receiver-valid");
  require(std::isnan(z_only.position.x()) && std::isnan(z_only.position.y()),
          "receiver does not fill missing controllable horizontal axes");

  w = command();
  w.type_mask = 4088; // Position-only legacy Takeoff, FORCE and every unused bit.
  std::fill(w.velocity, w.velocity + 3, inf);
  std::fill(w.acceleration, w.acceleration + 3, nan);
  w.yaw = inf;
  w.yaw_rate = nan;
  const auto p_only = decode(w);
  require(p_only.receiver_valid, "mask4088 position-only FORCE must remain accepted");
  require(p_only.position.allFinite(), "ignored NaN/Inf must not pollute position");
  require_nan(p_only.velocity, "ignored Inf velocity");
  require_nan(p_only.acceleration, "ignored NaN acceleration");
  require(std::isnan(p_only.yaw) && std::isnan(p_only.yaw_rate), "ignored heading payload");

  w = command();
  w.type_mask = 512u;
  require(!decode(w).receiver_valid, "actual finite acceleration plus FORCE rejected");
  std::fill(w.acceleration, w.acceleration + 3, nan);
  require(decode(w).receiver_valid, "FORCE bit with no finite A cannot reject finite P/V");
  w.acceleration[1] = 0;
  require(!decode(w).receiver_valid, "a single finite A suffices to reject FORCE");

  w = command();
  w.type_mask = 511u; // P/V/A ignored, both yaw inputs live.
  require(!decode(w).receiver_valid, "yaw and yawrate alone cannot refresh heartbeat");
  equal(decode(w).yaw, w.yaw, "both heading fields retained without heartbeat");
  equal(decode(w).yaw_rate, w.yaw_rate, "yaw rate must not be overridden by yaw");
  w.type_mask = 4095;
  require(!decode(w).receiver_valid, "all ignored is not a heartbeat");

  w = command();
  w.coordinate_frame = 8;
  for (int axis = 0; axis < 3; ++axis) {
    w.type_mask = static_cast<uint16_t>(8u << axis);
    require_nan(decode(w).velocity, "BODY any V-ignore disables whole group");
    w.type_mask = static_cast<uint16_t>(64u << axis);
    require_nan(decode(w).acceleration, "BODY any A-ignore disables whole group");
  }
  // BODY position payload (including ignored nonfinites) cannot act as a P command.
  w.type_mask = 56u | 448u;
  require(!decode(w).receiver_valid, "BODY P alone cannot sustain offboard");

  for (unsigned frame = 0; frame <= 255; ++frame) {
    if (frame == 1 || frame == 8)
      continue;
    w = command();
    w.coordinate_frame = static_cast<uint8_t>(frame);
    const auto out = decode(w);
    require(!out.receiver_valid, "unsupported frame rejected without exception");
    require_nan(out.position, "unsupported frame produces no usable setpoint");
  }
}

#ifdef XGC_PX4_SOURCE_ORACLE
void original_function_differential() {
  size_t comparisons = 0, accepted = 0;
  // Finite, unmasked NaN, Inf, huge finite-to-MAVLink-float overflow and
  // ignored-poison payloads. Every uint16 mask (including unknown high bits),
  // selected frame and tilted heading is tested against the original methods.
  for (unsigned frame : {1u, 8u, 7u, 9u}) {
    for (double actual_yaw : {-1.1, 0.0, 0.63, 2.7}) {
      for (unsigned pattern = 0; pattern < 5; ++pattern) {
        for (unsigned mask = 0; mask < 65536; ++mask) {
          auto w = command();
          w.coordinate_frame = static_cast<uint8_t>(frame);
          w.type_mask = static_cast<uint16_t>(mask);
          if (pattern == 1) {
            w.position[0] = nan;
            w.velocity[1] = nan;
            w.acceleration[2] = nan;
          } else if (pattern == 2) {
            w.position[1] = inf;
            w.velocity[2] = -inf;
            w.acceleration[0] = inf;
          } else if (pattern == 3) {
            w.position[2] = 1e100;
            w.velocity[0] = -1e100;
            w.acceleration[1] = 1e100;
          } else if (pattern == 4) {
            // Set poison on components actually ignored in the native frame.
            for (int axis = 0; axis < 3; ++axis) {
              const int ros_axis = frame == 1 && axis < 2 ? 1 - axis : axis;
              if (mask & (1u << axis)) w.position[ros_axis] = nan;
              if (mask & (8u << axis)) w.velocity[ros_axis] = inf;
              if (mask & (64u << axis)) w.acceleration[ros_axis] = -inf;
            }
            if (mask & 1024u) w.yaw = inf;
            if (mask & 2048u) w.yaw_rate = nan;
          }
          const auto got = decodePositionTarget(w, attitude(actual_yaw), actual_yaw);
          const auto want = source_oracle(w, attitude(actual_yaw));
          const std::string label = "frame=" + std::to_string(frame) + " mask=" +
              std::to_string(mask) + " pattern=" + std::to_string(pattern);
          require(got.receiver_valid == want.receiver_valid, label + " receiver heartbeat");
          ++comparisons;
          if (!want.receiver_valid)
            continue; // The original receiver publishes no setpoint on rejection.
          ++accepted;
          for (int axis = 0; axis < 3; ++axis) {
            equal(got.position[axis], want.position[axis], label + " P");
            equal(got.velocity[axis], want.velocity[axis], label + " V");
            equal(got.acceleration[axis], want.acceleration[axis], label + " A");
          }
          equal(got.yaw, want.yaw, label + " yaw", true);
          equal(got.yaw_rate, want.yaw_rate, label + " yawrate");
        }
      }
    }
  }
  // Upper mask bits do not acquire meanings invented by the decoder.
  auto w = command();
  w.type_mask = 0xf000 | 448;
  const auto got = decode(w), want = source_oracle(w, attitude(0.63));
  require(got.receiver_valid == want.receiver_valid, "high mask bits");
  for (int i = 0; i < 3; ++i) equal(got.position[i], want.position[i], "high mask P");
  for (unsigned frame : {1u, 8u}) {
    for (double yaw : {nan, inf, -inf, 1e100, -2.7, 0.41}) {
      for (double rate : {nan, inf, -inf, 1e100, -0.23}) {
        w = command();
        w.coordinate_frame = static_cast<uint8_t>(frame);
        w.yaw = yaw;
        w.yaw_rate = rate;
        const auto actual = decode(w), expected = source_oracle(w, attitude(0.63));
        require(actual.receiver_valid == expected.receiver_valid, "nonfinite yaw must not reject PVA");
        equal(actual.yaw, expected.yaw, "MAVROS quaternion yaw boundary", true);
        equal(actual.yaw_rate, expected.yaw_rate, "MAVLink yawrate float boundary");
      }
    }
  }
  std::cout << "verbatim MAVROS1.20/PX4v1.12.3 differential cases=" << comparisons
            << " accepted=" << accepted << " (finite values <=3e-5 relative/absolute, masks/nonfinite exact)\n";
}
#endif
} // namespace

int main() {
  try {
    frame_and_heading_examples();
    masks_nan_and_receiver_layer();
#ifdef XGC_PX4_SOURCE_ORACLE
    original_function_differential();
#endif
    std::cout << "PASS frames, yaw+yawrate, all BODY group masks, NaN/Inf, FORCE, receiver/downstream separation\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "FAIL " << e.what() << '\n';
    return 1;
  }
}
