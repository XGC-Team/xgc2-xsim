#pragma once
#include "components.hpp"

namespace xsim {
enum class Op {
  Add, Remove, Pause, Resume, Step, Reset, Provider, SetEnabled,
  Arm, Mode, Pva, Attitude, Velocity
};

// Numeric input only. Reliable requests add their completion state in Command.
struct ControlSample {
  Op op = Op::Pause;
  Key key;
  int64_t at = 0;
  int64_t arrival_ns = 0; // not before realtime receipt during catch-up
  int64_t received_ns = 0; // Velocity: HOLD-domain receipt time
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  PositionTarget pva{};
  FlightAttitudeSetpoint attitude;
  Clock::time_point deadline = Clock::now() + std::chrono::milliseconds(1500);
};

// One ROS-input producer, one world consumer. A single tagged mailbox keeps
// the newest control kind as well as its value. No producer writes the slot
// currently owned by the consumer; exchanging indices transfers ownership.
class ControlMailbox {
public:
  bool publish(const ControlSample &sample) noexcept {
    slots_[back_].value = sample;
    const auto previous = middle_.exchange(uint8_t(back_ | dirty), std::memory_order_acq_rel);
    back_ = previous & index_mask;
    return (previous & dirty) != 0;
  }
  bool take(ControlSample &sample) noexcept {
    if (!(middle_.load(std::memory_order_acquire) & dirty)) return false;
    front_ = middle_.exchange(front_, std::memory_order_acq_rel) & index_mask;
    sample = slots_[front_].value;
    return true;
  }

private:
  static constexpr uint8_t dirty = 4, index_mask = 3;
  struct alignas(64) Slot { ControlSample value; };
  std::array<Slot, 3> slots_;
  alignas(64) std::atomic<uint8_t> middle_{2};
  alignas(64) uint8_t back_ = 0; // producer only
  alignas(64) uint8_t front_ = 1; // consumer only
};
} // namespace xsim
