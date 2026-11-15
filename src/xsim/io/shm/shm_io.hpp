#pragma once
#include "io/runtime_io.hpp"
#include "systems/sensors.hpp"
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <string_view>

namespace xsim {

using RawPositionTarget = xgc_lightweight::PositionTarget;

enum class ShmCommandOp : uint8_t {
  None = 0,
  Pva = 1,
  Attitude = 2,
  Velocity = 3,
  Arm = 4,
  Mode = 5,
};

struct alignas(64) ShmHeader {
  static constexpr uint64_t kMagic = 0x5853494D53484D31ULL; // "XSIMSHM1"
  static constexpr uint32_t kVersion = 1;

  uint64_t magic = kMagic;
  uint32_t version = kVersion;
  uint32_t capacity = 0;
  std::atomic<uint32_t> active_count{0};
  std::atomic<uint32_t> server_waiting{0};
  std::atomic<int64_t> sim_ns{0};
  std::atomic<uint64_t> steps{0};
  std::atomic<uint64_t> frame_seq{0};
  std::atomic<uint32_t> control_bell{0};
  uint32_t reserved1 = 0;
};

struct ShmEntityStateSnapshot {
  uint64_t entity_id = 0;
  uint64_t generation = 0;
  int64_t stamp_ns = 0;
  double position[3]{};
  double velocity[3]{};
  double orientation_xyzw[4]{0.0, 0.0, 0.0, 1.0};
  double omega_body[3]{};
  double specific_force_body[3]{};
  double rotors[4]{};
  uint8_t kind = 0;
  uint8_t enabled = 0;
  uint8_t armed = 0;
  uint8_t landed = 0;
  char mode[24]{};
  char name[32]{};
};

struct alignas(64) ShmEntitySlot {
  // Writer: xsim output lane; Readers: external processes (lock-free seqlock).
  std::atomic<uint64_t> state_seq{0};
  ShmEntityStateSnapshot state{};

  // Separate cache line so controller writes never false-share with state reads.
  alignas(64) std::atomic<uint64_t> cmd_write_seq{0};
  std::atomic<uint64_t> cmd_read_seq{0};
  uint8_t cmd_op = 0;
  uint8_t cmd_coordinate_frame = 1;
  uint8_t cmd_arm = 0;
  uint8_t cmd_force = 0;
  uint32_t cmd_type_mask = 0;
  int64_t cmd_at_ns = 0;
  double cmd_vec0[3]{};
  double cmd_vec1[3]{};
  double cmd_vec2[3]{};
  double cmd_quat_wxyz[4]{1.0, 0.0, 0.0, 0.0};
  double cmd_scalar0 = 0.0;
  double cmd_scalar1 = 0.0;
  char cmd_mode[24]{};
};

inline size_t shm_region_bytes(uint32_t capacity) {
  return sizeof(ShmHeader) + static_cast<size_t>(capacity) * sizeof(ShmEntitySlot);
}

// Lock-free client view over a mapped ShmHeader + ShmEntitySlot[capacity] region.
class ShmClientView {
public:
  ShmClientView() = default;
  ShmClientView(std::string shm_name, bool read_only = false);
  ~ShmClientView();
  ShmClientView(const ShmClientView &) = delete;
  ShmClientView &operator=(const ShmClientView &) = delete;
  ShmClientView(ShmClientView &&other) noexcept;
  ShmClientView &operator=(ShmClientView &&other) noexcept;

  const ShmHeader *header() const noexcept { return header_; }
  uint32_t capacity() const noexcept { return header_ ? header_->capacity : 0; }
  uint32_t active_count() const noexcept {
    return header_ ? header_->active_count.load(std::memory_order_acquire) : 0;
  }

  int find_slot(std::string_view entity_name) const noexcept;
  bool read_state(uint32_t slot_index, ShmEntityStateSnapshot &out) const noexcept;

  bool publish_pva(uint32_t slot_index, const RawPositionTarget &pva,
                   int64_t at_ns = 0) noexcept;
  bool publish_attitude(uint32_t slot_index, const FlightAttitudeSetpoint &att, int64_t at_ns = 0) noexcept;
  bool publish_velocity(uint32_t slot_index, const Eigen::Vector3d &vel) noexcept;
  bool publish_arm(uint32_t slot_index, bool arm, bool force = false) noexcept;
  bool publish_mode(uint32_t slot_index, std::string_view mode) noexcept;

private:
  void notify_bell() noexcept;
  std::string name_;
  void *base_ = nullptr;
  size_t bytes_ = 0;
  ShmHeader *header_ = nullptr;
  ShmEntitySlot *slots_ = nullptr;
  bool read_only_ = false;
};

RuntimeIO make_shm_io(const Json &config, World &world, Sensors &sensors);
RuntimeIO compose_runtime_io(RuntimeIO primary, RuntimeIO secondary);

} // namespace xsim
