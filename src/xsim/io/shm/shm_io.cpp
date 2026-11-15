#include "io/shm/shm_io.hpp"
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <fcntl.h>
#include <linux/futex.h>
#include <mutex>
#include <new>
#include <pthread.h>
#include <stdexcept>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <utility>
#include <vector>

namespace xsim {
namespace {

inline void futex_wait(std::atomic<uint32_t> *addr, uint32_t expected) noexcept {
  static_assert(sizeof(std::atomic<uint32_t>) == sizeof(uint32_t));
  (void)::syscall(SYS_futex, reinterpret_cast<uint32_t *>(addr), FUTEX_WAIT,
                  expected, nullptr, nullptr, 0);
}

inline void futex_wake(std::atomic<uint32_t> *addr, int count = INT32_MAX) noexcept {
  static_assert(sizeof(std::atomic<uint32_t>) == sizeof(uint32_t));
  (void)::syscall(SYS_futex, reinterpret_cast<uint32_t *>(addr), FUTEX_WAKE,
                  count, nullptr, nullptr, 0);
}

std::string normalize_shm_name(std::string name) {
  if (name.empty())
    throw std::invalid_argument("shm_name must be non-empty");
  if (name.front() != '/')
    name.insert(name.begin(), '/');
  for (size_t i = 1; i < name.size(); ++i) {
    const char c = name[i];
    if (c == '/' || c == '\0')
      name[i] = '_';
  }
  if (name.size() > 240)
    name.resize(240);
  return name;
}

void copy_bounded(char *dst, size_t cap, std::string_view src) noexcept {
  if (!cap) return;
  std::memset(dst, 0, cap);
  const size_t n = std::min(cap - 1, src.size());
  if (n) std::memcpy(dst, src.data(), n);
}

void snapshot_from_state(const State &s, const Entity &e, ShmEntityStateSnapshot &out) noexcept {
  out.entity_id = s.key.id;
  out.generation = s.key.generation;
  out.stamp_ns = s.stamp;
  for (int a = 0; a < 3; ++a) {
    out.position[a] = s.position[a];
    out.velocity[a] = s.velocity[a];
    out.omega_body[a] = s.omega[a];
    out.specific_force_body[a] = s.specific_force[a];
  }
  out.orientation_xyzw[0] = s.orientation.x();
  out.orientation_xyzw[1] = s.orientation.y();
  out.orientation_xyzw[2] = s.orientation.z();
  out.orientation_xyzw[3] = s.orientation.w();
  for (int i = 0; i < 4; ++i)
    out.rotors[i] = s.rotors[i];
  out.kind = static_cast<uint8_t>(e.config.kind);
  out.enabled = s.enabled ? 1 : 0;
  out.armed = s.armed ? 1 : 0;
  out.landed = s.landed ? 1 : 0;
  std::memcpy(out.mode, s.mode.data(), sizeof(out.mode));
  out.mode[sizeof(out.mode) - 1] = '\0';
  copy_bounded(out.name, sizeof(out.name), e.config.name);
}

struct ShmEntityBinding {
  uint32_t slot_index = 0;
  std::weak_ptr<Entity> entity;
};

struct ShmRuntime {
  std::string shm_name;
  uint32_t capacity = 0;
  size_t mapped_bytes = 0;
  void *mapped = nullptr;
  ShmHeader *header = nullptr;
  ShmEntitySlot *slots = nullptr;
  World &world;

  std::mutex roster_mutex;
  std::vector<ShmEntityBinding> bindings;
  std::unordered_map<uint64_t, uint32_t> id_to_slot;
  uint32_t last_control_bell = 0;

  std::function<void()> input_wake;
  std::atomic<bool> bell_running{false};
  std::thread bell_thread;

  std::atomic<uint64_t> published_frames{0};
  std::atomic<uint64_t> accepted_controls{0};
  std::atomic<uint64_t> accepted_ReliableOps{0};

  ShmRuntime(const Json &config, World &w) : world(w) {
    const std::string default_name =
        "xsim_" + config.value("run_id", std::string("default"));
    shm_name = normalize_shm_name(config.value("shm_name", default_name));
    const size_t configured_entities =
        config.contains("entities") && config.at("entities").is_array()
            ? config.at("entities").size()
            : 0;
    const uint32_t requested = config.value(
        "shm_capacity",
        static_cast<uint32_t>(std::max<size_t>(1024, configured_entities + 64)));
    if (requested == 0 || requested > 16384)
      throw std::invalid_argument("shm_capacity must be in 1..16384");
    capacity = requested;
    mapped_bytes = shm_region_bytes(capacity);

    const int fd = ::shm_open(shm_name.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0600);
    if (fd < 0)
      throw std::runtime_error("shm_open failed for " + shm_name + ": " + std::strerror(errno));
    if (::ftruncate(fd, static_cast<off_t>(mapped_bytes)) != 0) {
      const int err = errno;
      ::close(fd);
      ::shm_unlink(shm_name.c_str());
      throw std::runtime_error("ftruncate failed for " + shm_name + ": " + std::strerror(err));
    }
    void *ptr = ::mmap(nullptr, mapped_bytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    const int map_err = errno;
    ::close(fd);
    if (ptr == MAP_FAILED) {
      ::shm_unlink(shm_name.c_str());
      throw std::runtime_error("mmap failed for " + shm_name + ": " + std::strerror(map_err));
    }
    mapped = ptr;
    std::memset(mapped, 0, mapped_bytes);
    header = new (mapped) ShmHeader();
    header->capacity = capacity;
    header->sim_ns.store(world.epoch, std::memory_order_relaxed);
    slots = reinterpret_cast<ShmEntitySlot *>(
        static_cast<uint8_t *>(mapped) + sizeof(ShmHeader));
    for (uint32_t i = 0; i < capacity; ++i)
      new (&slots[i]) ShmEntitySlot();
    std::atomic_thread_fence(std::memory_order_release);
  }

  ~ShmRuntime() {
    stop_bell();
    if (mapped && mapped != MAP_FAILED) {
      ::munmap(mapped, mapped_bytes);
      mapped = nullptr;
    }
    if (!shm_name.empty())
      ::shm_unlink(shm_name.c_str());
  }

  void start_bell() {
    if (!input_wake || bell_running.exchange(true))
      return;
    bell_thread = std::thread([this] {
      pthread_setname_np(pthread_self(), "xsim-shm-bell");
      uint32_t seen = header->control_bell.load(std::memory_order_acquire);
      while (bell_running.load(std::memory_order_acquire)) {
        const uint32_t cur = header->control_bell.load(std::memory_order_acquire);
        if (cur != seen) {
          seen = cur;
          if (input_wake) input_wake();
          continue;
        }
        header->server_waiting.store(1, std::memory_order_seq_cst);
        const uint32_t recheck = header->control_bell.load(std::memory_order_seq_cst);
        if (recheck == seen && bell_running.load(std::memory_order_acquire))
          futex_wait(&header->control_bell, seen);
        header->server_waiting.store(0, std::memory_order_relaxed);
      }
    });
  }

  void stop_bell() {
    if (!bell_running.exchange(false))
      return;
    if (header) {
      header->control_bell.fetch_add(1, std::memory_order_acq_rel);
      futex_wake(&header->control_bell);
    }
    if (bell_thread.joinable())
      bell_thread.join();
  }

  uint32_t bind_entity(const std::shared_ptr<Entity> &entity) {
    std::lock_guard<std::mutex> lock(roster_mutex);
    const auto existing = id_to_slot.find(entity->id);
    if (existing != id_to_slot.end())
      return existing->second;
    if (bindings.size() >= capacity)
      throw std::runtime_error("SHM entity capacity exhausted");
    const uint32_t idx = static_cast<uint32_t>(bindings.size());
    bindings.push_back({idx, entity});
    id_to_slot.emplace(entity->id, idx);

    auto &slot = slots[idx];
    const uint64_t seq0 = slot.state_seq.load(std::memory_order_relaxed);
    slot.state_seq.store(seq0 + 1, std::memory_order_release);
    slot.state.entity_id = entity->id;
    slot.state.generation = entity->generation;
    slot.state.kind = static_cast<uint8_t>(entity->config.kind);
    slot.state.enabled = entity->enabled ? 1 : 0;
    for (int a = 0; a < 3; ++a)
      slot.state.position[a] = entity->config.initial[a];
    copy_bounded(slot.state.name, sizeof(slot.state.name), entity->config.name);
    slot.state_seq.store(seq0 + 2, std::memory_order_release);

    header->active_count.store(static_cast<uint32_t>(bindings.size()),
                               std::memory_order_release);
    return idx;
  }

  void write_slot(uint32_t idx, const State &s, const Entity &e) noexcept {
    auto &slot = slots[idx];
    const uint64_t seq0 = slot.state_seq.load(std::memory_order_relaxed);
    slot.state_seq.store(seq0 + 1, std::memory_order_release);
    std::atomic_thread_fence(std::memory_order_acq_rel);
    snapshot_from_state(s, e, slot.state);
    slot.state_seq.store(seq0 + 2, std::memory_order_release);
  }

  void publish_frame_states(const Frame &frame) {
    {
      std::lock_guard<std::mutex> lock(roster_mutex);
      for (const auto &s : frame.states) {
        auto e = s.entity.lock();
        if (!e || !e->alive) continue;
        const auto it = id_to_slot.find(s.key.id);
        if (it != id_to_slot.end())
          write_slot(it->second, s, *e);
      }
    }
    header->sim_ns.store(frame.stamp, std::memory_order_release);
    header->steps.store(frame.steps, std::memory_order_release);
    header->frame_seq.fetch_add(1, std::memory_order_release);
    published_frames.fetch_add(1, std::memory_order_relaxed);
  }

  bool inputs_pending() const noexcept {
    return header->control_bell.load(std::memory_order_acquire) != last_control_bell;
  }

  void poll_inputs() {
    const uint32_t bell = header->control_bell.load(std::memory_order_acquire);
    if (bell == last_control_bell)
      return;
    last_control_bell = bell;

    std::vector<ShmEntityBinding> snapshot_bindings;
    {
      std::lock_guard<std::mutex> lock(roster_mutex);
      snapshot_bindings = bindings;
    }
    const uint32_t n = std::min<uint32_t>(
        header->active_count.load(std::memory_order_acquire),
        static_cast<uint32_t>(snapshot_bindings.size()));
    for (uint32_t idx = 0; idx < n; ++idx) {
      auto &slot = slots[idx];
      const uint64_t wseq1 = slot.cmd_write_seq.load(std::memory_order_acquire);
      const uint64_t rseq = slot.cmd_read_seq.load(std::memory_order_relaxed);
      if (wseq1 == rseq || (wseq1 & 1ULL))
        continue;

      const uint8_t op = slot.cmd_op;
      const uint8_t frame_id = slot.cmd_coordinate_frame;
      const uint8_t arm = slot.cmd_arm;
      const uint8_t force = slot.cmd_force;
      const uint32_t mask = slot.cmd_type_mask;
      const int64_t at_ns = slot.cmd_at_ns;
      double v0[3], v1[3], v2[3], qwxyz[4];
      std::memcpy(v0, slot.cmd_vec0, sizeof(v0));
      std::memcpy(v1, slot.cmd_vec1, sizeof(v1));
      std::memcpy(v2, slot.cmd_vec2, sizeof(v2));
      std::memcpy(qwxyz, slot.cmd_quat_wxyz, sizeof(qwxyz));
      const double s0 = slot.cmd_scalar0;
      const double s1 = slot.cmd_scalar1;
      char mode_buf[24];
      std::memcpy(mode_buf, slot.cmd_mode, sizeof(mode_buf));
      mode_buf[sizeof(mode_buf) - 1] = '\0';

      std::atomic_thread_fence(std::memory_order_acquire);
      const uint64_t wseq2 = slot.cmd_write_seq.load(std::memory_order_relaxed);
      if (wseq1 != wseq2)
        continue; // Writer raced; pick up stable latest on next poll.
      slot.cmd_read_seq.store(wseq1, std::memory_order_release);

      auto entity = snapshot_bindings[idx].entity.lock();
      if (!entity || !entity->alive)
        continue;
      const uint64_t gen = entity->generation.load(std::memory_order_acquire);

      switch (static_cast<ShmCommandOp>(op)) {
      case ShmCommandOp::Pva: {
        ControlSample c;
        c.op = Op::Pva;
        c.key = {entity->id, gen};
        c.at = at_ns;
        c.pva.coordinate_frame = frame_id;
        c.pva.type_mask = static_cast<uint16_t>(mask);
        for (int a = 0; a < 3; ++a) {
          c.pva.position[a] = v0[a];
          c.pva.velocity[a] = v1[a];
          c.pva.acceleration[a] = v2[a];
        }
        c.pva.yaw = s0;
        c.pva.yaw_rate = s1;
        world.submit_control(*entity, c);
        accepted_controls.fetch_add(1, std::memory_order_relaxed);
        break;
      }
      case ShmCommandOp::Attitude: {
        ControlSample c;
        c.op = Op::Attitude;
        c.key = {entity->id, gen};
        c.at = at_ns;
        c.attitude.q = Eigen::Quaterniond(qwxyz[0], qwxyz[1], qwxyz[2], qwxyz[3]);
        c.attitude.body_rate = {v1[0], v1[1], v1[2]};
        c.attitude.thrust = s0;
        c.attitude.type_mask = mask;
        world.submit_control(*entity, c);
        accepted_controls.fetch_add(1, std::memory_order_relaxed);
        break;
      }
      case ShmCommandOp::Velocity: {
        const auto received = world.hold().now();
        if (!world.hold().admit(entity->public_id, received)) {
          ++world.metrics.hold_refused;
          break;
        }
        ControlSample c;
        c.op = Op::Velocity;
        c.key = {entity->id, gen};
        c.velocity = {v0[0], v0[1], v0[2]};
        c.received_ns = received;
        world.submit_control(*entity, c);
        accepted_controls.fetch_add(1, std::memory_order_relaxed);
        break;
      }
      case ShmCommandOp::Arm: {
        auto t = std::make_shared<Command>();
        t->op = Op::Arm;
        t->key = {entity->id, gen};
        t->arm = arm != 0;
        t->action = force ? 1 : 0;
        world.submit(t);
        accepted_ReliableOps.fetch_add(1, std::memory_order_relaxed);
        break;
      }
      case ShmCommandOp::Mode: {
        auto t = std::make_shared<Command>();
        t->op = Op::Mode;
        t->key = {entity->id, gen};
        t->mode = mode_buf;
        world.submit(t);
        accepted_ReliableOps.fetch_add(1, std::memory_order_relaxed);
        break;
      }
      default:
        break;
      }
    }
  }

  Json status() const {
    return {{"backend", "shm"},
            {"shm_name", shm_name},
            {"capacity", capacity},
            {"active_entities", header->active_count.load(std::memory_order_relaxed)},
            {"published_frames", published_frames.load(std::memory_order_relaxed)},
            {"accepted_controls", accepted_controls.load(std::memory_order_relaxed)},
            {"accepted_reliable_ops", accepted_ReliableOps.load(std::memory_order_relaxed)}};
  }
};

} // namespace

ShmClientView::ShmClientView(std::string shm_name, bool read_only)
    : name_(normalize_shm_name(std::move(shm_name))), read_only_(read_only) {
  const int flags = (read_only ? O_RDONLY : O_RDWR) | O_CLOEXEC;
  const int fd = ::shm_open(name_.c_str(), flags, 0);
  if (fd < 0)
    throw std::runtime_error("ShmClientView shm_open failed for " + name_ + ": " + std::strerror(errno));
  struct stat st{};
  if (::fstat(fd, &st) != 0 || st.st_size < static_cast<off_t>(sizeof(ShmHeader))) {
    ::close(fd);
    throw std::runtime_error("ShmClientView invalid shared-memory size for " + name_);
  }
  bytes_ = static_cast<size_t>(st.st_size);
  const int prot = read_only ? PROT_READ : (PROT_READ | PROT_WRITE);
  void *ptr = ::mmap(nullptr, bytes_, prot, MAP_SHARED, fd, 0);
  ::close(fd);
  if (ptr == MAP_FAILED)
    throw std::runtime_error("ShmClientView mmap failed for " + name_);
  base_ = ptr;
  header_ = static_cast<ShmHeader *>(base_);
  if (header_->magic != ShmHeader::kMagic || header_->version != ShmHeader::kVersion ||
      bytes_ < shm_region_bytes(header_->capacity)) {
    ::munmap(base_, bytes_);
    base_ = nullptr;
    header_ = nullptr;
    throw std::runtime_error("ShmClientView header magic/version mismatch for " + name_);
  }
  slots_ = reinterpret_cast<ShmEntitySlot *>(static_cast<uint8_t *>(base_) + sizeof(ShmHeader));
}

ShmClientView::~ShmClientView() {
  if (base_) {
    ::munmap(base_, bytes_);
    base_ = nullptr;
  }
}

ShmClientView::ShmClientView(ShmClientView &&other) noexcept
    : name_(std::move(other.name_)), base_(other.base_), bytes_(other.bytes_),
      header_(other.header_), slots_(other.slots_), read_only_(other.read_only_) {
  other.base_ = nullptr;
  other.bytes_ = 0;
  other.header_ = nullptr;
  other.slots_ = nullptr;
}

ShmClientView &ShmClientView::operator=(ShmClientView &&other) noexcept {
  if (this == &other) return *this;
  if (base_) ::munmap(base_, bytes_);
  name_ = std::move(other.name_);
  base_ = other.base_;
  bytes_ = other.bytes_;
  header_ = other.header_;
  slots_ = other.slots_;
  read_only_ = other.read_only_;
  other.base_ = nullptr;
  other.bytes_ = 0;
  other.header_ = nullptr;
  other.slots_ = nullptr;
  return *this;
}

int ShmClientView::find_slot(std::string_view entity_name) const noexcept {
  if (!header_) return -1;
  const uint32_t n = active_count();
  for (uint32_t i = 0; i < n; ++i) {
    ShmEntityStateSnapshot snap;
    if (read_state(i, snap) && entity_name == snap.name)
      return static_cast<int>(i);
  }
  return -1;
}

bool ShmClientView::read_state(uint32_t slot_index, ShmEntityStateSnapshot &out) const noexcept {
  if (!header_ || slot_index >= header_->capacity)
    return false;
  const auto &slot = slots_[slot_index];
  for (int attempt = 0; attempt < 64; ++attempt) {
    const uint64_t seq1 = slot.state_seq.load(std::memory_order_acquire);
    if (seq1 & 1ULL)
      continue;
    out = slot.state;
    std::atomic_thread_fence(std::memory_order_acquire);
    const uint64_t seq2 = slot.state_seq.load(std::memory_order_relaxed);
    if (seq1 == seq2)
      return seq1 != 0;
  }
  return false;
}

void ShmClientView::notify_bell() noexcept {
  header_->control_bell.fetch_add(1, std::memory_order_seq_cst);
  if (header_->server_waiting.load(std::memory_order_seq_cst))
    futex_wake(&header_->control_bell, 1);
}

bool ShmClientView::publish_pva(uint32_t slot_index,
                                const RawPositionTarget &pva,
                                int64_t at_ns) noexcept {
  if (!header_ || read_only_ || slot_index >= header_->capacity)
    return false;
  auto &slot = slots_[slot_index];
  const uint64_t seq0 = slot.cmd_write_seq.load(std::memory_order_relaxed);
  slot.cmd_write_seq.store(seq0 + 1, std::memory_order_release);
  std::atomic_thread_fence(std::memory_order_acq_rel);
  slot.cmd_op = static_cast<uint8_t>(ShmCommandOp::Pva);
  slot.cmd_coordinate_frame = pva.coordinate_frame;
  slot.cmd_type_mask = pva.type_mask;
  slot.cmd_at_ns = at_ns;
  for (int a = 0; a < 3; ++a) {
    slot.cmd_vec0[a] = pva.position[a];
    slot.cmd_vec1[a] = pva.velocity[a];
    slot.cmd_vec2[a] = pva.acceleration[a];
  }
  slot.cmd_scalar0 = pva.yaw;
  slot.cmd_scalar1 = pva.yaw_rate;
  slot.cmd_write_seq.store(seq0 + 2, std::memory_order_release);
  notify_bell();
  return true;
}

bool ShmClientView::publish_attitude(uint32_t slot_index, const FlightAttitudeSetpoint &att,
                                     int64_t at_ns) noexcept {
  if (!header_ || read_only_ || slot_index >= header_->capacity)
    return false;
  auto &slot = slots_[slot_index];
  const uint64_t seq0 = slot.cmd_write_seq.load(std::memory_order_relaxed);
  slot.cmd_write_seq.store(seq0 + 1, std::memory_order_release);
  std::atomic_thread_fence(std::memory_order_acq_rel);
  slot.cmd_op = static_cast<uint8_t>(ShmCommandOp::Attitude);
  slot.cmd_type_mask = att.type_mask;
  slot.cmd_at_ns = at_ns;
  slot.cmd_quat_wxyz[0] = att.q.w();
  slot.cmd_quat_wxyz[1] = att.q.x();
  slot.cmd_quat_wxyz[2] = att.q.y();
  slot.cmd_quat_wxyz[3] = att.q.z();
  for (int a = 0; a < 3; ++a)
    slot.cmd_vec1[a] = att.body_rate[a];
  slot.cmd_scalar0 = att.thrust;
  slot.cmd_write_seq.store(seq0 + 2, std::memory_order_release);
  notify_bell();
  return true;
}

bool ShmClientView::publish_velocity(uint32_t slot_index, const Eigen::Vector3d &vel) noexcept {
  if (!header_ || read_only_ || slot_index >= header_->capacity)
    return false;
  auto &slot = slots_[slot_index];
  const uint64_t seq0 = slot.cmd_write_seq.load(std::memory_order_relaxed);
  slot.cmd_write_seq.store(seq0 + 1, std::memory_order_release);
  std::atomic_thread_fence(std::memory_order_acq_rel);
  slot.cmd_op = static_cast<uint8_t>(ShmCommandOp::Velocity);
  for (int a = 0; a < 3; ++a)
    slot.cmd_vec0[a] = vel[a];
  slot.cmd_write_seq.store(seq0 + 2, std::memory_order_release);
  notify_bell();
  return true;
}

bool ShmClientView::publish_arm(uint32_t slot_index, bool arm, bool force) noexcept {
  if (!header_ || read_only_ || slot_index >= header_->capacity)
    return false;
  auto &slot = slots_[slot_index];
  const uint64_t seq0 = slot.cmd_write_seq.load(std::memory_order_relaxed);
  slot.cmd_write_seq.store(seq0 + 1, std::memory_order_release);
  std::atomic_thread_fence(std::memory_order_acq_rel);
  slot.cmd_op = static_cast<uint8_t>(ShmCommandOp::Arm);
  slot.cmd_arm = arm ? 1 : 0;
  slot.cmd_force = force ? 1 : 0;
  slot.cmd_write_seq.store(seq0 + 2, std::memory_order_release);
  notify_bell();
  return true;
}

bool ShmClientView::publish_mode(uint32_t slot_index, std::string_view mode) noexcept {
  if (!header_ || read_only_ || slot_index >= header_->capacity)
    return false;
  auto &slot = slots_[slot_index];
  const uint64_t seq0 = slot.cmd_write_seq.load(std::memory_order_relaxed);
  slot.cmd_write_seq.store(seq0 + 1, std::memory_order_release);
  std::atomic_thread_fence(std::memory_order_acq_rel);
  slot.cmd_op = static_cast<uint8_t>(ShmCommandOp::Mode);
  copy_bounded(slot.cmd_mode, sizeof(slot.cmd_mode), mode);
  slot.cmd_write_seq.store(seq0 + 2, std::memory_order_release);
  notify_bell();
  return true;
}

RuntimeIO make_shm_io(const Json &config, World &world, Sensors &) {
  auto runtime = std::make_shared<ShmRuntime>(config, world);
  RuntimeIO io;
  io.attach_entity = [runtime](const std::shared_ptr<Entity> &entity, const Json &) {
    const uint32_t idx = runtime->bind_entity(entity);
    entity->io = std::make_shared<EntityIO>(EntityIO{
        {},
        [runtime, idx](const State &s, const TelemetryRates &) {
          if (auto e = s.entity.lock(); e && e->alive)
            runtime->write_slot(idx, s, *e);
        },
        {},
        "shm://" + runtime->shm_name + "/" + std::to_string(idx),
    });
  };
  io.poll_inputs = [runtime] { runtime->poll_inputs(); };
  io.inputs_pending = [runtime] { return runtime->inputs_pending(); };
  io.bind_input_wake = [runtime](std::function<void()> wake) {
    runtime->input_wake = std::move(wake);
  };
  io.start_outputs = [runtime] { runtime->start_bell(); };
  io.stop_outputs = [runtime] { runtime->stop_bell(); };
  io.okay = [] { return true; };
  io.publish_frame = [runtime](const Frame &frame) {
    runtime->header->sim_ns.store(frame.stamp, std::memory_order_release);
    runtime->header->steps.store(frame.steps, std::memory_order_release);
  };
  io.publish_entities = [runtime](const std::shared_ptr<const Frame> &frame,
                                  const std::shared_ptr<const TelemetryRates> &) {
    if (frame) runtime->publish_frame_states(*frame);
  };
  io.publication_status = [runtime] { return runtime->status(); };
  io.facts = [runtime] {
    return Json{{"shm", {{"shm_name", runtime->shm_name}, {"capacity", runtime->capacity}}}};
  };
  return io;
}

RuntimeIO compose_runtime_io(RuntimeIO a, RuntimeIO b) {
  auto pa = std::make_shared<RuntimeIO>(std::move(a));
  auto pb = std::make_shared<RuntimeIO>(std::move(b));
  RuntimeIO out;
  out.attach_entity = [pa, pb](const std::shared_ptr<Entity> &entity, const Json &settings) {
    std::shared_ptr<EntityIO> io_a, io_b;
    if (pa->attach_entity) {
      pa->attach_entity(entity, settings);
      io_a = entity->io;
      entity->io.reset();
    }
    if (pb->attach_entity) {
      pb->attach_entity(entity, settings);
      io_b = entity->io;
      entity->io.reset();
    }
    const std::string topic = io_b && !io_b->localization_pose_topic.empty()
                                  ? io_b->localization_pose_topic
                                  : (io_a ? io_a->localization_pose_topic : std::string{});
    entity->io = std::make_shared<EntityIO>(EntityIO{
        [io_a, io_b] {
          if (io_a && io_a->reconcile) io_a->reconcile();
          if (io_b && io_b->reconcile) io_b->reconcile();
        },
        [io_a, io_b](const State &s, const TelemetryRates &r) {
          if (io_a && io_a->publish) io_a->publish(s, r);
          if (io_b && io_b->publish) io_b->publish(s, r);
        },
        [io_a, io_b](const Sample &sample, const std::shared_ptr<const SensorPayload> &payload) {
          if (io_a && io_a->publish_sensor) io_a->publish_sensor(sample, payload);
          if (io_b && io_b->publish_sensor) io_b->publish_sensor(sample, payload);
        },
        topic,
    });
  };
  out.poll_inputs = [pa, pb] {
    if (pa->poll_inputs) pa->poll_inputs();
    if (pb->poll_inputs) pb->poll_inputs();
  };
  out.inputs_pending = [pa, pb] {
    return (pa->inputs_pending && pa->inputs_pending()) ||
           (pb->inputs_pending && pb->inputs_pending());
  };
  out.bind_input_wake = [pa, pb](std::function<void()> wake) {
    if (pa->bind_input_wake) pa->bind_input_wake(wake);
    if (pb->bind_input_wake) pb->bind_input_wake(std::move(wake));
  };
  out.poll_services = [pa, pb] {
    if (pa->poll_services && pa->poll_services()) return true;
    return pb->poll_services && pb->poll_services();
  };
  out.bind_service_wake = [pa, pb](std::function<void()> wake) {
    if (pa->bind_service_wake) pa->bind_service_wake(wake);
    if (pb->bind_service_wake) pb->bind_service_wake(std::move(wake));
  };
  out.okay = [pa, pb] {
    return (!pa->okay || pa->okay()) && (!pb->okay || pb->okay());
  };
  out.publish_frame = [pa, pb](const Frame &frame) {
    if (pa->publish_frame) pa->publish_frame(frame);
    if (pb->publish_frame) pb->publish_frame(frame);
  };
  out.publish_entities = [pa, pb](const std::shared_ptr<const Frame> &frame,
                                  const std::shared_ptr<const TelemetryRates> &rates) {
    if (pa->publish_entities) pa->publish_entities(frame, rates);
    if (pb->publish_entities) pb->publish_entities(frame, rates);
  };
  out.start_outputs = [pa, pb] {
    if (pa->start_outputs) pa->start_outputs();
    if (pb->start_outputs) pb->start_outputs();
  };
  out.stop_outputs = [pa, pb] {
    if (pa->stop_outputs) pa->stop_outputs();
    if (pb->stop_outputs) pb->stop_outputs();
  };
  out.publication_status = [pa, pb] {
    Json out_json = Json::object();
    if (pa->publication_status) out_json["primary"] = pa->publication_status();
    if (pb->publication_status) out_json["secondary"] = pb->publication_status();
    return out_json;
  };
  out.facts = [pa, pb] {
    Json out_json = Json::object();
    if (pa->facts) {
      const Json fa = pa->facts();
      for (auto &[k, v] : fa.items()) out_json[k] = v;
    }
    if (pb->facts) {
      const Json fb = pb->facts();
      for (auto &[k, v] : fb.items()) out_json[k] = v;
    }
    return out_json;
  };
  return out;
}

} // namespace xsim
