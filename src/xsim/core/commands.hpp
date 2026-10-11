#pragma once
#include "entity.hpp"
#include "systems/robots.hpp"
#include <condition_variable>
#include <mutex>

namespace xsim {
struct Prepared {
  std::shared_ptr<Entity> entity;
  Model model;
};
enum class Op {
  Add,
  Remove,
  Pause,
  Resume,
  Step,
  Reset,
  Provider,
  SetEnabled,
  Arm,
  Mode,
  Pva,
  Attitude,
  Velocity
};
struct Result {
  bool applied = false, success = false, enabled = false, paused = false;
  // Stop may interrupt an already claimed Step. Only that terminal path uses
  // applied as an effects flag; ordinary results retain the ROS boundary-
  // processed meaning. completed_steps counts this Step's actual progress.
  bool interrupted = false;
  uint64_t completed_steps = 0;
  // Optional management result, captured by the world owner before completion.
  // It retains no strong ownership of an entity's IO or sensor resources.
  bool has_state = false;
  State state;
  uint32_t reason = 0;
  Key key;
  uint64_t step = 0;
  int64_t stamp = 0;
};
struct Command {
  Op op = Op::Pause;
  Key key;
  int action = 0;
  bool arm = false;
  bool enabled = false;
  bool capture_state = false, capture_states = false;
  // The management caller reserves capacity before submission. The world
  // fills this only when capture_states is requested, without allocating.
  std::vector<State> states;
  std::string mode;
  uint64_t steps = 1;
  uint64_t starting_step = 0; // world-owned, assigned when Step is claimed
  int64_t at = 0;
  int64_t arrival_ns = 0; // realtime arrival guard; not a coalescing/event key
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()}; // forward,left,yaw rate
  // Velocity commands: when the producer received the command, on the HOLD domain clock
  // (World::hold().now(), not simulation time). A command received before the last release of its entity
  // is dropped when it would be applied, so a stale command never replays.
  int64_t received_ns = 0;
  PositionTarget pva{};
  FlightAttitudeSetpoint attitude;
  std::unique_ptr<Prepared> prepared;
  std::shared_ptr<Entity> retired;
  std::vector<std::pair<Key, Model>> resets;
  Clock::time_point deadline = Clock::now() + std::chrono::milliseconds(1500);
  // 0 queued; 1 claimed; 2 completed; 3 cancelled before application;
  // 4 failed (including an interrupted Step with explicit partial effects).
  // Cancellation wins before claim, or waits for the actual boundary result.
  std::atomic<int> phase{0};
  Result result;
  void (*notification)(void*) noexcept = nullptr;
  void* notification_context = nullptr;
  void signal() const noexcept { if (notification) notification(notification_context); }
  std::mutex mutex;
  std::condition_variable done;
};
using Ticket = std::shared_ptr<Command>;
} // namespace xsim
