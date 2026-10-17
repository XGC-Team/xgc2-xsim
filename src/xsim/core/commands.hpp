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
struct Command : ControlSample {
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
  std::unique_ptr<Prepared> prepared;
  std::shared_ptr<Entity> retired;
  std::vector<std::pair<Key, Model>> resets;
  // Separated by the producer, the world thread, and the waiting caller.
  Clock::time_point queued_at{};
  Clock::time_point applied_at{};
  Clock::time_point feedback_at{};
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
