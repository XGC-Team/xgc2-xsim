#pragma once
#include "models/vehicle_model.hpp"
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace xsim {
using Clock = std::chrono::steady_clock;
using namespace xgc_lightweight;
enum class Kind { FS150, Scout, Mecanum };
struct Key {
  uint64_t id = 0, generation = 0;
};
struct Config {
  std::string name;
  Kind kind = Kind::FS150;
  Eigen::Vector3d initial{Eigen::Vector3d::Zero()},
      local_origin{Eigen::Vector3d::Zero()};
  double yaw = 0, ground_z = 0;
  FlightControllerParameters fcu;
};
struct Entity;
struct State {
  // Snapshots observe lifecycle; they must not keep removed IO/sensor
  // resources alive.
  std::weak_ptr<Entity> entity;
  Key key;
  int64_t stamp = 0;
  Eigen::Vector3d position{Eigen::Vector3d::Zero()},
      velocity{Eigen::Vector3d::Zero()}, omega{Eigen::Vector3d::Zero()},
      specific_force{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  FlightControlOutput control;
  Eigen::Vector4d rotors{Eigen::Vector4d::Zero()};
  bool enabled = false, armed = false, landed = true;
  std::array<char, 24> mode{};
};
struct Frame {
  int64_t stamp = 0;
  // Geometry tokens cover body key/generation/enabled/position, not time.
  // Zero denotes an ad-hoc fixture snapshot without an authoritative token.
  uint64_t steps = 0, revision = 0, geometry_revision = 0;
  std::vector<State> states;
};
struct Metrics {
  std::atomic<uint64_t> steps{0}, output_misses{0}, input_misses{0}, frame_array_grows{0};
  std::atomic<uint64_t> hold_refused{0}; // velocity commands refused by chassis HOLD
  std::atomic<int64_t> sim_ns{0}, lag_ns{0}, step_latency_ns{0},
      max_step_latency_ns{0};
  std::atomic<int64_t> last_dt_ns{0}, scheduling_period_ns{2000000},
      realtime_ns{0}, active_wall_ns{0};
  std::atomic<uint64_t> clock_degradations{0};
  std::atomic<bool> paused{false};
};
} // namespace xsim
