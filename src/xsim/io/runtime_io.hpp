#pragma once
#include "config.hpp"
#include "core/world.hpp"
#include "telemetry_rates.hpp"
#include <functional>

namespace xsim {
// One optional IO attachment per existing Entity. Its callbacks own the concrete
// transport resources; there is no second entity roster or lifecycle authority.
struct EntityIO {
  std::function<void()> reconcile;
  std::function<void(const State &, const TelemetryRates &)> publish;
  std::function<bool()> publish_sensor;
  const std::string localization_pose_topic{};
};

// Main composes the optional ROS boundary. Empty hooks give the same native
// server and model/sensor systems a transport-free, headless runtime.
struct RuntimeIO {
  std::function<void(const std::shared_ptr<Entity> &, const Json &)> attach_entity;
  std::function<void()> poll_inputs, poll_services;
  std::function<bool()> okay;
  std::function<void(const Frame &)> publish_frame;
  std::function<void(const std::shared_ptr<const Frame> &,
                     const std::shared_ptr<const TelemetryRates> &)> publish_entities;
  std::function<void()> start_outputs, stop_outputs;
  std::function<Json()> publication_status;
  // Facts about the IO dependencies of this world for the readiness contract (describe `facts`).
  std::function<Json()> facts;
};
} // namespace xsim
