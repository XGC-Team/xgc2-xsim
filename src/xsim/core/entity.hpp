#pragma once
#include "components.hpp"
#include <utility>

namespace xsim {
struct EntityIO;
struct Sensor;
struct Entity {
  // `identity` is the name the management API and the HOLD roster know the entity by; the entity name when
  // empty (headless worlds and fixtures).
  explicit Entity(Config c, std::string identity = {})
      : config(std::move(c)), public_id(identity.empty() ? config.name : std::move(identity)) {}
  const Config config;
  const std::string public_id;
  uint64_t id = 0;
  // Written only at world boundaries; callback/output projections use atomics.
  std::atomic<uint64_t> generation{0};
  std::atomic<bool> alive{false}, enabled{false};
  std::atomic<int64_t> generation_stamp{0};
  std::shared_ptr<EntityIO> io;
  std::shared_ptr<Sensor> sensor;
};
} // namespace xsim
