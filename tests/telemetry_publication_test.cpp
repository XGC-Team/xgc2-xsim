// Verifies that ROS telemetry work follows the due groups and that the local
// twist does not depend on localization publication. The benchmark mode is a
// development measurement, not a runtime gate.
#include "core/world.hpp"
#include "io/ros/entity.hpp"
#include <cassert>
#include <chrono>
#include <cstring>
#include <iostream>

using namespace xsim;

namespace {
constexpr int64_t epoch = 1700000000000000000LL;
constexpr int64_t frame_ns = 8000000;

struct Fixture {
  ros::CallbackQueue input, services;
  World world{epoch};
  std::shared_ptr<Entity> entity;
  std::shared_ptr<RosEntity> ros;

  Fixture(Kind kind, const char *name) {
    Config config;
    config.name = name;
    config.kind = kind;
    config.initial = {1.0, 2.0, 3.0};
    entity = std::make_shared<Entity>(config);
    entity->id = 1;
    entity->generation = 1;
    entity->generation_stamp = epoch;
    entity->alive.store(true);
    entity->enabled.store(true);
    ros = std::make_shared<RosEntity>(entity, world, input, services,
                                      Json::object());
    // No ROS master or publication is needed: publish() still constructs the
    // reusable messages, while an unbound BufferedPublisher stays inert.
    ros->bound.store(true, std::memory_order_release);
  }
};

State state_at(const std::shared_ptr<Entity> &entity, int64_t stamp) {
  State s;
  s.entity = entity;
  s.key = {entity->id, entity->generation.load()};
  s.stamp = stamp;
  s.enabled = true;
  s.armed = true;
  s.landed = false;
  s.position = {1.0, 2.0, 3.0};
  s.velocity = {4.0, 5.0, 6.0};
  s.omega = {0.10, 0.20, 0.30};
  s.orientation =
      Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ());
  s.specific_force = {0.0, 0.0, 9.8};
  std::strncpy(s.mode.data(), "OFFBOARD", s.mode.size() - 1);
  return s;
}

void disabled_localization_does_not_fill_messages() {
  Fixture fixture(Kind::Scout, "disabled_localization");
  TelemetryRates rates;
  rates.periods[size_t(TelemetryGroup::Localization)] = 0;
  rates.periods[size_t(TelemetryGroup::Local)] = 0;
  auto s = state_at(fixture.entity, epoch);
  for (int n = 0; n < 10; ++n) {
    s.stamp = epoch + int64_t(n) * frame_ns;
    fixture.ros->publish(s, rates);
  }
  assert(fixture.ros->localization_pose_message.pose.position.x == 0.0);
  assert(fixture.ros->localization_pose_message.pose.position.y == 0.0);
  assert(fixture.ros->localization_twist_message.twist.linear.x == 0.0);
  assert(fixture.ros->localization_twist_message.twist.linear.y == 0.0);
}

void local_twist_survives_disabled_localization() {
  Fixture fixture(Kind::FS150, "local_without_localization");
  TelemetryRates rates;
  rates.periods[size_t(TelemetryGroup::Localization)] = 0;
  rates.periods[size_t(TelemetryGroup::Local)] = 33333333;
  auto s = state_at(fixture.entity, epoch);
  fixture.ros->publish(s, rates);
  assert(fixture.ros->localization_pose_message.pose.position.x == 0.0);
  assert(fixture.ros->local_twist_message.twist.linear.x == 4.0);
  assert(fixture.ros->local_twist_message.twist.linear.y == 5.0);
  assert(fixture.ros->local_twist_message.twist.linear.z == 6.0);
  const Eigen::Vector3d angular = s.orientation * s.omega;
  assert(std::abs(fixture.ros->local_twist_message.twist.angular.x - angular.x()) < 1e-12);
  assert(std::abs(fixture.ros->local_twist_message.twist.angular.y - angular.y()) < 1e-12);
  assert(std::abs(fixture.ros->local_twist_message.twist.angular.z - angular.z()) < 1e-12);
  // The local pose and odometry path must still receive the same state.
  assert(std::abs(fixture.ros->local_pose_message.pose.position.x - 1.0) < 1e-12);
  assert(std::abs(fixture.ros->odometry_message.pose.pose.position.x - 1.0) < 1e-12);
}

void enabled_localization_still_fills_and_publishes() {
  Fixture fixture(Kind::FS150, "enabled_localization");
  TelemetryRates rates;
  rates.periods[size_t(TelemetryGroup::Localization)] = frame_ns;
  rates.periods[size_t(TelemetryGroup::Local)] = 0;
  auto s = state_at(fixture.entity, epoch);
  fixture.ros->publish(s, rates);
  assert(fixture.ros->localization_pose_message.pose.position.x == 1.0);
  assert(fixture.ros->localization_pose_message.pose.position.y == 2.0);
  assert(fixture.ros->localization_pose_message.pose.position.z == 3.0);
  assert(fixture.ros->localization_twist_message.twist.linear.x == 4.0);
  assert(fixture.ros->localization_twist_message.twist.linear.y == 5.0);
  assert(fixture.ros->localization_twist_message.twist.linear.z == 6.0);
}

double benchmark(int iterations, int64_t localization_period) {
  Fixture fixture(Kind::FS150, "benchmark");
  TelemetryRates rates;
  rates.periods[size_t(TelemetryGroup::Localization)] = localization_period;
  rates.periods[size_t(TelemetryGroup::Local)] = 33333333;
  auto s = state_at(fixture.entity, epoch);
  const auto started = Clock::now();
  for (int n = 0; n < iterations; ++n) {
    s.stamp = epoch + int64_t(n) * frame_ns;
    fixture.ros->publish(s, rates);
  }
  const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
      Clock::now() - started).count();
  return double(elapsed) / double(iterations);
}
} // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "xsim_telemetry_publication_fixture");
  if (argc > 1 && std::string(argv[1]) == "--benchmark") {
    const int iterations = argc > 2 ? std::stoi(argv[2]) : 2000000;
    const int64_t localization_period =
        argc > 3 ? std::stoll(argv[3]) : 0;
    const double ns = benchmark(iterations, localization_period);
    std::cout << Json({{"result", "BENCHMARK"},
                       {"iterations", iterations},
                       {"localization_period_ns", localization_period},
                       {"ns_per_publish", ns}})
                     .dump()
              << '\n';
    return 0;
  }
  disabled_localization_does_not_fill_messages();
  local_twist_survives_disabled_localization();
  enabled_localization_still_fills_and_publishes();
  std::cout << Json({{"result", "PASS"},
                     {"checks", "disabled localization fill, local twist, enabled localization"}})
                   .dump()
            << '\n';
}
