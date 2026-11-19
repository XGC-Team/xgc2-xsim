#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>

namespace xsim {
enum class TelemetryGroup : size_t { Localization, Local, Imu, RawImu, State, ExtendedState, Target, Count };

struct TelemetryRates {
  static constexpr size_t count = size_t(TelemetryGroup::Count);
  inline static constexpr std::array<const char *, count> names{
      "localization", "local", "imu", "imu_raw", "state", "extended_state", "target"};
  std::array<double, count> hz{125, 100, 30, 30, 1, 1, 10};
  std::array<int64_t, count> periods{8000000, 10000000, 33333333, 33333333,
                                     1000000000, 1000000000, 100000000};

  TelemetryRates patched(const nlohmann::json &values) const {
    if (!values.is_object()) throw std::invalid_argument("rates_hz must be an object");
    auto result = *this;
    for (auto i = values.begin(); i != values.end(); ++i) {
      const auto name = std::find_if(names.begin(), names.end(),
          [&](const char *key) { return i.key() == key; });
      if (name == names.end()) throw std::invalid_argument("unknown telemetry rate: " + i.key());
      if (!i.value().is_number()) throw std::invalid_argument("telemetry rates must be numbers");
      const double rate = i.value().get<double>();
      if (!std::isfinite(rate) || rate < 0 || rate > 1000)
        throw std::invalid_argument("telemetry rates require finite 0..1000 Hz");
      const size_t index = size_t(name - names.begin());
      const double period = rate ? 1e9 / rate : 0;
      if (!std::isfinite(period) || period >= double(std::numeric_limits<int64_t>::max()))
        throw std::invalid_argument("telemetry period is not representable");
      result.hz[index] = rate;
      result.periods[index] = rate ? std::max<int64_t>(1, std::llround(period)) : 0;
    }
    return result;
  }
  nlohmann::json json() const {
    auto result = nlohmann::json::object();
    for (size_t i = 0; i < count; ++i) result[names[i]] = hz[i];
    return result;
  }
  nlohmann::json caps(int64_t snapshot_period) const {
    auto result = nlohmann::json::object();
    const double source = 1e9 / double(snapshot_period);
    for (size_t i = 0; i < count; ++i) result[names[i]] = std::min(hz[i], source);
    return result;
  }
};

// Per-entity deadlines share the world's scientific epoch. No backfill.
class TelemetrySchedule {
public:
  explicit TelemetrySchedule(int64_t epoch = 0) : epoch_(epoch) {
    if (epoch < 0) throw std::invalid_argument("telemetry epoch must be nonnegative");
  }
  void reset() { next_.fill(0); periods_.fill(-1); last_.fill(-1); }
  bool due(TelemetryGroup group, int64_t stamp, const TelemetryRates &rates) {
    const size_t i = size_t(group);
    if (stamp <= last_[i]) return false;
    const auto period = rates.periods[i];
    if (periods_[i] != period) { periods_[i] = period; next_[i] = stamp; }
    if (!period || stamp < next_[i]) return false;
    last_[i] = stamp;
    auto phase = (stamp - epoch_) % period;
    if (phase < 0) phase += period;
    const auto remaining = period - phase;
    const auto limit = std::numeric_limits<int64_t>::max();
    next_[i] = stamp > limit - remaining ? limit : stamp + remaining;
    return true;
  }
private:
  const int64_t epoch_;
  std::array<int64_t, TelemetryRates::count> next_{};
  std::array<int64_t, TelemetryRates::count> periods_{-1, -1, -1, -1, -1, -1, -1};
  std::array<int64_t, TelemetryRates::count> last_{-1, -1, -1, -1, -1, -1, -1};
};
} // namespace xsim
