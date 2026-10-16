#pragma once
#include <algorithm>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace xsim {
// Absent phase keeps the process epoch. A present phase is the sampling origin
// and is not a replacement for that epoch.
inline int64_t sampling_phase_ns(const int64_t *phase_t0_ns, int64_t epoch_ns) {
  if (!phase_t0_ns) return epoch_ns;
  if (*phase_t0_ns < 0) throw std::invalid_argument("phase T0 must be nonnegative");
  return *phase_t0_ns;
}
// Without phase_origin, the next deadline stays on the previous sample's grid.
// With it, the next deadline is the following T0+kP and does not fill the gap.
inline int64_t observation_deadline(int64_t previous, int64_t stamp, int64_t period,
                                    const int64_t *phase_origin = nullptr) {
  int64_t remaining;
  if (phase_origin) {
    auto phase = (stamp - *phase_origin) % period;
    if (phase < 0) phase += period;
    remaining = period - phase;
  } else {
    if (!previous) previous = stamp;
    remaining = period - (stamp - previous) % period;
  }
  const auto limit = std::numeric_limits<int64_t>::max();
  return stamp > limit - remaining ? limit : stamp + remaining;
}
// Fixed-quality scans run as best-effort observations. Increase their interval
// under queue/output pressure, then recover slowly after sustained headroom.
// This state is touched only while the owning Sensor mutex is held.
struct AdaptiveCadence {
  int64_t period = 100000000, effective = 100000000, latency = 0;
  int64_t ceiling = 1000000000, recovery_at = 0;
  static constexpr int64_t recovery_window_ns = 250000000;

  explicit AdaptiveCadence(int64_t requested = 100000000)
      : period(requested), effective(requested) {
    const auto limit = std::numeric_limits<int64_t>::max();
    ceiling = std::max<int64_t>(1000000000, requested > limit / 8 ? limit : requested * 8);
  }

  int64_t recover(int64_t wall_ns) {
    if (wall_ns >= recovery_at && effective > period) {
      effective -= std::max<int64_t>(1, (effective - period) / 8);
      effective = std::max(period, effective);
      recovery_at = observation_deadline(0, wall_ns, recovery_window_ns);
    }
    return effective;
  }

  int64_t observe(int64_t elapsed, bool overloaded, int64_t wall_ns) {
    elapsed = std::max<int64_t>(1, elapsed);
    latency = latency ? latency - latency / 8 + elapsed / 8 : elapsed;
    const auto limit = std::numeric_limits<int64_t>::max();
    const int64_t budget = latency > limit - latency / 4
                               ? limit
                               : latency + latency / 4;
    if (overloaded || budget > effective) {
      const auto increase = std::max<int64_t>(1, effective / 4);
      const int64_t backoff = effective > limit - increase ? limit : effective + increase;
      effective = std::min(ceiling, std::max(period, std::max(budget, backoff)));
      recovery_at = observation_deadline(0, wall_ns, recovery_window_ns);
    } else recover(wall_ns);
    return effective;
  }
};
} // namespace xsim
