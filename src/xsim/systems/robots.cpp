#include "robots.hpp"
namespace xsim {
Model prepare_model(const Config &c) {
  if (c.kind == Kind::FS150)
    return Flight(c);
  if (c.kind == Kind::Scout)
    return Scout(c);
  return Mecanum(c);
}
void step_robot(Flight &f, bool enabled, double h) {
  if (enabled) {
    if (f.model.step(h) == FlightEvent::OffboardLost)
      f.set_mode("AUTO.LOITER");
  } else if (f.ever_started)
    f.model.stepPhysicsOnly(h);
}
void step_robot(Scout &s, int64_t dt) {
  if (dt < 0 || dt > INT64_MAX - s.age_ns)
    throw std::invalid_argument("scout model: invalid elapsed time");
  const auto next_ns = s.age_ns + dt;
  const double next_age = double(next_ns) * 1e-9;
  s.model.advance(next_age);
  s.age_ns = next_ns;
  s.age = next_age;
}
void step_robot(Mecanum &m, double h) { m.model.step(h); }
} // namespace xsim
