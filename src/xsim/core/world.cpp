#include "world.hpp"
#include <cmath>
#include <cstring>
#include <limits>
#include <pthread.h>
namespace xsim {
namespace {
bool continuous(Op op) {
  return op == Op::Pva || op == Op::Attitude || op == Op::Velocity;
}
xgc2::chassis_hold::DomainOptions hold_options(std::string instance) {
  xgc2::chassis_hold::DomainOptions options;
  options.instance_id = std::move(instance);
  return options;
}
} // namespace
World::World(int64_t e, int64_t d, int64_t o, unsigned c, int64_t maximum,
             std::string instance)
    : epoch(e), dt(d), output_period(o), hold_(hold_options(std::move(instance))),
      time_(e), next_output_(e), catchup_(c), physics_clock_(d, maximum) {
  if (e <= 0 || d <= 0 || o <= 0 || d > INT64_MAX-e || !c || e > INT64_MAX - o)
    throw std::invalid_argument(
        "explicit positive session epoch, step and output period required");
  metrics.sim_ns = e;
  metrics.scheduling_period_ns = d;
  next_output_ = epoch + output_period;
  for (auto &frame : frame_pool_) frame = std::make_shared<Frame>();
}
World::~World() { stop(); }
void World::submit(const Ticket &t) {
  // Arrival is a not-before guard, not a separate world integration event.
  if (continuous(t->op) && running_ && !metrics.paused) {
    const auto wall = std::chrono::duration_cast<std::chrono::nanoseconds>(
                          Clock::now().time_since_epoch()).count();
    const auto offset = wall_offset_ns_.load();
    t->arrival_ns = offset > INT64_MAX - wall ? INT64_MAX : wall + offset;
  }
  {
    std::lock_guard<std::mutex> l(input_mutex_);
    bool replaced = false;
    if (continuous(t->op)) {
      for (auto i = inbox_.rbegin(); i != inbox_.rend(); ++i) {
        if (!continuous((*i)->op)) break;
        if ((*i)->key.id == t->key.id &&
            (*i)->key.generation == t->key.generation && (*i)->op == t->op &&
            (*i)->at == t->at) {
          (*i)->phase = 3;
          *i = t;
          ++metrics.input_misses;
          replaced = true;
          break;
        }
      }
    }
    if (!replaced) inbox_.push_back(t);
  }
  wake();
}
void World::wake() {
  // What the caller published (an inbox entry, a HOLD state change) precedes the wake revision, which is taken
  // under the wake mutex.
  {
    std::lock_guard<std::mutex> l(wake_mutex_);
    ++wake_revision_;
  }
  wake_.notify_one();
}
bool World::wait(const Ticket &t) {
  std::unique_lock<std::mutex> l(t->mutex);
  while (t->phase.load(std::memory_order_acquire) < 2) {
    if (Clock::now() >= t->deadline) {
      int queued = 0;
      if (t->phase.compare_exchange_strong(queued, 3))
        return false;
      // Claimed work has won the race: await its actual result. World never
      // takes this caller-owned mutex. Short timed waits also cover a notify
      // between the atomic predicate read and condition-variable sleep.
    }
    t->done.wait_for(l, std::chrono::milliseconds(1));
  }
  return t->phase.load(std::memory_order_acquire) == 2;
}
void World::finish(const Ticket &t, Result r) {
  const bool capture = r.success || r.interrupted;
  if (capture && t->capture_state && !r.has_state && r.key.id) {
    const auto found = slots_.find(r.key.id);
    if (found != slots_.end()) {
      r.state = state(found->first, found->second);
      r.has_state = true;
    }
  }
  if (t->capture_states) {
    t->states.clear();
    // A delayed Step can finish after membership changes. Never allocate on
    // the world thread if its caller supplied insufficient result capacity.
    if (capture && t->states.capacity() < slots_.size()) {
      r.success = false;
      r.reason = 5;
    } else if (capture) {
      for (const auto &entry : slots_)
        t->states.push_back(state(entry.first, entry.second));
    }
  }
  if (!r.interrupted) r.applied = true;
  r.stamp = time_;
  r.step = steps_;
  r.paused = metrics.paused;
  const int terminal_phase = r.interrupted ? 4 : 2;
  t->result = std::move(r);
  t->phase.store(terminal_phase, std::memory_order_release);
  t->done.notify_all();
  t->signal();
}
void World::cancel_pending_controls(Key key) {
  const auto cancel = [key](const Ticket &ticket) {
    if (!continuous(ticket->op) || ticket->key.id != key.id ||
        ticket->key.generation != key.generation)
      return;
    int queued = 0;
    if (ticket->phase.compare_exchange_strong(queued, 3)) {
      ticket->done.notify_all();
      ticket->signal();
    }
  };
  for (const auto &ticket : commands_) cancel(ticket);
  std::lock_guard<std::mutex> lock(input_mutex_);
  for (const auto &ticket : inbox_) cancel(ticket);
}
void World::zero_command(const Entity &e, size_t i) {
  if (e.config.kind == Kind::Scout)
    scouts_[i].model.command(scouts_[i].age, 0, 0);
  else if (e.config.kind == Kind::Mecanum)
    mecanums_[i].model.command(0, 0, 0);
}
void World::reset(Slot &s, Model &&m) {
  auto &e = *s.entity;
  const auto i = s.dense;
  if (e.config.kind == Kind::FS150) {
    flights_[i] = std::get<Flight>(std::move(m));
    flights_[i].model.bind(bodies_, i);
    flights_[i].ever_started = e.enabled;
  } else if (e.config.kind == Kind::Scout) {
    scouts_[i] = std::get<Scout>(std::move(m));
    scouts_[i].model.bind(scout_poses_, i);
  } else {
    mecanums_[i] = std::get<Mecanum>(std::move(m));
    mecanums_[i].model.bind(mecanum_poses_, i);
  }
  ++e.generation;
  e.generation_stamp = time_;
}
Result World::apply(Command &c) {
  Result r;
  r.success = true;
  if (c.capture_states) {
    c.states.clear();
    const auto required = slots_.size() + (c.op == Op::Add ? 1 : 0);
    if (c.states.capacity() < required) {
      r.success = false;
      r.reason = 5;
      return r;
    }
  }
  if (c.op == Op::Pause) {
    metrics.paused = true;
    return r;
  }
  if (c.op == Op::Resume) {
    if (stepping_) { r.success = false; r.reason = 2; return r; }
    if (!metrics.paused) return r;
    const auto wall = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
    // Rebase both clocks at the transition, including Pause/Resume commands
    // consumed in one boundary before run() can observe the paused state.
    physics_clock_.rebase(wall, time_);
    wall_offset_ns_ = time_ - wall;
    metrics.paused = false;
    return r;
  }
  if (c.op == Op::Step) {
    r.success = metrics.paused && c.steps > 0 && stepping_ == 0 && c.steps<=uint64_t((INT64_MAX-time_)/dt);
    if (r.success) {
      c.starting_step = steps_;
      stepping_ = c.steps;
    } else
      r.reason = 2;
    return r;
  }
  if (c.op == Op::Add) {
    auto &p = *c.prepared;
    auto e = p.entity;
    for (const auto &s : slots_)
      if (s.second.entity->config.name == e->config.name) {
        r.success = false;
        r.reason = 4;
        return r;
      }
    // Scout and Mecanum entities are the HOLD roster. An ID of a removed entity gets its HOLD state back.
    const bool chassis = e->config.kind != Kind::FS150;
    if (chassis && !hold_.add(e->public_id, true)) {
      r.success = false;
      r.reason = 4;
      return r;
    }
    e->id = next_id_++;
    e->generation_stamp = time_;
    size_t i;
    if (e->config.kind == Kind::FS150) {
      i = flights_.size();
      bodies_.append(std::get<Flight>(p.model).model.body_state());
      flights_.push_back(std::get<Flight>(std::move(p.model)));
      flights_.back().model.bind(bodies_, i);
      flight_ids_.push_back(e->id);
      flight_enabled_.push_back(0);
    } else if (e->config.kind == Kind::Scout) {
      i = scouts_.size();
      scout_poses_.append(std::get<Scout>(p.model).model.pose());
      scouts_.push_back(std::get<Scout>(std::move(p.model)));
      scouts_.back().model.bind(scout_poses_, i);
      scout_ids_.push_back(e->id);
    } else {
      i = mecanums_.size();
      mecanum_poses_.append(std::get<Mecanum>(p.model).model.pose());
      mecanums_.push_back(std::get<Mecanum>(std::move(p.model)));
      mecanums_.back().model.bind(mecanum_poses_, i);
      mecanum_ids_.push_back(e->id);
    }
    slots_.emplace(e->id, Slot{e, i});
    if (chassis)
      hold_ids_.emplace(e->public_id, e->id);
    e->alive = true;
    geometry_dirty_ = true;
    reserve_frames();
    r.key = {e->id, e->generation};
    return r;
  }
  if (c.op == Op::Reset && !c.key.id) {
    if (c.action == 0 && c.resets.size() != slots_.size()) {
      r.success = false;
      r.reason = 1;
      return r;
    }
    for (auto &v : c.resets) {
      auto s = slots_.find(v.first.id);
      if (s == slots_.end() ||
          s->second.entity->generation != v.first.generation) {
        r.success = false;
        r.reason = 1;
        return r;
      }
    }
    for (auto &v : c.resets)
      reset(slots_.at(v.first.id), std::move(v.second));
    return r;
  }
  auto found = slots_.find(c.key.id);
  if (found == slots_.end()) {
    r.success = false;
    r.reason = 1;
    return r;
  }
  auto &s = found->second;
  auto &e = *s.entity;
  auto i = s.dense;
  r.key = {e.id, e.generation};
  r.enabled = e.enabled;
  const bool observe = c.op == Op::Provider && c.action == 0;
  const bool repeated_start = c.op == Op::Provider && c.action == 1 &&
                              e.enabled && e.generation > 0 &&
                              c.key.generation == e.generation - 1;
  if (!observe && !repeated_start && c.key.generation != e.generation) {
    r.success = false;
    r.reason = 1;
    return r;
  }
  if (c.op == Op::Remove) {
    c.retired = s.entity;
    e.alive = false;
    e.enabled = false;
    if (c.capture_state) {
      r.state = state(e.id, s);
      r.has_state = true;
    }
    auto erase = [&](auto &models, auto &ids, auto &columns) {
      if (i + 1 != models.size()) {
        models[i] = std::move(models.back());
        ids[i] = ids.back();
        slots_.at(ids[i]).dense = i;
        models[i].model.rebind(i);
      }
      columns.erase(i);
      models.pop_back();
      ids.pop_back();
    };
    if (e.config.kind == Kind::FS150) {
      flight_enabled_[i] = flight_enabled_.back();
      flight_enabled_.pop_back();
      erase(flights_, flight_ids_, bodies_);
    }
    else if (e.config.kind == Kind::Scout)
      erase(scouts_, scout_ids_, scout_poses_);
    else
      erase(mecanums_, mecanum_ids_, mecanum_poses_);
    if (e.config.kind != Kind::FS150) {
      hold_ids_.erase(e.public_id);
      hold_.remove(e.public_id); // the HOLD state of the ID stays for a re-created entity
    }
    slots_.erase(found);
    geometry_dirty_ = true;
    r.enabled = false;
    return r;
  }
  if (c.op == Op::Reset) {
    reset(s, std::move(c.prepared->model));
    r.key.generation = e.generation;
    return r;
  }
  if (c.op == Op::SetEnabled) {
    if (!c.enabled) {
      cancel_pending_controls({e.id, e.generation});
      zero_command(e, i);
    }
    e.enabled = c.enabled;
    if (e.config.kind == Kind::FS150) {
      flight_enabled_[i] = c.enabled;
      if (c.enabled) flights_[i].ever_started = true;
    }
    // Unlike the legacy Provider start, this does not reconstruct a model,
    // change its generation, or reset its physical/controller state.
    r.enabled = c.enabled;
    return r;
  }
  if (c.op == Op::Provider) {
    if (c.action == 1 && !e.enabled) {
      reset(s, std::move(c.prepared->model));
      e.enabled = true;
      if (e.config.kind == Kind::FS150)
        flights_[i].ever_started = true;
    } else if (c.action == 2) {
      e.enabled = false;
      zero_command(e, i);
    } else if (c.action < 0 || c.action > 2) {
      r.success = false;
      r.reason = 2;
    }
    if (e.config.kind == Kind::FS150)
      flight_enabled_[i] = e.enabled;
    r.key.generation = e.generation;
    r.enabled = e.enabled;
    return r;
  }
  if (!e.enabled) {
    r.success = false;
    r.reason = 2;
    return r;
  }
  if (c.op == Op::Velocity && e.config.kind != Kind::FS150) {
    if (!c.velocity.allFinite()) {
      r.success = false;
      r.reason = 2;
      return r;
    }
    // The HOLD gate sits where the command would reach the model: a command that was queued before an
    // engage, or received before the last release, never executes. It is separate from `enabled`: a held
    // entity keeps its sensors and publication.
    if (!hold_.admit(e.public_id, c.received_ns)) {
      ++metrics.hold_refused;
      r.success = false;
      r.reason = 6;
      return r;
    }
    if (e.config.kind == Kind::Scout)
      scouts_[i].model.command(scouts_[i].age, c.velocity.x(), c.velocity.z());
    else
      mecanums_[i].model.command(c.velocity.x(), c.velocity.y(),
                                 c.velocity.z());
    return r;
  }
  if (e.config.kind != Kind::FS150) {
    r.success = false;
    r.reason = 3;
    return r;
  }
  auto &f = flights_[i];
  if (c.op == Op::Arm && c.action != 0) {
    r.success = false;
    r.reason = 3;
    return r;
  }
  if (c.op == Op::Arm)
    r.success = f.model.request_arm(c.arm);
  else if (c.op == Op::Mode) {
    FlightMode mode;
    if (c.mode == "OFFBOARD")
      mode = FlightMode::Offboard;
    else if (c.mode == "POSCTL" || c.mode == "ALTCTL" ||
             c.mode == "AUTO.LOITER")
      mode = FlightMode::Hold;
    else if (c.mode == "AUTO.LAND")
      mode = FlightMode::Land;
    else {
      r.success = false;
      r.reason = 3;
      return r;
    }
    r.success = f.model.request_mode(mode);
    if (r.success)
      f.mode = c.mode;
  } else if (c.op == Op::Pva) {
    // ROS local position coordinates differ by the declared origin translation.
    auto p = c.pva;
    if (p.coordinate_frame == 1)
      for (int axis = 0; axis < 3; ++axis)
        p.position[axis] += e.config.local_origin[axis];
    r.success = f.model.setpoint(
        decodePositionTarget(p, f.model.orientation(), f.model.yaw()));
  } else if (c.op == Op::Attitude)
    r.success = f.model.attitude_setpoint(c.attitude);
  else {
    r.success = false;
    r.reason = 3;
  }
  if (!r.success && !r.reason)
    r.reason = 2;
  return r;
}
void World::boundary() {
  const auto now = Clock::now();
  {
    std::unique_lock<std::mutex> l(input_mutex_, std::try_to_lock);
    if (l.owns_lock())
      commands_.splice(commands_.end(), inbox_);
  }
  for (auto i = commands_.begin(); i != commands_.end();) {
    auto t = *i;
    if (std::max(t->at, t->arrival_ns) > time_ && continuous(t->op) && now < t->deadline &&
        t->phase == 0) {
      ++i;
      continue;
    }
    i = commands_.erase(i);
    int expected = 0;
    if (Clock::now() >= t->deadline) {
      t->phase.compare_exchange_strong(expected, 3);
      t->done.notify_all();
  t->signal();
      continue;
    }
    if (!t->phase.compare_exchange_strong(expected, 1))
      continue;
    Result r;
    try {
      r = apply(*t);
    } catch (const std::exception &) {
      r.success = false;
      r.reason = 5;
    }
    if (r.success)
      ++revision_;
    if (t->op == Op::Step && r.success)
      step_waiters_.push_back(t);
    else
      finish(t, r);
  }
  // The HOLD tick: after this boundary's commands, zero for every held entity, every boundary. The sink
  // records each entity's twist; reported after the zero write, it is the feedback behind `stopped`.
  hold_samples_.clear();
  hold_.tick(*this);
  if (!hold_samples_.empty()) {
    const auto stamp = hold_.now();
    for (const auto &sample : hold_samples_)
      hold_.observe(slots_.at(sample.id).entity->public_id, sample.linear,
                    sample.angular, stamp);
  }
}
bool World::write_zero(const std::string &public_id, std::string &error) {
  const auto roster = hold_ids_.find(public_id);
  if (roster == hold_ids_.end()) {
    error = "entity is not in the world";
    return false;
  }
  const auto &slot = slots_.at(roster->second);
  const auto &e = *slot.entity;
  // Zero replaces the model's command and drops the velocity commands still waiting for their time.
  cancel_pending_controls({e.id, e.generation});
  zero_command(e, slot.dense);
  if (e.config.kind == Kind::Scout) {
    const auto twist = scouts_[slot.dense].model.velocity();
    hold_samples_.push_back({e.id, std::abs(twist.linear_m_s), std::abs(twist.yaw_rad_s)});
  } else {
    const auto &m = mecanums_[slot.dense].model;
    hold_samples_.push_back({e.id, m.body_velocity().norm(), std::abs(m.yaw_rate())});
  }
  return true;
}
void World::advance() { advance(dt); }
void World::advance(int64_t elapsed_ns) {
  if (elapsed_ns < PhysicsClock::minimum_step || elapsed_ns > 20000000)
    throw std::invalid_argument("integration duration must be 1 us..20 ms");
  const auto begin = Clock::now();
  if (elapsed_ns > INT64_MAX - time_) { metrics.paused = true; return; }
  const double h = double(elapsed_ns) * 1e-9;
  for (size_t i = 0; i < flights_.size(); ++i) {
    auto &f = flights_[i];
    step_robot(f, flight_enabled_[i], h);
  }
  for (auto &s : scouts_)
    step_robot(s, elapsed_ns);
  for (auto &m : mecanums_)
    step_robot(m, h);
  ++steps_;
  time_ += elapsed_ns;
  metrics.last_dt_ns = elapsed_ns;
  metrics.steps = steps_;
  metrics.sim_ns = time_;
  if (time_ >= next_output_) {
    emit();
    const auto next_index=(time_-epoch)/output_period+1;
    next_output_=next_index>(INT64_MAX-epoch)/output_period?INT64_MAX:epoch+next_index*output_period;
  }
  if (stepping_ && --stepping_ == 0) {
    for (auto &t : step_waiters_) {
      Result r;
      r.success = true;
      r.completed_steps = steps_ - t->starting_step;
      finish(t, r);
    }
    step_waiters_.clear();
    emit();
  }
  const auto elapsed =
      std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - begin)
          .count();
  metrics.step_latency_ns = elapsed;
  if (elapsed > metrics.max_step_latency_ns)
    metrics.max_step_latency_ns = elapsed;
}
State World::state(uint64_t id, const Slot &s) const {
  State v;
  v.entity = s.entity;
  auto &e = *s.entity;
  v.key = {id, e.generation};
  v.stamp = time_;
  v.enabled = e.enabled;
  const auto i = s.dense;
  if (e.config.kind == Kind::FS150) {
    const auto &f = flights_[i];
    v.position = f.model.state().position;
    v.velocity = f.model.state().velocity;
    v.orientation = f.model.orientation();
    v.omega = f.model.angular_velocity_body();
    v.specific_force = f.model.specific_force_body();
    v.armed = f.model.armed();
    v.landed = f.model.landed();
    v.control = f.model.control_output();
    v.rotors = f.model.rotor_speed();
    std::strncpy(v.mode.data(), f.mode.c_str(), v.mode.size() - 1);
  } else {
    auto pose = e.config.kind == Kind::Scout ? scouts_[i].model.pose()
                                             : mecanums_[i].model.pose();
    Eigen::Vector2d body;
    if (e.config.kind == Kind::Scout) {
      auto u = scouts_[i].model.velocity();
      body = {u.linear_m_s, 0};
      v.omega.z() = u.yaw_rad_s;
      v.specific_force = scouts_[i].model.specific_force_body();
    } else {
      body = mecanums_[i].model.body_velocity();
      v.omega.z() = mecanums_[i].model.yaw_rate();
      v.specific_force = mecanums_[i].model.specific_force_body();
    }
    v.position = {pose.position.x(), pose.position.y(), e.config.initial.z()};
    v.orientation = Eigen::AngleAxisd(pose.yaw, Eigen::Vector3d::UnitZ());
    v.velocity.head<2>() = xgc2_math::rotationMatrix2(pose.yaw) * body;
  }
  return v;
}
Frame World::capture() const {
  Frame f;
  f.states.reserve(slots_.size());
  f.stamp = time_;
  f.steps = steps_;
  f.revision = revision_;
  // An ad-hoc capture has no authoritative emitted-geometry token. The legacy
  // sensor fixture entry point computes its own exact geometry observation.
  for (const auto &s : slots_)
    f.states.push_back(state(s.first, s.second));
  return f;
}
void World::reserve_frames() {
  const size_t required = slots_.size();
  for (auto &frame : frame_pool_)
    if (frame.unique() && frame->states.capacity() < required) {
      // libstdc++ unique() reads its reference count relaxed. Pair the observed
      // final consumer release with an acquire before mutating reusable storage.
      std::atomic_thread_fence(std::memory_order_acquire);
      frame->states.reserve(std::max(required, frame->states.capacity() * 2));
      ++metrics.frame_array_grows;
    }
  if (geometry_.capacity() < required)
    geometry_.reserve(std::max(required, geometry_.capacity() * 2));
}
void World::observe_geometry(Frame &frame) {
  bool changed = geometry_dirty_ || geometry_.size() != frame.states.size();
  if (!changed)
    for (size_t i = 0; i < geometry_.size(); ++i) {
      const auto &previous = geometry_[i];
      const auto &current = frame.states[i];
      if (previous.key.id != current.key.id ||
          previous.key.generation != current.key.generation ||
          previous.enabled != current.enabled ||
          !(previous.position.array() == current.position.array()).all()) {
        changed = true;
        break;
      }
    }
  if (changed) {
    ++geometry_revision_;
    geometry_.resize(frame.states.size());
    for (size_t i = 0; i < geometry_.size(); ++i) {
      const auto &current = frame.states[i];
      geometry_[i] = {current.key, current.enabled, current.position};
    }
    geometry_dirty_ = false;
  }
  frame.geometry_revision = geometry_revision_;
}
void World::emit() {
  if (emitted_stamp_ == time_ && emitted_revision_ == revision_)
    return;
  Frame *frame = nullptr;
  size_t index = 0;
  for (size_t offset = 0; offset < frame_pool_size; ++offset) {
    index = (frame_cursor_ + offset) % frame_pool_size;
    if (frame_pool_[index].unique()) {
      std::atomic_thread_fence(std::memory_order_acquire);
      frame = frame_pool_[index].get();
      break;
    }
  }
  if (!frame) {
    ++metrics.output_misses;
    return;
  }
  // A slot retained across a topology change catches up once after release.
  // Ordinary frames only resize within previously reserved storage.
  if (frame->states.capacity() < slots_.size()) {
    frame->states.reserve(std::max(slots_.size(), frame->states.capacity() * 2));
    ++metrics.frame_array_grows;
  }
  frame->states.resize(slots_.size());
  size_t at = 0;
  for (const auto &s : slots_) frame->states[at++] = state(s.first, s.second);
  frame->stamp = time_;
  frame->steps = steps_;
  frame->revision = revision_;
  observe_geometry(*frame);
  std::unique_lock<std::mutex> l(output_mutex_, std::try_to_lock);
  if (!l.owns_lock()) {
    ++metrics.output_misses;
    return;
  }
  if (ready_) ++metrics.output_misses;
  ready_ = frame_pool_[index];
  frame_cursor_ = (index + 1) % frame_pool_size;
  emitted_revision_ = revision_;
  emitted_stamp_ = time_;
}
bool World::take_frame(std::shared_ptr<const Frame> &f) {
  std::lock_guard<std::mutex> l(output_mutex_);
  if (!ready_) return false;
  f = std::move(ready_);
  return true;
}
bool World::take_frame(Frame &f) {
  std::shared_ptr<const Frame> shared;
  if (!take_frame(shared)) return false;
  f = *shared;
  return true;
}
void World::start() {
  std::unique_lock<std::mutex> lock(wake_mutex_);
  if (!running_) {
    if (thread_.joinable())
      throw std::logic_error("previous world thread must be stopped before restart");
    started_ = false;
    startup_error_ = nullptr;
    running_ = true;
    try {
      thread_ = std::thread([this] { run(); });
    } catch (...) {
      running_ = false;
      throw;
    }
  }
  started_wake_.wait(lock, [this] { return started_ || startup_error_ || !running_; });
  if (startup_error_) std::rethrow_exception(startup_error_);
  if (!started_) throw std::runtime_error("world stopped before native startup completed");
}
void World::stop() {
  {
    std::lock_guard<std::mutex> l(wake_mutex_);
    running_ = false;
  }
  wake_.notify_all();
  started_wake_.notify_all();
  if (thread_.joinable())
    thread_.join();
  std::lock_guard<std::mutex> l(input_mutex_);
  commands_.splice(commands_.end(), inbox_);
  for (auto &t : commands_) {
    t->phase = 3;
    t->done.notify_all();
  t->signal();
  }
  commands_.clear();
  for (auto &t : step_waiters_) {
    Result result;
    result.interrupted = true;
    result.completed_steps = steps_ - t->starting_step;
    result.applied = result.completed_steps != 0;
    finish(t, std::move(result));
  }
  step_waiters_.clear();
  stepping_ = 0;
}
void World::run() {
  pthread_setname_np(pthread_self(), "xsim-world");
  const auto nanoseconds = [](Clock::time_point p) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(p.time_since_epoch()).count();
  };
  auto wall = nanoseconds(Clock::now());
  bool was_paused = metrics.paused;
  bool catching_up = false;
  try {
    physics_clock_.rebase(wall, time_);
    wall_offset_ns_ = time_ - wall;
    boundary();
    // Every start must publish this native generation's first frame, including
    // a restart at an unchanged simulation time. A saturated output pool must
    // not be reported as initialized merely because its thread exists.
    emitted_stamp_ = -1;
    emitted_revision_ = UINT64_MAX;
    emit();
    if (emitted_stamp_ != time_ || emitted_revision_ != revision_)
      throw std::runtime_error("initial world snapshot could not be published");
  } catch (...) {
    {
      std::lock_guard<std::mutex> lock(wake_mutex_);
      startup_error_ = std::current_exception();
      running_ = false;
    }
    started_wake_.notify_all();
    return;
  }
  {
    std::lock_guard<std::mutex> lock(wake_mutex_);
    started_ = true;
  }
  started_wake_.notify_all();
  while (running_) {
    // Capture the wake revision before draining the inbox.
    const auto wake_revision = wake_revision_.load();
    boundary();
    const auto begin = nanoseconds(Clock::now());
    if (metrics.paused) {
      metrics.lag_ns = 0;
      if (!was_paused) metrics.active_wall_ns += begin - wall;
      was_paused = true;
      catching_up = false;
      unsigned batch = 0;
      while (running_ && stepping_ && batch++ < catchup_) {
        advance();
        boundary();
      }
      emit();
      std::unique_lock<std::mutex> l(wake_mutex_);
      const auto changed = [&] { return !running_ || wake_revision_ != wake_revision; };
      if (stepping_) wake_.wait_for(l, std::chrono::microseconds(100), changed);
      else wake_.wait(l, changed);
      continue;
    }
    if (was_paused) {
      // Resume already set the shared anchor. Include any work after that
      // command in this boundary in the active-wall denominator as well.
      wall = time_ - wall_offset_ns_.load();
      was_paused = false;
    }
    if (!catching_up && begin < physics_clock_.next_wake()) {
      std::unique_lock<std::mutex> l(wake_mutex_);
      wake_.wait_until(l, Clock::time_point(std::chrono::nanoseconds(physics_clock_.next_wake())),
                      [&] { return !running_ || wake_revision_ != wake_revision; });
      continue;
    }
    const auto before = time_;
    unsigned batch = 0;
    while (running_ && !metrics.paused && batch++ < catchup_) {
      // A boundary can pause and resume within this batch. Re-read the clock
      // mapping so its old catch-up target cannot outlive the resume rebase.
      auto h = physics_clock_.step(physics_clock_.target(begin), time_);
      if (!h) break;
      advance(h);
      boundary();
      if (nanoseconds(Clock::now()) - begin >= 4000000) break;
    }
    const auto end = nanoseconds(Clock::now());
    const auto integrated = time_ - before;
    physics_clock_.observe(integrated, integrated ? end - begin : 0);
    physics_clock_.schedule(end);
    metrics.realtime_ns += integrated;
    metrics.active_wall_ns += end - wall;
    wall = end;
    metrics.scheduling_period_ns = physics_clock_.period();
    metrics.clock_degradations = physics_clock_.degradations();
    metrics.lag_ns = std::max<int64_t>(0, physics_clock_.target(end) - time_);
    std::unique_lock<std::mutex> l(wake_mutex_);
    const auto changed = [&] { return !running_ || wake_revision_ != wake_revision; };
    catching_up = physics_clock_.target(begin) - time_ >= PhysicsClock::minimum_step;
    if (catching_up) {
      // Catch-up batches wait 100 us, interruptible by stop.
      wake_.wait_for(l, std::chrono::microseconds(100), [&] { return !running_; });
    } else {
      wake_.wait_until(l, Clock::time_point(std::chrono::nanoseconds(physics_clock_.next_wake())), changed);
    }
  }
}
} // namespace xsim
