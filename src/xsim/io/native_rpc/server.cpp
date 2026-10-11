#include "server.hpp"
#include "io/startup.hpp"
#include <xgc2/xrpc/bounded_output.hpp>
#include <algorithm>
#include <iostream>
#include <optional>
#include <sstream>
#include <pthread.h>
#include <unordered_set>
#include <random>
#include <iomanip>
namespace xsim {
namespace {
bool safe_request_id(const std::string &text) {
  return !text.empty() && text.size() <= 128 &&
      std::all_of(text.begin(), text.end(), [](unsigned char c) {
        return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
               (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == ':';
      });
}
} // namespace

Server::Server(const Json &config, const std::string &path, RpcOptions options,
               World &world, Sensors &sensors, RuntimeIO io)
    : instance_(world.hold().instance_id()), socket_path_(path),
      target_id_(std::move(options.target_id)), limits_(options.limits),
      io_(std::move(io)), world_(world), sensors_(sensors) {
  if (!safe_request_id(target_id_))
    throw std::invalid_argument("explicit target_id required");
  telemetry_rates_ = std::make_shared<const TelemetryRates>(
      TelemetryRates{}.patched(config.value("telemetry_rates_hz", Json::object())));
  if (instance_.empty())
    throw std::invalid_argument("instance_id required");
  ensure_socket_parent(path);
  world_.metrics.paused = config.value("paused", false);
  input_poll_ns_=config.value("input_poll_ns",int64_t(1000000));
  if(input_poll_ns_<=0 || input_poll_ns_>std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::time_point::max()-Clock::now()).count())throw std::invalid_argument("input_poll_ns must be positive");
  world_configuration_ = {{"epoch_ns", world_.epoch}, {"model_step_ns", world_.dt},
      {"output_period_ns", world_.output_period}, {"input_poll_ns", input_poll_ns_},
      {"max_model_step_ns", config.value("max_model_step_ns", int64_t(10000000))},
      {"catchup_batch", config.value("catchup_batch", 8u)},
      {"sensor_workers", config.value("sensor_workers", 2u)},
      {"publish_workers", config.value("publish_workers", 2u)},
      {"publish_clock", config.value("publish_clock", false)}};
  for (const auto &j : config.value("entities", Json::array())) {
    const auto name = j.at("name").get<std::string>();
    if (options.frozen_experiment && !options.robot_bindings.contains(name))
      throw std::invalid_argument("frozen robot binding is missing");
    const auto id = options.robot_bindings.contains(name)
        ? options.robot_bindings.at(name).get<std::string>()
        : j.value("public_id", name);
    if (!safe_request_id(id)) throw std::invalid_argument("invalid public entity ID");
    auto t = std::make_shared<Command>();
    t->op = Op::Add;
    t->prepared = prepare(j, id);
    world_.submit(t);
    world_.boundary();
    if (!t->result.success)
      throw std::runtime_error("initial entity rejected");
    Json spec = j.value("specification", Json{{"id", id}, {"role", "robot"}});
    if (!entities_.emplace(id, PublicEntity{next_generation_++, std::move(spec), t->prepared->entity}).second)
      throw std::invalid_argument("duplicate initial entity ID");
  }
  latest_ = std::make_shared<const Frame>(world_.capture());
  initialized_ = true;
  hold_service_ = std::make_unique<xgc2::chassis_hold::Service>(world_.hold());
  xgc2::xrpc::UnixOptions endpoint; endpoint.path = path;
  transport_.reset(new xgc2::xrpc::HttpServer(endpoint,
      [this](xgc2::xrpc::HttpRequest request, xgc2::xrpc::HttpReply reply) {
        http_request(std::move(request), std::move(reply));
      }, limits_, xgc2::xrpc::HttpIdentity{instance_, {"/v1/describe"}}, options.retained_parent_fd));
  transport_->set_wakeup_handler([this] { harvest(); answer_ready_waiters(); });
}

Server::~Server() noexcept { shutdown(); }

void Server::run(const volatile sig_atomic_t &stopping) {
  if (io_.start_outputs) io_.start_outputs();
  world_.start();
  preparing_ = true;
  for (unsigned i = 0; i < 2; ++i)
    service_workers_.emplace_back([this] {
      try { service_work(); } catch (...) { worker_failed(); }
    });
  output_running_ = true;
  output_ = std::thread([this] {
    try { output(); } catch (...) { worker_failed(); }
  });
  {
    std::unique_lock<std::mutex> lock(started_mutex_);
    started_wake_.wait(lock, [this] { return (service_started_ == 2 && output_started_) || worker_failed_; });
  }
  if (worker_failed_) { shutdown(); std::rethrow_exception(worker_error_); }
  workers_started_ = true;
  outputs_started_ = true;
  change_health("ready");
  pthread_setname_np(pthread_self(), "xsim-input");
  auto next_input=Clock::now();
  while (!stopping && !worker_failed_ && (!io_.okay || io_.okay())) {
    if(Clock::now()>=next_input){if (io_.poll_inputs) io_.poll_inputs();next_input=Clock::now()+std::chrono::nanoseconds(input_poll_ns_);}
    std::shared_ptr<const Frame> view;
    {
      std::lock_guard<std::mutex> l(view_mutex_);
      view = latest_;
    }
    for (const auto &s : view->states)
      if (auto e = s.entity.lock(); e && e->alive && e->io && e->io->reconcile)
        e->io->reconcile();
    transport_->poll(std::chrono::milliseconds(1));
    prune_waiters();
    answer_ready_waiters();
    if (Clock::now() >= next_expiry_) { harvest(); next_expiry_ = Clock::now() + std::chrono::seconds(1); }
  }
  shutdown();
  if (worker_error_) std::rethrow_exception(worker_error_);
  if (cleanup_error_) std::rethrow_exception(cleanup_error_);
}

void Server::harvest() {
    for (auto i = requests_.begin(); i != requests_.end();) {
      auto &request = i->second;
      auto &t = request.ticket;
      if (!t) {
        if (request.expiry < Clock::now()) i = requests_.erase(i);
        else ++i;
        continue;
      }
      if (request.preparation && !request.submitted) {
        auto &prep = *request.preparation;
        if (t->phase == 0 && Clock::now() >= t->deadline)
          t->phase = 3;
        if (prep.done.load(std::memory_order_acquire)) {
          if (t->phase == 0 && !shutting_down_) {
            if (!prep.error.empty()) {
              t->result.reason = 5;
              t->phase = 4;
            } else {
              t->prepared = std::move(prep.value);
              t->resets = std::move(prep.resets);
              world_.submit(t);
            }
          }
          prep.value.reset();
          request.submitted = true;
        }
      }
      if (t->phase == 2 && t->retired) {
        request.retiring_sensor = t->retired->sensor;
        request.retirement_started = true;
        if (t->retired->io && t->retired->io->reconcile) {
          t->retired->io->reconcile();
        }
        t->retired.reset();
      }
      if (t->phase >= 2)
        t->prepared.reset();
      // Cancelled cold preparation still owns work until its worker has
      // dropped the prepared model/IO resources.
      if (request.preparation && !request.preparation->done.load(std::memory_order_acquire)) {
        ++i;
        continue;
      }
      // A removed entity may still be held by a publisher or native sensor
      // worker. Completion acknowledges actual release, not just world erase.
      if (request.retirement_started &&
          (!request.resource.expired() || !request.retiring_sensor.expired())) {
        ++i;
        continue;
      }
      if (t->phase >= 2 && !request.terminal_seen) {
        request.expiry = Clock::now() + std::chrono::minutes(5);
        request.terminal_seen = true;
        const auto result = receipt(i->first, t);
        request.immediate = result;
        for (auto& waiter : request.waiters) respond(waiter, 200, result);
        request.waiters.clear();
      }
      if (i->second.ticket->phase >= 2 && i->second.expiry < Clock::now())
        i = requests_.erase(i);
      else
        ++i;
    }

}

std::unique_ptr<Prepared> Server::prepare(const Json &j, const std::string &public_id) {
  auto e = std::make_shared<Entity>(parse_entity(j), public_id);
  return prepare(e, j);
}

std::unique_ptr<Prepared> Server::prepare(const std::shared_ptr<Entity> &e, const Json &settings) {
  auto model = prepare_model(e->config);
  e->sensor = sensors_.prepare(e, settings.value("sensor", Json::object()));
  if (io_.attach_entity)
    io_.attach_entity(e, settings.value("ros", Json::object()));
  return std::make_unique<Prepared>(Prepared{e, std::move(model)});
}

void Server::worker_failed() noexcept {
  {
    std::lock_guard<std::mutex> lock(started_mutex_);
    if (!worker_error_) worker_error_ = std::current_exception();
    worker_failed_ = true;
  }
  started_wake_.notify_all();
  if (transport_) transport_->wake();
}

void Server::shutdown() noexcept {
  if (shutting_down_) return;
  shutting_down_ = true;
  const auto attempt = [this](auto action) {
    try { action(); }
    catch (...) { if (!cleanup_error_) cleanup_error_ = std::current_exception(); }
  };
  attempt([&] { change_health("stopping"); });
  preparing_ = false;
  for (auto &entry : requests_) if (entry.second.ticket) {
    int queued = 0;
    entry.second.ticket->phase.compare_exchange_strong(queued, 3);
  }
  attempt([&] {
    std::lock_guard<std::mutex> lock(preparation_mutex_);
    preparation_tasks_.clear();
  });
  preparation_wake_.notify_all();
  attempt([&] { world_.stop(); });
  for (auto &worker : service_workers_)
    if (worker.joinable()) attempt([&] { worker.join(); });
  service_workers_.clear();
  output_running_ = false;
  if (output_.joinable()) attempt([&] { output_.join(); });
  attempt([&] { if (io_.stop_outputs) io_.stop_outputs(); });
  attempt([&] { sensors_.stop(); });
  // All native producers have quiesced before any retained reply or endpoint
  // lease is released. A failed response cannot skip the remaining cleanup.
  for (auto &entry : requests_) if (entry.second.ticket) {
    auto &ticket = entry.second.ticket;
    if (ticket->phase < 2) ticket->phase = 3;
  }
  // Queued tasks discarded above never execute their done publication. The
  // joined workers prove quiescence; retire their cold resources here.
  for (auto &entry : requests_) if (entry.second.preparation) {
    entry.second.preparation->value.reset();
    entry.second.preparation->resets.clear();
    entry.second.preparation->done.store(true, std::memory_order_release);
  }
  attempt([&] { harvest(); });
  for (auto &entry : requests_) {
    entry.second.waiters.clear();
    entry.second.preparation.reset();
  }
  health_waiters_.clear();
  ready_waiters_.clear();
  // Completes pending Engage replies with the state reached, so the drain does not wait for their timers.
  attempt([&] { hold_service_.reset(); });
  if (transport_) {
    attempt([&] { transport_->drain(); });
    attempt([&] { transport_->set_wakeup_handler({}); });
  }
}

void Server::prune_waiters() {
  auto prune = [](auto &items) {
    items.erase(std::remove_if(items.begin(), items.end(),
        [](const auto &reply) { return reply.cancelled(); }), items.end());
  };
  prune(health_waiters_);
  for (auto &entry : requests_) prune(entry.second.waiters);
}

Json Server::health() const {
  const auto component = [this](bool ready) {
    return health_state_ == "stopping" ? "stopping" : (ready ? "ready" : "starting");
  };
  return {{"state", health_state_}, {"revision", health_revision_},
      {"components", {{"world", component(initialized_ && workers_started_)},
                       {"workers", component(workers_started_)},
                       {"outputs", component(outputs_started_)}}}};
}

void Server::change_health(const std::string &state) {
  if (health_state_ == state) return;
  health_state_ = state;
  ++health_revision_;
  const auto snapshot = health();
  for (auto &reply : health_waiters_) respond(reply, 200, snapshot);
  health_waiters_.clear();
  answer_ready_waiters();
}

void Server::output() {
  pthread_setname_np(pthread_self(), "xsim-output");
  {
    std::lock_guard<std::mutex> lock(started_mutex_);
    output_started_ = true;
  }
  started_wake_.notify_all();
  std::shared_ptr<const Frame> frame;
  size_t sensor_cursor=0;
  while (output_running_) {
    if (world_.take_frame(frame)) {
      if (!frames_flowing_.exchange(true)) transport_->wake(); // readiness may have changed
      sensors_.submit_frame(frame);
      {
        std::lock_guard<std::mutex> l(view_mutex_);
        latest_ = frame;
      }
      if (io_.publish_frame) io_.publish_frame(*frame);
      const auto rates = std::atomic_load(&telemetry_rates_);
      // Flush the entire telemetry snapshot before any large cloud. After
      // each cloud, check for newer telemetry again instead of serializing
      // a fleet's clouds ahead of everyone else's heartbeat.
      if (io_.publish_entities) io_.publish_entities(frame, rates);
      else for(const auto& s:frame->states)if(auto e=s.entity.lock();e && e->alive && e->io && e->io->publish)e->io->publish(s, *rates);
    }
    if (io_.publish_entities) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    bool published=false;
    const auto count = frame ? frame->states.size() : 0;
    for(size_t n=0;n<count;++n) {
      const size_t i=(sensor_cursor+n)%count;
      if(auto e=frame->states[i].entity.lock();e && e->alive && e->io && e->io->publish_sensor && e->io->publish_sensor()) {
        sensor_cursor=(i+1)%count;published=true;break;
      }
    }
    if(!published)std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

Json Server::status() {
  auto &m = world_.metrics;
  const auto completed = m.steps.load();
  auto seconds = std::chrono::duration<double>(Clock::now() - began_).count();
  return {
      {"instance_id", instance_},
      {"steps", completed},
      {"simulation_time_ns", m.sim_ns.load()},
      {"epoch_ns", world_.epoch},
      {"model_step_ns", world_.dt},
      {"scheduling_period_ns", m.scheduling_period_ns.load()},
      {"last_dt_ns", m.last_dt_ns.load()},
      {"clock_degradations", m.clock_degradations.load()},
      {"realtime_rtf", m.active_wall_ns > 0 ? double(m.realtime_ns.load()) / double(m.active_wall_ns.load()) : 0.0},
      {"paused", m.paused.load()},
      {"rtf", double(m.sim_ns.load() - world_.epoch) * 1e-9 / seconds},
      {"lag_ns", m.lag_ns.load()},
      {"step_latency_ns", m.step_latency_ns.load()},
      {"max_step_latency_ns", m.max_step_latency_ns.load()},
      {"output_misses", m.output_misses.load()},
      {"frame_slots", World::frame_pool_size},
      {"frame_array_grows", m.frame_array_grows.load()},
      {"input_coalesced", m.input_misses.load()},
      {"hold_refused_commands", m.hold_refused.load()},
      {"sensors", sensors_.status()},
      {"telemetry", telemetry_rates()},
      {"publication", io_.publication_status ? io_.publication_status() : Json::object()}};
}

Json Server::telemetry_rates() {
  const auto rates = std::atomic_load(&telemetry_rates_);
  return {{"instance_id", instance_},
          {"requested_rates_hz", rates->json()},
          {"snapshot_period_ns", world_.output_period},
          {"snapshot_cap_hz", 1e9 / double(world_.output_period)},
          {"effective_cap_hz", rates->caps(world_.output_period)}};
}


std::shared_ptr<const Frame> Server::view() {
  std::lock_guard<std::mutex> l(view_mutex_);
  return latest_;
}

Json Server::receipt(const std::string &id, const Ticket &t) {
  const auto &record = requests_.at(id);
  if (record.terminal_seen && !record.immediate.is_null()) return record.immediate;
  Json result{{"id", id}, {"kind", record.kind}, {"state", "accepted"}};
  if (!record.public_id.empty()) result["target"] = record.public_id;
  if (!t) throw std::logic_error("operation has no native command");
  const int phase = t->phase.load(std::memory_order_acquire);
  const auto snapshot_result = [&] {
    Json entities = Json::array();
    for (const auto &identity : record.result_entities) {
      const State *snapshot = nullptr;
      const auto native_id = identity.at("native_id").get<uint64_t>();
      if (t->result.has_state &&
          (t->result.state.key.id == native_id || record.kind == "entities.create"))
        snapshot = &t->result.state;
      else for (const auto &state : t->states)
        if (state.key.id == native_id) { snapshot = &state; break; }
      if (!snapshot) continue;
      PublicEntity public_entity{identity.at("generation").get<uint64_t>(),
                                  identity.at("specification"), snapshot->entity};
      auto entity = entity_json(identity.at("id").get<std::string>(), public_entity, snapshot);
      entity["lifecycle"] = record.kind == "entities.remove" ? "removed" : "ready";
      entities.push_back(std::move(entity));
    }
    Json snapshot{{"time", {{"epoch", 1}, {"nanoseconds", std::to_string(t->result.stamp)}}},
                  {"paused", t->result.paused}, {"step_size_seconds", double(world_.dt) * 1e-9},
                  {"entities", std::move(entities)}};
    if (record.kind == "world.step") snapshot["steps_completed"] = t->result.completed_steps;
    return snapshot;
  };
  if (phase == 1 ||
      (phase == 3 && record.preparation && !record.submitted) ||
      (phase == 2 && t->result.success && record.kind == "entities.remove" &&
      (t->retired || !record.resource.expired() || !record.retiring_sensor.expired())))
    result["state"] = "running";
  else if (phase == 3) {
    result["state"] = "cancelled";
    result["effects"] = {{"applied", false}};
  } else if (phase == 4 || (phase == 2 && !t->result.success)) {
    result["state"] = "failed";
    result["error"] = {{"code", t->result.interrupted ? "interrupted" :
        (t->result.reason == 1 ? "conflict" : "invalid_argument")},
        {"message", t->result.interrupted ? "native step interrupted by shutdown" :
                                           "native engine rejected operation"}};
    result["effects"] = {{"applied", t->result.interrupted && t->result.applied}};
    if (t->result.interrupted) result["result"] = snapshot_result();
  } else if (phase == 2) {
    result["state"] = "succeeded";
    result["effects"] = {{"applied", true}};
    result["result"] = snapshot_result();
  }
  return result;
}

void Server::respond(xgc2::xrpc::HttpReply reply, int code, Json result) {
  result["instance_id"] = instance_;
  xgc2::xrpc::HttpResponse response; response.status = code;
  response.headers.emplace_back("Content-Type", "application/json");
  xgc2::xrpc::BoundedOutput output(limits_.response_bytes); output.stream() << result;
  if (!output.good()) reply.complete(xgc2::xrpc::http_error(500, "resource_exhausted", "JSON response exceeds limit"));
  else { response.body = output.value(); reply.complete(std::move(response)); }
}
void Server::http_request(xgc2::xrpc::HttpRequest request, xgc2::xrpc::HttpReply reply) {
  try {
    std::vector<std::unordered_set<std::string>> keys;
    auto unique_keys = [&keys](int depth, Json::parse_event_t event, Json &parsed) {
      if (depth > 64) throw std::invalid_argument("JSON nesting exceeds limit");
      if (event == Json::parse_event_t::object_start) keys.emplace_back();
      else if (event == Json::parse_event_t::object_end) keys.pop_back();
      else if (event == Json::parse_event_t::key && !keys.back().insert(parsed.get<std::string>()).second)
        throw std::invalid_argument("duplicate JSON field");
      return true;
    };
    const Json body = request.body.empty() ? Json::object() : Json::parse(request.body, unique_keys);
    if (!body.is_object()) throw std::invalid_argument("request must be an object");
    if (!simulation_request(request, body, reply)) reply.complete(xgc2::xrpc::http_error(404, "not_found", "unknown simulation endpoint"));
  } catch (const std::exception &error) { reply.complete(xgc2::xrpc::http_error(400, "invalid_argument", error.what())); }
}

void Server::service_work() {
  {
    std::lock_guard<std::mutex> lock(started_mutex_);
    ++service_started_;
  }
  started_wake_.notify_all();
  while (preparing_) {
    std::function<void()> task;
    {
      std::unique_lock<std::mutex> lock(preparation_mutex_);
      if (preparation_tasks_.empty() && !io_.poll_services)
        preparation_wake_.wait(lock, [this] { return !preparing_ || !preparation_tasks_.empty(); });
      if (!preparing_) return;
      if (!preparation_tasks_.empty()) {
        task = std::move(preparation_tasks_.front());
        preparation_tasks_.pop_front();
      }
    }
    if (task) task();
    // These are the original two service workers, not an additional pool.
    // Native preparation is ordinary C++; only actual ROS services enter its queue.
    if (io_.poll_services) io_.poll_services();
  }
}
} // namespace xsim
