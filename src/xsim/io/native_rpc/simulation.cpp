#include "server.hpp"
#include <algorithm>
#include <limits>
#include <unordered_set>

namespace xsim {
namespace {
constexpr std::size_t entity_limit = 256, operation_limit = 256;
constexpr std::size_t inflight_limit = 32, waiter_limit = 32;
constexpr std::size_t payload_limit = 16 * 1024 * 1024;
constexpr uint64_t operation_timeout_limit = 5000;
void fields(const Json &body, std::initializer_list<const char *> allowed) {
  if (!body.is_object()) throw std::invalid_argument("expected an object");
  for (auto i = body.begin(); i != body.end(); ++i)
    if (std::none_of(allowed.begin(), allowed.end(), [&](const char *name) { return i.key() == name; }))
      throw std::invalid_argument("unknown field: " + i.key());
}
bool valid_id(const std::string &id) {
  return !id.empty() && id.size() <= 128 &&
    id.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.:-") == std::string::npos;
}
uint64_t integer(const Json &value, const char *name, bool positive = true) {
  if (!value.is_number_integer() || (!value.is_number_unsigned() && value.get<int64_t>() < 0))
    throw std::invalid_argument(std::string(name) + " must be an integer");
  const auto result = value.get<uint64_t>();
  if (positive && !result) throw std::invalid_argument(std::string(name) + " must be positive");
  return result;
}
bool terminal(const Json &operation) {
  const auto state = operation.at("state").get<std::string>();
  return state == "succeeded" || state == "failed" || state == "cancelled";
}
// The only describe query: wait_ready_ms=<0..30000>, how long to hold the call while the service is not ready.
std::chrono::milliseconds describe_wait(const std::string &query) {
  if (query.empty()) return std::chrono::milliseconds::zero();
  const std::string key = "wait_ready_ms=";
  const auto digits = query.compare(0, key.size(), key) == 0 ? query.substr(key.size()) : std::string();
  if (digits.empty() || digits.size() > 5 || (digits.size() > 1 && digits[0] == '0') ||
      digits.find_first_not_of("0123456789") != std::string::npos)
    throw std::invalid_argument("describe takes only wait_ready_ms=<integer>");
  const auto milliseconds = std::stoul(digits);
  if (milliseconds > 30000) throw std::invalid_argument("wait_ready_ms exceeds 30000");
  return std::chrono::milliseconds(milliseconds);
}
std::vector<std::string> segments(const std::string &path) {
  std::vector<std::string> parts;
  std::size_t begin = 1;
  while (begin < path.size()) {
    const auto end = path.find('/', begin);
    parts.push_back(path.substr(begin, end - begin));
    if (end == std::string::npos) break;
    begin = end + 1;
  }
  return parts;
}
}

Json Server::entity_json(const std::string &id, const PublicEntity &record) {
  const auto entity = record.resource.lock();
  const auto snapshot = view();
  if (entity && entity->alive)
    for (const auto &state : snapshot->states)
      if (state.key.id == entity->id && state.key.generation == entity->generation.load())
        return entity_json(id, record, &state);
  return entity_json(id, record, nullptr);
}
Json Server::entity_json(const std::string &id, const PublicEntity &record, const State *state) {
  Json result{{"ref", {{"id", id}, {"generation", record.generation}}},
    {"specification", record.specification}, {"role", record.specification.value("role", "robot")}};
  const auto entity = record.resource.lock();
  result["lifecycle"] = entity && entity->alive ? "ready" : "removed";
  if (state) {
    const Eigen::Vector3d angular = state->orientation * state->omega;
    result["state"] = {{"pose", {{"position", {state->position.x(), state->position.y(), state->position.z()}},
      {"orientation", {state->orientation.x(), state->orientation.y(), state->orientation.z(), state->orientation.w()}}}},
      {"twist", {{"linear", {state->velocity.x(), state->velocity.y(), state->velocity.z()}},
                 {"angular", {angular.x(), angular.y(), angular.z()}}}}, {"enabled", state->enabled}};
  }
  return result;
}
bool Server::ready() const { return health_state_ == "ready" && frames_flowing_; }
Json Server::facts() {
  // xsim loads its one world when the process starts: world_generation is fixed, and a different world is a
  // different process with another instance_id.
  Json result{{"world_generation", 1}, {"frames_flowing", frames_flowing_.load()}, {"health", health_state_}};
  result["capabilities"] = capabilities();
  if (io_.facts) {
    const auto io = io_.facts();
    for (auto i = io.begin(); i != io.end(); ++i) result[i.key()] = i.value();
  }
  return result;
}
void Server::answer_ready_waiters() {
  if (ready_waiters_.empty()) return;
  const bool settled = ready() || health_state_ == "stopping";
  const auto now = Clock::now();
  Json current;
  ready_waiters_.erase(std::remove_if(ready_waiters_.begin(), ready_waiters_.end(),
      [&](const ReadyWaiter &waiter) {
        if (waiter.reply.cancelled()) return true;
        if (!settled && now < waiter.until) return false;
        if (current.is_null()) current = describe();
        respond(waiter.reply, 200, current);
        return true;
      }), ready_waiters_.end());
}
Json Server::describe() {
  return {{"service", "xgc2.simulation"}, {"api_version", "v1"}, {"ready", ready()}, {"facts", facts()},
    {"describe_path", "/v1/describe"},
    {"service_ref", {{"target_id", target_id_}, {"service", "xgc2.simulation"}, {"api_version", "v1"},
      {"instance_id", instance_}, {"profile", "http.v1"}, {"endpoint", {{"kind", "unix"}, {"address", socket_path_}}}}},
    {"engine", {{"name", "xsim"}}},
    {"capabilities", {{"health.observe", true}, {"entities.create", true}, {"entities.remove", true}, {"entities.reset", true},
      {"entities.set_state", {{"enabled", true}, {"pose", false}, {"twist", false}}},
      {"world.pause", true}, {"world.resume", true}, {"world.step", true},
      {"world.reset", {{"all_entities", true}, {"entities", true}, {"reset_time", false}, {"atomic", true}}},
      {"sensors.describe", false}, {"sensors.create", false}, {"sensors.configure", false}, {"sensors.remove", false}}},
    {"limits", {{"request_bytes", limits_.request_bytes}, {"response_bytes", limits_.response_bytes},
      {"connections", limits_.connections}, {"transport_inflight", limits_.inflight},
      {"operations", operation_limit}, {"operation_payload_bytes", payload_limit}, {"inflight_operations", inflight_limit},
      {"operation_timeout_ms", operation_timeout_limit}, {"retained_terminal_ms", 300000},
      {"entities", entity_limit}, {"waiters_per_operation", waiter_limit}, {"health_waiters", waiter_limit}}},
    {"world_frame", {{"id", "world"}, {"axes", "right-handed ENU"}, {"gravity", {0, 0, -9.81}}}},
    {"artifact_types", {"application/vnd.xgc2.xsim.world+json", "application/vnd.xgc2.xsim.entity+json"}},
    {"asset_schemas", {{"application/vnd.xgc2.xsim.entity+json", {{"kinds", {"fs150", "scout", "mecanum"}},
      {"initial_orientation", "yaw-only"}}}}}};
}

bool Server::simulation_request(const xgc2::xrpc::HttpRequest &request, const Json &body,
                                xgc2::xrpc::HttpReply reply) {
  const auto &method = request.method;
  const auto query_at = request.target.find('?');
  const std::string path = request.target.substr(0, query_at);
  const std::string query = query_at == std::string::npos ? std::string() : request.target.substr(query_at + 1);
  if (path.empty() || path.back() == '/' || (!query.empty() && path != "/v1/describe")) return false;
  const auto parts = segments(path);
  if (parts.size() < 2 || parts[0] != "v1") return false;
  auto error = [&](int status, const std::string &code, const std::string &message) {
    reply.complete(xgc2::xrpc::http_error(status, code, message)); return true;
  };
  if (method == "POST" && parts.size() == 4 && parts[1] == "call")
    return capability_call(request, parts[2], parts[3], reply);
  harvest();
  prune_waiters();
  if (method == "GET" && path == "/v1/describe") {
    fields(body, {});
    const auto wait = describe_wait(query);
    if (wait.count() == 0 || ready()) respond(reply, 200, describe());
    else {
      if (ready_waiters_.size() >= waiter_limit) return error(503, "resource_exhausted", "describe waiter limit reached");
      ready_waiters_.push_back({reply, std::min(Clock::now() + wait, request.deadline)});
    }
    return true;
  }
  if (method == "GET" && path == "/v1/health") {
    fields(body, {}); respond(reply, 200, health()); return true;
  }
  if (method == "POST" && path == "/v1/health/observe") {
    fields(body, {"after_revision"});
    const auto revision = integer(body.at("after_revision"), "after_revision", false);
    if (revision != health_revision_) respond(reply, 200, health());
    else {
      if (health_waiters_.size() >= waiter_limit) return error(503, "resource_exhausted", "health waiter limit reached");
      health_waiters_.push_back(reply);
    }
    return true;
  }
  if (method == "GET" && path == "/v1/world") {
    fields(body, {});
    Json members = Json::array();
    for (const auto &entry : entities_)
      if (auto e = entry.second.resource.lock(); e && e->alive) members.push_back(entity_json(entry.first, entry.second));
    respond(reply, 200, {{"mode", world_.metrics.paused ? "paused" : "running"},
      {"paused", world_.metrics.paused.load()},
      {"time", {{"epoch", 1}, {"nanoseconds", std::to_string(world_.metrics.sim_ns.load())}}},
      {"step_size_nanoseconds", std::to_string(world_.dt)}, {"step_size_seconds", double(world_.dt) * 1e-9},
      {"steps", world_.metrics.steps.load()}, {"entities", std::move(members)}, {"diagnostics", status()}});
    return true;
  }
  if (parts[1] == "operations" && parts.size() >= 3) {
    if (!valid_id(parts[2])) throw std::invalid_argument("invalid operation ID");
    auto found = requests_.find(parts[2]);
    if (found == requests_.end()) return error(404, "not_found", "unknown or expired operation");
    auto &operation = found->second;
    const auto result = receipt(found->first, operation.ticket);
    if (method == "GET" && parts.size() == 3) {
      fields(body, {}); respond(reply, 200, result); return true;
    }
    if (method == "POST" && parts.size() == 4 && parts[3] == "wait") {
      fields(body, {});
      if (terminal(result)) respond(reply, 200, result);
      else {
        if (operation.waiters.size() >= waiter_limit) return error(503, "resource_exhausted", "operation waiter limit reached");
        operation.waiters.push_back(reply);
      }
      return true;
    }
    if (method == "POST" && parts.size() == 4 && parts[3] == "cancel") {
      fields(body, {});
      if (operation.ticket) {
        int queued = 0;
        if (operation.ticket->phase.compare_exchange_strong(queued, 3)) operation.ticket->signal();
      }
      respond(reply, 200, receipt(found->first, operation.ticket)); return true;
    }
    return false;
  }
  if (parts[1] == "sensors") {
    if (parts.size() > 3) return false;
    return error(422, "unsupported", "managed sensor description and mutation are not implemented");
  }
  if (method == "GET" && path == "/v1/entities") {
    fields(body, {});
    Json list = Json::array();
    for (const auto &entry : entities_)
      if (auto e = entry.second.resource.lock(); e && e->alive) list.push_back(entity_json(entry.first, entry.second));
    respond(reply, 200, {{"entities", std::move(list)}}); return true;
  }
  const auto fingerprint = method + " " + path + " " + body.dump();
  if (method == "POST" || method == "DELETE") {
    if (!valid_id(request.request_id)) throw std::invalid_argument("invalid request ID");
    const auto prior = requests_.find(request.request_id);
    if (prior != requests_.end()) {
      if (prior->second.fingerprint != fingerprint) return error(409, "conflict", "request ID reused with different input");
      const auto result = receipt(prior->first, prior->second.ticket);
      respond(reply, terminal(result) ? 200 : 202, result); return true;
    }
  }
  PublicEntity *record = nullptr;
  std::shared_ptr<Entity> selected_entity;
  if (parts[1] == "entities" && parts.size() >= 3) {
    if (!valid_id(parts[2])) throw std::invalid_argument("invalid entity ID");
    const auto found = entities_.find(parts[2]);
    if (found == entities_.end()) return error(404, "not_found", "unknown entity");
    record = &found->second;
    selected_entity = record->resource.lock();
    if (!selected_entity || !selected_entity->alive) return error(404, "not_found", "entity is not realized");
    if (method == "GET" && parts.size() == 3) {
      fields(body, {}); respond(reply, 200, entity_json(parts[2], *record)); return true;
    }
  }
  const bool create = method == "POST" && path == "/v1/entities";
  const bool entity_mutation = record && ((method == "DELETE" && parts.size() == 3) ||
    (method == "POST" && parts.size() == 4 && (parts[3] == "state" || parts[3] == "reset")));
  const bool world_mutation = method == "POST" && parts.size() == 3 && parts[1] == "world" &&
    (parts[2] == "pause" || parts[2] == "resume" || parts[2] == "step" || parts[2] == "reset");
  if (!create && !entity_mutation && !world_mutation) return false;
  if (!valid_id(request.request_id)) throw std::invalid_argument("invalid request ID");
  if (health_state_ != "ready" || shutting_down_ || !preparing_)
    return error(503, "unavailable", "world is not accepting mutations");
  const auto timeout = integer(body.at("operation_timeout_ms"), "operation_timeout_ms");
  if (timeout > operation_timeout_limit) throw std::invalid_argument("operation_timeout_ms exceeds 5000");
  auto ticket = std::make_shared<Command>();
  Request operation;
  operation.ticket = ticket;
  operation.expiry = Clock::time_point::max();
  operation.fingerprint = fingerprint;
  Json native;
  std::shared_ptr<Entity> addition;
  std::vector<std::pair<Key, std::shared_ptr<Entity>>> reset_entities;
  auto identity = [](const std::string &id, const PublicEntity &value, uint64_t native_id) {
    return Json{{"id", id}, {"generation", value.generation}, {"specification", value.specification}, {"native_id", native_id}};
  };
  uint64_t public_generation = 0;
  Json specification;
  if (create) {
    fields(body, {"entity", "operation_timeout_ms"});
    specification = body.at("entity");
    fields(specification, {"id", "role", "asset", "pose", "parameters", "sensors"});
    fields(specification.at("asset"), {"id", "revision", "realization"});
    fields(specification.at("asset").at("realization"), {"media_type", "uri", "content"});
    fields(specification.at("pose"), {"position", "orientation"});
    native = entity_artifact(specification);
    operation.public_id = specification.contains("id") ? specification.at("id").get<std::string>() : xgc2::xrpc::new_instance_id();
    if (!valid_id(operation.public_id)) throw std::invalid_argument("invalid entity ID");
    for (auto i = entities_.begin(); i != entities_.end();) {
      if (i->second.resource.expired()) i = entities_.erase(i); else ++i;
    }
    if (entities_.count(operation.public_id)) return error(409, "conflict", "entity ID already exists");
    if (entities_.size() >= entity_limit) return error(503, "resource_exhausted", "entity limit reached");
    addition = std::make_shared<Entity>(parse_entity(native), operation.public_id);
    for (const auto &entry : entities_)
      if (auto e = entry.second.resource.lock(); e && e->config.name == addition->config.name)
        return error(409, "conflict", "entity model name already exists");
    if (next_generation_ == std::numeric_limits<uint64_t>::max()) return error(503, "resource_exhausted", "entity generation exhausted");
    public_generation = next_generation_;
    specification["id"] = operation.public_id;
    ticket->op = Op::Add;
    ticket->capture_state = true;
    operation.kind = "entities.create";
    operation.resource = addition;
    operation.preparation = std::make_shared<Preparation>();
    operation.result_entities.push_back({{"id", operation.public_id}, {"generation", public_generation},
      {"specification", specification}, {"native_id", 0}});
  } else if (entity_mutation) {
    const auto generation = integer(body.at("generation"), "generation");
    if (generation != record->generation) return error(409, "conflict", "entity generation mismatch");
    operation.public_id = parts[2];
    operation.resource = selected_entity;
    operation.result_entities.push_back(identity(parts[2], *record, selected_entity->id));
    ticket->key = {selected_entity->id, selected_entity->generation.load()};
    ticket->capture_state = true;
    if (method == "DELETE") {
      fields(body, {"generation", "operation_timeout_ms"});
      ticket->op = Op::Remove; operation.kind = "entities.remove";
    } else if (parts[3] == "state") {
      fields(body, {"generation", "operation_timeout_ms", "state"});
      const auto &state = body.at("state");
      fields(state, {"enabled", "pose", "twist"});
      if (state.contains("pose") || state.contains("twist"))
        return error(422, "unsupported", "only enabled state is supported");
      if (!state.at("enabled").is_boolean()) throw std::invalid_argument("enabled must be boolean");
      ticket->op = Op::SetEnabled; ticket->enabled = state.at("enabled").get<bool>();
      operation.kind = "entities.set_state";
    } else {
      fields(body, {"generation", "operation_timeout_ms", "state"});
      if (body.contains("state")) return error(422, "unsupported", "explicit reset state is not supported");
      ticket->op = Op::Reset; operation.kind = "entities.reset";
      operation.preparation = std::make_shared<Preparation>();
      reset_entities.emplace_back(ticket->key, selected_entity);
    }
  } else {
    ticket->capture_states = true;
    ticket->states.reserve(entity_limit);
    for (const auto &entry : entities_)
      if (auto e = entry.second.resource.lock(); e && e->alive)
        operation.result_entities.push_back(identity(entry.first, entry.second, e->id));
    operation.kind = "world." + parts[2];
    if (parts[2] == "pause" || parts[2] == "resume") {
      fields(body, {"operation_timeout_ms"});
      ticket->op = parts[2] == "pause" ? Op::Pause : Op::Resume;
    } else if (parts[2] == "step") {
      fields(body, {"operation_timeout_ms", "steps"});
      ticket->op = Op::Step; ticket->steps = integer(body.at("steps"), "steps");
      const auto remaining = (std::numeric_limits<int64_t>::max() - world_.metrics.sim_ns.load()) / world_.dt;
      if (ticket->steps > uint64_t(remaining)) throw std::invalid_argument("steps overflow simulation time");
    } else {
      fields(body, {"operation_timeout_ms", "scope", "entities", "reset_time", "state"});
      if (!body.at("reset_time").is_boolean()) throw std::invalid_argument("reset_time must be boolean");
      if (body.at("reset_time").get<bool>() || body.contains("state"))
        return error(422, "unsupported", "time rewind and explicit reset state are not supported");
      ticket->op = Op::Reset;
      const auto scope = body.at("scope").get<std::string>();
      if (scope == "entities") {
        ticket->action = 1;
        const auto &refs = body.at("entities");
        if (!refs.is_array() || refs.empty() || refs.size() > entity_limit)
          throw std::invalid_argument("entities reset requires 1..256 EntityRefs");
        std::unordered_set<std::string> ids;
        for (const auto &ref : refs) {
          fields(ref, {"id", "generation"});
          const auto id = ref.at("id").get<std::string>();
          if (!valid_id(id) || !ids.insert(id).second) throw std::invalid_argument("invalid or duplicate reset entity ID");
          const auto generation = integer(ref.at("generation"), "generation");
          const auto found = entities_.find(id);
          if (found == entities_.end()) return error(404, "not_found", "reset entity missing");
          if (found->second.generation != generation) return error(409, "conflict", "reset entity generation mismatch");
          auto entity = found->second.resource.lock();
          if (!entity || !entity->alive) return error(404, "not_found", "reset entity is not realized");
          reset_entities.emplace_back(Key{entity->id, entity->generation.load()}, std::move(entity));
        }
      } else if (scope == "all_entities") {
        if (body.contains("entities")) throw std::invalid_argument("all_entities omits entities");
        ticket->action = 0;
        for (const auto &entry : entities_)
          if (auto e = entry.second.resource.lock(); e && e->alive)
            reset_entities.emplace_back(Key{e->id, e->generation.load()}, std::move(e));
      } else throw std::invalid_argument("invalid reset scope");
      operation.preparation = std::make_shared<Preparation>();
    }
  }
  operation.retained_bytes = fingerprint.size() + operation.result_entities.dump().size() + native.dump().size();
  std::size_t pending = 0, payload_bytes = operation.retained_bytes;
  for (const auto &entry : requests_) {
    payload_bytes += entry.second.retained_bytes;
    if ((entry.second.ticket && entry.second.ticket->phase < 2) ||
        (entry.second.preparation && !entry.second.preparation->done.load(std::memory_order_acquire))) ++pending;
  }
  if (requests_.size() >= operation_limit || pending >= inflight_limit || payload_bytes > payload_limit)
    return error(503, "resource_exhausted", "operation admission limit reached");
  const auto id = request.request_id;
  auto inserted = requests_.emplace(id, std::move(operation));
  auto &admitted = inserted.first->second;
  ticket->deadline = Clock::now() + std::chrono::milliseconds(timeout);
  ticket->notification_context = transport_.get();
  ticket->notification = [](void *context) noexcept { static_cast<xgc2::xrpc::HttpServer *>(context)->wake(); };
  try {
    if (addition) {
      entities_.emplace(admitted.public_id, PublicEntity{public_generation, std::move(specification), addition});
      ++next_generation_;
    }
    if (admitted.preparation) {
      const auto preparation = admitted.preparation;
      const bool adding = bool(addition);
      {
        std::lock_guard<std::mutex> lock(preparation_mutex_);
        preparation_tasks_.push_back([this, ticket, preparation, addition, native = std::move(native),
                                      reset_entities = std::move(reset_entities), adding]() mutable {
          try {
            if (ticket->phase == 0 && Clock::now() < ticket->deadline) {
              if (adding) preparation->value = prepare(addition, native);
              else if (ticket->key.id) {
                auto entity = reset_entities.at(0).second;
                preparation->value = std::make_unique<Prepared>(Prepared{entity, prepare_model(entity->config)});
              } else {
                preparation->resets.reserve(reset_entities.size());
                for (const auto &entry : reset_entities)
                  preparation->resets.emplace_back(entry.first, prepare_model(entry.second->config));
              }
            } else {
              int queued = 0; ticket->phase.compare_exchange_strong(queued, 3);
            }
          } catch (const std::exception &failure) { preparation->error = failure.what(); }
          addition.reset();
          reset_entities.clear();
          native = Json();
          preparation->done.store(true, std::memory_order_release);
          ticket->signal();
        });
      }
      preparation_wake_.notify_one();
    } else {
      admitted.submitted = true;
      world_.submit(ticket);
    }
  } catch (...) {
    if (addition) entities_.erase(admitted.public_id);
    requests_.erase(inserted.first);
    throw;
  }
  respond(reply, 202, receipt(id, ticket));
  return true;
}
} // namespace xsim
