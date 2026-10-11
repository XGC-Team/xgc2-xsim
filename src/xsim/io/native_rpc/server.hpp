#pragma once
#include "io/runtime_io.hpp"
#include "systems/sensors.hpp"
#include <csignal>
#include <deque>
#include <mutex>
#include <xgc2/chassis_hold/service.hpp>
#include <xgc2/xrpc/http.hpp>
#include <thread>
#include <unordered_map>
namespace xsim {
// The capability the world serves through generic method calls, POST /v1/call/<capability>/<Method>.
inline constexpr const char *chassis_hold_capability = "xgc2.chassis.hold";
struct RpcOptions {
  std::string target_id;
  xgc2::xrpc::HttpLimits limits;
  int retained_parent_fd = -1;
  Json robot_bindings = Json::object();
  bool frozen_experiment = false;
};
class Server {
public:
  Server(const Json &config, const std::string &socket, RpcOptions,
         World &, Sensors &, RuntimeIO io = {});
  ~Server() noexcept;
  void run(const volatile sig_atomic_t &stopping);
private:
  struct Preparation {
    std::atomic<bool> done{false};
    std::unique_ptr<Prepared> value;
    std::vector<std::pair<Key, Model>> resets;
    std::string error;
  };
  struct Request {
    Ticket ticket;
    std::weak_ptr<Entity> resource;
    Clock::time_point expiry;
    std::string payload;
    std::shared_ptr<Preparation> preparation;
    bool submitted = false;
    Json immediate = nullptr;
    bool terminal_seen = false;
    std::string fingerprint, kind, public_id;
    std::vector<xgc2::xrpc::HttpReply> waiters;
    // Frozen public identities for this operation; no IO/resource ownership.
    Json result_entities = Json::array();
    std::size_t retained_bytes = 0;
    std::weak_ptr<Sensor> retiring_sensor;
    bool retirement_started = false;
  };
  struct PublicEntity {
    uint64_t generation;
    Json specification;
    std::weak_ptr<Entity> resource;

  };
  std::string instance_; // the incarnation of the world, shared by the HTTP binding and its HOLD domain
  std::string socket_path_;
  std::string target_id_;
  xgc2::xrpc::HttpLimits limits_;
  std::unordered_map<std::string, PublicEntity> entities_;
  uint64_t next_generation_ = 1;
  Clock::time_point next_expiry_ = Clock::now();
  void harvest();
  Json entity_json(const std::string &, const PublicEntity &);
  Json entity_json(const std::string &, const PublicEntity &, const State *);
  Json describe();
  Json health() const;
  void change_health(const std::string &);
  void prune_waiters();
  std::string health_state_ = "starting";
  uint64_t health_revision_ = 1;
  bool initialized_ = false, workers_started_ = false, outputs_started_ = false;
  bool shutting_down_ = false;
  std::vector<xgc2::xrpc::HttpReply> health_waiters_;
  bool simulation_request(const xgc2::xrpc::HttpRequest &, const Json &, xgc2::xrpc::HttpReply);
  // The capabilities of the world, served through POST /v1/call/<service>/<Method> (chassis.cpp).
  bool capability_call(const xgc2::xrpc::HttpRequest &, const std::string &service, const std::string &method,
                       xgc2::xrpc::HttpReply);
  void respond(xgc2::xrpc::HttpReply, int, Json);

  RuntimeIO io_;
  Json world_configuration_;
  World &world_;
  Sensors &sensors_;
  std::unique_ptr<xgc2::xrpc::HttpServer> transport_;
  std::unique_ptr<xgc2::chassis_hold::Service> hold_service_; // before the transport it replies on goes
  std::unordered_map<std::string, Request> requests_;
  std::mutex view_mutex_;
  std::shared_ptr<const Frame> latest_;
  std::shared_ptr<const TelemetryRates> telemetry_rates_;
  std::thread output_;
  std::atomic<bool> output_running_{false};
  Clock::time_point began_ = Clock::now();
  int64_t input_poll_ns_ = 1000000;
  std::mutex preparation_mutex_;
  std::condition_variable preparation_wake_;
  std::deque<std::function<void()>> preparation_tasks_;
  std::vector<std::thread> service_workers_;
  std::atomic<bool> preparing_{false};
  std::mutex started_mutex_;
  std::condition_variable started_wake_;
  unsigned service_started_ = 0;
  bool output_started_ = false;
  std::atomic<bool> worker_failed_{false};
  std::exception_ptr worker_error_, cleanup_error_;
  void worker_failed() noexcept;
  std::unique_ptr<Prepared> prepare(const Json &, const std::string &public_id);
  std::unique_ptr<Prepared> prepare(const std::shared_ptr<Entity> &, const Json &);
  void service_work();
  void shutdown() noexcept;
  void output();
  Json status();
  Json telemetry_rates();
  std::shared_ptr<const Frame> view();
  Json receipt(const std::string &, const Ticket &);
  void http_request(xgc2::xrpc::HttpRequest, xgc2::xrpc::HttpReply);
};
} // namespace xsim
