#include "io/native_rpc/server.hpp"
#include <xgc2/xrpc/bootstrap.hpp>
#include <atomic>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {
using namespace xsim;
namespace fs = std::filesystem;

fs::path input_thread() {
  for (const auto &entry : fs::directory_iterator("/proc/self/task")) {
    std::ifstream comm(entry.path() / "comm");
    std::string name;
    std::getline(comm, name);
    if (name == "xsim-input") return entry.path();
  }
  assert(false && "actual input thread is missing");
  return {};
}
unsigned long context_switches(const fs::path &task) {
  std::ifstream status(task / "status");
  std::string key;
  while (status >> key) {
    if (key == "voluntary_ctxt_switches:") {
      unsigned long value = 0;
      status >> value;
      return value;
    }
    std::string rest;
    std::getline(status, rest);
  }
  assert(false && "actual input thread status is missing");
  return 0;
}
struct Inputs {
  std::mutex mutex;
  std::condition_variable changed;
  std::function<void()> wake;
  unsigned pending = 0;
  std::vector<Clock::time_point> received;
  void bind(std::function<void()> value) {
    std::lock_guard<std::mutex> lock(mutex);
    wake = std::move(value);
    if (wake && pending) wake();
  }
  void post() {
    std::lock_guard<std::mutex> lock(mutex);
    ++pending;
    if (wake) wake();
  }
  bool is_pending() {
    std::lock_guard<std::mutex> lock(mutex);
    return pending != 0;
  }
  void drain() {
    std::lock_guard<std::mutex> lock(mutex);
    if (pending) {
      pending = 0;
      received.push_back(Clock::now());
      changed.notify_all();
    }
  }
  bool wait_for(unsigned count) {
    std::unique_lock<std::mutex> lock(mutex);
    return changed.wait_for(lock, std::chrono::seconds(3), [&] { return received.size() >= count; });
  }
};
}

int main() {
  using namespace std::chrono_literals;
  char temporary[] = "/tmp/xsim-input-wake-XXXXXX";
  assert(mkdtemp(temporary));
  const fs::path directory(temporary);
  const int parent = open(directory.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
  assert(parent >= 0);
  auto grant = xgc2::xrpc::DirectoryGrant::from_owned_directory(parent, xgc2::xrpc::GrantPurpose::Runtime);
  const int runtime_fd = grant.duplicate_fd();
  assert(runtime_fd >= 0);

  World world(1700000000000000000LL);
  Sensors sensors(Json::object(), 1);
  Json entity = {{"name", "probe"}, {"kind", "scout"}};
  Json config = {{"paused", true}, {"input_poll_ns", 200000000}, {"entities", Json::array({entity})}};
  Inputs inputs;
  std::atomic<unsigned> frames{0}, reconciles{0};
  RuntimeIO io;
  io.attach_entity = [&](const std::shared_ptr<Entity> &item, const Json &) {
    item->io = std::make_shared<EntityIO>();
    item->io->reconcile = [&] { ++reconciles; };
    item->io->publish = [&](const State &, const TelemetryRates &) { ++frames; };
  };
  io.poll_inputs = [&] { inputs.drain(); };
  io.inputs_pending = [&] { return inputs.is_pending(); };
  io.bind_input_wake = [&](std::function<void()> wake) { inputs.bind(std::move(wake)); };
  RpcOptions rpc;
  rpc.target_id = "input-wake";
  rpc.run_id = "input-wake-run";
  rpc.retained_parent_fd = runtime_fd;
  rpc.frozen_experiment = true;
  rpc.robot_bindings = {{"probe", "probe"}};
  Server server(config, (directory / "world.sock").string(), std::move(rpc), world, sensors, std::move(io));
  volatile sig_atomic_t stopping = 0;
  std::atomic<bool> done{false};
  inputs.post(); // a callback queued before the owner installs its wake
  std::thread running([&] { server.run(stopping); done = true; });
  assert(inputs.wait_for(1));
  {
    std::lock_guard<std::mutex> lock(inputs.mutex);
    inputs.received.clear();
  }
  const auto first_deadline = Clock::now() + 3s;
  while ((!frames || !reconciles) && Clock::now() < first_deadline) std::this_thread::sleep_for(1ms);
  assert(frames && reconciles);
  std::this_thread::sleep_for(50ms); // allow the initial frame wake to settle
  const auto task = input_thread();
  const auto before_wakes = context_switches(task);
  const auto before_reconciles = reconciles.load();
  std::this_thread::sleep_for(250ms);
  const auto idle_wakes = context_switches(task) - before_wakes;
  const auto idle_reconciles = reconciles.load() - before_reconciles;
  std::cout << "{\"phase\":\"paused-idle\",\"input_idle_voluntary_switches\":" << idle_wakes
            << ",\"idle_entity_reconciles\":" << idle_reconciles << "}" << std::endl;
  assert(idle_wakes <= 8);
  assert(idle_reconciles == 0);

  inputs.post();
  assert(inputs.wait_for(1));
  inputs.post();
  std::this_thread::sleep_for(70ms);
  {
    std::lock_guard<std::mutex> lock(inputs.mutex);
    assert(inputs.received.size() == 1); // configured 200ms input deadline
  }
  assert(inputs.wait_for(2));
  {
    std::lock_guard<std::mutex> lock(inputs.mutex);
    assert(inputs.received[1] - inputs.received[0] >= 190ms);
    assert(inputs.received[1] - inputs.received[0] < 1s);
  }
  stopping = 1;
  inputs.post(); // model a wake racing Stop; real process SIGTERM writes the XRPC eventfd
  const auto stopped_by = Clock::now() + 3s;
  while (!done && Clock::now() < stopped_by) std::this_thread::sleep_for(1ms);
  assert(done);
  running.join();
  close(runtime_fd);
  close(parent);
  fs::remove_all(directory);
  std::cout << "{\"result\":\"PASS\",\"received\":2,\"input_idle_voluntary_switches\":"
            << idle_wakes << ",\"idle_entity_reconciles\":" << idle_reconciles << "}\n";
}
