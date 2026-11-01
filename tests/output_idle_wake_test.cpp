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

namespace {
using namespace xsim;
namespace fs = std::filesystem;

fs::path output_thread() {
  for (const auto &entry : fs::directory_iterator("/proc/self/task")) {
    std::ifstream comm(entry.path() / "comm");
    std::string name;
    std::getline(comm, name);
    if (name == "xsim-output") return entry.path();
  }
  assert(false && "actual output thread is missing");
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
  assert(false && "actual output thread status is missing");
  return 0;
}
}

int main() {
  using namespace std::chrono_literals;
  char temporary[] = "/tmp/xsim-output-wake-XXXXXX";
  assert(mkdtemp(temporary));
  const fs::path directory(temporary);
  const int parent = open(directory.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
  assert(parent >= 0);
  auto grant = xgc2::xrpc::DirectoryGrant::from_owned_directory(parent, xgc2::xrpc::GrantPurpose::Runtime);
  const int runtime_fd = grant.duplicate_fd();
  assert(runtime_fd >= 0);

  World world(1700000000000000000LL);
  Sensors sensors(Json::object(), 1);
  Json entity = {{"name", "probe"}, {"kind", "scout"},
                 {"sensor", {{"backend", "cpu"}, {"h_res", 4}, {"v_res", 2}, {"rate_hz", 10}}}};
  Json config = {{"paused", true}, {"entities", Json::array({entity})}};
  std::mutex mutex;
  std::condition_variable changed;
  unsigned frames = 0, samples = 0;
  RuntimeIO io;
  io.attach_entity = [&](const std::shared_ptr<Entity> &item, const Json &) {
    item->io = std::make_shared<EntityIO>();
    item->io->publish = [&](const State &, const TelemetryRates &) {
      std::lock_guard<std::mutex> lock(mutex);
      ++frames;
      changed.notify_all();
    };
    item->io->publish_sensor = [&](const Sample &, const std::shared_ptr<const SensorPayload> &payload) {
      assert(payload);
      std::lock_guard<std::mutex> lock(mutex);
      ++samples;
      changed.notify_all();
    };
  };
  RpcOptions rpc;
  rpc.target_id = "output-wake";
  rpc.run_id = "output-wake-run";
  rpc.retained_parent_fd = runtime_fd;
  rpc.frozen_experiment = true;
  rpc.robot_bindings = {{"probe", "probe"}};
  Server server(config, (directory / "world.sock").string(), std::move(rpc), world, sensors, std::move(io));
  volatile sig_atomic_t stopping = 0;
  std::atomic<bool> done{false};
  std::thread running([&] { server.run(stopping); done = true; });
  {
    std::unique_lock<std::mutex> lock(mutex);
    assert(changed.wait_for(lock, 5s, [&] { return frames && samples; }));
    assert(frames == 1 && samples == 1 && world.metrics.steps == 0);
  }
  const auto task = output_thread();
  const auto before = context_switches(task);
  std::this_thread::sleep_for(250ms);
  const auto idle_wakes = context_switches(task) - before;
  // A paused world has no next frame. The real output worker must sleep until
  // a completion or Stop, not wake every millisecond to inspect empty queues.
  std::cout << "{\"phase\":\"paused-idle\",\"output_idle_voluntary_switches\":"
            << idle_wakes << "}" << std::endl;
  assert(idle_wakes <= 8);

  stopping = 1;
  const auto until = Clock::now() + 3s;
  while (!done && Clock::now() < until) std::this_thread::sleep_for(1ms);
  assert(done);
  running.join();
  close(runtime_fd);
  close(parent);
  fs::remove_all(directory);
  std::cout << "{\"result\":\"PASS\",\"paused_frames\":" << frames
            << ",\"completed_samples\":" << samples
            << ",\"output_idle_voluntary_switches\":" << idle_wakes << "}\n";
}
