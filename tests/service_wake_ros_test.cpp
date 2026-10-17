#include "io/native_rpc/server.hpp"
#include "io/ros/runtime.hpp"
#include <xgc2/xrpc/bootstrap.hpp>
#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <atomic>
#include <cassert>
#include <cstdlib>
#include <chrono>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <thread>
#include <vector>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/wait.h>
namespace {
const char *kUri = "http://127.0.0.1:33141";

pid_t spawn(const std::string &path, const std::vector<std::string> &args, const std::string &log) {
  pid_t pid = fork();
  assert(pid >= 0);
  if (pid == 0) {
    setpgid(0, 0);
    int fd = open(log.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    assert(fd >= 0);
    dup2(fd, STDOUT_FILENO);
    dup2(fd, STDERR_FILENO);
    int devnull = open("/dev/null", O_RDONLY);
    dup2(devnull, STDIN_FILENO);
    std::vector<char *> argv;
    argv.push_back(const_cast<char *>(path.c_str()));
    for (const auto &arg : args) argv.push_back(const_cast<char *>(arg.c_str()));
    argv.push_back(nullptr);
    execvp(path.c_str(), argv.data());
    _exit(127);
  }
  setpgid(pid, pid);
  return pid;
}
bool tcp_open() {
  int fd = socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(33141);
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  bool open = connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof address) == 0;
  close(fd);
  return open;
}
std::string http(const std::string &path, const std::string &method, const std::string &route,
                 const std::string &request_id, const std::string &instance, const std::string &body) {
  int fd = socket(AF_UNIX, SOCK_STREAM, 0);
  assert(fd >= 0);
  timeval tv{.tv_sec = 5, .tv_usec = 0};
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  assert(path.size() < sizeof address.sun_path);
  std::memcpy(address.sun_path, path.c_str(), path.size() + 1);
  assert(connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof address) == 0);
  std::string request = method + " " + route + " HTTP/1.1\r\nHost: localhost\r\n"
      "Content-Type: application/json\r\nX-Request-ID: " + request_id + "\r\nX-Xrpc-Timeout-Ms: 3000\r\n";
  if (!instance.empty()) request += "X-Xrpc-Instance-ID: " + instance + "\r\n";
  request += "Content-Length: " + std::to_string(body.size()) + "\r\nConnection: close\r\n\r\n" + body;
  assert(write(fd, request.data(), request.size()) == static_cast<ssize_t>(request.size()));
  std::string response;
  char buffer[1024];
  for (;;) {
    ssize_t n = read(fd, buffer, sizeof buffer);
    if (n <= 0) break;
    response.append(buffer, buffer + n);
  }
  close(fd);
  auto body_at = response.find("\r\n\r\n");
  assert(body_at != std::string::npos);
  if (response.compare(0, 12, "HTTP/1.1 200") != 0 && response.compare(0, 12, "HTTP/1.1 202") != 0) {
    std::cerr << response << '\n';
    assert(false);
  }
  return response.substr(body_at + 4);
}
bool arm_once() {
  ros::NodeHandle handle;
  auto client = handle.serviceClient<mavros_msgs::CommandBool>("/wake/mavros/cmd/arming");
  mavros_msgs::CommandBool call;
  call.request.value = true;
  return client.call(call);
}
}

int main(int argc, char **argv) {
  char pattern[] = "/tmp/xsim-service-wake-XXXXXX";
  assert(mkdtemp(pattern));
  const std::filesystem::path root(pattern);
  struct Cleanup {
    std::filesystem::path root;
    bool ok = false;
    ~Cleanup() { if (ok) std::filesystem::remove_all(root); }
  } cleanup{root};
  std::filesystem::create_directories(root / "ros");
  std::filesystem::permissions(root, std::filesystem::perms::owner_all);
  setenv("ROS_MASTER_URI", kUri, 1);
  setenv("ROS_IP", "127.0.0.1", 1);
  unsetenv("ROS_HOSTNAME");
  setenv("ROS_HOME", (root / "ros").c_str(), 1);
  setenv("HOME", root.c_str(), 1);
  const auto master = spawn("roscore", {"-p", "33141"}, (root / "roscore.log").string());
  const auto master_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
  while (!tcp_open() && std::chrono::steady_clock::now() < master_deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  assert(tcp_open());
  ros::init(argc, argv, "xsim_service_wake_ros", ros::init_options::NoSigintHandler);

  xsim::World world(1700000000000000000LL, 2000000, 8000000, 8, 10000000,
                    "0123456789abcdef0123456789abcdef");
  xsim::Sensors sensors(xsim::Json::object(), 1);
  xsim::Json config{{"entities", xsim::Json::array({
                        {{"name", "wake"}, {"kind", "fs150"}},
                        {{"name", "task"}, {"kind", "fs150"}},
                    })}};
  auto io = xsim::make_ros_io(config, world, sensors);
  std::atomic<uint64_t> polls{0};
  auto inner = std::move(io.poll_services);
  io.poll_services = [&polls, inner] {
    ++polls;
    return inner();
  };
  xsim::RpcOptions rpc;
  rpc.target_id = "wake-target";
  rpc.run_id = "wake-run";
  const int parent = open(root.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
  assert(parent >= 0);
  auto grant = xgc2::xrpc::DirectoryGrant::from_owned_directory(parent, xgc2::xrpc::GrantPurpose::Runtime);
  const int runtime_fd = grant.duplicate_fd();
  assert(runtime_fd >= 0);
  rpc.retained_parent_fd = runtime_fd;
  const auto socket = (root / "world.sock").string();
  xsim::Server server(config, socket, std::move(rpc), world, sensors, std::move(io));
  volatile sig_atomic_t stopping = 0;
  std::atomic<bool> server_done{false};
  std::thread running([&] { server.run(stopping); server_done = true; });

  ros::NodeHandle handle;
  auto client = handle.serviceClient<mavros_msgs::CommandBool>("/wake/mavros/cmd/arming");
  auto task_client = handle.serviceClient<mavros_msgs::CommandBool>("/task/mavros/cmd/arming");
  assert(client.waitForExistence(ros::Duration(15)));
  assert(task_client.waitForExistence(ros::Duration(15)));
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  const auto idle_before = polls.load();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  const auto idle_delta = polls.load() - idle_before;
  assert(idle_delta == 0);

  const auto described = nlohmann::json::parse(http(socket, "GET", "/v1/describe", "describe", {}, {}));
  const auto instance = described.at("instance_id").get<std::string>();
  const auto entity = nlohmann::json::parse(http(socket, "GET", "/v1/entities/wake", "entity", instance, {}));
  const auto generation = entity.at("ref").at("generation").get<uint64_t>();
  xsim::Json enable = xsim::Json::object();
  enable["generation"] = generation;
  enable["operation_timeout_ms"] = 3000;
  xsim::Json enabled_state = xsim::Json::object();
  enabled_state["enabled"] = true;
  enable["state"] = std::move(enabled_state);
  const auto enabled = nlohmann::json::parse(http(socket, "POST", "/v1/entities/wake/state", "enable", instance, enable.dump()));
  const auto enabled_done = nlohmann::json::parse(http(
      socket, "POST", "/v1/operations/" + enabled.at("id").get<std::string>() + "/wait", "enable-wait", instance, "{}"));
  assert(enabled_done.at("state") == "succeeded");

  constexpr int kBurst = 4;
  std::atomic<int> armed{0};
  std::vector<std::thread> burst;
  for (int i = 0; i < kBurst; ++i)
    burst.emplace_back([&] { if (arm_once()) ++armed; });
  const auto task = nlohmann::json::parse(http(socket, "GET", "/v1/entities/task", "task", instance, {}));
  const auto task_generation = task.at("ref").at("generation").get<uint64_t>();
  xsim::Json reset_body = xsim::Json::object();
  reset_body["generation"] = task_generation;
  reset_body["operation_timeout_ms"] = 3000;
  const auto reset = nlohmann::json::parse(http(socket, "POST", "/v1/entities/task/reset", "reset", instance, reset_body.dump()));
  const auto operation = nlohmann::json::parse(http(
      socket, "POST", "/v1/operations/" + reset.at("id").get<std::string>() + "/wait", "reset-wait", instance, "{}"));
  for (auto &thread : burst) thread.join();
  assert(armed == kBurst);
  assert(operation.at("state") == "succeeded");
  const auto after_burst = polls.load();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  const auto settled_delta = polls.load() - after_burst;
  assert(settled_delta == 0);

  std::atomic<bool> caller_done{false};
  std::thread caller([&] { arm_once(); caller_done = true; });
  stopping = 1;
  const auto stop_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
  while (!server_done && std::chrono::steady_clock::now() < stop_deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  assert(server_done.load());
  running.join();
  ros::shutdown();
  caller.join();
  (void)caller_done;
  kill(-master, SIGINT);
  waitpid(master, nullptr, 0);
  xsim::Json report = xsim::Json::object();
  report["result"] = "PASS";
  report["ros"] = true;
  report["idle_polls"] = idle_delta;
  report["burst"] = kBurst;
  report["native_task"] = operation.at("kind");
  report["settled_polls"] = settled_delta;
  report["shutdown"] = "joined";
  cleanup.ok = true;
  std::cout << report.dump() << '\n';
}
