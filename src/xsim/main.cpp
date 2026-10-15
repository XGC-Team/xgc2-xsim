#include "io/config.hpp"
#include "io/native_rpc/server.hpp"
#include "io/startup.hpp"
#include <xgc2/xrpc/bootstrap.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <cstdlib>
#include <chrono>
#ifdef XSIM_ROS
#include "io/ros/runtime.hpp"
#include <ros/ros.h>
#endif
#include <csignal>
#include <array>
#include <iostream>
#include <limits>
#include <algorithm>
namespace {
struct OwnedFd {
  int value;
  ~OwnedFd() { if (value >= 0) ::close(value); }
};
xgc2::xrpc::DirectoryGrant allocated_directory(const std::string &path, xgc2::xrpc::GrantPurpose purpose) {
  OwnedFd fd{::open(path.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC)};
  if (fd.value < 0) throw std::invalid_argument("native directory allocation is unavailable");
  return xgc2::xrpc::DirectoryGrant::from_owned_directory(fd.value, purpose);
}
xsim::Json experiment_stdin() {
  constexpr size_t limit = 8u << 20;
  std::string bytes;
  std::array<char, 8192> chunk{};
  while (std::cin.read(chunk.data(), chunk.size()) || std::cin.gcount() != 0) {
    const auto count = static_cast<size_t>(std::cin.gcount());
    if (count > limit - bytes.size()) throw std::invalid_argument("Experiment stdin exceeds 8 MiB");
    bytes.append(chunk.data(), count);
  }
  if (!std::cin.eof()) throw std::invalid_argument("Experiment stdin read failed");
  return xsim::parse_json_object(bytes);
}
volatile sig_atomic_t stopping = 0;
void signal_stop(int) { stopping = 1; }
#ifdef XSIM_ROS
xgc2::xrpc::DirectoryGrant ros_directory(const char *name) {
  const char *path = std::getenv(name);
  if (!path || *path != '/')
    throw std::invalid_argument(std::string(name) + " requires an explicit absolute directory");
  return allocated_directory(path, xgc2::xrpc::GrantPurpose::Storage);
}
struct RosShutdown {
  ~RosShutdown() { ros::shutdown(); }
};
#endif
} // namespace
int main(int argc, char **argv) {
  try {
    std::string socket, target_id, run_id;
    unsigned sensor_initialization_timeout_ms = 10000;
    bool has_experiment_stdin = false, has_socket = false, has_target = false, has_run = false, has_sensor_timeout = false;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--help") {
        std::cout << "xsim --experiment-stdin --run-id RUN --socket SOCKET --target-id TARGET [--sensor-init-timeout-ms MS]\n";
        return 0;
      }
      if (arg == "--experiment-stdin") {
        if (has_experiment_stdin) throw std::invalid_argument("argument occurs more than once: " + arg);
        has_experiment_stdin = true;
        continue;
      }
      std::string *value = nullptr;
      bool *present = nullptr;
      std::string timeout;
      if (arg == "--socket") { value = &socket; present = &has_socket; }
      else if (arg == "--target-id") { value = &target_id; present = &has_target; }
      else if (arg == "--run-id") { value = &run_id; present = &has_run; }
      else if (arg == "--sensor-init-timeout-ms") { value = &timeout; present = &has_sensor_timeout; }
      else throw std::invalid_argument("unknown argument: " + arg);
      if (*present) throw std::invalid_argument("argument occurs more than once: " + arg);
      if (i + 1 >= argc || std::string(argv[i + 1]).compare(0, 2, "--") == 0)
        throw std::invalid_argument("missing value for " + arg);
      *present = true;
      *value = argv[++i];
      if (arg == "--sensor-init-timeout-ms") {
        if (timeout.empty() || timeout.find_first_not_of("0123456789") != std::string::npos || timeout.size() > 6)
          throw std::invalid_argument("sensor initialization timeout must be 1 through 600000 ms");
        sensor_initialization_timeout_ms = std::stoul(timeout);
        if (!sensor_initialization_timeout_ms || sensor_initialization_timeout_ms > 600000)
          throw std::invalid_argument("sensor initialization timeout must be 1 through 600000 ms");
      }
    }
    if (socket.empty() || target_id.empty() || run_id.empty() || !has_experiment_stdin)
      throw std::invalid_argument("--experiment-stdin, --run-id, --socket and --target-id are required");
    const auto valid_identity = [](const std::string& value) {
      return !value.empty() && value.size() <= 128 && std::all_of(value.begin(), value.end(), [](unsigned char c) {
        return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
               c == '-' || c == '_' || c == '.' || c == ':';
      });
    };
    if (!valid_identity(target_id) || !valid_identity(run_id))
      throw std::invalid_argument("target and run identifiers must be explicit bounded identifiers");
    if (socket.front() != '/' || socket.back() == '/' || socket.find('\0') != std::string::npos)
      throw std::invalid_argument("Unix socket requires an absolute filesystem path and filename");
    const auto runtime = allocated_directory(socket.substr(0, socket.rfind('/')), xgc2::xrpc::GrantPurpose::Runtime);
    // Validate every path component without replacing the retained owner allocation.
    xsim::ensure_socket_parent(socket);
    OwnedFd runtime_fd{runtime.duplicate_fd()};
    xsim::RpcOptions rpc;
    rpc.target_id = target_id;
    rpc.run_id = run_id;
    rpc.retained_parent_fd = runtime_fd.value;
    // The management host keeps the SDK's default HTTP limits (xgc2::xrpc::HttpLimits).
    const auto frozen = experiment_stdin();
    const auto epoch_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    auto config = xsim::resolve_scene_file(xsim::experiment_config(frozen, epoch_ns), "");
    config["run_id"] = run_id;
    rpc.robot_bindings = xsim::experiment_robot_bindings(frozen);
    rpc.frozen_experiment = true;
    const auto &epoch = config.at("epoch_ns");
    if (!epoch.is_number_integer() ||
        (epoch.is_number_unsigned() && epoch.get<uint64_t>() > uint64_t(std::numeric_limits<int64_t>::max())))
      throw std::invalid_argument("epoch_ns must be an explicit int64 integer from the frozen configuration");
#ifdef XSIM_ROS
    const std::array<xgc2::xrpc::DirectoryGrant, 2> ros_directories = {
      ros_directory("ROS_HOME"), ros_directory("ROS_LOG_DIR")};
    ros::init(argc, argv, "xsim", ros::init_options::NoSigintHandler);
    RosShutdown ros_shutdown;
#endif
    signal(SIGINT, signal_stop);
    signal(SIGTERM, signal_stop);
    xsim::World world(config.at("epoch_ns").get<int64_t>(),
                     config.at("model_step_ns").get<int64_t>(),
                     config.at("output_period_ns").get<int64_t>(),
                     config.value("catchup_batch", 8u),
                     config.value("max_model_step_ns", int64_t(10000000)));
    xsim::Sensors sensors(config.value("scene", xsim::Json::object()),
                          config.value("sensor_workers", 2u),
                          std::chrono::milliseconds(sensor_initialization_timeout_ms));
    xsim::RuntimeIO io;
#ifdef XSIM_ROS
    io = xsim::make_ros_io(config, world, sensors);
#endif
    xsim::Server server(config, socket, std::move(rpc), world, sensors, std::move(io));
    server.run(stopping);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "xsim: " << error.what() << '\n';
    return 1;
  }
}
