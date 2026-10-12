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
    std::string config_path, experiment_path, manifest_path, socket, target_id, scene_file;
    bool has_experiment_stdin = false;
    bool has_config = false, has_experiment = false, has_manifest = false, has_socket = false, has_target = false, has_scene = false;
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      if (arg == "--help") {
        std::cout << "xsim --socket SOCKET --target-id TARGET (--config WORLD.json | --experiment-file EXPERIMENT.json | --experiment-stdin | --manifest MANIFEST.json) [--scene-file SCENE.yaml]\n";
        return 0;
      }
      if (arg == "--experiment-stdin") {
        if (has_experiment_stdin) throw std::invalid_argument("argument occurs more than once: " + arg);
        has_experiment_stdin = true;
        continue;
      }
      std::string *value = nullptr;
      bool *present = nullptr;
      if (arg == "--manifest") { value = &manifest_path; present = &has_manifest; }
      else if (arg == "--config") { value = &config_path; present = &has_config; }
      else if (arg == "--experiment-file") { value = &experiment_path; present = &has_experiment; }
      else if (arg == "--socket") { value = &socket; present = &has_socket; }
      else if (arg == "--target-id") { value = &target_id; present = &has_target; }
      else if (arg == "--scene-file") { value = &scene_file; present = &has_scene; }
      else throw std::invalid_argument("unknown argument: " + arg);
      if (*present) throw std::invalid_argument("argument occurs more than once: " + arg);
      if (i + 1 >= argc || std::string(argv[i + 1]).compare(0, 2, "--") == 0)
        throw std::invalid_argument("missing value for " + arg);
      *present = true;
      *value = argv[++i];
    }
    if (socket.empty() || target_id.empty() || int(has_config) + int(has_experiment) + int(has_manifest) + int(has_experiment_stdin) != 1 ||
        (has_config && config_path.empty()) || (has_experiment && experiment_path.empty()) ||
        (has_manifest && manifest_path.empty()))
      throw std::invalid_argument("--socket, --target-id and exactly one native world input are required");
    if (has_manifest && has_scene) throw std::invalid_argument("manifest world owns its scene artifact");
    if (socket.front() != '/' || socket.back() == '/' || socket.find('\0') != std::string::npos)
      throw std::invalid_argument("Unix socket requires an absolute filesystem path and filename");
    const auto runtime = allocated_directory(socket.substr(0, socket.rfind('/')), xgc2::xrpc::GrantPurpose::Runtime);
    // Validate every path component without replacing the retained owner allocation.
    xsim::ensure_socket_parent(socket);
    OwnedFd runtime_fd{runtime.duplicate_fd()};
    xsim::RpcOptions rpc;
    rpc.target_id = target_id;
    rpc.retained_parent_fd = runtime_fd.value;
    // The management host keeps the SDK's default HTTP limits (xgc2::xrpc::HttpLimits).
    xsim::Json config;
    if (has_experiment || has_experiment_stdin) {
      const auto frozen = has_experiment_stdin ? experiment_stdin() : xsim::load_json_object(experiment_path);
      const auto epoch_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count();
      config = xsim::resolve_scene_file(xsim::experiment_config(frozen, epoch_ns), scene_file);
      rpc.robot_bindings = xsim::experiment_robot_bindings(frozen);
      rpc.frozen_experiment = true;
    } else config = has_manifest ? xsim::load_manifest(manifest_path) : xsim::load_config(config_path, scene_file);
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
                     config.value("model_step_ns", int64_t(2000000)),
                     config.value("output_period_ns", int64_t(8000000)),
                     config.value("catchup_batch", 8u),
                     config.value("max_model_step_ns", int64_t(10000000)));
    xsim::Sensors sensors(config.value("scene", xsim::Json::object()),
                          config.value("sensor_workers", 2u));
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
