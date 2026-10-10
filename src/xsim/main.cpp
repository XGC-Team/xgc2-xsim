#include "io/config.hpp"
#include "io/native_rpc/server.hpp"
#include <xgc2/xrpc/bootstrap.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <cstdlib>
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
struct RosShutdown {
  ~RosShutdown() { ros::shutdown(); }
};
#endif
} // namespace
int main(int argc, char **argv) {
  try {
    std::string config_path, experiment_path, manifest_path, bootstrap_path, socket, scene_file;
    bool has_experiment_stdin = false;
    bool has_config = false, has_experiment = false, has_manifest = false, has_bootstrap = false, has_socket = false, has_scene = false;
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      if (arg == "--help") {
        std::cout << "xsim --bootstrap-input BOOTSTRAP.json (--config WORLD.json | --experiment-file EXPERIMENT.json | --experiment-stdin | --manifest MANIFEST.json) [--scene-file SCENE.yaml]\n";
        return 0;
      }
      if (arg == "--experiment-stdin") {
        if (has_experiment_stdin) throw std::invalid_argument("argument occurs more than once: " + arg);
        has_experiment_stdin = true;
        continue;
      }
      std::string *value = nullptr;
      bool *present = nullptr;
      if (arg == "--bootstrap-input") { value = &bootstrap_path; present = &has_bootstrap; }
      else if (arg == "--manifest") { value = &manifest_path; present = &has_manifest; }
      else if (arg == "--config") { value = &config_path; present = &has_config; }
      else if (arg == "--experiment-file") { value = &experiment_path; present = &has_experiment; }
      else if (arg == "--socket") { value = &socket; present = &has_socket; }
      else if (arg == "--scene-file") { value = &scene_file; present = &has_scene; }
      else throw std::invalid_argument("unknown argument: " + arg);
      if (*present) throw std::invalid_argument("argument occurs more than once: " + arg);
      if (i + 1 >= argc || std::string(argv[i + 1]).compare(0, 2, "--") == 0)
        throw std::invalid_argument("missing value for " + arg);
      *present = true;
      *value = argv[++i];
    }
    if (bootstrap_path.empty() || int(has_config) + int(has_experiment) + int(has_manifest) + int(has_experiment_stdin) != 1 ||
        (has_config && config_path.empty()) || (has_experiment && experiment_path.empty()) ||
        (has_manifest && manifest_path.empty()))
      throw std::invalid_argument("--bootstrap-input and exactly one native world input are required");
    if (has_manifest && has_scene) throw std::invalid_argument("manifest world owns its scene artifact");
    auto bootstrap = xgc2::xrpc::loadBootstrapInput(bootstrap_path);
    const auto &binding = bootstrap.binding();
    if (binding.service() != "xgc2.simulation" || binding.api_version() != "v1" ||
        binding.profile() != "http.v1" || binding.endpoint().kind != "unix" ||
        binding.authentication() != "local_private")
      throw std::invalid_argument("xsim requires local-private simulation HTTP v1");
    if (has_socket && socket != binding.endpoint().address)
      throw std::invalid_argument("socket does not match the bootstrap endpoint");
    socket = binding.endpoint().address;
    const auto runtime = bootstrap.resolve_runtime([&](const auto &, const auto &) {
      return allocated_directory(socket.substr(0, socket.rfind('/')), xgc2::xrpc::GrantPurpose::Runtime);
    });
    OwnedFd runtime_fd{runtime.duplicate_fd()};
    xsim::RpcOptions rpc;
    rpc.target_id = std::string(binding.target_id());
    rpc.retained_parent_fd = runtime_fd.value;
    // The management host keeps the SDK's default HTTP limits (xgc2::xrpc::HttpLimits).
    xsim::Json config;
    if (has_experiment || has_experiment_stdin) {
      const auto frozen = has_experiment_stdin ? experiment_stdin() : xsim::load_json_object(experiment_path);
      config = xsim::resolve_scene_file(xsim::experiment_config(frozen), scene_file);
      rpc.robot_bindings = xsim::experiment_robot_bindings(frozen);
      rpc.frozen_experiment = true;
    } else config = has_manifest ? xsim::load_manifest(manifest_path) : xsim::load_config(config_path, scene_file);
    const auto &epoch = config.at("epoch_ns");
    if (!epoch.is_number_integer() ||
        (epoch.is_number_unsigned() && epoch.get<uint64_t>() > uint64_t(std::numeric_limits<int64_t>::max())))
      throw std::invalid_argument("epoch_ns must be an explicit int64 integer from the frozen configuration");
#ifdef XSIM_ROS
    const auto application = bootstrap.application_json();
    if (!application) throw std::invalid_argument("ROS allocation grant names are required");
    const auto allocations = xsim::Json::parse(*application);
    if (!allocations.is_object() || allocations.size() != 2 ||
        !allocations.contains("rosHomeGrant") || !allocations.contains("rosLogGrant") ||
        !allocations.at("rosHomeGrant").is_string() || !allocations.at("rosLogGrant").is_string() ||
        allocations.at("rosHomeGrant") == allocations.at("rosLogGrant") || binding.storage_grants().size() != 2)
      throw std::invalid_argument("ROS requires distinct rosHomeGrant and rosLogGrant allocations");
    const auto storage = bootstrap.resolve_storage([&](const auto &handle, const auto &) {
      const char *name = nullptr;
      if (handle.matches(allocations.at("rosHomeGrant").get<std::string>())) name = "ROS_HOME";
      if (handle.matches(allocations.at("rosLogGrant").get<std::string>())) name = "ROS_LOG_DIR";
      if (!name) throw std::invalid_argument("unresolved ROS storage allocation");
      const char *path = std::getenv(name);
      if (!path || *path != '/') throw std::invalid_argument("ROS requires an explicit absolute storage directory");
      return allocated_directory(path, xgc2::xrpc::GrantPurpose::Storage);
    });
    ros::init(argc, argv, "xsim", ros::init_options::NoSigintHandler);
    RosShutdown ros_shutdown;
    ros::param::set("/use_sim_time", true);
#else
    if (!binding.storage_grants().empty())
      throw std::invalid_argument("headless xsim has no storage allocations");
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
