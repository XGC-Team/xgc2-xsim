#include "io/ros/runtime.hpp"
#include "scene_fixture.hpp"

#include <ros/ros.h>
#include <xmlrpcpp/XmlRpc.h>

#include <nlohmann/json.hpp>

#include <cassert>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <csignal>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>

using namespace xsim;

namespace {
const char *kAdapter = "/opt/ros/noetic/lib/xgc_scene_ros1_adapter/xgc2_scene_adapter";
const char *kRoot = "/tmp/xgc-p4k-scene-link";
const char *kUri = "http://127.0.0.1:33131";

pid_t spawn(const std::string &path, const std::vector<std::string> &args, const std::string &log) {
  const auto pid = fork();
  assert(pid >= 0);
  if (pid == 0) {
    setpgid(0, 0);
    const int output = open(log.c_str(), O_CREAT | O_WRONLY | O_TRUNC, 0644);
    const int input = open("/dev/null", O_RDONLY);
    if (output >= 0) { dup2(output, 1); dup2(output, 2); }
    if (input >= 0) dup2(input, 0);
    if (output > 2) close(output);
    if (input > 2) close(input);
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
  const int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) return false;
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(33131);
  inet_pton(AF_INET, "127.0.0.1", &address.sin_addr);
  const bool open = connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof address) == 0;
  close(fd);
  return open;
}

bool unix_open(const std::string &path) {
  const int fd = socket(AF_UNIX, SOCK_STREAM, 0);
  if (fd < 0) return false;
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  assert(path.size() < sizeof address.sun_path);
  std::strncpy(address.sun_path, path.c_str(), sizeof address.sun_path - 1);
  const bool open = connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof address) == 0;
  close(fd);
  return open;
}

std::string http(const std::string &path, const std::string &method, const std::string &route,
                 const std::string &request_id, const std::string &instance, const std::string &body) {
  const int fd = socket(AF_UNIX, SOCK_STREAM, 0);
  assert(fd >= 0);
  timeval budget{35, 0};
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &budget, sizeof budget);
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  std::strncpy(address.sun_path, path.c_str(), sizeof address.sun_path - 1);
  assert(connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof address) == 0);
  std::string request = method + " " + route + " HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n"
      "X-Xrpc-Timeout-Ms: 35000\r\nX-Request-ID: " + request_id + "\r\n";
  if (!instance.empty()) request += "X-Xrpc-Instance-ID: " + instance + "\r\n";
  if (!body.empty()) request += "Content-Type: application/json\r\n";
  request += "Content-Length: " + std::to_string(body.size()) + "\r\n\r\n" + body;
  assert(write(fd, request.data(), request.size()) == static_cast<ssize_t>(request.size()));
  std::string response;
  char buffer[4096];
  for (;;) {
    const auto count = read(fd, buffer, sizeof buffer);
    if (count <= 0) break;
    response.append(buffer, buffer + count);
  }
  close(fd);
  const auto split = response.find("\r\n\r\n");
  assert(split != std::string::npos);
  assert(response.compare(0, 12, "HTTP/1.1 200") == 0);
  return response.substr(split + 4);
}

std::vector<std::string> topic_nodes(const char *kind, const std::string &topic) {
  XmlRpc::XmlRpcValue args, result, payload;
  args[0] = ros::this_node::getName();
  assert(ros::master::execute("getSystemState", args, result, payload, true));
  const auto &table = payload[std::string(kind) == "publishers" ? 0 : 1];
  std::vector<std::string> names;
  for (int i = 0; i < table.size(); ++i) {
    if (std::string(table[i][0]) != topic) continue;
    for (int n = 0; n < table[i][1].size(); ++n) names.emplace_back(table[i][1][n]);
  }
  return names;
}

Json compiled_scene() {
  const Json box = {{"type", "box"}, {"size", {1.0, 4.0, 4.0}}};
  const Json identity = {{"position", {0.0, 0.0, 0.0}}, {"orientation", {0.0, 0.0, 0.0, 1.0}}};
  Json document = {{"schema", "xgc2.scene.compiled.v1"}, {"id", "link-proof"}, {"frame", "world"},
                   {"ground", {{"mode", "none"}}}, {"geometries", Json::array({box})},
                   {"models", {{"body", {{"parts", Json::array({{{"id", "body"}, {"pose", identity}, {"geometryIndex", 0}}})}}}}},
                   {"obstacles", Json::array({
                       {{"id", "shuttle"}, {"name", "Shuttle"}, {"model", "body"},
                        {"pose", {{"position", {4.0, 0.0, 0.0}}, {"orientation", {0.0, 0.0, 0.0, 1.0}}}},
                        {"motion", {{"type", "ping_pong"}}}},
                       {{"id", "orbiter"}, {"name", "Orbiter"}, {"model", "body"},
                        {"pose", {{"position", {2.0, 25.0, 0.0}}, {"orientation", {0.0, 0.0, 0.0, 1.0}}}},
                        {"motion", {{"type", "circle"}}}}})}};
  return {{"document", std::move(document)}};
}
}

int main(int argc, char **argv) {
  std::filesystem::remove_all(kRoot);
  std::filesystem::create_directories(std::filesystem::path(kRoot) / "ros");
  std::filesystem::permissions(kRoot, std::filesystem::perms::owner_all);
  const auto scene_path = std::filesystem::path(kRoot) / "scene.yaml";
  const auto scene_socket = std::filesystem::path(kRoot) / "scene.sock";
  const auto lidar_socket = std::filesystem::path(kRoot) / "lidar.sock";
  {
    std::ofstream scene(scene_path);
    scene << R"({
      "schema": "xgc2.scene.v1",
      "id": "link-proof",
      "frame": "world",
      "obstacles": [
        {"id": "shuttle", "pose": {"position": [4, 0, 0], "orientation": [0, 0, 0, 1]},
         "motion": {"type": "ping_pong", "point_a": [4, 0, 0], "point_b": [8, 0, 0], "speed": 4},
         "parts": [{"id": "body", "geometry": {"type": "box", "size": [1, 4, 4]}}]},
        {"id": "orbiter", "pose": {"position": [2, 25, 0], "orientation": [0, 0, 0, 1]},
         "motion": {"type": "circle", "center": [0, 25, 0], "radius": 2, "angular_speed": 1},
         "parts": [{"id": "body", "geometry": {"type": "box", "size": [1, 4, 4]}}]}
      ]
    })";
  }
  setenv("ROS_MASTER_URI", kUri, 1);
  setenv("ROS_IP", "127.0.0.1", 1);
  unsetenv("ROS_HOSTNAME");
  setenv("ROS_HOME", (std::filesystem::path(kRoot) / "ros").c_str(), 1);
  setenv("HOME", kRoot, 1);
  const auto master = spawn("roscore", {"-p", "33131"}, (std::filesystem::path(kRoot) / "roscore.log").string());
  const auto master_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
  while (!tcp_open() && std::chrono::steady_clock::now() < master_deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  assert(tcp_open());
  // xsim profile rejects play. The same scene_node binary's physical profile is the
  // existing entry that publishes this namespace's snapshot and 30 Hz state.
  const auto adapter = spawn(kAdapter, {
      "--socket-path", scene_socket.string(), "--target-id", "fixture", "--run-id", "link-proof",
      "--node-name", "scene_link_proof", "--ros-master-uri", kUri, "--ros-ip", "127.0.0.1",
      "_scene_file:=" + scene_path.string(), "_profile:=physical", "__ns:=/xgc_scene",
      "_lidar_xrpc_socket:=" + lidar_socket.string()}, (std::filesystem::path(kRoot) / "adapter.log").string());
  const auto adapter_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  while (!unix_open(scene_socket.string()) && std::chrono::steady_clock::now() < adapter_deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  if (!unix_open(scene_socket.string())) {
    std::cerr << std::ifstream(std::filesystem::path(kRoot) / "adapter.log").rdbuf();
    assert(false);
  }
  const auto described = nlohmann::json::parse(http(scene_socket.string(), "GET", "/v1/describe?wait_ready_ms=30000", "ready", {}, {}));
  assert(described.at("ready") == true);
  const auto instance = described.at("instance_id").get<std::string>();
  const auto envelope = nlohmann::json::parse(http(scene_socket.string(), "GET", "/v1/scene", "scene", instance, {}));

  ros::init(argc, argv, "xsim_scene_link");
  assert(ros::master::check());
  const auto publishers = topic_nodes("publishers", "/xgc_scene/snapshot");
  assert(publishers.size() == 1 && publishers[0] != ros::this_node::getName());
  assert(topic_nodes("subscribers", "/xgc_scene/snapshot").empty());
  assert(topic_nodes("subscribers", "/xgc_scene/state").empty());

  World world(1700000000000000000LL);
  Sensors still(sceneFixture({{"obstacles", Json::array({{{"type", "box"}, {"position", {4, 0, 0}}, {"size", {1, 1, 1}}}})}}), 2);
  Sensors moving(compiled_scene(), 2);
  auto quiet = make_ros_io(Json::object(), world, still);
  auto live = make_ros_io(Json::object(), world, moving);
  quiet.start_outputs();
  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  assert(topic_nodes("subscribers", "/xgc_scene/snapshot").empty());
  assert(topic_nodes("subscribers", "/xgc_scene/state").empty());
  live.start_outputs();
  auto robot = std::make_shared<Entity>(parse_entity({{"name", "link"}, {"kind", "scout"}}));
  robot->id = 3;
  robot->alive = robot->enabled = true;
  robot->generation = 1;
  robot->sensor = moving.prepare(robot, {{"backend", "cpu"}, {"h_fov_deg", 20}, {"v_fov_deg", 20},
                                         {"h_res", 4}, {"v_res", 2}, {"range", 30}});
  const auto ready_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
  while (!moving.scene_ready() && std::chrono::steady_clock::now() < ready_deadline) {
    live.poll_inputs();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  assert(moving.scene_ready() && moving.scene_version() == 1);
  assert(topic_nodes("subscribers", "/xgc_scene/snapshot").size() == 1);
  assert(topic_nodes("subscribers", "/xgc_scene/state").size() == 1);
  assert(still.scene_ready() && still.scene_version() == 1);

  State sample;
  sample.entity = robot;
  sample.key = {robot->id, robot->generation};
  sample.stamp = 1700000000000000000LL;
  moving.submit(sample);
  std::vector<uint8_t> cloud, beams;
  Sample rest;
  const auto rest_deadline = Clock::now() + std::chrono::seconds(3);
  bool got = false;
  while (!got && Clock::now() < rest_deadline) {
    live.poll_inputs();
    got = Sensors::take(robot->sensor, rest, cloud, &beams);
    if (!got) std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  assert(got && rest.scene && rest.scene_version == 1);
  assert(std::abs(rest.scene->obstacles()[0].position.x() - 4) < 1e-6);
  const auto rest_mean = [&] {
    double sum = 0;
    int count = 0;
    for (size_t i = 0; i < cloud.size(); i += 12) {
      float x = 0;
      std::memcpy(&x, cloud.data() + i, sizeof x);
      sum += x;
      ++count;
    }
    assert(count > 0);
    return sum / count;
  }();
  assert(moving.scene_version() == 1);

  nlohmann::json play_body = nlohmann::json::object();
  play_body["operation"] = "play";
  play_body["expectedEpoch"] = envelope.at("epoch");
  play_body["expectedRevision"] = 1;
  const auto played = nlohmann::json::parse(http(scene_socket.string(), "POST", "/v1/commands", "play", instance, play_body.dump()));
  assert(played.at("success") == true && played.at("playing") == true);
  const auto motion_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(400);
  while (std::chrono::steady_clock::now() < motion_deadline) {
    live.poll_inputs();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  sample.stamp += 200000000;
  moving.submit(sample);
  Sample claimed;
  got = false;
  const auto claimed_deadline = Clock::now() + std::chrono::seconds(3);
  while (!got && Clock::now() < claimed_deadline) {
    live.poll_inputs();
    got = Sensors::take(robot->sensor, claimed, cloud, &beams);
    if (!got) std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  assert(got && claimed.scene.get() == rest.scene.get() && claimed.scene_version == 1);
  assert(moving.scene_ready() && moving.scene_version() >= 2);
  sample.stamp += 200000000;
  moving.submit(sample);
  Sample moved;
  got = false;
  const auto moved_deadline = Clock::now() + std::chrono::seconds(3);
  while (!got && Clock::now() < moved_deadline) {
    live.poll_inputs();
    got = Sensors::take(robot->sensor, moved, cloud, &beams);
    if (!got) std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  assert(got && moved.scene && moved.scene.get() != rest.scene.get() && moved.scene_version >= 2);
  assert(moved.scene->obstacles()[0].position.x() > 4.5);
  double moved_sum = 0;
  int moved_count = 0;
  for (size_t i = 0; i < cloud.size(); i += 12) {
    float x = 0;
    std::memcpy(&x, cloud.data() + i, sizeof x);
    moved_sum += x;
    ++moved_count;
  }
  assert(moved_count > 0 && moved_sum / moved_count > rest_mean + 0.4);
  assert(moving.scene_ready());
  assert(topic_nodes("subscribers", "/xgc_scene/snapshot").size() == 1);
  assert(still.scene_version() == 1);

  live.stop_outputs();
  quiet.stop_outputs();
  ros::shutdown();
  kill(-adapter, SIGTERM);
  kill(-master, SIGINT);
  waitpid(adapter, nullptr, 0);
  waitpid(master, nullptr, 0);
  std::cout << "{\"result\":\"PASS\",\"snapshot\":\"latched\",\"state\":\"30Hz\"}\n";
}
