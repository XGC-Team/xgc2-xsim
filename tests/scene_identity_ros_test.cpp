#include "io/ros/runtime.hpp"
#include "scene_fixture.hpp"

#include <ros/ros.h>

#include <xgc2_geometry_msgs/SceneSnapshot.h>
#include <xgc2_geometry_msgs/SceneState.h>

#include <cassert>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <csignal>
#include <sys/wait.h>
#include <unistd.h>

using namespace xsim;

namespace {
Json dynamic_document() {
  const Json box = {{"type", "box"}, {"size", {1.0, 4.0, 4.0}}};
  const Json identity = {{"position", {0.0, 0.0, 0.0}}, {"orientation", {0.0, 0.0, 0.0, 1.0}}};
  const Json local = {{"position", {0.0, 0.5, 0.0}}, {"orientation", {0.0, 0.0, 0.0, 1.0}}};
  Json document = {{"schema", "xgc2.scene.compiled.v1"}, {"id", "dynamic"}, {"frame", "world"},
                   {"ground", {{"mode", "none"}}}, {"geometries", Json::array({box})},
                   {"models", {{"mover", {{"parts", Json::array({{{"id", "body"}, {"pose", local}, {"geometryIndex", 0}}})}}},
                               {"post", {{"parts", Json::array({{{"id", "body"}, {"pose", identity}, {"geometryIndex", 0}}})}}}}},
                   {"obstacles", Json::array({
                       {{"id", "mover"}, {"name", "Mover"}, {"model", "mover"},
                        {"pose", {{"position", {4.0, 0.0, 0.0}}, {"orientation", {0.0, 0.0, 0.0, 1.0}}}},
                        {"motion", {{"type", "constant_twist"}}}},
                       {{"id", "post"}, {"name", "Post"}, {"model", "post"},
                        {"pose", {{"position", {0.0, 30.0, 0.0}}, {"orientation", {0.0, 0.0, 0.0, 1.0}}}},
                        {"motion", {{"type", "hold"}}}}})}};
  return {{"document", std::move(document)}};
}

xgc2_geometry_msgs::SceneSnapshot snapshot_message(uint64_t revision) {
  xgc2_geometry_msgs::SceneSnapshot message;
  message.header.frame_id = "world";
  message.scene_id = "dynamic";
  message.epoch = "store-epoch";
  message.revision = revision;
  xgc2_geometry_msgs::SceneObstacle mover;
  mover.id = "mover";
  mover.name = "Mover";
  mover.dynamic = true;
  mover.motion_type = "constant_twist";
  mover.pose.orientation.w = 1;
  mover.pose.position.x = 4;
  xgc2_geometry_msgs::SceneObstacle post;
  post.id = "post";
  post.name = "Post";
  post.dynamic = false;
  post.motion_type = "hold";
  post.pose.orientation.w = 1;
  post.pose.position.y = 30;
  message.obstacles.push_back(mover);
  message.obstacles.push_back(post);
  return message;
}

xgc2_geometry_msgs::SceneState state_message(uint64_t revision, double x) {
  xgc2_geometry_msgs::SceneState message;
  message.header.frame_id = "world";
  message.epoch = "store-epoch";
  message.revision = revision;
  xgc2_geometry_msgs::SceneObstacleState mover;
  mover.id = "mover";
  mover.pose.position.x = x;
  mover.pose.orientation.w = 1;
  message.obstacles.push_back(mover);
  return message;
}

pid_t start_master(const std::string &uri) {
  const auto pid = fork();
  assert(pid >= 0);
  if (pid == 0) {
    setenv("ROS_MASTER_URI", uri.c_str(), 1);
    setenv("ROS_IP", "127.0.0.1", 1);
    setenv("ROS_HOSTNAME", "127.0.0.1", 1);
    execlp("roscore", "roscore", "-p", "33121", nullptr);
    _exit(127);
  }
  return pid;
}
}

int main(int argc, char **argv) {
  const char *uri = "http://127.0.0.1:33121";
  setenv("ROS_MASTER_URI", uri, 1);
  setenv("ROS_IP", "127.0.0.1", 1);
  setenv("ROS_HOSTNAME", "127.0.0.1", 1);
  const auto master = start_master(uri);
  ros::init(argc, argv, "xsim_scene_identity_proof");
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
  while (!ros::master::check() && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  assert(ros::master::check());

  ros::NodeHandle node;
  auto snapshots = node.advertise<xgc2_geometry_msgs::SceneSnapshot>("/xgc_scene/snapshot", 1, true);
  auto states = node.advertise<xgc2_geometry_msgs::SceneState>("/xgc_scene/state", 1); // 30 Hz stream, not latched

  World world(1700000000000000000LL);
  Sensors still(sceneFixture({{"obstacles", Json::array({{{"type", "box"}, {"position", {4, 0, 0}}, {"size", {1, 1, 1}}}})}}), 2);
  Sensors moving(dynamic_document(), 2);
  auto quiet = make_ros_io(Json::object(), world, still);
  auto live = make_ros_io(Json::object(), world, moving);
  quiet.start_outputs();
  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  assert(snapshots.getNumSubscribers() == 0);
  live.start_outputs();
  const auto subscribed = std::chrono::steady_clock::now() + std::chrono::seconds(8);
  while (snapshots.getNumSubscribers() < 1 && std::chrono::steady_clock::now() < subscribed)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  assert(snapshots.getNumSubscribers() == 1);

  auto robot = std::make_shared<Entity>(parse_entity({{"name", "proof"}, {"kind", "scout"}}));
  robot->id = 3;
  robot->alive = robot->enabled = true;
  robot->generation = 1;
  robot->sensor = moving.prepare(robot, {{"backend", "cpu"}, {"h_fov_deg", 10}, {"v_fov_deg", 10},
                                         {"h_res", 4}, {"v_res", 2}, {"range", 30}});
  snapshots.publish(snapshot_message(1));
  states.publish(state_message(1, 4));
  const auto ready_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!moving.scene_ready() && std::chrono::steady_clock::now() < ready_deadline) {
    live.poll_inputs();
    quiet.poll_inputs();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  assert(moving.scene_ready() && moving.scene_version() == 1);
  assert(still.scene_ready() && still.scene_version() == 1);

  State sample;
  sample.entity = robot;
  sample.key = {robot->id, robot->generation};
  sample.stamp = 1700000000000000000LL;
  sample.orientation = Eigen::Quaterniond::Identity();
  moving.submit(sample);
  std::vector<uint8_t> cloud, beams;
  Sample taken;
  const auto scan_deadline = Clock::now() + std::chrono::seconds(3);
  bool got = false;
  while (!got && Clock::now() < scan_deadline) {
    live.poll_inputs();
    got = Sensors::take(robot->sensor, taken, cloud, &beams);
    if (!got) std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  assert(got && taken.scene && taken.scene_version == 1);
  assert(std::abs(taken.scene->obstacles()[0].position.x() - 4) < 1e-9);

  const auto version = moving.scene_version();
  snapshots.publish(snapshot_message(2));
  states.publish(state_message(2, 9));
  const auto invalidated = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (moving.scene_ready() && std::chrono::steady_clock::now() < invalidated) {
    live.poll_inputs();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  assert(!moving.scene_ready() && moving.scene_version() == version);
  states.publish(state_message(1, 3));
  for (int i = 0; i < 20; ++i) {
    live.poll_inputs();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  assert(!moving.scene_ready() && moving.scene_version() == version);
  assert(snapshots.getNumSubscribers() == 1);

  live.stop_outputs();
  quiet.stop_outputs();
  ros::shutdown();
  kill(master, SIGINT);
  waitpid(master, nullptr, 0);
  std::cout << "{\"result\":\"PASS\",\"subscribers\":1}\n";
}
