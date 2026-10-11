#pragma once
#include "systems/sensors.hpp"
#include "io/runtime_io.hpp"
#include "publishing.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <random>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
namespace xsim {
struct RosEntity {
  RosEntity(const std::shared_ptr<Entity> &, World &, ros::CallbackQueue &,
            ros::CallbackQueue &, const Json &config, std::shared_ptr<void> owner = {},
            std::shared_ptr<PublicationMetrics> telemetry_metrics = {},
            std::shared_ptr<PublicationMetrics> cloud_metrics = {});
  void bind(); // ROS connection worker only; publications become immutable after binding
  bool request_reconcile(); // event thread only; coalesce the desired graph state
  void reconcile(); // ROS connection worker only; bind immutable-generation callbacks
  void publish(const State &, const TelemetryRates &); // stable publication shard only
  bool publish_sensor(); // at most one completed scan; same shard only
  const Json ros_config;
  std::weak_ptr<Entity> entity;
  World &world;
  std::shared_ptr<void> runtime_owner;
  ros::CallbackQueue &input_queue, &service_queue;
  std::unique_ptr<ros::NodeHandle> input, services;
  const std::shared_ptr<PublicationMetrics> telemetry_metrics, cloud_metrics;
  std::string localization_pose_topic; // resolved once before the deferred advertisement
  std::atomic<bool> bound{false};
  std::atomic<uint64_t> desired_revision{1}, applied_revision{0};
  uint64_t requested_generation = UINT64_MAX;
  bool requested_enabled = false, requested_alive = false;
  BufferedPublisher localization_pose, localization_twist, pose, velocity, imu, raw_imu,
      state, extended, target, odom, cloud, beams;
  ros::Subscriber pva_sub, attitude_sub, velocity_sub;
  ros::ServiceServer arm, mode, command;
  uint64_t bound_generation = UINT64_MAX;
  bool bound_enabled = false, bound_alive = false;
  std::string frame, body_frame;
  std::mt19937 rng;
  std::normal_distribution<double> normal{0, 1};
  Eigen::Vector3d noise{Eigen::Vector3d::Zero()};
  bool last_enabled = false, last_armed = false;
  uint8_t last_landed_state = UINT8_MAX;
  std::string last_mode;
  int64_t last_stamp = -1;
  uint64_t last_generation = UINT64_MAX;
  TelemetrySchedule schedule;
  geometry_msgs::PoseStamped localization_pose_message, local_pose_message;
  geometry_msgs::TwistStamped localization_twist_message, local_twist_message;
  nav_msgs::Odometry odometry_message;
  sensor_msgs::Imu imu_message;
  mavros_msgs::State state_message;
  mavros_msgs::ExtendedState extended_message;
  mavros_msgs::AttitudeTarget target_message;
  sensor_msgs::PointCloud2 cloud_message, beam_message;
  std::string topic(const std::string &, const std::string &) const;
};
} // namespace xsim
