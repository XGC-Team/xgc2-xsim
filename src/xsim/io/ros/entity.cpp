#include "entity.hpp"
#include "point_cloud_view.hpp"
#include <cstring>
namespace xsim {
namespace {
Ticket ticket(const std::shared_ptr<Entity> &e, Op op, uint64_t generation) {
  auto t = std::make_shared<Command>();
  t->op = op;
  t->key = {e->id, generation};
  return t;
}
void vector(geometry_msgs::Vector3 &o, const Eigen::Vector3d &v) {
  o.x = v.x();
  o.y = v.y();
  o.z = v.z();
}
void orientation(geometry_msgs::Quaternion &o, const Eigen::Quaterniond &q) {
  o.w = q.w();
  o.x = q.x();
  o.y = q.y();
  o.z = q.z();
}
bool current(const std::shared_ptr<Entity> &e, uint64_t generation) {
  return e->alive && e->enabled && e->generation == generation;
}
} // namespace
std::string RosEntity::topic(const std::string &key,
                             const std::string &suffix) const {
  auto e = entity.lock();
  return ros_config.value(key, "/" + e->config.name + suffix);
}
RosEntity::RosEntity(const std::shared_ptr<Entity> &e, World &w,
                     ros::CallbackQueue &iq, ros::CallbackQueue &sq,
                     const Json &config, std::shared_ptr<void> owner,
                     std::shared_ptr<PublicationMetrics> telemetry_metrics,
                     std::shared_ptr<PublicationMetrics> cloud_metrics)
    : ros_config(config), entity(e), world(w), runtime_owner(std::move(owner)),
      rng(ros_config.value("mocap_seed", 1u)), schedule(w.epoch) {
  for (const char *key : {"mocap_topic", "mocap_velocity_topic"})
    if (ros_config.contains(key))
      throw std::invalid_argument("unsupported ROS configuration: " + std::string(key));
  input.setCallbackQueue(&iq);
  services.setCallbackQueue(&sq);
  frame = ros_config.value(
      "frame", std::string(e->config.kind == Kind::FS150 ? "map" : "world"));
  body_frame = ros_config.value("body_frame", std::string("base_link"));
  localization_pose_message.header.frame_id = "world";
  localization_twist_message.header.frame_id = "world";
  local_pose_message.header.frame_id = frame;
  local_twist_message.header.frame_id = frame;
  odometry_message.header.frame_id = frame;
  odometry_message.child_frame_id = body_frame;
  imu_message.header.frame_id = body_frame;
  target_message.header.frame_id = frame;
  last_mode.reserve(31);
  state_message.mode.reserve(31);
  if (ros_config.contains("mocap_noise")) {
    const auto &n = ros_config.at("mocap_noise");
    if (n.size() != 3)
      throw std::invalid_argument("mocap_noise needs 3 standard deviations");
    for (int a = 0; a < 3; ++a)
      noise[a] = n[a].get<double>();
    if (!noise.allFinite() || (noise.array() < 0).any())
      throw std::invalid_argument("invalid mocap noise");
  }
  auto pose_advertisement = input.advertise<geometry_msgs::PoseStamped>(
      topic("localization_pose_topic", "/pose"), 1);
  localization_pose_topic = pose_advertisement.getTopic();
  localization_pose.bind(std::move(pose_advertisement), telemetry_metrics);
  localization_twist.bind(input.advertise<geometry_msgs::TwistStamped>(
      topic("localization_twist_topic", "/twist"), 1), telemetry_metrics);
  const bool flight = e->config.kind == Kind::FS150;
  if (flight) {
    pose.bind(input.advertise<geometry_msgs::PoseStamped>(
        topic("pose_topic", "/mavros/local_position/pose"), 1), telemetry_metrics);
    velocity.bind(input.advertise<geometry_msgs::TwistStamped>(
        topic("velocity_topic", "/mavros/local_position/velocity_local"), 1), telemetry_metrics);
    odom.bind(input.advertise<nav_msgs::Odometry>(
        topic("odometry_topic", "/mavros/local_position/odom"), 1), telemetry_metrics);
    imu.bind(input.advertise<sensor_msgs::Imu>(
        topic("imu_topic", "/mavros/imu/data"), 1), telemetry_metrics);
    raw_imu.bind(input.advertise<sensor_msgs::Imu>(
        topic("raw_imu_topic", "/mavros/imu/data_raw"), 1), telemetry_metrics);
    state.bind(input.advertise<mavros_msgs::State>(
        topic("state_topic", "/mavros/state"), 1), telemetry_metrics);
    extended.bind(input.advertise<mavros_msgs::ExtendedState>(
        topic("extended_state_topic", "/mavros/extended_state"), 1), telemetry_metrics);
    target.bind(input.advertise<mavros_msgs::AttitudeTarget>(
        topic("target_attitude_topic", "/mavros/setpoint_raw/target_attitude"),
        1), telemetry_metrics);
  } else if (e->config.kind == Kind::Scout) {
    raw_imu.bind(input.advertise<sensor_msgs::Imu>(
        topic("raw_imu_topic", "/imu/data_raw"), 1), telemetry_metrics);
  } else {
    imu.bind(input.advertise<sensor_msgs::Imu>(
        topic("imu_topic", "/imu"), 1), telemetry_metrics);
  }
  if (e->sensor) {
    cloud_message.header.frame_id = e->sensor->frame;
    beam_message.header.frame_id = e->sensor->frame;
    cloud.bind(input.advertise<sensor_msgs::PointCloud2>(e->sensor->topic, 1), cloud_metrics);
    cloud_message.fields.resize((e->sensor->gpu || e->sensor->with_bodies) ? 4 : 3);
    for (unsigned a = 0; a < cloud_message.fields.size(); ++a) {
      auto &f = cloud_message.fields[a];
      f.name = a == 3 ? (e->sensor->gpu ? "intensity" : "vehicle_id") : std::string(1, "xyz"[a]);
      f.offset = a * 4;
      f.datatype = a==3 && e->sensor->with_bodies ? sensor_msgs::PointField::INT32 : sensor_msgs::PointField::FLOAT32;
      f.count = 1;
    }
    cloud_message.point_step = (e->sensor->gpu || e->sensor->with_bodies) ? 16 : 12;
    cloud_message.height = 1;
    cloud_message.is_dense = true;
    cloud_message.is_bigendian = false;
    if(e->sensor->publish_beams){
      beams.bind(input.advertise<sensor_msgs::PointCloud2>("/"+e->config.name+"/simple_lidar/beams",1), cloud_metrics);
      beam_message.height=1;beam_message.is_dense=true;beam_message.point_step=e->sensor->with_bodies?36:32;
      const char* names[]={"x","y","z","dx","dy","dz","range","hit","vehicle_id"};
      for(unsigned a=0;a<(e->sensor->with_bodies?9u:8u);++a){sensor_msgs::PointField f;f.name=names[a];f.offset=a*4;f.datatype=a==8?sensor_msgs::PointField::INT32:sensor_msgs::PointField::FLOAT32;f.count=1;beam_message.fields.push_back(f);}
    }
  }
  reconcile();
}
void RosEntity::reconcile() {
  auto e = entity.lock();
  if (!e)
    return;
  const uint64_t gen = e->generation;
  const bool enabled = e->alive && e->enabled;
  if (bound_generation == gen && bound_enabled == enabled &&
      bound_alive == e->alive)
    return;
  // Enablement changes input admission, not this generation's service
  // endpoint. Re-advertising here disconnects persistent ROS clients that
  // connected while the entity was disabled.
  const bool keep_services = bound_generation == gen && bound_alive && e->alive;
  bound_generation = gen;
  bound_enabled = enabled;
  bound_alive = e->alive;
  pva_sub.shutdown();
  attitude_sub.shutdown();
  velocity_sub.shutdown();
  if (!keep_services) {
    arm.shutdown();
    mode.shutdown();
    command.shutdown();
  }
  if (!e->alive)
    return;
  if (e->config.kind != Kind::FS150) {
    if (!enabled)
      return;
    velocity_sub = input.subscribe<geometry_msgs::Twist>(
        topic("cmd_vel_topic", "/cmd_vel"), 1,
        [weak = entity, gen, this](const geometry_msgs::Twist::ConstPtr &m) {
          auto e = weak.lock();
          if (!e || !current(e, gen))
            return;
          auto t = ticket(e, Op::Velocity, gen);
          t->velocity = {m->linear.x, m->linear.y, m->angular.z};
          world.submit(t);
        });
    return;
  }
  if (enabled) {
    pva_sub = input.subscribe<mavros_msgs::PositionTarget>(
        topic("setpoint_topic", "/mavros/setpoint_raw/local"), 1,
        [weak = entity, gen,
         this](const mavros_msgs::PositionTarget::ConstPtr &m) {
          auto e = weak.lock();
          if (!e || !current(e, gen))
            return;
          const int64_t stamp = m->header.stamp.toNSec();
          if ((stamp && stamp < e->generation_stamp) ||
              stamp > world.metrics.sim_ns + 1000000000)
            return;
          auto t = ticket(e, Op::Pva, gen);
          t->at = stamp;
          auto &p = t->pva;
          p.coordinate_frame = m->coordinate_frame;
          p.type_mask = m->type_mask;
          p.position[0] = m->position.x;
          p.position[1] = m->position.y;
          p.position[2] = m->position.z;
          p.velocity[0] = m->velocity.x;
          p.velocity[1] = m->velocity.y;
          p.velocity[2] = m->velocity.z;
          p.acceleration[0] = m->acceleration_or_force.x;
          p.acceleration[1] = m->acceleration_or_force.y;
          p.acceleration[2] = m->acceleration_or_force.z;
          p.yaw = m->yaw;
          p.yaw_rate = m->yaw_rate;
          world.submit(t);
        });
    attitude_sub = input.subscribe<mavros_msgs::AttitudeTarget>(
        topic("attitude_topic", "/mavros/setpoint_raw/attitude"), 1,
        [weak = entity, gen,
         this](const mavros_msgs::AttitudeTarget::ConstPtr &m) {
          auto e = weak.lock();
          if (!e || !current(e, gen))
            return;
          const int64_t stamp = m->header.stamp.toNSec();
          if ((stamp && stamp < e->generation_stamp) ||
              stamp > world.metrics.sim_ns + 1000000000)
            return;
          auto t = ticket(e, Op::Attitude, gen);
          t->at = stamp;
          t->attitude.q = {m->orientation.w, m->orientation.x, m->orientation.y,
                           m->orientation.z};
          t->attitude.body_rate = {m->body_rate.x, m->body_rate.y,
                                   m->body_rate.z};
          t->attitude.thrust = m->thrust;
          t->attitude.type_mask = m->type_mask;
          world.submit(t);
        });
  }
  if (keep_services)
    return;
  boost::function<bool(mavros_msgs::CommandBool::Request &,
                       mavros_msgs::CommandBool::Response &)>
      arm_fn = [weak = entity, gen, this](auto &req, auto &res) {
        auto e = weak.lock();
        res.success = false;
        res.result = 4;
        if (!e || !current(e, gen))
          return true;
        auto t = ticket(e, Op::Arm, gen);
        t->arm = req.value;
        world.submit(t);
        if (World::wait(t)) {
          res.success = t->result.success;
          res.result =
              res.success ? 0 : t->result.reason == 1 ? 4 : t->result.reason;
        }
        return true;
      };
  arm = services.advertiseService(topic("arming_service", "/mavros/cmd/arming"),
                                  arm_fn);
  boost::function<bool(mavros_msgs::SetMode::Request &,
                       mavros_msgs::SetMode::Response &)>
      mode_fn = [weak = entity, gen, this](auto &req, auto &res) {
        auto e = weak.lock();
        res.mode_sent = false;
        if (!e || !current(e, gen) || req.base_mode ||
            req.custom_mode.empty() || req.custom_mode.size() >= 32 ||
            req.custom_mode.find('\0') != std::string::npos)
          return true;
        auto t = ticket(e, Op::Mode, gen);
        t->mode = req.custom_mode;
        world.submit(t);
        // MAVROS mode_sent is transmission, not Commander mode acceptance. The
        // response waits for execution; rejected mode is observable in state.
        if (World::wait(t))
          res.mode_sent =
              t->result.applied && t->result.enabled && t->result.reason != 1;
        return true;
      };
  mode = services.advertiseService(topic("mode_service", "/mavros/set_mode"),
                                   mode_fn);
  boost::function<bool(mavros_msgs::CommandLong::Request &,
                       mavros_msgs::CommandLong::Response &)>
      command_fn = [weak = entity, gen, this](auto &req, auto &res) {
        res.success = false;
        res.result = 3;
        if (req.command != 400)
          return true;
        if (req.param1 != 0 && req.param1 != 1) {
          res.result = 2;
          return true;
        }
        if (req.broadcast || req.confirmation ||
            (req.param2 != 0 && req.param2 != 21196) || req.param3 ||
            req.param4 || req.param5 || req.param6 || req.param7)
          return true;
        auto e = weak.lock();
        res.result = 4;
        if (!e || !current(e, gen))
          return true;
        auto t = ticket(e, Op::Arm, gen);
        t->arm = req.param1 == 1;
        t->action = req.param2 == 21196 ? 1 : 0;
        world.submit(t);
        if (World::wait(t)) {
          res.success = t->result.success;
          res.result =
              res.success ? 0 : t->result.reason == 1 ? 4 : t->result.reason;
        }
        return true;
      };
  command = services.advertiseService(
      topic("command_service", "/mavros/cmd/command"), command_fn);
}
void RosEntity::publish(const State &s, const TelemetryRates &rates) {
  auto e = entity.lock();
  if (!e || !e->alive || s.key.generation != e->generation)
    return;
  const bool reset = s.key.generation != last_generation || s.stamp < last_stamp;
  if (reset) schedule.reset();
  const bool fresh = reset || s.stamp != last_stamp;
  const bool status_changed = reset || last_enabled != s.enabled ||
      last_armed != s.armed || last_mode != s.mode.data();
  ros::Time stamp;
  stamp.fromNSec(s.stamp);
  const bool flight = e->config.kind == Kind::FS150;
  const uint8_t landed_state = s.landed ? (s.armed && s.velocity.z() > 0 ? 3 : 1)
      : (std::strcmp(s.mode.data(), "AUTO.LAND") == 0 ? 4 : 2);
  const bool state_due = flight && fresh && schedule.due(TelemetryGroup::State, s.stamp, rates);
  const bool extended_due = flight && fresh && schedule.due(TelemetryGroup::ExtendedState, s.stamp, rates);
  if (flight && (status_changed || state_due)) {
    auto &m = state_message;
    m.header.stamp = stamp;
    m.connected = s.enabled;
    m.armed = s.enabled && s.armed;
    m.guided = s.enabled;
    m.system_status = !s.enabled ? 0 : s.armed ? 4 : 3;
    m.mode = s.mode.data();
    state.publish(m);
  }
  if (flight && (status_changed || last_landed_state != landed_state || extended_due)) {
    auto &x = extended_message;
    x.header.stamp = stamp;
    x.landed_state = landed_state;
    extended.publish(x);
  }
  last_enabled = s.enabled;
  last_armed = s.armed;
  last_mode = s.mode.data();
  last_landed_state = landed_state;
  if (fresh) {
    last_stamp = s.stamp;
    last_generation = s.key.generation;
    if (s.enabled) {
      auto &p = localization_pose_message;
      p.header.stamp = stamp;
      p.pose.position.x = s.position.x();
      p.pose.position.y = s.position.y();
      p.pose.position.z = s.position.z();
      orientation(p.pose.orientation, s.orientation);
      auto &v = localization_twist_message;
      v.header.stamp = stamp;
      vector(v.twist.linear, s.velocity);
      vector(v.twist.angular, s.orientation * s.omega);
      if (schedule.due(TelemetryGroup::Localization, s.stamp, rates)) {
        double *axes[3] = {&p.pose.position.x, &p.pose.position.y,
                           &p.pose.position.z};
        for (int a = 0; a < 3; ++a)
          if (noise[a] > 0) *axes[a] += noise[a] * normal(rng);
        localization_pose.publish(p);
        localization_twist.publish(v);
      }
      if (flight && schedule.due(TelemetryGroup::Local, s.stamp, rates)) {
        auto &local = local_pose_message;
        local.header.stamp = stamp;
        local.pose.position.x = s.position.x() - e->config.local_origin.x();
        local.pose.position.y = s.position.y() - e->config.local_origin.y();
        local.pose.position.z = s.position.z() - e->config.local_origin.z();
        orientation(local.pose.orientation, s.orientation);
        pose.publish(local);
        local_twist_message.header.stamp = stamp;
        local_twist_message.twist = v.twist;
        velocity.publish(local_twist_message);
        auto &o = odometry_message;
        o.header.stamp = stamp;
        o.pose.pose = local.pose;
        vector(o.twist.twist.linear, s.orientation.conjugate() * s.velocity);
        vector(o.twist.twist.angular, s.omega);
        odom.publish(o);
      }
      {
        const bool imu_due = (flight || e->config.kind == Kind::Mecanum) &&
            schedule.due(TelemetryGroup::Imu, s.stamp, rates);
        const bool raw_due = (flight || e->config.kind == Kind::Scout) &&
            schedule.due(TelemetryGroup::RawImu, s.stamp, rates);
        if (imu_due || raw_due) {
          auto &m = imu_message;
          m.header.stamp = stamp;
          m.orientation_covariance[0] = 0.0;
          orientation(m.orientation, s.orientation);
          vector(m.angular_velocity, s.omega);
          vector(m.linear_acceleration, s.specific_force);
          if (imu_due) imu.publish(m);
          if (raw_due) {
            m.orientation = geometry_msgs::Quaternion{};
            m.orientation_covariance[0] = -1.0;
            raw_imu.publish(m);
          }
        }
      }
      if (flight && schedule.due(TelemetryGroup::Target, s.stamp, rates)) {
        auto &a = target_message;
        a.header.stamp = stamp;
        orientation(a.orientation, s.control.desired_orientation);
        vector(a.body_rate, s.control.desired_body_rate);
        a.thrust = s.control.normalized_thrust;
        a.type_mask = s.control.type_mask;
        target.publish(a);
      }
    }
  }
}
bool RosEntity::publish_sensor() {
  auto e=entity.lock();
  if(!e || !e->alive || !e->enabled || !e->sensor)return false;
  Sample sample;
  std::shared_ptr<const SensorPayload> payload;
  if(!Sensors::take_shared(e->sensor,sample,payload))return false;
  cloud_message.header.stamp.fromNSec(sample.stamp);
  cloud_message.width=payload->data.size()/cloud_message.point_step;
  cloud_message.row_step=cloud_message.width*cloud_message.point_step;
  if(beams){beam_message.header.stamp=cloud_message.header.stamp;beam_message.width=payload->beam_data.size()/beam_message.point_step;beam_message.row_step=payload->beam_data.size();}
  const PointCloudView cloud_view{cloud_message, payload->data};
  const PointCloudView beam_view{beam_message, payload->beam_data};
  if (!current(e, sample.key.generation)) return true;
  cloud.publish(cloud_view);
  if (beams) beams.publish(beam_view);
  return true;
}
} // namespace xsim
