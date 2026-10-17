#include "runtime.hpp"
#include "entity.hpp"
#include <rosgraph_msgs/Clock.h>
#include <xgc2_geometry_msgs/SceneSnapshot.h>
#include <xgc2_geometry_msgs/SceneState.h>
#include <boost/bind.hpp>
#include <algorithm>
#include <cstdint>
#include <condition_variable>
#include <limits>
#include <cerrno>
#include <iostream>
#include <poll.h>
#include <pthread.h>
#include <random>
#include <sys/eventfd.h>
#include <thread>
#include <unistd.h>
#include <xmlrpcpp/XmlRpcClient.h>
#include <xmlrpcpp/XmlRpcUtil.h>
namespace xsim {
namespace {
thread_local bool connection_worker = false;
// xmlrpcpp otherwise prints two errors per refused discovery connection. The
// worker reports those failures with its outage summary; other callers keep
// the existing error handler. This handler lives until process shutdown.
struct RosConnectionErrors : XmlRpc::XmlRpcErrorHandler {
  XmlRpc::XmlRpcErrorHandler *previous = getErrorHandler();
  RosConnectionErrors() { setErrorHandler(this); }
  void error(const char *message) override {
    if (!connection_worker && previous) previous->error(message);
  }
};
struct ServiceQueue : ros::CallbackQueue {
  std::mutex wake_mutex;
  std::function<void()> wake;
  void set_wake(std::function<void()> fn) {
    std::lock_guard<std::mutex> lock(wake_mutex);
    wake = std::move(fn);
  }
  void addCallback(const ros::CallbackInterfacePtr &callback, uint64_t removal_id = 0) override {
    ros::CallbackQueue::addCallback(callback, removal_id);
    std::lock_guard<std::mutex> lock(wake_mutex);
    if (wake) wake();
  }
};
struct RosRuntime {
  struct Lane {
    std::mutex mutex;
    std::condition_variable wake;
    std::shared_ptr<const Frame> next;
    std::shared_ptr<const TelemetryRates> rates;
    std::thread thread;
  };
  ros::CallbackQueue inputs;
  ServiceQueue services;
  std::unique_ptr<ros::NodeHandle> node;
  const Json config;
  int wake_fd = -1;
  std::thread connection;
  std::atomic<bool> watching{false}, peer_ready{false}, failed{false};
  std::atomic<uint64_t> attempts{0}, replays{0}, entity_bindings{0};
  std::atomic<int64_t> latest_stamp{-1};
  std::mutex connection_mutex;
  std::vector<std::weak_ptr<RosEntity>> entities;
  std::string peer_state = "waiting_for_master", peer_reason = "ROS master is unavailable", master_run_id;
  int master_pid = 0;
  BufferedPublisher clock;
  ros::Publisher reference;
  sensor_msgs::PointCloud2 reference_message;
  bool reference_sent = false;
  int64_t clock_stamp = -1;
  std::shared_ptr<PublicationMetrics> telemetry_metrics = std::make_shared<PublicationMetrics>();
  std::shared_ptr<PublicationMetrics> cloud_metrics = std::make_shared<PublicationMetrics>();
  std::shared_ptr<PublicationMetrics> clock_metrics = std::make_shared<PublicationMetrics>();
  Lane telemetry;
  std::thread cloud;
  std::atomic<bool> running{false};
  std::atomic<uint64_t> frames{0}, coalesced{0}, prepare_ns{0}, prepare_ns_max{0}, errors{0};
  std::mutex error_mutex;
  std::string error;
  bool has_phase_t0 = false;
  int64_t phase_t0_ns = 0;
  Sensors *scene_sensors = nullptr;
  bool scene_inputs = false;
  ros::Subscriber scene_snapshot, scene_state;

  void on_snapshot(const xgc2_geometry_msgs::SceneSnapshot::ConstPtr &message) {
    if (!scene_sensors) return;
    std::vector<std::string> ids;
    ids.reserve(message->obstacles.size());
    for (const auto &obstacle : message->obstacles)
      if (obstacle.dynamic) ids.push_back(obstacle.id);
    scene_sensors->bind_dynamic_scene(message->epoch, message->revision, message->header.frame_id, ids);
  }
  void on_state(const xgc2_geometry_msgs::SceneState::ConstPtr &message) {
    if (!scene_sensors) return;
    std::vector<SceneObstaclePose> poses;
    poses.reserve(message->obstacles.size());
    for (const auto &obstacle : message->obstacles) {
      SceneObstaclePose pose;
      pose.id = obstacle.id;
      pose.pose.position = {obstacle.pose.position.x, obstacle.pose.position.y, obstacle.pose.position.z};
      pose.pose.orientation = Eigen::Quaterniond(obstacle.pose.orientation.w, obstacle.pose.orientation.x,
                                                 obstacle.pose.orientation.y, obstacle.pose.orientation.z);
      poses.push_back(std::move(pose));
    }
    scene_sensors->update_scene_state(message->header.frame_id, message->epoch, message->revision, poses);
  }
  void subscribe_scene() {
    // snapshot is the latched definition. state is SceneNode's existing 30 Hz
    // stream and is not latched; queue 1 retains the latest pose for this callback.
    if (!scene_inputs || !node || scene_snapshot) return;
    ros::SubscribeOptions snapshot;
    snapshot.template init<xgc2_geometry_msgs::SceneSnapshot>(
        "/xgc_scene/snapshot", 1, boost::bind(&RosRuntime::on_snapshot, this, boost::placeholders::_1));
    snapshot.callback_queue = &inputs;
    scene_snapshot = node->subscribe(snapshot);
    ros::SubscribeOptions state;
    state.template init<xgc2_geometry_msgs::SceneState>(
        "/xgc_scene/state", 1, boost::bind(&RosRuntime::on_state, this, boost::placeholders::_1));
    state.callback_queue = &inputs;
    scene_state = node->subscribe(state);
  }

  RosRuntime(const Json &settings, Sensors &sensors) : config(settings) {
    scene_sensors = &sensors;
    scene_inputs = sensors.has_dynamic_obstacles();
    if (config.contains("phase_t0_ns")) {
      const auto &value = config.at("phase_t0_ns");
      if (!value.is_number_integer() ||
          (value.is_number_unsigned() && value.get<uint64_t>() > uint64_t(INT64_MAX)))
        throw std::invalid_argument("phase T0 must be a nonnegative int64 nanosecond count");
      phase_t0_ns = value.get<int64_t>();
      if (phase_t0_ns < 0) throw std::invalid_argument("phase T0 must be nonnegative");
      has_phase_t0 = true;
    }
    const auto topic = config.value("reference_cloud_topic", std::string{});
    if (!topic.empty()) {
      reference_message.height=1;reference_message.point_step=12;reference_message.is_dense=true;reference_message.header.frame_id="world";
      for(unsigned a=0;a<3;++a){sensor_msgs::PointField f;f.name=std::string(1,"xyz"[a]);f.offset=a*4;f.datatype=sensor_msgs::PointField::FLOAT32;f.count=1;reference_message.fields.push_back(f);}
      reference_message.data=sensors.reference_cloud();reference_message.width=reference_message.data.size()/12;reference_message.row_step=reference_message.data.size();
    }
    wake_fd = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
    if (wake_fd < 0) throw std::runtime_error("cannot create ROS connection wakeup");
  }
  ~RosRuntime() { stop(); close(wake_fd); }
  void start() {
    watching = true;
    connection = std::thread([this] {
      try { connect(); }
      catch (const std::exception &failure) {
        set_peer("failed", failure.what());
        failed = true;
        std::cerr << "xsim: ROS binding failed: " << failure.what() << '\n';
      }
    });
  }
  void start_lanes() {
    if (running.exchange(true)) return;
    try {
      scene_sensors->start_completions();
      telemetry.thread = std::thread([this] { work_telemetry(); });
      cloud = std::thread([this] { work_cloud(); });
    } catch (...) {
      stop_lanes();
      throw;
    }
  }
  void stop() {
    watching = false;
    wake_connection();
    if (connection.joinable()) connection.join();
    stop_lanes();
    telemetry.next.reset();
    telemetry.rates.reset();
  }
  void stop_lanes() {
    {
      std::lock_guard<std::mutex> lock(telemetry.mutex);
      // Entity IO can retain this stopped runtime past the Sensors owner.
      if (!running && !telemetry.thread.joinable() && !cloud.joinable()) return;
      running = false;
    }
    telemetry.wake.notify_all();
    scene_sensors->interrupt_completions();
    if (telemetry.thread.joinable()) telemetry.thread.join();
    if (cloud.joinable()) cloud.join();
  }
  void wake_connection() {
    const uint64_t one = 1;
    const auto written = write(wake_fd, &one, sizeof(one));
    (void)written; // A full event counter already guarantees a wakeup.
  }
  void attach(const std::shared_ptr<RosEntity> &entity) {
    {
      std::lock_guard<std::mutex> lock(connection_mutex);
      entities.erase(std::remove_if(entities.begin(), entities.end(),
          [](const auto &weak) { return weak.expired(); }), entities.end());
      entities.emplace_back(entity);
    }
    wake_connection();
  }
  void set_peer(const std::string &state, const std::string &reason) {
    std::lock_guard<std::mutex> lock(connection_mutex);
    peer_state = state;
    peer_reason = reason;
    peer_ready.store(state == "ready", std::memory_order_release);
  }
  Json ros_status() {
    std::lock_guard<std::mutex> lock(connection_mutex);
    return {{"ready", peer_ready.load()}, {"state", peer_state}, {"reason", peer_reason},
            {"master_pid", master_pid}, {"master_run_id", master_run_id},
            {"connect_attempts", attempts.load()}, {"desired_revision", 1},
            {"applied_revision", replays.load()}, {"state_replays", replays.load()},
            {"entity_bindings", entity_bindings.load()}};
  }
  // rosmaster has no readiness event. Use one interruptible, bounded XMLRPC
  // flight on this worker; topic discovery stays with roscpp publisher updates.
  bool master_call(const char *method, const XmlRpc::XmlRpcValue &args, XmlRpc::XmlRpcValue &response) {
    XmlRpc::XmlRpcClient client(ros::master::getHost().c_str(), ros::master::getPort(), "/");
    if (!client.executeNonBlock(method, args)) return false;
    const auto until = Clock::now() + std::chrono::seconds(1);
    while (watching && Clock::now() < until) {
      client._disp.work(.05);
      if (client.executeCheckDone(response))
        return !client.isFault() && response.getType() == XmlRpc::XmlRpcValue::TypeArray && response.size() == 3;
    }
    return false;
  }
  void reconcile_entities() {
    std::vector<std::shared_ptr<RosEntity>> pending;
    {
      std::lock_guard<std::mutex> lock(connection_mutex);
      for (const auto &weak : entities) if (auto entity = weak.lock()) pending.emplace_back(std::move(entity));
    }
    for (const auto &entity : pending) {
      if (!watching) return;
      const auto revision = entity->desired_revision.load(std::memory_order_acquire);
      if (revision == entity->applied_revision.load()) continue;
      if (!entity->bound) {
        entity->bind();
        if (!entity->bound) continue;
        ++entity_bindings;
      } else entity->reconcile();
      if (auto current = entity->entity.lock(); current && current->sensor)
        scene_sensors->enable_completions(current->sensor);
      entity->applied_revision.store(revision, std::memory_order_release);
    }
  }
  void connect() {
    pthread_setname_np(pthread_self(), "xsim-ros-peer");
    static RosConnectionErrors errors;
    connection_worker = true;
    // roscpp's internal fixed-interval retries must not become an outage loop.
    // Graph changes are serialized here after discovery, outside all hot paths.
    ros::master::setRetryTimeout(ros::WallDuration(.001));
    std::mt19937 random(std::random_device{}());
    int backoff_ms = 250;
    auto next_probe = Clock::now();
    auto next_log = Clock::time_point::min();
    uint64_t suppressed = 0;
    while (watching) {
      if (!scene_inputs && !master_pid && !config.value("publish_clock", false) &&
          config.value("reference_cloud_topic", std::string{}).empty()) {
        bool needed;
        {
          std::lock_guard<std::mutex> lock(connection_mutex);
          needed = std::any_of(entities.begin(), entities.end(), [](const auto &weak) { return !weak.expired(); });
        }
        if (!needed) {
          set_peer("unused", "No ROS input or publication requested");
          pollfd wake{wake_fd, POLLIN, 0};
          const int result = poll(&wake, 1, -1);
          if (result < 0 && errno != EINTR) throw std::runtime_error("ROS connection wakeup failed");
          if (result > 0) { uint64_t count; const auto read_count = read(wake_fd, &count, sizeof(count)); (void)read_count; }
          continue;
        }
      }
      if (Clock::now() >= next_probe) {
        ++attempts;
        XmlRpc::XmlRpcValue args, response;
        args[0] = ros::this_node::getName();
        bool available = master_call("getPid", args, response) &&
            response[0].getType() == XmlRpc::XmlRpcValue::TypeInt && int(response[0]) == 1 &&
            response[2].getType() == XmlRpc::XmlRpcValue::TypeInt;
        const int pid = available ? int(response[2]) : 0;
        std::string run_id;
        if (available) {
          args[1] = "/run_id";
          available = master_call("getParam", args, response);
          if (available && response[0].getType() == XmlRpc::XmlRpcValue::TypeInt && int(response[0]) == 1 &&
              response[2].getType() == XmlRpc::XmlRpcValue::TypeString)
            run_id = std::string(response[2]);
        }
        if (!watching) break;
        if (!available) {
          set_peer(node ? "master_unavailable" : "waiting_for_master", "ROS master is unavailable");
          if (running) stop_lanes();
          if (Clock::now() >= next_log) {
            std::cerr << "xsim: ROS master unavailable; suppressed " << suppressed << " attempts\n";
            suppressed = 0;
            next_log = Clock::now() + std::chrono::seconds(10);
          } else ++suppressed;
          next_probe = Clock::now() + std::chrono::milliseconds(
              std::uniform_int_distribution<int>(0, backoff_ms)(random));
          backoff_ms = std::min(backoff_ms * 2, 30000);
        } else {
          bool replaced;
          {
            std::lock_guard<std::mutex> lock(connection_mutex);
            replaced = master_pid && (master_pid != pid || (!master_run_id.empty() && master_run_id != run_id));
            if (!replaced) { master_pid = pid; master_run_id = run_id; }
          }
          if (replaced) {
            set_peer("master_replaced", "ROS master identity changed; ending this Run binding");
            failed = true;
            std::cerr << "xsim: ROS master identity changed; ending this Run binding\n";
            return;
          }
          if (!node) {
            if (config.value("use_sim_time", true))
              ros::param::set("/use_sim_time", true);
            node = std::make_unique<ros::NodeHandle>();
            const auto topic = config.value("reference_cloud_topic", std::string{});
            if (!topic.empty()) reference = node->advertise<sensor_msgs::PointCloud2>(topic, 1, true);
            if (config.value("publish_clock", false))
              clock.bind(node->advertise<rosgraph_msgs::Clock>("/clock", 1), clock_metrics);
            subscribe_scene();
            ++replays;
            reconcile_entities();
            // A paused world may have delivered its only frame before ROS bound.
            Frame snapshot;
            snapshot.stamp = latest_stamp.load();
            if (snapshot.stamp >= 0) publish(snapshot);
          }
          reconcile_entities();
          set_peer("ready", "");
          start_lanes();
          backoff_ms = 250;
          next_log = Clock::time_point::min();
          suppressed = 0;
          // Only master identity needs a periodic check; never poll topic peers.
          next_probe = Clock::now() + std::chrono::seconds(10);
        }
      }
      if (peer_ready) reconcile_entities();
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(next_probe - Clock::now()).count();
      pollfd wake{wake_fd, POLLIN, 0};
      const int result = poll(&wake, 1, int(std::max<int64_t>(1, remaining + 1)));
      if (result < 0 && errno != EINTR) throw std::runtime_error("ROS connection wakeup failed");
      if (result > 0) { uint64_t count; const auto read_count = read(wake_fd, &count, sizeof(count)); (void)read_count; }
    }
  }
  void submit(const std::shared_ptr<const Frame> &frame,
              const std::shared_ptr<const TelemetryRates> &rates) {
    ++frames;
    {
      std::lock_guard<std::mutex> lock(telemetry.mutex);
      if (telemetry.next) ++coalesced;
      telemetry.next = frame;
      telemetry.rates = rates;
    }
    telemetry.wake.notify_one();
  }
  void publication_error(const std::exception &failure) {
    ++errors;
    std::lock_guard<std::mutex> lock(error_mutex);
    if (error.empty()) error = failure.what();
  }
  void work_telemetry() {
    pthread_setname_np(pthread_self(), "xsim-telemetry");
    std::shared_ptr<const Frame> frame;
    std::shared_ptr<const TelemetryRates> rates;
    while (running) {
      {
        std::unique_lock<std::mutex> lock(telemetry.mutex);
        telemetry.wake.wait(lock, [&] { return !running || bool(telemetry.next); });
        if (!running) break;
        frame = std::move(telemetry.next);
        rates = std::move(telemetry.rates);
      }
      try {
        const auto started = Clock::now();
        // This captured frame owns every state; no scan or roster polling.
        for (const auto &state : frame->states)
          if (auto e = state.entity.lock(); e && e->alive && e->io && e->io->publish)
            e->io->publish(state, *rates);
        const auto ns = uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - started).count());
        prepare_ns += ns;
        auto previous = prepare_ns_max.load();
        while (ns > previous && !prepare_ns_max.compare_exchange_weak(previous, ns)) {}
      } catch (const std::exception &failure) { publication_error(failure); }
      frame.reset(); // An idle telemetry worker must not retain an old roster.
      rates.reset();
    }
  }
  void work_cloud() {
    pthread_setname_np(pthread_self(), "xsim-cloud");
    while (auto sensor = scene_sensors->wait_completion()) {
      // Always take/clear the token, including removed or invalid entities.
      // No queue mutex is held during this sensor lock or ROS serialization.
      Sample sample;
      std::shared_ptr<const SensorPayload> payload;
      if (!Sensors::take_completion(sensor, sample, payload)) continue;
      try {
        if (auto e = sensor->entity.lock(); e && e->io && e->io->publish_sensor)
          e->io->publish_sensor(sample, payload);
      } catch (const std::exception &failure) { publication_error(failure); }
    }
  }
  Json status() {
    std::lock_guard<std::mutex> lock(error_mutex);
    return {{"workers", 2}, {"telemetry_workers", 1}, {"cloud_workers", 1}, {"initial_buffer_slots_per_topic", BufferedPublisher::buffer_count},
            {"submitted_frames", frames.load()}, {"coalesced_worker_frames", coalesced.load()},
            {"prepare_ns", prepare_ns.load()}, {"prepare_ns_max", prepare_ns_max.load()},
            {"errors", errors.load()}, {"error", error},
            {"ros", ros_status()},
            {"telemetry", telemetry_metrics->as_json()}, {"cloud", cloud_metrics->as_json()},
            {"clock", clock_metrics->as_json()}};
  }
  void publish(const Frame &frame) {
    if (clock && frame.stamp != clock_stamp) {
      rosgraph_msgs::Clock c;
      c.clock.fromNSec(frame.stamp);
      clock.publish(c);
      clock_stamp = frame.stamp;
    }
    if(reference && !reference_sent){reference_message.header.stamp.fromNSec(frame.stamp);reference.publish(reference_message);reference_sent=true;}
  }
};
} // namespace
RuntimeIO make_ros_io(const Json &config, World &world, Sensors &sensors) {
  auto runtime = std::make_shared<RosRuntime>(config, sensors);
  RuntimeIO io;
  io.attach_entity = [runtime, &world](const std::shared_ptr<Entity> &entity, const Json &settings) {
    const int64_t *phase = runtime->has_phase_t0 ? &runtime->phase_t0_ns : nullptr;
    auto ros = std::make_shared<RosEntity>(entity, world, runtime->inputs, runtime->services, settings, runtime,
                                         runtime->telemetry_metrics, runtime->cloud_metrics, phase);
    entity->io = std::make_shared<EntityIO>(EntityIO{
      [ros, runtime] { if (ros->request_reconcile()) runtime->wake_connection(); },
      [ros, runtime](const State &state, const TelemetryRates &rates) { if (runtime->peer_ready) ros->publish(state, rates); },
      [ros](const Sample &sample, const std::shared_ptr<const SensorPayload> &payload) {
        // The cloud owner starts after binding and is joined on an outage.
        // A claimed completion finishes on its immutable publication even if
        // master discovery changes after take_completion cleared its token.
        ros->publish_sensor(sample, payload);
      },
      ros->localization_pose_topic,
    });
    runtime->attach(ros);
  };
  io.poll_inputs = [runtime] { runtime->inputs.callAvailable(ros::WallDuration(0)); };
  io.bind_service_wake = [runtime](std::function<void()> wake) {
    runtime->services.set_wake(std::move(wake));
  };
  io.poll_services = [runtime] {
    return runtime->services.callOne(ros::WallDuration(0)) == ros::CallbackQueue::Called;
  };
  io.inputs_ready = [runtime] { return runtime->peer_ready.load(); };
  io.okay = [runtime] { return !runtime->failed && ros::ok(); };
  io.publish_frame = [runtime](const Frame &frame) {
    runtime->latest_stamp = frame.stamp;
    if (runtime->peer_ready) runtime->publish(frame);
  };
  io.start_outputs = [runtime] { runtime->start(); };
  io.stop_outputs = [runtime] { runtime->stop(); };
  io.publish_entities = [runtime](const std::shared_ptr<const Frame> &frame,
                                  const std::shared_ptr<const TelemetryRates> &rates) { runtime->submit(frame, rates); };
  io.publication_status = [runtime] { return runtime->status(); };
  io.facts = [runtime] { return Json{{"ros_master_uri", ros::master::getURI()}, {"ros", runtime->ros_status()}}; };
  return io;
}
} // namespace xsim
