#include "runtime.hpp"
#include "entity.hpp"
#include <rosgraph_msgs/Clock.h>
#include <condition_variable>
#include <pthread.h>
#include <thread>
namespace xsim {
namespace {
struct RosRuntime {
  struct Lane {
    std::mutex mutex;
    std::condition_variable wake;
    std::shared_ptr<const Frame> next;
    std::shared_ptr<const TelemetryRates> rates;
    std::thread thread;
  };
  ros::CallbackQueue inputs, services;
  ros::NodeHandle node;
  BufferedPublisher clock;
  ros::Publisher reference;
  sensor_msgs::PointCloud2 reference_message;
  bool reference_sent = false;
  int64_t clock_stamp = -1;
  std::shared_ptr<PublicationMetrics> telemetry_metrics = std::make_shared<PublicationMetrics>();
  std::shared_ptr<PublicationMetrics> cloud_metrics = std::make_shared<PublicationMetrics>();
  std::shared_ptr<PublicationMetrics> clock_metrics = std::make_shared<PublicationMetrics>();
  std::vector<std::unique_ptr<Lane>> lanes;
  std::atomic<bool> running{false};
  std::atomic<uint64_t> frames{0}, coalesced{0}, prepare_ns{0}, prepare_ns_max{0}, errors{0};
  std::mutex error_mutex;
  std::string error;

  RosRuntime(const Json &config, Sensors &sensors) {
    const auto workers = config.value("publish_workers", Json(2));
    if (!workers.is_number_integer() || workers < 1 || workers > 8)
      throw std::invalid_argument("publish_workers must be an integer in 1..8");
    for (unsigned i = 0; i < workers.get<unsigned>(); ++i)
      lanes.emplace_back(std::make_unique<Lane>());
    const auto topic = config.value("reference_cloud_topic", std::string{});
    if (!topic.empty()) {
      reference = node.advertise<sensor_msgs::PointCloud2>(topic, 1, true);
      reference_message.height=1;reference_message.point_step=12;reference_message.is_dense=true;reference_message.header.frame_id="world";
      for(unsigned a=0;a<3;++a){sensor_msgs::PointField f;f.name=std::string(1,"xyz"[a]);f.offset=a*4;f.datatype=sensor_msgs::PointField::FLOAT32;f.count=1;reference_message.fields.push_back(f);}
      reference_message.data=sensors.reference_cloud();reference_message.width=reference_message.data.size()/12;reference_message.row_step=reference_message.data.size();
    }
    if (config.value("publish_clock", false))
      clock.bind(node.advertise<rosgraph_msgs::Clock>("/clock", 1), clock_metrics);
  }
  ~RosRuntime() { stop(); }
  void start() {
    if (running.exchange(true)) return;
    try {
      for (size_t i = 0; i < lanes.size(); ++i)
        lanes[i]->thread = std::thread([this, i] { work(i); });
    } catch (...) { stop(); throw; }
  }
  void stop() {
    running = false;
    for (const auto &lane : lanes) lane->wake.notify_all();
    for (const auto &lane : lanes) {
      if (lane->thread.joinable()) lane->thread.join();
      lane->next.reset();
      lane->rates.reset();
    }
  }
  void submit(const std::shared_ptr<const Frame> &frame,
              const std::shared_ptr<const TelemetryRates> &rates) {
    ++frames;
    for (const auto &lane : lanes) {
      {
        std::lock_guard<std::mutex> lock(lane->mutex);
        if (lane->next) ++coalesced;
        lane->next = frame;
        lane->rates = rates;
      }
      lane->wake.notify_one();
    }
  }
  void work(size_t index) {
    pthread_setname_np(pthread_self(), "xsim-publish");
    auto &lane = *lanes[index];
    std::shared_ptr<const Frame> frame;
    std::shared_ptr<const TelemetryRates> rates;
    size_t cursor = 0;
    bool busy = false;
    while (running) {
      bool fresh = false;
      {
        std::unique_lock<std::mutex> lock(lane.mutex);
        if (!busy) lane.wake.wait_for(lock, std::chrono::milliseconds(1),
                                     [&] { return !running || bool(lane.next); });
        if (!running) break;
        if (lane.next) {
          frame = std::move(lane.next);
          rates = lane.rates;
          fresh = true;
        }
      }
      busy = false;
      if (!frame) continue;
      try {
        if (fresh) {
          const auto started = Clock::now();
          for (const auto &state : frame->states)
            if (state.key.id % lanes.size() == index)
              if (auto e = state.entity.lock(); e && e->alive && e->io && e->io->publish)
                e->io->publish(state, *rates);
          const auto ns = uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - started).count());
          prepare_ns += ns;
          auto previous = prepare_ns_max.load();
          while (ns > previous && !prepare_ns_max.compare_exchange_weak(previous, ns)) {}
        }
        for (size_t n = 0; n < frame->states.size(); ++n) {
          const size_t at = (cursor + n) % frame->states.size();
          const auto &state = frame->states[at];
          if (state.key.id % lanes.size() != index) continue;
          if (auto e = state.entity.lock(); e && e->alive && e->io && e->io->publish_sensor && e->io->publish_sensor()) {
            cursor = (at + 1) % frame->states.size();
            busy = true;
            break;
          }
        }
      } catch (const std::exception &failure) {
        ++errors;
        std::lock_guard<std::mutex> lock(error_mutex);
        if (error.empty()) error = failure.what();
      }
    }
  }
  Json status() {
    std::lock_guard<std::mutex> lock(error_mutex);
    return {{"workers", lanes.size()}, {"initial_buffer_slots_per_topic", BufferedPublisher::buffer_count},
            {"submitted_frames", frames.load()}, {"coalesced_worker_frames", coalesced.load()},
            {"prepare_ns", prepare_ns.load()}, {"prepare_ns_max", prepare_ns_max.load()},
            {"errors", errors.load()}, {"error", error},
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
    auto ros = std::make_shared<RosEntity>(entity, world, runtime->inputs, runtime->services, settings, runtime,
                                         runtime->telemetry_metrics, runtime->cloud_metrics);
    entity->io = std::make_shared<EntityIO>(EntityIO{
      [ros] { ros->reconcile(); },
      [ros](const State &state, const TelemetryRates &rates) { ros->publish(state, rates); },
      [ros] { return ros->publish_sensor(); },
      ros->localization_pose_topic,
    });
  };
  io.poll_inputs = [runtime] { runtime->inputs.callAvailable(ros::WallDuration(0)); };
  io.poll_services = [runtime] { runtime->services.callOne(ros::WallDuration(.01)); };
  io.okay = [] { return ros::ok(); };
  io.publish_frame = [runtime](const Frame &frame) { runtime->publish(frame); };
  io.start_outputs = [runtime] { runtime->start(); };
  io.stop_outputs = [runtime] { runtime->stop(); };
  io.publish_entities = [runtime](const std::shared_ptr<const Frame> &frame,
                                  const std::shared_ptr<const TelemetryRates> &rates) { runtime->submit(frame, rates); };
  io.publication_status = [runtime] { return runtime->status(); };
  io.facts = [] { return Json{{"ros_master_uri", ros::master::getURI()}}; };
  return io;
}
} // namespace xsim
