#pragma once
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <nlohmann/json.hpp>
#include <ros/poll_manager.h>
#include <ros/poll_set.h>
#include <ros/publication.h>
#include <ros/publisher.h>
#include <ros/serialization.h>
#include <ros/topic_manager.h>
#include <stdexcept>
#include <utility>
#include <vector>

namespace xsim {

// Handoffs and estimated bytes, not acknowledgements of subscriber delivery.
struct PublicationMetrics {
  std::atomic<uint64_t> accepted{0}, bytes_estimate{0}, no_subscribers{0},
      buffer_drops{0}, serialize_ns{0}, serialize_ns_max{0}, queue_ns{0},
      queue_ns_max{0}, backpressure_ns{0}, buffer_allocations{0}, slot_capacity_bytes{0};

  nlohmann::json as_json() const {
    constexpr auto order = std::memory_order_relaxed;
    return {{"accepted", accepted.load(order)},
            {"bytes_estimate", bytes_estimate.load(order)},
            {"no_subscribers", no_subscribers.load(order)},
            {"buffer_drops", buffer_drops.load(order)},
            {"serialize_ns", serialize_ns.load(order)},
            {"serialize_ns_max", serialize_ns_max.load(order)},
            {"queue_ns", queue_ns.load(order)},
            {"queue_ns_max", queue_ns_max.load(order)},
            {"backpressure_ns", backpressure_ns.load(order)},
            {"buffer_allocations", buffer_allocations.load(order)},
            {"slot_capacity_bytes", slot_capacity_bytes.load(order)}};
  }
};

// One stable shard owns bind()/publish() and each slot's sole producer reference.
// This is a Noetic internal-API adapter for nonlatched typed advertisements.
// Stop its producer before Publisher/NodeHandle shutdown. The cached Publication
// keeps an old, dropped advertisement distinct from a new one with the same name.
class BufferedPublisher {
public:
  static constexpr size_t buffer_count = 4;

  BufferedPublisher() = default;
  ~BufferedPublisher() {
    if (metrics_) for (const auto &slot : slots_)
      metrics_->slot_capacity_bytes.fetch_sub(slot.capacity, std::memory_order_relaxed);
  }
  BufferedPublisher(const BufferedPublisher &) = delete;
  BufferedPublisher &operator=(const BufferedPublisher &) = delete;
  BufferedPublisher(BufferedPublisher &&) = delete;
  BufferedPublisher &operator=(BufferedPublisher &&) = delete;

  void bind(ros::Publisher publisher,
            const std::shared_ptr<PublicationMetrics> &metrics) {
    if (publication_) throw std::logic_error("buffered publication is already bound");
    if (!publisher)
      throw std::invalid_argument("buffered publication requires a valid publisher");
    auto publication = ros::TopicManager::instance()->lookupPublication(publisher.getTopic());
    if (!publication)
      throw std::invalid_argument("buffered publication is not advertised");
    if (publisher.isLatched() || publication->isLatching())
      throw std::invalid_argument("buffered publication does not support latched topics");
    publisher_ = std::move(publisher);
    publication_ = std::move(publication);
    metrics_ = metrics ? metrics : std::make_shared<PublicationMetrics>();
    slots_.resize(buffer_count);
    cursor_ = 0;
  }

  static void flush_poll_set() {
    ros::PollManager::instance()->getPollSet().signal();
  }

  explicit operator bool() const { return bool(publication_); }
  template <typename M> bool publish(const M &message, bool signal_poll = true) {
    if (!publication_) return false;
    const auto subscribers = publication_->getNumSubscribers();
    if (!subscribers) {
      publication_->incrementSequence();
      metrics_->no_subscribers.fetch_add(1, std::memory_order_relaxed);
      return false;
    }

    Slot *available = nullptr;
    size_t index = cursor_;
    for (size_t offset = 0; offset < slots_.size(); ++offset) {
      auto &slot = slots_[index];
      index = index + 1 == slots_.size() ? 0 : index + 1;
      if (!slot.buffer || slot.buffer.unique()) {
        available = &slot;
        cursor_ = index;
        break;
      }
    }
    if (!available && slots_.size() < size_t(subscribers) + buffer_count) {
      // Each TCP connection can retain a different in-flight message. Add
      // room only on exhaustion; ROS holds bytes, never addresses of Slots.
      const size_t previous_size = slots_.size();
      slots_.resize(size_t(subscribers) + buffer_count);
      available = &slots_[previous_size];
      cursor_ = previous_size + 1 == slots_.size() ? 0 : previous_size + 1;
    }
    if (!available) {
      const auto started = Clock::now();
      uint64_t oldest_buffer_ns = 0;
      for (const auto &slot : slots_)
        oldest_buffer_ns = std::max(oldest_buffer_ns,
                                    elapsed(slot.submitted, started));
      metrics_->buffer_drops.fetch_add(1, std::memory_order_relaxed);
      // Sum of observed ages at rejected handoffs, not time spent blocking.
      metrics_->backpressure_ns.fetch_add(oldest_buffer_ns,
                                         std::memory_order_relaxed);
      return false;
    }

    const auto serialization_started = Clock::now();
    const uint32_t payload = ros::serialization::serializationLength(message);
    if (payload > std::numeric_limits<uint32_t>::max() - 4)
      throw std::length_error("buffered ROS message exceeds the wire length limit");
    const uint32_t length = payload + 4;
    if (available->capacity < length) {
      const uint64_t grown = uint64_t(available->capacity) * 2;
      const uint64_t capacity = std::max<uint64_t>(length, grown);
      const auto new_capacity = uint32_t(std::min<uint64_t>(
          capacity, std::numeric_limits<uint32_t>::max()));
      available->buffer.reset(new uint8_t[new_capacity]);
      metrics_->buffer_allocations.fetch_add(1, std::memory_order_relaxed);
      metrics_->slot_capacity_bytes.fetch_add(new_capacity - available->capacity,
                                            std::memory_order_relaxed);
      available->capacity = new_capacity;
    }
    ros::serialization::OStream stream(available->buffer.get(), length);
    ros::serialization::serialize(stream, payload);
    uint8_t *const message_start = stream.getData();
    ros::serialization::serialize(stream, message);
    const auto serialized = Clock::now();
    const uint64_t serialization_time = elapsed(serialization_started, serialized);
    metrics_->serialize_ns.fetch_add(serialization_time, std::memory_order_relaxed);
    update_max(metrics_->serialize_ns_max, serialization_time);

    ros::SerializedMessage prepared;
    prepared.buf = available->buffer;
    prepared.num_bytes = length;
    prepared.message_start = message_start; // Skip the four-byte length prefix.
    // message/type_info stay empty: both remote and intraprocess subscribers
    // consume the serialized representation; Header.seq is rewritten by ROS.
    available->submitted = serialized;
    publication_->publish(prepared);
    if (signal_poll) ros::PollManager::instance()->getPollSet().signal();
    const uint64_t queue_time = elapsed(serialized, Clock::now());
    metrics_->queue_ns.fetch_add(queue_time, std::memory_order_relaxed);
    update_max(metrics_->queue_ns_max, queue_time);
    metrics_->accepted.fetch_add(1, std::memory_order_relaxed);
    metrics_->bytes_estimate.fetch_add(uint64_t(length) * subscribers,
                                       std::memory_order_relaxed);
    return true;
  }

private:
  using Clock = std::chrono::steady_clock;
  struct Slot {
    boost::shared_array<uint8_t> buffer;
    uint32_t capacity = 0;
    Clock::time_point submitted{};
  };
  static uint64_t elapsed(Clock::time_point from, Clock::time_point to) {
    return uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(to - from).count());
  }
  static void update_max(std::atomic<uint64_t> &counter, uint64_t value) {
    auto observed = counter.load(std::memory_order_relaxed);
    while (value > observed && !counter.compare_exchange_weak(
               observed, value, std::memory_order_relaxed)) {}
  }

  // Reverse destruction order releases ring/cache before the strong Publisher.
  ros::Publisher publisher_;
  ros::PublicationPtr publication_;
  std::shared_ptr<PublicationMetrics> metrics_;
  std::vector<Slot> slots_;
  size_t cursor_ = 0;
};
} // namespace xsim
