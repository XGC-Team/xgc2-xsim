#include "core/control_input.hpp"
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <new>
#include <thread>

namespace {
std::atomic<size_t> allocations{0};
xsim::ControlSample sample(uint64_t n) {
  xsim::ControlSample s;
  s.op = n % 2 ? xsim::Op::Pva : xsim::Op::Attitude;
  s.key = {n, n};
  s.at = int64_t(n);
  s.arrival_ns = int64_t(n * 2);
  s.received_ns = int64_t(n * 3);
  s.velocity.setConstant(double(n));
  for (int i = 0; i < 3; ++i) s.pva.position[i] = double(n);
  s.attitude.thrust = double(n);
  return s;
}
void check(const xsim::ControlSample &s) {
  const auto n = s.key.id;
  assert(s.key.generation == n && s.at == int64_t(n));
  assert(s.arrival_ns == int64_t(n * 2) && s.received_ns == int64_t(n * 3));
  assert(s.op == (n % 2 ? xsim::Op::Pva : xsim::Op::Attitude));
  assert(s.velocity == Eigen::Vector3d::Constant(double(n)));
  for (int i = 0; i < 3; ++i) assert(s.pva.position[i] == double(n));
  assert(s.attitude.thrust == double(n));
}
}

void *operator new(std::size_t size) {
  ++allocations;
  if (void *p = std::malloc(size ? size : 1)) return p;
  throw std::bad_alloc();
}
void operator delete(void *p) noexcept { std::free(p); }
void operator delete(void *p, std::size_t) noexcept { std::free(p); }

int main() {
  std::array<xsim::ControlMailbox, 100> controls;
  xsim::ControlSample out;
  assert(!controls[0].take(out));
  assert(!controls[0].publish(sample(1)));
  assert(controls[0].publish(sample(2)));
  assert(controls[0].take(out) && out.key.id == 2);
  check(out);
  assert(!controls[0].take(out));

  // Both ordinary and 100-slot use stay allocation-free after cold setup.
  const auto before = allocations.load();
  for (size_t count : {size_t(20), size_t(100)}) {
    for (uint64_t n = 1; n <= 125; ++n) {
      for (size_t i = 0; i < count; ++i) {
        controls[i].publish(sample(n * 3));
        controls[i].publish(sample(n * 3 + 1));
        controls[i].publish(sample(n * 3 + 2));
      }
      for (size_t i = 0; i < count; ++i) {
        assert(controls[i].take(out));
        assert(out.key.id == n * 3 + 2);
        check(out);
      }
    }
  }
  assert(allocations.load() == before);

  // The producer never waits for the reader. Every observed payload must be
  // coherent even when intermediate samples are replaced during the copy.
  xsim::ControlMailbox concurrent;
  constexpr uint64_t total = 100000;
  std::atomic<bool> done{false};
  std::thread writer([&] {
    for (uint64_t n = 1; n <= total; ++n) concurrent.publish(sample(n));
    done.store(true, std::memory_order_release);
  });
  uint64_t last = 0;
  for (;;) {
    if (!concurrent.take(out)) {
      if (done.load(std::memory_order_acquire)) {
        // A final sample may have been published between take() and done.
        if (concurrent.take(out)) {
          check(out);
          assert(out.key.id > last);
          last = out.key.id;
        }
        break;
      }
      std::this_thread::yield();
      continue;
    }
    check(out);
    assert(out.key.id > last);
    last = out.key.id;
  }
  writer.join();
  assert(last == total);
  std::cout << "latest mailbox: coherent replacement, 20/100 zero-allocation, concurrent handoff PASS\n";
}
