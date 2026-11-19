#include "io/receipt_clock.hpp"

#include <cassert>
#include <cstring>
#include <sys/socket.h>
#include <sys/un.h>

int main() {
  char directory[] = "/tmp/xsim-receipt-clock-XXXXXX";
  assert(mkdtemp(directory));
  const auto world = std::string(directory) + "/world.sock";
  const auto clock_file = world + ".clock-v1";
  const int parent = open(directory, O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  const int endpoint = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
  assert(parent >= 0 && endpoint >= 0);
  sockaddr_un address {};
  address.sun_family = AF_UNIX;
  assert(world.size() < sizeof(address.sun_path));
  std::strcpy(address.sun_path, world.c_str());
  assert(bind(endpoint, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) == 0);
  struct stat socket_stat {};
  assert(lstat(world.c_str(), &socket_stat) == 0);
  const std::uint64_t *read_clock;
  constexpr std::uint64_t last_tick = 1'795'058'631'654'793'063;
  {
    xsim::ReceiptClock clock(parent, world);
    clock.bind();
    const auto fd = open(clock_file.c_str(), O_RDONLY | O_NOFOLLOW | O_CLOEXEC);
    assert(fd >= 0);
    const auto *view = mmap(nullptr, 2 * sizeof(*read_clock), PROT_READ, MAP_SHARED, fd, 0);
    assert(view != MAP_FAILED);
    close(fd);
    read_clock = static_cast<const std::uint64_t *>(view);
    assert(read_clock[0] == socket_stat.st_ino);
    assert(__atomic_load_n(read_clock + 1, __ATOMIC_ACQUIRE) == 0);
    clock.publish(last_tick - 8'000'000);
    assert(__atomic_load_n(read_clock + 1, __ATOMIC_ACQUIRE) == last_tick - 8'000'000);
    clock.publish(last_tick);
    assert(__atomic_load_n(read_clock + 1, __ATOMIC_ACQUIRE) == last_tick);
  }
  assert(__atomic_load_n(read_clock + 1, __ATOMIC_ACQUIRE) == (last_tick | (std::uint64_t{1} << 63)));
  assert(lstat(clock_file.c_str(), &socket_stat) == -1 && errno == ENOENT);
  munmap(const_cast<std::uint64_t *>(read_clock), 2 * sizeof(*read_clock));
  close(endpoint);
  assert(unlink(world.c_str()) == 0);
  close(parent);
  assert(rmdir(directory) == 0);
}
