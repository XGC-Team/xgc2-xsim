#pragma once
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <system_error>
#include <unistd.h>

namespace xsim {

// Session-local clock capability beside the current world socket. Version 1
// contains its socket inode and the last published frame's simulation time.
// A set high bit marks retirement while preserving the last honest tick.
class ReceiptClock {
 public:
  ReceiptClock(int parent, const std::string &socket)
      : parent_(parent), socket_(socket.substr(socket.rfind('/') + 1)),
        name_(socket_ + ".clock-v1"), temporary_(name_ + "." + std::to_string(getpid())) {}
  ReceiptClock(const ReceiptClock &) = delete;
  ReceiptClock &operator=(const ReceiptClock &) = delete;

  void bind() {
    struct stat socket_stat {};
    if (fstatat(parent_, socket_.c_str(), &socket_stat, AT_SYMLINK_NOFOLLOW) ||
        !S_ISSOCK(socket_stat.st_mode))
      throw std::runtime_error("receipt clock requires the owned world socket");
    const int fd = openat(parent_, temporary_.c_str(), O_CREAT | O_EXCL | O_RDWR |
                          O_NOFOLLOW | O_CLOEXEC, 0600);
    if (fd < 0) fail("create");
    owns_temporary_ = true;
    if (ftruncate(fd, sizeof(Fields))) {
      const auto error = errno;
      close(fd);
      errno = error;
      fail("size");
    }
    if (fstat(fd, &identity_)) {
      const auto error = errno;
      close(fd);
      errno = error;
      fail("identify");
    }
    auto *mapped = mmap(nullptr, sizeof(Fields), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    const auto mapping_error = errno;
    close(fd);
    if (mapped == MAP_FAILED) {
      errno = mapping_error;
      fail("map");
    }
    fields_ = static_cast<Fields *>(mapped);
    fields_->socket_inode = socket_stat.st_ino;
    if (renameat(parent_, temporary_.c_str(), parent_, name_.c_str())) fail("publish");
    owns_temporary_ = false;
  }

  void publish(std::uint64_t stamp) noexcept {
    __atomic_store_n(&fields_->stamp, stamp, __ATOMIC_RELEASE);
  }

  ~ReceiptClock() {
    if (fields_) {
      __atomic_fetch_or(&fields_->stamp, retired, __ATOMIC_RELEASE);
      munmap(fields_, sizeof(Fields));
    }
    if (owns_temporary_) unlinkat(parent_, temporary_.c_str(), 0);
    else if (fields_) {
      struct stat current {};
      if (!fstatat(parent_, name_.c_str(), &current, AT_SYMLINK_NOFOLLOW) &&
          current.st_dev == identity_.st_dev && current.st_ino == identity_.st_ino)
        unlinkat(parent_, name_.c_str(), 0);
    }
  }

 private:
  struct Fields { std::uint64_t socket_inode, stamp; };
  static_assert(__atomic_always_lock_free(sizeof(std::uint64_t), nullptr));
  static constexpr std::uint64_t retired = std::uint64_t{1} << 63;

  [[noreturn]] static void fail(const char *operation) {
    throw std::system_error(errno, std::generic_category(), std::string("receipt clock ") + operation);
  }

  int parent_;
  std::string socket_, name_, temporary_;
  Fields *fields_ = nullptr;
  struct stat identity_ {};
  bool owns_temporary_ = false;
};

} // namespace xsim
