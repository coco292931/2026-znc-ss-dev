#pragma once

#include <memory>
#include <mutex>
#include <cstdio>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

namespace smartcar {

// Cooperative, process-wide ownership. Never unlink the lock file: a second
// inode would let two processes both hold a lock. /dev/mem still needs privilege.
class MotorProcessLock {
public:
    MotorProcessLock() = default;
    MotorProcessLock(const MotorProcessLock&) = delete;
    MotorProcessLock& operator=(const MotorProcessLock&) = delete;
    ~MotorProcessLock() { if (fd_ >= 0) ::close(fd_); }

    bool acquire(const char* path = "/run/smartcar-motor.lock") {
        if (fd_ >= 0) return true;
        const int fd = ::open(path, O_RDWR | O_CREAT | O_CLOEXEC | O_NOFOLLOW, 0600);
        if (fd < 0) return false;
        struct stat st{};
        if (::fstat(fd, &st) != 0 || !S_ISREG(st.st_mode) ||
            st.st_uid != ::geteuid() || (st.st_mode & 0022) != 0 ||
            ::flock(fd, LOCK_EX | LOCK_NB) != 0) {
            ::close(fd);
            return false;
        }
        fd_ = fd;
        return true;
    }
private:
    int fd_ = -1;
};

// A wheel pair and its encoder HAL share one lease inside the process.
// Do not fork while controlling hardware; child processes must exec before use.
inline std::shared_ptr<MotorProcessLock> acquire_motor_process_lock() {
    static std::mutex mutex;
    static std::weak_ptr<MotorProcessLock> current;
    std::lock_guard<std::mutex> guard(mutex);
    if (auto lease = current.lock()) return lease;
    auto lease = std::make_shared<MotorProcessLock>();
    if (!lease->acquire()) {
        std::fprintf(stderr, "[SAFETY] cannot acquire /run/smartcar-motor.lock; "
                             "another controller may own the hardware\n");
        return {};
    }
    current = lease;
    return lease;
}

}  // namespace smartcar
