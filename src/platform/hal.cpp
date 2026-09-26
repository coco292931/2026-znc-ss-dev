#include "hal.hpp"
#include "control_safety.hpp"

#include "encoder_capture.hpp"
#include "encoder_filter.hpp"
#include "success_motor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>

#if !defined(SMARTCAR_SIM)
#include "motor_process_lock.hpp"
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#endif

namespace smartcar {
namespace {

template <typename T>
T clamp_value(T value, T low, T high) {
    return std::max(low, std::min(value, high));
}

class SimVehicleHardware final : public VehicleHardware {
public:
    SimVehicleHardware(HardwareConfig config, bool enable_motors)
        : config_(config), enable_motors_(enable_motors) {}

    bool start() override {
        output_faulted_ = false;
        stop();
        std::printf("[HAL] simulation backend active; motor output is virtual\n");
        return true;
    }

    WheelFeedback read_feedback(double dt_seconds) override {
        // 模拟硬件不会读真实编码器，而是把“上一次给的 PWM”平滑映射成
        // 一个虚拟转速，方便主循环和控制器在主机上完整联调。
        const double alpha = clamp_value(dt_seconds * 8.0, 0.0, 1.0);
        left_rpm_ += (last_left_percent_ / 100.0 * max_sim_rpm_ - left_rpm_) * alpha;
        right_rpm_ += (last_right_percent_ / 100.0 * max_sim_rpm_ - right_rpm_) * alpha;
        return WheelFeedback{left_rpm_, right_rpm_, true};
    }

    void set_motor_percent(double left_percent, double right_percent) override {
        if (!finite_motor_pair(left_percent, right_percent)) {
            if (!output_faulted_) std::fprintf(stderr, "[SAFETY] invalid wheel pair; both motors stopped\n");
            output_faulted_ = true;
        }
        if (output_faulted_) { stop(); return; }
        if (!enable_motors_) {
            last_left_percent_ = 0.0;
            last_right_percent_ = 0.0;
            return;
        }
        last_left_percent_ = clamp_value(left_percent,
                                         -config_.max_motor_percent,
                                         config_.max_motor_percent);
        last_right_percent_ = clamp_value(right_percent,
                                          -config_.max_motor_percent,
                                          config_.max_motor_percent);
    }

    void stop() override {
        last_left_percent_ = 0.0;
        last_right_percent_ = 0.0;
    }

    bool motors_enabled() const override {
        return enable_motors_;
    }

private:
    HardwareConfig config_;
    bool enable_motors_ = false;
    bool output_faulted_ = false;
    double left_rpm_ = 0.0;
    double right_rpm_ = 0.0;
    double last_left_percent_ = 0.0;
    double last_right_percent_ = 0.0;
    const double max_sim_rpm_ = 900.0;
};

#if !defined(SMARTCAR_SIM)

constexpr std::uintptr_t kPwmBase = 0x1611B000UL;
constexpr std::uintptr_t kPwmOffset = 0x10UL;
constexpr std::uintptr_t kPwmLowOffset = 0x04UL;
constexpr std::uintptr_t kPwmFullOffset = 0x08UL;
constexpr std::uintptr_t kPwmCtrlOffset = 0x0CUL;
constexpr std::uintptr_t kGpioReuseBase = 0x16000490UL;
constexpr std::uintptr_t kGpioReuseOffset = 0x04UL;
constexpr std::size_t kMapSpan = 0x10000UL;

constexpr std::uint32_t bit(int n) {
    return static_cast<std::uint32_t>(1UL << n);
}

class MappedPage {
public:
    MappedPage() = default;
    MappedPage(const MappedPage&) = delete;
    MappedPage& operator=(const MappedPage&) = delete;

    ~MappedPage() {
        close();
    }

    bool open(std::uintptr_t physical_address) {
        close();

        const int fd = ::open("/dev/mem", O_RDWR | O_SYNC);
        if (fd < 0) {
            std::perror("[HAL] open /dev/mem");
            return false;
        }

        aligned_address_ = physical_address & ~(kMapSpan - 1);
        offset_ = physical_address & (kMapSpan - 1);
        base_ = ::mmap(nullptr, kMapSpan, PROT_READ | PROT_WRITE, MAP_SHARED,
                       fd, static_cast<off_t>(aligned_address_));
        ::close(fd);

        if (base_ == MAP_FAILED) {
            std::perror("[HAL] mmap");
            base_ = nullptr;
            return false;
        }

        return true;
    }

    void close() {
        if (base_ != nullptr) {
            ::munmap(base_, kMapSpan);
            base_ = nullptr;
        }
    }

    volatile std::uint32_t* reg(std::uintptr_t relative_offset = 0) const {
        if (base_ == nullptr) {
            return nullptr;
        }
        auto* bytes = reinterpret_cast<volatile std::uint8_t*>(base_);
        return reinterpret_cast<volatile std::uint32_t*>(bytes + offset_ + relative_offset);
    }

private:
    void* base_ = nullptr;
    std::uintptr_t aligned_address_ = 0;
    std::uintptr_t offset_ = 0;
};

void configure_pin_mux(int pin, int mux) {
    // 龙芯板上很多引脚是复用的，这里先把目标引脚切到 PWM/捕获等外设功能。
    MappedPage page;
    const std::uintptr_t reg_addr =
        kGpioReuseBase + static_cast<std::uintptr_t>(pin / 16) * kGpioReuseOffset;
    if (!page.open(reg_addr)) {
        return;
    }

    volatile std::uint32_t* reg = page.reg();
    const int shift = (pin % 16) * 2;
    std::uint32_t value = *reg;
    value &= ~(0b11u << shift);
    value |= (static_cast<std::uint32_t>(mux) & 0b11u) << shift;
    *reg = value;
}

class CaptureEncoder {
public:
    CaptureEncoder(int pin, int channel, HardwareConfig config)
        : pin_(pin), channel_(channel), config_(config) {}

    bool start() {
        // 编码器这里复用了 PWM 捕获硬件：读取周期计数，再换算成轮速。
        configure_pin_mux(pin_, 1);

        const std::uintptr_t channel_base =
            kPwmBase + static_cast<std::uintptr_t>(channel_) * kPwmOffset;
        if (!page_.open(channel_base)) {
            return false;
        }

        volatile std::uint32_t* ctrl = page_.reg(kPwmCtrlOffset);
        volatile std::uint32_t* full = page_.reg(kPwmFullOffset);
        volatile std::uint32_t* low = page_.reg(kPwmLowOffset);
        if (ctrl == nullptr || full == nullptr || low == nullptr) {
            return false;
        }

        *ctrl = 0;
        ::usleep(1000);
        *full = 0;
        *low = 0;
        *ctrl = bit(0) | bit(8) | bit(5);
        ::usleep(5000);
        capture_period_.reset();
        filtered_rpm_ = 0.0;
        return true;
    }

    double read_wheel_rpm(double commanded_percent, double dt) {
        volatile std::uint32_t* full = page_.reg(kPwmFullOffset);
        if (full == nullptr) {
            return filtered_rpm_;
        }

        // 按新捕获更新时间，不按周期数值是否变化判断新鲜度。
        // 缓冲被消费后读到零时先保持上一周期，确认超时后才送零速滤波。
        const double now_s = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        const std::uint32_t full_count = capture_period_.read(
            *full, now_s, config_.encoder_clock_hz);

        double rpm = 0.0;
        if (full_count > 0) {
            const double encoder_rpm =
                config_.encoder_clock_hz / static_cast<double>(full_count) /
                static_cast<double>(config_.encoder_lines) * 60.0;
            rpm = encoder_rpm * config_.encoder_gear_ratio;
            if (std::abs(rpm) > 3000.0) {
                rpm = std::abs(filtered_rpm_);
            }
        }

        // Bound a sudden sample instead of holding the old filtered value.
        // Holding forever allowed repeated rejected pulses to masquerade as a
        // valid, frozen nonzero wheel speed and drove the PI loop to full PWM.
        rpm = bound_encoder_rpm_magnitude(rpm, filtered_rpm_);

        if (commanded_percent < -0.1) {
            rpm = -std::abs(rpm);
        } else if (commanded_percent > 0.1) {
            rpm = std::abs(rpm);
        }

        // 采样值最后做一次低通，降低抖动对轮速环的影响。
        const double alpha = encoder_alpha_for_dt(
            config_.encoder_filter_alpha, dt, config_.encoder_filter_reference_hz);
        filtered_rpm_ += (rpm - filtered_rpm_) * alpha;
        return filtered_rpm_;
    }

private:
    int pin_ = 0;
    int channel_ = 0;
    HardwareConfig config_;
    MappedPage page_;
    double filtered_rpm_ = 0.0;
    EncoderCapturePeriod capture_period_;
};

class LqVehicleHardware final : public VehicleHardware {
public:
    LqVehicleHardware(HardwareConfig config, bool enable_motors)
        : config_(config),
          enable_motors_(enable_motors),
          left_encoder_(config.left_encoder_pin, config.left_encoder_channel, config),
          right_encoder_(config.right_encoder_pin, config.right_encoder_channel, config) {}

    bool start() override {
        ownership_ = acquire_motor_process_lock();
        if (!ownership_) return false;
        output_faulted_ = false;
        // 真机启动顺序是：先拉起编码器反馈；如果没有显式解锁电机，
        // 就停在只读硬件模式，只允许看状态不允许出 PWM。
        const bool left_ok = left_encoder_.start();
        const bool right_ok = right_encoder_.start();
        feedback_valid_ = left_ok && right_ok;
        if (!feedback_valid_) {
            std::printf("[HAL] encoder init failed; rpm display may stay zero\n");
        }

        if (!enable_motors_) {
            std::printf("[HAL] motor output DISABLED; pass --enable-motors to arm PWM\n");
            return true;
        }

        SuccessMotorConfig motor_config;
        motor_config.pwm_frequency_hz = config_.pwm_frequency_hz;
        motor_config.duty_max = config_.duty_max;
        motor_config.max_speed_percent = config_.max_motor_percent;

        left_motor_ = std::make_unique<SuccessMotor>(
            SuccessMotorPins{"left",
                             config_.left_pwm_pin,
                             config_.left_pwm_channel,
                             config_.left_dir_pin,
                             config_.left_motor_reversed},
            motor_config);
        right_motor_ = std::make_unique<SuccessMotor>(
            SuccessMotorPins{"right",
                             config_.right_pwm_pin,
                             config_.right_pwm_channel,
                             config_.right_dir_pin,
                             config_.right_motor_reversed},
            motor_config);

        const bool motor_ok = left_motor_->start() && right_motor_->start();
        stop();
        return motor_ok;
    }

    WheelFeedback read_feedback(double dt) override {
        const double left_cmd = left_motor_ ? left_motor_->last_speed_percent() : 0.0;
        const double right_cmd = right_motor_ ? right_motor_->last_speed_percent() : 0.0;
        const double left = left_encoder_.read_wheel_rpm(left_cmd, dt);
        const double right = right_encoder_.read_wheel_rpm(right_cmd, dt);
        return WheelFeedback{left, right, feedback_valid_};
    }

    void set_motor_percent(double left_percent, double right_percent) override {
        if (!finite_motor_pair(left_percent, right_percent)) {
            if (!output_faulted_) std::fprintf(stderr, "[SAFETY] invalid wheel pair; both motors stopped\n");
            output_faulted_ = true;
        }
        if (output_faulted_) { stop(); return; }
        if (!enable_motors_ || !left_motor_ || !right_motor_) {
            return;
        }
        // 到硬件层这里就不再做控制决策了，只负责把上层给出的百分比落到驱动。
        left_motor_->set_speed(left_percent);
        right_motor_->set_speed(right_percent);
    }

    void stop() override {
        if (left_motor_) {
            left_motor_->stop();
        }
        if (right_motor_) {
            right_motor_->stop();
        }
    }

    bool motors_enabled() const override {
        return enable_motors_;
    }

private:
    HardwareConfig config_;
    bool enable_motors_ = false;
    bool output_faulted_ = false;
    std::unique_ptr<SuccessMotor> left_motor_;
    std::unique_ptr<SuccessMotor> right_motor_;
    std::shared_ptr<MotorProcessLock> ownership_;
    CaptureEncoder left_encoder_;
    CaptureEncoder right_encoder_;
    bool feedback_valid_ = false;
};

#endif  // !SMARTCAR_SIM

}  // namespace

std::unique_ptr<VehicleHardware> make_vehicle_hardware(const HardwareConfig& config,
                                                       bool dry_run,
                                                       bool enable_motors) {
    // 硬件入口统一在这里切换：仿真构建只能走模拟后端；
    // 真机构建时，dry-run 也会强制使用模拟电机输出，避免误上电。
#if defined(SMARTCAR_SIM)
    (void)dry_run;
    return std::make_unique<SimVehicleHardware>(config, enable_motors);
#else
    if (dry_run) {
        return std::make_unique<SimVehicleHardware>(config, true);
    }
    return std::make_unique<LqVehicleHardware>(config, enable_motors);
#endif
}

std::string hardware_backend_name(bool dry_run, bool enable_motors) {
#if defined(SMARTCAR_SIM)
    (void)dry_run;
    (void)enable_motors;
    return "simulation";
#else
    if (dry_run) {
        return "dry-run simulation";
    }
    return enable_motors ? "Loongson 2K301 hardware ARMED"
                         : "Loongson 2K301 hardware READ_ONLY";
#endif
}

}  // namespace smartcar
