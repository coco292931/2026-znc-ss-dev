#include "success_motor.hpp"
#include "control_safety.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <mutex>

#if !defined(SMARTCAR_SIM)
#include "motor_process_lock.hpp"
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "LQ_HW_GPIO.hpp"
#endif

namespace smartcar {
namespace {

template <typename T>
T clamp_value(T value, T low, T high) {
    return std::max(low, std::min(value, high));
}

#if !defined(SMARTCAR_SIM)

constexpr std::uintptr_t kAtimBase = 0x16118000UL;
constexpr std::uintptr_t kAtimCr1 = 0x00UL;
constexpr std::uintptr_t kAtimEgr = 0x14UL;
constexpr std::uintptr_t kAtimCcmr1 = 0x18UL;
constexpr std::uintptr_t kAtimCcmr2 = 0x1CUL;
constexpr std::uintptr_t kAtimCcer = 0x20UL;
constexpr std::uintptr_t kAtimArr = 0x2CUL;
constexpr std::uintptr_t kAtimCcr1 = 0x34UL;
constexpr std::uintptr_t kAtimBdtr = 0x44UL;
constexpr std::uintptr_t kAtimCcrOffset = 0x04UL;
constexpr std::uintptr_t kGpioReuseBase = 0x16000490UL;
constexpr std::uintptr_t kGpioReuseOffset = 0x04UL;
constexpr std::size_t kMapSpan = 0x10000UL;
constexpr std::uint32_t kAtimClockHz = 160000000UL;
constexpr std::uint32_t kModePwm2 = 0x7;
constexpr std::uint32_t kDutyMax = 10000;

std::mutex& atim_mutex() {
    static std::mutex mutex;
    return mutex;
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
            std::perror("[MOTOR] open /dev/mem");
            return false;
        }

        aligned_address_ = physical_address & ~(kMapSpan - 1);
        offset_ = physical_address & (kMapSpan - 1);
        base_ = ::mmap(nullptr, kMapSpan, PROT_READ | PROT_WRITE, MAP_SHARED,
                       fd, static_cast<off_t>(aligned_address_));
        ::close(fd);

        if (base_ == MAP_FAILED) {
            std::perror("[MOTOR] mmap");
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

#endif  // !SMARTCAR_SIM

}  // namespace

struct SuccessMotor::Impl {
#if !defined(SMARTCAR_SIM)
    std::shared_ptr<MotorProcessLock> ownership;
    MappedPage atim;
    std::unique_ptr<HWGpio> dir;
    std::uint32_t arr = 999;
    int channel_index = 0;
#endif
};

SuccessMotor::SuccessMotor(SuccessMotorPins pins, SuccessMotorConfig config)
    : pins_(pins), config_(config), impl_(std::make_unique<Impl>()) {}

SuccessMotor::~SuccessMotor() {
    stop();
}

bool SuccessMotor::start() {
    if (!std::isfinite(config_.max_speed_percent) || config_.max_speed_percent < 0.0 ||
        config_.max_speed_percent > 100.0 || !std::isfinite(config_.pwm_frequency_hz) ||
        config_.pwm_frequency_hz < 1.0 || config_.pwm_frequency_hz > 80000000.0 ||
        config_.duty_max <= 0) {
        faulted_ = true;
        stop();
        return false;
    }
    faulted_ = false;
#if defined(SMARTCAR_SIM)
    last_speed_percent_ = 0.0;
    return true;
#else
    if (pins_.pwm_channel < 1 || pins_.pwm_channel > 4) {
        std::fprintf(stderr, "[MOTOR] invalid ATIM channel %d\n", pins_.pwm_channel);
        return false;
    }

    impl_->ownership = acquire_motor_process_lock();
    if (!impl_->ownership) return false;
    impl_->channel_index = pins_.pwm_channel - 1;
    const double pwm_freq = std::max(1.0, config_.pwm_frequency_hz);
    impl_->arr = static_cast<std::uint32_t>(
        std::max(1.0, std::round(static_cast<double>(kAtimClockHz) / pwm_freq) - 1.0));

    configure_pin_mux(pins_.pwm_pin, 0b11);
    impl_->dir = std::make_unique<HWGpio>(static_cast<std::uint8_t>(pins_.dir_pin),
                                          GPIO_Mode_Out);
    if (!impl_->atim.open(kAtimBase)) {
        return false;
    }

    const auto stop_duty = duty_for_speed(0.0);
    const int ch = impl_->channel_index;
    const int pair = ch / 2;
    const int shift = (ch % 2) * 8;
    const int ccer_shift = ch * 4;

    std::lock_guard<std::mutex> lock(atim_mutex());
    volatile std::uint32_t* ccer = impl_->atim.reg(kAtimCcer);
    volatile std::uint32_t* ccmr = impl_->atim.reg(pair == 0 ? kAtimCcmr1 : kAtimCcmr2);
    volatile std::uint32_t* arr = impl_->atim.reg(kAtimArr);
    volatile std::uint32_t* ccr = impl_->atim.reg(kAtimCcr1 + ch * kAtimCcrOffset);
    volatile std::uint32_t* bdtr = impl_->atim.reg(kAtimBdtr);
    volatile std::uint32_t* cr1 = impl_->atim.reg(kAtimCr1);
    volatile std::uint32_t* egr = impl_->atim.reg(kAtimEgr);

    *ccer &= ~(1u << ccer_shift);

    std::uint32_t mode_reg = *ccmr;
    mode_reg &= ~(0x3u << shift);
    mode_reg &= ~(0x7u << (shift + 4));
    mode_reg |= (kModePwm2 << (shift + 4));
    mode_reg |= (1u << (shift + 3));
    *ccmr = mode_reg;

    std::uint32_t ccer_reg = *ccer;
    // 对齐龙邱验证可用的 ls_atim_pwm：使用 INV 极性（CCxP=1）。
    // 旧实现清零=NORMAL 极性，配合反比 duty 公式，实测有效输出极低。
    // 改为 INV 极性 + 正比 duty（见 duty_for_speed）后与 lq_motor 行为一致。
    ccer_reg |= (1u << (ccer_shift + 1));
    *ccer = ccer_reg;

    *arr = impl_->arr;
    *ccr = stop_duty;
    *bdtr |= (1u << 15);
    *egr = 0x01;
    *cr1 = 0b10000001;
    *ccer |= (1u << ccer_shift);

    last_speed_percent_ = 0.0;
    const double effective_hz = static_cast<double>(kAtimClockHz) /
                                static_cast<double>(impl_->arr + 1u);
    std::printf("[MOTOR] %s pwm=PIN_%d/ch%d dir=PIN_%d reversed=%d "
                "freq=%.1fHz arr=%u ccr(5%%)=%u ccr(max %.0f%%)=%u stop_ccr=%u\n",
                pins_.name,
                pins_.pwm_pin,
                pins_.pwm_channel,
                pins_.dir_pin,
                pins_.reversed ? 1 : 0,
                effective_hz,
                impl_->arr,
                duty_for_speed(5.0),
                config_.max_speed_percent,
                duty_for_speed(config_.max_speed_percent),
                stop_duty);
    return true;
#endif
}

void SuccessMotor::set_speed(double speed_percent) {
    if (!std::isfinite(speed_percent)) {
        if (!faulted_) std::fprintf(stderr, "[SAFETY] non-finite motor output; latched stop\n");
        faulted_ = true;
    }
    if (faulted_) { stop(); return; }
    const double bounded = clamp_value(speed_percent,
                                       -config_.max_speed_percent,
                                       config_.max_speed_percent);
#if !defined(SMARTCAR_SIM)
    if (!impl_->dir || !impl_->atim.reg()) {
        return;
    }

    bool forward = bounded >= 0.0;
    if (pins_.reversed) {
        forward = !forward;
    }

    const int ch = impl_->channel_index;
    volatile std::uint32_t* ccr = impl_->atim.reg(kAtimCcr1 + ch * kAtimCcrOffset);
    impl_->dir->SetGpioValue(forward ? 1 : 0);
    {
        std::lock_guard<std::mutex> lock(atim_mutex());
        *ccr = duty_for_speed(bounded);
        *impl_->atim.reg(kAtimCcer) |= (1u << (ch * 4));
    }
#endif
    last_speed_percent_ = bounded;
}

void SuccessMotor::stop() {
#if !defined(SMARTCAR_SIM)
    if (impl_ && impl_->atim.reg()) {
        const int ch = impl_->channel_index;
        std::lock_guard<std::mutex> lock(atim_mutex());
        *impl_->atim.reg(kAtimCcr1 + ch * kAtimCcrOffset) = duty_for_speed(0.0);
        *impl_->atim.reg(kAtimCcer) |= (1u << (ch * 4));
    }
#endif
    last_speed_percent_ = 0.0;
}

double SuccessMotor::last_speed_percent() const {
    return last_speed_percent_;
}

const SuccessMotorPins& SuccessMotor::pins() const {
    return pins_;
}

std::uint32_t SuccessMotor::duty_for_speed(double bounded_speed) const {
    const double speed = clamp_value(std::abs(bounded_speed), 0.0, 100.0);
#if defined(SMARTCAR_SIM)
    const double period_ticks = static_cast<double>(config_.duty_max);
#else
    const double period_ticks = static_cast<double>(impl_->arr + 1);
#endif
    // 对齐龙邱验证可用的 ls_atim_pwm：INV 极性 + 正比 duty 公式（CCR ∝ speed），
    // speed=0 → CCR=0（停），speed=100 → CCR=period（最快）。
    // 旧实现用 NORMAL 极性 + 反比公式 period*(1-speed/100)，实测有效输出极低
    // （30% 才勉强起转），改回正比后与 lq_motor（2% 起转）一致。
    const double duty = period_ticks * (speed / 100.0);
    const auto rounded = static_cast<int>(std::lround(duty));
    return static_cast<std::uint32_t>(clamp_value(rounded, 0, static_cast<int>(period_ticks)));
}

}  // namespace smartcar
