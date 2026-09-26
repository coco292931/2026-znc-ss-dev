#include "tof_slope_sensor.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

#ifndef PATH_FOLLOW_NO_HW
#include "lq_vl53l0x.hpp"
#endif

namespace rewrite_path {

namespace {

std::int64_t monotonic_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

TofSlopeDetector::TofSlopeDetector(const PathParams& params) : p_(params) {}

void TofSlopeDetector::reset() {
    baseline_samples_.clear();
    last_sequence_ = 0;
    filtered_distance_mm_ = 0.0;
    baseline_distance_mm_ = 0.0;
    ramp_active_s_ = 0.0;
    enter_count_ = 0;
    exit_count_ = 0;
    filter_ready_ = false;
    baseline_ready_ = false;
    ramp_active_ = false;
    rearm_blocked_ = false;
}

void TofSlopeDetector::set_ramp_active(bool active) {
    ramp_active_ = active;
    if (active) rearm_blocked_ = false;
    ramp_active_s_ = 0.0;
    enter_count_ = 0;
    exit_count_ = 0;
}

TofSlopeFeedback TofSlopeDetector::update(
    const TofDistanceSample& sample,
    double dt) {
    dt = clamp_value(dt, 0.001, 0.10);
    if (ramp_active_) {
        ramp_active_s_ += dt;
        if (ramp_active_s_ >= p_.ramp_boost_max_s) {
            set_ramp_active(false);
            rearm_blocked_ = true;
        }
    }

    const bool fresh_sample =
        sample.sequence != 0 && sample.sequence != last_sequence_;
    if (fresh_sample) last_sequence_ = sample.sequence;

    if (fresh_sample && sample.valid) {
        if (!filter_ready_) {
            filtered_distance_mm_ = sample.distance_mm;
            filter_ready_ = true;
        } else {
            filtered_distance_mm_ +=
                (sample.distance_mm - filtered_distance_mm_) * 0.35;
        }

        if (!baseline_ready_) {
            baseline_samples_.push_back(filtered_distance_mm_);
            if (static_cast<int>(baseline_samples_.size()) >=
                p_.tof_baseline_frames) {
                std::sort(
                    baseline_samples_.begin(), baseline_samples_.end());
                baseline_distance_mm_ =
                    baseline_samples_[baseline_samples_.size() / 2];
                baseline_ready_ = true;
            }
        } else {
            const double ramp_distance_mm = p_.tof_ramp_delta_mm;
            const double release_distance_mm =
                ramp_distance_mm + p_.tof_ramp_hysteresis_mm;

            if (!ramp_active_) {
                if (rearm_blocked_) {
                    if (sample.distance_mm > release_distance_mm) {
                        ++exit_count_;
                        if (exit_count_ >= p_.tof_ramp_exit_frames) {
                            rearm_blocked_ = false;
                            exit_count_ = 0;
                        }
                    } else {
                        exit_count_ = 0;
                    }
                    enter_count_ = 0;
                } else {
                    if (sample.distance_mm > release_distance_mm) {
                        baseline_distance_mm_ +=
                            (filtered_distance_mm_ -
                             baseline_distance_mm_) *
                            0.01;
                    }
                    if (sample.distance_mm <= ramp_distance_mm) {
                        ++enter_count_;
                        if (enter_count_ >= p_.tof_ramp_enter_frames) {
                            set_ramp_active(true);
                        }
                    } else {
                        enter_count_ = 0;
                    }
                }
            } else {
                if (sample.distance_mm > release_distance_mm) {
                    ++exit_count_;
                    if (exit_count_ >= p_.tof_ramp_exit_frames) {
                        set_ramp_active(false);
                    }
                } else {
                    exit_count_ = 0;
                }
            }
        }
    }

    TofSlopeFeedback out;
    out.distance_mm = sample.distance_mm;
    out.filtered_distance_mm = filtered_distance_mm_;
    out.baseline_distance_mm = baseline_distance_mm_;
    out.signed_delta_mm = baseline_ready_
        ? p_.tof_ramp_sign *
            (filtered_distance_mm_ - baseline_distance_mm_)
        : 0.0;
    out.age_s = sample.age_s;
    out.valid = sample.valid;
    out.baseline_ready = baseline_ready_;
    out.ramp_detected = ramp_active_;
    return out;
}

bool apply_tof_stop_latch(const PathParams& params,
                          const TofSlopeFeedback& tof,
                          bool* stop_latched,
                          NavigationCommand* navigation) {
    if (!params.tof_stop_test || stop_latched == nullptr ||
        navigation == nullptr) {
        return false;
    }
    if (!tof.baseline_ready) {
        navigation->state = DriveState::Stopped;
        navigation->target_speed_cmps = 0.0;
        navigation->target_yaw_rate_dps = 0.0;
        navigation->heading_hold = false;
        navigation->power_boost_percent = 0.0;
        navigation->stop_reason = StopReason::TofUnavailable;
        return true;
    }
    if (tof.ramp_detected) {
        *stop_latched = true;
    }
    if (!*stop_latched) return false;

    navigation->state = DriveState::Stopped;
    navigation->target_speed_cmps = 0.0;
    navigation->target_yaw_rate_dps = 0.0;
    navigation->heading_hold = false;
    navigation->power_boost_percent = 0.0;
    navigation->stop_reason = StopReason::RampDetected;
    return true;
}

struct Vl53l0xDistanceSensor::Impl {
#ifndef PATH_FOLLOW_NO_HW
    std::unique_ptr<lq_vl53l0x> device;
    std::thread worker;
#endif
    std::atomic<bool> running{false};
    std::atomic<bool> valid{false};
    std::atomic<double> distance_mm{0.0};
    std::atomic<std::uint64_t> sequence{0};
    std::atomic<std::int64_t> stamp_ns{0};
};

Vl53l0xDistanceSensor::Vl53l0xDistanceSensor(const PathParams& params)
    : p_(params), impl_(new Impl) {}

Vl53l0xDistanceSensor::~Vl53l0xDistanceSensor() {
    stop();
}

bool Vl53l0xDistanceSensor::start() {
#ifdef PATH_FOLLOW_NO_HW
    return false;
#else
    if (!p_.enable_tof_slope) return false;
    if (impl_->running.load()) return true;

    impl_->device.reset(new lq_vl53l0x(
        static_cast<gpio_pin_t>(84),
        static_cast<gpio_pin_t>(85),
        static_cast<gpio_pin_t>(88)));
    impl_->device->hardware_reset();
    bool initialized = impl_->device->init(true);
    if (!initialized) {
        std::fprintf(
            stderr,
            "[TOF] VL53L0X first init failed; hardware reset and retry\n");
        impl_->device->hardware_reset();
        initialized = impl_->device->init(true);
    }
    if (!initialized) {
        std::fprintf(
            stderr,
            "[TOF] VL53L0X init failed; ramp boost disabled\n");
        impl_->device.reset();
        return false;
    }
    impl_->running.store(true);
    impl_->worker = std::thread([this]() {
        unsigned consecutive_failures = 0;
        while (impl_->running.load()) {
            vl53l0x_result_t result;
            const bool read_ok =
                impl_->device->read_result(result, true, 1000);
            if (read_ok) {
                consecutive_failures = 0;
            } else {
                ++consecutive_failures;
                if (consecutive_failures == 1 ||
                    consecutive_failures % 20 == 0) {
                    std::fprintf(
                        stderr,
                        "[TOF] read failed #%u: %s distance=%umm "
                        "range=0x%02X int=0x%02X\n",
                        consecutive_failures,
                        result.status_text,
                        result.distance_mm,
                        result.range_status,
                        result.interrupt_status);
                }
            }
            impl_->distance_mm.store(result.distance_mm);
            impl_->valid.store(read_ok && result.valid);
            impl_->stamp_ns.store(monotonic_ns());
            impl_->sequence.fetch_add(1);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    });
    std::printf(
        "[TOF] VL53L0X single-shot worker ready "
        "on SCL=84(CH1N) SDA=85(CH2N) XSHUT=88(TIM2_CH2)\n");
    return true;
#endif
}

void Vl53l0xDistanceSensor::stop() {
    impl_->running.store(false);
#ifndef PATH_FOLLOW_NO_HW
    if (impl_->worker.joinable()) {
        impl_->worker.join();
    }
    if (impl_->device) {
        impl_->device.reset();
    }
#endif
    impl_->valid.store(false);
}

TofDistanceSample Vl53l0xDistanceSensor::read() const {
    TofDistanceSample out;
    const std::int64_t stamp = impl_->stamp_ns.load();
    out.sequence = impl_->sequence.load();
    out.distance_mm = impl_->distance_mm.load();
    if (stamp <= 0) return out;
    out.age_s = (monotonic_ns() - stamp) * 1e-9;
    out.valid =
        impl_->valid.load() && out.age_s >= 0.0 &&
        out.age_s <= p_.tof_max_age_s;
    return out;
}

}  // namespace rewrite_path
