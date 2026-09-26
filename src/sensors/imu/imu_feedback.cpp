#include "imu_feedback.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>
#include <vector>

#ifndef PATH_FOLLOW_NO_HW
#include "lq_lsm6dsr.hpp"
#include "lsm6dsr_spi1.hpp"
#include <pthread.h>
#include <unistd.h>
#endif

namespace rewrite_path {

namespace {

constexpr double kGravityMps2 = 9.80665;
constexpr double kRadiansPerDegree = 0.01745329251994329577;

std::int64_t monotonic_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

struct ImuYawSensor::Impl {
#ifndef PATH_FOLLOW_NO_HW
    std::unique_ptr<lq_lsm6dsr> i2c_device;
    std::unique_ptr<Lsm6dsrSpi1> spi_device;
    std::thread worker;

    bool read_sample(lsm6dsr_sample_t& sample) {
        if (i2c_device) return i2c_device->read_sample(sample);
        if (spi_device) return spi_device->read_sample(sample);
        return false;
    }

    void power_down() {
        if (i2c_device) i2c_device->power_down();
        if (spi_device) spi_device->power_down();
    }

    void clear_device() {
        i2c_device.reset();
        spi_device.reset();
    }
#endif
    std::atomic<bool> running{false};
    std::atomic<bool> valid{false};
    std::atomic<bool> stationary{false};
    std::atomic<double> yaw_rate_dps{0.0};
    std::atomic<double> heading_deg{0.0};
    std::atomic<double> bias_dps{0.0};
    std::atomic<double> accel_x_bias_g{0.0};
    std::atomic<double> accel_y_bias_g{0.0};
    std::atomic<double> raw_accel_x_g{0.0};
    std::atomic<double> raw_accel_y_g{0.0};
    std::atomic<double> raw_accel_z_g{0.0};
    std::atomic<double> forward_accel_mps2{0.0};
    std::atomic<double> right_accel_mps2{0.0};
    std::atomic<double> velocity_x_mps{0.0};
    std::atomic<double> velocity_y_mps{0.0};
    std::atomic<double> position_x_m{0.0};
    std::atomic<double> position_y_m{0.0};
    std::atomic<std::int64_t> stamp_ns{0};
};

ImuYawSensor::ImuYawSensor(const PathParams& params)
    : p_(params), impl_(new Impl) {}

ImuYawSensor::~ImuYawSensor() {
    stop();
}

bool ImuYawSensor::start() {
#ifdef PATH_FOLLOW_NO_HW
    return false;
#else
    if (impl_->running.load()) return true;

    bool initialized = false;
    if (p_.imu_transport == "auto" || p_.imu_transport == "i2c") {
        std::printf("[IMU] probing I2C SCL=84 SDA=85 addr=0x6A\n");
        impl_->i2c_device.reset(new lq_lsm6dsr());
        initialized = impl_->i2c_device->init(
            LSM6DSR_ODR_104HZ,
            LSM6DSR_ACCEL_FS_2G,
            LSM6DSR_GYRO_FS_500DPS);
        if (!initialized) impl_->i2c_device.reset();
    }
    if (!initialized &&
        (p_.imu_transport == "auto" || p_.imu_transport == "spi")) {
        std::printf(
            "[IMU] probing shared SPI1 SCK=60 MOSI=62 MISO=61 CS=25 "
            "speed=%dHz\n",
            p_.imu_spi_speed_hz);
        impl_->spi_device.reset(new Lsm6dsrSpi1(
            p_.display_spi_device,
            static_cast<std::uint32_t>(p_.imu_spi_speed_hz)));
        initialized = impl_->spi_device->init(
            LSM6DSR_ODR_104HZ,
            LSM6DSR_ACCEL_FS_2G,
            LSM6DSR_GYRO_FS_500DPS);
        if (!initialized) {
            std::fprintf(stderr, "[IMU] SPI1 init failed: %s\n",
                         impl_->spi_device->last_error().c_str());
            impl_->spi_device.reset();
        }
    }
    if (!initialized) {
        std::fprintf(
            stderr,
            "[IMU] LSM6DSR init failed on transport=%s; "
            "using degraded control\n",
            p_.imu_transport.c_str());
        impl_->clear_device();
        return false;
    }
    std::printf("[IMU] transport=%s\n",
                impl_->spi_device ? "SPI" : "I2C");

    const double calibration_s = std::max(0.4, p_.imu_calibrate_s);
    std::printf("[IMU] stationary calibration %.1fs\n", calibration_s);
    std::vector<double> gyro_samples;
    std::vector<double> accel_x_samples;
    std::vector<double> accel_y_samples;
    const auto begin = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(
               std::chrono::steady_clock::now() - begin).count() <
           calibration_s) {
        lsm6dsr_sample_t sample{};
        if (impl_->read_sample(sample)) {
            gyro_samples.push_back(sample.gyro_dps[2]);
            accel_x_samples.push_back(sample.accel_g[0]);
            accel_y_samples.push_back(sample.accel_g[1]);
        }
        ::usleep(8000);
    }
    if (gyro_samples.size() < 12) {
        std::fprintf(stderr, "[IMU] calibration samples insufficient: %zu\n",
                     gyro_samples.size());
        impl_->power_down();
        impl_->clear_device();
        return false;
    }
    std::sort(gyro_samples.begin(), gyro_samples.end());
    std::sort(accel_x_samples.begin(), accel_x_samples.end());
    std::sort(accel_y_samples.begin(), accel_y_samples.end());
    impl_->bias_dps.store(gyro_samples[gyro_samples.size() / 2]);
    impl_->accel_x_bias_g.store(
        accel_x_samples[accel_x_samples.size() / 2]);
    impl_->accel_y_bias_g.store(
        accel_y_samples[accel_y_samples.size() / 2]);
    impl_->heading_deg.store(0.0);
    impl_->forward_accel_mps2.store(0.0);
    impl_->right_accel_mps2.store(0.0);
    impl_->velocity_x_mps.store(0.0);
    impl_->velocity_y_mps.store(0.0);
    impl_->position_x_m.store(0.0);
    impl_->position_y_m.store(0.0);
    impl_->running.store(true);

    impl_->worker = std::thread([this] {
        // Raise this thread's scheduling priority so the 8ms IMU sampling loop
        // is not starved by vision, NCNN recognition, or JPEG encoding. When
        // starved past imu_max_age_s the sample reads stale, the controller
        // drops out of Full mode, and vehicle speed chops. Requires
        // CAP_SYS_NICE (typically root); on failure we log and continue at the
        // default priority.
        sched_param param{};
        param.sched_priority = 10;
        const int rc = pthread_setschedparam(
            pthread_self(), SCHED_FIFO, &param);
        if (rc != 0) {
            std::fprintf(
                stderr,
                "[IMU] could not raise thread priority (rc=%d); "
                "running at default scheduling\n",
                rc);
        }
        auto previous = std::chrono::steady_clock::now();
        double filtered_rate = 0.0;
        double filtered_forward_accel = 0.0;
        double filtered_right_accel = 0.0;
        double velocity_x = 0.0;
        double velocity_y = 0.0;
        double position_x = 0.0;
        double position_y = 0.0;
        double stationary_candidate_s = 0.0;
        int failures = 0;
        while (impl_->running.load()) {
            lsm6dsr_sample_t sample{};
            if (impl_->read_sample(sample)) {
                const auto now = std::chrono::steady_clock::now();
                const double dt = clamp_value(
                    std::chrono::duration<double>(now - previous).count(),
                    0.001, 0.05);
                previous = now;

                double bias = impl_->bias_dps.load();
                double rate =
                    p_.imu_yaw_sign * (sample.gyro_dps[2] - bias);
                double accel_x_bias = impl_->accel_x_bias_g.load();
                double accel_y_bias = impl_->accel_y_bias_g.load();
                double corrected_x = sample.accel_g[0] - accel_x_bias;
                double corrected_y = sample.accel_g[1] - accel_y_bias;
                double forward_g = p_.imu_accel_forward_sign *
                    (p_.imu_accel_swap_xy ? corrected_y : corrected_x);
                double right_g = p_.imu_accel_right_sign *
                    (p_.imu_accel_swap_xy ? corrected_x : corrected_y);
                const double horizontal_accel =
                    std::hypot(forward_g, right_g);
                const bool stationary_candidate =
                    std::abs(rate) <= p_.imu_stationary_yaw_dps &&
                    horizontal_accel <= p_.imu_stationary_accel_g;
                stationary_candidate_s = stationary_candidate
                    ? stationary_candidate_s + dt
                    : 0.0;
                const bool still = stationary_candidate &&
                    stationary_candidate_s >= p_.imu_stationary_hold_s;
                if (stationary_candidate) {
                    const double adapt = clamp_value(
                        p_.imu_bias_adapt_rate * dt, 0.0, 0.02);
                    bias += (sample.gyro_dps[2] - bias) * adapt;
                    impl_->bias_dps.store(bias);
                    rate = 0.0;
                } else if (std::abs(rate) < p_.imu_deadband_dps) {
                    rate = 0.0;
                }
                filtered_rate += (rate - filtered_rate) * 0.45;
                filtered_forward_accel +=
                    (forward_g * kGravityMps2 - filtered_forward_accel) *
                    0.35;
                filtered_right_accel +=
                    (right_g * kGravityMps2 - filtered_right_accel) * 0.35;
                const double previous_heading = impl_->heading_deg.load();
                const double heading = previous_heading + filtered_rate * dt;
                const double middle_heading =
                    (previous_heading + 0.5 * filtered_rate * dt) *
                    kRadiansPerDegree;
                const double cosine = std::cos(middle_heading);
                const double sine = std::sin(middle_heading);
                const double integration_forward_accel =
                    std::abs(forward_g) < p_.imu_accel_deadband_g
                    ? 0.0
                    : filtered_forward_accel;
                const double integration_right_accel =
                    std::abs(right_g) < p_.imu_accel_deadband_g
                    ? 0.0
                    : filtered_right_accel;
                const double accel_x =
                    cosine * integration_forward_accel -
                    sine * integration_right_accel;
                const double accel_y =
                    sine * integration_forward_accel +
                    cosine * integration_right_accel;
                if (p_.imu_stationary_zero && still) {
                    velocity_x = 0.0;
                    velocity_y = 0.0;
                } else {
                    position_x +=
                        velocity_x * dt + 0.5 * accel_x * dt * dt;
                    position_y +=
                        velocity_y * dt + 0.5 * accel_y * dt * dt;
                    velocity_x += accel_x * dt;
                    velocity_y += accel_y * dt;
                }
                impl_->yaw_rate_dps.store(filtered_rate);
                impl_->heading_deg.store(heading);
                impl_->raw_accel_x_g.store(sample.accel_g[0]);
                impl_->raw_accel_y_g.store(sample.accel_g[1]);
                impl_->raw_accel_z_g.store(sample.accel_g[2]);
                impl_->forward_accel_mps2.store(filtered_forward_accel);
                impl_->right_accel_mps2.store(filtered_right_accel);
                impl_->velocity_x_mps.store(velocity_x);
                impl_->velocity_y_mps.store(velocity_y);
                impl_->position_x_m.store(position_x);
                impl_->position_y_m.store(position_y);
                impl_->stationary.store(still);
                impl_->stamp_ns.store(monotonic_ns());
                impl_->valid.store(true);
                failures = 0;
            } else if (++failures >= 5) {
                impl_->valid.store(false);
            }
            ::usleep(8000);
        }
    });
    std::printf(
        "[IMU] ready gyro_bias=%+.3fdps accel_bias=%+.4f/%+.4fg "
        "sign=%+.0f samples=%zu\n",
        impl_->bias_dps.load(),
        impl_->accel_x_bias_g.load(),
        impl_->accel_y_bias_g.load(),
        p_.imu_yaw_sign,
        gyro_samples.size());
    return true;
#endif
}

void ImuYawSensor::stop() {
    impl_->running.store(false);
#ifndef PATH_FOLLOW_NO_HW
    if (impl_->worker.joinable()) impl_->worker.join();
    impl_->power_down();
    impl_->clear_device();
#endif
    impl_->valid.store(false);
}

ImuFeedback ImuYawSensor::read() const {
    ImuFeedback out;
    const std::int64_t stamp = impl_->stamp_ns.load();
    if (stamp <= 0) return out;
    out.age_s = (monotonic_ns() - stamp) * 1e-9;
    out.valid =
        impl_->valid.load() && out.age_s >= 0.0 &&
        out.age_s <= p_.imu_max_age_s;
    out.yaw_rate_dps = impl_->yaw_rate_dps.load();
    out.heading_deg = impl_->heading_deg.load();
    out.raw_accel_x_g = impl_->raw_accel_x_g.load();
    out.raw_accel_y_g = impl_->raw_accel_y_g.load();
    out.raw_accel_z_g = impl_->raw_accel_z_g.load();
    out.forward_accel_mps2 = impl_->forward_accel_mps2.load();
    out.right_accel_mps2 = impl_->right_accel_mps2.load();
    out.velocity_x_mps = impl_->velocity_x_mps.load();
    out.velocity_y_mps = impl_->velocity_y_mps.load();
    out.position_x_m = impl_->position_x_m.load();
    out.position_y_m = impl_->position_y_m.load();
    out.stationary = impl_->stationary.load();
    return out;
}

}  // namespace rewrite_path
