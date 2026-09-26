#include "motion_control.hpp"
#include "motor_adapter.hpp"

#if defined(MOTOR_TEST_USE_WORKER)
#include "motor_io_worker.hpp"
#endif

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#if !defined(SMARTCAR_SIM)
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#endif

#ifndef MOTOR_TEST_VARIANT
#define MOTOR_TEST_VARIANT "unknown"
#endif

namespace {

using Clock = std::chrono::steady_clock;
using rewrite_path::ControlCommand;
using rewrite_path::ImuFeedback;
using rewrite_path::MotionController;
using rewrite_path::MotorAdapter;
using rewrite_path::MotorFeedbackLite;
using rewrite_path::NavigationCommand;
using rewrite_path::PathParams;

std::atomic<bool> g_running{true};

void on_signal(int) {
    g_running.store(false);
}

struct Options {
    bool enable_motors = false;
    bool confirmed_wheels_lifted = false;
    bool imu_invalid = false;
    double percent = 12.0;
    double target_speed_cmps = 20.0;
    double phase_seconds = 8.0;
    double coast_seconds = 0.0;
    int control_hz = 50;
    int stall_every = 0;
    int stall_ms = 0;
    std::string phase = "all";
    std::string wheel = "both";
    std::string output = std::string("/tmp/motor_stutter_") +
        MOTOR_TEST_VARIANT + ".csv";
};

void print_usage(const char* exe) {
    std::printf(
        "Usage: %s [--dry-run | --enable-motors --confirm-wheels-lifted]\n"
        "          [--percent 12] [--target-speed 20] [--seconds 8]\n"
        "          [--coast-seconds 0] (record feedback after PWM becomes zero)\n"
        "          [--control-hz 50] [--stall-every 0] [--stall-ms 0]\n"
        "          [--output /tmp/motor_stutter.csv]\n"
        "          [--phase all|constant_once|constant_repeat|closed_loop_app]\n"
        "          [--wheel both|left|right] (single wheel: one phase only)\n"
        "          [--imu-invalid] Match main-program IMU degradation\n"
        "          [controller/hardware options]\n\n"
        "Hardware options: --pwm-frequency, --encoder-alpha, --gear-ratio,\n"
        "  --max-percent, --max-speed, --target-accel, --target-decel,\n"
        "  --pwm-slew, --speed-ff, --speed-kp, --speed-ki,\n"
        "  --no-flip-motors, --swap-motors, --swap-encoders.\n",
        exe);
}

bool parse_double(int argc, char** argv, int* index, double* value) {
    if (*index + 1 >= argc) return false;
    *value = std::atof(argv[++(*index)]);
    return std::isfinite(*value);
}

bool parse_int(int argc, char** argv, int* index, int* value) {
    if (*index + 1 >= argc) return false;
    *value = std::atoi(argv[++(*index)]);
    return true;
}

bool parse_args(int argc, char** argv, Options* options, PathParams* params) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        if (arg == "-h" || arg == "--help") {
            print_usage(argv[0]);
            std::exit(0);
        } else if (arg == "--enable-motors") {
            options->enable_motors = true;
        } else if (arg == "--dry-run") {
            options->enable_motors = false;
        } else if (arg == "--confirm-wheels-lifted") {
            options->confirmed_wheels_lifted = true;
        } else if (arg == "--imu-invalid") {
            options->imu_invalid = true;
        } else if (arg == "--percent") {
            if (!parse_double(argc, argv, &i, &options->percent)) return false;
        } else if (arg == "--target-speed") {
            if (!parse_double(argc, argv, &i, &options->target_speed_cmps)) return false;
        } else if (arg == "--seconds") {
            if (!parse_double(argc, argv, &i, &options->phase_seconds)) return false;
        } else if (arg == "--coast-seconds") {
            if (!parse_double(argc, argv, &i, &options->coast_seconds)) return false;
        } else if (arg == "--control-hz") {
            if (!parse_int(argc, argv, &i, &options->control_hz)) return false;
        } else if (arg == "--stall-every") {
            if (!parse_int(argc, argv, &i, &options->stall_every)) return false;
        } else if (arg == "--stall-ms") {
            if (!parse_int(argc, argv, &i, &options->stall_ms)) return false;
        } else if (arg == "--output") {
            if (i + 1 >= argc) return false;
            options->output = argv[++i];
        } else if (arg == "--phase") {
            if (i + 1 >= argc) return false;
            options->phase = argv[++i];
            if (options->phase != "all" && options->phase != "constant_once" &&
                options->phase != "constant_repeat" &&
                options->phase != "closed_loop_app") return false;
        } else if (arg == "--wheel") {
            if (i + 1 >= argc) return false;
            options->wheel = argv[++i];
            if (options->wheel != "both" && options->wheel != "left" &&
                options->wheel != "right") return false;
        } else if (arg == "--pwm-frequency") {
            if (!parse_double(argc, argv, &i, &params->pwm_frequency_hz)) return false;
        } else if (arg == "--encoder-alpha") {
            if (!parse_double(argc, argv, &i, &params->encoder_filter_alpha)) return false;
        } else if (arg == "--gear-ratio") {
            if (!parse_double(argc, argv, &i, &params->encoder_gear_ratio)) return false;
        } else if (arg == "--max-percent") {
            if (!parse_double(argc, argv, &i, &params->max_percent)) return false;
        } else if (arg == "--max-speed") {
            if (!parse_double(argc, argv, &i, &params->max_speed_cmps)) return false;
        } else if (arg == "--target-accel") {
            if (!parse_double(argc, argv, &i, &params->target_accel_cmps2)) return false;
        } else if (arg == "--target-decel") {
            if (!parse_double(argc, argv, &i, &params->target_decel_cmps2)) return false;
        } else if (arg == "--pwm-slew") {
            if (!parse_double(argc, argv, &i, &params->max_percent_delta_per_s)) return false;
        } else if (arg == "--speed-ff") {
            if (!parse_double(argc, argv, &i, &params->speed_ff)) return false;
        } else if (arg == "--speed-kp") {
            if (!parse_double(argc, argv, &i, &params->wheel_speed_kp)) return false;
        } else if (arg == "--speed-ki") {
            if (!parse_double(argc, argv, &i, &params->wheel_speed_ki)) return false;
        } else if (arg == "--no-flip-motors") {
            params->flip_motors = false;
        } else if (arg == "--swap-motors") {
            params->swap_motors = true;
        } else if (arg == "--swap-encoders") {
            params->swap_encoders = true;
        } else {
            std::fprintf(stderr, "unknown option: %s\n", arg.c_str());
            return false;
        }
    }

    if (options->wheel != "both" && options->phase == "all") {
        std::fprintf(stderr, "single-wheel tests require one selected phase\n");
        return false;
    }
    if (options->coast_seconds < 0.0 ||
        (options->coast_seconds > 0.0 && options->phase != "constant_once" &&
         options->phase != "constant_repeat")) {
        std::fprintf(stderr, "coast recording requires a single constant phase\n");
        return false;
    }
    if (params->max_percent <= 0.0 || params->max_speed_cmps <= 0.0 ||
        params->target_accel_cmps2 <= 0.0 || params->target_decel_cmps2 <= 0.0 ||
        params->max_percent_delta_per_s <= 0.0) {
        std::fprintf(stderr, "speed and PWM limits must be positive\n");
        return false;
    }
    options->control_hz = std::max(20, std::min(200, options->control_hz));
    options->phase_seconds = std::max(1.0, options->phase_seconds);
    options->percent = std::max(0.0, std::min(options->percent, params->max_percent));
    options->target_speed_cmps = std::max(1.0, options->target_speed_cmps);
    options->stall_every = std::max(0, options->stall_every);
    options->stall_ms = std::max(0, options->stall_ms);
    params->control_hz = options->control_hz;
    params->dry_run = !options->enable_motors;
    params->encoder_filter_alpha = std::max(
        0.01, std::min(1.0, params->encoder_filter_alpha));
    params->pwm_frequency_hz = std::max(100.0, params->pwm_frequency_hz);
    return true;
}

struct RawCapture {
    bool valid = false;
    bool atim_valid = false;
    std::uint32_t left_count = 0;
    std::uint32_t right_count = 0;
    std::uint32_t atim_arr = 0;
    std::uint32_t atim_ccr1 = 0;
    std::uint32_t atim_ccr2 = 0;
    double left_unchanged_ms = 0.0;
    double right_unchanged_ms = 0.0;
};

class RawEncoderProbe {
public:
    RawEncoderProbe() {
#if !defined(SMARTCAR_SIM)
        // 与HAL一致按64KiB边界映射，兼容板端大于4KiB的页大小
        constexpr std::uintptr_t kBase = 0x16110000UL;
        constexpr std::size_t kMapSize = 0x10000;
        fd_ = ::open("/dev/mem", O_RDONLY | O_SYNC);
        if (fd_ < 0) {
            std::perror("[PROBE] open /dev/mem");
            return;
        }
        map_ = ::mmap(nullptr, kMapSize, PROT_READ, MAP_SHARED, fd_, kBase);
        if (map_ == MAP_FAILED) {
            std::perror("[PROBE] mmap");
            map_ = nullptr;
        }
#endif
    }

    ~RawEncoderProbe() {
#if !defined(SMARTCAR_SIM)
        if (map_) ::munmap(map_, 0x10000);
        if (fd_ >= 0) ::close(fd_);
#endif
    }

    RawCapture read() {
        RawCapture out;
#if !defined(SMARTCAR_SIM)
        if (!map_) return out;
        constexpr std::size_t kChannelStride = 0x10;
        constexpr std::size_t kFullOffset = 0x08;
        const auto* bytes = static_cast<volatile std::uint8_t*>(map_) + 0xB000;
        const auto* left = reinterpret_cast<volatile const std::uint32_t*>(
            bytes + 3 * kChannelStride + kFullOffset);
        const auto* right = reinterpret_cast<volatile const std::uint32_t*>(
            bytes + 1 * kChannelStride + kFullOffset);
        out.left_count = *left;
        out.right_count = *right;
        out.valid = true;
        // 只读回查ATIM：CH1驱动PIN81，CH2驱动PIN82。
        const auto* atim = static_cast<volatile std::uint8_t*>(map_) + 0x8000;
        out.atim_arr = *reinterpret_cast<volatile const std::uint32_t*>(atim + 0x2C);
        out.atim_ccr1 = *reinterpret_cast<volatile const std::uint32_t*>(atim + 0x34);
        out.atim_ccr2 = *reinterpret_cast<volatile const std::uint32_t*>(atim + 0x38);
        out.atim_valid = out.atim_arr > 0;
#endif
        const auto now = Clock::now();
        update_age(out.left_count, &last_left_, &left_changed_, now,
                   &out.left_unchanged_ms);
        update_age(out.right_count, &last_right_, &right_changed_, now,
                   &out.right_unchanged_ms);
        return out;
    }

private:
    static void update_age(std::uint32_t value,
                           std::uint32_t* previous,
                           Clock::time_point* changed,
                           Clock::time_point now,
                           double* age_ms) {
        if (value != *previous) {
            *previous = value;
            *changed = now;
        }
        *age_ms = std::chrono::duration<double, std::milli>(
            now - *changed).count();
    }

#if !defined(SMARTCAR_SIM)
    int fd_ = -1;
    void* map_ = nullptr;
#endif
    std::uint32_t last_left_ = 0;
    std::uint32_t last_right_ = 0;
    Clock::time_point left_changed_ = Clock::now();
    Clock::time_point right_changed_ = Clock::now();
};

struct PhaseStats {
    std::string name;
    int samples = 0;
    int late_ticks = 0;
    int rpm_drops = 0;
    int pwm_swings = 0;
    int capture_decay_events = 0;
    double rpm_sum = 0.0;
    double rpm_square_sum = 0.0;
    double max_dt_ms = 0.0;
    double max_read_us = 0.0;
    double previous_rpm = 0.0;
    double previous_pwm = 0.0;
    std::uint64_t worker_cycles = 0;
    std::uint64_t worker_deadline_misses = 0;
    double worker_max_dt_ms = 0.0;

    void observe(double elapsed_s, double expected_ms, double dt_ms,
                 double read_us, double rpm, double pwm,
                 const RawCapture& raw) {
        max_dt_ms = std::max(max_dt_ms, dt_ms);
        max_read_us = std::max(max_read_us, read_us);
        if (dt_ms > expected_ms * 1.5) ++late_ticks;
        if (elapsed_s >= 0.25) {
            ++samples;
            rpm_sum += rpm;
            rpm_square_sum += rpm * rpm;
            if (previous_rpm > 10.0 && rpm < previous_rpm * 0.70) ++rpm_drops;
            if (std::abs(pwm - previous_pwm) > 2.0) ++pwm_swings;
            if (raw.valid && previous_rpm > 10.0 &&
                rpm < previous_rpm * 0.85 &&
                std::max(raw.left_unchanged_ms, raw.right_unchanged_ms) >
                    expected_ms * 2.0) {
                ++capture_decay_events;
            }
        }
        previous_rpm = rpm;
        previous_pwm = pwm;
    }

    void print() const {
        const double mean = samples > 0 ? rpm_sum / samples : 0.0;
        const double variance = samples > 0
            ? std::max(0.0, rpm_square_sum / samples - mean * mean) : 0.0;
        const double cv = mean > 1e-6 ? std::sqrt(variance) / mean : 0.0;
        std::printf(
            "[SUMMARY] %-20s mean_rpm=%7.2f cv=%5.3f rpm_drops=%d "
            "pwm_swings=%d capture_decay=%d late=%d max_dt=%6.2fms "
            "max_read=%7.1fus\n",
            name.c_str(), mean, cv, rpm_drops, pwm_swings,
            capture_decay_events, late_ticks, max_dt_ms, max_read_us);
        if (worker_cycles > 0) {
            std::printf(
                "[WORKER] cycles=%llu deadline_misses=%llu max_dt=%.2fms\n",
                static_cast<unsigned long long>(worker_cycles),
                static_cast<unsigned long long>(worker_deadline_misses),
                worker_max_dt_ms);
        }
    }
};

struct WorkerCsvSample {
    std::uint64_t sequence = 0;
    std::uint64_t target_sequence = 0;
    std::uint64_t deadline_misses = 0;
    double dt_ms = 0.0;
    double max_dt_ms = 0.0;
    double lateness_ms = 0.0;
    double target_age_ms = 0.0;
    double control_dt_ms = 0.0;
    bool stop_latched = false;
};

class CsvLog {
public:
    bool open(const std::string& path) {
        stream_.open(path);
        if (!stream_) return false;
        stream_ << "variant,phase,t_s,dt_ms,lateness_ms,apply_us,read_us,"
                   "left_command_pct,right_command_pct,left_rpm,right_rpm,"
                   "left_speed_cmps,right_speed_cmps,left_valid,right_valid,"
                   "capture_probe_valid,left_full_count,right_full_count,"
                   "left_count_unchanged_ms,"
                   "right_count_unchanged_ms,atim_valid,atim_arr,atim_ccr1,"
                   "atim_ccr2,atim_ccr1_pct,atim_ccr2_pct,worker_sequence,"
                   "worker_target_sequence,worker_dt_ms,worker_max_dt_ms,"
                   "worker_deadline_misses,worker_lateness_ms,"
                   "worker_target_age_ms,worker_stop_latched,"
                   "worker_control_dt_ms\n";
        stream_ << std::fixed << std::setprecision(6);
        return true;
    }

    void write(const std::string& phase, double elapsed_s, double dt_ms,
               double lateness_ms, double apply_us, double read_us,
               const ControlCommand& command, const MotorFeedbackLite& feedback,
               const RawCapture& raw,
               const WorkerCsvSample& worker = WorkerCsvSample{}) {
        stream_ << MOTOR_TEST_VARIANT << ',' << phase << ',' << elapsed_s << ','
                << dt_ms << ',' << lateness_ms << ',' << apply_us << ','
                << read_us << ',' << command.left_percent << ','
                << command.right_percent << ',' << feedback.left_rpm << ','
                << feedback.right_rpm << ',' << feedback.left_speed_cmps << ','
                << feedback.right_speed_cmps << ',' << (feedback.left_valid ? 1 : 0)
                << ',' << (feedback.right_valid ? 1 : 0) << ','
                << (raw.valid ? 1 : 0) << ',' << raw.left_count << ','
                << raw.right_count << ','
                << raw.left_unchanged_ms << ',' << raw.right_unchanged_ms << ','
                << (raw.atim_valid ? 1 : 0) << ',' << raw.atim_arr << ','
                << raw.atim_ccr1 << ',' << raw.atim_ccr2 << ','
                << (raw.atim_valid ? 100.0 * raw.atim_ccr1 / (raw.atim_arr + 1.0) : 0.0)
                << ','
                << (raw.atim_valid ? 100.0 * raw.atim_ccr2 / (raw.atim_arr + 1.0) : 0.0)
                << ',' << worker.sequence << ',' << worker.target_sequence
                << ',' << worker.dt_ms << ',' << worker.max_dt_ms
                << ',' << worker.deadline_misses << ',' << worker.lateness_ms
                << ',' << worker.target_age_ms << ','
                << (worker.stop_latched ? 1 : 0)
                << ',' << worker.control_dt_ms
                << '\n';
    }

private:
    std::ofstream stream_;
};

void inject_stall(const Options& options, int tick) {
    if (options.stall_every > 0 && options.stall_ms > 0 && tick > 0 &&
        tick % options.stall_every == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(options.stall_ms));
    }
}

PhaseStats run_constant_phase(MotorAdapter* adapter,
                              RawEncoderProbe* probe,
                              CsvLog* csv,
                              const Options& options,
                              const char* phase,
                              bool repeat_apply) {
    PhaseStats stats;
    stats.name = phase;
    const ControlCommand command{
        options.wheel == "right" ? 0.0 : options.percent,
        options.wheel == "left" ? 0.0 : options.percent};
    if (!repeat_apply) adapter->apply(command);
    const double expected_ms = 1000.0 / options.control_hz;
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / options.control_hz));
    const auto start = Clock::now();
    auto previous = start;
    auto next = start;
    int tick = 0;
    while (g_running.load()) {
        const auto now = Clock::now();
        const double elapsed_s = std::chrono::duration<double>(now - start).count();
        if (elapsed_s >= options.phase_seconds) break;
        const double dt_s = std::max(
            0.001, std::chrono::duration<double>(now - previous).count());
        previous = now;
        const double lateness_ms = std::max(
            0.0, std::chrono::duration<double, std::milli>(now - next).count());

        double apply_us = 0.0;
        if (repeat_apply) {
            const auto begin = Clock::now();
            adapter->apply(command);
            apply_us = std::chrono::duration<double, std::micro>(
                Clock::now() - begin).count();
        }
        // 在HAL消费FULL前读取，只读探针不清除捕获缓冲
        const RawCapture raw = probe->read();
        const auto read_begin = Clock::now();
        const MotorFeedbackLite feedback = adapter->read(dt_s);
        const double read_us = std::chrono::duration<double, std::micro>(
            Clock::now() - read_begin).count();
        csv->write(phase, elapsed_s, dt_s * 1000.0, lateness_ms,
                   apply_us, read_us, command, feedback, raw);
        stats.observe(elapsed_s, expected_ms, dt_s * 1000.0, read_us,
                      options.wheel == "left" ? std::abs(feedback.left_rpm) :
                      options.wheel == "right" ? std::abs(feedback.right_rpm) :
                      0.5 * (std::abs(feedback.left_rpm) + std::abs(feedback.right_rpm)),
                      options.percent, raw);

        ++tick;
        inject_stall(options, tick);
        next += period;
        std::this_thread::sleep_until(next);
    }
    adapter->apply(ControlCommand{});
    return stats;
}

void record_coast(MotorAdapter* adapter, RawEncoderProbe* probe,
                  CsvLog* csv, const Options& options) {
    if (options.coast_seconds <= 0.0) return;
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / options.control_hz));
    const auto start = Clock::now();
    auto previous = start;
    auto next = start;
    while (g_running.load()) {
        const auto now = Clock::now();
        const double elapsed_s = std::chrono::duration<double>(now - start).count();
        if (elapsed_s >= options.coast_seconds) break;
        const double dt_s = std::max(
            0.001, std::chrono::duration<double>(now - previous).count());
        previous = now;
        const double lateness_ms = std::max(
            0.0, std::chrono::duration<double, std::milli>(now - next).count());
        const RawCapture raw = probe->read();
        const auto read_begin = Clock::now();
        const MotorFeedbackLite feedback = adapter->read(dt_s);
        const double read_us = std::chrono::duration<double, std::micro>(
            Clock::now() - read_begin).count();
        csv->write("coast", elapsed_s, dt_s * 1000.0, lateness_ms,
                   0.0, read_us, ControlCommand{}, feedback, raw);
        next += period;
        std::this_thread::sleep_until(next);
    }
}

#if !defined(MOTOR_TEST_USE_WORKER)
ControlCommand selected_command(ControlCommand command, const Options& options) {
    if (options.wheel == "left") command.right_percent = 0.0;
    if (options.wheel == "right") command.left_percent = 0.0;
    return command;
}
#endif

double selected_rpm(const MotorFeedbackLite& feedback, const Options& options) {
    if (options.wheel == "left") return std::abs(feedback.left_rpm);
    if (options.wheel == "right") return std::abs(feedback.right_rpm);
    return 0.5 * (std::abs(feedback.left_rpm) + std::abs(feedback.right_rpm));
}

double selected_pwm(const ControlCommand& command, const Options& options) {
    if (options.wheel == "left") return command.left_percent;
    if (options.wheel == "right") return command.right_percent;
    return 0.5 * (command.left_percent + command.right_percent);
}

#if !defined(MOTOR_TEST_USE_WORKER)
PhaseStats run_closed_loop_direct(MotorAdapter* adapter,
                                  RawEncoderProbe* probe,
                                  CsvLog* csv,
                                  const Options& options,
                                  const PathParams& params) {
    PhaseStats stats;
    stats.name = "closed_loop_app";
    MotionController motion(params);
    NavigationCommand navigation;
    navigation.target_speed_cmps = options.target_speed_cmps;
    navigation.line_good = true;
    ImuFeedback imu;
    imu.valid = !options.imu_invalid;
    const double expected_ms = 1000.0 / options.control_hz;
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / options.control_hz));
    const auto start = Clock::now();
    auto previous = start;
    auto next = start;
    int tick = 0;
    while (g_running.load()) {
        const auto now = Clock::now();
        const double elapsed_s = std::chrono::duration<double>(now - start).count();
        if (elapsed_s >= options.phase_seconds) break;
        const double dt_s = std::max(
            0.001, std::chrono::duration<double>(now - previous).count());
        previous = now;
        const double lateness_ms = std::max(
            0.0, std::chrono::duration<double, std::milli>(now - next).count());
        const auto read_begin = Clock::now();
        const MotorFeedbackLite feedback = adapter->read(dt_s);
        const double read_us = std::chrono::duration<double, std::micro>(
            Clock::now() - read_begin).count();
        const ControlCommand command = selected_command(
            motion.update(navigation, feedback, imu, dt_s).command, options);
        const auto apply_begin = Clock::now();
        adapter->apply(command);
        const double apply_us = std::chrono::duration<double, std::micro>(
            Clock::now() - apply_begin).count();
        const RawCapture raw = probe->read();
        csv->write(stats.name, elapsed_s, dt_s * 1000.0, lateness_ms,
                   apply_us, read_us, command, feedback, raw);
        stats.observe(elapsed_s, expected_ms, dt_s * 1000.0, read_us,
                      selected_rpm(feedback, options),
                      selected_pwm(command, options), raw);
        ++tick;
        inject_stall(options, tick);
        next += period;
        std::this_thread::sleep_until(next);
    }
    adapter->apply(ControlCommand{});
    return stats;
}
#endif

#if defined(MOTOR_TEST_USE_WORKER)
PhaseStats run_closed_loop_worker(RawEncoderProbe* probe,
                                  CsvLog* csv,
                                  const Options& options,
                                  const PathParams& params) {
    PhaseStats stats;
    stats.name = "closed_loop_app";
    ImuFeedback imu;
    imu.valid = !options.imu_invalid;
    const auto selection = options.wheel == "left"
        ? rewrite_path::MotorOutputSelection::LeftOnly
        : (options.wheel == "right"
            ? rewrite_path::MotorOutputSelection::RightOnly
            : rewrite_path::MotorOutputSelection::Both);
    rewrite_path::MotorIoWorker worker(params, [imu] { return imu; }, selection);
    if (!worker.start()) {
        std::fprintf(stderr, "MotorIoWorker initialization failed\n");
        return stats;
    }
    NavigationCommand navigation;
    navigation.target_speed_cmps = options.target_speed_cmps;
    navigation.line_good = true;
    const double expected_ms = 1000.0 / options.control_hz;
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / options.control_hz));
    const auto start = Clock::now();
    auto previous = start;
    auto next = start;
    int tick = 0;
    while (g_running.load()) {
        const auto now = Clock::now();
        const double elapsed_s = std::chrono::duration<double>(now - start).count();
        if (elapsed_s >= options.phase_seconds) break;
        const double dt_s = std::max(
            0.001, std::chrono::duration<double>(now - previous).count());
        previous = now;
        const double lateness_ms = std::max(
            0.0, std::chrono::duration<double, std::milli>(now - next).count());
        const auto submit_begin = Clock::now();
        worker.submit(navigation, now);
        const double submit_us = std::chrono::duration<double, std::micro>(
            Clock::now() - submit_begin).count();
        const auto execution = worker.latest();
        const MotorFeedbackLite& feedback = execution.feedback;
        const ControlCommand& command = execution.control.command;
        WorkerCsvSample worker_sample;
        worker_sample.sequence = execution.loop.sequence;
        worker_sample.target_sequence = execution.loop.target_sequence;
        worker_sample.deadline_misses = execution.loop.deadline_misses;
        worker_sample.dt_ms = execution.loop.dt_s * 1000.0;
        worker_sample.max_dt_ms = execution.loop.max_dt_s * 1000.0;
        worker_sample.lateness_ms = execution.loop.lateness_s * 1000.0;
        worker_sample.target_age_ms = execution.loop.target_age_s * 1000.0;
        worker_sample.stop_latched = execution.loop.stop_latched;
        worker_sample.control_dt_ms = execution.loop.control_dt_s * 1000.0;
        stats.worker_cycles = execution.loop.sequence;
        stats.worker_deadline_misses = execution.loop.deadline_misses;
        stats.worker_max_dt_ms = worker_sample.max_dt_ms;
        const RawCapture raw = probe->read();
        csv->write(stats.name, elapsed_s, dt_s * 1000.0, lateness_ms,
                   submit_us, -1.0, command, feedback, raw, worker_sample);
        stats.observe(elapsed_s, expected_ms, dt_s * 1000.0, -1.0,
                      selected_rpm(feedback, options),
                      selected_pwm(command, options), raw);
        ++tick;
        inject_stall(options, tick);
        next += period;
        std::this_thread::sleep_until(next);
    }
    worker.stop();
    return stats;
}
#endif

void idle_pause() {
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
}

}  // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    std::signal(SIGHUP, on_signal);

    Options options;
    PathParams params;
    if (!parse_args(argc, argv, &options, &params)) {
        print_usage(argv[0]);
        return 2;
    }
    if (options.enable_motors && !options.confirmed_wheels_lifted) {
        std::fprintf(stderr,
                     "Refusing to arm motors: lift the wheels and pass "
                     "--confirm-wheels-lifted.\n");
        return 2;
    }

    CsvLog csv;
    if (!csv.open(options.output)) {
        std::fprintf(stderr, "cannot open CSV: %s\n", options.output.c_str());
        return 2;
    }
    std::printf("[TEST] variant=%s motors=%s phase=%.1fs percent=%.1f "
                "target=%.1fcm/s hz=%d csv=%s\n",
                MOTOR_TEST_VARIANT,
                options.enable_motors ? "ARMED" : "dry-run",
                options.phase_seconds, options.percent,
                options.target_speed_cmps, options.control_hz,
                options.output.c_str());
    std::printf("[TEST] wheel=%s pwm_frequency=%.1fHz swap_motors=%d swap_encoders=%d\n",
                options.wheel.c_str(), params.pwm_frequency_hz,
                params.swap_motors, params.swap_encoders);
    std::printf("[TEST] imu_valid=%d\n", !options.imu_invalid);
    std::printf("[TEST] max_speed=%.1fcm/s max_pwm=%.1f%% accel=%.1fcm/s2 "
                "decel=%.1fcm/s2 pwm_slew=%.1f%%/s ff=%.3f kp=%.3f ki=%.3f "
                "encoder_ratio=%.6f encoder_alpha=%.3f\n",
                params.max_speed_cmps, params.max_percent,
                params.target_accel_cmps2, params.target_decel_cmps2,
                params.max_percent_delta_per_s, params.speed_ff,
                params.wheel_speed_kp, params.wheel_speed_ki,
                params.encoder_gear_ratio, params.encoder_filter_alpha);

    RawEncoderProbe probe;
    PhaseStats once;
    PhaseStats repeat;
    if (options.phase != "closed_loop_app") {
        MotorAdapter adapter(params);
        if (!adapter.init()) {
            std::fprintf(stderr, "motor adapter initialization failed\n");
            return 1;
        }
        if (options.phase == "all" || options.phase == "constant_once") {
            std::printf("[PHASE] constant_once: write PWM once, then only read encoders\n");
            once = run_constant_phase(
                &adapter, &probe, &csv, options, "constant_once", false);
            record_coast(&adapter, &probe, &csv, options);
        }
        if (options.phase == "all") idle_pause();
        if (g_running.load() &&
            (options.phase == "all" || options.phase == "constant_repeat")) {
            std::printf("[PHASE] constant_repeat: rewrite the same PWM every tick\n");
            repeat = run_constant_phase(
                &adapter, &probe, &csv, options, "constant_repeat", true);
            record_coast(&adapter, &probe, &csv, options);
        }
        adapter.stop();
    }

    if (options.phase == "all") idle_pause();
    PhaseStats closed;
    if (g_running.load() &&
        (options.phase == "all" || options.phase == "closed_loop_app")) {
        std::printf("[PHASE] closed_loop_app: actual %s control architecture\n",
                    MOTOR_TEST_VARIANT);
#if defined(MOTOR_TEST_USE_WORKER)
        closed = run_closed_loop_worker(&probe, &csv, options, params);
#else
        MotorAdapter adapter(params);
        if (!adapter.init()) {
            std::fprintf(stderr, "motor adapter reinitialization failed\n");
            return 1;
        }
        closed = run_closed_loop_direct(&adapter, &probe, &csv, options, params);
        adapter.stop();
#endif
    }

    if (!once.name.empty()) once.print();
    if (!repeat.name.empty()) repeat.print();
    if (!closed.name.empty()) closed.print();
    std::printf(
        "[HINT] constant_once stutters => PWM/electrical/mechanical; "
        "repeat-only => PWM write path; closed-loop-only with pwm_swings => "
        "encoder/PI; capture_decay => inspect capture stale detection.\n");
    std::printf("[DONE] CSV written to %s\n", options.output.c_str());
    return g_running.load() ? 0 : 130;
}


