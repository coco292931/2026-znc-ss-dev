#include "http_streamer.hpp"
#include "imu_feedback.hpp"
#include "inertial_navigation.hpp"
#include "motion_control.hpp"
#include "motor_adapter.hpp"
#include "odometry.hpp"
#include "path_controller.hpp"
#include "status_display.hpp"
#include "target_recognizer.hpp"
#include "tof_slope_sensor.hpp"
#include "vision_pipeline.hpp"

#ifndef PATH_FOLLOW_NO_OPENCV

#include "LQ_HW_GPIO.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <thread>
#include <utility>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

using namespace rewrite_path;

namespace {

std::atomic<bool> g_running{true};
constexpr double kRampAssistFadeYawRateDps = 25.0;
constexpr double kDegreesPerRadian = 57.2957795130823208768;
// Upper bound for the control timestep. A stalled camera/vision frame produces
// a large real dt; instead of masking it as a fixed 0.02 (which makes the rate
// limiters crawl on recovery and produce a stop-then-lurch stutter), clamp it
// to a realistic ceiling so limiters advance with the true elapsed time.
constexpr double kMaxControlDt = 0.05;

void on_signal(int) {
    g_running = false;
}

class Sw2ForceStopMonitor {
public:
    Sw2ForceStopMonitor() : gpio_(44, GPIO_Mode_In) {}

    ~Sw2ForceStopMonitor() {
        stop();
    }

    void start(MotorAdapter* motors) {
        if (motors == nullptr || running_.exchange(true)) return;
        worker_ = std::thread([this, motors] { loop(motors); });
    }

    void stop() {
        running_.store(false);
        if (worker_.joinable()) worker_.join();
    }

private:
    void loop(MotorAdapter* motors) {
        constexpr auto kDebounce = std::chrono::milliseconds(25);
        constexpr auto kReleaseQualification =
            std::chrono::milliseconds(100);
        auto now = std::chrono::steady_clock::now();
        bool candidate = gpio_.GetGpioValue();
        bool stable = candidate;
        bool armed = false;
        auto candidate_since = now;
        auto released_since = now;

        while (running_.load() && g_running.load()) {
            now = std::chrono::steady_clock::now();
            const bool active = gpio_.GetGpioValue();
            if (active != candidate) {
                candidate = active;
                candidate_since = now;
            }
            if (candidate != stable && now - candidate_since >= kDebounce) {
                stable = candidate;
                if (!stable) {
                    released_since = now;
                } else if (armed) {
                    std::fprintf(
                        stderr,
                        "[SAFETY] SW2 force stop: zeroing motors and "
                        "returning to menu\n");
                    motors->emergency_stop();
                    g_running.store(false);
                    running_.store(false);
                    break;
                }
            }
            if (!stable && !armed &&
                now - released_since >= kReleaseQualification) {
                armed = true;
                std::printf("[SAFETY] SW2 force stop armed\n");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    HWGpio gpio_;
    std::atomic<bool> running_{false};
    std::thread worker_;
};

cv::VideoCapture open_camera(const PathParams& p) {
    cv::VideoCapture cap(p.camera_device, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        cap.open(p.camera_device);
    }
    if (!cap.isOpened()) {
        return cap;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, p.camera_width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, p.camera_height);
    cap.set(cv::CAP_PROP_FPS, p.camera_fps);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    return cap;
}

class AsyncCameraReader {
public:
    AsyncCameraReader(cv::VideoCapture* capture, bool rotate_180)
        : capture_(capture), rotate_180_(rotate_180) {}

    ~AsyncCameraReader() {
        stop();
    }

    bool start() {
        if (capture_ == nullptr || !capture_->isOpened()) return false;
        if (running_.exchange(true)) return true;
        worker_ = std::thread([this] { capture_loop(); });
        return true;
    }

    void stop() {
        if (!running_.exchange(false)) return;
        if (worker_.joinable()) worker_.join();
    }

    bool read_new(std::uint64_t* consumed_sequence, cv::Mat* frame) {
        if (consumed_sequence == nullptr || frame == nullptr) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        if (sequence_ == 0 || sequence_ == *consumed_sequence) return false;
        *frame = latest_;
        *consumed_sequence = sequence_;
        return !frame->empty();
    }

private:
    void capture_loop() {
        while (running_.load() && g_running.load()) {
            cv::Mat frame;
            if (!capture_->read(frame) || frame.empty()) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(5));
                continue;
            }
            if (rotate_180_) {
                cv::rotate(frame, frame, cv::ROTATE_180);
            }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                latest_ = std::move(frame);
                ++sequence_;
            }
        }
    }

    cv::VideoCapture* capture_ = nullptr;
    bool rotate_180_ = false;
    std::atomic<bool> running_{false};
    std::thread worker_;
    std::mutex mutex_;
    cv::Mat latest_;
    std::uint64_t sequence_ = 0;
};

StepInput build_step_input(const RoadEstimateLite& road,
                           const MotorFeedbackLite& motor,
                           const TargetObservation& target) {
    StepInput input;
    input.line_error = road.line_error;
    input.far_error = road.far_error;
    input.vehicle_center_error = road.vehicle_center_error;
    input.line_confidence = road.line_confidence;
    input.line_lost = road.line_lost;
    input.bottom_pair_valid = road.bottom_pair_valid;
    input.cross = road.elements.cross;
    input.zebra = road.elements.zebra;
    input.two_side_stable = road.elements.two_side_stable;
    input.roundabout = road.elements.roundabout;
    input.side_open = road.elements.side_open;
    input.roundabout_stage = road.elements.roundabout_stage;
    input.distance_cm = motor.distance_cm;
    input.distance_valid = motor.distance_valid;
    input.target_valid = target.valid;
    input.target_kind = target.kind;
    input.target_confidence = target.confidence;
    input.target_size = target.size;
    input.target_center_y_ratio = target.center_y_ratio;
    return input;
}

bool is_target_action(DriveState state) {
    return state == DriveState::BypassLeft ||
        state == DriveState::BypassRight ||
        state == DriveState::StraightOver;
}

bool validate_target_inertial_routes(
    const PathParams& params,
    InertialPathNavigator* navigator) {
    if (!navigator) return false;
    const std::array<std::pair<const char*, const std::string*>, 8> routes = {{
        {"straight-left", &params.target_straight_left_path},
        {"straight-right", &params.target_straight_right_path},
        {"cross-1-left", &params.target_cross_1_left_path},
        {"cross-1-right", &params.target_cross_1_right_path},
        {"cross-2-left", &params.target_cross_2_left_path},
        {"cross-2-right", &params.target_cross_2_right_path},
        {"chicane-left", &params.target_chicane_left_path},
        {"chicane-right", &params.target_chicane_right_path},
    }};
    for (const auto& route : routes) {
        if (route.second->empty()) continue;
        if (!navigator->load(*route.second)) {
            std::fprintf(
                stderr,
                "failed to load target inertial route %s (%s): %s\n",
                route.first,
                route.second->c_str(),
                navigator->last_error().c_str());
            return false;
        }
        std::printf(
            "[TARGET-NAV] validated %s: %s (%.1fcm, %zu points)\n",
            route.first,
            route.second->c_str(),
            navigator->status().path_length_cm,
            navigator->status().waypoint_count);
    }
    navigator->reset();
    return true;
}

class ImuCsvWriter {
public:
    ~ImuCsvWriter() {
        if (file_) {
            std::fflush(file_);
            std::fclose(file_);
        }
    }

    bool open(const std::string& path) {
        if (path.empty()) return true;
        file_ = std::fopen(path.c_str(), "w");
        if (!file_) return false;
        std::fprintf(file_,
            "# rewrite direct IMU velocity/path v1\n"
            "# frame: +X recording-start forward, +Y recording-start right\n"
            "elapsed_s,route_distance_cm,x_cm,y_cm,heading_deg,speed_cmps,"
            "velocity_x_cmps,velocity_y_cmps,raw_accel_x_g,raw_accel_y_g,"
            "raw_accel_z_g,forward_accel_mps2,"
            "right_accel_mps2,imu_stationary\n");
        return true;
    }

    void write(double elapsed_s, const ImuFeedback& imu) {
        if (!file_ || !imu.valid) return;
        if (!origin_valid_) {
            origin_valid_ = true;
            origin_time_s_ = elapsed_s;
            origin_heading_deg_ = imu.heading_deg;
            origin_x_m_ = imu.position_x_m;
            origin_y_m_ = imu.position_y_m;
        }

        const double angle = origin_heading_deg_ / kDegreesPerRadian;
        const double cosine = std::cos(angle);
        const double sine = std::sin(angle);
        const double dx_m = imu.position_x_m - origin_x_m_;
        const double dy_m = imu.position_y_m - origin_y_m_;
        const double x_cm = 100.0 * (cosine * dx_m + sine * dy_m);
        const double y_cm = 100.0 * (-sine * dx_m + cosine * dy_m);
        const double velocity_x_cmps = 100.0 *
            (cosine * imu.velocity_x_mps + sine * imu.velocity_y_mps);
        const double velocity_y_cmps = 100.0 *
            (-sine * imu.velocity_x_mps + cosine * imu.velocity_y_mps);
        if (row_count_ > 0) {
            route_distance_cm_ +=
                std::hypot(x_cm - previous_x_cm_, y_cm - previous_y_cm_);
        }
        previous_x_cm_ = x_cm;
        previous_y_cm_ = y_cm;
        std::fprintf(
            file_,
            "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
            "%.6f,%.6f,%.6f,%d\n",
            elapsed_s - origin_time_s_,
            route_distance_cm_,
            x_cm,
            y_cm,
            wrap_degrees(imu.heading_deg - origin_heading_deg_),
            std::hypot(velocity_x_cmps, velocity_y_cmps),
            velocity_x_cmps,
            velocity_y_cmps,
            imu.raw_accel_x_g,
            imu.raw_accel_y_g,
            imu.raw_accel_z_g,
            imu.forward_accel_mps2,
            imu.right_accel_mps2,
            imu.stationary ? 1 : 0);
        if (++row_count_ % 20 == 0) std::fflush(file_);
    }

private:
    static double wrap_degrees(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    FILE* file_ = nullptr;
    bool origin_valid_ = false;
    double origin_time_s_ = 0.0;
    double origin_heading_deg_ = 0.0;
    double origin_x_m_ = 0.0;
    double origin_y_m_ = 0.0;
    double previous_x_cm_ = 0.0;
    double previous_y_cm_ = 0.0;
    double route_distance_cm_ = 0.0;
    int row_count_ = 0;
};

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
#ifdef SIGPIPE
    std::signal(SIGPIPE, SIG_IGN);
#endif

    PathParams params;
    params.parse(argc, argv);
    params.print_banner();

    InertialPathNavigator inertial_navigation(params);
    InertialPathNavigator target_inertial_navigation(params);
    InertialTakeoverDirectionGuard inertial_takeover_guard(params);
    const bool inertial_path_enabled = !params.inertial_path.empty();
    const bool target_inertial_enabled =
        params.target_inertial_routes_enabled();
    if (inertial_path_enabled && target_inertial_enabled) {
        std::fprintf(
            stderr,
            "--inertial-path cannot be combined with target inertial routes\n");
        return 2;
    }
    const bool inertial_fallback_enabled =
        inertial_path_enabled && params.line_lost_inertial;
    const bool inertial_only =
        inertial_path_enabled && !inertial_fallback_enabled;
    if (inertial_path_enabled &&
        !inertial_navigation.load(params.inertial_path)) {
        std::fprintf(stderr, "failed to load inertial path: %s\n",
                     inertial_navigation.last_error().c_str());
        return 2;
    }
    if (target_inertial_enabled &&
        !validate_target_inertial_routes(
            params, &target_inertial_navigation)) {
        return 2;
    }

    cv::VideoCapture cap = open_camera(params);
    if (!cap.isOpened() && !inertial_only) {
        std::fprintf(stderr, "failed to open camera: %s\n",
                     params.camera_device.c_str());
        return 1;
    }
    if (!cap.isOpened()) {
        std::fprintf(stderr,
                     "camera unavailable; inertial navigation continues headless\n");
    }

    LegacyVisionPipeline vision(params);
    PathController path(params);
    MotionController motion(params);
    MotorAdapter motors(params);
    if (!motors.init()) {
        std::fprintf(stderr, "motor adapter init failed; continuing dry path loop\n");
    }
    Sw2ForceStopMonitor sw2_force_stop;
    sw2_force_stop.start(&motors);
    Vl53l0xDistanceSensor tof_sensor(params);
    TofSlopeDetector tof_detector(params);
    TofSlopeFeedback tof_feedback;
    const bool tof_started =
        params.enable_tof_slope && tof_sensor.start();
    if (params.enable_tof_slope && !tof_started) {
        std::fprintf(
            stderr,
            "[TOF] unavailable; continuing without ramp boost\n");
    }
    if (tof_started) {
        std::printf("[TOF] calibrating flat-road baseline; keep vehicle still\n");
        auto calibration_previous = std::chrono::steady_clock::now();
        const auto calibration_deadline =
            calibration_previous +
            std::chrono::duration_cast<
                std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(
                    params.tof_baseline_timeout_s));
        while (g_running.load() &&
               std::chrono::steady_clock::now() < calibration_deadline &&
               !tof_feedback.baseline_ready) {
            const auto calibration_now = std::chrono::steady_clock::now();
            const double calibration_dt =
                std::chrono::duration<double>(
                    calibration_now - calibration_previous).count();
            calibration_previous = calibration_now;
            tof_feedback = tof_detector.update(
                tof_sensor.read(), calibration_dt);
            tof_feedback.sensor_started = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (tof_feedback.baseline_ready) {
            std::printf(
                "[TOF] baseline ready: %.0fmm\n",
                tof_feedback.baseline_distance_mm);
        } else {
            std::fprintf(
                stderr,
                "[TOF] baseline not ready after %.1fs; "
                "ramp boost disabled for this run\n",
                params.tof_baseline_timeout_s);
        }
    }
    ImuYawSensor imu(params);
    const bool imu_started = imu.start();
    if (!imu_started) {
        std::fprintf(stderr, "IMU unavailable; encoder/vision degradation active\n");
    }
    PlanarOdometry odometry(params);

    AsyncTargetRecognizer recognizer(params);
    const bool recog_ok =
        params.use_ncnn && params.enable_target_actions && recognizer.init();
    if (recog_ok) recognizer.start();

    HttpMjpegStreamer http(params, &vision);
    if (params.use_http && !http.start()) {
        std::fprintf(stderr, "http streamer failed to start on port %u\n",
                     params.http_port);
    }

    StatusDisplay status_display(params);
    const bool display_started =
        params.enable_display && status_display.start();
    if (params.enable_display && !display_started) {
        std::fprintf(stderr, "status display unavailable; continuing headless\n");
    }

    ImuCsvWriter imu_csv;
    if (!imu_csv.open(params.imu_csv_path)) {
        std::fprintf(stderr, "failed to open IMU CSV: %s\n",
                     params.imu_csv_path.c_str());
        sw2_force_stop.stop();
        motors.stop();
        imu.stop();
        tof_sensor.stop();
        recognizer.stop();
        http.stop();
        status_display.stop();
        return 2;
    }

    auto last = std::chrono::steady_clock::now();
    auto last_vision = last;
    auto next_control_tick = last;
    const auto started_at = last;
    const auto control_period =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(
                1.0 / static_cast<double>(params.control_hz)));
    int control_count = 0;
    int vision_frame_count = 0;
    bool tof_stop_latched = false;
    RunawayStopGuard runaway_guard;
    bool runaway_reported = false;
    bool inertial_fallback_active = false;
    bool target_inertial_active = false;
    bool target_inertial_started = false;
    int target_encounter = 0;
    DriveState previous_visual_state = DriveState::Follow;
    std::string active_target_path;
    RoadEstimateLite road;
    NavigationCommand visual_command;
    std::uint64_t consumed_camera_sequence = 0;
    AsyncCameraReader camera_reader(&cap, params.rotate_180);
    camera_reader.start();
    while (g_running.load()) {
        cv::Mat frame;
        const bool frame_valid = camera_reader.read_new(
            &consumed_camera_sequence, &frame);

        const auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last).count();
        last = now;
        if (dt <= 0.0) {
            dt = 0.02;
        } else if (dt > kMaxControlDt) {
            dt = kMaxControlDt;
        }

        double vision_dt = dt;
        if (frame_valid) {
            vision_dt =
                std::chrono::duration<double>(now - last_vision).count();
            last_vision = now;
            road = vision.process_bgr(frame);
            ++vision_frame_count;
        }
        MotorFeedbackLite feedback = motors.read(dt);
        ImuFeedback imu_feedback = imu.read();
        const double elapsed_s =
            std::chrono::duration<double>(now - started_at).count();
        static double next_imu_csv_s = 0.0;
        if (!params.imu_csv_path.empty() && elapsed_s >= next_imu_csv_s) {
            imu_csv.write(elapsed_s, imu_feedback);
            next_imu_csv_s = elapsed_s + 1.0 / params.imu_csv_hz;
        }
        if (tof_started) {
            tof_feedback = tof_detector.update(tof_sensor.read(), dt);
            tof_feedback.sensor_started = true;
        }
        OdometrySample odometry_sample =
            odometry.update(feedback, imu_feedback);

        ++control_count;
        if (recog_ok && frame_valid && vision_frame_count % 2 == 0) {
            recognizer.submit(frame);
        }

        const TargetObservation target_observation = recognizer.latest();
        if (frame_valid && !inertial_only && !target_inertial_active) {
            StepInput input = build_step_input(
                road, feedback, target_observation);
            input.encoder_heading_deg =
                (feedback.left_distance_cm - feedback.right_distance_cm) /
                std::max(1.0, params.wheel_base_cm) * kDegreesPerRadian;
            input.encoder_heading_valid =
                feedback.left_valid && feedback.right_valid;
            input.heading_deg = imu_feedback.valid
                ? imu_feedback.heading_deg : odometry_sample.heading_deg;
            input.heading_valid = imu_feedback.valid ||
                (feedback.left_valid && feedback.right_valid);
            input.heading_from_imu = imu_feedback.valid;
            input.yaw_rate_dps = imu_feedback.yaw_rate_dps;
            input.yaw_rate_valid = imu_feedback.valid;
            input.tof_valid = tof_feedback.valid;
            input.tof_baseline_ready = tof_feedback.baseline_ready;
            input.ramp_detected = tof_feedback.ramp_detected;
            visual_command = path.update(input, vision_dt);
            const bool entered_target_action =
                target_inertial_enabled &&
                is_target_action(visual_command.state) &&
                !is_target_action(previous_visual_state);
            previous_visual_state = visual_command.state;
            if (entered_target_action) {
                ++target_encounter;
                const std::string route = params.target_inertial_path_for(
                    target_encounter, visual_command.action_kind);
                if (route.empty()) {
                    std::printf(
                        "[TARGET-NAV] target %d %s uses legacy action\n",
                        target_encounter,
                        target_kind_name(visual_command.action_kind));
                } else if (!target_inertial_navigation.load(route)) {
                    std::fprintf(
                        stderr,
                        "[TARGET-NAV] target %d route reload failed (%s): "
                        "%s; using legacy action\n",
                        target_encounter,
                        route.c_str(),
                        target_inertial_navigation.last_error().c_str());
                } else {
                    target_inertial_active = true;
                    target_inertial_started = true;
                    active_target_path = route;
                    std::printf(
                        "[TARGET-NAV] target %d %s -> %s (%.1fcm)\n",
                        target_encounter,
                        target_kind_name(visual_command.action_kind),
                        route.c_str(),
                        target_inertial_navigation.status().path_length_cm);
                }
            }
        }
        if (inertial_fallback_enabled &&
            !inertial_fallback_active &&
            visual_command.state == DriveState::Stopped &&
            visual_command.stop_reason == StopReason::LineLost) {
            inertial_takeover_guard.capture(visual_command);
            const bool reanchored =
                inertial_navigation.reanchor_at_current_progress(
                    odometry_sample, imu_feedback);
            inertial_fallback_active = true;
            std::fprintf(
                stderr,
                "[NAV] visual line lost; inertial path takeover at %.1fcm "
                "with yaw continuity %.1fdps, reanchor=%s\n",
                odometry_sample.distance_cm,
                inertial_takeover_guard.takeover_yaw_rate_dps(),
                reanchored ? "yes" : "no");
        }
        NavigationCommand inertial_command;
        if (inertial_path_enabled) {
            inertial_command = inertial_navigation.update(
                odometry_sample,
                imu_feedback,
                dt,
                inertial_only || inertial_fallback_active);
            if (inertial_fallback_active) {
                inertial_takeover_guard.apply(&inertial_command);
            }
        }
        NavigationCommand target_inertial_command;
        if (target_inertial_active) {
            target_inertial_command = target_inertial_navigation.update(
                odometry_sample, imu_feedback, dt, true);
        }
        NavigationCommand navigation = target_inertial_active
            ? target_inertial_command
            : ((inertial_only || inertial_fallback_active)
                ? inertial_command : visual_command);
        if (target_inertial_active &&
            target_inertial_navigation.status().state ==
                InertialNavState::Complete) {
            std::printf(
                "[TARGET-NAV] target %d route complete: %s\n",
                target_encounter,
                active_target_path.c_str());
            target_inertial_active = false;
            active_target_path.clear();
            path.finish_external_target_action();
            previous_visual_state = DriveState::Follow;
        }
        const bool was_tof_stop_latched = tof_stop_latched;
        if (apply_tof_stop_latch(
                params, tof_feedback, &tof_stop_latched, &navigation)) {
            if (!was_tof_stop_latched &&
                navigation.stop_reason == StopReason::RampDetected) {
                std::fprintf(
                    stderr,
                    "[TOF-STOP-TEST] ramp detected: distance=%.0fmm "
                    "baseline=%.0fmm delta=%.0fmm; motors stopped\n",
                    tof_feedback.filtered_distance_mm,
                    tof_feedback.baseline_distance_mm,
                    tof_feedback.signed_delta_mm);
            }
        } else if (tof_started &&
            tof_feedback.baseline_ready &&
            tof_feedback.ramp_detected &&
            uses_follow_control(navigation.state) &&
            navigation.line_good) {
            const double ramp_straightness = clamp_value(
                1.0 - std::abs(navigation.target_yaw_rate_dps) /
                    kRampAssistFadeYawRateDps,
                0.0,
                1.0);
            const double ramp_speed = std::max(
                navigation.target_speed_cmps,
                params.ramp_boost_speed_cmps);
            const double ramp_speed_blend =
                0.25 + 0.75 * ramp_straightness;
            navigation.target_speed_cmps +=
                (ramp_speed - navigation.target_speed_cmps) *
                ramp_speed_blend;
            navigation.power_boost_percent =
                params.ramp_boost_percent * ramp_straightness;
        }
        runaway_guard.update(feedback, imu_feedback, navigation);
        if (runaway_guard.apply(&navigation) && !runaway_reported) {
            runaway_reported = true;
            std::fprintf(
                stderr,
                "[SAFETY] runaway stopped: wheel=%.1f/%.1fcm/s "
                "yaw=%.1fdps; restart required\n",
                feedback.left_speed_cmps,
                feedback.right_speed_cmps,
                imu_feedback.yaw_rate_dps);
        }
        ControlResult control =
            motion.update(navigation, feedback, imu_feedback, dt);
        motors.apply(control.command);

        if (params.use_http || display_started) {
            TelemetrySample telemetry;
            telemetry.elapsed_s = elapsed_s;
            telemetry.recognition_valid = target_observation.valid;
            telemetry.recognition_kind = target_observation.kind;
            telemetry.recognition_confidence =
                target_observation.confidence;
            telemetry.recognition_size = target_observation.size;
            telemetry.recognition_center_y_ratio =
                target_observation.center_y_ratio;
            telemetry.target_encounter = target_encounter;
            telemetry.target_route_active = target_inertial_active;
            telemetry.road = road;
            telemetry.navigation = navigation;
            telemetry.control = control;
            telemetry.motor = feedback;
            telemetry.imu = imu_feedback;
            telemetry.tof = tof_feedback;
            telemetry.odometry = odometry_sample;
            telemetry.inertial_navigation = target_inertial_started
                ? target_inertial_navigation.status()
                : inertial_navigation.status();
            telemetry.target_recognition_enabled = recog_ok;
            telemetry.target_recognition_valid = target_observation.valid;
            telemetry.target_recognition_kind = target_observation.kind;
            telemetry.target_recognition_confidence =
                target_observation.confidence;
            telemetry.target_recognition_size = target_observation.size;
            telemetry.target_recognition_center_y_ratio =
                target_observation.center_y_ratio;
            telemetry.target_recognition_x = target_observation.x;
            telemetry.target_recognition_y = target_observation.y;
            telemetry.target_recognition_w = target_observation.w;
            telemetry.target_recognition_h = target_observation.h;
            if (params.use_http) {
                if (frame_valid) {
                    http.publish(
                        frame, road, navigation, vision.debug_frame());
                }
                http.publish_telemetry(telemetry);
            }
            if (display_started) {
                status_display.publish(telemetry);
            }
        }

        if (control_count % params.control_hz == 0) {
            std::printf("[%s/%s/%s] err=%.2f far=%.2f conf=%.2f "
                        "br=%d/%d rec=%d/%d "
                        "zb=%d/%d/%s "
                        "v=%.1f yaw=%.1f L=%.1f R=%.1f "
                        "tof=%.0f/%s boost=%.0f%% clients=%d\n",
                        drive_state_name(navigation.state),
                        sensor_mode_name(control.diagnostics.sensor_mode),
                        roundabout_stage_name(road.elements.roundabout_stage),
                        road.line_error,
                        road.far_error,
                        road.line_confidence,
                        road.elements.left_branch_count,
                        road.elements.right_branch_count,
                        road.elements.left_recovery_count,
                        road.elements.right_recovery_count,
                        road.elements.zebra ? 1 : 0,
                        navigation.zebra_encounter_count,
                        navigation.zebra_active ? "ON" : "off",
                        control.diagnostics.limited_speed_cmps,
                        control.diagnostics.measured_yaw_rate_dps,
                        control.command.left_percent,
                        control.command.right_percent,
                        tof_feedback.filtered_distance_mm,
                        tof_feedback.ramp_detected ? "RAMP" : "flat",
                        control.diagnostics.power_boost_percent,
                        http.active_clients());
        }

        next_control_tick += control_period;
        const auto control_finished = std::chrono::steady_clock::now();
        if (next_control_tick > control_finished) {
            std::this_thread::sleep_until(next_control_tick);
        } else {
            // Do not execute a burst of back-to-back catch-up iterations after
            // an unusually expensive vision frame.
            next_control_tick = control_finished;
        }
    }

    sw2_force_stop.stop();
    camera_reader.stop();
    motors.stop();
    imu.stop();
    tof_sensor.stop();
    recognizer.stop();
    http.stop();
    status_display.stop();
    return 0;
}

#else

#include <cstdio>

int main() {
    std::fprintf(stderr, "lq_path_follow requires OpenCV. Build path_selftest for host checks.\n");
    return 2;
}

#endif
