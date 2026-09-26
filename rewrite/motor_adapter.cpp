#include "motor_adapter.hpp"

#include <cmath>

namespace rewrite_path {

MotorAdapter::MotorAdapter(const PathParams& params) : p_(params) {}

MotorAdapter::~MotorAdapter() {
    stop();
}

bool MotorAdapter::init() {
    std::lock_guard<std::mutex> lock(mutex_);
    emergency_stopped_ = false;
#ifndef PATH_FOLLOW_NO_HW
    smartcar::HardwareConfig cfg;
    cfg.max_motor_percent = p_.max_percent;
    cfg.pwm_frequency_hz = p_.pwm_frequency_hz;
    cfg.encoder_lines = p_.encoder_lines;
    cfg.encoder_gear_ratio = p_.encoder_gear_ratio;
    cfg.encoder_filter_alpha = p_.encoder_filter_alpha;
    if (p_.flip_motors) {
        cfg.left_motor_reversed = !cfg.left_motor_reversed;
        cfg.right_motor_reversed = !cfg.right_motor_reversed;
    }
    hardware_ = smartcar::make_vehicle_hardware(cfg, p_.dry_run, !p_.dry_run);
    return hardware_ && hardware_->start();
#else
    return true;
#endif
}

MotorFeedbackLite MotorAdapter::read(double dt) {
    std::lock_guard<std::mutex> lock(mutex_);
    MotorFeedbackLite out;
    dt = clamp_value(dt, 0.001, 0.10);
    bool hardware_valid = false;
#ifndef PATH_FOLLOW_NO_HW
    if (hardware_) {
        smartcar::WheelFeedback fb = hardware_->read_feedback(dt);
        const bool swap_feedback = p_.swap_motors != p_.swap_encoders;
        out.left_rpm = swap_feedback ? fb.right_rpm : fb.left_rpm;
        out.right_rpm = swap_feedback ? fb.left_rpm : fb.right_rpm;
        hardware_valid = fb.valid && !p_.dry_run;
    }
#endif

    auto update_health = [&](double command,
                             double rpm,
                             double* zero_time) {
        const bool expected_motion =
            std::abs(command) >= p_.min_move_percent + 0.5;
        if (hardware_valid && expected_motion &&
            std::abs(rpm) < p_.encoder_recover_rpm) {
            *zero_time += dt;
        } else {
            *zero_time = 0.0;
        }
        return hardware_valid &&
            *zero_time < p_.encoder_zero_timeout_s;
    };

    out.left_valid =
        update_health(last_left_pct_, out.left_rpm, &left_zero_s_);
    out.right_valid =
        update_health(last_right_pct_, out.right_rpm, &right_zero_s_);
    const double measured_left =
        rpm_to_cmps(out.left_rpm, p_.wheel_diameter_cm);
    const double measured_right =
        rpm_to_cmps(out.right_rpm, p_.wheel_diameter_cm);
    out.left_speed_cmps =
        out.left_valid ? measured_left : fallback_speed(last_left_pct_);
    out.right_speed_cmps =
        out.right_valid ? measured_right : fallback_speed(last_right_pct_);
    out.average_speed_cmps =
        0.5 * (out.left_speed_cmps + out.right_speed_cmps);

    if (!p_.dry_run) {
        left_distance_cm_ += out.left_speed_cmps * dt;
        right_distance_cm_ += out.right_speed_cmps * dt;
    }
    out.left_distance_cm = left_distance_cm_;
    out.right_distance_cm = right_distance_cm_;
    out.distance_cm = 0.5 * (left_distance_cm_ + right_distance_cm_);
    out.distance_valid = !p_.dry_run &&
        (out.left_valid || out.right_valid);
    return out;
}

void MotorAdapter::apply(const ControlCommand& command) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (emergency_stopped_) return;
    last_left_pct_ = command.left_percent;
    last_right_pct_ = command.right_percent;
#ifndef PATH_FOLLOW_NO_HW
    if (hardware_) {
        const double left =
            p_.swap_motors ? command.right_percent : command.left_percent;
        const double right =
            p_.swap_motors ? command.left_percent : command.right_percent;
        hardware_->set_motor_percent(left, right);
    }
#else
    (void)command;
#endif
}

void MotorAdapter::stop() {
    std::lock_guard<std::mutex> lock(mutex_);
#ifndef PATH_FOLLOW_NO_HW
    if (hardware_) hardware_->stop();
#endif
    last_left_pct_ = 0.0;
    last_right_pct_ = 0.0;
}

void MotorAdapter::emergency_stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    emergency_stopped_ = true;
#ifndef PATH_FOLLOW_NO_HW
    if (hardware_) hardware_->stop();
#endif
    last_left_pct_ = 0.0;
    last_right_pct_ = 0.0;
}

double MotorAdapter::rpm_to_cmps(double rpm, double wheel_diameter_cm) {
    return rpm / 60.0 * 3.14159265358979323846 * wheel_diameter_cm;
}

double MotorAdapter::fallback_speed(double percent) const {
    if (std::abs(percent) < 0.5) return 0.0;
    const double estimate =
        percent / std::max(0.05, p_.speed_ff);
    return clamp_value(estimate, -p_.max_speed_cmps, p_.max_speed_cmps);
}

}  // namespace rewrite_path
