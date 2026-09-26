#include "motion_control.hpp"

#include <algorithm>
#include <cmath>

namespace rewrite_path {

namespace {

constexpr double kDegreesPerRadian = 57.29577951308232;
constexpr double kPi = 3.14159265358979323846;
constexpr double kBoostMaxWheelDifferentialCmps = 5.0;
constexpr double kBoostMaxWheelDifferentialRatio = 0.10;
constexpr double kBoostWheelSyncKp = 0.40;
constexpr double kBoostMaxBalancePercent = 4.0;
constexpr double kBoostMaxOutputPercent = 36.0;
constexpr double kWheelFeedforwardMaxOutputRatio = 0.45;
// A closed-loop wheel can retain more speed than its new target when the
// visual command changes. With forbid_reverse enabled, a negative PI output
// used to be clamped to zero, which made both wheels coast when both were
// simultaneously overspeed. Allow a small, bounded braking torque instead.
constexpr double kGeneralOverspeedBrakeKp = 0.20;
constexpr double kGeneralOverspeedBrakeMaxPercent = 6.0;
constexpr double kGeneralOverspeedBrakeMarginCmps = 5.0;
constexpr double kGeneralOverspeedBrakeMinSpeedCmps = 8.0;
// The speed feed-forward is deliberately capped for safety. On a suspended
// or lightly loaded wheel that cap can still exceed the duty needed to hold
// the requested speed, so the integral must be able to cancel it fully.
constexpr double kWheelNegativeIntegralFraction = 1.0;
constexpr double kInnerWheelOverspeedBrakeKp = 0.30;
constexpr double kMinimumForwardWheelTargetRatio = 0.15;
// Hard-turn handling is intended for a wheel-speed split that is large
// relative to the vehicle's forward speed.  Basing this on
// target_yaw/max_yaw_rate makes a low configured yaw limit classify ordinary
// centering corrections as a full hard turn and collapse the inner wheel to
// zero.  Use the physical differential instead, with a floor on the speed
// reference so startup cannot trigger the same false hard-turn mode.
constexpr double kHardTurnBeginDifferentialRatio = 0.30;
constexpr double kHardTurnFullDifferentialRatio = 0.70;
constexpr double kHardTurnFullBoostYawTrackingRatio = 0.50;
constexpr double kHardTurnReleaseBoostYawTrackingRatio = 0.95;
constexpr double kHardTurnMinimumDirectedAuthority = 0.25;
constexpr double kHardTurnYawBalanceMaxPercent = 30.0;
constexpr double kHardTurnYawBalanceOutputRatio = 0.45;
constexpr double kPwmReleaseSlewMultiplier = 2.0;
constexpr double kRunawayAverageWheelSpeedCmps = 600.0;
constexpr double kRunawaySpinWheelSpeedCmps = 80.0;
constexpr double kRunawaySpinYawRateDps = 350.0;
constexpr int kRunawayConfirmFrames = 2;
// Maximum rate at which the smoothed safety speed limit may fall (cm/s per s).
// A spurious one-frame IMU dropout flips the sensor mode and its speed cap; a
// slow fall keeps a brief flicker from chopping vehicle speed while a sustained
// sensor failure still lowers the cap within a fraction of a second.
constexpr double kSpeedLimitFallRateCmps2 = 40.0;

}  // namespace

bool RunawayStopGuard::update(const MotorFeedbackLite& feedback,
                              const ImuFeedback& imu,
                              const NavigationCommand& navigation) {
    if (latched_) return true;

    const bool both_encoders = feedback.left_valid && feedback.right_valid;
    const double average_wheel_speed = 0.5 * (
        std::abs(feedback.left_speed_cmps) +
        std::abs(feedback.right_speed_cmps));
    const bool wheel_overspeed =
        both_encoders &&
        average_wheel_speed >= kRunawayAverageWheelSpeedCmps;
    wheel_overspeed_frames_ =
        wheel_overspeed ? wheel_overspeed_frames_ + 1 : 0;

    double valid_wheel_speed = 0.0;
    if (feedback.left_valid) {
        valid_wheel_speed = std::max(
            valid_wheel_speed, std::abs(feedback.left_speed_cmps));
    }
    if (feedback.right_valid) {
        valid_wheel_speed = std::max(
            valid_wheel_speed, std::abs(feedback.right_speed_cmps));
    }
    const bool commanded_fast_turn =
        navigation.state != DriveState::Stopped &&
        navigation.line_good &&
        navigation.target_yaw_rate_dps * imu.yaw_rate_dps > 0.0 &&
        std::abs(navigation.target_yaw_rate_dps) >=
            0.50 * std::abs(imu.yaw_rate_dps);
    const bool high_speed_spin =
        imu.valid &&
        valid_wheel_speed >= kRunawaySpinWheelSpeedCmps &&
        std::abs(imu.yaw_rate_dps) >= kRunawaySpinYawRateDps &&
        !commanded_fast_turn;
    spin_frames_ = high_speed_spin ? spin_frames_ + 1 : 0;

    latched_ =
        wheel_overspeed_frames_ >= kRunawayConfirmFrames ||
        spin_frames_ >= kRunawayConfirmFrames;
    return latched_;
}

bool RunawayStopGuard::apply(NavigationCommand* navigation) const {
    if (!latched_ || navigation == nullptr) return false;
    navigation->state = DriveState::Stopped;
    navigation->target_speed_cmps = 0.0;
    navigation->target_yaw_rate_dps = 0.0;
    navigation->heading_hold = false;
    navigation->power_boost_percent = 0.0;
    navigation->stop_reason = StopReason::RunawayDetected;
    return true;
}

MotionController::MotionController(const PathParams& params) : p_(params) {}

void MotionController::reset() {
    left_integral_ = 0.0;
    right_integral_ = 0.0;
    limited_speed_cmps_ = 0.0;
    motion_time_s_ = 0.0;
    last_left_percent_ = 0.0;
    last_right_percent_ = 0.0;
    filtered_encoder_yaw_rate_dps_ = 0.0;
    limited_target_yaw_rate_dps_ = 0.0;
    smoothed_speed_limit_cmps_ = 0.0;
    last_significant_target_yaw_sign_ = 0.0;
    imu_yaw_disagreement_frames_ = 0;
    imu_yaw_recovery_frames_ = 0;
    imu_yaw_rejected_ = false;
    target_yaw_rate_ready_ = false;
}

ControlResult MotionController::update(const NavigationCommand& navigation,
                                       const MotorFeedbackLite& feedback,
                                       const ImuFeedback& imu,
                                       double dt) {
    dt = clamp_value(dt, 0.001, 0.10);
    ControlResult result;

    SensorMode mode = SensorMode::VisionOpenLoop;
    if (feedback.left_valid && feedback.right_valid) {
        mode = imu.valid && !imu_yaw_rejected_
            ? SensorMode::Full : SensorMode::EncoderYaw;
    } else if (feedback.left_valid || feedback.right_valid) {
        mode = SensorMode::SingleEncoder;
    } else if (imu.valid) {
        mode = SensorMode::OpenLoopEncoders;
    }

    double mode_limit = speed_limit_for(mode);
    // Let the safety cap rise immediately when sensors recover, but rate-limit
    // its fall so a one-frame IMU dropout (Full -> EncoderYaw) does not chop the
    // vehicle between full speed and imu_fail_speed. A sustained failure still
    // drives the cap down within a fraction of a second.
    if (mode == SensorMode::VisionOpenLoop) {
        // With no IMU or encoder feedback, apply the fail-safe cap
        // immediately. Smoothing is only for a transient partial-sensor
        // downgrade and must not delay the all-feedback-loss response.
        smoothed_speed_limit_cmps_ = mode_limit;
    } else if (mode_limit >= smoothed_speed_limit_cmps_) {
        smoothed_speed_limit_cmps_ = mode_limit;
    } else {
        smoothed_speed_limit_cmps_ = std::max(
            mode_limit,
            smoothed_speed_limit_cmps_ - kSpeedLimitFallRateCmps2 * dt);
    }
    const double effective_limit = smoothed_speed_limit_cmps_;
    const double requested_speed = clamp_value(
        navigation.target_speed_cmps, 0.0, p_.max_speed_cmps);
    const double wanted_speed = std::min(requested_speed, effective_limit);
    if (navigation.state == DriveState::Stopped || wanted_speed <= 0.05) {
        limited_speed_cmps_ = 0.0;
        motion_time_s_ = 0.0;
        left_integral_ = 0.0;
        right_integral_ = 0.0;
        last_left_percent_ = 0.0;
        last_right_percent_ = 0.0;
        limited_target_yaw_rate_dps_ = 0.0;
        target_yaw_rate_ready_ = false;
        // Stopped: snap the smoothed cap to the true limit so the next launch
        // begins from a clean baseline rather than a stale smoothed value.
        smoothed_speed_limit_cmps_ = mode_limit;
        result.diagnostics.sensor_mode = mode;
        result.diagnostics.speed_limit_cmps = mode_limit;
        result.diagnostics.requested_speed_cmps = requested_speed;
        result.diagnostics.measured_yaw_rate_dps =
            imu.valid ? imu.yaw_rate_dps : 0.0;
        result.diagnostics.left_closed_loop = feedback.left_valid;
        result.diagnostics.right_closed_loop = feedback.right_valid;
        return result;
    }

    motion_time_s_ += dt;
    const double delta = wanted_speed - limited_speed_cmps_;
    const double rate =
        delta >= 0.0 ? p_.target_accel_cmps2 : p_.target_decel_cmps2;
    limited_speed_cmps_ += clamp_value(delta, -rate * dt, rate * dt);
    if (p_.startup_ramp_s > 1e-6) {
        limited_speed_cmps_ = std::min(
            limited_speed_cmps_,
            wanted_speed *
                clamp_value(motion_time_s_ / p_.startup_ramp_s, 0.0, 1.0));
    }

    double target_yaw_rate = clamp_value(
        navigation.target_yaw_rate_dps,
        -p_.max_yaw_rate_dps, p_.max_yaw_rate_dps);
    if (navigation.heading_hold && imu.valid) {
        const double heading_error =
            wrap_degrees(navigation.target_heading_deg - imu.heading_deg);
        target_yaw_rate = clamp_value(
            p_.heading_hold_kp * heading_error,
            -p_.heading_hold_max_rate_dps,
            p_.heading_hold_max_rate_dps);
    }
    if (uses_follow_control(navigation.state) &&
        !navigation.heading_hold) {
        // Scale normal FOLLOW yaw with speed, but retain a configurable floor
        // so curve slowdown does not remove the wheel differential exactly
        // when the vehicle needs its strongest steering authority.
        const double follow_speed_ratio = clamp_value(
            wanted_speed / std::max(1.0, p_.base_speed_cmps),
            0.30, 1.0);
        const double follow_yaw_scale = std::max(
            p_.minimum_follow_yaw_scale, follow_speed_ratio);
        const double follow_yaw_limit =
            p_.max_yaw_rate_dps * follow_yaw_scale;
        target_yaw_rate = clamp_value(
            target_yaw_rate, -follow_yaw_limit, follow_yaw_limit);
    }
    if (!target_yaw_rate_ready_) {
        limited_target_yaw_rate_dps_ = target_yaw_rate;
        target_yaw_rate_ready_ = true;
    } else {
        const double max_yaw_step =
            std::max(1.0, p_.target_yaw_slew_dps2) * dt;
        const double previous_limited_yaw =
            limited_target_yaw_rate_dps_;
        limited_target_yaw_rate_dps_ += clamp_value(
            target_yaw_rate - limited_target_yaw_rate_dps_,
            -max_yaw_step, max_yaw_step);
        // A noisy one-frame sign reversal must pass through zero before it
        // can command the opposite turn.
        if (previous_limited_yaw * limited_target_yaw_rate_dps_ < 0.0) {
            limited_target_yaw_rate_dps_ = 0.0;
        }
    }
    target_yaw_rate = limited_target_yaw_rate_dps_;
    if (std::abs(target_yaw_rate) >=
        p_.yaw_direction_guard_target_dps) {
        last_significant_target_yaw_sign_ =
            std::copysign(1.0, target_yaw_rate);
    }

    bool yaw_feedback_valid = false;
    double measured_yaw_rate = 0.0;
    double raw_encoder_yaw_rate = 0.0;
    const bool encoder_yaw_valid =
        feedback.left_valid && feedback.right_valid;
    if (encoder_yaw_valid) {
        raw_encoder_yaw_rate =
            (feedback.left_speed_cmps - feedback.right_speed_cmps) /
            std::max(1.0, p_.wheel_base_cm) * kDegreesPerRadian;
        const double encoder_alpha =
            dt / (p_.encoder_yaw_filter_tau_s + dt);
        filtered_encoder_yaw_rate_dps_ +=
            (raw_encoder_yaw_rate - filtered_encoder_yaw_rate_dps_) *
            encoder_alpha;
    } else {
        const double decay =
            dt / (p_.encoder_yaw_filter_tau_s + dt);
        filtered_encoder_yaw_rate_dps_ +=
            (0.0 - filtered_encoder_yaw_rate_dps_) * decay;
    }

    const bool strong_yaw_disagreement =
        imu.valid && encoder_yaw_valid &&
        std::abs(imu.yaw_rate_dps) >=
            p_.yaw_sensor_disagreement_dps &&
        std::abs(raw_encoder_yaw_rate) >=
            p_.yaw_sensor_disagreement_dps &&
        imu.yaw_rate_dps * raw_encoder_yaw_rate < 0.0;
    if (strong_yaw_disagreement) {
        imu_yaw_disagreement_frames_ = std::min(
            p_.yaw_sensor_disagreement_frames,
            imu_yaw_disagreement_frames_ + 1);
        imu_yaw_recovery_frames_ = 0;
        if (imu_yaw_disagreement_frames_ >=
            p_.yaw_sensor_disagreement_frames) {
            imu_yaw_rejected_ = true;
        }
    } else if (imu_yaw_rejected_) {
        const bool both_quiet =
            imu.valid && encoder_yaw_valid &&
            std::abs(imu.yaw_rate_dps) <
                p_.yaw_sensor_disagreement_dps &&
            std::abs(raw_encoder_yaw_rate) <
                p_.yaw_sensor_disagreement_dps;
        const bool compatible_direction =
            imu.valid && encoder_yaw_valid &&
            std::abs(imu.yaw_rate_dps) >=
                0.5 * p_.yaw_sensor_disagreement_dps &&
            std::abs(raw_encoder_yaw_rate) >=
                0.5 * p_.yaw_sensor_disagreement_dps &&
            imu.yaw_rate_dps * raw_encoder_yaw_rate > 0.0;
        if (both_quiet || compatible_direction) {
            ++imu_yaw_recovery_frames_;
            if (imu_yaw_recovery_frames_ >=
                p_.yaw_sensor_recovery_frames) {
                imu_yaw_rejected_ = false;
                imu_yaw_disagreement_frames_ = 0;
                imu_yaw_recovery_frames_ = 0;
            }
        } else {
            imu_yaw_recovery_frames_ = 0;
        }
    } else {
        imu_yaw_disagreement_frames_ = 0;
    }

    if (imu.valid && !imu_yaw_rejected_) {
        measured_yaw_rate = imu.yaw_rate_dps;
        yaw_feedback_valid = true;
    } else if (encoder_yaw_valid) {
        measured_yaw_rate = filtered_encoder_yaw_rate_dps_;
        yaw_feedback_valid = true;
    }
    if (encoder_yaw_valid && imu_yaw_rejected_) {
        mode = SensorMode::EncoderYaw;
        mode_limit = speed_limit_for(mode);
        limited_speed_cmps_ = std::min(limited_speed_cmps_, mode_limit);
    } else if (encoder_yaw_valid && imu.valid) {
        mode = SensorMode::Full;
        mode_limit = speed_limit_for(mode);
    }

    const double physical_diff =
        target_yaw_rate / kDegreesPerRadian *
        p_.wheel_base_cm * 0.5;
    const double yaw_demand_fraction = clamp_value(
        std::abs(target_yaw_rate) /
            std::max(1.0, p_.max_yaw_rate_dps),
        0.0,
        1.0);
    double effective_power_boost_percent =
        navigation.power_boost_percent;
    if (effective_power_boost_percent > 0.0 &&
        (feedback.left_valid || feedback.right_valid)) {
        double measured_speed = 0.0;
        int measured_wheels = 0;
        if (feedback.left_valid) {
            measured_speed += std::abs(feedback.left_speed_cmps);
            ++measured_wheels;
        }
        if (feedback.right_valid) {
            measured_speed += std::abs(feedback.right_speed_cmps);
            ++measured_wheels;
        }
        measured_speed /= std::max(1, measured_wheels);
        const double boost_speed_error =
            limited_speed_cmps_ - measured_speed;
        effective_power_boost_percent *= clamp_value(
            boost_speed_error /
                std::max(5.0, 0.20 * limited_speed_cmps_),
            0.0,
            1.0);
    }
    const double boost_blend = clamp_value(
        effective_power_boost_percent /
            std::max(1.0, p_.ramp_boost_percent),
        0.0,
        1.0);
    double correction = 0.0;
    if (yaw_feedback_valid) {
        double feedback_gain = p_.yaw_rate_kp;
        double correction_limit = p_.yaw_rate_correction_limit_cmps;
        if (!imu.valid) {
            feedback_gain *= p_.encoder_yaw_kp_scale;
            correction_limit = std::min(
                correction_limit,
                p_.encoder_yaw_correction_limit_cmps);
        }
        const double opposing_yaw_error_fraction =
            yaw_demand_fraction >= 0.10 &&
            target_yaw_rate * measured_yaw_rate < 0.0
            ? clamp_value(
                std::abs(target_yaw_rate - measured_yaw_rate) /
                    std::max(1.0, p_.max_yaw_rate_dps),
                0.0,
                1.0)
            : 0.0;
        const double feedback_scale =
            0.25 + 0.75 * std::max(
                yaw_demand_fraction,
                opposing_yaw_error_fraction);
        feedback_gain *= feedback_scale;
        correction_limit *= feedback_scale;
        correction = clamp_value(
            feedback_gain * (target_yaw_rate - measured_yaw_rate),
            -correction_limit,
            correction_limit);
    }

    double drive_differential = physical_diff + correction;
    if (boost_blend > 0.0) {
        const double boost_differential_limit = std::min(
            kBoostMaxWheelDifferentialCmps,
            limited_speed_cmps_ * kBoostMaxWheelDifferentialRatio);
        const double boost_differential = clamp_value(
            drive_differential,
            -boost_differential_limit,
            boost_differential_limit);
        drive_differential +=
            (boost_differential - drive_differential) * boost_blend;
    }
    if (navigation.state == DriveState::Roundabout ||
        navigation.state == DriveState::RoundaboutExit) {
        if (target_yaw_rate > 0.0) {
            drive_differential = std::max(0.0, drive_differential);
        } else if (target_yaw_rate < 0.0) {
            drive_differential = std::min(0.0, drive_differential);
        }
    }
    const double hard_turn_speed_reference = std::max(
        limited_speed_cmps_,
        0.25 * std::max(1.0, p_.base_speed_cmps));
    const double differential_ratio =
        std::abs(physical_diff) /
        std::max(1.0, hard_turn_speed_reference);
    const double hard_turn_blend = yaw_feedback_valid
        ? clamp_value(
            (differential_ratio - kHardTurnBeginDifferentialRatio) /
            (kHardTurnFullDifferentialRatio -
             kHardTurnBeginDifferentialRatio),
            0.0,
            1.0)
        : 0.0;
    double hard_turn_effort_need = 1.0;
    if (yaw_feedback_valid && std::abs(target_yaw_rate) > 1.0) {
        const double aligned_measured_yaw =
            std::copysign(1.0, target_yaw_rate) *
            measured_yaw_rate;
        const double yaw_tracking_ratio =
            aligned_measured_yaw /
            std::abs(target_yaw_rate);
        hard_turn_effort_need = clamp_value(
            (kHardTurnReleaseBoostYawTrackingRatio -
             yaw_tracking_ratio) /
                (kHardTurnReleaseBoostYawTrackingRatio -
                 kHardTurnFullBoostYawTrackingRatio),
            0.0,
            1.0);
    }
    const double normal_outer_speed_ceiling =
        p_.base_speed_cmps +
        0.5 * (p_.max_speed_cmps - p_.base_speed_cmps) *
            hard_turn_blend;
    const double aggressive_outer_speed_ceiling =
        p_.max_speed_cmps *
        p_.hard_turn_outer_speed_scale;
    const double outer_speed_ceiling =
        normal_outer_speed_ceiling +
        (aggressive_outer_speed_ceiling -
         normal_outer_speed_ceiling) *
            hard_turn_blend *
            hard_turn_effort_need;
    if (hard_turn_blend > 0.0) {
        const double full_turn_differential = std::max(
            limited_speed_cmps_,
            outer_speed_ceiling - limited_speed_cmps_);
        const double directed_authority =
            kHardTurnMinimumDirectedAuthority +
            (1.0 - kHardTurnMinimumDirectedAuthority) *
                hard_turn_effort_need;
        const double minimum_directed_differential =
            std::max(
                0.5 * std::abs(physical_diff),
                full_turn_differential) *
            hard_turn_blend *
            directed_authority *
            (1.0 - boost_blend);
        if (target_yaw_rate > 0.0) {
            drive_differential = std::max(
                drive_differential,
                minimum_directed_differential);
        } else if (target_yaw_rate < 0.0) {
            drive_differential = std::min(
                drive_differential,
                -minimum_directed_differential);
        }
    }
    if (p_.forbid_reverse && limited_speed_cmps_ > 0.0) {
        const double minimum_forward_ratio =
            kMinimumForwardWheelTargetRatio *
            (1.0 - hard_turn_blend);
        const double normal_forward_differential =
            limited_speed_cmps_ * (1.0 - minimum_forward_ratio);
        const double full_turn_differential = std::max(
            normal_forward_differential,
            outer_speed_ceiling - limited_speed_cmps_);
        const double maximum_forward_differential =
            normal_forward_differential +
            (full_turn_differential - normal_forward_differential) *
                hard_turn_blend;
        drive_differential = clamp_value(
            drive_differential,
            -maximum_forward_differential,
            maximum_forward_differential);
    }
    double left_target = limited_speed_cmps_ + drive_differential;
    double right_target = limited_speed_cmps_ - drive_differential;
    if (p_.forbid_reverse && limited_speed_cmps_ > 0.0) {
        left_target = std::max(0.0, left_target);
        right_target = std::max(0.0, right_target);
    }
    const double wheel_target_limit =
        p_.forbid_reverse
            ? p_.max_speed_cmps *
                (1.0 +
                 (p_.hard_turn_outer_speed_scale - 1.0) *
                     hard_turn_blend *
                     hard_turn_effort_need)
            : p_.max_speed_cmps;
    left_target = clamp_value(
        left_target, -wheel_target_limit, wheel_target_limit);
    right_target = clamp_value(
        right_target, -wheel_target_limit, wheel_target_limit);

    double left_percent = wheel_output(
        left_target, feedback.left_speed_cmps, feedback.left_valid,
        effective_power_boost_percent, &left_integral_, dt);
    double right_percent = wheel_output(
        right_target, feedback.right_speed_cmps, feedback.right_valid,
        effective_power_boost_percent, &right_integral_, dt);
    const auto overspeed_brake_limit = [&](double target_cmps,
                                           double measured_cmps,
                                           bool feedback_valid) {
        if (!p_.forbid_reverse || !feedback_valid ||
            measured_cmps <= kGeneralOverspeedBrakeMinSpeedCmps) {
            return 0.0;
        }
        const double overspeed = measured_cmps -
            std::max(0.0, target_cmps) -
            kGeneralOverspeedBrakeMarginCmps;
        if (overspeed <= 0.0) return 0.0;
        return std::min(
            kGeneralOverspeedBrakeMaxPercent,
            kGeneralOverspeedBrakeKp * overspeed);
    };
    const double left_overspeed_brake = overspeed_brake_limit(
        left_target, feedback.left_speed_cmps, feedback.left_valid);
    const double right_overspeed_brake = overspeed_brake_limit(
        right_target, feedback.right_speed_cmps, feedback.right_valid);
    if (p_.forbid_reverse && hard_turn_blend > 0.0 &&
        boost_blend < 1.0 && target_yaw_rate != 0.0) {
        double* outer_percent =
            target_yaw_rate > 0.0 ? &left_percent : &right_percent;
        const double previous_outer_percent =
            target_yaw_rate > 0.0
                ? last_left_percent_
                : last_right_percent_;
        const double outer_target =
            target_yaw_rate > 0.0 ? left_target : right_target;
        const double outer_speed = std::abs(
            target_yaw_rate > 0.0
                ? feedback.left_speed_cmps
                : feedback.right_speed_cmps);
        const double speed_margin =
            std::max(5.0, 0.10 * std::abs(outer_target));
        const double sustain_scale = clamp_value(
            (1.15 * std::abs(outer_target) - outer_speed) /
                speed_margin,
            0.0,
            1.0);
        const double sustain_percent = std::max(
            previous_outer_percent,
            std::min(
                p_.max_percent *
                    kWheelFeedforwardMaxOutputRatio,
                p_.speed_ff * std::abs(outer_target)) *
                hard_turn_blend) *
            sustain_scale *
            (1.0 - boost_blend);
        *outer_percent = std::max(*outer_percent, sustain_percent);
    }
    if (p_.forbid_reverse && limited_speed_cmps_ > 0.0) {
        if (hard_turn_blend > 0.0) {
            const double balance_limit = std::min(
                kHardTurnYawBalanceMaxPercent,
                p_.max_percent * kHardTurnYawBalanceOutputRatio);
            double yaw_balance_percent = clamp_value(
                p_.speed_ff * correction * hard_turn_blend *
                    (1.0 - boost_blend),
                -balance_limit,
                balance_limit);
            if (target_yaw_rate > 0.0) {
                yaw_balance_percent =
                    std::max(0.0, yaw_balance_percent);
                if (feedback.left_valid) {
                    const double speed_headroom =
                        1.15 * left_target -
                        std::abs(feedback.left_speed_cmps);
                    yaw_balance_percent *= clamp_value(
                        speed_headroom /
                            std::max(5.0, 0.15 * std::abs(left_target)),
                        0.0,
                        1.0);
                }
            } else {
                yaw_balance_percent =
                    std::min(0.0, yaw_balance_percent);
                if (feedback.right_valid) {
                    const double speed_headroom =
                        1.15 * right_target -
                        std::abs(feedback.right_speed_cmps);
                    yaw_balance_percent *= clamp_value(
                        speed_headroom /
                            std::max(5.0, 0.15 * std::abs(right_target)),
                        0.0,
                        1.0);
                }
            }
            left_percent += yaw_balance_percent;
            right_percent -= yaw_balance_percent;
        }
        left_percent = clamp_value(
            left_percent, -left_overspeed_brake, p_.max_percent);
        right_percent = clamp_value(
            right_percent, -right_overspeed_brake, p_.max_percent);
    }

    if (boost_blend > 0.0) {
        const double boost_output_limit = std::min(
            p_.max_percent, kBoostMaxOutputPercent);
        const double normal_left_percent = left_percent;
        const double normal_right_percent = right_percent;
        double common_percent = clamp_value(
            0.5 * (left_percent + right_percent),
            0.0,
            boost_output_limit);

        const double desired_wheel_difference =
            left_target - right_target;
        const double measured_wheel_difference =
            feedback.left_speed_cmps - feedback.right_speed_cmps;
        double balance_percent = 0.5 * p_.speed_ff *
            desired_wheel_difference;
        if (feedback.left_valid && feedback.right_valid) {
            balance_percent = kBoostWheelSyncKp *
                (desired_wheel_difference -
                 measured_wheel_difference);
        }
        balance_percent = clamp_value(
            balance_percent,
            -kBoostMaxBalancePercent,
            kBoostMaxBalancePercent);
        common_percent = std::min(
            common_percent,
            boost_output_limit - std::abs(balance_percent));
        common_percent = std::max(0.0, common_percent);
        const double boost_left_percent = clamp_value(
            common_percent + balance_percent,
            0.0,
            boost_output_limit);
        const double boost_right_percent = clamp_value(
            common_percent - balance_percent,
            0.0,
            boost_output_limit);
        left_percent +=
            (boost_left_percent - normal_left_percent) * boost_blend;
        right_percent +=
            (boost_right_percent - normal_right_percent) * boost_blend;
    }
    // forbid_reverse constrains wheel targets, but zero PWM cannot quickly
    // shed the stored speed seen when an S-bend changes turn direction.
    // During a confirmed hard turn only, apply bounded reverse torque to the
    // new inner wheel while it is still moving forward well above target.
    if (p_.forbid_reverse && hard_turn_blend > 0.0 &&
        p_.inner_wheel_brake_max_percent > 0.0 &&
        target_yaw_rate != 0.0) {
        const bool left_is_inner = target_yaw_rate < 0.0;
        double* inner_percent =
            left_is_inner ? &left_percent : &right_percent;
        double* inner_integral =
            left_is_inner ? &left_integral_ : &right_integral_;
        const double inner_target =
            left_is_inner ? left_target : right_target;
        const double inner_speed =
            left_is_inner ? feedback.left_speed_cmps
                          : feedback.right_speed_cmps;
        const bool inner_feedback_valid =
            left_is_inner ? feedback.left_valid : feedback.right_valid;
        const double overspeed =
            inner_speed - std::max(0.0, inner_target) -
            p_.inner_wheel_brake_margin_cmps;
        if (inner_feedback_valid && inner_speed > 0.0 &&
            overspeed > 0.0) {
            const double brake_percent = std::min(
                p_.inner_wheel_brake_max_percent * hard_turn_blend,
                kInnerWheelOverspeedBrakeKp * overspeed);
            if (brake_percent >= p_.min_move_percent) {
                *inner_percent = std::min(
                    *inner_percent, -brake_percent);
                *inner_integral = 0.0;
            }
        }
    }
    bool yaw_direction_guard_active = false;
    const bool follow_centering_correction =
        uses_follow_control(navigation.state) &&
        hard_turn_blend == 0.0 &&
        std::abs(target_yaw_rate) <= 0.5 * p_.max_yaw_rate_dps &&
        encoder_yaw_valid &&
        target_yaw_rate * raw_encoder_yaw_rate <= 0.0;
    if (p_.forbid_reverse && !follow_centering_correction &&
        encoder_yaw_valid &&
        last_significant_target_yaw_sign_ != 0.0 &&
        std::abs(raw_encoder_yaw_rate) >=
            p_.yaw_direction_guard_dps &&
        raw_encoder_yaw_rate *
                last_significant_target_yaw_sign_ > 0.0) {
        const double pwm_yaw_direction =
            left_percent - right_percent;
        if (pwm_yaw_direction *
                last_significant_target_yaw_sign_ < 0.0) {
            yaw_direction_guard_active = true;
            if (last_significant_target_yaw_sign_ < 0.0) {
                left_percent = right_percent;
                left_integral_ = 0.0;
            } else {
                right_percent = left_percent;
                right_integral_ = 0.0;
            }
        }
    }
    left_percent = ramp(
        left_percent, last_left_percent_, dt, 1.0);
    right_percent = ramp(
        right_percent, last_right_percent_, dt, 1.0);
    last_left_percent_ = left_percent;
    last_right_percent_ = right_percent;
    const bool saturated =
        std::abs(left_percent) >= p_.max_percent - 1e-6 ||
        std::abs(right_percent) >= p_.max_percent - 1e-6;

    result.command.left_percent = left_percent;
    result.command.right_percent = right_percent;
    result.diagnostics.sensor_mode = mode;
    result.diagnostics.speed_limit_cmps = effective_limit;
    result.diagnostics.requested_speed_cmps = requested_speed;
    result.diagnostics.limited_speed_cmps = limited_speed_cmps_;
    result.diagnostics.requested_yaw_rate_dps = target_yaw_rate;
    result.diagnostics.measured_yaw_rate_dps = measured_yaw_rate;
    result.diagnostics.raw_encoder_yaw_rate_dps = raw_encoder_yaw_rate;
    result.diagnostics.filtered_encoder_yaw_rate_dps =
        filtered_encoder_yaw_rate_dps_;
    result.diagnostics.yaw_sensor_disagreement_frames =
        imu_yaw_disagreement_frames_;
    result.diagnostics.left_target_cmps = left_target;
    result.diagnostics.right_target_cmps = right_target;
    result.diagnostics.left_error_cmps =
        left_target - feedback.left_speed_cmps;
    result.diagnostics.right_error_cmps =
        right_target - feedback.right_speed_cmps;
    result.diagnostics.target_steering_utilization = clamp_value(
        std::abs(left_target - right_target) /
            std::max(1.0, 2.0 * limited_speed_cmps_),
        0.0,
        1.0);
    result.diagnostics.pwm_steering_utilization = clamp_value(
        std::abs(left_percent - right_percent) /
            std::max(1.0, p_.max_percent),
        0.0,
        1.0);
    result.diagnostics.power_boost_percent =
        effective_power_boost_percent;
    result.diagnostics.left_closed_loop = feedback.left_valid;
    result.diagnostics.right_closed_loop = feedback.right_valid;
    result.diagnostics.imu_yaw_rejected = imu_yaw_rejected_;
    result.diagnostics.yaw_direction_guard_active =
        yaw_direction_guard_active;
    result.diagnostics.saturated = saturated;
    return result;
}

double MotionController::wheel_output(double target_cmps,
                                      double measured_cmps,
                                      bool closed_loop,
                                      double power_boost_percent,
                                      double* integral,
                                      double dt) {
    if (std::abs(target_cmps) <= 0.05) {
        *integral = 0.0;
        return 0.0;
    }

    const double boost_scale = clamp_value(
        std::abs(target_cmps) /
            std::max(1.0, p_.ramp_boost_speed_cmps),
        0.0, 1.0);
    const double base_feedforward = clamp_value(
        p_.speed_ff * target_cmps,
        -p_.max_percent * kWheelFeedforwardMaxOutputRatio,
        p_.max_percent * kWheelFeedforwardMaxOutputRatio);
    const double feedforward =
        base_feedforward +
        std::copysign(
            std::max(0.0, power_boost_percent) * boost_scale,
            target_cmps);
    double correction = 0.0;
    if (closed_loop) {
        const double error = target_cmps - measured_cmps;
        const double candidate = clamp_value(
            *integral + error * dt,
            -p_.wheel_speed_i_limit *
                kWheelNegativeIntegralFraction,
            p_.wheel_speed_i_limit);
        correction =
            p_.wheel_speed_kp * error + p_.wheel_speed_ki * candidate;
        const double unsaturated = feedforward + correction;
        const bool pushes_high =
            unsaturated > p_.max_percent && error > 0.0;
        const bool pushes_low =
            unsaturated < -p_.max_percent && error < 0.0;
        if (!pushes_high && !pushes_low) {
            *integral = candidate;
        } else {
            correction =
                p_.wheel_speed_kp * error + p_.wheel_speed_ki * (*integral);
        }
    } else {
        *integral = 0.0;
    }

    double output = clamp_value(
        feedforward + correction, -p_.max_percent, p_.max_percent);
    return dead_zone(output, measured_cmps, closed_loop);
}

double MotionController::dead_zone(double percent,
                                   double measured_cmps,
                                   bool closed_loop) const {
    if (std::abs(percent) < 0.5) return 0.0;
    // min_move_percent is breakaway effort for a stopped wheel, not the
    // minimum sustainable duty once the wheel is already turning. Forcing
    // every small closed-loop correction up to that floor makes an unloaded
    // wheel alternate between zero and min_move_percent.
    const bool wheel_is_moving =
        closed_loop &&
        std::abs(measured_cmps) >=
            p_.encoder_recover_rpm / 60.0 *
                kPi * p_.wheel_diameter_cm;
    if (wheel_is_moving) return percent;
    if (std::abs(percent) < p_.min_move_percent) {
        return std::copysign(p_.min_move_percent, percent);
    }
    return percent;
}

double MotionController::ramp(double wanted, double previous, double dt,
                              double rise_scale) const {
    const bool releasing =
        std::abs(wanted) < std::abs(previous) ||
        wanted * previous < 0.0;
    const double slew_multiplier =
        releasing ? kPwmReleaseSlewMultiplier :
            clamp_value(rise_scale, 0.0, 1.0);
    const double step =
        p_.max_percent_delta_per_s * slew_multiplier * dt;
    return clamp_value(wanted, previous - step, previous + step);
}

double MotionController::speed_limit_for(SensorMode mode) const {
    switch (mode) {
        case SensorMode::Full:
            return p_.max_speed_cmps;
        case SensorMode::EncoderYaw:
            return std::min(p_.max_speed_cmps, p_.imu_fail_speed_cmps);
        case SensorMode::SingleEncoder:
            return std::min(p_.max_speed_cmps, p_.single_encoder_speed_cmps);
        case SensorMode::OpenLoopEncoders:
            return std::min(p_.max_speed_cmps, p_.encoder_fail_speed_cmps);
        case SensorMode::VisionOpenLoop:
            return std::min(p_.max_speed_cmps, p_.all_sensor_fail_speed_cmps);
    }
    return p_.all_sensor_fail_speed_cmps;
}

double MotionController::wrap_degrees(double angle) {
    while (angle >= 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

}  // namespace rewrite_path
