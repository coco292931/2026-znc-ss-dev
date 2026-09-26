#include "path_controller.hpp"

#include <algorithm>
#include <cmath>

namespace rewrite_path {

namespace {

constexpr double kPairlessCurveMinError = 0.28;
constexpr double kPairlessCurveMaxDelta = 0.22;
constexpr double kPairlessCurveMinConfidence = 0.75;
constexpr double kPairlessCurveGraceS = 0.50;

}  // namespace

PathController::PathController(const PathParams& params) : p_(params) {}

void PathController::reset() {
    state_ = DriveState::Follow;
    stop_reason_ = StopReason::None;
    action_kind_ = TargetKind::None;
    state_time_s_ = 0.0;
    lost_s_ = 0.0;
    pairless_curve_s_ = 0.0;
    cross_refractory_s_ = 0.0;
    side_refractory_s_ = 0.0;
    target_cooldown_s_ = 0.0;
    state_start_distance_cm_ = 0.0;
    state_start_heading_deg_ = 0.0;
    state_heading_valid_ = false;
    state_start_encoder_heading_deg_ = 0.0;
    state_encoder_heading_valid_ = false;
    imu_heading_origin_deg_ = 0.0;
    imu_heading_origin_valid_ = false;
    last_yaw_rate_dps_ = 0.0;
    vision_integral_error_s_ = 0.0;
    vision_derivative_error_per_s_ = 0.0;
    previous_visual_error_ = 0.0;
    visual_derivative_ready_ = false;
    has_clean_line_ = false;
    side_consumed_ = false;
    cross_enter_count_ = 0;
    cross_exit_count_ = 0;
    side_exit_count_ = 0;
    resume_count_ = 0;
    target_enter_count_ = 0;
    zebra_enter_count_ = 0;
    zebra_exit_count_ = 0;
    zebra_encounter_count_ = 0;
    zebra_active_ = false;
    action_phase_ = 0;
    side_direction_ = 0;
}

void PathController::finish_external_target_action() {
    target_cooldown_s_ = p_.target_cooldown_s;
    transition(DriveState::Follow);
}

NavigationCommand PathController::update(const StepInput& input, double dt) {
    dt = clamp_value(dt, 0.001, 0.10);
    if (!imu_heading_origin_valid_ &&
        input.heading_valid && input.heading_from_imu) {
        imu_heading_origin_deg_ = input.heading_deg;
        imu_heading_origin_valid_ = true;
    }
    state_time_s_ += dt;
    cross_refractory_s_ = std::max(0.0, cross_refractory_s_ - dt);
    side_refractory_s_ = std::max(0.0, side_refractory_s_ - dt);
    target_cooldown_s_ = std::max(0.0, target_cooldown_s_ - dt);
    update_zebra_state(input);

    const bool visual_line_good =
        !input.line_lost && input.line_confidence >= p_.min_line_confidence;
    const bool topology_pair_substitute =
        (input.cross && input.two_side_stable) ||
        (input.side_open != FeatureSide::None &&
         input.roundabout_stage != RoundaboutVisionStage::None);
    const double signed_near_error = p_.vision_yaw_sign *
        clamp_value(input.line_error, -1.0, 1.0);
    const double signed_far_error = p_.vision_yaw_sign *
        clamp_value(input.far_error, -1.0, 1.0);
    const bool strong_consistent_curve =
        visual_line_good &&
        !input.cross &&
        input.line_confidence >= kPairlessCurveMinConfidence &&
        std::abs(signed_near_error) >= kPairlessCurveMinError &&
        std::abs(signed_far_error) >= kPairlessCurveMinError &&
        signed_near_error * signed_far_error > 0.0 &&
        std::abs(signed_far_error - signed_near_error) <=
            kPairlessCurveMaxDelta;
    const bool raw_pair_missing_in_follow =
        uses_follow_control(state_) &&
        !input.bottom_pair_valid &&
        !input.two_side_stable &&
        !topology_pair_substitute;
    if (raw_pair_missing_in_follow && strong_consistent_curve) {
        pairless_curve_s_ += dt;
    } else {
        pairless_curve_s_ = 0.0;
    }
    const bool pairless_curve_grace =
        strong_consistent_curve &&
        pairless_curve_s_ <= kPairlessCurveGraceS;
    const bool pair_missing_in_follow =
        raw_pair_missing_in_follow && !pairless_curve_grace;
    const bool line_good =
        visual_line_good && !pair_missing_in_follow;
    update_visual_integral(input, line_good, dt);
    const bool zebra_line_loss_guard =
        input.zebra || zebra_active_ || zebra_enter_count_ > 0;
    if (line_good) {
        lost_s_ = 0.0;
        last_yaw_rate_dps_ = follow_yaw_rate(input);
        has_clean_line_ = true;
    } else if (zebra_line_loss_guard) {
        lost_s_ = 0.0;
        last_yaw_rate_dps_ *= p_.lost_yaw_decay;
    } else {
        lost_s_ += dt;
        last_yaw_rate_dps_ *= pair_missing_in_follow
            ? 0.35 : p_.lost_yaw_decay;
    }

    if (p_.enable_stop && lost_s_ >= p_.lost_stop_s &&
        state_ != DriveState::Stopped) {
        stop_reason_ = StopReason::LineLost;
        transition(DriveState::Stopped);
    }

    update_state(input, line_good);

    NavigationCommand out;
    out.state = state_;
    out.line_good = line_good;
    out.vision_integral_error_s = vision_integral_error_s_;
    out.vision_integral_yaw_rate_dps =
        p_.vision_i_gain * vision_integral_error_s_;
    out.vision_derivative_error_per_s =
        vision_derivative_error_per_s_;
    out.vision_derivative_yaw_rate_dps =
        p_.vision_d_gain * vision_derivative_error_per_s_;
    const VisionYawBreakdown vision_yaw = vision_yaw_breakdown(input);
    out.vision_near_yaw_rate_dps = vision_yaw.near_yaw_rate_dps;
    out.vision_far_yaw_rate_dps = vision_yaw.far_yaw_rate_dps;
    out.vision_center_yaw_rate_dps = vision_yaw.center_yaw_rate_dps;
    out.vision_curve_gain = vision_yaw.curve_gain;
    out.vision_unclamped_yaw_rate_dps =
        vision_yaw.unclamped_yaw_rate_dps;
    out.stop_reason = stop_reason_;
    out.action_kind = action_kind_;
    out.cross_score = cross_enter_count_;
    out.zebra_active = zebra_active_;
    out.zebra_encounter_count = zebra_encounter_count_;

    const double base = std::min(p_.base_speed_cmps, p_.max_speed_cmps);
    if (state_ == DriveState::Stopped) {
        return out;
    }

    if (state_ == DriveState::CrossLock) {
        out.target_speed_cmps = base * p_.cross_speed_scale;
        out.heading_hold = state_heading_valid_;
        out.target_heading_deg = state_start_heading_deg_;
        if (state_heading_valid_ && input.heading_valid) {
            out.target_yaw_rate_dps = clamp_value(
                p_.heading_hold_kp *
                    wrap_degrees(state_start_heading_deg_ -
                                 input.heading_deg),
                -p_.heading_hold_max_rate_dps,
                p_.heading_hold_max_rate_dps);
        }
        return out;
    }

    if (state_ == DriveState::RoundaboutExit) {
        out.target_speed_cmps = base * p_.round_speed_scale;
        out.heading_hold = state_heading_valid_;
        out.target_heading_deg =
            state_start_heading_deg_ +
            static_cast<double>(side_direction_) * 360.0;
        if (state_heading_valid_ && input.heading_valid) {
            out.target_yaw_rate_dps = clamp_value(
                p_.heading_hold_kp *
                    wrap_degrees(out.target_heading_deg -
                                 input.heading_deg),
                -p_.heading_hold_max_rate_dps,
                p_.heading_hold_max_rate_dps);
        }
        return out;
    }

    if (state_ == DriveState::Roundabout) {
        out.target_speed_cmps = base * p_.round_speed_scale;
        const double visual_yaw_rate = line_good
            ? follow_yaw_rate(input)
            : clamp_value(last_yaw_rate_dps_,
                          -p_.max_yaw_rate_dps,
                          p_.max_yaw_rate_dps);
        if (side_direction_ != 0) {
            const double directed_turn_deg = side_direction_ *
                (input.heading_deg - state_start_heading_deg_);
            const bool orbit_incomplete =
                !input.heading_valid ||
                directed_turn_deg < 360.0;
            const bool entry_direction_lock =
                state_ == DriveState::Roundabout &&
                (state_time_s_ < 0.8 || directed_turn_deg < 90.0);
            const bool imu_turning_opposite =
                input.yaw_rate_valid &&
                side_direction_ * input.yaw_rate_dps <
                    -p_.imu_deadband_dps;
            double minimum_yaw_rate = std::max(
                p_.imu_deadband_dps,
                p_.max_yaw_rate_dps *
                    (orbit_incomplete ? 0.25 : 0.10));
            if (entry_direction_lock) {
                minimum_yaw_rate = std::max(
                    minimum_yaw_rate,
                    p_.max_yaw_rate_dps * 0.35);
            }
            if (imu_turning_opposite) {
                minimum_yaw_rate = std::max(
                    minimum_yaw_rate,
                    p_.max_yaw_rate_dps * 0.40);
            }
            const double directed_yaw_rate =
                side_direction_ * visual_yaw_rate;
            out.target_yaw_rate_dps = side_direction_ * clamp_value(
                std::max(directed_yaw_rate, minimum_yaw_rate),
                minimum_yaw_rate,
                p_.max_yaw_rate_dps);
        } else {
            out.target_yaw_rate_dps = visual_yaw_rate;
        }
        return out;
    }

    if (state_ == DriveState::BypassLeft ||
        state_ == DriveState::BypassRight) {
        const int direction =
            state_ == DriveState::BypassRight ? 1 : -1;
        out.target_speed_cmps = base * p_.bypass_speed_scale;
        if (action_phase_ == 0) {
            out.target_yaw_rate_dps =
                direction * p_.bypass_out_yaw_rate_dps;
        } else if (action_phase_ == 1) {
            out.target_yaw_rate_dps =
                -direction * p_.bypass_return_yaw_rate_dps;
        } else {
            out.target_yaw_rate_dps =
                line_good ? follow_yaw_rate(input) : 0.0;
        }
        return out;
    }

    if (state_ == DriveState::StraightOver) {
        out.target_speed_cmps = base * p_.bypass_speed_scale;
        out.heading_hold = state_heading_valid_;
        out.target_heading_deg = state_start_heading_deg_;
        if (state_heading_valid_ && input.heading_valid) {
            out.target_yaw_rate_dps = clamp_value(
                p_.heading_hold_kp *
                    wrap_degrees(state_start_heading_deg_ -
                                 input.heading_deg),
                -p_.heading_hold_max_rate_dps,
                p_.heading_hold_max_rate_dps);
        }
        return out;
    }

    if (!line_good) {
        out.target_speed_cmps =
            has_clean_line_ ? base * p_.lost_speed_scale : 0.0;
        out.target_yaw_rate_dps = last_yaw_rate_dps_;
        return out;
    }

    const double yaw = follow_yaw_rate(input);
    const double curvature =
        std::min(1.0, std::abs(yaw) / std::max(1.0, p_.max_yaw_rate_dps));
    const double lateral_error = std::max(
        std::max(
            std::abs(input.line_error),
            std::abs(input.far_error)),
        std::abs(input.vehicle_center_error));
    const double preview_disagreement =
        std::abs(input.line_error - input.far_error);
    const double tracking_risk = std::max(
        curvature,
        clamp_value(
            0.75 * lateral_error + 0.50 * preview_disagreement,
            0.0,
            1.0));
    const double confidence_loss =
        1.0 - clamp_value(input.line_confidence, 0.0, 1.0);
    const double preview_turn_risk = clamp_value(
        (std::abs(input.far_error) - 0.08) / 0.40,
        0.0,
        1.0);
    const double speed_scale = clamp_value(
        1.0 - p_.curvature_slowdown * tracking_risk -
            0.45 * preview_turn_risk -
            p_.confidence_slowdown * confidence_loss,
        p_.minimum_follow_speed_scale, 1.0);
    out.target_speed_cmps = base * speed_scale;
    out.target_yaw_rate_dps = yaw;
    return out;
}

void PathController::update_state(const StepInput& input, bool line_good) {
    if (state_ == DriveState::Stopped) {
        if (stop_reason_ == StopReason::ZebraComplete) {
            return;
        }
        if (line_good && input.two_side_stable) {
            ++resume_count_;
            if (resume_count_ >= p_.resume_stable_frames) {
                stop_reason_ = StopReason::None;
                transition(DriveState::Follow);
            }
        } else {
            resume_count_ = 0;
        }
        return;
    }

    if (state_ == DriveState::ZebraPass) {
        return;
    }

    if (state_ == DriveState::CrossLock) {
        const bool held_long_enough =
            state_time_s_ >= p_.cross_min_time_s;
        const bool traveled =
            !input.distance_valid ||
            state_distance(input) >= p_.cross_min_dist_cm;
        const bool recentered =
            std::abs(input.line_error) <= 0.25 &&
            std::abs(input.far_error) <= 0.25 &&
            std::abs(input.line_error - input.far_error) <= 0.20;
        const bool clear =
            held_long_enough && traveled &&
            !input.cross &&
            input.side_open == FeatureSide::None &&
            line_good &&
            input.two_side_stable && recentered;
        if (clear) {
            ++cross_exit_count_;
            if (cross_exit_count_ >= p_.cross_exit_frames) {
                cross_refractory_s_ = p_.cross_refractory_s;
                transition(DriveState::Follow);
            }
        } else {
            cross_exit_count_ = 0;
        }
        if (state_time_s_ >= p_.cross_timeout_s) {
            cross_refractory_s_ = p_.cross_refractory_s;
            transition(DriveState::Follow);
        }
        return;
    }

    if (state_ == DriveState::Roundabout) {
        const bool early_tof_failure =
            state_time_s_ < 0.5 && p_.side_require_tof &&
            (input.ramp_detected ||
             !input.tof_valid || !input.tof_baseline_ready);
        if (early_tof_failure) {
            side_refractory_s_ = p_.round_refractory_s;
            transition(DriveState::Follow);
            return;
        }
        const double direction =
            side_direction_ < 0 ? -1.0 : 1.0;
        const bool direction_lock_complete =
            state_time_s_ >= 0.5 &&
            input.heading_valid &&
            input.heading_from_imu &&
            input.encoder_heading_valid &&
            direction * (input.heading_deg -
                         state_start_heading_deg_) >= 360.0 &&
            direction * (input.encoder_heading_deg -
                         state_start_encoder_heading_deg_) >= 360.0;
        if (direction_lock_complete) {
            enter_side_exit();
            return;
        }
        if (state_time_s_ >= p_.round_timeout_s) {
            stop_reason_ = StopReason::SideTimeout;
            transition(DriveState::Stopped);
        }
        return;
    }

    if (state_ == DriveState::RoundaboutExit) {
        const double heading_tolerance_deg = clamp_value(
            p_.cross_heading_tolerance_deg, 15.0, 60.0);
        const double direction =
            side_direction_ < 0 ? -1.0 : 1.0;
        const double imu_turn_deg = direction *
            (input.heading_deg - state_start_heading_deg_);
        const double encoder_turn_deg = direction *
            (input.encoder_heading_deg -
             state_start_encoder_heading_deg_);
        const bool sensor_lock_ready =
            side_direction_ != 0 &&
            state_heading_valid_ &&
            input.heading_valid &&
            input.heading_from_imu &&
            state_encoder_heading_valid_ &&
            input.encoder_heading_valid &&
            input.distance_valid &&
            std::abs(imu_turn_deg - 360.0) <= heading_tolerance_deg &&
            encoder_turn_deg >= 360.0 - heading_tolerance_deg;
        const bool exit_ready =
            sensor_lock_ready && line_good && input.two_side_stable;
        side_exit_count_ =
            exit_ready ? side_exit_count_ + 1 : 0;
        if (side_exit_count_ >= p_.round_exit_frames) {
            side_refractory_s_ = p_.round_refractory_s;
            transition(DriveState::Follow);
            return;
        }
        if (state_time_s_ >= p_.round_timeout_s * 2.0) {
            stop_reason_ = StopReason::SideTimeout;
            transition(DriveState::Stopped);
        }
        return;
    }

    if (state_ == DriveState::BypassLeft ||
        state_ == DriveState::BypassRight) {
        const double heading = std::abs(state_heading_delta(input));
        const double distance = state_distance(input);
        if (action_phase_ == 0 &&
            (heading >= p_.bypass_out_heading_deg ||
             distance >= p_.bypass_out_distance_cm)) {
            action_phase_ = 1;
        } else if (action_phase_ == 1 &&
                   distance >= p_.bypass_out_distance_cm +
                                   p_.bypass_around_distance_cm &&
                   (!input.heading_valid || heading <= 12.0)) {
            action_phase_ = 2;
        } else if (action_phase_ == 2 &&
                   distance >= p_.bypass_total_distance_cm &&
                   line_good && input.two_side_stable) {
            target_cooldown_s_ = p_.target_cooldown_s;
            transition(DriveState::Follow);
        }
        if (state_time_s_ >= p_.action_timeout_s) {
            stop_reason_ = StopReason::ActionTimeout;
            transition(DriveState::Stopped);
        }
        return;
    }

    if (state_ == DriveState::StraightOver) {
        if (state_distance(input) >= p_.straight_over_distance_cm &&
            line_good) {
            target_cooldown_s_ = p_.target_cooldown_s;
            transition(DriveState::Follow);
        } else if (state_time_s_ >= p_.action_timeout_s) {
            stop_reason_ = StopReason::ActionTimeout;
            transition(DriveState::Stopped);
        }
        return;
    }

    if (state_ != DriveState::Follow) {
        transition(DriveState::Follow);
        return;
    }

    const bool zebra_follow_guard =
        input.zebra || zebra_active_ || zebra_enter_count_ > 0;
    if (zebra_follow_guard && zebra_encounter_count_ < 2) {
        cross_enter_count_ = 0;
        target_enter_count_ = 0;
        return;
    }

    if (cross_refractory_s_ <= 0.0 && input.cross) {
        cross_enter_count_ = std::min(
            p_.cross_enter_frames,
            cross_enter_count_ + 1);
        if (cross_enter_count_ >= p_.cross_enter_frames &&
            cross_heading_ready(input)) {
            enter_cross(input);
            return;
        }
    } else {
        cross_enter_count_ = std::max(0, cross_enter_count_ - 1);
    }

    const bool side_approach_feedback_ready =
        input.heading_valid &&
        input.heading_from_imu &&
        input.distance_valid &&
        input.encoder_heading_valid &&
        !input.ramp_detected &&
        (!p_.side_require_tof ||
         (input.tof_valid && input.tof_baseline_ready));
    const bool side_entry_feedback_ready =
        side_approach_feedback_ready &&
        input.distance_cm >= p_.side_start_distance_cm;
    if (p_.enable_side_road && !side_consumed_ &&
        side_refractory_s_ <= 0.0 &&
        side_entry_feedback_ready &&
        !input.cross &&
        input.roundabout_stage == RoundaboutVisionStage::Inside &&
        (input.roundabout == FeatureSide::Left ||
         input.roundabout == FeatureSide::Right)) {
        enter_side(input);
        return;
    }

    const bool target_candidate =
        p_.enable_target_actions && target_cooldown_s_ <= 0.0 &&
        !input.cross && input.target_valid &&
        input.target_kind != TargetKind::None &&
        input.target_confidence >= p_.target_enter_confidence &&
        (input.target_size >= p_.target_close_size ||
         input.target_kind == TargetKind::Vehicle) &&
        (!p_.target_inertial_routes_enabled() ||
         input.target_center_y_ratio >= p_.target_path_trigger_y);
    if (target_candidate) {
        ++target_enter_count_;
        if (target_enter_count_ >= p_.target_enter_frames) {
            enter_target(input);
        }
    } else {
        target_enter_count_ = 0;
    }
}

void PathController::update_zebra_state(const StepInput& input) {
    if (state_ != DriveState::Follow &&
        state_ != DriveState::ZebraPass) {
        zebra_enter_count_ = 0;
        return;
    }

    if (input.zebra) {
        zebra_exit_count_ = 0;
        if (!zebra_active_) {
            zebra_enter_count_ = std::min(
                p_.zebra_enter_frames, zebra_enter_count_ + 1);
            if (zebra_enter_count_ >= p_.zebra_enter_frames) {
                zebra_active_ = true;
                zebra_enter_count_ = 0;
                zebra_encounter_count_ = std::min(
                    2, zebra_encounter_count_ + 1);
            }
        }
        if (zebra_active_ && zebra_encounter_count_ >= 2 &&
            state_ == DriveState::Follow) {
            transition(DriveState::ZebraPass);
        }
        return;
    }

    zebra_enter_count_ = 0;
    if (!zebra_active_) {
        zebra_exit_count_ = 0;
        return;
    }

    zebra_exit_count_ = std::min(
        p_.zebra_exit_frames, zebra_exit_count_ + 1);
    if (zebra_exit_count_ < p_.zebra_exit_frames) {
        return;
    }

    zebra_active_ = false;
    zebra_exit_count_ = 0;
    if (zebra_encounter_count_ >= 2) {
        stop_reason_ = StopReason::ZebraComplete;
        transition(DriveState::Stopped);
    }
}

void PathController::transition(DriveState next) {
    if (state_ == next) return;
    vision_integral_error_s_ = 0.0;
    vision_derivative_error_per_s_ = 0.0;
    visual_derivative_ready_ = false;
    state_ = next;
    state_time_s_ = 0.0;
    resume_count_ = 0;
    if (next == DriveState::Follow) {
        side_direction_ = 0;
        state_encoder_heading_valid_ = false;
        action_kind_ = TargetKind::None;
        action_phase_ = 0;
        cross_enter_count_ = 0;
        cross_exit_count_ = 0;
        side_exit_count_ = 0;
        target_enter_count_ = 0;
    }
}

void PathController::enter_cross(const StepInput& input) {
    state_start_distance_cm_ = input.distance_cm;
    state_start_heading_deg_ = snapped_cross_heading(input);
    state_heading_valid_ = input.heading_valid;
    cross_exit_count_ = 0;
    transition(DriveState::CrossLock);
}

void PathController::enter_side(const StepInput& input) {
    side_consumed_ = true;
    const int topology_direction =
        input.roundabout == FeatureSide::Right ? 1 : -1;
    side_direction_ =
        p_.vision_yaw_sign < 0.0
            ? -topology_direction : topology_direction;
    side_exit_count_ = 0;
    state_start_distance_cm_ = input.distance_cm;
    state_start_heading_deg_ = input.heading_deg;
    state_heading_valid_ = input.heading_valid;
    state_start_encoder_heading_deg_ = input.encoder_heading_deg;
    state_encoder_heading_valid_ = input.encoder_heading_valid;
    transition(DriveState::Roundabout);
}

void PathController::enter_side_exit() {
    side_exit_count_ = 0;
    transition(DriveState::RoundaboutExit);
}

void PathController::enter_target(const StepInput& input) {
    action_kind_ = input.target_kind;
    action_phase_ = 0;
    state_start_distance_cm_ = input.distance_cm;
    state_start_heading_deg_ = input.heading_deg;
    state_heading_valid_ = input.heading_valid;
    switch (input.target_kind) {
        case TargetKind::Weapon:
            transition(DriveState::BypassLeft);
            break;
        case TargetKind::Supply:
            transition(DriveState::BypassRight);
            break;
        case TargetKind::Vehicle:
            transition(DriveState::StraightOver);
            break;
        default:
            break;
    }
}

PathController::VisionYawBreakdown PathController::vision_yaw_breakdown(
    const StepInput& input) const {
    VisionYawBreakdown result;
    const double near_error = p_.vision_yaw_sign *
        clamp_value(input.line_error, -1.0, 1.0);
    const double far_error = p_.vision_yaw_sign *
        clamp_value(input.far_error, -1.0, 1.0);
    const double center_error = p_.vision_yaw_sign *
        clamp_value(input.vehicle_center_error, -1.0, 1.0);
    const auto curve_gain_for = [this](double error) {
        const double strength = clamp_value(
            (std::abs(error) - 0.16) / 0.24, 0.0, 1.0);
        return 1.0 + p_.curve_yaw_boost * strength *
            std::pow(1.0 + strength, p_.curve_yaw_shape);
    };
    const double near_curve_gain = curve_gain_for(near_error);
    const double far_curve_gain = curve_gain_for(far_error);
    result.curve_gain = std::max(near_curve_gain, far_curve_gain);
    result.near_yaw_rate_dps =
        near_curve_gain * p_.near_yaw_gain * near_error;
    result.far_yaw_rate_dps =
        far_curve_gain * p_.far_yaw_gain * far_error;
    const double curve_strength = clamp_value(
        (std::max(std::abs(near_error), std::abs(far_error)) - 0.16) /
            0.24,
        0.0,
        1.0);
    result.center_yaw_rate_dps =
        (1.0 - curve_strength) *
        p_.center_yaw_weight * p_.near_yaw_gain * center_error;
    result.unclamped_yaw_rate_dps =
        result.near_yaw_rate_dps +
        result.far_yaw_rate_dps +
        result.center_yaw_rate_dps +
        p_.vision_i_gain * vision_integral_error_s_ +
        p_.vision_d_gain * vision_derivative_error_per_s_;
    return result;
}

double PathController::follow_yaw_rate(const StepInput& input) const {
    return clamp_value(
        vision_yaw_breakdown(input).unclamped_yaw_rate_dps,
        -p_.max_yaw_rate_dps, p_.max_yaw_rate_dps);
}

void PathController::update_visual_integral(const StepInput& input,
                                            bool line_good,
                                            double dt) {
    const double near_error = p_.vision_yaw_sign *
        clamp_value(input.line_error, -1.0, 1.0);
    const double far_error = p_.vision_yaw_sign *
        clamp_value(input.far_error, -1.0, 1.0);
    const bool stable_straight_line =
        uses_follow_control(state_) &&
        line_good &&
        input.two_side_stable &&
        !input.cross &&
        input.side_open == FeatureSide::None &&
        std::abs(near_error) <= p_.vision_i_max_error &&
        std::abs(far_error - near_error) <=
            p_.vision_i_curve_delta;
    if (stable_straight_line && p_.vision_i_gain > 0.0) {
        vision_integral_error_s_ = clamp_value(
            vision_integral_error_s_ + near_error * dt,
            -p_.vision_i_limit,
            p_.vision_i_limit);
    } else {
        const double decay = std::exp(-p_.vision_i_decay_rate * dt);
        vision_integral_error_s_ *= decay;
        if (std::abs(vision_integral_error_s_) < 1e-6) {
            vision_integral_error_s_ = 0.0;
        }
    }

    const bool derivative_allowed =
        uses_follow_control(state_) &&
        line_good &&
        !input.cross &&
        input.side_open == FeatureSide::None;
    if (!derivative_allowed || p_.vision_d_gain <= 0.0) {
        vision_derivative_error_per_s_ *=
            std::exp(-dt / p_.vision_d_filter_tau_s);
        previous_visual_error_ = near_error;
        visual_derivative_ready_ = false;
        return;
    }

    if (!visual_derivative_ready_) {
        previous_visual_error_ = near_error;
        visual_derivative_ready_ = true;
        return;
    }

    const double raw_rate = clamp_value(
        (near_error - previous_visual_error_) / dt,
        -p_.vision_d_max_error_rate,
        p_.vision_d_max_error_rate);
    previous_visual_error_ = near_error;
    const double alpha = dt / (p_.vision_d_filter_tau_s + dt);
    vision_derivative_error_per_s_ +=
        alpha * (raw_rate - vision_derivative_error_per_s_);
}

double PathController::state_distance(const StepInput& input) const {
    return std::max(0.0, input.distance_cm - state_start_distance_cm_);
}

double PathController::state_heading_delta(const StepInput& input) const {
    if (!state_heading_valid_ || !input.heading_valid) return 0.0;
    return wrap_degrees(input.heading_deg - state_start_heading_deg_);
}

bool PathController::cross_heading_ready(const StepInput& input) const {
    if (!input.heading_valid || !input.heading_from_imu) {
        return true;
    }
    const double target = snapped_cross_heading(input);
    const double heading_error =
        wrap_degrees(target - input.heading_deg);
    const double visual_yaw = last_yaw_rate_dps_;
    const bool visual_turn_is_significant =
        std::abs(visual_yaw) >= 10.0;
    const bool imu_agrees_with_visual_turn =
        !input.yaw_rate_valid ||
        std::abs(input.yaw_rate_dps) < 5.0 ||
        input.yaw_rate_dps * visual_yaw > 0.0;
    const bool snapped_heading_opposes_visual_turn =
        std::abs(heading_error) >= 10.0 &&
        heading_error * visual_yaw < 0.0;
    // If vision and IMU agree that the car is still turning away from the
    // nearest grid heading, the cross evidence belongs to that turn. Wait
    // until the next grid direction becomes the natural straight-ahead lock.
    if (visual_turn_is_significant &&
        imu_agrees_with_visual_turn &&
        snapped_heading_opposes_visual_turn) {
        return false;
    }
    if (input.yaw_rate_valid &&
        std::abs(input.yaw_rate_dps) >
            p_.cross_enter_max_yaw_rate_dps) {
        const bool converging =
            heading_error * input.yaw_rate_dps > 0.0;
        return converging &&
            std::abs(heading_error) <=
                0.5 * p_.cross_heading_tolerance_deg;
    }
    if (!imu_heading_origin_valid_ ||
        p_.cross_heading_grid_deg <= 0.0) {
        return true;
    }
    return std::abs(heading_error) <=
        p_.cross_heading_tolerance_deg;
}

double PathController::snapped_cross_heading(
        const StepInput& input) const {
    if (!input.heading_valid || !input.heading_from_imu ||
        !imu_heading_origin_valid_ ||
        p_.cross_heading_grid_deg <= 0.0) {
        return input.heading_deg;
    }
    const double relative =
        input.heading_deg - imu_heading_origin_deg_;
    const double snapped_relative =
        std::round(relative / p_.cross_heading_grid_deg) *
        p_.cross_heading_grid_deg;
    return imu_heading_origin_deg_ + snapped_relative;
}

double PathController::wrap_degrees(double angle) {
    while (angle >= 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

}  // namespace rewrite_path
