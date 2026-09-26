#pragma once

#include "path_params.hpp"
#include "path_types.hpp"

namespace rewrite_path {

class PathController {
public:
    explicit PathController(const PathParams& params);

    void reset();
    void finish_external_target_action();
    NavigationCommand update(const StepInput& input, double dt);
    DriveState state() const { return state_; }

private:
    struct VisionYawBreakdown {
        double near_yaw_rate_dps = 0.0;
        double far_yaw_rate_dps = 0.0;
        double center_yaw_rate_dps = 0.0;
        double curve_gain = 1.0;
        double unclamped_yaw_rate_dps = 0.0;
    };

    void transition(DriveState next);
    void enter_cross(const StepInput& input);
    void enter_side(const StepInput& input);
    void enter_side_exit();
    void enter_target(const StepInput& input);
    void update_zebra_state(const StepInput& input);
    void update_state(const StepInput& input, bool line_good);
    void update_visual_integral(const StepInput& input,
                                bool line_good,
                                double dt);
    VisionYawBreakdown vision_yaw_breakdown(const StepInput& input) const;
    double follow_yaw_rate(const StepInput& input) const;
    double state_distance(const StepInput& input) const;
    double state_heading_delta(const StepInput& input) const;
    bool cross_heading_ready(const StepInput& input) const;
    double snapped_cross_heading(const StepInput& input) const;
    static double wrap_degrees(double angle);

    const PathParams& p_;
    DriveState state_ = DriveState::Follow;
    StopReason stop_reason_ = StopReason::None;
    TargetKind action_kind_ = TargetKind::None;

    double state_time_s_ = 0.0;
    double lost_s_ = 0.0;
    double pairless_curve_s_ = 0.0;
    double cross_refractory_s_ = 0.0;
    double side_refractory_s_ = 0.0;
    double target_cooldown_s_ = 0.0;
    double state_start_distance_cm_ = 0.0;
    double state_start_heading_deg_ = 0.0;
    bool state_heading_valid_ = false;
    double state_start_encoder_heading_deg_ = 0.0;
    bool state_encoder_heading_valid_ = false;
    double imu_heading_origin_deg_ = 0.0;
    bool imu_heading_origin_valid_ = false;

    double last_yaw_rate_dps_ = 0.0;
    double vision_integral_error_s_ = 0.0;
    double vision_derivative_error_per_s_ = 0.0;
    double previous_visual_error_ = 0.0;
    bool visual_derivative_ready_ = false;
    bool has_clean_line_ = false;
    bool side_consumed_ = false;

    int cross_enter_count_ = 0;
    int cross_exit_count_ = 0;
    int side_exit_count_ = 0;
    int resume_count_ = 0;
    int target_enter_count_ = 0;
    int zebra_enter_count_ = 0;
    int zebra_exit_count_ = 0;
    int zebra_encounter_count_ = 0;
    bool zebra_active_ = false;
    int action_phase_ = 0;
    int side_direction_ = 0;  // -1 left, +1 right. Positive yaw is right.
};

}  // namespace rewrite_path
