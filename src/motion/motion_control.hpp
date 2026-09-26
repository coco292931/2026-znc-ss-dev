#pragma once

#include "path_params.hpp"
#include "path_types.hpp"

namespace rewrite_path {

class RunawayStopGuard {
public:
    bool update(const MotorFeedbackLite& feedback,
                const ImuFeedback& imu,
                const NavigationCommand& navigation);
    bool apply(NavigationCommand* navigation) const;
    bool latched() const { return latched_; }

private:
    int wheel_overspeed_frames_ = 0;
    int spin_frames_ = 0;
    bool latched_ = false;
};

class MotionController {
public:
    explicit MotionController(const PathParams& params);

    void reset();
    ControlResult update(const NavigationCommand& navigation,
                         const MotorFeedbackLite& feedback,
                         const ImuFeedback& imu,
                         double dt);

private:
    double wheel_output(double target_cmps,
                        double measured_cmps,
                        bool closed_loop,
                        double power_boost_percent,
                        double* integral,
                        double dt);
    double dead_zone(double percent,
                     double measured_cmps,
                     bool closed_loop) const;
    double ramp(double wanted, double previous, double dt,
                double rise_scale) const;
    double speed_limit_for(SensorMode mode) const;
    static double wrap_degrees(double angle);

    const PathParams& p_;
    double left_integral_ = 0.0;
    double right_integral_ = 0.0;
    double limited_speed_cmps_ = 0.0;
    double motion_time_s_ = 0.0;
    double last_left_percent_ = 0.0;
    double last_right_percent_ = 0.0;
    double filtered_encoder_yaw_rate_dps_ = 0.0;
    double limited_target_yaw_rate_dps_ = 0.0;
    double smoothed_speed_limit_cmps_ = 0.0;
    double last_significant_target_yaw_sign_ = 0.0;
    int imu_yaw_disagreement_frames_ = 0;
    int imu_yaw_recovery_frames_ = 0;
    bool imu_yaw_rejected_ = false;
    bool target_yaw_rate_ready_ = false;
};

}  // namespace rewrite_path
