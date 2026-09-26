#pragma once

#include "path_params.hpp"
#include "path_types.hpp"

#include <memory>
#include <mutex>

#ifndef PATH_FOLLOW_NO_HW
#include "hal.hpp"
#endif

namespace rewrite_path {

class MotorAdapter {
public:
    explicit MotorAdapter(const PathParams& params);
    ~MotorAdapter();

    bool init();
    MotorFeedbackLite read(double dt);
    void apply(const ControlCommand& command);
    void stop();
    void emergency_stop();
    static double rpm_to_cmps(double rpm, double wheel_diameter_cm);

private:
    double fallback_speed(double percent) const;

    const PathParams& p_;
    double left_distance_cm_ = 0.0;
    double right_distance_cm_ = 0.0;
    double left_zero_s_ = 0.0;
    double right_zero_s_ = 0.0;
    double last_left_pct_ = 0.0;
    double last_right_pct_ = 0.0;
    bool emergency_stopped_ = false;
    std::mutex mutex_;

#ifndef PATH_FOLLOW_NO_HW
    std::unique_ptr<smartcar::VehicleHardware> hardware_;
#endif
};

}  // namespace rewrite_path
