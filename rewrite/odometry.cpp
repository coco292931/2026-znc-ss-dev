#include "odometry.hpp"

#include <cmath>

namespace rewrite_path {

namespace {

constexpr double kRadiansPerDegree = 0.01745329251994329577;

}  // namespace

PlanarOdometry::PlanarOdometry(const PathParams& params) : p_(params) {}

void PlanarOdometry::reset() {
    sample_ = OdometrySample{};
    previous_left_cm_ = 0.0;
    previous_right_cm_ = 0.0;
    previous_heading_deg_ = 0.0;
    initialized_ = false;
}

OdometrySample PlanarOdometry::update(const MotorFeedbackLite& motor,
                                      const ImuFeedback& imu) {
    if (!initialized_) {
        previous_left_cm_ = motor.left_distance_cm;
        previous_right_cm_ = motor.right_distance_cm;
        previous_heading_deg_ = imu.valid ? imu.heading_deg : 0.0;
        sample_.heading_deg = previous_heading_deg_;
        initialized_ = true;
        return sample_;
    }

    const double delta_left =
        motor.left_distance_cm - previous_left_cm_;
    const double delta_right =
        motor.right_distance_cm - previous_right_cm_;
    previous_left_cm_ = motor.left_distance_cm;
    previous_right_cm_ = motor.right_distance_cm;

    double heading = previous_heading_deg_;
    if (imu.valid) {
        heading = imu.heading_deg;
        sample_.heading_from_imu = true;
    } else if (motor.left_valid && motor.right_valid) {
        heading += (delta_left - delta_right) /
            std::max(1.0, p_.wheel_base_cm) / kRadiansPerDegree;
        sample_.heading_from_imu = false;
    }

    const double delta_distance = 0.5 * (delta_left + delta_right);
    const double middle_heading =
        0.5 * (previous_heading_deg_ + heading) * kRadiansPerDegree;
    sample_.x_cm += delta_distance * std::cos(middle_heading);
    // Heading/yaw is positive clockwise (right turn). The map frame follows
    // the same handed convention: +X forward and +Y to the vehicle's right.
    sample_.y_cm += delta_distance * std::sin(middle_heading);
    sample_.distance_cm += std::abs(delta_distance);
    sample_.heading_deg = heading;
    sample_.valid =
        motor.left_valid || motor.right_valid || std::abs(delta_distance) > 1e-6;
    previous_heading_deg_ = heading;
    return sample_;
}

}  // namespace rewrite_path
