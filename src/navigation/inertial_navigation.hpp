#pragma once

#include "path_params.hpp"
#include "path_types.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace rewrite_path {

struct InertialWaypoint {
    double x_cm = 0.0;
    double y_cm = 0.0;
    double heading_deg = 0.0;
    double speed_cmps = 0.0;
};

class InertialTakeoverDirectionGuard {
public:
    explicit InertialTakeoverDirectionGuard(const PathParams& params);

    void capture(const NavigationCommand& visual_command);
    bool apply(NavigationCommand* inertial_command);
    void reset();

    bool active() const { return active_; }
    double takeover_yaw_rate_dps() const { return takeover_yaw_rate_dps_; }

private:
    const PathParams& p_;
    double takeover_yaw_rate_dps_ = 0.0;
    bool active_ = false;
};

class InertialPathNavigator {
public:
    explicit InertialPathNavigator(const PathParams& params);

    bool load(const std::string& path);
    bool set_path(const std::vector<InertialWaypoint>& waypoints);
    void reset();
    bool reanchor_at_current_progress(const OdometrySample& odometry,
                                      const ImuFeedback& imu);

    NavigationCommand update(const OdometrySample& odometry,
                             const ImuFeedback& imu,
                             double dt,
                             bool enforce_faults = true);

    const InertialNavigationStatus& status() const { return status_; }
    const std::string& last_error() const { return error_; }
    bool ready() const { return waypoints_.size() >= 2; }

private:
    struct RouteSample {
        double x_cm = 0.0;
        double y_cm = 0.0;
        double heading_deg = 0.0;
        double speed_cmps = 0.0;
        std::size_t upper_index = 1;
    };

    struct Projection {
        double progress_cm = 0.0;
        double distance_cm = 0.0;
        std::size_t segment_index = 0;
    };

    NavigationCommand stopped_command(StopReason reason) const;
    RouteSample sample_route(double distance_cm) const;
    Projection project(double x_cm, double y_cm) const;
    void capture_origin(const OdometrySample& odometry,
                        const ImuFeedback& imu);
    void local_pose(const OdometrySample& odometry,
                    const ImuFeedback& imu,
                    double* x_cm,
                    double* y_cm,
                    double* heading_deg) const;
    void fail(StopReason reason, const std::string& message);
    static double wrap_degrees(double angle);

    const PathParams& p_;
    std::vector<InertialWaypoint> waypoints_;
    std::vector<double> cumulative_cm_;
    InertialNavigationStatus status_;
    InertialNavState resume_state_ = InertialNavState::Aligning;
    StopReason fault_reason_ = StopReason::None;
    std::string error_;

    std::size_t segment_index_ = 0;
    double progress_cm_ = 0.0;
    double invalid_sensor_s_ = 0.0;
    double excessive_deviation_s_ = 0.0;
    int aligned_frames_ = 0;

    bool origin_valid_ = false;
    double origin_x_cm_ = 0.0;
    double origin_y_cm_ = 0.0;
    double origin_heading_deg_ = 0.0;
};

}  // namespace rewrite_path
