#include "inertial_navigation.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

namespace rewrite_path {

namespace {

constexpr double kRadiansPerDegree = 0.01745329251994329577;
constexpr double kDegreesPerRadian = 57.2957795130823208768;

std::string trim(std::string value) {
    const std::size_t begin = value.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) return std::string();
    const std::size_t end = value.find_last_not_of(" \t\r\n");
    return value.substr(begin, end - begin + 1);
}

std::vector<std::string> split_csv(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) {
        fields.push_back(trim(field));
    }
    return fields;
}

int column_index(const std::vector<std::string>& header,
                 const char* name) {
    for (std::size_t i = 0; i < header.size(); ++i) {
        if (header[i] == name) return static_cast<int>(i);
    }
    return -1;
}

bool parse_number(const std::vector<std::string>& fields,
                  int index,
                  double default_value,
                  double* value) {
    if (index < 0) {
        *value = default_value;
        return true;
    }
    if (static_cast<std::size_t>(index) >= fields.size()) return false;
    try {
        std::size_t consumed = 0;
        const double parsed = std::stod(fields[index], &consumed);
        if (consumed != fields[index].size() || !std::isfinite(parsed)) {
            return false;
        }
        *value = parsed;
        return true;
    } catch (...) {
        return false;
    }
}

}  // namespace

InertialPathNavigator::InertialPathNavigator(const PathParams& params)
    : p_(params) {}

InertialTakeoverDirectionGuard::InertialTakeoverDirectionGuard(
    const PathParams& params)
    : p_(params) {}

void InertialTakeoverDirectionGuard::capture(
    const NavigationCommand& visual_command) {
    double yaw_rate = visual_command.target_yaw_rate_dps;
    const double significant_yaw_rate = std::min(
        5.0, std::max(1.0, p_.yaw_direction_guard_target_dps));
    if (std::abs(yaw_rate) < significant_yaw_rate) {
        yaw_rate = visual_command.vision_unclamped_yaw_rate_dps;
    }
    if (std::abs(yaw_rate) < significant_yaw_rate) {
        reset();
        return;
    }
    takeover_yaw_rate_dps_ = clamp_value(
        yaw_rate, -p_.max_yaw_rate_dps, p_.max_yaw_rate_dps);
    active_ = true;
}

bool InertialTakeoverDirectionGuard::apply(
    NavigationCommand* inertial_command) {
    if (!active_ || inertial_command == nullptr ||
        inertial_command->state == DriveState::Stopped) {
        return false;
    }
    const double requested_yaw = inertial_command->target_yaw_rate_dps;
    if (std::abs(requested_yaw) >= 1.0 &&
        requested_yaw * takeover_yaw_rate_dps_ > 0.0) {
        reset();
        return false;
    }
    inertial_command->target_yaw_rate_dps = takeover_yaw_rate_dps_;
    inertial_command->target_speed_cmps = std::min(
        inertial_command->target_speed_cmps,
        p_.inertial_min_speed_cmps);
    inertial_command->heading_hold = false;
    inertial_command->inertial_direction_guard_active = true;
    inertial_command->inertial_takeover_yaw_rate_dps =
        takeover_yaw_rate_dps_;
    return true;
}

void InertialTakeoverDirectionGuard::reset() {
    takeover_yaw_rate_dps_ = 0.0;
    active_ = false;
}

bool InertialPathNavigator::load(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        error_ = "cannot open inertial path: " + path;
        return false;
    }

    std::vector<std::string> header;
    std::vector<InertialWaypoint> points;
    int x_column = -1;
    int y_column = -1;
    int heading_column = -1;
    int speed_column = -1;
    std::string line;
    int line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;
        const std::vector<std::string> fields = split_csv(line);
        if (header.empty()) {
            header = fields;
            x_column = column_index(header, "x_cm");
            y_column = column_index(header, "y_cm");
            heading_column = column_index(header, "heading_deg");
            speed_column = column_index(header, "speed_cmps");
            if (x_column < 0 || y_column < 0) {
                error_ = "inertial path header requires x_cm,y_cm";
                return false;
            }
            continue;
        }

        InertialWaypoint point;
        if (!parse_number(fields, x_column, 0.0, &point.x_cm) ||
            !parse_number(fields, y_column, 0.0, &point.y_cm) ||
            !parse_number(fields, heading_column, 0.0, &point.heading_deg) ||
            !parse_number(fields, speed_column, p_.base_speed_cmps,
                          &point.speed_cmps)) {
            error_ = "invalid numeric value in inertial path line " +
                std::to_string(line_number);
            return false;
        }
        points.push_back(point);
    }
    return set_path(points);
}

bool InertialPathNavigator::set_path(
    const std::vector<InertialWaypoint>& waypoints) {
    error_.clear();
    waypoints_.clear();
    cumulative_cm_.clear();
    if (waypoints.size() < 2) {
        error_ = "inertial path requires at least two waypoints";
        return false;
    }

    const InertialWaypoint& origin = waypoints.front();
    const double angle = origin.heading_deg * kRadiansPerDegree;
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    for (const InertialWaypoint& source : waypoints) {
        if (!std::isfinite(source.x_cm) || !std::isfinite(source.y_cm) ||
            !std::isfinite(source.heading_deg) ||
            !std::isfinite(source.speed_cmps)) {
            error_ = "inertial path contains a non-finite waypoint";
            waypoints_.clear();
            return false;
        }
        const double dx = source.x_cm - origin.x_cm;
        const double dy = source.y_cm - origin.y_cm;
        InertialWaypoint point;
        point.x_cm = cosine * dx + sine * dy;
        point.y_cm = -sine * dx + cosine * dy;
        point.heading_deg = wrap_degrees(
            source.heading_deg - origin.heading_deg);
        point.speed_cmps = source.speed_cmps;
        if (!waypoints_.empty()) {
            const double gap = std::hypot(
                point.x_cm - waypoints_.back().x_cm,
                point.y_cm - waypoints_.back().y_cm);
            if (gap < 0.05) {
                waypoints_.back() = point;
                continue;
            }
        }
        waypoints_.push_back(point);
    }
    if (waypoints_.size() < 2) {
        error_ = "inertial path has no measurable length";
        waypoints_.clear();
        return false;
    }

    cumulative_cm_.reserve(waypoints_.size());
    cumulative_cm_.push_back(0.0);
    for (std::size_t i = 1; i < waypoints_.size(); ++i) {
        cumulative_cm_.push_back(
            cumulative_cm_.back() +
            std::hypot(waypoints_[i].x_cm - waypoints_[i - 1].x_cm,
                       waypoints_[i].y_cm - waypoints_[i - 1].y_cm));
    }
    if (cumulative_cm_.back() < 1.0) {
        error_ = "inertial path length is less than 1 cm";
        waypoints_.clear();
        cumulative_cm_.clear();
        return false;
    }
    reset();
    return true;
}

void InertialPathNavigator::reset() {
    segment_index_ = 0;
    progress_cm_ = 0.0;
    invalid_sensor_s_ = 0.0;
    excessive_deviation_s_ = 0.0;
    aligned_frames_ = 0;
    origin_valid_ = false;
    fault_reason_ = StopReason::None;
    resume_state_ = InertialNavState::Aligning;
    status_ = InertialNavigationStatus{};
    status_.state = ready()
        ? InertialNavState::WaitingForSensors
        : InertialNavState::Disabled;
    status_.waypoint_count = waypoints_.size();
    status_.path_length_cm =
        cumulative_cm_.empty() ? 0.0 : cumulative_cm_.back();
}

bool InertialPathNavigator::reanchor_at_current_progress(
    const OdometrySample& odometry,
    const ImuFeedback& imu) {
    if (!ready() || !odometry.valid || !imu.valid) return false;

    const RouteSample anchor = sample_route(progress_cm_);
    origin_heading_deg_ = wrap_degrees(
        imu.heading_deg - anchor.heading_deg);
    const double angle = origin_heading_deg_ * kRadiansPerDegree;
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    const double anchor_world_x =
        cosine * anchor.x_cm - sine * anchor.y_cm;
    const double anchor_world_y =
        sine * anchor.x_cm + cosine * anchor.y_cm;
    origin_x_cm_ = odometry.x_cm - anchor_world_x;
    origin_y_cm_ = odometry.y_cm - anchor_world_y;
    origin_valid_ = true;

    invalid_sensor_s_ = 0.0;
    excessive_deviation_s_ = 0.0;
    aligned_frames_ = 3;
    fault_reason_ = StopReason::None;
    error_.clear();
    resume_state_ = InertialNavState::Tracking;
    status_.state = InertialNavState::Tracking;
    status_.waypoint_index = anchor.upper_index;
    status_.takeover_reanchored = true;
    status_.progress_cm = progress_cm_;
    status_.cross_track_error_cm = 0.0;
    status_.heading_error_deg = 0.0;
    status_.target_x_cm = anchor.x_cm;
    status_.target_y_cm = anchor.y_cm;
    return true;
}

NavigationCommand InertialPathNavigator::stopped_command(
    StopReason reason) const {
    NavigationCommand command;
    command.state = status_.state == InertialNavState::WaitingForSensors
        ? DriveState::InertialWait : DriveState::Stopped;
    command.stop_reason = reason;
    return command;
}

void InertialPathNavigator::capture_origin(const OdometrySample& odometry,
                                           const ImuFeedback& imu) {
    origin_x_cm_ = odometry.x_cm;
    origin_y_cm_ = odometry.y_cm;
    origin_heading_deg_ = imu.heading_deg;
    origin_valid_ = true;
}

void InertialPathNavigator::local_pose(const OdometrySample& odometry,
                                       const ImuFeedback& imu,
                                       double* x_cm,
                                       double* y_cm,
                                       double* heading_deg) const {
    const double angle = origin_heading_deg_ * kRadiansPerDegree;
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    const double dx = odometry.x_cm - origin_x_cm_;
    const double dy = odometry.y_cm - origin_y_cm_;
    *x_cm = cosine * dx + sine * dy;
    *y_cm = -sine * dx + cosine * dy;
    *heading_deg = wrap_degrees(imu.heading_deg - origin_heading_deg_);
}

InertialPathNavigator::Projection InertialPathNavigator::project(
    double x_cm, double y_cm) const {
    Projection best;
    best.distance_cm = std::numeric_limits<double>::infinity();
    if (waypoints_.size() < 2) return best;

    const std::size_t begin = segment_index_ > 0 ? segment_index_ - 1 : 0;
    const std::size_t end = std::min(
        waypoints_.size() - 2, segment_index_ + 80);
    for (std::size_t i = begin; i <= end; ++i) {
        const double dx = waypoints_[i + 1].x_cm - waypoints_[i].x_cm;
        const double dy = waypoints_[i + 1].y_cm - waypoints_[i].y_cm;
        const double length_squared = dx * dx + dy * dy;
        if (length_squared <= 1e-9) continue;
        const double t = clamp_value(
            ((x_cm - waypoints_[i].x_cm) * dx +
             (y_cm - waypoints_[i].y_cm) * dy) / length_squared,
            0.0, 1.0);
        const double projected_x = waypoints_[i].x_cm + t * dx;
        const double projected_y = waypoints_[i].y_cm + t * dy;
        const double distance = std::hypot(
            x_cm - projected_x, y_cm - projected_y);
        const double progress = cumulative_cm_[i] +
            t * std::sqrt(length_squared);
        if (progress + p_.inertial_lookahead_cm < progress_cm_) continue;
        const double maximum_forward_projection = progress_cm_ + std::max(
            3.0 * p_.inertial_lookahead_cm, 100.0);
        if (progress > maximum_forward_projection) continue;
        if (distance < best.distance_cm) {
            best.distance_cm = distance;
            best.progress_cm = progress;
            best.segment_index = i;
        }
    }
    return best;
}

InertialPathNavigator::RouteSample InertialPathNavigator::sample_route(
    double distance_cm) const {
    RouteSample sample;
    distance_cm = clamp_value(distance_cm, 0.0, cumulative_cm_.back());
    const auto upper = std::upper_bound(
        cumulative_cm_.begin(), cumulative_cm_.end(), distance_cm);
    const std::size_t index = upper == cumulative_cm_.end()
        ? waypoints_.size() - 1
        : static_cast<std::size_t>(upper - cumulative_cm_.begin());
    const std::size_t lower = index > 0 ? index - 1 : 0;
    const double span = cumulative_cm_[index] - cumulative_cm_[lower];
    const double ratio = span > 1e-9
        ? (distance_cm - cumulative_cm_[lower]) / span : 0.0;
    sample.x_cm = waypoints_[lower].x_cm +
        ratio * (waypoints_[index].x_cm - waypoints_[lower].x_cm);
    sample.y_cm = waypoints_[lower].y_cm +
        ratio * (waypoints_[index].y_cm - waypoints_[lower].y_cm);
    sample.heading_deg = waypoints_[lower].heading_deg + ratio *
        wrap_degrees(waypoints_[index].heading_deg -
                     waypoints_[lower].heading_deg);
    sample.speed_cmps = waypoints_[lower].speed_cmps +
        ratio * (waypoints_[index].speed_cmps -
                 waypoints_[lower].speed_cmps);
    sample.upper_index = index;
    return sample;
}

void InertialPathNavigator::fail(StopReason reason,
                                 const std::string& message) {
    fault_reason_ = reason;
    error_ = message;
    status_.state = InertialNavState::Fault;
}

NavigationCommand InertialPathNavigator::update(
    const OdometrySample& odometry,
    const ImuFeedback& imu,
    double dt,
    bool enforce_faults) {
    dt = clamp_value(dt, 0.001, 0.10);
    if (!ready()) return stopped_command(StopReason::InertialSensorLost);
    if (status_.state == InertialNavState::Complete) {
        return stopped_command(StopReason::PathComplete);
    }
    if (status_.state == InertialNavState::Fault) {
        return stopped_command(fault_reason_);
    }

    const bool sensors_valid = imu.valid && odometry.valid;
    if (!sensors_valid) {
        invalid_sensor_s_ += dt;
        if (status_.state != InertialNavState::WaitingForSensors) {
            resume_state_ = status_.state;
            status_.state = InertialNavState::WaitingForSensors;
        }
        if (enforce_faults &&
            invalid_sensor_s_ >= p_.inertial_sensor_timeout_s) {
            fail(StopReason::InertialSensorLost,
                 "IMU or encoder odometry became unavailable");
            return stopped_command(fault_reason_);
        }
        return stopped_command(StopReason::None);
    }
    invalid_sensor_s_ = 0.0;
    if (!origin_valid_) {
        capture_origin(odometry, imu);
        resume_state_ = InertialNavState::Aligning;
    }
    if (status_.state == InertialNavState::WaitingForSensors) {
        status_.state = resume_state_;
    }

    double x_cm = 0.0;
    double y_cm = 0.0;
    double heading_deg = 0.0;
    local_pose(odometry, imu, &x_cm, &y_cm, &heading_deg);
    const Projection projection = project(x_cm, y_cm);
    if (!std::isfinite(projection.distance_cm)) {
        fail(StopReason::PathDeviation, "cannot project pose onto path");
        return stopped_command(fault_reason_);
    }
    progress_cm_ = std::max(progress_cm_, projection.progress_cm);
    segment_index_ = std::max(segment_index_, projection.segment_index);

    status_.progress_cm = progress_cm_;
    status_.cross_track_error_cm = projection.distance_cm;
    if (enforce_faults &&
        projection.distance_cm > p_.inertial_max_deviation_cm) {
        excessive_deviation_s_ += dt;
    } else {
        excessive_deviation_s_ = 0.0;
    }
    if (enforce_faults &&
        excessive_deviation_s_ >= p_.inertial_deviation_timeout_s) {
        fail(StopReason::PathDeviation,
             "cross-track error exceeded configured limit");
        return stopped_command(fault_reason_);
    }

    const double remaining_cm = cumulative_cm_.back() - progress_cm_;
    const InertialWaypoint& final = waypoints_.back();
    const double final_distance = std::hypot(
        final.x_cm - x_cm, final.y_cm - y_cm);
    if (remaining_cm <= p_.inertial_finish_tolerance_cm &&
        final_distance <= p_.inertial_finish_tolerance_cm) {
        progress_cm_ = cumulative_cm_.back();
        status_.progress_cm = progress_cm_;
        status_.waypoint_index = waypoints_.size() - 1;
        status_.state = InertialNavState::Complete;
        return stopped_command(StopReason::PathComplete);
    }

    const RouteSample target = sample_route(
        progress_cm_ + p_.inertial_lookahead_cm);
    status_.waypoint_index = target.upper_index;
    status_.target_x_cm = target.x_cm;
    status_.target_y_cm = target.y_cm;
    const double target_heading = std::atan2(
        target.y_cm - y_cm, target.x_cm - x_cm) * kDegreesPerRadian;
    const double heading_error = wrap_degrees(
        target_heading - heading_deg);
    status_.heading_error_deg = heading_error;

    if (status_.state == InertialNavState::Aligning) {
        if (std::abs(heading_error) <= p_.inertial_align_tolerance_deg) {
            ++aligned_frames_;
        } else {
            aligned_frames_ = 0;
        }
        if (aligned_frames_ >= 3) {
            status_.state = InertialNavState::Tracking;
        }
    }

    NavigationCommand command;
    command.state = status_.state == InertialNavState::Aligning
        ? DriveState::InertialAlign : DriveState::InertialTrack;
    command.line_good = true;
    command.target_heading_deg = target_heading + origin_heading_deg_;
    command.target_yaw_rate_dps = clamp_value(
        p_.inertial_heading_kp * heading_error,
        -p_.max_yaw_rate_dps, p_.max_yaw_rate_dps);

    double speed = p_.inertial_speed_cmps > 0.0
        ? p_.inertial_speed_cmps : target.speed_cmps;
    if (speed <= 0.0) speed = p_.base_speed_cmps;
    speed = clamp_value(speed, p_.inertial_min_speed_cmps,
                        p_.max_speed_cmps);
    if (status_.state == InertialNavState::Aligning) {
        speed = std::min(speed, p_.inertial_min_speed_cmps);
    }
    const double stopping_distance = std::max(
        2.0 * p_.inertial_finish_tolerance_cm,
        p_.inertial_lookahead_cm);
    if (remaining_cm < stopping_distance) {
        speed *= clamp_value(
            remaining_cm / stopping_distance, 0.25, 1.0);
    }
    command.target_speed_cmps = speed;
    return command;
}

double InertialPathNavigator::wrap_degrees(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

}  // namespace rewrite_path
