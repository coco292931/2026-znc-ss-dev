#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <iterator>
#include <string>
#include <vector>

namespace rewrite_path {

template <typename T>
constexpr T clamp_value(T value, T lo, T hi) {
    return value < lo ? lo : (value > hi ? hi : value);
}

constexpr int kBinaryWidth = 94;
constexpr int kBinaryHeight = 60;
constexpr int kDisplayWidth = 160;
constexpr int kDisplayHeight = 120;

enum class TargetKind {
    None,
    Weapon,
    Supply,
    Vehicle,
};

inline const char* target_kind_name(TargetKind kind) {
    switch (kind) {
        case TargetKind::Weapon:  return "WEAPON_LEFT";
        case TargetKind::Supply:  return "SUPPLY_RIGHT";
        case TargetKind::Vehicle: return "VEHICLE_OVER";
        default:                  return "NONE";
    }
}

inline TargetKind target_kind_from_class(int class_id) {
    switch (class_id) {
        case 0:
        case 1:
            return TargetKind::Weapon;
        case 2:
        case 3:
            return TargetKind::Supply;
        case 4:
        case 5:
            return TargetKind::Vehicle;
        default:
            return TargetKind::None;
    }
}

inline TargetKind target_kind_from_probabilities(
    const std::array<float, 6>& probabilities,
    float* confidence = nullptr) {
    const std::array<float, 3> scores = {{
        probabilities[0] + probabilities[1],
        probabilities[2] + probabilities[3],
        probabilities[4] + probabilities[5],
    }};
    const auto best = std::max_element(scores.begin(), scores.end());
    if (confidence) *confidence = *best;
    const std::array<TargetKind, 3> kinds = {{
        TargetKind::Weapon, TargetKind::Supply, TargetKind::Vehicle,
    }};
    return kinds[static_cast<std::size_t>(std::distance(scores.begin(), best))];
}

enum class DriveState {
    Follow,
    CrossLock,
    Roundabout,
    RoundaboutExit,
    InertialWait,
    InertialAlign,
    InertialTrack,
    BypassLeft,
    BypassRight,
    StraightOver,
    ZebraPass,
    Stopped,
};

inline const char* drive_state_name(DriveState state) {
    switch (state) {
        case DriveState::Follow:       return "FOLLOW";
        case DriveState::CrossLock:    return "CROSS_LOCK";
        case DriveState::Roundabout:   return "ROUNDABOUT";
        case DriveState::RoundaboutExit: return "ROUNDABOUT_EXIT";
        case DriveState::InertialWait: return "INERTIAL_WAIT";
        case DriveState::InertialAlign: return "INERTIAL_ALIGN";
        case DriveState::InertialTrack: return "INERTIAL_TRACK";
        case DriveState::BypassLeft:   return "BYPASS_L";
        case DriveState::BypassRight:  return "BYPASS_R";
        case DriveState::StraightOver: return "STRAIGHT_OVER";
        case DriveState::ZebraPass:    return "ZEBRA_PASS";
        case DriveState::Stopped:      return "STOPPED";
        default:                       return "?";
    }
}

inline bool uses_follow_control(DriveState state) {
    return state == DriveState::Follow || state == DriveState::ZebraPass;
}

enum class RoundaboutVisionStage {
    None,
    Approach,
    Inside,
    Exit,
    Reacquired,
};

inline const char* roundabout_stage_name(RoundaboutVisionStage stage) {
    switch (stage) {
        case RoundaboutVisionStage::Approach:   return "APPROACH";
        case RoundaboutVisionStage::Inside:     return "INSIDE";
        case RoundaboutVisionStage::Exit:       return "EXIT";
        case RoundaboutVisionStage::Reacquired: return "REACQUIRED";
        default:                                return "NONE";
    }
}

enum class StopReason {
    None,
    LineLost,
    SideTimeout,
    ActionTimeout,
    RampDetected,
    TofUnavailable,
    RunawayDetected,
    PathComplete,
    InertialSensorLost,
    PathDeviation,
    ZebraComplete,
};

inline const char* stop_reason_name(StopReason reason) {
    switch (reason) {
        case StopReason::LineLost:      return "LINE_LOST";
        case StopReason::SideTimeout:   return "SIDE_TIMEOUT";
        case StopReason::ActionTimeout: return "ACTION_TIMEOUT";
        case StopReason::RampDetected:  return "RAMP_DETECTED";
        case StopReason::TofUnavailable: return "TOF_UNAVAILABLE";
        case StopReason::RunawayDetected: return "RUNAWAY_DETECTED";
        case StopReason::PathComplete: return "PATH_COMPLETE";
        case StopReason::InertialSensorLost: return "INERTIAL_SENSOR_LOST";
        case StopReason::PathDeviation: return "PATH_DEVIATION";
        case StopReason::ZebraComplete: return "ZEBRA_COMPLETE";
        default:                        return "NONE";
    }
}

enum class SensorMode {
    Full,
    EncoderYaw,
    SingleEncoder,
    OpenLoopEncoders,
    VisionOpenLoop,
};

inline const char* sensor_mode_name(SensorMode mode) {
    switch (mode) {
        case SensorMode::Full:             return "FULL";
        case SensorMode::EncoderYaw:       return "ENCODER_YAW";
        case SensorMode::SingleEncoder:    return "SINGLE_ENCODER";
        case SensorMode::OpenLoopEncoders: return "OPEN_LOOP_ENCODERS";
        case SensorMode::VisionOpenLoop:   return "VISION_OPEN_LOOP";
        default:                           return "?";
    }
}

enum class FeatureSide {
    None,
    Left,
    Right,
    Both,
};

inline const char* feature_side_name(FeatureSide side) {
    switch (side) {
        case FeatureSide::Left:  return "LEFT";
        case FeatureSide::Right: return "RIGHT";
        case FeatureSide::Both:  return "BOTH";
        default:                 return "NONE";
    }
}

struct Point2i {
    int x = 0;
    int y = 0;
};

struct VisionFrame {
    using Row = std::array<std::uint8_t, kBinaryWidth>;

    std::array<Row, kBinaryHeight> gray{};
    std::array<Row, kBinaryHeight> binary{};
};

struct RoadImageInfo {
    int bottom = kBinaryHeight - 1;
    int top = 0;
    int last_mid = kBinaryWidth / 2;
    int white_num = 0;
    int max_column = kBinaryWidth / 2;
    int control_row = 0;
    int far_row = 0;
    int vehicle_anchor_row = 0;
    int left_lost_count = 0;
    int right_lost_count = 0;
    int both_lost_count = 0;
    bool left_straight = false;
    bool right_straight = false;
};

struct CornerPoint {
    int row = 0;
    int col = 0;
    bool valid = false;
};

struct ElementFlags {
    bool cross = false;
    FeatureSide roundabout = FeatureSide::None;
    FeatureSide side_open = FeatureSide::None;
    FeatureSide stable_side = FeatureSide::None;
    RoundaboutVisionStage roundabout_stage = RoundaboutVisionStage::None;
    bool two_side_stable = false;
    int left_branch_count = 0;
    int right_branch_count = 0;
    int left_recovery_count = 0;
    int right_recovery_count = 0;
    bool left_prediction_stable = false;
    bool right_prediction_stable = false;
    bool roundabout_prediction_symmetric = false;
    double left_prediction_mean_error = 0.0;
    double right_prediction_mean_error = 0.0;
    double left_prediction_max_error = 0.0;
    double right_prediction_max_error = 0.0;
    bool ramp = false;
    bool zebra = false;
    bool small_obstacle = false;
    bool red_block = false;
};

// BOOM 连通域拓扑观测结果。
// 只在 --topology 打开时由 TrackTopology 填写；不参与任何巡线或控制计算。
// 行序与网格一致：row 0 = 远端，row (kBinaryHeight-1) = 车头。
struct TopologyRegion {
    // 主信号：该侧边界列在近端范围内出现白色的行数。
    // 正常直道的边界列一定是背景黑，因此 >0 就说明该侧白色区域
    // 超出了主赛道（岔口 / 环岛入口 / 车库 / 或者是反光噪点）。
    int border_white_rows = 0;
    int border_near_row = -1;    // 边界白中最靠车头的行
    int border_far_row = -1;     // 边界白中最远的行
    // 边界白是否属于主赛道连通域（赛道本身顶到了该侧边界）。
    bool border_is_track = false;

    // 边界白中"不属于赛道"的那部分：从边界白出发、只在白色上生长
    // 且不越过赛道得到的区域。
    int area = 0;
    int seed_count = 0;          // 独立的生长起点个数
    int near_row = -1;
    int far_row = -1;
    int min_col = 0;
    int max_col = 0;
    double centroid_col = 0.0;
    double centroid_row = 0.0;
};

struct TopologyReport {
    bool valid = false;   // 赛道连通域有效
    int width = 0;
    int height = 0;
    int seed_col = -1;    // 实际使用的洪水填充起点
    int seed_row = -1;
    int track_area = 0;
    int track_far_row = -1;
    int track_near_row = -1;
    TopologyRegion left;
    TopologyRegion right;
};

struct RoadEstimateLite {
    double line_error = 0.0;
    double far_error = 0.0;
    double vehicle_center_error = 0.0;
    bool bottom_pair_valid = false;
    double line_confidence = 0.0;
    bool line_lost = true;
    int vision_threshold = 0;
    int vision_blue_mask_pixels = 0;
    int vision_color_filter_enabled = 0;
    ElementFlags elements;
    RoadImageInfo info;

    // BOOM 连通域拓扑观测结果（--topology，默认关）。
    TopologyReport topology;

    // 归一化误差的物理厘米换算（--cm-error，默认关）。
    bool cm_scale_valid = false;
    double cm_per_col_control = 0.0;
    double cm_per_col_far = 0.0;
    double line_error_cm = 0.0;
    double far_error_cm = 0.0;
    double vehicle_center_error_cm = 0.0;
    std::array<int, kBinaryHeight> left{};
    std::array<int, kBinaryHeight> right{};
    std::array<int, kBinaryHeight> mid{};
    std::array<std::uint8_t, kBinaryHeight> left_valid{};
    std::array<std::uint8_t, kBinaryHeight> right_valid{};
    std::array<int, kBinaryHeight> width{};
    CornerPoint left_lower;
    CornerPoint left_upper;
    CornerPoint right_lower;
    CornerPoint right_upper;
    CornerPoint roundabout_track_point;
    CornerPoint stable_track_point;
    CornerPoint roundabout_recovery_point;
};

struct StepInput {
    double line_error = 0.0;
    double far_error = 0.0;
    double vehicle_center_error = 0.0;
    double line_confidence = 0.0;
    bool line_lost = true;
    bool bottom_pair_valid = true;

    bool cross = false;
    bool zebra = false;
    FeatureSide roundabout = FeatureSide::None;
    FeatureSide side_open = FeatureSide::None;
    RoundaboutVisionStage roundabout_stage = RoundaboutVisionStage::None;
    bool two_side_stable = false;

    bool target_valid = false;
    TargetKind target_kind = TargetKind::None;
    double target_confidence = 0.0;
    double target_size = 0.0;
    double target_center_y_ratio = 0.0;

    double distance_cm = 0.0;
    bool distance_valid = false;
    double encoder_heading_deg = 0.0;
    bool encoder_heading_valid = false;

    double heading_deg = 0.0;
    bool heading_valid = false;
    bool heading_from_imu = false;
    double yaw_rate_dps = 0.0;
    bool yaw_rate_valid = false;
    bool tof_valid = false;
    bool tof_baseline_ready = false;
    bool ramp_detected = false;
};

struct NavigationCommand {
    DriveState state = DriveState::Follow;
    double target_speed_cmps = 0.0;
    double target_yaw_rate_dps = 0.0;
    double inertial_takeover_yaw_rate_dps = 0.0;
    bool heading_hold = false;
    bool inertial_direction_guard_active = false;
    double target_heading_deg = 0.0;
    double vision_integral_error_s = 0.0;
    double vision_integral_yaw_rate_dps = 0.0;
    double vision_derivative_error_per_s = 0.0;
    double vision_derivative_yaw_rate_dps = 0.0;
    double vision_near_yaw_rate_dps = 0.0;
    double vision_far_yaw_rate_dps = 0.0;
    double vision_center_yaw_rate_dps = 0.0;
    double vision_curve_gain = 1.0;
    double vision_unclamped_yaw_rate_dps = 0.0;
    bool line_good = false;
    StopReason stop_reason = StopReason::None;
    TargetKind action_kind = TargetKind::None;
    double power_boost_percent = 0.0;
    int cross_score = 0;
    bool zebra_active = false;
    int zebra_encounter_count = 0;
};

struct ControlCommand {
    double left_percent = 0.0;
    double right_percent = 0.0;
};

struct MotorFeedbackLite {
    double left_rpm = 0.0;
    double right_rpm = 0.0;
    double left_speed_cmps = 0.0;
    double right_speed_cmps = 0.0;
    double average_speed_cmps = 0.0;
    double left_distance_cm = 0.0;
    double right_distance_cm = 0.0;
    double distance_cm = 0.0;
    bool left_valid = false;
    bool right_valid = false;
    bool distance_valid = false;
};

struct ImuFeedback {
    double yaw_rate_dps = 0.0;
    double heading_deg = 0.0;
    double raw_accel_x_g = 0.0;
    double raw_accel_y_g = 0.0;
    double raw_accel_z_g = 0.0;
    double forward_accel_mps2 = 0.0;
    double right_accel_mps2 = 0.0;
    double velocity_x_mps = 0.0;
    double velocity_y_mps = 0.0;
    double position_x_m = 0.0;
    double position_y_m = 0.0;
    double age_s = 0.0;
    bool valid = false;
    bool stationary = false;
};

struct TofDistanceSample {
    double distance_mm = 0.0;
    double age_s = 0.0;
    std::uint64_t sequence = 0;
    bool valid = false;
};

struct TofSlopeFeedback {
    double distance_mm = 0.0;
    double filtered_distance_mm = 0.0;
    double baseline_distance_mm = 0.0;
    double signed_delta_mm = 0.0;
    double age_s = 0.0;
    bool sensor_started = false;
    bool valid = false;
    bool baseline_ready = false;
    bool ramp_detected = false;
};

struct OdometrySample {
    // Startup map frame: +X forward, +Y right, heading right-positive.
    double x_cm = 0.0;
    double y_cm = 0.0;
    double heading_deg = 0.0;
    double distance_cm = 0.0;
    bool heading_from_imu = false;
    bool valid = false;
};

enum class InertialNavState {
    Disabled,
    WaitingForSensors,
    Aligning,
    Tracking,
    Complete,
    Fault,
};

inline const char* inertial_nav_state_name(InertialNavState state) {
    switch (state) {
        case InertialNavState::WaitingForSensors: return "WAITING_FOR_SENSORS";
        case InertialNavState::Aligning: return "ALIGNING";
        case InertialNavState::Tracking: return "TRACKING";
        case InertialNavState::Complete: return "COMPLETE";
        case InertialNavState::Fault: return "FAULT";
        default: return "DISABLED";
    }
}

struct InertialNavigationStatus {
    InertialNavState state = InertialNavState::Disabled;
    std::size_t waypoint_index = 0;
    std::size_t waypoint_count = 0;
    bool takeover_reanchored = false;
    double progress_cm = 0.0;
    double path_length_cm = 0.0;
    double cross_track_error_cm = 0.0;
    double heading_error_deg = 0.0;
    double target_x_cm = 0.0;
    double target_y_cm = 0.0;
};

struct ControlDiagnostics {
    SensorMode sensor_mode = SensorMode::VisionOpenLoop;
    double speed_limit_cmps = 0.0;
    double requested_speed_cmps = 0.0;
    double limited_speed_cmps = 0.0;
    double requested_yaw_rate_dps = 0.0;
    double measured_yaw_rate_dps = 0.0;
    double raw_encoder_yaw_rate_dps = 0.0;
    double filtered_encoder_yaw_rate_dps = 0.0;
    int yaw_sensor_disagreement_frames = 0;
    double left_target_cmps = 0.0;
    double right_target_cmps = 0.0;
    double left_error_cmps = 0.0;
    double right_error_cmps = 0.0;
    double target_steering_utilization = 0.0;
    double pwm_steering_utilization = 0.0;
    double power_boost_percent = 0.0;
    bool left_closed_loop = false;
    bool right_closed_loop = false;
    bool imu_yaw_rejected = false;
    bool yaw_direction_guard_active = false;
    bool saturated = false;
};

struct ControlResult {
    ControlCommand command;
    ControlDiagnostics diagnostics;
};

struct TelemetrySample {
    double elapsed_s = 0.0;
    bool recognition_valid = false;
    TargetKind recognition_kind = TargetKind::None;
    double recognition_confidence = 0.0;
    double recognition_size = 0.0;
    double recognition_center_y_ratio = 0.0;
    int target_encounter = 0;
    bool target_route_active = false;
    RoadEstimateLite road;
    NavigationCommand navigation;
    ControlResult control;
    MotorFeedbackLite motor;
    ImuFeedback imu;
    TofSlopeFeedback tof;
    OdometrySample odometry;
    InertialNavigationStatus inertial_navigation;
    bool target_recognition_enabled = false;
    bool target_recognition_valid = false;
    TargetKind target_recognition_kind = TargetKind::None;
    double target_recognition_confidence = 0.0;
    double target_recognition_size = 0.0;
    double target_recognition_center_y_ratio = 0.0;
    int target_recognition_x = 0;
    int target_recognition_y = 0;
    int target_recognition_w = 0;
    int target_recognition_h = 0;
};

}  // namespace rewrite_path
