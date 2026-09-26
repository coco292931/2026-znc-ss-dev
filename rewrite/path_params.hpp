#pragma once

#include <cstdint>
#include <string>

namespace rewrite_path {

enum class TargetKind;

struct PathParams {
    std::string camera_device = "/dev/video0";
    std::string model_dir = ".";
    int camera_width = 320;
    int camera_height = 240;
    int camera_fps = 60;
    int control_hz = 50;
    bool rotate_180 = true;

    bool use_http = false;
    std::uint16_t http_port = 8080;
    int http_fps_limit = 20;
    int http_jpeg_quality = 60;

    bool dry_run = true;
    bool use_ncnn = true;
    bool enable_stop = true;
    bool enable_side_road = false;
    bool enable_target_actions = false;
    int target_input_size = 32;

    double base_speed_cmps = 35.0;
    double max_speed_cmps = 40.0;
    double hard_turn_outer_speed_scale = 1.50;
    double target_accel_cmps2 = 25.0;
    double target_decel_cmps2 = 60.0;
    double target_yaw_slew_dps2 = 840.0;

    double min_move_percent = 4.0;
    double max_percent = 30.0;
    double max_percent_delta_per_s = 180.0;
    double startup_ramp_s = 0.65;
    double pwm_frequency_hz = 10000.0;
    bool flip_motors = true;
    bool swap_motors = false;
    bool swap_encoders = false;
    bool forbid_reverse = true;

    double min_line_confidence = 0.18;
    double lost_stop_s = 1.2;
    int resume_stable_frames = 5;
    double vision_yaw_sign = 1.0;
    double vision_i_gain = 0.0;
    double vision_i_limit = 0.35;
    double vision_i_max_error = 0.28;
    double vision_i_curve_delta = 0.14;
    double vision_i_decay_rate = 3.0;
    double vision_d_gain = 0.0;
    double vision_d_filter_tau_s = 0.10;
    double vision_d_max_error_rate = 1.25;
    double near_yaw_gain = 58.0;
    double center_yaw_weight = 0.75;
    double far_yaw_gain = 82.0;
    double curve_yaw_boost = 0.0;
    double curve_yaw_shape = 3.0;
    double max_yaw_rate_dps = 120.0;
    double lost_yaw_decay = 0.82;
    double lost_speed_scale = 0.38;
    double curvature_slowdown = 0.52;
    double minimum_follow_speed_scale = 0.38;
    double minimum_follow_yaw_scale = 0.80;
    double confidence_slowdown = 0.35;

    double yaw_rate_kp = 0.045;
    double yaw_rate_correction_limit_cmps = 22.0;
    double encoder_yaw_filter_tau_s = 0.20;
    double encoder_yaw_kp_scale = 0.10;
    double encoder_yaw_correction_limit_cmps = 5.0;
    double yaw_sensor_disagreement_dps = 35.0;
    int yaw_sensor_disagreement_frames = 3;
    int yaw_sensor_recovery_frames = 12;
    double yaw_direction_guard_dps = 20.0;
    double yaw_direction_guard_target_dps = 12.0;
    double heading_hold_kp = 3.0;
    double heading_hold_max_rate_dps = 90.0;

    double speed_ff = 0.70;
    double wheel_speed_kp = 0.35;
    double wheel_speed_ki = 0.90;
    double wheel_speed_i_limit = 20.0;
    double inner_wheel_brake_max_percent = 12.0;
    double inner_wheel_brake_margin_cmps = 8.0;

    double wheel_diameter_cm = 6.5;
    double wheel_base_cm = 15.3;
    int encoder_lines = 1024;
    double encoder_gear_ratio = 30.0 / 68.0;
    double encoder_filter_alpha = 0.35;
    double encoder_zero_timeout_s = 0.45;
    double encoder_recover_rpm = 2.0;

    double imu_yaw_sign = 1.0;
    std::string imu_transport = "auto";
    int imu_spi_speed_hz = 8000000;
    double imu_calibrate_s = 1.2;
    double imu_max_age_s = 0.20;
    double imu_deadband_dps = 1.2;
    double imu_bias_adapt_rate = 0.04;
    bool imu_accel_swap_xy = false;
    double imu_accel_forward_sign = 1.0;
    double imu_accel_right_sign = 1.0;
    double imu_accel_deadband_g = 0.002;
    bool imu_stationary_zero = true;
    double imu_stationary_accel_g = 0.02;
    double imu_stationary_yaw_dps = 2.5;
    double imu_stationary_hold_s = 0.25;
    std::string imu_csv_path;
    int imu_csv_hz = 50;

    // Optional route replay using IMU heading plus encoder odometry.
    std::string inertial_path;
    bool line_lost_inertial = false;
    double inertial_lookahead_cm = 25.0;
    double inertial_speed_cmps = 0.0;  // 0 uses the recorded waypoint speed.
    double inertial_min_speed_cmps = 10.0;
    double inertial_heading_kp = 2.5;
    double inertial_align_tolerance_deg = 8.0;
    double inertial_finish_tolerance_cm = 10.0;
    double inertial_max_deviation_cm = 60.0;
    double inertial_deviation_timeout_s = 0.5;
    double inertial_sensor_timeout_s = 0.35;

    bool enable_tof_slope = false;
    bool tof_stop_test = false;
    int tof_baseline_frames = 10;
    double tof_baseline_timeout_s = 10.0;
    int tof_ramp_enter_frames = 3;
    int tof_ramp_exit_frames = 5;
    double tof_max_age_s = 0.30;
    double tof_ramp_sign = -1.0;
    double tof_ramp_delta_mm = 400.0;
    double tof_ramp_hysteresis_mm = 35.0;
    double ramp_boost_percent = 10.0;
    double ramp_boost_speed_cmps = 68.0;
    double ramp_boost_max_s = 4.0;

    double imu_fail_speed_cmps = 25.0;
    double single_encoder_speed_cmps = 20.0;
    double encoder_fail_speed_cmps = 15.0;
    double all_sensor_fail_speed_cmps = 12.0;

    int cross_enter_frames = 5;
    int cross_exit_frames = 3;
    double cross_min_dist_cm = 35.0;
    double cross_min_time_s = 0.80;
    double cross_timeout_s = 1.5;
    double cross_refractory_s = 0.7;
    double cross_speed_scale = 0.70;
    double cross_heading_grid_deg = 90.0;
    double cross_heading_tolerance_deg = 30.0;
    double cross_enter_max_yaw_rate_dps = 45.0;

    int zebra_enter_frames = 3;
    int zebra_exit_frames = 10;

    int round_enter_frames = 3;
    double round_enter_distance_cm = 35.0;
    double round_enter_max_path_error = 0.25;
    int round_exit_frames = 2;
    bool side_require_tof = false;
    double side_start_distance_cm = 100.0;
    double round_speed_scale = 0.75;
    double round_timeout_s = 10.0;
    double round_refractory_s = 1.0;

    double target_enter_confidence = 0.70;
    double target_close_size = 0.04;
    double target_path_trigger_y = 0.50;
    int target_enter_frames = 3;
    double target_cooldown_s = 2.0;
    double bypass_out_heading_deg = 25.0;
    double bypass_out_distance_cm = 15.0;
    double bypass_around_distance_cm = 35.0;
    double bypass_total_distance_cm = 70.0;
    double bypass_out_yaw_rate_dps = 70.0;
    double bypass_return_yaw_rate_dps = 45.0;
    double bypass_speed_scale = 0.65;
    double straight_over_distance_cm = 40.0;
    double action_timeout_s = 6.0;

    // Optional target-triggered inertial routes. Targets 1-2 use straight,
    // target 3 uses cross_1, target 4 uses cross_2, and target 5 uses
    // chicane. Weapon selects left, supply selects right; vehicle keeps the
    // existing straight-over action.
    std::string target_path_dir;
    std::string target_straight_left_path;
    std::string target_straight_right_path;
    std::string target_cross_1_left_path;
    std::string target_cross_1_right_path;
    std::string target_cross_2_left_path;
    std::string target_cross_2_right_path;
    std::string target_chicane_left_path;
    std::string target_chicane_right_path;

    int telemetry_hz = 20;

    // On-board TFT18 (ST7735S) status page. Opt-in; disabled by default so the
    // baseline control loop is unchanged. Runs on its own render thread.
    bool enable_display = false;
    std::string display_spi_device = "/dev/spidev1.0";
    int display_spi_speed_hz = 8000000;
    int display_dc_gpio = 48;
    int display_reset_gpio = 49;
    int display_rotation = 0;
    int display_scale = 1;
    int display_refresh_hz = 4;

    int threshold_floor = 70;
    bool vision_color_filter = true;
    int vision_saturation_penalty = 60;  // score -= saturation * value / 100
    bool vision_blue_reject = true;
    int vision_blue_hue_low = 85;        // OpenCV hue range 0..179
    int vision_blue_hue_high = 135;
    int vision_blue_saturation_min = 35;
    int vision_blue_value_min = 55;
    int vision_blue_penalty = 80;
    std::string calibration_path = "rewrite/标定数据.txt";
    int calibration_image_height = 240;
    int horizon_row = -1;             // >=0 overrides calibration horizon.
    double horizon_image_row = -1.0;  // >=0 overrides loaded image-row horizon.
    double control_distance_cm = 60.0;
    double far_distance_cm = 120.0;
    int forward_row = -1;  // >=0 overrides control_distance_cm
    int far_row = -1;      // >=0 overrides far_distance_cm
    double error_step_limit = 2.5;

    bool wheel_mask_enable = true;
    double wheel_box_center_ratio = 0.50;
    double wheel_box_width_ratio = 0.28;
    double wheel_box_top_ratio = 0.72;
    double wheel_box_bottom_ratio = 0.98;
    int wheel_box_margin_cols = 2;

    bool far_search_enable = true;
    int edge_smooth_window = 1;
    int bottom_fit_rows = 12;
    int cross_corner_row_tolerance = 5;
    int stable_two_side_min_rows = 6;
    int branch_window_rows = 5;
    int branch_min_rows = 3;
    int branch_return_rows = 3;
    double branch_slope_delta = 0.55;
    double branch_min_deviation = 5.0;
    double branch_return_tolerance = 3.0;
    double lock_slope_tolerance = 0.35;
    double round_stable_tolerance = 2.0;

    void parse(int argc, char** argv);
    void print_banner() const;
    bool target_inertial_routes_enabled() const;
    std::string target_inertial_path_for(int encounter,
                                         TargetKind kind) const;

private:
    void print_help(const char* exe) const;
};

}  // namespace rewrite_path
