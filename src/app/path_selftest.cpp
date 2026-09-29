#include "inertial_navigation.hpp"
#include "motion_control.hpp"
#include "motor_adapter.hpp"
#include "odometry.hpp"
#include "path_controller.hpp"
#include "tof_slope_sensor.hpp"
#include "vision_pipeline.hpp"

#include "encoder_filter.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

using namespace rewrite_path;

namespace {

void check(bool ok, const char* message) {
    if (!ok) {
        std::fprintf(stderr, "[FAIL] %s\n", message);
        std::exit(1);
    }
    std::printf("[OK] %s\n", message);
}

bool near(double actual, double expected, double tolerance) {
    return std::abs(actual - expected) <= tolerance;
}

std::vector<std::uint8_t> make_track(int width,
                                     int height,
                                     int left_bottom,
                                     int right_bottom,
                                     int left_top,
                                     int right_top) {
    std::vector<std::uint8_t> image(width * height, 20);
    for (int y = 0; y < height; ++y) {
        const double t = static_cast<double>(y) / (height - 1);
        const int left = static_cast<int>(
            left_top * (1.0 - t) + left_bottom * t);
        const int right = static_cast<int>(
            right_top * (1.0 - t) + right_bottom * t);
        for (int x = std::max(0, left);
             x <= std::min(width - 1, right); ++x) {
            image[y * width + x] = 220;
        }
    }
    return image;
}

void add_one_sided_arm_rows(std::vector<std::uint8_t>* image,
                            int width,
                            int height,
                            bool right_side,
                            int y0,
                            int y1) {
    y0 = clamp_value(y0, 0, height - 1);
    y1 = clamp_value(y1, y0, height - 1);
    for (int y = y0; y <= y1; ++y) {
        if (right_side) {
            for (int x = width / 2; x < width; ++x) {
                (*image)[y * width + x] = 220;
            }
        } else {
            for (int x = 0; x <= width / 2; ++x) {
                (*image)[y * width + x] = 220;
            }
        }
    }
}

void add_one_sided_arm(std::vector<std::uint8_t>* image,
                       int width,
                       int height,
                       bool right_side) {
    add_one_sided_arm_rows(
        image, width, height, right_side, height / 3, height / 2);
}

void add_two_sided_cross(std::vector<std::uint8_t>* image,
                         int width,
                         int height) {
    const int y0 = height / 4;
    const int y1 = height * 2 / 3;
    for (int y = y0; y <= y1; ++y) {
        for (int x = 4; x < width - 4; ++x) {
            (*image)[y * width + x] = 220;
        }
    }
}

void add_zebra_bars(std::vector<std::uint8_t>* image,
                    int width,
                    int height) {
    const int y0 = height / 2;
    const int y1 = height - 1;
    const int x0 = 0;
    const int x1 = width - 1;
    const int stripe_width = std::max(1, width / kBinaryWidth);
    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            const bool white = ((x - x0) / stripe_width) % 2 != 0;
            (*image)[y * width + x] = white ? 220 : 20;
        }
    }
}

StepInput good_line() {
    StepInput input;
    input.line_confidence = 0.85;
    input.line_lost = false;
    input.two_side_stable = true;
    input.distance_valid = true;
    input.heading_valid = true;
    input.heading_from_imu = true;
    input.yaw_rate_valid = true;
    input.encoder_heading_valid = true;
    return input;
}

}  // namespace

int main(int argc, char** argv) {
    PathParams p;
    p.cross_enter_frames = 3;
    p.cross_exit_frames = 2;
    p.round_enter_frames = 2;
    p.round_exit_frames = 2;
    p.target_enter_frames = 2;
    p.startup_ramp_s = 1.0;
    LegacyVisionPipeline vision(p);

    {
        PathParams parsed;
        char arg0[] = "rewrite_selftest";
        char arg1[] = "--threshold-floor";
        char arg2[] = "82";
        char arg3[] = "--saturation-penalty";
        char arg4[] = "75";
        char arg5[] = "--blue-hue-low";
        char arg6[] = "88";
        char arg7[] = "--blue-hue-high";
        char arg8[] = "132";
        char arg9[] = "--blue-saturation-min";
        char arg10[] = "42";
        char arg11[] = "--blue-value-min";
        char arg12[] = "65";
        char arg13[] = "--blue-penalty";
        char arg14[] = "96";
        char* args[] = {
            arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7,
            arg8, arg9, arg10, arg11, arg12, arg13, arg14,
        };
        parsed.parse(static_cast<int>(sizeof(args) / sizeof(args[0])), args);
        check(parsed.threshold_floor == 82 &&
                  parsed.vision_saturation_penalty == 75 &&
                  parsed.vision_blue_hue_low == 88 &&
                  parsed.vision_blue_hue_high == 132 &&
                  parsed.vision_blue_saturation_min == 42 &&
                  parsed.vision_blue_value_min == 65 &&
                  parsed.vision_blue_penalty == 96,
              "color road-score CLI parameters are configurable");

        LegacyVisionPipeline live_vision(parsed);
        VisionTuningParams live = live_vision.vision_tuning();
        live.threshold_floor = 81;
        live.saturation_penalty = 74;
        live.blue_hue_low = 140;
        live.blue_hue_high = 80;
        live.blue_penalty = 91;
        live_vision.set_vision_tuning(live);
        live = live_vision.vision_tuning();
        check(live.threshold_floor == 81 &&
                  live.saturation_penalty == 74 &&
                  live.blue_hue_low == 80 &&
                  live.blue_hue_high == 140 &&
                  live.blue_penalty == 91,
              "live vision tuning is atomic and clamps hue ordering");
    }

    {
        PathParams parsed;
        char arg0[] = "rewrite_selftest";
        char arg1[] = "--cross-enter";
        char arg2[] = "7";
        char arg3[] = "--cross-exit";
        char arg4[] = "4";
        char arg5[] = "--cross-min-distance";
        char arg6[] = "42";
        char arg7[] = "--cross-timeout";
        char arg8[] = "2.4";
        char arg9[] = "--cross-speed-scale";
        char arg10[] = "0.62";
        char arg11[] = "--heading-hold-kp";
        char arg12[] = "4.5";
        char arg13[] = "--heading-hold-max-rate";
        char arg14[] = "110";
        char arg15[] = "--cross-corner-tol";
        char arg16[] = "4";
        char arg17[] = "--cross-heading-grid";
        char arg18[] = "90";
        char arg19[] = "--cross-heading-tol";
        char arg20[] = "24";
        char arg21[] = "--cross-enter-max-yaw";
        char arg22[] = "38";
        char arg23[] = "--curve-yaw-boost";
        char arg24[] = "3.0";
        char arg25[] = "--curvature-slowdown";
        char arg26[] = "0.75";
        char arg27[] = "--center-yaw-weight";
        char arg28[] = "1.25";
        char arg29[] = "--curve-yaw-shape";
        char arg30[] = "4.0";
        char arg31[] = "--vision-error-step";
        char arg32[] = "1.8";
        char arg33[] = "--round-enter-distance";
        char arg34[] = "72";
        char arg35[] = "--round-enter-max-error";
        char arg36[] = "0.31";
        char arg37[] = "--cross-min-time";
        char arg38[] = "0.85";
        char arg39[] = "--zebra-enter";
        char arg40[] = "4";
        char arg41[] = "--zebra-exit";
        char arg42[] = "12";
        char* args[] = {
            arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8,
            arg9, arg10, arg11, arg12, arg13, arg14, arg15, arg16,
            arg17, arg18, arg19, arg20, arg21, arg22,
            arg23, arg24, arg25, arg26, arg27, arg28, arg29, arg30,
            arg31, arg32, arg33, arg34, arg35, arg36, arg37, arg38,
            arg39, arg40, arg41, arg42,
        };
        parsed.parse(
            static_cast<int>(sizeof(args) / sizeof(args[0])), args);
        check(parsed.cross_enter_frames == 7 &&
                  parsed.cross_exit_frames == 4 &&
                  near(parsed.cross_min_dist_cm, 42.0, 1e-9) &&
                  near(parsed.cross_min_time_s, 0.85, 1e-9) &&
                  near(parsed.cross_timeout_s, 2.4, 1e-9) &&
                  near(parsed.cross_speed_scale, 0.62, 1e-9) &&
                  near(parsed.heading_hold_kp, 4.5, 1e-9) &&
                  near(parsed.heading_hold_max_rate_dps, 110.0, 1e-9) &&
                  near(parsed.cross_heading_grid_deg, 90.0, 1e-9) &&
                  near(parsed.cross_heading_tolerance_deg, 24.0, 1e-9) &&
                  near(parsed.cross_enter_max_yaw_rate_dps, 38.0, 1e-9) &&
                  near(parsed.curve_yaw_boost, 3.0, 1e-9) &&
                  near(parsed.curve_yaw_shape, 4.0, 1e-9) &&
                  near(parsed.error_step_limit, 1.8, 1e-9) &&
                  near(parsed.round_enter_distance_cm, 72.0, 1e-9) &&
                  near(parsed.round_enter_max_path_error, 0.31, 1e-9) &&
                  near(parsed.curvature_slowdown, 0.75, 1e-9) &&
                  near(parsed.center_yaw_weight, 1.25, 1e-9) &&
                  parsed.zebra_enter_frames == 4 &&
                  parsed.zebra_exit_frames == 12 &&
                  parsed.cross_corner_row_tolerance == 4,
              "cross, zebra, heading, and round-entry parameters are configurable");
    }

    {
        PathParams parsed;
        char arg0[] = "selftest";
        char arg1[] = "--yaw-rate-limit";
        char arg2[] = "30";
        char arg3[] = "--min-follow-speed";
        char arg4[] = "0.65";
        char arg5[] = "--target-yaw-slew";
        char arg6[] = "540";
        char arg7[] = "--min-follow-yaw-scale";
        char arg8[] = "0.72";
        char arg9[] = "--hard-turn-outer-scale";
        char arg10[] = "1.85";
        char arg11[] = "--inner-brake";
        char arg12[] = "10";
        char arg13[] = "--inner-brake-margin";
        char arg14[] = "7";
        char arg15[] = "--pwm-slew";
        char arg16[] = "180";
        char arg17[] = "--encoder-filter-alpha";
        char arg18[] = "0.60";
        char arg19[] = "--control-hz";
        char arg20[] = "65";
        char* args[] = {
            arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8,
            arg9, arg10, arg11, arg12, arg13, arg14, arg15, arg16,
            arg17, arg18, arg19, arg20
        };
        parsed.parse(
            static_cast<int>(sizeof(args) / sizeof(args[0])), args);
        check(near(parsed.yaw_rate_correction_limit_cmps, 30.0, 1e-9) &&
                  near(parsed.minimum_follow_speed_scale, 0.65, 1e-9) &&
                  near(parsed.minimum_follow_yaw_scale, 0.72, 1e-9) &&
                  near(parsed.hard_turn_outer_speed_scale, 1.85, 1e-9) &&
                  near(parsed.inner_wheel_brake_max_percent, 10.0, 1e-9) &&
                  near(parsed.inner_wheel_brake_margin_cmps, 7.0, 1e-9) &&
                  near(parsed.target_yaw_slew_dps2, 540.0, 1e-9) &&
                  near(parsed.max_percent_delta_per_s, 180.0, 1e-9) &&
                  near(parsed.encoder_filter_alpha, 0.60, 1e-9) &&
                  parsed.control_hz == 65,
              "yaw, speed, hard-turn limit, and braking are configurable");
    }

    {
        PathParams parsed;
        char arg0[] = "selftest";
        char arg1[] = "--yaw-disagree-dps";
        char arg2[] = "42";
        char arg3[] = "--yaw-disagree-frames";
        char arg4[] = "4";
        char arg5[] = "--yaw-recover-frames";
        char arg6[] = "9";
        char arg7[] = "--yaw-direction-guard";
        char arg8[] = "24";
        char arg9[] = "--yaw-guard-target";
        char arg10[] = "15";
        char* args[] = {
            arg0, arg1, arg2, arg3, arg4, arg5,
            arg6, arg7, arg8, arg9, arg10
        };
        parsed.parse(
            static_cast<int>(sizeof(args) / sizeof(args[0])), args);
        check(near(parsed.yaw_sensor_disagreement_dps, 42.0, 1e-9) &&
                  parsed.yaw_sensor_disagreement_frames == 4 &&
                  parsed.yaw_sensor_recovery_frames == 9 &&
                  near(parsed.yaw_direction_guard_dps, 24.0, 1e-9) &&
                  near(parsed.yaw_direction_guard_target_dps, 15.0, 1e-9),
              "yaw consistency and zero-cross guards are configurable");
    }

    {
        PathParams parsed;
        char arg0[] = "selftest";
        char arg1[] = "--imu-accel-swap-xy";
        char arg2[] = "--imu-accel-forward-sign";
        char arg3[] = "-1";
        char arg4[] = "--imu-accel-right-sign";
        char arg5[] = "1";
        char arg6[] = "--imu-accel-deadband";
        char arg7[] = "0.004";
        char arg8[] = "--no-imu-stationary-zero";
        char arg9[] = "--imu-stationary-accel";
        char arg10[] = "0.035";
        char arg11[] = "--imu-stationary-yaw";
        char arg12[] = "4.0";
        char arg13[] = "--imu-stationary-hold";
        char arg14[] = "0.4";
        char* args[] = {
            arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7,
            arg8, arg9, arg10, arg11, arg12, arg13, arg14,
        };
        parsed.parse(
            static_cast<int>(sizeof(args) / sizeof(args[0])), args);
        check(parsed.imu_accel_swap_xy &&
                  near(parsed.imu_accel_forward_sign, -1.0, 1e-9) &&
                  near(parsed.imu_accel_right_sign, 1.0, 1e-9) &&
                  near(parsed.imu_accel_deadband_g, 0.004, 1e-9) &&
                  !parsed.imu_stationary_zero &&
                  near(parsed.imu_stationary_accel_g, 0.035, 1e-9) &&
                  near(parsed.imu_stationary_yaw_dps, 4.0, 1e-9) &&
                  near(parsed.imu_stationary_hold_s, 0.4, 1e-9),
              "IMU acceleration and stationary zeroing are configurable");
    }

    {
        PathParams parsed;
        char arg0[] = "selftest";
        char arg1[] = "--inertial-path";
        char arg2[] = "course.csv";
        char arg3[] = "--line-lost-inertial";
        char arg4[] = "--inertial-lookahead";
        char arg5[] = "30";
        char arg6[] = "--inertial-speed";
        char arg7[] = "24";
        char arg8[] = "--inertial-heading-kp";
        char arg9[] = "3.2";
        char arg10[] = "--inertial-max-error";
        char arg11[] = "45";
        char arg12[] = "--inertial-sensor-timeout";
        char arg13[] = "0.4";
        char* args[] = {
            arg0, arg1, arg2, arg3, arg4, arg5, arg6,
            arg7, arg8, arg9, arg10, arg11, arg12, arg13,
        };
        parsed.parse(
            static_cast<int>(sizeof(args) / sizeof(args[0])), args);
        check(parsed.inertial_path == "course.csv" &&
                  parsed.line_lost_inertial &&
                  near(parsed.inertial_lookahead_cm, 30.0, 1e-9) &&
                  near(parsed.inertial_speed_cmps, 24.0, 1e-9) &&
                  near(parsed.inertial_heading_kp, 3.2, 1e-9) &&
                  near(parsed.inertial_max_deviation_cm, 45.0, 1e-9) &&
                  near(parsed.inertial_sensor_timeout_s, 0.4, 1e-9),
              "inertial path CLI parameters are configurable");
    }

    {
        PathParams parsed;
        char arg0[] = "selftest";
        char arg1[] = "--target-input-size";
        char arg2[] = "32";
        char arg3[] = "--target-confidence";
        char arg4[] = "0.72";
        char arg5[] = "--target-close-size";
        char arg6[] = "0.04";
        char* args[] = {arg0, arg1, arg2, arg3, arg4, arg5, arg6};
        parsed.parse(
            static_cast<int>(sizeof(args) / sizeof(args[0])), args);
        check(parsed.target_input_size == 32 &&
                  near(parsed.target_enter_confidence, 0.72, 1e-9) &&
                  near(parsed.target_close_size, 0.04, 1e-9),
              "target model and action thresholds are configurable");
    }

    {
        PathParams parsed;
        char arg0[] = "selftest";
        char arg1[] = "--target-path-dir";
        char arg2[] = "/home/root";
        char arg3[] = "--target-path-trigger-y";
        char arg4[] = "0.55";
        char arg5[] = "--target-cross1-left";
        char arg6[] = "/tmp/third-left.csv";
        char* args[] = {
            arg0, arg1, arg2, arg3, arg4, arg5, arg6,
        };
        parsed.parse(
            static_cast<int>(sizeof(args) / sizeof(args[0])), args);
        check(parsed.enable_target_actions &&
                  parsed.target_inertial_routes_enabled() &&
                  near(parsed.target_path_trigger_y, 0.55, 1e-9) &&
                  parsed.target_straight_left_path ==
                      "/home/root/straight_left.csv" &&
                  parsed.target_cross_1_left_path ==
                      "/tmp/third-left.csv" &&
                  parsed.target_inertial_path_for(
                      1, TargetKind::Weapon) ==
                      "/home/root/straight_left.csv" &&
                  parsed.target_inertial_path_for(
                      2, TargetKind::Supply) ==
                      "/home/root/straight_right.csv" &&
                  parsed.target_inertial_path_for(
                      3, TargetKind::Weapon) ==
                      "/tmp/third-left.csv" &&
                  parsed.target_inertial_path_for(
                      4, TargetKind::Supply) ==
                      "/home/root/cross_2_right.csv" &&
                  parsed.target_inertial_path_for(
                      5, TargetKind::Weapon) ==
                      "/home/root/chicane_left.csv" &&
                  parsed.target_inertial_path_for(
                      6, TargetKind::Weapon).empty() &&
                  parsed.target_inertial_path_for(
                      1, TargetKind::Vehicle).empty(),
              "target route directory maps encounters 1-5 and permits overrides");

        PathParams route_params = p;
        route_params.enable_target_actions = true;
        route_params.target_straight_left_path = "left.csv";
        route_params.target_path_trigger_y = 0.50;
        PathController controller(route_params);
        StepInput input = good_line();
        input.target_valid = true;
        input.target_kind = TargetKind::Weapon;
        input.target_confidence = 0.9;
        input.target_size = 0.6;
        input.target_center_y_ratio = 0.49;
        controller.update(input, 0.02);
        controller.update(input, 0.02);
        check(controller.state() == DriveState::Follow,
              "target inertial action waits until red block reaches middle row");
        input.target_center_y_ratio = 0.51;
        controller.update(input, 0.02);
        controller.update(input, 0.02);
        check(controller.state() == DriveState::BypassLeft,
              "target inertial action triggers after middle-row confirmation");
        controller.finish_external_target_action();
        check(controller.state() == DriveState::Follow,
              "completed target inertial route returns to visual following");
    }

    {
        auto image = make_track(188, 120, 60, 128, 74, 114);
        RoadEstimateLite road =
            vision.process_gray(image.data(), 188, 120, 188);
        check(!road.line_lost, "straight synthetic track is visible");
        check(std::abs(road.line_error) < 0.20,
              "straight synthetic error is centered");
        check(road.bottom_pair_valid,
              "complete lower paired edges enable vehicle centering");
        check(road.info.control_row >= 24 && road.info.control_row <= 26,
              "calibration maps 60cm to binary row 25");
        check(road.info.top >= 8, "calibration horizon limits scan top");
    }

    {
        PathParams parsed;
        char arg0[] = "rewrite_selftest";
        char arg1[] = "--topology";
        char arg2[] = "--topology-side-rows";
        char arg3[] = "12";
        char arg4[] = "--topology-min-area";
        char arg5[] = "17";
        char arg6[] = "--cm-error";
        char arg7[] = "--track-width-cm";
        char arg8[] = "52.5";
        char* args[] = {arg0, arg1, arg2, arg3, arg4,
                        arg5, arg6, arg7, arg8};
        parsed.parse(static_cast<int>(sizeof(args) / sizeof(args[0])), args);
        check(parsed.enable_topology &&
                  parsed.topology_side_rows == 12 &&
                  parsed.topology_min_region_area == 17 &&
                  parsed.cm_error_enable &&
                  near(parsed.track_width_cm, 52.5, 1e-9),
              "topology and cm-error CLI parameters are configurable");
    }

    {
        // 直道：图像左右边界列都是背景黑，不应出现任何侧向开口证据。
        PathParams topo_params;
        topo_params.enable_topology = true;
        topo_params.wheel_mask_enable = false;
        LegacyVisionPipeline topo_vision(topo_params);
        auto image = make_track(188, 120, 60, 128, 74, 114);
        RoadEstimateLite road =
            topo_vision.process_gray(image.data(), 188, 120, 188);
        check(road.topology.valid && road.topology.track_area > 0,
              "topology flood fill finds the track region");
        check(road.topology.track_far_row >= 0 &&
                  road.topology.track_far_row <
                      road.topology.track_near_row,
              "topology reports the track vertical extent");
        check(road.topology.left.border_white_rows == 0 &&
                  road.topology.right.border_white_rows == 0,
              "straight track shows no white at the image borders");
        check(!road.topology.left.border_is_track &&
                  road.topology.left.area == 0,
              "straight track has no left-side region");
    }

    {
        // 左侧一块与主赛道不相连的白色区域（被黑缝隔开的侧路 / 车库）。
        PathParams topo_params;
        topo_params.enable_topology = true;
        topo_params.wheel_mask_enable = false;
        LegacyVisionPipeline topo_vision(topo_params);
        auto image = make_track(188, 120, 60, 128, 74, 114);
        for (int y = 100; y < 120; ++y) {
            for (int x = 0; x <= 40; ++x) {
                image[y * 188 + x] = 220;
            }
        }
        RoadEstimateLite road =
            topo_vision.process_gray(image.data(), 188, 120, 188);
        check(road.topology.left.border_white_rows >= 8 &&
                  !road.topology.left.border_is_track,
              "left border white outside the track reports an opening");
        check(road.topology.left.area >= 100 &&
                  road.topology.left.min_col == 0 &&
                  road.topology.left.seed_count >= 1,
              "left opening region reaches the image border");
        check(road.topology.right.border_white_rows == 0 &&
                  road.topology.right.area == 0,
              "left-side opening does not light up the right side");
    }

    {
        // 白色与主赛道相连并顶到左边界：标记 border_is_track 而非独立区域。
        PathParams topo_params;
        topo_params.enable_topology = true;
        topo_params.wheel_mask_enable = false;
        LegacyVisionPipeline topo_vision(topo_params);
        auto image = make_track(188, 120, 60, 128, 74, 114);
        for (int y = 100; y < 120; ++y) {
            for (int x = 0; x < 70; ++x) {
                image[y * 188 + x] = 220;
            }
        }
        RoadEstimateLite road =
            topo_vision.process_gray(image.data(), 188, 120, 188);
        check(road.topology.left.border_white_rows > 0 &&
                  road.topology.left.border_is_track,
              "white reaching the border through the track is flagged");
    }

    {
        // 边界上的小块反光：边界白度仍可见，但面积被门限滤掉。
        PathParams topo_params;
        topo_params.enable_topology = true;
        topo_params.wheel_mask_enable = false;
        topo_params.topology_min_region_area = 40;
        LegacyVisionPipeline topo_vision(topo_params);
        auto image = make_track(188, 120, 60, 128, 74, 114);
        for (int y = 116; y < 118; ++y) {
            for (int x = 0; x < 4; ++x) {
                image[y * 188 + x] = 220;
            }
        }
        RoadEstimateLite road =
            topo_vision.process_gray(image.data(), 188, 120, 188);
        check(road.topology.left.border_white_rows > 0 &&
                  road.topology.left.area == 0,
              "small border reflection is filtered by the area gate");
    }

    {
        // cm 标定：逐行比例由"实测赛道宽度 / 该行双边列宽"得到，
        // 透视下远处每列代表更多厘米。
        PathParams cm_params;
        cm_params.cm_error_enable = true;
        cm_params.track_width_cm = 45.0;
        cm_params.wheel_mask_enable = false;
        LegacyVisionPipeline cm_vision(cm_params);
        auto image = make_track(188, 120, 60, 128, 92, 96);
        RoadEstimateLite road =
            cm_vision.process_gray(image.data(), 188, 120, 188);
        check(road.cm_scale_valid && road.cm_per_col_control > 0.0 &&
                  road.cm_per_col_far > 0.0,
              "centimetre scale is derived from the measured track width");
        check(road.cm_per_col_far > road.cm_per_col_control,
              "far row has more centimetres per column than near row");
        check(std::abs(road.line_error_cm) < 5.0,
              "centred synthetic track keeps the centimetre error small");
    }

    {
        // 未开启开关时不得产生任何额外输出，保证零行为变更。
        PathParams plain_params;
        plain_params.wheel_mask_enable = false;
        LegacyVisionPipeline plain_vision(plain_params);
        auto image = make_track(188, 120, 60, 128, 74, 114);
        RoadEstimateLite road =
            plain_vision.process_gray(image.data(), 188, 120, 188);
        check(!road.topology.valid && !road.cm_scale_valid &&
                  road.topology.track_area == 0 &&
                  road.line_error_cm == 0.0,
              "topology and centimetre output stay disabled by default");
    }

    {
        auto image = make_track(188, 120, 60, 128, 74, 114);
        for (int y = 88; y < 120; ++y) {
            for (int x = 86; x <= 104; ++x) {
                image[y * 188 + x] = 20;
            }
        }
        LegacyVisionPipeline wheel_vision(p);
        wheel_vision.process_gray(image.data(), 188, 120, 188);
        const WheelMaskBox automatic = wheel_vision.wheel_mask_box();
        const int center = (automatic.left + automatic.right) / 2;
        check(automatic.initialized && automatic.auto_detected &&
                  automatic.left >= 39 && automatic.right <= 56 &&
                  automatic.top >= 40 && automatic.top <= 46 &&
                  automatic.bottom == kBinaryHeight - 1,
              "startup vision finds the central-bottom black wheel once");
        check(wheel_vision.debug_frame().binary[automatic.top][center] == 255 &&
                  wheel_vision.debug_frame().binary[automatic.bottom][center] == 255,
              "wheel box is filled white before path finding");

        auto moved = make_track(188, 120, 60, 128, 74, 114);
        for (int y = 78; y < 120; ++y) {
            for (int x = 106; x <= 126; ++x) {
                moved[y * 188 + x] = 20;
            }
        }
        wheel_vision.process_gray(moved.data(), 188, 120, 188);
        const WheelMaskBox locked = wheel_vision.wheel_mask_box();
        check(locked.left == automatic.left && locked.right == automatic.right &&
                  locked.top == automatic.top && locked.bottom == automatic.bottom,
              "automatic wheel search stays locked after initialization");

        check(wheel_vision.set_wheel_mask_box(0.42, 0.70, 0.62, 1.0),
              "manual wheel box accepts normalized drag coordinates");
        const WheelMaskBox manual = wheel_vision.wheel_mask_box();
        check(manual.initialized && !manual.auto_detected &&
                  manual.left < manual.right && manual.top < manual.bottom,
              "manual correction replaces the initialized wheel box");
    }

    {
        auto image = make_track(188, 120, 60, 128, 74, 114);
        PathParams shifted_vehicle_params = p;
        shifted_vehicle_params.wheel_box_center_ratio = 0.60;
        shifted_vehicle_params.wheel_mask_enable = false;
        LegacyVisionPipeline shifted_vehicle_vision(
            shifted_vehicle_params);
        RoadEstimateLite shifted_vehicle;
        for (int i = 0; i < 4; ++i) {
            shifted_vehicle = shifted_vehicle_vision.process_gray(
                image.data(), 188, 120, 188);
        }
        check(shifted_vehicle.vehicle_center_error < -0.08 &&
                  shifted_vehicle.line_error < -0.02,
              "wheel-to-midline offset contributes a centering correction");

        for (int y = 76; y <= 104; ++y) {
            for (int x = 0; x <= 94; ++x) {
                image[y * 188 + x] = 220;
            }
        }
        LegacyVisionPipeline incomplete_bottom_vision(
            shifted_vehicle_params);
        const RoadEstimateLite incomplete_bottom =
            incomplete_bottom_vision.process_gray(
                image.data(), 188, 120, 188);
        check(!incomplete_bottom.bottom_pair_valid &&
                  near(incomplete_bottom.vehicle_center_error, 0.0, 1e-9),
              "incomplete lower edge disables vehicle centering");

        const RoadEstimateLite fading_bottom =
            shifted_vehicle_vision.process_gray(
                image.data(), 188, 120, 188);
        check(!fading_bottom.bottom_pair_valid &&
                  fading_bottom.vehicle_center_error <
                      shifted_vehicle.vehicle_center_error + 0.06 &&
                  std::abs(fading_bottom.vehicle_center_error) <
                      std::abs(shifted_vehicle.vehicle_center_error),
              "incomplete lower edge fades vehicle centering smoothly");
    }

    {
        auto right_curve = make_track(188, 120, 58, 126, 105, 165);
        LegacyVisionPipeline right_vision(p);
        RoadEstimateLite right = right_vision.process_gray(
            right_curve.data(), 188, 120, 188);
        auto left_curve = make_track(188, 120, 62, 130, 20, 80);
        LegacyVisionPipeline left_vision(p);
        RoadEstimateLite left = left_vision.process_gray(
            left_curve.data(), 188, 120, 188);
        check(right.line_error > 0.05 && left.line_error < -0.05,
              "synthetic left/right curves preserve steering sign");
        auto one_arm = make_track(188, 120, 60, 128, 74, 114);
        add_one_sided_arm(&one_arm, 188, 120, true);
        LegacyVisionPipeline arm_vision(p);
        RoadEstimateLite arm =
            arm_vision.process_gray(one_arm.data(), 188, 120, 188);
        check(right.elements.side_open == FeatureSide::None &&
                  left.elements.side_open == FeatureSide::None,
              "ordinary left/right curves do not look like side openings");
        check(!arm.elements.cross,
              "one-sided visual arm is not classified as a complete cross");
        check(arm.elements.side_open == FeatureSide::Right,
              "one-sided visual arm provides a right opening");
        check(arm.elements.roundabout == FeatureSide::None &&
                  arm.elements.roundabout_stage ==
                      RoundaboutVisionStage::None,
              "disabled side-road leaves new roundabout tracking inactive");

        auto left_arm = make_track(188, 120, 60, 128, 74, 114);
        add_one_sided_arm(&left_arm, 188, 120, false);
        LegacyVisionPipeline left_arm_vision(p);
        RoadEstimateLite left_arm_road = left_arm_vision.process_gray(
            left_arm.data(), 188, 120, 188);
        check(left_arm_road.elements.side_open == FeatureSide::Left,
              "mirrored visual arm provides a left opening");
        check(left_arm_road.elements.roundabout_stage ==
                  RoundaboutVisionStage::None,
              "disabled side-road also ignores mirrored roundabout evidence");
    }

    {
        PathParams round_params = p;
        round_params.enable_side_road = true;
        round_params.round_enter_frames = 2;
        LegacyVisionPipeline baseline_vision(p);
        LegacyVisionPipeline round_vision(round_params);
        auto first = make_track(188, 120, 60, 128, 74, 114);
        add_one_sided_arm_rows(&first, 188, 120, true, 42, 62);
        const RoadEstimateLite baseline =
            baseline_vision.process_gray(first.data(), 188, 120, 188);
        RoadEstimateLite approach = round_vision.process_gray(
            first.data(), 188, 120, 188);
        check(approach.elements.roundabout_stage ==
                  RoundaboutVisionStage::Approach &&
                  approach.elements.side_open == FeatureSide::Right &&
                  approach.elements.roundabout == FeatureSide::Right,
              "new recovery evidence starts passive roundabout approach");
        check(near(approach.line_error, baseline.line_error, 1e-9) &&
                  near(approach.far_error, baseline.far_error, 1e-9),
              "roundabout approach does not replace normal line fitting");
        const int first_track_row = approach.roundabout_track_point.row;

        PathParams strict_bend_params = round_params;
        strict_bend_params.round_enter_max_path_error = 0.25;
        PathParams permissive_bend_params = round_params;
        permissive_bend_params.round_enter_max_path_error = 1.0;
        LegacyVisionPipeline strict_bend_vision(strict_bend_params);
        LegacyVisionPipeline permissive_bend_vision(permissive_bend_params);
        strict_bend_vision.process_gray(first.data(), 188, 120, 188);
        permissive_bend_vision.process_gray(first.data(), 188, 120, 188);
        auto sharp_bend = make_track(188, 120, 105, 173, 105, 165);
        add_one_sided_arm_rows(
            &sharp_bend, 188, 120, true, 42, 62);
        const RoadEstimateLite strict_bend =
            strict_bend_vision.process_gray(
                sharp_bend.data(), 188, 120, 188);
        const RoadEstimateLite permissive_bend =
            permissive_bend_vision.process_gray(
                sharp_bend.data(), 188, 120, 188);
        check(strict_bend.elements.roundabout_stage ==
                  RoundaboutVisionStage::None &&
              permissive_bend.elements.roundabout_stage !=
                  RoundaboutVisionStage::None,
              "live path-error guard rejects a 90-degree bend after "
              "roundabout-like opening evidence");

        LegacyVisionPipeline noisy_round_vision(round_params);
        auto noisy_stable_side =
            make_track(188, 120, 60, 128, 74, 114);
        add_one_sided_arm_rows(
            &noisy_stable_side, 188, 120, true, 24, 40);
        add_one_sided_arm_rows(
            &noisy_stable_side, 188, 120, true, 62, 94);
        add_one_sided_arm_rows(
            &noisy_stable_side, 188, 120, false, 48, 54);
        const RoadEstimateLite noisy_approach =
            noisy_round_vision.process_gray(
                noisy_stable_side.data(), 188, 120, 188);
        check(noisy_approach.elements.left_branch_count > 0 &&
                  noisy_approach.elements.right_branch_count >
                      noisy_approach.elements.left_branch_count &&
                  noisy_approach.elements.roundabout_stage ==
                      RoundaboutVisionStage::Approach &&
                  noisy_approach.elements.roundabout ==
                      FeatureSide::Right,
              "dominant ring opening tolerates one stable-side noise branch");
        RoadEstimateLite bilateral_approach = noisy_approach;
        for (int i = 0; i < 3; ++i) {
            bilateral_approach = noisy_round_vision.process_gray(
                noisy_stable_side.data(), 188, 120, 188);
        }
        check(bilateral_approach.elements.roundabout_stage ==
                  RoundaboutVisionStage::Approach,
              "bilateral cross branches cannot confirm roundabout entry");

        auto moved = make_track(188, 120, 60, 128, 74, 114);
        add_one_sided_arm_rows(&moved, 188, 120, true, 62, 94);
        const RoadEstimateLite baseline_moved =
            baseline_vision.process_gray(moved.data(), 188, 120, 188);
        approach = round_vision.process_gray(
            moved.data(), 188, 120, 188);
        check(approach.roundabout_track_point.row > first_track_row &&
                  approach.stable_track_point.row ==
                      approach.roundabout_track_point.row,
              "paired return points move toward the bottom on locked fits");
        check(near(approach.line_error, baseline_moved.line_error, 1e-9) &&
                  near(approach.far_error, baseline_moved.far_error, 1e-9),
              "moving approach evidence still follows live normal fits");

        auto second = make_track(188, 120, 60, 128, 74, 114);
        add_one_sided_arm_rows(&second, 188, 120, true, 24, 40);
        add_one_sided_arm_rows(&second, 188, 120, true, 62, 94);
        RoadEstimateLite inside = round_vision.process_gray(
            second.data(), 188, 120, 188);
        check(inside.elements.roundabout_stage ==
                  RoundaboutVisionStage::Inside &&
              inside.roundabout_recovery_point.valid,
              "second ring-side recovery reconstructs the entry prediction");

        PathParams moving_recovery_params = round_params;
        moving_recovery_params.round_enter_distance_cm = 65.0;
        LegacyVisionPipeline moving_recovery_vision(
            moving_recovery_params);
        moving_recovery_vision.process_gray(
            first.data(), 188, 120, 188);
        RoadEstimateLite moving_recovery;
        bool moving_recovery_entered = false;
        for (int i = 0; i < 14; ++i) {
            auto advancing = make_track(
                188, 120, 60, 128, 74, 114);
            add_one_sided_arm_rows(
                &advancing, 188, 120, true,
                44 + i * 2, 66 + i * 2);
            moving_recovery = moving_recovery_vision.process_gray(
                advancing.data(), 188, 120, 188);
            if (moving_recovery.elements.roundabout_stage ==
                RoundaboutVisionStage::Inside) {
                moving_recovery_entered = true;
                break;
            }
        }
        check(moving_recovery_entered &&
              moving_recovery.roundabout_recovery_point.valid,
              "persistent moving recovery enters a real ring without a "
              "simultaneous second segment");

        PathParams early_round_params = round_params;
        early_round_params.round_enter_distance_cm = 65.0;
        PathParams late_round_params = round_params;
        late_round_params.round_enter_distance_cm = 25.0;
        LegacyVisionPipeline early_round_vision(early_round_params);
        LegacyVisionPipeline late_round_vision(late_round_params);
        early_round_vision.process_gray(first.data(), 188, 120, 188);
        late_round_vision.process_gray(first.data(), 188, 120, 188);
        RoadEstimateLite early_round;
        RoadEstimateLite late_round;
        for (int i = 0; i < 3; ++i) {
            auto advancing = make_track(
                188, 120, 60, 128, 74, 114);
            add_one_sided_arm_rows(
                &advancing, 188, 120, true,
                44 + i * 4, 66 + i * 4);
            early_round = early_round_vision.process_gray(
                advancing.data(), 188, 120, 188);
            late_round = late_round_vision.process_gray(
                advancing.data(), 188, 120, 188);
        }
        check(early_round.elements.roundabout_stage ==
                  RoundaboutVisionStage::Inside &&
              late_round.elements.roundabout_stage ==
                  RoundaboutVisionStage::Approach,
              "larger round entry distance enters ring control earlier");

        LegacyVisionPipeline fast_round_vision(round_params);
        fast_round_vision.process_gray(
            first.data(), 188, 120, 188);
        RoadEstimateLite fast_round;
        bool fast_round_entered = false;
        for (int i = 0; i < 5; ++i) {
            auto advancing = make_track(
                188, 120, 60, 128, 74, 114);
            add_one_sided_arm_rows(
                &advancing, 188, 120, true,
                44 + i * 8, 66 + i * 8);
            fast_round = fast_round_vision.process_gray(
                advancing.data(), 188, 120, 188);
            if (fast_round.elements.roundabout_stage ==
                RoundaboutVisionStage::Inside) {
                fast_round_entered = true;
                break;
            }
        }
        check(fast_round_entered &&
              fast_round.roundabout_recovery_point.valid,
              "fast ring approach enters before the opening leaves view");

        LegacyVisionPipeline s_curve_vision(round_params);
        s_curve_vision.process_gray(first.data(), 188, 120, 188);
        s_curve_vision.process_gray(moved.data(), 188, 120, 188);
        auto single_return = make_track(188, 120, 60, 128, 74, 114);
        add_one_sided_arm_rows(
            &single_return, 188, 120, true, 24, 40);
        const RoadEstimateLite s_curve_candidate =
            s_curve_vision.process_gray(
                single_return.data(), 188, 120, 188);
        check(s_curve_candidate.elements.roundabout_stage !=
                  RoundaboutVisionStage::Inside,
              "sequential single S-curve returns cannot enter roundabout");

        auto straight = make_track(188, 120, 60, 128, 74, 114);
        for (int i = 0; i < std::max(2, p.round_exit_frames); ++i) {
            inside = round_vision.process_gray(
                straight.data(), 188, 120, 188);
        }
        check(inside.elements.roundabout_stage ==
                  RoundaboutVisionStage::Inside,
              "entry opening clears before exit evidence is accepted");

        auto exit_image = make_track(188, 120, 60, 128, 74, 114);
        add_one_sided_arm_rows(&exit_image, 188, 120, false, 42, 68);
        add_one_sided_arm_rows(&exit_image, 188, 120, true, 42, 68);
        RoadEstimateLite exiting = round_vision.process_gray(
            exit_image.data(), 188, 120, 188);
        exiting = round_vision.process_gray(
            exit_image.data(), 188, 120, 188);
        check(exiting.elements.roundabout_stage ==
                  RoundaboutVisionStage::Exit,
              "paired exit openings reconstruct the opposite prediction");

        RoadEstimateLite recovered;
        for (int i = 0; i < std::max(2, p.round_exit_frames); ++i) {
            recovered = round_vision.process_gray(
                straight.data(), 188, 120, 188);
        }
        check(recovered.elements.roundabout_stage ==
                  RoundaboutVisionStage::Reacquired,
              "stable symmetric predictions finish roundabout recovery");

        LegacyVisionPipeline exit_cross_vision(round_params);
        exit_cross_vision.process_gray(first.data(), 188, 120, 188);
        exit_cross_vision.process_gray(moved.data(), 188, 120, 188);
        exit_cross_vision.process_gray(second.data(), 188, 120, 188);
        for (int i = 0; i < std::max(2, p.round_exit_frames); ++i) {
            exit_cross_vision.process_gray(
                straight.data(), 188, 120, 188);
        }
        exiting = exit_cross_vision.process_gray(
            exit_image.data(), 188, 120, 188);
        if (exiting.elements.roundabout_stage !=
            RoundaboutVisionStage::Exit) {
            exiting = exit_cross_vision.process_gray(
                exit_image.data(), 188, 120, 188);
        }
        check(exiting.elements.roundabout_stage ==
                  RoundaboutVisionStage::Exit,
              "paired exit evidence enters visual exit state");
        exiting = exit_cross_vision.process_gray(
            exit_image.data(), 188, 120, 188);
        check(exiting.elements.roundabout_stage ==
                  RoundaboutVisionStage::None,
              "cross evidence clears stale visual exit lock");

        LegacyVisionPipeline false_round_vision(round_params);
        RoadEstimateLite false_approach = false_round_vision.process_gray(
            first.data(), 188, 120, 188);
        const auto clear_track = make_track(188, 120, 60, 128, 74, 114);
        false_approach = false_round_vision.process_gray(
            clear_track.data(), 188, 120, 188);
        check(false_approach.elements.roundabout_stage ==
                  RoundaboutVisionStage::Approach,
              "brief missing evidence preserves the locked first recovery");
        auto opposite = make_track(188, 120, 60, 128, 74, 114);
        add_one_sided_arm_rows(&opposite, 188, 120, false, 42, 62);
        false_approach = false_round_vision.process_gray(
            opposite.data(), 188, 120, 188);
        check(false_approach.elements.roundabout_stage ==
                  RoundaboutVisionStage::None,
              "opposite recovery evidence cancels passive approach");
    }

    {
        auto two_arm = make_track(188, 120, 60, 128, 74, 114);
        add_two_sided_cross(&two_arm, 188, 120);
        PathParams cross_params = p;
        cross_params.enable_side_road = true;
        LegacyVisionPipeline cross_vision(cross_params);
        RoadEstimateLite cross = cross_vision.process_gray(
            two_arm.data(), 188, 120, 188);
        check(cross.elements.cross &&
                  cross.elements.roundabout_stage ==
                      RoundaboutVisionStage::None,
              "paired symmetric cross has priority over roundabout evidence");
    }

    {
        auto zebra_image = make_track(188, 120, 60, 128, 74, 114);
        add_zebra_bars(&zebra_image, 188, 120);
        RoadEstimateLite zebra = vision.process_gray(
            zebra_image.data(), 188, 120, 188);
        check(zebra.elements.zebra,
              "alternating near-field bars produce zebra evidence");
    }

    {
        const double speed =
            MotorAdapter::rpm_to_cmps(60.0, p.wheel_diameter_cm);
        check(near(speed, 3.14159265358979323846 * 6.5, 1e-6),
              "RPM to linear speed uses configured wheel diameter");
    }

    {
        PathController controller(p);
        StepInput input = good_line();
        input.line_error = 0.25;
        input.far_error = 0.30;
        NavigationCommand navigation = controller.update(input, 0.02);
        check(navigation.target_yaw_rate_dps > 0.0,
              "positive vision error requests a right turn");
        check(navigation.target_speed_cmps < p.base_speed_cmps,
              "curvature continuously reduces target speed");

        PathParams faster_curve_params = p;
        faster_curve_params.minimum_follow_speed_scale = 0.65;
        PathController faster_curve_controller(faster_curve_params);
        input.line_error = 1.0;
        input.far_error = 1.0;
        navigation = faster_curve_controller.update(input, 0.02);
        check(near(
                  navigation.target_speed_cmps,
                  p.base_speed_cmps * 0.65,
                  1e-6),
              "hard curves use the configurable raised minimum speed");
    }

    {
        PathParams curve_params = p;
        curve_params.near_yaw_gain = 125.0;
        curve_params.far_yaw_gain = 145.0;
        curve_params.curve_yaw_boost = 0.65;
        curve_params.curvature_slowdown = 0.75;
        curve_params.max_yaw_rate_dps = 190.0;
        PathController baseline_controller(p);
        PathController curve_controller(curve_params);
        StepInput input = good_line();
        input.line_error = 0.25;
        input.far_error = 0.35;
        const NavigationCommand baseline =
            baseline_controller.update(input, 0.02);
        const NavigationCommand boosted =
            curve_controller.update(input, 0.02);
        check(boosted.target_yaw_rate_dps >
                  baseline.target_yaw_rate_dps + 50.0 &&
                  boosted.target_speed_cmps <
                      baseline.target_speed_cmps,
              "curve boost raises yaw demand and slows before the arc");
    }

    {
        PathController controller(p);
        StepInput input = good_line();
        controller.update(input, 0.02);
        input.cross = true;
        input.heading_deg = -292.0;
        input.yaw_rate_dps = 60.0;
        for (int i = 0; i < 6; ++i) controller.update(input, 0.02);
        check(controller.state() == DriveState::Follow,
              "high IMU turn rate blocks premature cross lock");

        input.yaw_rate_dps = -5.0;
        NavigationCommand navigation;
        for (int i = 0; i < p.cross_enter_frames; ++i) {
            navigation = controller.update(input, 0.02);
        }
        check(controller.state() == DriveState::CrossLock &&
                  near(navigation.target_heading_deg, -270.0, 1e-9) &&
                  navigation.target_yaw_rate_dps > 60.0,
              "cross lock snaps overshot IMU heading to nearest 90 degrees");
    }

    {
        PathController controller(p);
        StepInput input = good_line();
        controller.update(input, 0.02);
        input.cross = true;
        input.heading_deg = -258.0;
        input.yaw_rate_dps = -100.0;
        for (int i = 0; i < p.cross_enter_frames; ++i) {
            controller.update(input, 0.02);
        }
        check(controller.state() == DriveState::CrossLock,
              "converging high-rate cross entry hands control to heading lock");
    }

    {
        PathParams tolerant_cross_params = p;
        tolerant_cross_params.cross_heading_tolerance_deg = 45.0;
        PathController controller(tolerant_cross_params);
        StepInput input = good_line();
        controller.update(input, 0.02);
        input.cross = true;
        input.heading_deg = -42.0;
        input.yaw_rate_dps = 40.0;
        NavigationCommand navigation;
        for (int i = 0;
             i < tolerant_cross_params.cross_enter_frames;
             ++i) {
            navigation = controller.update(input, 0.02);
        }
        check(controller.state() == DriveState::CrossLock &&
                  near(navigation.target_heading_deg, 0.0, 1e-9) &&
                  navigation.target_yaw_rate_dps > 0.0,
              "complete cross evidence locks a 42 degree heading deviation");
    }

    {
        PathParams recorded_cross_params = p;
        recorded_cross_params.vision_yaw_sign = -1.0;
        recorded_cross_params.cross_heading_tolerance_deg = 45.0;
        PathController controller(recorded_cross_params);
        StepInput input = good_line();
        input.heading_deg = 0.0;
        controller.update(input, 0.02);

        input.cross = true;
        input.line_error = 0.126;
        input.far_error = 0.119;
        input.heading_deg = -38.8;
        input.yaw_rate_dps = -37.2;
        NavigationCommand navigation;
        for (int i = 0;
             i < recorded_cross_params.cross_enter_frames + 2;
             ++i) {
            navigation = controller.update(input, 0.02);
        }
        check(controller.state() == DriveState::Follow,
              "cross lock rejects a grid heading opposite the live visual turn");

        input.heading_deg = -50.0;
        input.yaw_rate_dps = -25.0;
        for (int i = 0;
             i < recorded_cross_params.cross_enter_frames;
             ++i) {
            navigation = controller.update(input, 0.02);
        }
        check(controller.state() == DriveState::CrossLock &&
                  near(navigation.target_heading_deg, -90.0, 1e-9) &&
                  navigation.target_yaw_rate_dps < 0.0,
              "persistent cross evidence locks the next grid in the turn direction");
    }

    {
        PathController controller(p);
        StepInput input = good_line();
        input.line_error = 0.10;
        input.far_error = 0.55;
        const NavigationCommand navigation =
            controller.update(input, 0.02);
        check(navigation.target_speed_cmps <
                  p.base_speed_cmps * 0.60,
              "far preview slows before a hard curve");
    }

    {
        PathController controller(p);
        StepInput input = good_line();
        input.line_error = 0.35;
        input.far_error = -0.35;
        input.vehicle_center_error = 0.50;
        const NavigationCommand navigation =
            controller.update(input, 0.02);
        check(navigation.target_speed_cmps <
                  p.base_speed_cmps * 0.80,
              "off-center or disagreeing visual fits reduce speed");
    }

    {
        PathParams mounted_camera_params = p;
        mounted_camera_params.vision_yaw_sign = -1.0;
        PathController controller(mounted_camera_params);
        StepInput input = good_line();
        input.line_error = 0.25;
        input.far_error = 0.30;
        const NavigationCommand navigation =
            controller.update(input, 0.02);
        check(navigation.target_yaw_rate_dps < 0.0,
              "mounted-camera sign converts raw image error once");
    }

    {
        PathController controller(p);
        StepInput input = good_line();
        input.line_error = 0.0;
        input.far_error = 0.0;
        input.vehicle_center_error = 0.40;
        const NavigationCommand navigation =
            controller.update(input, 0.02);
        check(navigation.target_yaw_rate_dps >
                  0.25 * p.near_yaw_gain,
              "vehicle-to-fitted-midline offset directly repairs centering");
    }

    {
        PathParams stronger_center_params = p;
        stronger_center_params.center_yaw_weight = 1.10;
        PathController baseline_controller(p);
        PathController stronger_controller(stronger_center_params);
        StepInput input = good_line();
        input.line_error = 0.0;
        input.far_error = 0.0;
        input.vehicle_center_error = 0.40;
        const NavigationCommand baseline =
            baseline_controller.update(input, 0.02);
        const NavigationCommand stronger =
            stronger_controller.update(input, 0.02);
        check(stronger.target_yaw_rate_dps >
                  baseline.target_yaw_rate_dps + 0.10 * p.near_yaw_gain,
              "center weight directly strengthens fitted-midline repair");
    }

    {
        PathParams hard_curve_params = p;
        hard_curve_params.center_yaw_weight = 1.10;
        PathController controller(hard_curve_params);
        StepInput input = good_line();
        input.line_error = 0.35;
        input.far_error = 0.45;
        input.vehicle_center_error = -1.00;
        const NavigationCommand navigation =
            controller.update(input, 0.02);
        check(navigation.target_yaw_rate_dps >
                  0.40 * hard_curve_params.max_yaw_rate_dps,
              "hard-curve center repair cannot cancel paired turn evidence");
    }

    {
        PathParams gradient_params = p;
        gradient_params.curve_yaw_boost = 1.0;
        gradient_params.max_yaw_rate_dps = 1000.0;
        PathController moderate_controller(gradient_params);
        StepInput moderate_input = good_line();
        moderate_input.line_error = 0.20;
        moderate_input.far_error = 0.20;
        const NavigationCommand moderate =
            moderate_controller.update(moderate_input, 0.02);

        PathController edge_controller(gradient_params);
        StepInput edge_input = good_line();
        edge_input.line_error = 0.35;
        edge_input.far_error = 0.35;
        const NavigationCommand edge =
            edge_controller.update(edge_input, 0.02);
        check(edge.target_yaw_rate_dps >
                  4.5 * moderate.target_yaw_rate_dps,
              "visual yaw gain rises steeply near the track edge");
        const double component_sum =
            edge.vision_near_yaw_rate_dps +
            edge.vision_far_yaw_rate_dps +
            edge.vision_center_yaw_rate_dps +
            edge.vision_integral_yaw_rate_dps +
            edge.vision_derivative_yaw_rate_dps;
        check(std::abs(component_sum -
                       edge.vision_unclamped_yaw_rate_dps) < 1e-9,
              "steering telemetry components reproduce the control formula");

        PathController preview_controller(gradient_params);
        StepInput preview_input = good_line();
        preview_input.line_error = -0.02;
        preview_input.far_error = -0.35;
        const NavigationCommand preview =
            preview_controller.update(preview_input, 0.02);
        check(std::abs(preview.vision_near_yaw_rate_dps) <
                  0.05 * gradient_params.near_yaw_gain &&
                  preview.target_yaw_rate_dps < 0.0,
              "far curve cannot amplify near-center sign noise");
    }

    {
        PathParams visual_pi_params = p;
        visual_pi_params.vision_i_gain = 20.0;
        visual_pi_params.vision_i_limit = 0.35;
        visual_pi_params.vision_i_max_error = 0.28;
        visual_pi_params.vision_i_curve_delta = 0.14;
        PathController controller(visual_pi_params);
        StepInput input = good_line();
        input.line_error = 0.08;
        input.far_error = 0.08;
        NavigationCommand first = controller.update(input, 0.02);
        NavigationCommand settled = first;
        for (int i = 0; i < 50; ++i) {
            settled = controller.update(input, 0.02);
        }
        check(settled.vision_integral_error_s > 0.05 &&
                  settled.target_yaw_rate_dps >
                      first.target_yaw_rate_dps + 1.0,
              "stable paired line builds bounded visual integral correction");

        input.side_open = FeatureSide::Right;
        NavigationCommand decayed = settled;
        for (int i = 0; i < 30; ++i) {
            decayed = controller.update(input, 0.02);
        }
        check(std::abs(decayed.vision_integral_error_s) <
                  std::abs(settled.vision_integral_error_s) * 0.25,
              "topology evidence decays visual integral instead of winding up");
    }

    {
        PathParams visual_pid_params = p;
        visual_pid_params.vision_d_gain = 30.0;
        visual_pid_params.vision_d_filter_tau_s = 0.10;
        visual_pid_params.vision_d_max_error_rate = 1.25;
        PathParams visual_p_params = visual_pid_params;
        visual_p_params.vision_d_gain = 0.0;
        PathController pid_controller(visual_pid_params);
        PathController p_controller(visual_p_params);
        StepInput input = good_line();
        input.line_error = 0.0;
        input.far_error = 0.0;
        pid_controller.update(input, 0.02);
        p_controller.update(input, 0.02);

        input.line_error = 0.20;
        input.far_error = 0.20;
        const NavigationCommand pid_growing =
            pid_controller.update(input, 0.02);
        const NavigationCommand p_growing =
            p_controller.update(input, 0.02);
        check(pid_growing.vision_derivative_yaw_rate_dps > 5.0 &&
                  pid_growing.target_yaw_rate_dps >
                      p_growing.target_yaw_rate_dps + 5.0,
              "visual derivative strengthens response while error grows");

        input.line_error = 0.05;
        input.far_error = 0.05;
        const NavigationCommand pid_recentering =
            pid_controller.update(input, 0.02);
        const NavigationCommand p_recentering =
            p_controller.update(input, 0.02);
        check(pid_recentering.vision_derivative_yaw_rate_dps < 0.0 &&
                  pid_recentering.target_yaw_rate_dps <
                      p_recentering.target_yaw_rate_dps,
              "visual derivative damps command while line recenters");

        input.side_open = FeatureSide::Right;
        const NavigationCommand topology =
            pid_controller.update(input, 0.02);
        check(std::abs(topology.vision_derivative_error_per_s) <
                  std::abs(pid_recentering.vision_derivative_error_per_s),
              "topology evidence suppresses visual derivative memory");
    }

    {
        PathController controller(p);
        StepInput input = good_line();
        input.side_open = FeatureSide::Right;
        for (int i = 0; i < 10; ++i) controller.update(input, 0.02);
        check(controller.state() == DriveState::Follow,
              "single-arm hint cannot enter cross lock");

        input.side_open = FeatureSide::None;
        input.cross = true;
        input.heading_deg = 12.0;
        for (int i = 0; i < 3; ++i) controller.update(input, 0.02);
        NavigationCommand navigation = controller.update(input, 0.02);
        check(controller.state() == DriveState::CrossLock,
              "symmetric cross evidence enters cross lock");
        check(navigation.heading_hold &&
                  near(navigation.target_heading_deg, 0.0, 1e-9),
              "cross lock holds the snapped IMU grid heading");

        input.cross = false;
        input.distance_cm = 40.0;
        input.line_error = 0.45;
        input.far_error = 0.40;
        for (int i = 0; i < 3; ++i) controller.update(input, 0.02);
        check(controller.state() == DriveState::CrossLock,
              "cross heading lock waits for a recentered main line");

        input.line_error = 0.05;
        input.far_error = 0.04;
        input.side_open = FeatureSide::Left;
        const int minimum_lock_frames = static_cast<int>(
            std::ceil(p.cross_min_time_s / 0.02)) + 2;
        for (int i = 0; i < minimum_lock_frames; ++i) {
            controller.update(input, 0.02);
        }
        check(controller.state() == DriveState::CrossLock,
              "cross lock cannot exit while a side opening remains visible");

        input.side_open = FeatureSide::None;
        for (int i = 0; i < p.cross_exit_frames; ++i) {
            controller.update(input, 0.02);
        }
        check(controller.state() == DriveState::Follow,
              "cross exits after minimum time, distance, and centered main line");
    }

    {
        PathParams zebra_params = p;
        zebra_params.zebra_enter_frames = 3;
        zebra_params.zebra_exit_frames = 4;
        zebra_params.cross_enter_frames = 1;
        PathController controller(zebra_params);
        StepInput input = good_line();
        input.zebra = true;
        input.cross = true;
        NavigationCommand navigation;
        for (int i = 0; i < zebra_params.zebra_enter_frames; ++i) {
            navigation = controller.update(input, 0.02);
        }
        check(controller.state() == DriveState::Follow &&
                  navigation.zebra_active &&
                  navigation.zebra_encounter_count == 1 &&
                  navigation.target_speed_cmps > 0.0,
              "first zebra encounter remains normal FOLLOW");

        input.zebra = false;
        input.cross = false;
        for (int i = 0; i < zebra_params.zebra_exit_frames; ++i) {
            navigation = controller.update(input, 0.02);
        }
        check(controller.state() == DriveState::Follow &&
                  !navigation.zebra_active &&
                  navigation.zebra_encounter_count == 1,
              "clearing the first zebra rearms encounter detection");

        input.zebra = true;
        for (int i = 0; i < zebra_params.zebra_enter_frames; ++i) {
            navigation = controller.update(input, 0.02);
        }
        check(controller.state() == DriveState::ZebraPass &&
                  navigation.zebra_active &&
                  navigation.zebra_encounter_count == 2 &&
                  navigation.target_speed_cmps > 0.0,
              "second zebra encounter keeps following while passing");

        input.zebra = false;
        for (int i = 0; i < zebra_params.zebra_exit_frames - 1; ++i) {
            navigation = controller.update(input, 0.02);
        }
        check(controller.state() == DriveState::ZebraPass,
              "brief zebra dropout cannot stop before passage confirmation");
        input.zebra = true;
        controller.update(input, 0.02);
        input.zebra = false;
        for (int i = 0; i < zebra_params.zebra_exit_frames; ++i) {
            navigation = controller.update(input, 0.02);
        }
        check(controller.state() == DriveState::Stopped &&
                  navigation.stop_reason == StopReason::ZebraComplete &&
                  navigation.target_speed_cmps == 0.0,
              "second zebra passage latches STOPPED after stable clearance");

        input = good_line();
        for (int i = 0; i < zebra_params.resume_stable_frames * 2; ++i) {
            navigation = controller.update(input, 0.02);
        }
        check(controller.state() == DriveState::Stopped &&
                  navigation.stop_reason == StopReason::ZebraComplete,
              "zebra completion stop cannot auto-resume on a good line");
    }

    {
        PathController controller(p);
        StepInput input = good_line();
        controller.update(input, 0.02);
        input.line_lost = true;
        input.line_confidence = 0.0;
        input.two_side_stable = false;
        for (int i = 0; i < 70; ++i) controller.update(input, 0.02);
        check(controller.state() == DriveState::Stopped,
              "persistent line loss stops the vehicle");
        input = good_line();
        for (int i = 0; i < p.resume_stable_frames; ++i) {
            controller.update(input, 0.02);
        }
        check(controller.state() == DriveState::Follow,
              "stable paired line releases line-loss stop");
    }

    {
        PathController controller(p);
        StepInput input = good_line();
        input.line_error = 0.60;
        input.far_error = 0.60;
        const NavigationCommand paired =
            controller.update(input, 0.02);
        input.bottom_pair_valid = false;
        input.two_side_stable = false;
        const NavigationCommand unpaired =
            controller.update(input, 0.02);
        check(unpaired.line_good &&
                  unpaired.target_yaw_rate_dps > 0.90 *
                      paired.target_yaw_rate_dps,
              "strong consistent curve keeps steering through pair dropout");
        for (int i = 0; i < 95; ++i) {
            controller.update(input, 0.02);
        }
        check(controller.state() == DriveState::Stopped,
              "pairless curve grace remains bounded for off-track safety");

        PathController stable_curve_controller(p);
        input = good_line();
        input.bottom_pair_valid = false;
        input.two_side_stable = true;
        input.line_error = -0.60;
        input.far_error = -0.70;
        const NavigationCommand stable_curve =
            stable_curve_controller.update(input, 0.02);
        check(stable_curve.line_good &&
                  stable_curve.target_yaw_rate_dps < -20.0,
              "stable two-sided curve remains steerable without bottom pair");

        PathParams integrated_params = p;
        integrated_params.base_speed_cmps = 70.0;
        integrated_params.max_speed_cmps = 95.0;
        integrated_params.max_yaw_rate_dps = 300.0;
        integrated_params.minimum_follow_speed_scale = 0.40;
        integrated_params.minimum_follow_yaw_scale = 0.80;
        integrated_params.near_yaw_gain = 80.0;
        integrated_params.far_yaw_gain = 100.0;
        integrated_params.curve_yaw_boost = 0.80;
        integrated_params.curve_yaw_shape = 2.0;
        integrated_params.startup_ramp_s = 0.0;
        integrated_params.target_accel_cmps2 = 5000.0;
        integrated_params.target_yaw_slew_dps2 = 5000.0;
        PathController integrated_path(integrated_params);
        MotionController integrated_motion(integrated_params);
        input = good_line();
        input.bottom_pair_valid = false;
        input.two_side_stable = false;
        input.line_confidence = 0.875;
        input.line_error = 0.725;
        input.far_error = 0.823;
        const NavigationCommand integrated_navigation =
            integrated_path.update(input, 0.02);
        MotorFeedbackLite feedback;
        feedback.left_valid = true;
        feedback.right_valid = true;
        feedback.left_speed_cmps = 70.0;
        feedback.right_speed_cmps = 70.0;
        ImuFeedback imu;
        imu.valid = true;
        const ControlResult integrated_control = integrated_motion.update(
            integrated_navigation, feedback, imu, 0.02);
        check(integrated_navigation.line_good &&
                  std::abs(integrated_navigation.target_yaw_rate_dps) >
                      290.0 &&
                  std::abs(
                      integrated_control.diagnostics.requested_yaw_rate_dps) >=
                      239.0 &&
                  std::abs(
                      integrated_control.diagnostics.left_target_cmps -
                      integrated_control.diagnostics.right_target_cmps) >
                      70.0,
              "recorded pairless hard curve retains target wheel steering");
    }

    {
        PathParams side_params = p;
        side_params.enable_side_road = true;
        PathParams tof_gated_params = side_params;
        tof_gated_params.side_require_tof = true;
        PathController tof_gated_controller(tof_gated_params);
        StepInput tof_input = good_line();
        tof_input.distance_cm =
            tof_gated_params.side_start_distance_cm + 1.0;
        tof_input.roundabout = FeatureSide::Right;
        tof_input.roundabout_stage = RoundaboutVisionStage::Inside;
        tof_gated_controller.update(tof_input, 0.02);
        check(tof_gated_controller.state() == DriveState::Follow,
              "side road requiring TOF rejects unavailable range sensor");
        tof_input.tof_valid = true;
        tof_input.tof_baseline_ready = true;
        tof_input.ramp_detected = true;
        tof_gated_controller.update(tof_input, 0.02);
        check(tof_gated_controller.state() == DriveState::Follow,
              "active ramp suppresses side-road entry");
        tof_input.ramp_detected = false;
        tof_gated_controller.update(tof_input, 0.02);
        check(tof_gated_controller.state() == DriveState::Roundabout,
              "flat valid TOF permits confirmed side-road entry");
        tof_input.ramp_detected = true;
        tof_gated_controller.update(tof_input, 0.02);
        check(tof_gated_controller.state() == DriveState::Follow,
              "late ramp confirmation cancels side-road entry phase");

        PathController controller(side_params);
        StepInput input = good_line();
        input.distance_cm = side_params.side_start_distance_cm + 1.0;
        input.heading_from_imu = false;
        input.roundabout = FeatureSide::Right;
        input.roundabout_stage = RoundaboutVisionStage::Inside;
        controller.update(input, 0.02);
        check(controller.state() == DriveState::Follow,
              "side road cannot start without IMU heading");
        input.heading_from_imu = true;
        input.distance_cm = side_params.side_start_distance_cm - 1.0;
        controller.update(input, 0.02);
        check(controller.state() == DriveState::Follow,
              "startup distance guard rejects early side opening");
        input.distance_cm = side_params.side_start_distance_cm + 1.0;
        input.encoder_heading_valid = false;
        controller.update(input, 0.02);
        check(controller.state() == DriveState::Follow,
              "side road cannot start without paired encoder heading");
        input.encoder_heading_valid = true;
        input.roundabout_stage = RoundaboutVisionStage::Approach;
        controller.update(input, 0.02);
        check(controller.state() == DriveState::Follow,
              "unconfirmed roundabout evidence cannot enter side road");
        input.roundabout_stage = RoundaboutVisionStage::Inside;
        NavigationCommand navigation = controller.update(input, 0.02);
        check(controller.state() == DriveState::Roundabout,
              "rebuilt ring prediction enters visual roundabout state");
        input.line_error = 0.12;
        input.far_error = 0.18;
        navigation = controller.update(input, 0.02);
        check(navigation.target_yaw_rate_dps > 0.0 &&
                  near(navigation.target_speed_cmps,
                       side_params.base_speed_cmps *
                           side_params.round_speed_scale,
                       1e-9),
              "roundabout uses rebuilt centerline at its configured speed");
        input.line_error = -0.80;
        input.far_error = -0.80;
        navigation = controller.update(input, 0.02);
        check(navigation.target_yaw_rate_dps >=
                  side_params.max_yaw_rate_dps * 0.25 - 1e-6,
              "right roundabout rejects an opposite visual yaw command");
        input.roundabout_stage = RoundaboutVisionStage::Exit;
        navigation = controller.update(input, 0.02);
        check(controller.state() == DriveState::Roundabout,
              "early visual exit stays locked before IMU/encoder progress");
        input.heading_deg = 100.0;
        input.encoder_heading_deg = 100.0;
        for (int i = 0; i < 25; ++i) {
            navigation = controller.update(input, 0.02);
        }
        check(controller.state() == DriveState::Roundabout,
              "partial turn remains locked inside the roundabout");
        input.heading_deg = 180.0;
        input.encoder_heading_deg = 180.0;
        navigation = controller.update(input, 0.02);
        check(controller.state() == DriveState::Roundabout,
              "half turn remains locked inside the roundabout");
        input.heading_deg = 360.0;
        input.encoder_heading_deg = 900.0;
        navigation = controller.update(input, 0.02);
        check(controller.state() == DriveState::RoundaboutExit &&
                  navigation.heading_hold &&
                  near(navigation.target_heading_deg, 360.0, 1e-9),
              "IMU completion enters exit despite encoder yaw overshoot");
        for (int i = 0; i < side_params.round_exit_frames; ++i) {
            navigation = controller.update(input, 0.02);
        }
        check(controller.state() == DriveState::Follow,
              "stable paired line releases lock after encoder yaw overshoot");
        input.roundabout_stage = RoundaboutVisionStage::Inside;
        for (int i = 0;
             i < static_cast<int>(
                     side_params.round_refractory_s / 0.02) + 5;
             ++i) {
            navigation = controller.update(input, 0.02);
        }
        check(controller.state() == DriveState::Follow,
              "completed roundabout cannot be counted a second time");

        PathController left_controller(side_params);
        input = good_line();
        input.distance_cm = side_params.side_start_distance_cm + 1.0;
        input.roundabout = FeatureSide::Left;
        input.roundabout_stage = RoundaboutVisionStage::Inside;
        input.line_error = 0.80;
        input.far_error = 0.80;
        input.yaw_rate_dps = 40.0;
        navigation = left_controller.update(input, 0.02);
        check(navigation.state == DriveState::Roundabout &&
                  navigation.target_yaw_rate_dps <=
                      -side_params.max_yaw_rate_dps * 0.40 + 1e-6,
              "left roundabout uses IMU to reject opposite rotation");

        PathParams mounted_params = side_params;
        mounted_params.vision_yaw_sign = -1.0;
        PathController mounted_controller(mounted_params);
        input = good_line();
        input.distance_cm = mounted_params.side_start_distance_cm - 1.0;
        input.roundabout = FeatureSide::Left;
        input.roundabout_stage = RoundaboutVisionStage::Approach;
        input.line_error = 0.0;
        input.far_error = 0.0;
        for (int i = 0;
             i < std::max(8, mounted_params.round_enter_frames * 4);
             ++i) {
            navigation = mounted_controller.update(input, 0.02);
        }
        check(navigation.state == DriveState::Follow &&
                  std::abs(navigation.target_yaw_rate_dps) <= 1e-6,
              "first roundabout opening remains passive until second recovery");
        input.distance_cm = mounted_params.side_start_distance_cm + 1.0;
        input.roundabout_stage = RoundaboutVisionStage::Inside;
        input.line_error = 0.80;
        input.far_error = 0.80;
        navigation = mounted_controller.update(input, 0.02);
        check(navigation.state == DriveState::Roundabout &&
                  navigation.target_speed_cmps > 0.0 &&
                  navigation.target_yaw_rate_dps >=
                      mounted_params.max_yaw_rate_dps * 0.35 - 1e-6,
              "mounted left roundabout keeps the mapped turn direction");

        PathController mounted_right_controller(mounted_params);
        input = good_line();
        input.distance_cm = mounted_params.side_start_distance_cm + 1.0;
        input.roundabout = FeatureSide::Right;
        input.roundabout_stage = RoundaboutVisionStage::Inside;
        navigation = mounted_right_controller.update(input, 0.02);
        check(navigation.state == DriveState::Roundabout &&
                  navigation.target_yaw_rate_dps <=
                      -mounted_params.max_yaw_rate_dps * 0.35 + 1e-6,
              "mounted right roundabout uses the opposite mapped direction");
    }

    {
        PathParams target_params = p;
        target_params.enable_target_actions = true;

        check(target_kind_from_class(0) == TargetKind::Weapon &&
                  target_kind_from_class(1) == TargetKind::Weapon &&
                  target_kind_from_class(2) == TargetKind::Supply &&
                  target_kind_from_class(3) == TargetKind::Supply &&
                  target_kind_from_class(4) == TargetKind::Vehicle &&
                  target_kind_from_class(5) == TargetKind::Vehicle,
              "six classifier classes map to weapon, supply, and vehicle groups");
        float grouped_confidence = 0.0f;
        const std::array<float, 6> grouped_probabilities = {{
            0.04f, 0.03f, 0.41f, 0.37f, 0.08f, 0.07f,
        }};
        check(target_kind_from_probabilities(
                  grouped_probabilities, &grouped_confidence) ==
                  TargetKind::Supply &&
                  near(grouped_confidence, 0.78, 1e-6),
              "paired class probabilities produce supply action confidence");

        StepInput input = good_line();
        input.target_valid = true;
        input.target_confidence = 0.9;
        input.target_size = 0.6;

        PathController weapon_controller(target_params);
        input.target_kind = TargetKind::Weapon;
        weapon_controller.update(input, 0.02);
        weapon_controller.update(input, 0.02);
        check(weapon_controller.state() == DriveState::BypassLeft,
              "confirmed weapon target enters left bypass");

        PathController supply_controller(target_params);
        input.target_kind = TargetKind::Supply;
        supply_controller.update(input, 0.02);
        supply_controller.update(input, 0.02);
        check(supply_controller.state() == DriveState::BypassRight,
              "confirmed supply target enters right bypass");

        PathController controller(target_params);
        input.target_kind = TargetKind::Vehicle;
        controller.update(input, 0.02);
        controller.update(input, 0.02);
        check(controller.state() == DriveState::StraightOver,
              "confirmed vehicle target enters distance-driven straight action");
        input.target_valid = false;
        input.distance_cm = target_params.straight_over_distance_cm + 1.0;
        controller.update(input, 0.02);
        check(controller.state() == DriveState::Follow,
              "straight action exits by encoder distance");
    }

    {
        MotionController motion(p);
        NavigationCommand navigation;
        navigation.state = DriveState::Follow;
        navigation.target_speed_cmps = 35.0;
        navigation.target_yaw_rate_dps = 40.0;
        MotorFeedbackLite feedback;
        feedback.left_valid = true;
        feedback.right_valid = true;
        feedback.left_speed_cmps = 5.0;
        feedback.right_speed_cmps = 5.0;
        ImuFeedback imu;
        imu.valid = true;
        ControlResult control = motion.update(navigation, feedback, imu, 0.02);
        check(control.diagnostics.sensor_mode == SensorMode::Full,
              "IMU plus two encoders selects full closed loop");
        check(control.diagnostics.left_target_cmps >
                  control.diagnostics.right_target_cmps,
              "positive yaw target makes left wheel faster");
        navigation.state = DriveState::Roundabout;
        imu.yaw_rate_dps = 200.0;
        control = motion.update(navigation, feedback, imu, 0.02);
        check(control.diagnostics.left_target_cmps >=
                  control.diagnostics.right_target_cmps,
              "roundabout yaw feedback cannot reverse its turn direction");
        navigation.state = DriveState::Follow;
        imu.yaw_rate_dps = 0.0;
        check(control.command.left_percent <= p.max_percent &&
                  control.command.right_percent <= p.max_percent,
              "wheel PI output respects PWM saturation");
        check(control.diagnostics.limited_speed_cmps < 2.0,
              "startup ramp limits the first speed command");

        imu.valid = false;
        for (int i = 0; i < 100; ++i) {
            control = motion.update(navigation, feedback, imu, 0.02);
        }
        check(control.diagnostics.sensor_mode == SensorMode::EncoderYaw &&
                  control.diagnostics.limited_speed_cmps <=
                      p.imu_fail_speed_cmps + 1e-6,
              "IMU loss uses encoder yaw and its speed limit");

        feedback.right_valid = false;
        control = motion.update(navigation, feedback, imu, 0.02);
        check(control.diagnostics.sensor_mode == SensorMode::SingleEncoder &&
                  control.diagnostics.right_closed_loop == false,
              "single encoder loss keeps the healthy wheel closed-loop");

        feedback.left_valid = false;
        control = motion.update(navigation, feedback, imu, 0.02);
        check(control.diagnostics.sensor_mode == SensorMode::VisionOpenLoop &&
                  control.diagnostics.speed_limit_cmps ==
                      p.all_sensor_fail_speed_cmps,
              "all feedback loss falls back to limited visual open loop");
    }

    {
        PathParams centering_params = p;
        centering_params.startup_ramp_s = 0.0;
        centering_params.base_speed_cmps = 80.0;
        centering_params.max_speed_cmps = 100.0;
        centering_params.target_accel_cmps2 = 5000.0;
        centering_params.target_decel_cmps2 = 5000.0;
        centering_params.max_yaw_rate_dps = 50.0;
        centering_params.minimum_follow_yaw_scale = 0.60;
        centering_params.target_yaw_slew_dps2 = 100.0;
        centering_params.yaw_rate_kp = 1.0;
        centering_params.yaw_rate_correction_limit_cmps = 22.0;
        centering_params.max_percent = 80.0;
        centering_params.max_percent_delta_per_s = 5000.0;
        centering_params.forbid_reverse = true;
        MotionController motion(centering_params);

        NavigationCommand navigation;
        navigation.state = DriveState::Follow;
        navigation.target_speed_cmps = 80.0;
        navigation.target_yaw_rate_dps = 30.0;
        MotorFeedbackLite feedback;
        feedback.left_valid = true;
        feedback.right_valid = true;
        feedback.left_speed_cmps = 45.0;
        feedback.right_speed_cmps = 55.0;
        ImuFeedback imu;
        imu.valid = true;
        imu.yaw_rate_dps = -5.0;

        ControlResult recovering;
        for (int i = 0; i < 20; ++i) {
            recovering = motion.update(navigation, feedback, imu, 0.02);
        }
        check(recovering.diagnostics.left_target_cmps >
                  recovering.diagnostics.right_target_cmps &&
                  recovering.command.left_percent >
                      recovering.command.right_percent &&
                  !recovering.diagnostics.yaw_direction_guard_active,
              "ordinary center recovery preserves differential drive when "
              "encoder yaw still carries the previous turn");

        PathParams hard_turn_params = centering_params;
        hard_turn_params.max_yaw_rate_dps = 190.0;
        hard_turn_params.base_speed_cmps = 35.0;
        hard_turn_params.minimum_follow_yaw_scale = 0.80;
        hard_turn_params.hard_turn_outer_speed_scale = 1.50;
        MotionController hard_turn_motion(hard_turn_params);
        NavigationCommand hard_turn_navigation = navigation;
        hard_turn_navigation.target_yaw_rate_dps = 190.0;
        hard_turn_navigation.target_speed_cmps = 24.0;
        MotorFeedbackLite stationary_feedback;
        stationary_feedback.left_valid = true;
        stationary_feedback.right_valid = true;
        ImuFeedback stationary_imu;
        stationary_imu.valid = true;
        const ControlResult hard_turn = hard_turn_motion.update(
            hard_turn_navigation, stationary_feedback, stationary_imu, 0.02);
        check(hard_turn.diagnostics.right_target_cmps <= 1e-6 &&
                  hard_turn.diagnostics.left_target_cmps > 140.0,
              "true high differential still releases the inner wheel in a "
              "hard turn");
    }

    {
        PathParams drive_params = p;
        drive_params.max_speed_cmps = 95.0;
        drive_params.max_percent = 90.0;
        drive_params.startup_ramp_s = 0.0;
        drive_params.target_accel_cmps2 = 5000.0;
        drive_params.max_percent_delta_per_s = 5000.0;
        drive_params.speed_ff = 0.95;
        drive_params.wheel_speed_kp = 0.68;
        drive_params.wheel_speed_ki = 0.70;
        MotionController motion(drive_params);

        NavigationCommand navigation;
        navigation.state = DriveState::Follow;
        navigation.target_speed_cmps = 60.0;
        MotorFeedbackLite feedback;
        feedback.left_valid = true;
        feedback.right_valid = true;
        feedback.left_speed_cmps = 90.0;
        feedback.right_speed_cmps = 90.0;
        ImuFeedback imu;
        imu.valid = true;
        ControlResult control;
        for (int i = 0; i < 100; ++i) {
            control = motion.update(navigation, feedback, imu, 0.02);
        }
        check(control.command.left_percent >
                  drive_params.min_move_percent &&
              control.command.right_percent >
                  drive_params.min_move_percent &&
              control.command.left_percent < 25.0 &&
              control.command.right_percent < 25.0,
              "persistent overspeed trims feedforward without starving "
              "the motors");
    }

    {
        PathParams free_spin_params = p;
        free_spin_params.base_speed_cmps = 93.0;
        free_spin_params.max_speed_cmps = 95.0;
        free_spin_params.max_percent = 36.0;
        free_spin_params.startup_ramp_s = 0.0;
        free_spin_params.target_accel_cmps2 = 5000.0;
        free_spin_params.target_decel_cmps2 = 5000.0;
        free_spin_params.max_percent_delta_per_s = 90.0;
        free_spin_params.speed_ff = 0.60;
        free_spin_params.wheel_speed_kp = 0.45;
        free_spin_params.wheel_speed_ki = 0.90;
        MotionController free_spin_motion(free_spin_params);

        NavigationCommand navigation;
        navigation.state = DriveState::Follow;
        navigation.target_speed_cmps = 93.0;
        MotorFeedbackLite feedback;
        feedback.left_valid = true;
        feedback.right_valid = true;
        ImuFeedback imu;
        imu.valid = true;

        double simulated_speed = 0.0;
        double settled_pwm_min = 1000.0;
        double settled_pwm_max = -1000.0;
        for (int i = 0; i < 600; ++i) {
            feedback.left_speed_cmps = simulated_speed;
            feedback.right_speed_cmps = simulated_speed;
            const ControlResult control =
                free_spin_motion.update(
                    navigation, feedback, imu, 0.02);
            const double free_running_speed =
                10.5 * control.command.left_percent;
            simulated_speed +=
                (free_running_speed - simulated_speed) *
                (0.02 / 0.12);
            if (i >= 500) {
                settled_pwm_min = std::min(
                    settled_pwm_min, control.command.left_percent);
                settled_pwm_max = std::max(
                    settled_pwm_max, control.command.left_percent);
            }
        }
        check(std::abs(simulated_speed - 93.0) < 8.0 &&
                  settled_pwm_max - settled_pwm_min < 2.0 &&
                  settled_pwm_min > free_spin_params.min_move_percent,
              "unloaded wheel PI settles without pulsed motor effort");
    }

    {
        PathParams low_load_params = p;
        low_load_params.base_speed_cmps = 30.0;
        low_load_params.max_speed_cmps = 95.0;
        low_load_params.max_percent = 36.0;
        low_load_params.startup_ramp_s = 0.0;
        low_load_params.target_accel_cmps2 = 5000.0;
        low_load_params.target_decel_cmps2 = 5000.0;
        low_load_params.max_percent_delta_per_s = 180.0;
        low_load_params.speed_ff = 0.60;
        low_load_params.wheel_speed_kp = 0.45;
        low_load_params.wheel_speed_ki = 0.90;
        MotionController low_load_motion(low_load_params);

        NavigationCommand navigation;
        navigation.state = DriveState::Follow;
        navigation.target_speed_cmps = 30.0;
        MotorFeedbackLite feedback;
        feedback.left_valid = true;
        feedback.right_valid = true;
        ImuFeedback imu;
        imu.valid = true;

        double simulated_speed = 0.0;
        double settled_pwm_min = 1000.0;
        double settled_pwm_max = -1000.0;
        for (int i = 0; i < 800; ++i) {
            feedback.left_speed_cmps = simulated_speed;
            feedback.right_speed_cmps = simulated_speed;
            const ControlResult control =
                low_load_motion.update(
                    navigation, feedback, imu, 0.02);
            const double free_running_speed =
                12.0 * control.command.left_percent;
            simulated_speed +=
                (free_running_speed - simulated_speed) *
                (0.02 / 0.10);
            if (i >= 700) {
                settled_pwm_min = std::min(
                    settled_pwm_min, control.command.left_percent);
                settled_pwm_max = std::max(
                    settled_pwm_max, control.command.left_percent);
            }
        }
        check(std::abs(simulated_speed - 30.0) < 3.0 &&
                  settled_pwm_max - settled_pwm_min < 1.0 &&
                  settled_pwm_min > 0.5 &&
                  settled_pwm_max < low_load_params.min_move_percent,
              "moving unloaded wheel sustains sub-breakaway PWM without "
              "zero/minimum pulsing");
    }

    {
        PathParams envelope_params = p;
        envelope_params.base_speed_cmps = 70.0;
        envelope_params.max_speed_cmps = 95.0;
        envelope_params.max_yaw_rate_dps = 240.0;
        envelope_params.minimum_follow_speed_scale = 0.38;
        envelope_params.minimum_follow_yaw_scale = 0.60;
        envelope_params.startup_ramp_s = 0.0;
        envelope_params.target_accel_cmps2 = 5000.0;
        envelope_params.target_yaw_slew_dps2 = 5000.0;
        MotionController follow_motion(envelope_params);

        NavigationCommand navigation;
        navigation.state = DriveState::Follow;
        navigation.target_speed_cmps = 25.0;
        navigation.target_yaw_rate_dps = 240.0;
        MotorFeedbackLite feedback;
        feedback.left_valid = true;
        feedback.right_valid = true;
        ImuFeedback imu;
        const ControlResult follow =
            follow_motion.update(navigation, feedback, imu, 0.02);
        check(follow.diagnostics.requested_yaw_rate_dps >= 143.0 &&
                  follow.diagnostics.requested_yaw_rate_dps <= 145.0 &&
                  std::abs(follow.diagnostics.left_target_cmps -
                           follow.diagnostics.right_target_cmps) > 35.0,
              "slow FOLLOW retains yaw floor and target wheel differential");

        MotionController round_motion(envelope_params);
        navigation.state = DriveState::Roundabout;
        const ControlResult round =
            round_motion.update(navigation, feedback, imu, 0.02);
        check(near(round.diagnostics.requested_yaw_rate_dps,
                   240.0, 1e-9),
              "roundabout keeps its dedicated absolute yaw command");
    }

    {
        PathParams fallback_params = p;
        fallback_params.startup_ramp_s = 0.0;
        fallback_params.target_accel_cmps2 = 5000.0;
        fallback_params.max_percent_delta_per_s = 5000.0;
        fallback_params.yaw_rate_kp = 0.30;
        fallback_params.encoder_yaw_filter_tau_s = 0.20;
        fallback_params.encoder_yaw_kp_scale = 0.25;
        fallback_params.encoder_yaw_correction_limit_cmps = 5.0;
        MotionController motion(fallback_params);

        NavigationCommand navigation;
        navigation.state = DriveState::Follow;
        navigation.target_speed_cmps = 25.0;
        navigation.target_yaw_rate_dps = 15.0;
        MotorFeedbackLite feedback;
        feedback.left_valid = true;
        feedback.right_valid = true;
        feedback.left_speed_cmps = 45.0;
        feedback.right_speed_cmps = 15.0;
        ImuFeedback imu;
        imu.valid = false;

        const ControlResult control =
            motion.update(navigation, feedback, imu, 0.065);
        check(
            std::abs(control.diagnostics.filtered_encoder_yaw_rate_dps) <
                std::abs(control.diagnostics.raw_encoder_yaw_rate_dps) &&
                control.diagnostics.left_target_cmps >
                    control.diagnostics.right_target_cmps,
              "encoder-yaw fallback filters a one-frame spike without "
              "reversing visual steering");
    }

    {
        PathParams consistency_params = p;
        consistency_params.startup_ramp_s = 0.0;
        consistency_params.target_accel_cmps2 = 5000.0;
        consistency_params.target_decel_cmps2 = 5000.0;
        consistency_params.target_yaw_slew_dps2 = 5000.0;
        consistency_params.max_percent_delta_per_s = 5000.0;
        consistency_params.yaw_sensor_disagreement_dps = 35.0;
        consistency_params.yaw_sensor_disagreement_frames = 3;
        consistency_params.yaw_sensor_recovery_frames = 3;
        consistency_params.yaw_direction_guard_dps = 20.0;
        consistency_params.yaw_direction_guard_target_dps = 12.0;
        MotionController motion(consistency_params);

        NavigationCommand navigation;
        navigation.state = DriveState::Follow;
        navigation.target_speed_cmps = 28.0;
        navigation.target_yaw_rate_dps = -40.0;
        MotorFeedbackLite feedback;
        feedback.left_valid = true;
        feedback.right_valid = true;
        feedback.left_speed_cmps = 14.6;
        feedback.right_speed_cmps = 109.3;
        ImuFeedback imu;
        imu.valid = true;
        imu.yaw_rate_dps = 70.0;

        ControlResult rejected;
        for (int i = 0; i < 3; ++i) {
            rejected = motion.update(
                navigation, feedback, imu, 0.065);
        }
        check(rejected.diagnostics.imu_yaw_rejected &&
                  rejected.diagnostics.sensor_mode ==
                      SensorMode::EncoderYaw &&
                  rejected.diagnostics.measured_yaw_rate_dps < 0.0,
              "sustained opposite IMU and encoder yaw falls back to encoder");
        check(rejected.diagnostics.yaw_direction_guard_active &&
                  rejected.command.left_percent <=
                      rejected.command.right_percent + 1e-9,
              "same-turn wheel control cannot command a differential "
              "through zero");

        feedback.left_speed_cmps = 70.0;
        feedback.right_speed_cmps = 20.0;
        imu.yaw_rate_dps = 100.0;
        ControlResult recovered;
        for (int i = 0; i < 3; ++i) {
            recovered = motion.update(
                navigation, feedback, imu, 0.065);
        }
        check(!recovered.diagnostics.imu_yaw_rejected &&
                  recovered.diagnostics.sensor_mode == SensorMode::Full &&
                  recovered.diagnostics.measured_yaw_rate_dps > 0.0,
              "IMU yaw returns only after sustained encoder agreement");
    }

    {
        PathParams damping_params = p;
        damping_params.startup_ramp_s = 0.0;
        damping_params.target_accel_cmps2 = 5000.0;
        damping_params.max_percent_delta_per_s = 5000.0;
        damping_params.max_speed_cmps = 95.0;
        damping_params.max_yaw_rate_dps = 190.0;
        damping_params.yaw_rate_kp = 0.60;
        damping_params.yaw_rate_correction_limit_cmps = 30.0;
        MotionController motion(damping_params);

        NavigationCommand navigation;
        navigation.state = DriveState::Follow;
        navigation.target_speed_cmps = 50.0;
        navigation.target_yaw_rate_dps = 5.0;
        MotorFeedbackLite feedback;
        feedback.left_valid = true;
        feedback.right_valid = true;
        ImuFeedback imu;
        imu.valid = true;
        imu.yaw_rate_dps = -60.0;

        const ControlResult control =
            motion.update(navigation, feedback, imu, 0.02);
        check(
            control.diagnostics.left_target_cmps -
                    control.diagnostics.right_target_cmps < 20.0 &&
                control.diagnostics.left_target_cmps >
                    control.diagnostics.right_target_cmps,
            "small yaw demand scales feedback to suppress steering oscillation");
    }

    {
        PathParams safety_params = p;
        safety_params.startup_ramp_s = 0.0;
        safety_params.target_accel_cmps2 = 5000.0;
        safety_params.max_percent_delta_per_s = 5000.0;
        safety_params.speed_ff = 0.70;
        safety_params.wheel_speed_kp = 0.45;
        safety_params.wheel_speed_ki = 1.00;
        safety_params.forbid_reverse = true;
        MotionController motion(safety_params);

        NavigationCommand navigation;
        navigation.state = DriveState::Follow;
        navigation.target_speed_cmps = 25.0;
        MotorFeedbackLite feedback;
        feedback.left_valid = true;
        feedback.right_valid = true;
        feedback.left_speed_cmps = 500.0;
        feedback.right_speed_cmps = 500.0;
        ImuFeedback imu;
        imu.valid = true;

        const ControlResult control =
            motion.update(navigation, feedback, imu, 0.02);
        check(control.command.left_percent < 0.0 &&
                  control.command.right_percent < 0.0 &&
                  control.command.left_percent >= -6.0 &&
                  control.command.right_percent >= -6.0,
              "overspeed uses bounded braking instead of zero PWM");
    }

    {
        PathParams brake_params = p;
        brake_params.startup_ramp_s = 0.0;
        brake_params.max_speed_cmps = 95.0;
        brake_params.max_yaw_rate_dps = 190.0;
        brake_params.target_accel_cmps2 = 5000.0;
        brake_params.target_decel_cmps2 = 5000.0;
        brake_params.max_percent_delta_per_s = 5000.0;
        brake_params.inner_wheel_brake_max_percent = 12.0;
        brake_params.inner_wheel_brake_margin_cmps = 8.0;
        brake_params.forbid_reverse = true;
        MotionController motion(brake_params);

        NavigationCommand navigation;
        navigation.state = DriveState::Follow;
        navigation.target_speed_cmps = 28.0;
        navigation.target_yaw_rate_dps = -190.0;
        MotorFeedbackLite feedback;
        feedback.left_valid = true;
        feedback.right_valid = true;
        feedback.left_speed_cmps = 160.0;
        feedback.right_speed_cmps = 90.0;
        ImuFeedback imu;
        imu.valid = true;
        imu.yaw_rate_dps = 40.0;

        const ControlResult braking =
            motion.update(navigation, feedback, imu, 0.02);
        check(braking.command.left_percent < 0.0 &&
                  braking.command.left_percent >= -12.0 &&
                  braking.command.right_percent >= 0.0,
              "S-curve reversal actively brakes only the overspeed inner wheel");

        feedback.left_speed_cmps =
            braking.diagnostics.left_target_cmps + 2.0;
        const ControlResult released =
            motion.update(navigation, feedback, imu, 0.02);
        check(released.command.left_percent >= 0.0,
              "inner-wheel brake releases before wheel reversal");
    }

    {
        PathParams blend_params = p;
        blend_params.startup_ramp_s = 0.0;
        blend_params.target_accel_cmps2 = 5000.0;
        blend_params.max_percent_delta_per_s = 5000.0;
        blend_params.max_speed_cmps = 95.0;
        blend_params.max_yaw_rate_dps = 190.0;
        blend_params.yaw_rate_kp = 0.60;
        blend_params.yaw_rate_correction_limit_cmps = 30.0;
        MotionController below_motion(blend_params);
        MotionController above_motion(blend_params);

        NavigationCommand below;
        below.state = DriveState::Follow;
        below.target_speed_cmps = 50.0;
        below.target_yaw_rate_dps = 132.0;
        NavigationCommand above = below;
        above.target_yaw_rate_dps = 134.0;
        MotorFeedbackLite feedback;
        feedback.left_valid = true;
        feedback.right_valid = true;
        ImuFeedback imu;
        imu.valid = true;

        const ControlResult below_result =
            below_motion.update(below, feedback, imu, 0.02);
        const ControlResult above_result =
            above_motion.update(above, feedback, imu, 0.02);
        const double below_split =
            below_result.diagnostics.left_target_cmps -
            below_result.diagnostics.right_target_cmps;
        const double above_split =
            above_result.diagnostics.left_target_cmps -
            above_result.diagnostics.right_target_cmps;
        check(std::abs(above_split - below_split) < 3.0,
              "hard-turn wheel differential blends continuously");
    }

    {
        PathParams forward_turn_params = p;
        forward_turn_params.startup_ramp_s = 0.0;
        forward_turn_params.target_accel_cmps2 = 5000.0;
        forward_turn_params.max_percent_delta_per_s = 5000.0;
        forward_turn_params.max_speed_cmps = 95.0;
        forward_turn_params.max_yaw_rate_dps = 190.0;
        forward_turn_params.yaw_rate_kp = 0.60;
        forward_turn_params.yaw_rate_correction_limit_cmps = 30.0;
        forward_turn_params.wheel_speed_kp = 0.90;
        forward_turn_params.forbid_reverse = true;
        MotionController motion(forward_turn_params);

        NavigationCommand navigation;
        navigation.state = DriveState::Follow;
        navigation.target_speed_cmps = 24.0;
        navigation.target_yaw_rate_dps = 190.0;
        MotorFeedbackLite feedback;
        feedback.left_valid = true;
        feedback.right_valid = true;
        feedback.left_speed_cmps = 20.0;
        feedback.right_speed_cmps = 20.0;
        ImuFeedback imu;
        imu.valid = true;

        const ControlResult control =
            motion.update(navigation, feedback, imu, 0.02);
        check(control.diagnostics.left_target_cmps > 140.0 &&
                  control.diagnostics.right_target_cmps <= 1e-6 &&
                  control.command.left_percent >
                      control.command.right_percent &&
                  control.command.left_percent >= 0.0 &&
                  control.command.right_percent <= 0.0 &&
                  control.command.right_percent >=
                      -forward_turn_params
                           .inner_wheel_brake_max_percent &&
                  control.diagnostics.target_steering_utilization >
                      0.99,
              "full-scale hard curve raises only the outer-wheel target "
              "and brakes an overspeed inner wheel");
    }

    {
        PathParams forward_turn_params = p;
        forward_turn_params.startup_ramp_s = 0.0;
        forward_turn_params.target_accel_cmps2 = 5000.0;
        forward_turn_params.max_percent_delta_per_s = 5000.0;
        forward_turn_params.max_speed_cmps = 95.0;
        forward_turn_params.max_yaw_rate_dps = 190.0;
        forward_turn_params.yaw_rate_kp = 0.60;
        forward_turn_params.yaw_rate_correction_limit_cmps = 30.0;
        forward_turn_params.forbid_reverse = true;
        MotionController motion(forward_turn_params);

        NavigationCommand navigation;
        navigation.state = DriveState::Follow;
        navigation.target_speed_cmps = 24.0;
        navigation.target_yaw_rate_dps = 110.0;
        MotorFeedbackLite feedback;
        feedback.left_valid = true;
        feedback.right_valid = true;
        ImuFeedback imu;
        imu.valid = true;

        const ControlResult control =
            motion.update(navigation, feedback, imu, 0.02);
        check(control.diagnostics.right_target_cmps <= 1e-6 &&
                  control.diagnostics.left_target_cmps > 0.0 &&
                  control.command.left_percent >= 0.0 &&
                  control.command.right_percent >= 0.0,
              "confirmed turn may stop its inner wheel without reversing");
    }

    {
        PathParams aggressive_params = p;
        aggressive_params.startup_ramp_s = 0.0;
        aggressive_params.target_accel_cmps2 = 5000.0;
        aggressive_params.max_percent_delta_per_s = 5000.0;
        aggressive_params.base_speed_cmps = 70.0;
        aggressive_params.max_percent = 36.0;
        aggressive_params.max_speed_cmps = 95.0;
        aggressive_params.max_yaw_rate_dps = 300.0;
        aggressive_params.minimum_follow_yaw_scale = 0.80;
        aggressive_params.speed_ff = 0.60;
        aggressive_params.wheel_speed_kp = 0.60;
        aggressive_params.forbid_reverse = true;
        MotionController motion(aggressive_params);

        NavigationCommand navigation;
        navigation.state = DriveState::Follow;
        navigation.target_speed_cmps = 28.0;
        navigation.target_yaw_rate_dps = 240.0;
        MotorFeedbackLite feedback;
        feedback.left_valid = true;
        feedback.right_valid = true;
        feedback.left_speed_cmps = 129.0;
        feedback.right_speed_cmps = 8.7;
        ImuFeedback imu;
        imu.valid = true;
        imu.yaw_rate_dps = 90.0;

        const ControlResult aggressive =
            motion.update(navigation, feedback, imu, 0.02);
        check(aggressive.diagnostics.left_target_cmps >= 142.0 &&
                  aggressive.diagnostics.left_target_cmps <= 143.0 &&
                  aggressive.diagnostics.right_target_cmps <= 1e-6 &&
                  aggressive.command.left_percent >= 20.0 &&
                  aggressive.command.left_percent <
                      aggressive_params.max_percent &&
                  aggressive.command.right_percent >= 0.0,
              "recorded full-steer case drives only the outer wheel "
              "with feedback headroom");

        MotionController caught_up_motion(aggressive_params);
        imu.yaw_rate_dps = 268.0;
        const ControlResult caught_up =
            caught_up_motion.update(navigation, feedback, imu, 0.02);
        check(caught_up.diagnostics.left_target_cmps <= 95.0 &&
                  caught_up.diagnostics.right_target_cmps >= 0.0 &&
                  caught_up.command.right_percent >= 0.0,
              "outer-wheel boost releases after measured yaw catches target");
    }

    {
        PathParams rise_params = p;
        rise_params.startup_ramp_s = 0.0;
        rise_params.target_accel_cmps2 = 5000.0;
        rise_params.max_percent_delta_per_s = 60.0;
        rise_params.max_yaw_rate_dps = 190.0;
        rise_params.forbid_reverse = true;
        MotionController straight_motion(rise_params);
        MotionController turn_motion(rise_params);

        NavigationCommand navigation;
        navigation.state = DriveState::Follow;
        navigation.target_speed_cmps = 50.0;
        MotorFeedbackLite feedback;
        feedback.left_valid = true;
        feedback.right_valid = true;
        ImuFeedback imu;
        imu.valid = true;

        const ControlResult straight =
            straight_motion.update(navigation, feedback, imu, 0.02);
        navigation.target_yaw_rate_dps = 190.0;
        const ControlResult hard_turn =
            turn_motion.update(navigation, feedback, imu, 0.02);
        check(near(hard_turn.command.left_percent,
                   straight.command.left_percent, 1e-6) &&
                  hard_turn.command.left_percent > 0.0,
              "hard-turn outer wheel keeps the full acceleration gradient");
    }

    {
        PathParams response_params = p;
        response_params.startup_ramp_s = 0.0;
        response_params.target_accel_cmps2 = 5000.0;
        response_params.target_decel_cmps2 = 5000.0;
        response_params.max_percent_delta_per_s = 60.0;
        response_params.max_speed_cmps = 95.0;
        response_params.max_percent = 90.0;
        response_params.max_yaw_rate_dps = 190.0;
        response_params.target_yaw_slew_dps2 = 5000.0;
        response_params.yaw_rate_kp = 0.60;
        response_params.yaw_rate_correction_limit_cmps = 30.0;
        response_params.speed_ff = 0.95;
        response_params.wheel_speed_kp = 0.90;
        response_params.forbid_reverse = true;
        MotionController motion(response_params);

        NavigationCommand navigation;
        navigation.state = DriveState::Follow;
        navigation.target_speed_cmps = 50.0;
        MotorFeedbackLite feedback;
        feedback.left_valid = true;
        feedback.right_valid = true;
        ImuFeedback imu;
        imu.valid = true;

        ControlResult straight;
        for (int i = 0; i < 20; ++i) {
            straight = motion.update(navigation, feedback, imu, 0.02);
        }
        navigation.target_yaw_rate_dps = 190.0;
        feedback.left_speed_cmps = 80.0;
        feedback.right_speed_cmps = 80.0;
        const ControlResult hard_turn =
            motion.update(navigation, feedback, imu, 0.02);
        const double steering_step =
            (hard_turn.command.left_percent -
             hard_turn.command.right_percent) -
            (straight.command.left_percent -
             straight.command.right_percent);
        check(
            hard_turn.command.right_percent >= 0.0 &&
                steering_step >= 3.5 &&
                steering_step <= 3.7 &&
                hard_turn.command.left_percent >
                    straight.command.left_percent &&
                hard_turn.diagnostics.pwm_steering_utilization >= 0.039,
              "hard turn changes wheel split within the steering slew limit");
    }

    {
        PathParams outer_target_params = p;
        outer_target_params.startup_ramp_s = 0.0;
        outer_target_params.target_accel_cmps2 = 5000.0;
        outer_target_params.max_percent_delta_per_s = 5000.0;
        outer_target_params.base_speed_cmps = 90.0;
        outer_target_params.max_speed_cmps = 150.0;
        outer_target_params.max_yaw_rate_dps = 220.0;
        outer_target_params.minimum_follow_speed_scale = 1.0;
        outer_target_params.yaw_rate_kp = 3.0;
        outer_target_params.yaw_rate_correction_limit_cmps = 60.0;
        outer_target_params.speed_ff = 0.70;
        outer_target_params.wheel_speed_kp = 1.80;
        outer_target_params.forbid_reverse = true;
        MotionController motion(outer_target_params);

        NavigationCommand navigation;
        navigation.state = DriveState::Roundabout;
        navigation.target_speed_cmps = 38.5;
        navigation.target_yaw_rate_dps = 220.0;
        MotorFeedbackLite feedback;
        feedback.left_valid = true;
        feedback.right_valid = true;
        feedback.left_speed_cmps = 80.0;
        feedback.right_speed_cmps = 80.0;
        ImuFeedback imu;
        imu.valid = true;
        imu.yaw_rate_dps = 60.0;

        const ControlResult control =
            motion.update(navigation, feedback, imu, 0.02);
        check(control.diagnostics.left_target_cmps > 224.0 &&
                  control.diagnostics.left_target_cmps <= 225.0 &&
                  control.diagnostics.right_target_cmps == 0.0 &&
                  control.command.left_percent > 0.0 &&
                  control.command.right_percent < 0.0 &&
                  control.command.right_percent >=
                      -outer_target_params
                           .inner_wheel_brake_max_percent,
              "hard turn keeps the raised outer target while braking the "
              "overspeed inner wheel");
    }

    {
        PathParams reversal_params = p;
        reversal_params.startup_ramp_s = 0.0;
        reversal_params.max_speed_cmps = 90.0;
        reversal_params.max_yaw_rate_dps = 190.0;
        reversal_params.target_accel_cmps2 = 5000.0;
        reversal_params.target_decel_cmps2 = 5000.0;
        reversal_params.max_percent_delta_per_s = 60.0;
        reversal_params.speed_ff = 0.9;
        reversal_params.wheel_speed_kp = 0.0;
        reversal_params.wheel_speed_ki = 0.0;
        reversal_params.yaw_rate_kp = 0.60;
        reversal_params.yaw_rate_correction_limit_cmps = 30.0;

        NavigationCommand navigation;
        navigation.state = DriveState::Follow;
        navigation.target_speed_cmps = 45.0;
        navigation.target_yaw_rate_dps = -40.0;
        MotorFeedbackLite feedback;
        feedback.left_valid = true;
        feedback.right_valid = true;
        ImuFeedback imu;
        imu.valid = true;
        imu.yaw_rate_dps = 140.0;

        MotionController tracking_motion(reversal_params);
        const ControlResult tracking =
            tracking_motion.update(navigation, feedback, imu, 0.02);
        check(std::abs(tracking.diagnostics.left_target_cmps -
                       tracking.diagnostics.right_target_cmps) > 50.0,
              "opposite IMU rotation gives S-curve reversal full feedback");

        MotionController slew_motion(reversal_params);
        navigation.target_yaw_rate_dps = 190.0;
        slew_motion.update(navigation, feedback, imu, 0.02);
        navigation.target_yaw_rate_dps = -190.0;
        const ControlResult reversed =
            slew_motion.update(navigation, feedback, imu, 0.02);
        check(reversed.diagnostics.requested_yaw_rate_dps > 0.0,
              "opposite yaw command cannot reverse direction in one frame");
        ControlResult crossed = reversed;
        for (int i = 0; i < 40; ++i) {
            crossed = slew_motion.update(
                navigation, feedback, imu, 0.02);
            if (crossed.diagnostics.requested_yaw_rate_dps < 0.0) break;
        }
        check(crossed.diagnostics.requested_yaw_rate_dps < 0.0,
              "sustained opposite yaw command crosses zero after slew");
    }

    {
        PathParams effort_params = p;
        effort_params.startup_ramp_s = 0.0;
        effort_params.target_accel_cmps2 = 5000.0;
        effort_params.max_percent_delta_per_s = 5000.0;
        effort_params.max_speed_cmps = 95.0;
        effort_params.max_percent = 90.0;
        effort_params.max_yaw_rate_dps = 190.0;
        effort_params.yaw_rate_kp = 0.60;
        effort_params.yaw_rate_correction_limit_cmps = 30.0;
        effort_params.speed_ff = 0.95;
        effort_params.wheel_speed_kp = 0.90;
        effort_params.forbid_reverse = true;
        MotionController motion(effort_params);

        NavigationCommand navigation;
        navigation.state = DriveState::Follow;
        navigation.target_speed_cmps = 50.0;
        navigation.target_yaw_rate_dps = 190.0;
        MotorFeedbackLite feedback;
        feedback.left_valid = true;
        feedback.right_valid = true;
        ImuFeedback imu;
        imu.valid = true;
        imu.yaw_rate_dps = 60.0;

        const ControlResult hard_turn =
            motion.update(navigation, feedback, imu, 0.02);
        check(
            hard_turn.command.left_percent >= 28.0 &&
                hard_turn.command.right_percent == 0.0 &&
                hard_turn.command.left_percent <=
                    effort_params.max_percent &&
                hard_turn.diagnostics.pwm_steering_utilization >= 0.30,
            "large yaw deficit applies full bounded outer-wheel effort");

        MotionController overspeed_motion(effort_params);
        feedback.left_speed_cmps = 300.0;
        feedback.right_speed_cmps = 300.0;
        const ControlResult overspeed_turn =
            overspeed_motion.update(navigation, feedback, imu, 0.02);
        check(
            overspeed_turn.command.left_percent <
                hard_turn.command.left_percent - 20.0 &&
                overspeed_turn.command.left_percent < 5.0 &&
                overspeed_turn.command.right_percent < 0.0 &&
                overspeed_turn.command.right_percent >=
                    -effort_params.inner_wheel_brake_max_percent,
              "outer effort fades above target while the overspeed inner "
              "wheel is actively braked");
    }

    {
        PathParams sustain_params = p;
        sustain_params.startup_ramp_s = 0.0;
        sustain_params.target_accel_cmps2 = 5000.0;
        sustain_params.max_percent_delta_per_s = 5000.0;
        sustain_params.max_speed_cmps = 100.0;
        sustain_params.max_percent = 90.0;
        sustain_params.max_yaw_rate_dps = 220.0;
        sustain_params.speed_ff = 0.70;
        sustain_params.wheel_speed_kp = 1.80;
        sustain_params.wheel_speed_ki = 0.30;
        sustain_params.forbid_reverse = true;
        MotionController motion(sustain_params);

        NavigationCommand navigation;
        navigation.state = DriveState::Follow;
        navigation.target_speed_cmps = 40.0;
        navigation.target_yaw_rate_dps = 220.0;
        MotorFeedbackLite feedback;
        feedback.left_valid = true;
        feedback.right_valid = true;
        ImuFeedback imu;
        imu.valid = true;

        const ControlResult accelerating =
            motion.update(navigation, feedback, imu, 0.02);
        feedback.left_speed_cmps =
            1.08 * accelerating.diagnostics.left_target_cmps;
        const ControlResult slight_overspeed =
            motion.update(navigation, feedback, imu, 0.02);
        feedback.left_speed_cmps =
            1.20 * accelerating.diagnostics.left_target_cmps;
        const ControlResult clear_overspeed =
            motion.update(navigation, feedback, imu, 0.02);
        check(
            slight_overspeed.command.left_percent >=
                    0.65 * accelerating.command.left_percent &&
                clear_overspeed.command.left_percent <
                    slight_overspeed.command.left_percent,
            "hard-turn outer effort persists through small speed ripple");
    }

    {
        PathParams turn_params = p;
        turn_params.startup_ramp_s = 0.0;
        turn_params.target_accel_cmps2 = 5000.0;
        turn_params.max_percent_delta_per_s = 5000.0;
        turn_params.minimum_follow_speed_scale = 1.0;
        turn_params.forbid_reverse = false;
        turn_params.yaw_rate_correction_limit_cmps = 22.0;
        MotionController motion(turn_params);

        NavigationCommand navigation;
        navigation.state = DriveState::Roundabout;
        navigation.target_speed_cmps = 20.0;
        navigation.target_yaw_rate_dps = 180.0;
        MotorFeedbackLite feedback;
        feedback.left_valid = true;
        feedback.right_valid = true;
        ImuFeedback imu;
        imu.valid = true;

        const ControlResult control =
            motion.update(navigation, feedback, imu, 0.02);
        check(control.diagnostics.left_target_cmps > 0.0 &&
                  control.diagnostics.right_target_cmps < 0.0 &&
                  control.command.right_percent < 0.0,
              "allow-reverse actively brakes the inner wheel in a hard turn");
    }

    {
        PathParams boost_params = p;
        boost_params.startup_ramp_s = 0.0;
        boost_params.max_speed_cmps = 80.0;
        boost_params.max_percent = 50.0;
        boost_params.ramp_boost_speed_cmps = 68.0;
        boost_params.target_accel_cmps2 = 5000.0;
        boost_params.target_decel_cmps2 = 5000.0;
        boost_params.max_percent_delta_per_s = 5000.0;
        boost_params.speed_ff = 0.3;
        boost_params.wheel_speed_kp = 0.0;
        boost_params.wheel_speed_ki = 0.0;
        MotionController normal_motion(boost_params);
        MotionController boosted_motion(boost_params);

        NavigationCommand navigation;
        navigation.state = DriveState::Follow;
        navigation.target_speed_cmps = 68.0;
        MotorFeedbackLite feedback;
        feedback.left_valid = true;
        feedback.right_valid = true;
        ImuFeedback imu;
        imu.valid = true;

        ControlResult normal =
            normal_motion.update(navigation, feedback, imu, 0.02);
        navigation.power_boost_percent = 8.0;
        ControlResult boosted =
            boosted_motion.update(navigation, feedback, imu, 0.02);
        check(boosted.command.left_percent >
                  normal.command.left_percent + 7.5 &&
                  boosted.command.right_percent >
                  normal.command.right_percent + 7.5,
              "mild ramp boost adds bounded common wheel feed-forward");

        boost_params.max_percent = 80.0;
        boost_params.ramp_boost_speed_cmps = 80.0;
        boost_params.speed_ff = 0.82;
        boost_params.wheel_speed_kp = 0.8;
        boost_params.wheel_speed_ki = 1.55;
        MotionController ramp_motion(boost_params);
        navigation.target_speed_cmps = 80.0;
        navigation.target_yaw_rate_dps = 220.0;
        navigation.power_boost_percent = 30.0;
        feedback.left_speed_cmps = 0.0;
        feedback.right_speed_cmps = 0.0;
        imu.yaw_rate_dps = -220.0;
        const ControlResult ramp =
            ramp_motion.update(navigation, feedback, imu, 0.02);
        check(ramp.diagnostics.left_target_cmps > 0.0 &&
                  ramp.diagnostics.right_target_cmps > 0.0 &&
                  std::abs(ramp.diagnostics.left_target_cmps -
                           ramp.diagnostics.right_target_cmps) <= 10.0 + 1e-6,
              "ramp boost keeps both wheels forward and limits steering split");
        check(ramp.command.left_percent <= 36.0 + 1e-6 &&
                  ramp.command.right_percent <= 36.0 + 1e-6,
              "ramp boost caps both PWM outputs below the runaway level");

        MotionController blended_motion(boost_params);
        navigation.power_boost_percent =
            0.5 * boost_params.ramp_boost_percent;
        const ControlResult blended =
            blended_motion.update(navigation, feedback, imu, 0.02);
        check(std::abs(blended.diagnostics.left_target_cmps -
                       blended.diagnostics.right_target_cmps) > 10.0 &&
                  std::abs(blended.diagnostics.left_target_cmps -
                           blended.diagnostics.right_target_cmps) <
                      boost_params.max_speed_cmps +
                          boost_params.yaw_rate_correction_limit_cmps,
              "partial ramp boost smoothly yields authority to steering");

        MotionController sync_motion(boost_params);
        navigation.target_speed_cmps = 58.0;
        navigation.target_yaw_rate_dps = 0.0;
        navigation.power_boost_percent =
            boost_params.ramp_boost_percent;
        feedback.left_speed_cmps = 25.0;
        feedback.right_speed_cmps = 45.0;
        imu.yaw_rate_dps = 0.0;
        const ControlResult synchronized =
            sync_motion.update(navigation, feedback, imu, 0.02);
        check(synchronized.command.left_percent >
                  synchronized.command.right_percent + 7.5 &&
                  synchronized.command.left_percent <= 36.0 + 1e-6,
              "bounded ramp wheel sync favors the slower wheel");

        MotionController downhill_motion(boost_params);
        navigation.target_speed_cmps = 52.0;
        feedback.left_speed_cmps = 100.0;
        feedback.right_speed_cmps = 105.0;
        const ControlResult downhill =
            downhill_motion.update(navigation, feedback, imu, 0.02);
        check(near(
                  downhill.diagnostics.power_boost_percent,
                  0.0,
                  1e-9),
              "ramp boost fades out when downhill wheel speed exceeds target");
    }

    {
        RunawayStopGuard wheel_guard;
        MotorFeedbackLite feedback;
        feedback.left_valid = true;
        feedback.right_valid = true;
        feedback.left_speed_cmps = 400.0;
        feedback.right_speed_cmps = 100.0;
        check(!wheel_guard.update(feedback, ImuFeedback(),
                                  NavigationCommand()) &&
                  !wheel_guard.update(feedback, ImuFeedback(),
                                      NavigationCommand()),
              "normal high differential turn stays below 600cmps guard");
        feedback.left_speed_cmps = 620.0;
        feedback.right_speed_cmps = 610.0;
        ImuFeedback imu;
        imu.valid = true;
        imu.yaw_rate_dps = 20.0;
        NavigationCommand navigation;
        navigation.state = DriveState::Follow;
        navigation.line_good = true;
        navigation.target_speed_cmps = 52.0;
        navigation.target_yaw_rate_dps = 25.0;
        check(!wheel_guard.update(feedback, imu, navigation) &&
                  wheel_guard.update(feedback, imu, navigation),
              "two ultra-high wheel-speed frames latch runaway stop");

        navigation.power_boost_percent = 5.0;
        check(wheel_guard.apply(&navigation) &&
                  navigation.state == DriveState::Stopped &&
                  navigation.stop_reason == StopReason::RunawayDetected &&
                  navigation.target_speed_cmps == 0.0 &&
                  navigation.target_yaw_rate_dps == 0.0 &&
                  navigation.power_boost_percent == 0.0,
              "runaway latch clears motion and ramp boost commands");

        RunawayStopGuard spin_guard;
        feedback.left_speed_cmps = 90.0;
        feedback.right_speed_cmps = 85.0;
        imu.yaw_rate_dps = 360.0;
        check(!spin_guard.update(feedback, imu, navigation) &&
                  spin_guard.update(feedback, imu, navigation),
              "sustained high-speed body rotation latches runaway stop");

        RunawayStopGuard commanded_turn_guard;
        feedback.left_speed_cmps = 153.4;
        feedback.right_speed_cmps = 78.4;
        navigation.state = DriveState::Follow;
        navigation.target_speed_cmps = 38.5;
        navigation.target_yaw_rate_dps = 220.0;
        navigation.line_good = true;
        imu.yaw_rate_dps = 360.0;
        check(!commanded_turn_guard.update(
                  feedback, imu, navigation) &&
                  !commanded_turn_guard.update(
                      feedback, imu, navigation),
              "matching commanded fast turn does not trip runaway stop");

        RunawayStopGuard inertial_turn_guard;
        navigation.target_yaw_rate_dps = -130.0;
        imu.yaw_rate_dps = -228.0;
        check(!inertial_turn_guard.update(
                  feedback, imu, navigation),
              "commanded turn tolerates bounded inertial yaw overshoot");
        navigation.target_yaw_rate_dps = -20.0;
        imu.yaw_rate_dps = -242.0;
        check(!inertial_turn_guard.update(
                  feedback, imu, navigation),
              "one post-turn inertial frame does not latch runaway stop");
        imu.yaw_rate_dps = -190.0;
        check(!inertial_turn_guard.update(
                  feedback, imu, navigation),
              "normal post-turn yaw clears runaway confirmation");

        RunawayStopGuard opposite_spin_guard;
        navigation.target_yaw_rate_dps = 220.0;
        imu.yaw_rate_dps = -360.0;
        check(!opposite_spin_guard.update(
                  feedback, imu, navigation) &&
                  opposite_spin_guard.update(
                      feedback, imu, navigation),
              "opposite high-speed spin still latches runaway stop");

        RunawayStopGuard transient_guard;
        check(!transient_guard.update(feedback, imu, navigation),
              "one rotation spike does not stop the vehicle");
        imu.yaw_rate_dps = 40.0;
        check(!transient_guard.update(feedback, imu, navigation),
              "normal rotation clears runaway confirmation");
    }

    {
        PathParams tof_params = p;
        tof_params.tof_baseline_frames = 3;
        tof_params.tof_ramp_enter_frames = 2;
        tof_params.tof_ramp_exit_frames = 2;
        tof_params.tof_ramp_delta_mm = 400.0;
        tof_params.tof_ramp_hysteresis_mm = 35.0;
        tof_params.tof_ramp_sign = -1.0;
        tof_params.ramp_boost_max_s = 0.2;
        TofSlopeDetector detector(tof_params);
        TofDistanceSample sample;
        for (std::uint64_t sequence = 1; sequence <= 3; ++sequence) {
            sample.sequence = sequence;
            sample.distance_mm = 700.0;
            sample.valid = true;
            detector.update(sample, 0.05);
        }
        TofSlopeFeedback tof = detector.update(sample, 0.05);
        check(tof.baseline_ready &&
                  near(tof.baseline_distance_mm, 700.0, 0.1),
              "TOF detector learns a stationary flat-road baseline");

        for (std::uint64_t sequence = 4; sequence <= 6; ++sequence) {
            sample.sequence = sequence;
            sample.distance_mm = 350.0;
            tof = detector.update(sample, 0.05);
        }
        check(tof.ramp_detected && tof.distance_mm <= 400.0,
              "sustained VL53L0X distance within 400mm enters ramp boost");

        for (std::uint64_t sequence = 7; sequence <= 14; ++sequence) {
            sample.sequence = sequence;
            sample.distance_mm = 350.0;
            tof = detector.update(sample, 0.10);
        }
        check(!tof.ramp_detected,
              "ramp boost timeout cannot immediately retrigger");

        for (std::uint64_t sequence = 15; sequence <= 22; ++sequence) {
            sample.sequence = sequence;
            sample.distance_mm = 500.0;
            tof = detector.update(sample, 0.05);
        }
        check(!tof.ramp_detected,
              "flat-road range reacquisition releases ramp boost");

        for (std::uint64_t sequence = 23; sequence <= 26; ++sequence) {
            sample.sequence = sequence;
            sample.distance_mm = 350.0;
            tof = detector.update(sample, 0.05);
        }
        check(tof.ramp_detected,
              "flat-road reacquisition rearms ramp detection");

        tof_params.tof_stop_test = true;
        bool stop_latched = false;
        TofSlopeFeedback unavailable_tof;
        NavigationCommand unavailable_navigation;
        unavailable_navigation.target_speed_cmps = 40.0;
        check(apply_tof_stop_latch(
                  tof_params, unavailable_tof, &stop_latched,
                  &unavailable_navigation) &&
                  !stop_latched &&
                  unavailable_navigation.state == DriveState::Stopped &&
                  unavailable_navigation.stop_reason ==
                      StopReason::TofUnavailable,
              "TOF stop test prevents motion before range baseline is ready");

        NavigationCommand navigation;
        navigation.state = DriveState::Follow;
        navigation.target_speed_cmps = 40.0;
        navigation.target_yaw_rate_dps = 30.0;
        navigation.power_boost_percent = 10.0;
        check(apply_tof_stop_latch(
                  tof_params, tof, &stop_latched, &navigation) &&
                  stop_latched &&
                  navigation.state == DriveState::Stopped &&
                  navigation.stop_reason == StopReason::RampDetected &&
                  navigation.target_speed_cmps == 0.0 &&
                  navigation.target_yaw_rate_dps == 0.0 &&
                  navigation.power_boost_percent == 0.0,
              "TOF stop test latches a zero-motion navigation command");

        MotionController stop_motion(tof_params);
        MotorFeedbackLite moving_feedback;
        moving_feedback.left_valid = true;
        moving_feedback.right_valid = true;
        moving_feedback.left_speed_cmps = 30.0;
        moving_feedback.right_speed_cmps = 20.0;
        ImuFeedback turning_imu;
        turning_imu.valid = true;
        turning_imu.yaw_rate_dps = 80.0;
        const ControlResult stopped =
            stop_motion.update(navigation, moving_feedback, turning_imu, 0.02);
        check(stopped.command.left_percent == 0.0 &&
                  stopped.command.right_percent == 0.0,
              "STOPPED bypasses yaw feedback and clears both PWM outputs");
    }

    {
        PathParams inertial_params = p;
        inertial_params.inertial_lookahead_cm = 20.0;
        inertial_params.inertial_finish_tolerance_cm = 8.0;
        inertial_params.inertial_sensor_timeout_s = 0.05;
        inertial_params.inertial_max_deviation_cm = 10.0;
        inertial_params.inertial_deviation_timeout_s = 0.05;
        const std::vector<InertialWaypoint> route = {
            {0.0, 0.0, 0.0, 20.0},
            {50.0, 0.0, 0.0, 20.0},
            {100.0, 0.0, 0.0, 20.0},
        };

        InertialPathNavigator navigator(inertial_params);
        check(navigator.set_path(route),
              "inertial navigator accepts a generated waypoint path");
        OdometrySample pose;
        pose.valid = true;
        ImuFeedback imu;
        imu.valid = true;
        NavigationCommand navigation;
        for (int i = 0; i < 3; ++i) {
            navigation = navigator.update(pose, imu, 0.02);
        }
        check(navigator.status().state == InertialNavState::Tracking &&
                  navigation.state == DriveState::InertialTrack &&
                  navigation.target_speed_cmps > 0.0 &&
                  std::abs(navigation.target_yaw_rate_dps) < 1e-9,
              "inertial state machine aligns then tracks a straight path");

        pose.x_cm = 55.0;
        navigation = navigator.update(pose, imu, 0.02);
        check(navigator.status().progress_cm > 50.0 &&
                  navigator.status().waypoint_index >= 2,
              "inertial navigator advances monotonically to a lookahead point");
        pose.x_cm = 96.0;
        navigation = navigator.update(pose, imu, 0.02);
        check(navigator.status().state == InertialNavState::Complete &&
                  navigation.state == DriveState::Stopped &&
                  navigation.stop_reason == StopReason::PathComplete,
              "inertial navigator stops at the final waypoint");

        InertialPathNavigator sensor_fault(inertial_params);
        check(sensor_fault.set_path(route),
              "sensor-fault navigator path setup succeeds");
        sensor_fault.update(OdometrySample{0.0, 0.0, 0.0, 0.0, false, true},
                            imu, 0.02);
        imu.valid = false;
        for (int i = 0; i < 3; ++i) {
            navigation = sensor_fault.update(pose, imu, 0.02);
        }
        check(sensor_fault.status().state == InertialNavState::Fault &&
                  navigation.stop_reason == StopReason::InertialSensorLost,
              "persistent IMU loss faults and stops inertial navigation");

        InertialPathNavigator deviation_fault(inertial_params);
        check(deviation_fault.set_path(route),
              "deviation-fault navigator path setup succeeds");
        imu.valid = true;
        pose.x_cm = 0.0;
        pose.y_cm = 0.0;
        deviation_fault.update(pose, imu, 0.02);
        pose.y_cm = 20.0;
        for (int i = 0; i < 3; ++i) {
            navigation = deviation_fault.update(pose, imu, 0.02);
        }
        check(deviation_fault.status().state == InertialNavState::Fault &&
                  navigation.stop_reason == StopReason::PathDeviation,
              "sustained cross-track error faults and stops navigation");

        InertialPathNavigator passive_sync(inertial_params);
        check(passive_sync.set_path(route),
              "passive inertial fallback path setup succeeds");
        pose.x_cm = 0.0;
        pose.y_cm = 0.0;
        passive_sync.update(pose, imu, 0.02, false);
        pose.y_cm = 20.0;
        for (int i = 0; i < 4; ++i) {
            navigation = passive_sync.update(pose, imu, 0.02, false);
        }
        check(passive_sync.status().state != InertialNavState::Fault,
              "passive visual phase does not pre-latch inertial deviation");
        for (int i = 0; i < 3; ++i) {
            navigation = passive_sync.update(pose, imu, 0.02, true);
        }
        check(passive_sync.status().state == InertialNavState::Fault &&
                  navigation.stop_reason == StopReason::PathDeviation,
              "inertial takeover enables deviation protection");

        InertialPathNavigator reanchored_sync(inertial_params);
        check(reanchored_sync.set_path(route),
              "reanchored inertial fallback path setup succeeds");
        pose.x_cm = 0.0;
        pose.y_cm = 0.0;
        reanchored_sync.update(pose, imu, 0.02, false);
        pose.y_cm = 20.0;
        for (int i = 0; i < 4; ++i) {
            navigation = reanchored_sync.update(pose, imu, 0.02, false);
        }
        check(reanchored_sync.reanchor_at_current_progress(pose, imu),
              "line-loss takeover reanchors at the current route progress");
        navigation = reanchored_sync.update(pose, imu, 0.02, true);
        check(navigation.state != DriveState::Stopped &&
                  reanchored_sync.status().takeover_reanchored &&
                  reanchored_sync.status().cross_track_error_cm < 1e-6,
              "reanchored takeover avoids an immediate deviation stop");

        InertialTakeoverDirectionGuard direction_guard(inertial_params);
        NavigationCommand lost_visual;
        lost_visual.state = DriveState::Stopped;
        lost_visual.stop_reason = StopReason::LineLost;
        lost_visual.vision_unclamped_yaw_rate_dps = 20.0;
        direction_guard.capture(lost_visual);
        NavigationCommand opposing_inertial;
        opposing_inertial.state = DriveState::InertialTrack;
        opposing_inertial.target_speed_cmps = 35.0;
        opposing_inertial.target_yaw_rate_dps = -300.0;
        check(direction_guard.apply(&opposing_inertial) &&
                  opposing_inertial.inertial_direction_guard_active &&
                  near(opposing_inertial.target_yaw_rate_dps, 20.0, 1e-9) &&
                  opposing_inertial.target_speed_cmps <=
                      inertial_params.inertial_min_speed_cmps,
              "inertial takeover preserves the preceding visual turn");
        NavigationCommand agreeing_inertial;
        agreeing_inertial.state = DriveState::InertialTrack;
        agreeing_inertial.target_speed_cmps = 20.0;
        agreeing_inertial.target_yaw_rate_dps = 30.0;
        check(!direction_guard.apply(&agreeing_inertial) &&
                  !direction_guard.active() &&
                  near(agreeing_inertial.target_yaw_rate_dps, 30.0, 1e-9),
              "inertial direction guard releases after route agreement");
    }

    {
        const double bounded_start =
            smartcar::bound_encoder_rpm_magnitude(1000.0, 0.0);
        const double bounded_running =
            smartcar::bound_encoder_rpm_magnitude(1000.0, 40.0);
        const double stopped =
            smartcar::bound_encoder_rpm_magnitude(0.0, 80.0);
        check(near(bounded_start, 40.0, 1e-9) &&
                  near(bounded_running, 120.0, 1e-9) &&
                  near(stopped, 0.0, 1e-9),
              "encoder RPM spikes are bounded without freezing stale speed");

        double filtered = 40.0;
        for (int i = 0; i < 6; ++i) {
            const double sample =
                smartcar::bound_encoder_rpm_magnitude(1000.0, filtered);
            filtered += (sample - filtered) * 0.35;
        }
        check(filtered > 500.0 && filtered < 1000.0,
              "persistent plausible acceleration converges instead of being held");
    }

    {
        PlanarOdometry odometry(p);
        MotorFeedbackLite motor;
        motor.left_valid = true;
        motor.right_valid = true;
        ImuFeedback imu;
        imu.valid = true;
        odometry.update(motor, imu);
        motor.left_distance_cm = 10.0;
        motor.right_distance_cm = 10.0;
        OdometrySample straight = odometry.update(motor, imu);
        check(near(straight.x_cm, 10.0, 0.01) &&
                  near(straight.y_cm, 0.0, 0.01),
              "odometry integrates a straight segment on +X");

        motor.left_distance_cm = 20.0;
        motor.right_distance_cm = 15.0;
        imu.heading_deg = 30.0;
        OdometrySample turn = odometry.update(motor, imu);
        check(turn.y_cm > 0.0 && turn.heading_deg > 0.0,
              "right-positive heading produces +Y trajectory");

        PlanarOdometry left_odometry(p);
        motor = MotorFeedbackLite{};
        motor.left_valid = true;
        motor.right_valid = true;
        imu.heading_deg = 0.0;
        left_odometry.update(motor, imu);
        motor.left_distance_cm = 5.0;
        motor.right_distance_cm = 10.0;
        imu.heading_deg = -30.0;
        OdometrySample left_turn = left_odometry.update(motor, imu);
        check(left_turn.y_cm < 0.0 && left_turn.heading_deg < 0.0,
              "left turn produces -Y trajectory");
    }

    if (argc > 1) {
        InertialPathNavigator file_navigator(p);
        check(file_navigator.load(argv[1]) &&
                  file_navigator.status().waypoint_count >= 2 &&
                  file_navigator.status().path_length_cm > 0.0,
              "C++ navigator loads a recorder-generated path CSV");
    }

    std::printf("all rewrite selftests passed\n");
    return 0;
}

