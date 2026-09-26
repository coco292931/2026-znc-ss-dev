#include "path_params.hpp"
#include "path_types.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace rewrite_path {

namespace {

bool has_value(int argc, int index) {
    return index + 1 < argc;
}

bool next_is_value(int argc, char** argv, int index) {
    return has_value(argc, index) && argv[index + 1][0] != '-';
}

std::string join_path(const std::string& directory, const char* filename) {
    if (directory.empty()) return std::string();
    const char last = directory.back();
    return directory + (last == '/' || last == '\\' ? "" : "/") + filename;
}

}  // namespace

void PathParams::print_help(const char* exe) const {
    std::printf(
        "Usage: %s [options]\n"
        "  --camera <dev>          Camera device, default /dev/video0\n"
        "  --control-hz <N>        Motor/control loop rate, default 50\n"
        "  --http [port]           Enable async MJPEG stream, default 8080\n"
        "  --http-fps <N>          Stream FPS cap, default 20\n"
        "  --http-quality <N>      JPEG quality 1..95, default 60\n"
        "  --model-dir <dir>       Model/config directory\n"
        "  --enable-motors         Drive motors, default dry-run\n"
        "  --dry-run               Do not drive motors\n"
        "  --no-ncnn               Disable target recognition\n"
        "  --no-stop               Do not latch STOPPED after long line loss\n"
        "  --zebra-enter <N>       Frames confirming each zebra encounter\n"
        "  --zebra-exit <N>        Clear frames confirming zebra passage\n"
        "  --base-speed <cm/s>     Cruise speed, default 35\n"
        "  --max-speed <cm/s>      Speed hard limit, default 40\n"
        "  --hard-turn-outer-scale <N> Full-steer outer speed limit multiplier\n"
        "  --max-percent <N>       Duty percent hard limit\n"
        "  --target-accel <cm/s2>  Target acceleration limit\n"
        "  --target-decel <cm/s2>  Target deceleration limit\n"
        "  --target-yaw-slew <deg/s2> Target yaw-rate slew limit\n"
        "  --pwm-slew <percent/s>  Motor PWM change limit, default 180\n"
        "  --speed-ff <N>          Wheel speed feed-forward %%/(cm/s)\n"
        "  --speed-kp <N>          Per-wheel speed P gain\n"
        "  --speed-ki <N>          Per-wheel speed I gain\n"
        "  --inner-brake <percent> Bounded hard-turn inner-wheel braking\n"
        "  --inner-brake-margin <cm/s> Overspeed before braking starts\n"
        "  --vision-yaw-sign <N>   Raw image error to right-positive yaw sign\n"
        "  --vision-i-gain <N>     Stable-line visual integral yaw gain\n"
        "  --vision-i-limit <N>    Visual integral error-seconds limit\n"
        "  --vision-i-max-error <N> Max near error eligible for integration\n"
        "  --vision-i-curve-delta <N> Max near/far delta for integration\n"
        "  --vision-d-gain <N>     Filtered visual error-rate yaw gain\n"
        "  --vision-d-filter <s>   Visual derivative low-pass time constant\n"
        "  --vision-d-max-rate <N> Visual derivative clamp in error/s\n"
        "  --vision-error-step <px> Max visual error change per frame\n"
        "  --near-yaw-gain <N>     Near vision error to deg/s\n"
        "  --center-yaw-weight <N> Wheel-to-midline correction weight\n"
        "  --far-yaw-gain <N>      Far vision error to deg/s\n"
        "  --curve-yaw-boost <N>   Extra P gain for confirmed curves\n"
        "  --curve-yaw-shape <N>   Curve response shape, 1 smooth..5 sharp\n"
        "  --curvature-slowdown <N> Speed reduction from path curvature\n"
        "  --min-follow-speed <N>  Minimum follow-speed scale\n"
        "  --max-yaw-rate <deg/s>  Navigation yaw-rate limit\n"
        "  --yaw-rate-kp <N>       Yaw feedback correction gain\n"
        "  --yaw-rate-limit <cm/s> Yaw feedback wheel correction limit\n"
        "  --encoder-yaw-filter <s> Encoder-yaw low-pass time constant\n"
        "  --encoder-yaw-kp-scale <N> Encoder-yaw gain relative to IMU\n"
        "  --encoder-yaw-limit <cm/s> Encoder-yaw correction limit\n"
        "  --yaw-disagree-dps <dps> IMU/encoder opposite-sign threshold\n"
        "  --yaw-disagree-frames <N> Frames before rejecting IMU yaw\n"
        "  --yaw-recover-frames <N> Compatible frames before restoring IMU\n"
        "  --yaw-direction-guard <dps> Wheel-yaw zero-cross guard threshold\n"
        "  --yaw-guard-target <dps> Target yaw that establishes turn direction\n"
        "  --imu-yaw-sign <N>      IMU Z sign; right turn must be positive\n"
        "  --imu-transport <mode>  auto, i2c, or spi\n"
        "  --imu-spi-speed <N>     Shared SPI1 IMU clock Hz, default 8000000\n"
        "  --imu-calibrate <s>     Startup stationary calibration time\n"
        "  --imu-accel-swap-xy     Swap forward/right accelerometer axes\n"
        "  --imu-accel-forward-sign <N> Forward acceleration sign\n"
        "  --imu-accel-right-sign <N> Right acceleration sign\n"
        "  --imu-accel-deadband <g> Acceleration integration deadband\n"
        "  --imu-stationary-zero  Zero integrated velocity while stationary\n"
        "  --no-imu-stationary-zero Disable stationary velocity zeroing\n"
        "  --imu-stationary-accel <g> Stationary acceleration threshold\n"
        "  --imu-stationary-yaw <dps> Stationary yaw-rate threshold\n"
        "  --imu-stationary-hold <s> Required stationary confirmation time\n"
        "  --imu-csv <file>        Write direct IMU velocity/path CSV\n"
        "  --imu-csv-hz <N>        Direct IMU CSV rate, default 50\n"
        "  --inertial-path <csv>   Replay a recorded IMU/odometry path\n"
        "  --line-lost-inertial    Use the inertial path after visual line loss\n"
        "  --inertial-lookahead <cm> Route lookahead distance, default 25\n"
        "  --inertial-speed <cm/s> Override recorded speed; 0 keeps it\n"
        "  --inertial-min-speed <cm/s> Minimum active route speed\n"
        "  --inertial-heading-kp <N> Route heading error to yaw rate\n"
        "  --inertial-align-tol <deg> Heading tolerance before tracking\n"
        "  --inertial-finish-tol <cm> Final waypoint stop radius\n"
        "  --inertial-max-error <cm> Maximum cross-track error\n"
        "  --inertial-error-timeout <s> Error duration before fault\n"
        "  --inertial-sensor-timeout <s> IMU/odometry loss before fault\n"
        "  --tof-slope             Enable VL53L0X ramp detection and boost\n"
        "  --tof-stop-test         Latch an immediate stop when ramp is found\n"
        "  --display               Show live status on the TFT18 (ST7735S)\n"
        "  --display-spi <dev>     Display SPI node, default /dev/spidev1.0\n"
        "  --display-spi-speed <N> Display SPI clock Hz, default 8000000\n"
        "  --display-dc <GPIO>     Display DC GPIO, default 48\n"
        "  --display-rst <GPIO>    Display reset GPIO, default 49\n"
        "  --display-rotation <N>  Display rotation 0..3, default 0\n"
        "  --display-scale <N>     Detail font scale, default 1\n"
        "  --display-hz <N>        Display refresh Hz, default 4\n"
        "  --tof-baseline-timeout <s> Max stationary baseline wait\n"
        "  --tof-ramp-sign <N>     -1: ramp shortens range; +1: lengthens\n"
        "  --tof-ramp-delta <mm>   Absolute ramp distance, default 400mm\n"
        "  --tof-ramp-hyst <mm>    Ramp exit hysteresis\n"
        "  --tof-ramp-enter <N>    Consecutive ramp samples to enter\n"
        "  --tof-ramp-exit <N>     Consecutive flat samples to exit\n"
        "  --ramp-boost <percent>  Added PWM feed-forward on a ramp\n"
        "  --ramp-boost-speed <N>  Minimum ramp target speed in cm/s\n"
        "  --ramp-boost-max <s>    Maximum continuous ramp boost time\n"
        "  --side-road             Enable side-ring navigation\n"
        "  --side-require-tof      Require valid flat TOF before side entry\n"
        "  --no-side-require-tof   Allow visual-only side entry\n"
        "  --side-enter-frames <N> Same-side frames needed for entry\n"
        "  --round-enter-distance <cm> Enter ring control this far ahead, default 35cm\n"
        "  --round-enter-max-error <N> Reject ring approach above this live path error, default 0.25\n"
        "  --side-start-distance <cm> Ignore side openings near startup\n"
        "  --target-actions        Enable NCNN target actions\n"
        "  --target-input-size <N> Classifier input size, default 32\n"
        "  --target-confidence <N> Minimum class confidence, default 0.70\n"
        "  --target-close-size <N> Red-area trigger ratio, default 0.04\n"
        "  --target-path-dir <dir> Target 1..5 inertial CSV directory\n"
        "  --target-path-trigger-y <R> Red-block center trigger row, default 0.50\n"
        "  --target-straight-left <csv> Override target 1/2 left route\n"
        "  --target-straight-right <csv> Override target 1/2 right route\n"
        "  --target-cross1-left <csv> Override target 3 left route\n"
        "  --target-cross1-right <csv> Override target 3 right route\n"
        "  --target-cross2-left <csv> Override target 4 left route\n"
        "  --target-cross2-right <csv> Override target 4 right route\n"
        "  --target-chicane-left <csv> Override target 5 left route\n"
        "  --target-chicane-right <csv> Override target 5 right route\n"
        "  --telemetry-hz <N>      NDJSON telemetry rate, default 20\n"
        "  --wheel-diameter <cm>   Wheel diameter, default 6.5\n"
        "  --wheel-base <cm>       Wheel center spacing, default 15.3\n"
        "  --encoder-lines <N>     Encoder lines, default 1024\n"
        "  --encoder-ratio <N>     Encoder/wheel gear ratio, default 30/68\n"
        "  --encoder-filter-alpha <N> Encoder speed response 0..1, default 0.35\n"
        "  --pwm-freq <Hz>         Motor PWM frequency, default 10000\n"
        "  --no-flip               Do not invert both motor directions\n"
        "  --swap-motors           Exchange logical left/right outputs\n"
        "  --no-swap-motors        Keep logical left/right outputs (default)\n"
        "  --swap-encoders         Exchange encoder feedback only\n"
        "  --no-swap-encoders      Keep raw encoder channels (default)\n"
        "  --allow-reverse         Permit reverse inner wheel targets\n"
        "  --calibration <file>    Row-to-distance table, default config/标定数据.txt\n"
        "  --control-distance <cm> Main lookahead distance, default 60cm\n"
        "  --far-distance <cm>     Far preview distance, default 120cm\n"
        "  --threshold-floor <N>   Minimum road-score threshold, default 70\n"
        "  --no-vision-color-filter Disable saturation/blue rejection\n"
        "  --saturation-penalty <N> Saturation penalty percent, default 60\n"
        "  --no-blue-reject        Disable explicit blue-highlight rejection\n"
        "  --blue-hue-low <N>      OpenCV blue hue lower bound, default 85\n"
        "  --blue-hue-high <N>     OpenCV blue hue upper bound, default 135\n"
        "  --blue-saturation-min <N> Blue saturation gate, default 35\n"
        "  --blue-value-min <N>    Blue brightness gate, default 55\n"
        "  --blue-penalty <N>      Extra blue road-score penalty, default 80\n"
        "  --horizon-row <N>       Override binary horizon row 0..59\n"
        "  --horizon-image-row <N> Override calibration/image horizon row\n"
        "  --no-wheel-mask         Disable lower wheel exclusion box\n"
        "  --wheel-box-center <R>  Wheel box center x ratio, default 0.50\n"
        "  --wheel-box-width <R>   Wheel box width ratio, default 0.28\n"
        "  --wheel-box-top <R>     Wheel box top y ratio, default 0.72\n"
        "  --wheel-box-bottom <R>  Wheel box bottom y ratio, default 0.98\n"
        "  --edge-smooth <N>       Side edge smoothing window, default 1 (live frame)\n"
        "  --bottom-fit-rows <N>   Rows used for bottom side-line fit, default 12\n"
        "  --cross-enter <N>       Consecutive cross frames to enter\n"
        "  --cross-exit <N>        Centered paired-line frames to exit\n"
        "  --cross-min-distance <cm> Minimum locked crossing distance\n"
        "  --cross-min-time <s>    Minimum cross heading-lock time, default 0.8\n"
        "  --cross-timeout <s>     Maximum cross heading-lock duration\n"
        "  --cross-speed-scale <N> Cruise-speed multiplier in a cross\n"
        "  --cross-heading-grid <deg> Snap cross heading to this IMU grid\n"
        "  --cross-heading-tol <deg> Maximum IMU angle from grid to enter\n"
        "  --cross-enter-max-yaw <deg/s> Maximum turn rate while confirming\n"
        "  --cross-corner-tol <N>  Max row gap for paired cross corners, default 5\n"
        "  --heading-hold-kp <N>   Heading error to yaw-rate gain\n"
        "  --heading-hold-max-rate <deg/s> Heading correction limit\n"
        "  --branch-window <N>     Sliding branch slope window rows, default 5\n"
        "  --branch-min-rows <N>   Minimum branch segment rows, default 3\n"
        "  --branch-return-rows <N> Return rows needed near fit line, default 3\n"
        "  --branch-slope <N>      Slope delta threshold, default 0.55\n"
        "  --branch-dev <N>        Outward deviation threshold px, default 5\n"
        "  --branch-return <N>     Fit-line return tolerance px, default 3\n"
        "  --lock-slope-tol <N>    Unlock CROSS_LOCK if quantile slope drifts, default 0.35\n"
        "  --round-stable-tol <N>  Max mean search/prediction gap on stable side, default 2\n"
        "  --no-far-search         Stop using horizon-wide far search\n"
        "  --no-rotate             Do not rotate camera frame 180 degrees\n"
        "  -h, --help              Show this help\n",
        exe);
}

void PathParams::parse(int argc, char** argv) {
    auto read_int = [&](int& out, int& i) {
        if (!has_value(argc, i)) return;
        out = std::atoi(argv[++i]);
    };
    auto read_double = [&](double& out, int& i) {
        if (!has_value(argc, i)) return;
        out = std::atof(argv[++i]);
    };

    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "-h" || arg == "--help") {
            print_help(argv[0]);
            std::exit(0);
        } else if (arg == "--camera") {
            if (has_value(argc, i)) camera_device = argv[++i];
        } else if (arg == "--control-hz") {
            read_int(control_hz, i);
        } else if (arg == "--http") {
            use_http = true;
            if (next_is_value(argc, argv, i)) {
                http_port = static_cast<std::uint16_t>(std::atoi(argv[++i]));
            }
        } else if (arg == "--http-fps") {
            read_int(http_fps_limit, i);
        } else if (arg == "--http-quality") {
            read_int(http_jpeg_quality, i);
        } else if (arg == "--model-dir") {
            if (has_value(argc, i)) model_dir = argv[++i];
        } else if (arg == "--enable-motors") {
            dry_run = false;
        } else if (arg == "--dry-run") {
            dry_run = true;
        } else if (arg == "--no-ncnn") {
            use_ncnn = false;
        } else if (arg == "--no-stop") {
            enable_stop = false;
        } else if (arg == "--base-speed") {
            read_double(base_speed_cmps, i);
        } else if (arg == "--max-speed") {
            read_double(max_speed_cmps, i);
        } else if (arg == "--hard-turn-outer-scale") {
            read_double(hard_turn_outer_speed_scale, i);
        } else if (arg == "--max-percent") {
            read_double(max_percent, i);
        } else if (arg == "--target-accel") {
            read_double(target_accel_cmps2, i);
        } else if (arg == "--target-decel") {
            read_double(target_decel_cmps2, i);
        } else if (arg == "--target-yaw-slew") {
            read_double(target_yaw_slew_dps2, i);
        } else if (arg == "--pwm-slew") {
            read_double(max_percent_delta_per_s, i);
        } else if (arg == "--speed-ff") {
            read_double(speed_ff, i);
        } else if (arg == "--speed-kp") {
            read_double(wheel_speed_kp, i);
        } else if (arg == "--speed-ki") {
            read_double(wheel_speed_ki, i);
        } else if (arg == "--inner-brake") {
            read_double(inner_wheel_brake_max_percent, i);
        } else if (arg == "--inner-brake-margin") {
            read_double(inner_wheel_brake_margin_cmps, i);
        } else if (arg == "--vision-yaw-sign") {
            read_double(vision_yaw_sign, i);
        } else if (arg == "--vision-i-gain") {
            read_double(vision_i_gain, i);
        } else if (arg == "--vision-i-limit") {
            read_double(vision_i_limit, i);
        } else if (arg == "--vision-i-max-error") {
            read_double(vision_i_max_error, i);
        } else if (arg == "--vision-i-curve-delta") {
            read_double(vision_i_curve_delta, i);
        } else if (arg == "--vision-d-gain") {
            read_double(vision_d_gain, i);
        } else if (arg == "--vision-d-filter") {
            read_double(vision_d_filter_tau_s, i);
        } else if (arg == "--vision-d-max-rate") {
            read_double(vision_d_max_error_rate, i);
        } else if (arg == "--vision-error-step") {
            read_double(error_step_limit, i);
        } else if (arg == "--near-yaw-gain") {
            read_double(near_yaw_gain, i);
        } else if (arg == "--center-yaw-weight") {
            read_double(center_yaw_weight, i);
        } else if (arg == "--far-yaw-gain") {
            read_double(far_yaw_gain, i);
        } else if (arg == "--curve-yaw-boost") {
            read_double(curve_yaw_boost, i);
        } else if (arg == "--curve-yaw-shape") {
            read_double(curve_yaw_shape, i);
        } else if (arg == "--curvature-slowdown") {
            read_double(curvature_slowdown, i);
        } else if (arg == "--min-follow-speed") {
            read_double(minimum_follow_speed_scale, i);
        } else if (arg == "--min-follow-yaw-scale") {
            read_double(minimum_follow_yaw_scale, i);
        } else if (arg == "--max-yaw-rate") {
            read_double(max_yaw_rate_dps, i);
        } else if (arg == "--yaw-rate-kp") {
            read_double(yaw_rate_kp, i);
        } else if (arg == "--yaw-rate-limit") {
            read_double(yaw_rate_correction_limit_cmps, i);
        } else if (arg == "--encoder-yaw-filter") {
            read_double(encoder_yaw_filter_tau_s, i);
        } else if (arg == "--encoder-yaw-kp-scale") {
            read_double(encoder_yaw_kp_scale, i);
        } else if (arg == "--encoder-yaw-limit") {
            read_double(encoder_yaw_correction_limit_cmps, i);
        } else if (arg == "--yaw-disagree-dps") {
            read_double(yaw_sensor_disagreement_dps, i);
        } else if (arg == "--yaw-disagree-frames") {
            read_int(yaw_sensor_disagreement_frames, i);
        } else if (arg == "--yaw-recover-frames") {
            read_int(yaw_sensor_recovery_frames, i);
        } else if (arg == "--yaw-direction-guard") {
            read_double(yaw_direction_guard_dps, i);
        } else if (arg == "--yaw-guard-target") {
            read_double(yaw_direction_guard_target_dps, i);
        } else if (arg == "--imu-yaw-sign") {
            read_double(imu_yaw_sign, i);
        } else if (arg == "--imu-transport") {
            if (has_value(argc, i)) imu_transport = argv[++i];
        } else if (arg == "--imu-spi-speed") {
            read_int(imu_spi_speed_hz, i);
        } else if (arg == "--imu-calibrate") {
            read_double(imu_calibrate_s, i);
        } else if (arg == "--imu-accel-swap-xy") {
            imu_accel_swap_xy = true;
        } else if (arg == "--imu-accel-forward-sign") {
            read_double(imu_accel_forward_sign, i);
        } else if (arg == "--imu-accel-right-sign") {
            read_double(imu_accel_right_sign, i);
        } else if (arg == "--imu-accel-deadband") {
            read_double(imu_accel_deadband_g, i);
        } else if (arg == "--imu-stationary-zero") {
            imu_stationary_zero = true;
        } else if (arg == "--no-imu-stationary-zero") {
            imu_stationary_zero = false;
        } else if (arg == "--imu-stationary-accel") {
            read_double(imu_stationary_accel_g, i);
        } else if (arg == "--imu-stationary-yaw") {
            read_double(imu_stationary_yaw_dps, i);
        } else if (arg == "--imu-stationary-hold") {
            read_double(imu_stationary_hold_s, i);
        } else if (arg == "--imu-csv") {
            if (has_value(argc, i)) imu_csv_path = argv[++i];
        } else if (arg == "--imu-csv-hz") {
            read_int(imu_csv_hz, i);
        } else if (arg == "--inertial-path") {
            if (has_value(argc, i)) inertial_path = argv[++i];
        } else if (arg == "--line-lost-inertial") {
            line_lost_inertial = true;
        } else if (arg == "--inertial-lookahead") {
            read_double(inertial_lookahead_cm, i);
        } else if (arg == "--inertial-speed") {
            read_double(inertial_speed_cmps, i);
        } else if (arg == "--inertial-min-speed") {
            read_double(inertial_min_speed_cmps, i);
        } else if (arg == "--inertial-heading-kp") {
            read_double(inertial_heading_kp, i);
        } else if (arg == "--inertial-align-tol") {
            read_double(inertial_align_tolerance_deg, i);
        } else if (arg == "--inertial-finish-tol") {
            read_double(inertial_finish_tolerance_cm, i);
        } else if (arg == "--inertial-max-error") {
            read_double(inertial_max_deviation_cm, i);
        } else if (arg == "--inertial-error-timeout") {
            read_double(inertial_deviation_timeout_s, i);
        } else if (arg == "--inertial-sensor-timeout" ||
                   arg == "--inertial-imu-timeout") {
            read_double(inertial_sensor_timeout_s, i);
        } else if (arg == "--tof-slope") {
            enable_tof_slope = true;
        } else if (arg == "--tof-stop-test") {
            enable_tof_slope = true;
            tof_stop_test = true;
        } else if (arg == "--no-tof-slope") {
            enable_tof_slope = false;
            tof_stop_test = false;
        } else if (arg == "--tof-baseline-timeout") {
            read_double(tof_baseline_timeout_s, i);
        } else if (arg == "--tof-ramp-sign") {
            read_double(tof_ramp_sign, i);
        } else if (arg == "--tof-ramp-delta") {
            read_double(tof_ramp_delta_mm, i);
        } else if (arg == "--tof-ramp-hyst") {
            read_double(tof_ramp_hysteresis_mm, i);
        } else if (arg == "--tof-ramp-enter") {
            read_int(tof_ramp_enter_frames, i);
        } else if (arg == "--tof-ramp-exit") {
            read_int(tof_ramp_exit_frames, i);
        } else if (arg == "--ramp-boost") {
            read_double(ramp_boost_percent, i);
        } else if (arg == "--ramp-boost-speed") {
            read_double(ramp_boost_speed_cmps, i);
        } else if (arg == "--ramp-boost-max") {
            read_double(ramp_boost_max_s, i);
        } else if (arg == "--side-road") {
            enable_side_road = true;
        } else if (arg == "--side-require-tof") {
            side_require_tof = true;
        } else if (arg == "--no-side-require-tof") {
            side_require_tof = false;
        } else if (arg == "--side-enter-frames") {
            read_int(round_enter_frames, i);
        } else if (arg == "--round-enter-distance") {
            read_double(round_enter_distance_cm, i);
        } else if (arg == "--round-enter-max-error") {
            read_double(round_enter_max_path_error, i);
        } else if (arg == "--side-start-distance") {
            read_double(side_start_distance_cm, i);
        } else if (arg == "--target-actions") {
            enable_target_actions = true;
        } else if (arg == "--target-input-size") {
            read_int(target_input_size, i);
        } else if (arg == "--target-confidence") {
            read_double(target_enter_confidence, i);
        } else if (arg == "--target-close-size") {
            read_double(target_close_size, i);
        } else if (arg == "--target-path-dir") {
            if (has_value(argc, i)) target_path_dir = argv[++i];
        } else if (arg == "--target-path-trigger-y") {
            read_double(target_path_trigger_y, i);
        } else if (arg == "--target-straight-left") {
            if (has_value(argc, i)) target_straight_left_path = argv[++i];
        } else if (arg == "--target-straight-right") {
            if (has_value(argc, i)) target_straight_right_path = argv[++i];
        } else if (arg == "--target-cross1-left") {
            if (has_value(argc, i)) target_cross_1_left_path = argv[++i];
        } else if (arg == "--target-cross1-right") {
            if (has_value(argc, i)) target_cross_1_right_path = argv[++i];
        } else if (arg == "--target-cross2-left") {
            if (has_value(argc, i)) target_cross_2_left_path = argv[++i];
        } else if (arg == "--target-cross2-right") {
            if (has_value(argc, i)) target_cross_2_right_path = argv[++i];
        } else if (arg == "--target-chicane-left") {
            if (has_value(argc, i)) target_chicane_left_path = argv[++i];
        } else if (arg == "--target-chicane-right") {
            if (has_value(argc, i)) target_chicane_right_path = argv[++i];
        } else if (arg == "--telemetry-hz") {
            read_int(telemetry_hz, i);
        } else if (arg == "--display") {
            enable_display = true;
        } else if (arg == "--no-display") {
            enable_display = false;
        } else if (arg == "--display-spi") {
            if (has_value(argc, i)) display_spi_device = argv[++i];
        } else if (arg == "--display-spi-speed") {
            read_int(display_spi_speed_hz, i);
        } else if (arg == "--display-dc") {
            read_int(display_dc_gpio, i);
        } else if (arg == "--display-rst") {
            read_int(display_reset_gpio, i);
        } else if (arg == "--display-rotation") {
            read_int(display_rotation, i);
        } else if (arg == "--display-scale") {
            read_int(display_scale, i);
        } else if (arg == "--display-hz") {
            read_int(display_refresh_hz, i);
        } else if (arg == "--wheel-diameter") {
            read_double(wheel_diameter_cm, i);
        } else if (arg == "--wheel-base") {
            read_double(wheel_base_cm, i);
        } else if (arg == "--encoder-lines") {
            read_int(encoder_lines, i);
        } else if (arg == "--encoder-ratio") {
            read_double(encoder_gear_ratio, i);
        } else if (arg == "--encoder-filter-alpha") {
            read_double(encoder_filter_alpha, i);
        } else if (arg == "--pwm-freq") {
            read_double(pwm_frequency_hz, i);
        } else if (arg == "--no-flip") {
            flip_motors = false;
        } else if (arg == "--no-swap-motors") {
            swap_motors = false;
        } else if (arg == "--swap-motors") {
            swap_motors = true;
        } else if (arg == "--no-swap-encoders") {
            swap_encoders = false;
        } else if (arg == "--swap-encoders") {
            swap_encoders = true;
        } else if (arg == "--allow-reverse") {
            forbid_reverse = false;
        } else if (arg == "--calibration") {
            if (has_value(argc, i)) calibration_path = argv[++i];
        } else if (arg == "--control-distance") {
            read_double(control_distance_cm, i);
        } else if (arg == "--far-distance") {
            read_double(far_distance_cm, i);
        } else if (arg == "--calib-height") {
            read_int(calibration_image_height, i);
        } else if (arg == "--horizon-row") {
            read_int(horizon_row, i);
        } else if (arg == "--horizon-image-row") {
            read_double(horizon_image_row, i);
        } else if (arg == "--no-wheel-mask") {
            wheel_mask_enable = false;
        } else if (arg == "--wheel-box-center") {
            read_double(wheel_box_center_ratio, i);
        } else if (arg == "--wheel-box-width") {
            read_double(wheel_box_width_ratio, i);
        } else if (arg == "--wheel-box-top") {
            read_double(wheel_box_top_ratio, i);
        } else if (arg == "--wheel-box-bottom") {
            read_double(wheel_box_bottom_ratio, i);
        } else if (arg == "--edge-smooth") {
            read_int(edge_smooth_window, i);
        } else if (arg == "--bottom-fit-rows") {
            read_int(bottom_fit_rows, i);
        } else if (arg == "--cross-corner-tol") {
            read_int(cross_corner_row_tolerance, i);
        } else if (arg == "--stable-two-side-rows") {
            read_int(stable_two_side_min_rows, i);
        } else if (arg == "--branch-window") {
            read_int(branch_window_rows, i);
        } else if (arg == "--branch-min-rows") {
            read_int(branch_min_rows, i);
        } else if (arg == "--branch-return-rows") {
            read_int(branch_return_rows, i);
        } else if (arg == "--branch-slope") {
            read_double(branch_slope_delta, i);
        } else if (arg == "--branch-dev") {
            read_double(branch_min_deviation, i);
        } else if (arg == "--branch-return") {
            read_double(branch_return_tolerance, i);
        } else if (arg == "--lock-slope-tol") {
            read_double(lock_slope_tolerance, i);
        } else if (arg == "--round-stable-tol") {
            read_double(round_stable_tolerance, i);
        } else if (arg == "--no-far-search") {
            far_search_enable = false;
        } else if (arg == "--no-rotate") {
            rotate_180 = false;
        } else if (arg == "--cross-enter") {
            read_int(cross_enter_frames, i);
        } else if (arg == "--cross-exit") {
            read_int(cross_exit_frames, i);
        } else if (arg == "--cross-min-distance") {
            read_double(cross_min_dist_cm, i);
        } else if (arg == "--cross-min-time") {
            read_double(cross_min_time_s, i);
        } else if (arg == "--cross-timeout") {
            read_double(cross_timeout_s, i);
        } else if (arg == "--cross-speed-scale") {
            read_double(cross_speed_scale, i);
        } else if (arg == "--cross-heading-grid") {
            read_double(cross_heading_grid_deg, i);
        } else if (arg == "--cross-heading-tol") {
            read_double(cross_heading_tolerance_deg, i);
        } else if (arg == "--cross-enter-max-yaw") {
            read_double(cross_enter_max_yaw_rate_dps, i);
        } else if (arg == "--zebra-enter") {
            read_int(zebra_enter_frames, i);
        } else if (arg == "--zebra-exit") {
            read_int(zebra_exit_frames, i);
        } else if (arg == "--heading-hold-kp") {
            read_double(heading_hold_kp, i);
        } else if (arg == "--heading-hold-max-rate") {
            read_double(heading_hold_max_rate_dps, i);
        } else if (arg == "--round-enter-frames") {
            read_int(round_enter_frames, i);
        } else if (arg == "--round-enter-distance") {
            read_double(round_enter_distance_cm, i);
        } else if (arg == "--round-enter-max-error") {
            read_double(round_enter_max_path_error, i);
        } else if (arg == "--threshold-floor") {
            read_int(threshold_floor, i);
        } else if (arg == "--no-vision-color-filter") {
            vision_color_filter = false;
        } else if (arg == "--vision-color-filter") {
            vision_color_filter = true;
        } else if (arg == "--saturation-penalty") {
            read_int(vision_saturation_penalty, i);
        } else if (arg == "--no-blue-reject") {
            vision_blue_reject = false;
        } else if (arg == "--blue-reject") {
            vision_blue_reject = true;
        } else if (arg == "--blue-hue-low") {
            read_int(vision_blue_hue_low, i);
        } else if (arg == "--blue-hue-high") {
            read_int(vision_blue_hue_high, i);
        } else if (arg == "--blue-saturation-min") {
            read_int(vision_blue_saturation_min, i);
        } else if (arg == "--blue-value-min") {
            read_int(vision_blue_value_min, i);
        } else if (arg == "--blue-penalty") {
            read_int(vision_blue_penalty, i);
        } else if (arg == "--forward-row") {
            read_int(forward_row, i);
        } else if (arg == "--far-row") {
            read_int(far_row, i);
        } else {
            std::fprintf(stderr, "Unknown option: %s\n", arg.c_str());
            print_help(argv[0]);
            std::exit(2);
        }
    }

    if (http_fps_limit < 1) http_fps_limit = 1;
    if (http_fps_limit > 30) http_fps_limit = 30;
    if (http_jpeg_quality < 1) http_jpeg_quality = 1;
    if (http_jpeg_quality > 95) http_jpeg_quality = 95;
    control_hz = std::max(20, std::min(100, control_hz));
    telemetry_hz = std::max(1, std::min(100, telemetry_hz));
    base_speed_cmps = std::max(0.0, base_speed_cmps);
    max_speed_cmps = std::max(base_speed_cmps, max_speed_cmps);
    hard_turn_outer_speed_scale = clamp_value(
        hard_turn_outer_speed_scale, 1.0, 2.5);
    wheel_diameter_cm = std::max(1.0, wheel_diameter_cm);
    wheel_base_cm = std::max(1.0, wheel_base_cm);
    encoder_lines = std::max(1, encoder_lines);
    encoder_gear_ratio = std::max(1e-6, encoder_gear_ratio);
    encoder_filter_alpha = clamp_value(
        encoder_filter_alpha, 0.05, 1.0);
    target_accel_cmps2 = std::max(1.0, target_accel_cmps2);
    target_decel_cmps2 = std::max(1.0, target_decel_cmps2);
    target_yaw_slew_dps2 = std::max(1.0, target_yaw_slew_dps2);
    max_percent_delta_per_s = clamp_value(
        max_percent_delta_per_s, 10.0, 1000.0);
    speed_ff = std::max(0.0, speed_ff);
    wheel_speed_kp = std::max(0.0, wheel_speed_kp);
    wheel_speed_ki = std::max(0.0, wheel_speed_ki);
    inner_wheel_brake_max_percent = clamp_value(
        inner_wheel_brake_max_percent, 0.0, max_percent);
    inner_wheel_brake_margin_cmps = std::max(
        0.0, inner_wheel_brake_margin_cmps);
    vision_yaw_sign = vision_yaw_sign < 0.0 ? -1.0 : 1.0;
    vision_i_gain = std::max(0.0, vision_i_gain);
    vision_i_limit = clamp_value(vision_i_limit, 0.0, 2.0);
    vision_i_max_error = clamp_value(vision_i_max_error, 0.02, 1.0);
    vision_i_curve_delta = clamp_value(
        vision_i_curve_delta, 0.01, 1.0);
    vision_i_decay_rate = clamp_value(
        vision_i_decay_rate, 0.1, 20.0);
    vision_d_gain = std::max(0.0, vision_d_gain);
    vision_d_filter_tau_s = clamp_value(
        vision_d_filter_tau_s, 0.01, 1.0);
    vision_d_max_error_rate = clamp_value(
        vision_d_max_error_rate, 0.05, 10.0);
    error_step_limit = clamp_value(error_step_limit, 0.5, 5.0);
    center_yaw_weight = std::max(0.0, center_yaw_weight);
    curve_yaw_boost = std::max(0.0, curve_yaw_boost);
    curve_yaw_shape = clamp_value(curve_yaw_shape, 1.0, 5.0);
    curvature_slowdown = clamp_value(
        curvature_slowdown, 0.0, 0.90);
    minimum_follow_speed_scale = clamp_value(
        minimum_follow_speed_scale, 0.10, 1.0);
    minimum_follow_yaw_scale = clamp_value(
        minimum_follow_yaw_scale, 0.30, 1.0);
    max_yaw_rate_dps = std::max(1.0, max_yaw_rate_dps);
    yaw_rate_kp = std::max(0.0, yaw_rate_kp);
    yaw_rate_correction_limit_cmps = std::max(
        0.0, yaw_rate_correction_limit_cmps);
    encoder_yaw_filter_tau_s = clamp_value(
        encoder_yaw_filter_tau_s, 0.02, 2.0);
    encoder_yaw_kp_scale = clamp_value(
        encoder_yaw_kp_scale, 0.0, 1.0);
    encoder_yaw_correction_limit_cmps = std::max(
        0.0, encoder_yaw_correction_limit_cmps);
    yaw_sensor_disagreement_dps = clamp_value(
        yaw_sensor_disagreement_dps, 5.0, 500.0);
    yaw_sensor_disagreement_frames = clamp_value(
        yaw_sensor_disagreement_frames, 1, 50);
    yaw_sensor_recovery_frames = clamp_value(
        yaw_sensor_recovery_frames, 1, 200);
    yaw_direction_guard_dps = clamp_value(
        yaw_direction_guard_dps, 0.0, 500.0);
    yaw_direction_guard_target_dps = clamp_value(
        yaw_direction_guard_target_dps, 1.0, max_yaw_rate_dps);
    if (imu_transport != "auto" &&
        imu_transport != "i2c" &&
        imu_transport != "spi") {
        std::fprintf(
            stderr,
            "Invalid --imu-transport: %s (expected auto, i2c, or spi)\n",
            imu_transport.c_str());
        std::exit(2);
    }
    imu_spi_speed_hz = std::max(100000, std::min(10000000, imu_spi_speed_hz));
    imu_accel_forward_sign = imu_accel_forward_sign < 0.0 ? -1.0 : 1.0;
    imu_accel_right_sign = imu_accel_right_sign < 0.0 ? -1.0 : 1.0;
    imu_accel_deadband_g = clamp_value(imu_accel_deadband_g, 0.0, 0.10);
    imu_stationary_accel_g = clamp_value(
        imu_stationary_accel_g, 0.001, 0.25);
    imu_stationary_yaw_dps = clamp_value(
        imu_stationary_yaw_dps, 0.1, 50.0);
    imu_stationary_hold_s = clamp_value(
        imu_stationary_hold_s, 0.0, 5.0);
    imu_csv_hz = clamp_value(imu_csv_hz, 1, 104);
    inertial_lookahead_cm = clamp_value(inertial_lookahead_cm, 5.0, 200.0);
    inertial_speed_cmps = clamp_value(inertial_speed_cmps, 0.0, max_speed_cmps);
    inertial_min_speed_cmps = clamp_value(
        inertial_min_speed_cmps, 1.0, max_speed_cmps);
    inertial_heading_kp = clamp_value(inertial_heading_kp, 0.1, 10.0);
    inertial_align_tolerance_deg = clamp_value(
        inertial_align_tolerance_deg, 1.0, 45.0);
    inertial_finish_tolerance_cm = clamp_value(
        inertial_finish_tolerance_cm, 1.0, 100.0);
    inertial_max_deviation_cm = clamp_value(
        inertial_max_deviation_cm, 5.0, 500.0);
    inertial_deviation_timeout_s = clamp_value(
        inertial_deviation_timeout_s, 0.05, 5.0);
    inertial_sensor_timeout_s = clamp_value(
        inertial_sensor_timeout_s, 0.05, 5.0);
    tof_baseline_frames = std::max(3, tof_baseline_frames);
    tof_baseline_timeout_s = clamp_value(
        tof_baseline_timeout_s, 1.0, 30.0);
    tof_ramp_enter_frames = std::max(1, tof_ramp_enter_frames);
    tof_ramp_exit_frames = std::max(1, tof_ramp_exit_frames);
    cross_enter_frames = std::max(1, cross_enter_frames);
    cross_exit_frames = std::max(1, cross_exit_frames);
    zebra_enter_frames = std::max(1, zebra_enter_frames);
    zebra_exit_frames = std::max(1, zebra_exit_frames);
    cross_min_dist_cm = std::max(0.0, cross_min_dist_cm);
    cross_timeout_s = clamp_value(cross_timeout_s, 0.2, 10.0);
    cross_min_time_s = clamp_value(
        cross_min_time_s, 0.1, cross_timeout_s);
    cross_speed_scale = clamp_value(cross_speed_scale, 0.10, 1.0);
    cross_heading_grid_deg = clamp_value(
        cross_heading_grid_deg, 0.0, 180.0);
    cross_heading_tolerance_deg = clamp_value(
        cross_heading_tolerance_deg, 0.0,
        cross_heading_grid_deg > 0.0
            ? cross_heading_grid_deg * 0.5 : 90.0);
    cross_enter_max_yaw_rate_dps = clamp_value(
        cross_enter_max_yaw_rate_dps, 1.0, max_yaw_rate_dps);
    heading_hold_kp = std::max(0.0, heading_hold_kp);
    heading_hold_max_rate_dps = clamp_value(
        heading_hold_max_rate_dps, 1.0, max_yaw_rate_dps);
    round_enter_frames = std::max(2, round_enter_frames);
    round_enter_distance_cm = clamp_value(
        round_enter_distance_cm, 10.0, 300.0);
    round_enter_max_path_error = clamp_value(
        round_enter_max_path_error, 0.05, 1.0);
    target_input_size = clamp_value(target_input_size, 16, 512);
    target_enter_confidence = clamp_value(target_enter_confidence, 0.0, 1.0);
    target_close_size = clamp_value(target_close_size, 0.001, 1.0);
    target_path_trigger_y = clamp_value(target_path_trigger_y, 0.0, 1.0);
    target_enter_frames = std::max(1, target_enter_frames);
    side_start_distance_cm = std::max(0.0, side_start_distance_cm);
    tof_max_age_s = std::max(0.05, tof_max_age_s);
    tof_ramp_sign = tof_ramp_sign < 0.0 ? -1.0 : 1.0;
    tof_ramp_delta_mm = std::max(5.0, tof_ramp_delta_mm);
    tof_ramp_hysteresis_mm = clamp_value(
        tof_ramp_hysteresis_mm, 0.0, tof_ramp_delta_mm);
    ramp_boost_percent = clamp_value(
        ramp_boost_percent, 0.0, max_percent);
    ramp_boost_speed_cmps = clamp_value(
        ramp_boost_speed_cmps, 0.0, max_speed_cmps);
    ramp_boost_max_s = std::max(0.2, ramp_boost_max_s);
    pwm_frequency_hz = std::max(100.0, pwm_frequency_hz);
    threshold_floor = clamp_value(threshold_floor, 0, 255);
    vision_saturation_penalty = clamp_value(
        vision_saturation_penalty, 0, 200);
    vision_blue_hue_low = clamp_value(vision_blue_hue_low, 0, 179);
    vision_blue_hue_high = clamp_value(vision_blue_hue_high, 0, 179);
    if (vision_blue_hue_high < vision_blue_hue_low) {
        std::swap(vision_blue_hue_low, vision_blue_hue_high);
    }
    vision_blue_saturation_min = clamp_value(
        vision_blue_saturation_min, 0, 255);
    vision_blue_value_min = clamp_value(vision_blue_value_min, 0, 255);
    vision_blue_penalty = clamp_value(vision_blue_penalty, 0, 255);
    if (horizon_row >= kBinaryHeight) horizon_row = kBinaryHeight - 1;
    if (edge_smooth_window < 1) edge_smooth_window = 1;
    if (bottom_fit_rows < 2) bottom_fit_rows = 2;
    if (cross_corner_row_tolerance < 0) cross_corner_row_tolerance = 0;
    if (stable_two_side_min_rows < 1) stable_two_side_min_rows = 1;
    if (branch_window_rows < 2) branch_window_rows = 2;
    if (branch_min_rows < 1) branch_min_rows = 1;
    if (branch_return_rows < 1) branch_return_rows = 1;
    if (branch_slope_delta < 0.0) branch_slope_delta = 0.0;
    if (branch_min_deviation < 0.0) branch_min_deviation = 0.0;
    if (branch_return_tolerance < 0.0) branch_return_tolerance = 0.0;
    if (lock_slope_tolerance < 0.0) lock_slope_tolerance = 0.0;
    if (round_stable_tolerance < 0.1) round_stable_tolerance = 0.1;
    wheel_box_center_ratio = std::max(0.0, std::min(1.0, wheel_box_center_ratio));
    wheel_box_width_ratio = std::max(0.02, std::min(0.90, wheel_box_width_ratio));
    wheel_box_top_ratio = std::max(0.0, std::min(1.0, wheel_box_top_ratio));
    wheel_box_bottom_ratio = std::max(0.0, std::min(1.0, wheel_box_bottom_ratio));
    if (wheel_box_bottom_ratio < wheel_box_top_ratio) {
        std::swap(wheel_box_bottom_ratio, wheel_box_top_ratio);
    }

    if (!target_path_dir.empty()) {
        if (target_straight_left_path.empty()) {
            target_straight_left_path = join_path(
                target_path_dir, "straight_left.csv");
        }
        if (target_straight_right_path.empty()) {
            target_straight_right_path = join_path(
                target_path_dir, "straight_right.csv");
        }
        if (target_cross_1_left_path.empty()) {
            target_cross_1_left_path = join_path(
                target_path_dir, "cross_1_left.csv");
        }
        if (target_cross_1_right_path.empty()) {
            target_cross_1_right_path = join_path(
                target_path_dir, "cross_1_right.csv");
        }
        if (target_cross_2_left_path.empty()) {
            target_cross_2_left_path = join_path(
                target_path_dir, "cross_2_left.csv");
        }
        if (target_cross_2_right_path.empty()) {
            target_cross_2_right_path = join_path(
                target_path_dir, "cross_2_right.csv");
        }
        if (target_chicane_left_path.empty()) {
            target_chicane_left_path = join_path(
                target_path_dir, "chicane_left.csv");
        }
        if (target_chicane_right_path.empty()) {
            target_chicane_right_path = join_path(
                target_path_dir, "chicane_right.csv");
        }
    }
    if (target_inertial_routes_enabled()) enable_target_actions = true;
}

bool PathParams::target_inertial_routes_enabled() const {
    return !target_straight_left_path.empty() ||
        !target_straight_right_path.empty() ||
        !target_cross_1_left_path.empty() ||
        !target_cross_1_right_path.empty() ||
        !target_cross_2_left_path.empty() ||
        !target_cross_2_right_path.empty() ||
        !target_chicane_left_path.empty() ||
        !target_chicane_right_path.empty();
}

std::string PathParams::target_inertial_path_for(
    int encounter, TargetKind kind) const {
    const bool left = kind == TargetKind::Weapon;
    const bool right = kind == TargetKind::Supply;
    if (!left && !right) return std::string();
    if (encounter == 1 || encounter == 2) {
        return left ? target_straight_left_path : target_straight_right_path;
    }
    if (encounter == 3) {
        return left ? target_cross_1_left_path : target_cross_1_right_path;
    }
    if (encounter == 4) {
        return left ? target_cross_2_left_path : target_cross_2_right_path;
    }
    if (encounter == 5) {
        return left ? target_chicane_left_path : target_chicane_right_path;
    }
    return std::string();
}

void PathParams::print_banner() const {
    std::printf("rewrite path follow\n");
    std::printf("  camera=%s %dx%d@%d control=%dHz rotate=%s\n",
                camera_device.c_str(), camera_width, camera_height, camera_fps,
                control_hz,
                rotate_180 ? "on" : "off");
    std::printf(
                "  motors=%s speed=%.1f/%.1fcm/s "
                "hard_outer=%.2fx/%.1fcm/s max_pwm=%.1f%%\n",
                dry_run ? "dry-run" : "live", base_speed_cmps,
                max_speed_cmps, hard_turn_outer_speed_scale,
                max_speed_cmps * hard_turn_outer_speed_scale,
                max_percent);
    std::printf("  target_yaw_slew=%.0fdeg/s2 max_yaw=%.0fdeg/s\n",
                target_yaw_slew_dps2, max_yaw_rate_dps);
    std::printf("  http=%s port=%u fps=%d quality=%d\n",
                use_http ? "on" : "off", http_port, http_fps_limit,
                http_jpeg_quality);
    std::printf("  display=%s spi=%s@%dHz rotation=%d scale=%d refresh=%dHz\n",
                enable_display ? "on" : "off", display_spi_device.c_str(),
                display_spi_speed_hz, display_rotation, display_scale,
                display_refresh_hz);
    std::printf("  ncnn=%s input=%d target_actions=%s side_road=%s stop=%s\n",
                use_ncnn ? "on" : "off",
                target_input_size,
                enable_target_actions ? "on" : "off",
                enable_side_road ? "on" : "off",
                enable_stop ? "on" : "off");
    if (target_inertial_routes_enabled()) {
        std::printf(
            "  target_inertial=on trigger_y>=%.2f sequence="
            "straight(1/2),cross_1(3),cross_2(4),chicane(5)\n",
            target_path_trigger_y);
    }
    std::printf(
        "  vision_yaw_sign=%+.0f PID(p=%.0f/%.0f center=%.2fx curve=%.2f/%.1f "
        "i=%.1f limit=%.2f d=%.1f tau=%.2fs rate<=%.2f step<=%.1fpx) "
        "slow=%.2f min_speed=%.2f min_yaw=%.2f\n",
        vision_yaw_sign,
        near_yaw_gain,
        far_yaw_gain,
        center_yaw_weight,
        curve_yaw_boost,
        curve_yaw_shape,
        vision_i_gain,
        vision_i_limit,
        vision_d_gain,
        vision_d_filter_tau_s,
        vision_d_max_error_rate,
        error_step_limit,
        curvature_slowdown,
        minimum_follow_speed_scale,
        minimum_follow_yaw_scale);
    std::printf(
        "  cross=%d/%d frames distance>=%.0fcm time>=%.1fs timeout=%.1fs "
        "speed=%.2f heading_kp=%.1f rate<=%.0fdps "
        "grid=%.0f+/-%.0fdeg enter_yaw<=%.0fdps\n",
        cross_enter_frames,
        cross_exit_frames,
        cross_min_dist_cm,
        cross_min_time_s,
        cross_timeout_s,
        cross_speed_scale,
        heading_hold_kp,
        heading_hold_max_rate_dps,
        cross_heading_grid_deg,
        cross_heading_tolerance_deg,
        cross_enter_max_yaw_rate_dps);
    std::printf(
        "  zebra=first-follow second-stop-after-clear enter=%d exit=%d frames\n",
        zebra_enter_frames,
        zebra_exit_frames);
    if (enable_side_road) {
        std::printf(
            "  side_entry=%d frames enter_distance=%.0fcm "
            "max_path_error=%.2f "
            "start>=%.0fcm imu_required=yes "
            "tof_required=%s "
            "visual_timeout=%.0f/%.0fs\n",
            round_enter_frames,
            round_enter_distance_cm,
            round_enter_max_path_error,
            side_start_distance_cm,
            side_require_tof ? "yes" : "no",
            round_timeout_s,
            round_timeout_s * 2.0);
    }
    std::printf("  wheel=%.1fcm base=%.1fcm encoder=%d ratio=%.5f "
                "filter=%.2f pwm=%.0fHz slew=%.0f%%/s\n",
                wheel_diameter_cm, wheel_base_cm, encoder_lines,
                encoder_gear_ratio, encoder_filter_alpha,
                pwm_frequency_hz, max_percent_delta_per_s);
    std::printf(
                "  imu=%s spi=%dHz sign=%+.0f accel=%s/%+.0f/%+.0f "
                "deadband=%.3fg zupt=%s/%.3fg/%.1fdps/%.2fs "
                "encoder_yaw=tau%.2fs gainx%.2f "
                "limit%.1fcm/s telemetry=%dHz flip=%s motor_swap=%s "
                "encoder_swap=%s\n",
                imu_transport.c_str(), imu_spi_speed_hz, imu_yaw_sign,
                imu_accel_swap_xy ? "Y/X" : "X/Y",
                imu_accel_forward_sign,
                imu_accel_right_sign,
                imu_accel_deadband_g,
                imu_stationary_zero ? "on" : "off",
                imu_stationary_accel_g,
                imu_stationary_yaw_dps,
                imu_stationary_hold_s,
                encoder_yaw_filter_tau_s,
                encoder_yaw_kp_scale,
                encoder_yaw_correction_limit_cmps,
                telemetry_hz,
                flip_motors ? "on" : "off",
                swap_motors ? "on" : "off",
                swap_encoders ? "on" : "off");
    if (!imu_csv_path.empty()) {
        std::printf("  imu_csv=%s rate=%dHz (direct, no encoders)\n",
                    imu_csv_path.c_str(), imu_csv_hz);
    }
    if (!inertial_path.empty()) {
        std::printf(
            "  inertial_path=%s lookahead=%.0fcm speed=%s min=%.1fcm/s "
            "heading_kp=%.2f max_error=%.0fcm\n",
            inertial_path.c_str(),
            inertial_lookahead_cm,
            inertial_speed_cmps > 0.0 ? "override" : "recorded",
            inertial_min_speed_cmps,
            inertial_heading_kp,
            inertial_max_deviation_cm);
    }
    std::printf(
        "  tof_slope=%s mode=%s sign=%+.0f distance<=%.0fmm hyst=%.0fmm "
        "confirm=%d/%d baseline_wait=%.1fs boost=%.1f%% "
        "speed=%.1f max=%.1fs\n",
        enable_tof_slope ? "on" : "off",
        tof_stop_test ? "STOP_TEST" : "BOOST",
        tof_ramp_sign,
        tof_ramp_delta_mm,
        tof_ramp_hysteresis_mm,
        tof_ramp_enter_frames,
        tof_ramp_exit_frames,
        tof_baseline_timeout_s,
        ramp_boost_percent,
        ramp_boost_speed_cmps,
        ramp_boost_max_s);
    std::printf("  calibration=%s control=%.0fcm far=%.0fcm%s%s\n",
                calibration_path.c_str(),
                control_distance_cm,
                far_distance_cm,
                forward_row >= 0 ? " forward-row-override" : "",
                far_row >= 0 ? " far-row-override" : "");
    std::printf(
        "  road_score=threshold%d color=%s sat_penalty=%d%% "
        "blue=%s H%d..%d S>=%d V>=%d penalty=%d\n",
        threshold_floor,
        vision_color_filter ? "on" : "off",
        vision_saturation_penalty,
        vision_blue_reject ? "on" : "off",
        vision_blue_hue_low,
        vision_blue_hue_high,
        vision_blue_saturation_min,
        vision_blue_value_min,
        vision_blue_penalty);
    std::printf("  horizon=%s wheel_mask=%s box(cx=%.2f w=%.2f top=%.2f bot=%.2f)\n",
                horizon_row >= 0 ? "row-override" :
                    (horizon_image_row >= 0.0 ? "image-row-override" : "calibration"),
                wheel_mask_enable ? "on" : "off",
                wheel_box_center_ratio,
                wheel_box_width_ratio,
                wheel_box_top_ratio,
                wheel_box_bottom_ratio);
    std::printf("  far_search=%s edge_smooth=%d bottom_fit_rows=%d cross_corner_tol=%d\n",
                far_search_enable ? "on" : "off",
                edge_smooth_window,
                bottom_fit_rows,
                cross_corner_row_tolerance);
    std::printf("  branch window=%d min_rows=%d return_rows=%d slope=%.2f dev=%.1f return=%.1f lock_slope=%.2f round_stable=%.1f\n",
                branch_window_rows,
                branch_min_rows,
                branch_return_rows,
                branch_slope_delta,
                branch_min_deviation,
                branch_return_tolerance,
                lock_slope_tolerance,
                round_stable_tolerance);
}

}  // namespace rewrite_path
