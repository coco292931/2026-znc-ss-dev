#include "status_display.hpp"

#include "path_params.hpp"

#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>

namespace rewrite_path {

namespace {

constexpr std::uint16_t kWhite = St7735s::rgb565(255, 255, 255);
constexpr std::uint16_t kGreen = St7735s::rgb565(0, 255, 96);
constexpr std::uint16_t kYellow = St7735s::rgb565(255, 220, 0);
constexpr std::uint16_t kRed = St7735s::rgb565(255, 64, 48);
constexpr std::uint16_t kCyan = St7735s::rgb565(0, 220, 255);
constexpr std::uint16_t kGray = St7735s::rgb565(170, 170, 170);
constexpr std::uint16_t kBg = St7735s::rgb565(0, 0, 0);

std::string format_line(const char* fmt, ...) {
    char buffer[48];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    return std::string(buffer);
}

std::uint16_t state_color(DriveState state) {
    switch (state) {
        case DriveState::Follow:         return kGreen;
        case DriveState::Stopped:        return kRed;
        case DriveState::CrossLock:
        case DriveState::ZebraPass:
        case DriveState::Roundabout:
        case DriveState::RoundaboutExit: return kYellow;
        case DriveState::InertialWait:   return kYellow;
        case DriveState::InertialAlign:
        case DriveState::InertialTrack:  return kCyan;
        default:                         return kCyan;
    }
}

}  // namespace

namespace {

St7735sConfig make_config(const PathParams& p) {
    St7735sConfig config;
    config.spi_device = p.display_spi_device;
    config.spi_speed_hz = static_cast<std::uint32_t>(p.display_spi_speed_hz);
    config.dc_gpio = p.display_dc_gpio;
    config.reset_gpio = p.display_reset_gpio;
    config.rotation = p.display_rotation;
    return config;
}

}  // namespace

StatusDisplay::StatusDisplay(const PathParams& params)
    : p_(params), display_(make_config(params)) {}

StatusDisplay::~StatusDisplay() {
    stop();
}

bool StatusDisplay::start() {
    if (running_.load()) return true;
    if (!display_.initialize()) {
        std::fprintf(stderr, "[DISPLAY] init failed: %s\n",
                     display_.last_error().c_str());
        return false;
    }
    if (!display_.fill(kBg)) {
        std::fprintf(stderr, "[DISPLAY] clear failed: %s\n",
                     display_.last_error().c_str());
        return false;
    }

    scale_ = p_.display_scale < 1 ? 1 : p_.display_scale;
    header_scale_ = scale_ == 1 ? 2 : scale_;
    header_height_ = St7735s::kGlyphHeight * header_scale_ + 2;
    line_height_ = St7735s::kGlyphHeight * scale_ + 1;
    // Line 0 is the large colored state header; lines 1.. are detail rows.
    line_count_ = 1 + (display_.height() - header_height_) / line_height_;
    if (line_count_ > kMaxLines) line_count_ = kMaxLines;

    running_.store(true);
    render_thread_ = std::thread([this] { render_loop(); });
    std::printf("[DISPLAY] ready spi=%s speed=%dHz refresh=%dHz\n",
                p_.display_spi_device.c_str(),
                p_.display_spi_speed_hz,
                p_.display_refresh_hz);
    return true;
}

void StatusDisplay::stop() {
    if (!running_.exchange(false)) return;
    cv_.notify_all();
    if (render_thread_.joinable()) render_thread_.join();
}

void StatusDisplay::publish(const TelemetrySample& sample) {
    Status status;
    status.elapsed_s = sample.elapsed_s;
    status.state = sample.navigation.state;
    status.sensor_mode = sample.control.diagnostics.sensor_mode;
    status.roundabout_stage = sample.road.elements.roundabout_stage;
    status.stop_reason = sample.navigation.stop_reason;
    status.recognition_valid = sample.recognition_valid;
    status.recognition_kind = sample.recognition_kind;
    status.recognition_confidence = sample.recognition_confidence;
    status.recognition_size = sample.recognition_size;
    status.recognition_center_y_ratio =
        sample.recognition_center_y_ratio;
    status.action_kind = sample.navigation.action_kind;
    status.target_encounter = sample.target_encounter;
    status.target_route_active = sample.target_route_active;
    status.line_error = sample.road.line_error;
    status.far_error = sample.road.far_error;
    status.line_confidence = sample.road.line_confidence;
    status.line_lost = sample.road.line_lost;
    status.target_speed_cmps = sample.navigation.target_speed_cmps;
    status.limited_speed_cmps = sample.control.diagnostics.limited_speed_cmps;
    status.requested_yaw_rate_dps = sample.navigation.target_yaw_rate_dps;
    status.measured_yaw_rate_dps =
        sample.control.diagnostics.measured_yaw_rate_dps;
    status.left_percent = sample.control.command.left_percent;
    status.right_percent = sample.control.command.right_percent;
    status.left_speed_cmps = sample.motor.left_speed_cmps;
    status.right_speed_cmps = sample.motor.right_speed_cmps;
    status.imu_heading_deg = sample.imu.heading_deg;
    status.imu_valid = sample.imu.valid;
    status.tof_distance_mm = sample.tof.filtered_distance_mm;
    status.ramp_detected = sample.tof.ramp_detected;
    status.power_boost_percent = sample.control.diagnostics.power_boost_percent;
    status.distance_cm = sample.motor.distance_cm;
    status.saturated = sample.control.diagnostics.saturated;
    status.inertial_state = sample.inertial_navigation.state;
    status.inertial_waypoint_index =
        sample.inertial_navigation.waypoint_index;
    status.inertial_waypoint_count =
        sample.inertial_navigation.waypoint_count;
    status.inertial_cross_track_error_cm =
        sample.inertial_navigation.cross_track_error_cm;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ = status;
        has_sample_ = true;
        ++sample_seq_;
    }
    cv_.notify_one();
}

void StatusDisplay::compose(
    const Status& s,
    std::array<std::string, kMaxLines>& lines,
    std::array<std::uint16_t, kMaxLines>& colors) const {
    for (int i = 0; i < kMaxLines; ++i) {
        lines[i].clear();
        colors[i] = kWhite;
    }

    int n = 0;
    auto add = [&](std::uint16_t color, const std::string& text) {
        if (n < line_count_) {
            lines[n] = text;
            colors[n] = color;
            ++n;
        }
    };

    // Line 0: large colored drive state (rendered at header scale).
    add(state_color(s.state), drive_state_name(s.state));

    add(kGray, format_line("MODE %s", sensor_mode_name(s.sensor_mode)));
    add(s.recognition_valid ? kYellow : kGray,
        s.recognition_valid
            ? format_line("REC  %s %.0f%%",
                          target_kind_name(s.recognition_kind),
                          100.0 * s.recognition_confidence)
            : std::string("REC  NONE"));
    if (s.recognition_valid) {
        add(kYellow, format_line("RED  s%.2f y%.2f",
                                 s.recognition_size,
                                 s.recognition_center_y_ratio));
    }
    add(s.target_route_active ? kCyan : kGray,
        s.action_kind != TargetKind::None
            ? format_line("ACT  #%d %s",
                          s.target_encounter,
                          target_kind_name(s.action_kind))
            : format_line("ACT  #%d NONE", s.target_encounter));
    if (s.inertial_state != InertialNavState::Disabled) {
        add(kCyan, format_line("NAV %zu/%zu E%.0f",
                              s.inertial_waypoint_index,
                              s.inertial_waypoint_count,
                              s.inertial_cross_track_error_cm));
    }
    if (s.roundabout_stage != RoundaboutVisionStage::None) {
        add(kYellow, format_line("RND  %s",
                                 roundabout_stage_name(s.roundabout_stage)));
    }
    add(s.line_lost ? kRed : kWhite,
        format_line("ERR n%+.2f f%+.2f", s.line_error, s.far_error));
    add(s.line_lost ? kRed : kWhite,
        format_line("CONF %.2f %s", s.line_confidence,
                    s.line_lost ? "LOST" : "OK"));
    add(kWhite, format_line("SPD  t%.0f l%.0f", s.target_speed_cmps,
                            s.limited_speed_cmps));
    add(kWhite, format_line("YAW  r%+.0f m%+.0f", s.requested_yaw_rate_dps,
                            s.measured_yaw_rate_dps));
    add(s.saturated ? kYellow : kWhite,
        format_line("PWM  L%+.0f R%+.0f", s.left_percent, s.right_percent));
    add(kWhite, format_line("WHL  L%.0f R%.0f", s.left_speed_cmps,
                            s.right_speed_cmps));
    add(s.imu_valid ? kWhite : kRed,
        format_line("IMU  %+.0f %s", s.imu_heading_deg,
                    s.imu_valid ? "OK" : "NA"));
    add(s.ramp_detected ? kYellow : kWhite,
        format_line("TOF  %.0f %s", s.tof_distance_mm,
                    s.ramp_detected ? "RAMP" : "flat"));
    add(kWhite, format_line("BST  %.0f%% D%.0f", s.power_boost_percent,
                            s.distance_cm));
    if (s.stop_reason != StopReason::None) {
        add(kRed, format_line("STOP %s", stop_reason_name(s.stop_reason)));
    }
    add(kGray, format_line("T    %.1fs", s.elapsed_s));
}

void StatusDisplay::render_loop() {
    const auto refresh_period = std::chrono::milliseconds(
        p_.display_refresh_hz > 0 ? 1000 / p_.display_refresh_hz : 250);
    std::array<std::string, kMaxLines> lines;
    std::array<std::uint16_t, kMaxLines> colors;
    std::uint64_t rendered_seq = 0;

    while (running_.load()) {
        Status status;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait_for(lock, refresh_period,
                         [this] { return !running_.load(); });
            if (!running_.load()) break;
            if (!has_sample_ || sample_seq_ == rendered_seq) continue;
            status = latest_;
            rendered_seq = sample_seq_;
        }

        compose(status, lines, colors);

        for (int i = 0; i < line_count_; ++i) {
            // Redraw only rows whose text or color changed; blank rows below
            // the current content are cleared once.
            if (lines[i] == drawn_text_[i] && colors[i] == drawn_color_[i] &&
                cleared_) {
                continue;
            }
            const int scale = i == 0 ? header_scale_ : scale_;
            const int y = i == 0 ? 0
                                 : header_height_ + (i - 1) * line_height_;
            const int cell_h = St7735s::kGlyphHeight * scale;
            // Erase the full row width first so shorter text leaves no trail.
            display_.fill_rect(0, y, display_.width(), cell_h, kBg);
            if (!lines[i].empty()) {
                display_.draw_text(0, y, lines[i], colors[i], kBg, scale);
            }
            drawn_text_[i] = lines[i];
            drawn_color_[i] = colors[i];
        }
        cleared_ = true;
    }
}

}  // namespace rewrite_path
