#pragma once

#include "path_types.hpp"
#include "st7735s.hpp"

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace rewrite_path {

struct PathParams;

// On-board TFT18 (ST7735S) status page for on-track debugging. It owns a
// background render thread so the 1 MHz SPI refresh never blocks the control
// loop; the loop only copies a compact scalar snapshot under a short lock.
// Enabling the display never changes navigation or motor behavior.
class StatusDisplay {
public:
    explicit StatusDisplay(const PathParams& params);
    ~StatusDisplay();

    StatusDisplay(const StatusDisplay&) = delete;
    StatusDisplay& operator=(const StatusDisplay&) = delete;

    bool start();
    void stop();
    bool running() const { return running_.load(); }

    // Copies the fields worth showing out of the per-frame telemetry. Cheap
    // and non-blocking; the render thread picks up the latest snapshot.
    void publish(const TelemetrySample& sample);

private:
    static constexpr int kMaxLines = 22;

    struct Status {
        double elapsed_s = 0.0;
        DriveState state = DriveState::Follow;
        SensorMode sensor_mode = SensorMode::VisionOpenLoop;
        RoundaboutVisionStage roundabout_stage = RoundaboutVisionStage::None;
        StopReason stop_reason = StopReason::None;
        bool recognition_valid = false;
        TargetKind recognition_kind = TargetKind::None;
        double recognition_confidence = 0.0;
        double recognition_size = 0.0;
        double recognition_center_y_ratio = 0.0;
        TargetKind action_kind = TargetKind::None;
        int target_encounter = 0;
        bool target_route_active = false;
        double line_error = 0.0;
        double far_error = 0.0;
        double line_confidence = 0.0;
        bool line_lost = true;
        double target_speed_cmps = 0.0;
        double limited_speed_cmps = 0.0;
        double requested_yaw_rate_dps = 0.0;
        double measured_yaw_rate_dps = 0.0;
        double left_percent = 0.0;
        double right_percent = 0.0;
        double left_speed_cmps = 0.0;
        double right_speed_cmps = 0.0;
        double imu_heading_deg = 0.0;
        bool imu_valid = false;
        double tof_distance_mm = 0.0;
        bool ramp_detected = false;
        double power_boost_percent = 0.0;
        double distance_cm = 0.0;
        bool saturated = false;
        InertialNavState inertial_state = InertialNavState::Disabled;
        std::size_t inertial_waypoint_index = 0;
        std::size_t inertial_waypoint_count = 0;
        double inertial_cross_track_error_cm = 0.0;
    };

    void render_loop();
    void compose(const Status& status,
                 std::array<std::string, kMaxLines>& lines,
                 std::array<std::uint16_t, kMaxLines>& colors) const;

    const PathParams& p_;
    St7735s display_;
    int scale_ = 1;
    int header_scale_ = 2;
    int header_height_ = 0;
    int line_height_ = 0;
    int line_count_ = 0;

    std::atomic<bool> running_{false};
    std::thread render_thread_;

    std::mutex mutex_;
    std::condition_variable cv_;
    Status latest_;
    bool has_sample_ = false;
    std::uint64_t sample_seq_ = 0;

    // Last text/color drawn per line, so the render thread only repaints rows
    // that actually changed and avoids flicker at low SPI clock.
    std::array<std::string, kMaxLines> drawn_text_;
    std::array<std::uint16_t, kMaxLines> drawn_color_{};
    bool cleared_ = false;
};

}  // namespace rewrite_path
