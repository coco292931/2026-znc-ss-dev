#pragma once

#include "path_params.hpp"
#include "path_types.hpp"
#include "track_topology.hpp"

#include <atomic>
#include <cstdint>
#include <mutex>
#include <vector>

#ifndef PATH_FOLLOW_NO_OPENCV
#include <opencv2/core.hpp>
#endif

namespace rewrite_path {

struct CalibrationPoint {
    double image_row = 0.0;
    double distance_cm = 0.0;
};

struct VisionTuningParams {
    int threshold_floor = 70;
    bool color_filter_enabled = true;
    int saturation_penalty = 60;
    bool blue_reject_enabled = true;
    int blue_hue_low = 85;
    int blue_hue_high = 135;
    int blue_saturation_min = 35;
    int blue_value_min = 55;
    int blue_penalty = 80;
};

struct WheelMaskBox {
    int left = 0;
    int right = 0;
    int top = 0;
    int bottom = 0;
    bool initialized = false;
    bool auto_detected = false;
};

class LegacyVisionPipeline {
public:
    explicit LegacyVisionPipeline(const PathParams& params);

    VisionTuningParams vision_tuning() const;
    void set_vision_tuning(const VisionTuningParams& tuning);
    WheelMaskBox wheel_mask_box() const;
    bool set_wheel_mask_box(double left_ratio,
                            double top_ratio,
                            double right_ratio,
                            double bottom_ratio);

    RoadEstimateLite process_gray(const std::uint8_t* gray,
                                  int width,
                                  int height,
                                  int step);
    const VisionFrame& debug_frame() const { return frame_; }

#ifndef PATH_FOLLOW_NO_OPENCV
    RoadEstimateLite process_bgr(const cv::Mat& bgr);
    cv::Mat draw_overlay(const cv::Mat& bgr,
                         const RoadEstimateLite& road,
                         const NavigationCommand& control) const;
    cv::Mat draw_longest_white_debug(
        const VisionFrame& vision_frame,
        const RoadEstimateLite& road,
        const NavigationCommand& control,
        const cv::Size& output_size) const;
#endif

private:
    RoadEstimateLite process_loaded_grid();
    void resize_to_binary_grid(const std::uint8_t* gray,
                               int width,
                               int height,
                               int step);
    std::uint8_t otsu_threshold(int threshold_floor) const;
    void binarize(std::uint8_t threshold);
    void initialize_wheel_box();
    bool detect_wheel_box(WheelMaskBox* box) const;
    WheelMaskBox configured_wheel_box() const;
    void apply_wheel_mask();
    void find_longest_white_column(RoadEstimateLite* out);
    void find_sidelines(RoadEstimateLite* out);
    void fit_missing_edges_from_bottom(RoadEstimateLite* out);
    void smooth_sidelines(RoadEstimateLite* out);
    void find_corners(RoadEstimateLite* out);
    void find_midline(RoadEstimateLite* out);
    void classify_geometry(RoadEstimateLite* out);
    double calc_error_at_row(const RoadEstimateLite& out, int row) const;
    void load_calibration();
    int row_for_distance(double distance_cm) const;
    double distance_for_binary_row(int row) const;
    void update_cm_scale(RoadEstimateLite* out);
    void update_topology(RoadEstimateLite* out);
    int horizon_row() const;
    int image_row_to_binary_row(double image_row) const;
    void wheel_box(int* left, int* right, int* top, int* bottom) const;
    bool row_in_wheel_box(int row) const;
    bool find_edge_from_anchor(int row, int start_col, int direction, int* edge_col) const;

    bool white(int row, int col) const;
    int safe_row(int row) const;
    int safe_col(int col) const;
    double line_slope(int row1, int col1, int row2, int col2) const;

    const PathParams& p_;
    std::atomic<int> threshold_floor_{70};
    std::atomic<bool> color_filter_enabled_{true};
    std::atomic<int> saturation_penalty_{60};
    std::atomic<bool> blue_reject_enabled_{true};
    std::atomic<int> blue_hue_low_{85};
    std::atomic<int> blue_hue_high_{135};
    std::atomic<int> blue_saturation_min_{35};
    std::atomic<int> blue_value_min_{55};
    std::atomic<int> blue_penalty_{80};
    mutable std::mutex wheel_box_mutex_;
    WheelMaskBox wheel_mask_box_;
    VisionFrame frame_;
    // BOOM 连通域拓扑观测层。只在 --topology 打开时被调用；
    // 内部网格放在堆上，避免每个 LegacyVisionPipeline 实例膨胀。
    TrackTopology topology_;
    int last_blue_mask_pixels_ = 0;
    double last_error_ = 0.0;
    double last_far_error_ = 0.0;
    double last_vehicle_center_error_ = 0.0;
    int last_mid_col_ = kBinaryWidth / 2;
    std::vector<CalibrationPoint> calibration_;
    double horizon_image_row_ = 34.0;
    bool has_filtered_left_fit_ = false;
    bool has_filtered_right_fit_ = false;
    double filtered_left_slope_ = 0.0;
    double filtered_left_intercept_ = 0.0;
    double filtered_right_slope_ = 0.0;
    double filtered_right_intercept_ = 0.0;
    bool prediction_cross_lock_ = false;
    bool has_locked_left_fit_ = false;
    bool has_locked_right_fit_ = false;
    double locked_left_slope_ = 0.0;
    double locked_left_intercept_ = 0.0;
    double locked_right_slope_ = 0.0;
    double locked_right_intercept_ = 0.0;

    RoundaboutVisionStage roundabout_stage_ = RoundaboutVisionStage::None;
    FeatureSide roundabout_side_ = FeatureSide::None;
    FeatureSide roundabout_stable_side_ = FeatureSide::None;
    int roundabout_stage_frames_ = 0;
    int roundabout_reacquire_frames_ = 0;
    int roundabout_inside_clear_frames_ = 0;
    bool roundabout_inside_clear_seen_ = false;
    int roundabout_exit_candidate_count_ = 0;
    int roundabout_exit_candidate_age_frames_ = 0;
    bool has_roundabout_locked_left_fit_ = false;
    bool has_roundabout_locked_right_fit_ = false;
    double roundabout_locked_left_slope_ = 0.0;
    double roundabout_locked_left_intercept_ = 0.0;
    double roundabout_locked_right_slope_ = 0.0;
    double roundabout_locked_right_intercept_ = 0.0;
    bool has_roundabout_override_left_fit_ = false;
    bool has_roundabout_override_right_fit_ = false;
    double roundabout_override_left_slope_ = 0.0;
    double roundabout_override_left_intercept_ = 0.0;
    double roundabout_override_right_slope_ = 0.0;
    double roundabout_override_right_intercept_ = 0.0;
    CornerPoint roundabout_entry_anchor_point_;
    CornerPoint roundabout_track_point_;
    CornerPoint stable_track_point_;
    CornerPoint roundabout_recovery_point_;
};

}  // namespace rewrite_path
