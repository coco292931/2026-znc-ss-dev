#include "vision_pipeline.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <numeric>
#include <queue>
#include <sstream>

#ifndef PATH_FOLLOW_NO_OPENCV
#include <opencv2/imgproc.hpp>
#endif

namespace rewrite_path {

namespace {

constexpr int kNominalTrackWidth = 46;
constexpr int kMaxRoundaboutSegments = 3;

struct BottomLineFit {
    bool valid = false;
    double slope = 0.0;      // x = slope * row + intercept
    double intercept = 0.0;
    int samples = 0;

    int predict(int row) const {
        return clamp_value(static_cast<int>(std::lround(slope * row + intercept)),
                           1, kBinaryWidth - 2);
    }
};

struct DeviationSegment {
    CornerPoint departure;
    CornerPoint peak;
    CornerPoint recovery;
    double peak_deviation = 0.0;
};

struct RoundaboutSideAnalysis {
    std::array<DeviationSegment, kMaxRoundaboutSegments> segments{};
    int segment_count = 0;
    int valid_rows = 0;
    double mean_abs_error = 0.0;
    double max_abs_error = 0.0;
    int near_valid_rows = 0;
    double near_mean_abs_error = 0.0;
    double near_max_abs_error = 0.0;
    bool unfinished_deviation = false;
    bool stable = false;
    bool near_stable = false;
};

BottomLineFit line_through(const CornerPoint& a, const CornerPoint& b) {
    BottomLineFit fit;
    if (!a.valid || !b.valid || std::abs(a.row - b.row) < 2) return fit;
    fit.slope = static_cast<double>(b.col - a.col) /
        static_cast<double>(b.row - a.row);
    if (!std::isfinite(fit.slope) || std::abs(fit.slope) > 4.0) return fit;
    fit.intercept = a.col - fit.slope * a.row;
    fit.samples = 2;
    fit.valid = true;
    return fit;
}

RoundaboutSideAnalysis analyze_roundabout_side(
    const std::array<int, kBinaryHeight>& side,
    const std::array<std::uint8_t, kBinaryHeight>& valid,
    const BottomLineFit& fit,
    int top,
    int bottom,
    bool left_side,
    const PathParams& p) {
    RoundaboutSideAnalysis out;
    if (!fit.valid) return out;

    bool deviation_confirmed = false;
    int deviation_rows = 0;
    int return_rows = 0;
    double abs_error_sum = 0.0;
    double near_abs_error_sum = 0.0;
    const int near_top = top + (bottom - top) / 2;
    DeviationSegment current;
    CornerPoint return_candidate;

    for (int y = bottom; y >= top; --y) {
        if (!valid[y]) continue;
        const int predicted = fit.predict(y);
        const double diff = static_cast<double>(side[y] - predicted);
        const double outward = left_side ? -diff : diff;
        const double abs_error = std::abs(diff);
        abs_error_sum += abs_error;
        out.max_abs_error = std::max(out.max_abs_error, abs_error);
        ++out.valid_rows;
        if (y >= near_top) {
            near_abs_error_sum += abs_error;
            out.near_max_abs_error =
                std::max(out.near_max_abs_error, abs_error);
            ++out.near_valid_rows;
        }

        if (!deviation_confirmed) {
            if (outward >= p.branch_min_deviation) {
                if (deviation_rows == 0) {
                    current = DeviationSegment{};
                    current.departure = CornerPoint{y, side[y], true};
                }
                ++deviation_rows;
                if (!current.peak.valid || outward > current.peak_deviation) {
                    current.peak = CornerPoint{y, side[y], true};
                    current.peak_deviation = outward;
                }
                if (deviation_rows >= p.branch_min_rows) {
                    deviation_confirmed = true;
                    return_rows = 0;
                }
            } else {
                deviation_rows = 0;
                current = DeviationSegment{};
            }
            continue;
        }

        if (!current.peak.valid || outward > current.peak_deviation) {
            current.peak = CornerPoint{y, side[y], true};
            current.peak_deviation = outward;
        }
        if (abs_error <= p.branch_return_tolerance) {
            if (return_rows == 0) {
                return_candidate = CornerPoint{y, side[y], true};
            }
            ++return_rows;
            if (return_rows >= p.branch_return_rows) {
                current.recovery = return_candidate;
                if (out.segment_count < kMaxRoundaboutSegments) {
                    out.segments[out.segment_count++] = current;
                }
                deviation_confirmed = false;
                deviation_rows = 0;
                return_rows = 0;
                current = DeviationSegment{};
            }
        } else {
            return_rows = 0;
        }
    }

    out.unfinished_deviation = deviation_confirmed;
    if (out.valid_rows > 0) {
        out.mean_abs_error = abs_error_sum / out.valid_rows;
    }
    if (out.near_valid_rows > 0) {
        out.near_mean_abs_error =
            near_abs_error_sum / out.near_valid_rows;
    }
    const int min_stable_rows = std::max(6, p.stable_two_side_min_rows);
    const double max_stable_error = std::max(
        p.branch_return_tolerance, p.round_stable_tolerance * 2.0);
    out.stable =
        out.valid_rows >= min_stable_rows &&
        out.segment_count == 0 &&
        !out.unfinished_deviation &&
        out.mean_abs_error <= p.round_stable_tolerance &&
        out.max_abs_error <= max_stable_error;
    const double recovery_tolerance = p.round_stable_tolerance * 1.75;
    out.near_stable =
        out.near_valid_rows >= std::max(4, min_stable_rows / 2) &&
        out.near_mean_abs_error <= recovery_tolerance &&
        out.near_max_abs_error <= std::max(
            p.branch_min_deviation * 2.0, recovery_tolerance * 2.0);
    return out;
}

BottomLineFit fit_bottom_line(const std::array<int, kBinaryHeight>& side,
                              const std::array<std::uint8_t, kBinaryHeight>& valid,
                              int top,
                              int bottom,
                              int fit_rows,
                              const std::array<std::uint8_t, kBinaryHeight>* reject = nullptr) {
    BottomLineFit fit;
    double sum_y = 0.0;
    double sum_x = 0.0;
    double sum_yy = 0.0;
    double sum_yx = 0.0;
    for (int y = bottom; y >= top && fit.samples < fit_rows; --y) {
        if (!valid[y]) continue;
        if (reject && (*reject)[y]) continue;
        const double dy = static_cast<double>(y);
        const double dx = static_cast<double>(side[y]);
        sum_y += dy;
        sum_x += dx;
        sum_yy += dy * dy;
        sum_yx += dy * dx;
        ++fit.samples;
    }
    if (fit.samples < 3) {
        return fit;
    }
    const double denom = fit.samples * sum_yy - sum_y * sum_y;
    if (std::abs(denom) < 1e-6) {
        return fit;
    }
    fit.slope = (fit.samples * sum_yx - sum_y * sum_x) / denom;
    fit.intercept = (sum_x - fit.slope * sum_y) / fit.samples;
    fit.valid = true;
    return fit;
}

BottomLineFit fit_quantile_prediction_line(
    const std::array<int, kBinaryHeight>& side,
    const std::array<std::uint8_t, kBinaryHeight>& valid,
    int top,
    int bottom,
    int fit_rows,
    bool left_side,
    const std::array<std::uint8_t, kBinaryHeight>* reject = nullptr) {
    std::vector<double> ys;
    std::vector<double> xs;
    ys.reserve(kBinaryHeight);
    xs.reserve(kBinaryHeight);

    for (int y = bottom; y >= top; --y) {
        if (!valid[y]) continue;
        if (reject && (*reject)[y]) continue;
        ys.push_back(static_cast<double>(y));
        xs.push_back(static_cast<double>(side[y]));
    }
    if (ys.size() < 3) {
        return fit_bottom_line(side, valid, top, bottom, fit_rows, reject);
    }

    auto solve_weighted = [&](const std::vector<double>* weights,
                              double* slope,
                              double* intercept) {
        double sum_w = 0.0;
        double sum_y = 0.0;
        double sum_x = 0.0;
        double sum_yy = 0.0;
        double sum_yx = 0.0;
        for (std::size_t i = 0; i < ys.size(); ++i) {
            const double w = weights ? (*weights)[i] : 1.0;
            sum_w += w;
            sum_y += w * ys[i];
            sum_x += w * xs[i];
            sum_yy += w * ys[i] * ys[i];
            sum_yx += w * ys[i] * xs[i];
        }
        const double denom = sum_w * sum_yy - sum_y * sum_y;
        if (sum_w <= 0.0 || std::abs(denom) < 1e-6) return false;
        *slope = (sum_w * sum_yx - sum_y * sum_x) / denom;
        *intercept = (sum_x - (*slope) * sum_y) / sum_w;
        return true;
    };

    BottomLineFit fit;
    fit.samples = static_cast<int>(ys.size());
    if (!solve_weighted(nullptr, &fit.slope, &fit.intercept)) {
        return fit_bottom_line(side, valid, top, bottom, fit_rows, reject);
    }

    // Left uses the upper 1% envelope; right is symmetric on the lower 1%.
    const double tau = left_side ? 0.99 : 0.01;
    std::vector<double> weights(ys.size(), 1.0);
    for (int iter = 0; iter < 18; ++iter) {
        for (std::size_t i = 0; i < ys.size(); ++i) {
            const double predicted = fit.slope * ys[i] + fit.intercept;
            const double residual = xs[i] - predicted;
            const double asym = residual >= 0.0 ? tau : (1.0 - tau);
            weights[i] = asym / std::max(std::abs(residual), 0.5);
        }
        double next_slope = fit.slope;
        double next_intercept = fit.intercept;
        if (!solve_weighted(&weights, &next_slope, &next_intercept)) break;
        const bool converged =
            std::abs(next_slope - fit.slope) < 1e-4 &&
            std::abs(next_intercept - fit.intercept) < 1e-3;
        fit.slope = next_slope;
        fit.intercept = next_intercept;
        if (converged) break;
    }
    fit.valid = true;
    return fit;
}

struct SchmittResult {
    std::array<std::uint8_t, kBinaryHeight> tags{};
    bool peak_valid = false;
    int peak_row = 0;
    double peak_mag = 0.0;
};

SchmittResult schmitt_anomaly_tags(const std::array<int, kBinaryHeight>& side,
                                   const std::array<std::uint8_t, kBinaryHeight>& valid,
                                   const BottomLineFit& fit,
                                   int top,
                                   int bottom,
                                   bool left_side,
                                   const PathParams& p) {
    SchmittResult result;
    if (!fit.valid) return result;
    bool abnormal = false;
    bool has_prev = false;
    double prev_signal = 0.0;
    double enter_delta = p.branch_slope_delta;
    int tagged_rows = 0;
    int max_tagged_rows = 0;
    const double trigger = std::max(0.05, p.branch_slope_delta);

    for (int y = bottom; y >= top; --y) {
        if (!valid[y]) continue;
        const double predicted = fit.slope * y + fit.intercept;
        const double diff = static_cast<double>(side[y]) - predicted;
        const double signal = left_side ? -diff : diff;
        if (!has_prev) {
            prev_signal = signal;
            has_prev = true;
            continue;
        }
        const double delta = signal - prev_signal;
        if (abnormal) {
            result.tags[y] = 1;
            ++tagged_rows;
            max_tagged_rows = std::max(max_tagged_rows, tagged_rows);
            if (signal > result.peak_mag) {
                result.peak_mag = signal;
                result.peak_row = y;
                result.peak_valid = true;
            }
            const bool falling_edge =
                delta <= -trigger &&
                std::abs(delta) >= std::max(trigger, std::abs(enter_delta) * 0.65);
            const bool back_to_baseline = signal <= p.branch_return_tolerance;
            if (falling_edge || back_to_baseline) {
                abnormal = false;
            }
        } else {
            tagged_rows = 0;
            const bool rising_edge =
                delta >= trigger && signal >= p.branch_min_deviation;
            if (rising_edge) {
                abnormal = true;
                enter_delta = delta;
                result.tags[y] = 1;
                tagged_rows = 1;
                max_tagged_rows = std::max(max_tagged_rows, tagged_rows);
                result.peak_row = y;
                result.peak_mag = signal;
                result.peak_valid = true;
            }
        }
        prev_signal = signal;
    }

    if (!result.peak_valid || max_tagged_rows < p.branch_min_rows) {
        result.peak_valid = false;
    }
    return result;
}

int count_branch_segments(const std::array<int, kBinaryHeight>& side,
                          const std::array<std::uint8_t, kBinaryHeight>& valid,
                          const BottomLineFit& fit,
                          int top,
                          int bottom,
                          bool left_side,
                          const PathParams& p) {
    const SchmittResult schmitt =
        schmitt_anomaly_tags(side, valid, fit, top, bottom, left_side, p);
    int count = 0;
    int run = 0;
    for (int y = bottom; y >= top; --y) {
        if (schmitt.tags[y]) {
            ++run;
        } else if (run > 0) {
            if (run >= p.branch_min_rows) ++count;
            run = 0;
        }
    }
    if (run >= p.branch_min_rows) ++count;
    return count;
}


}  // namespace

LegacyVisionPipeline::LegacyVisionPipeline(const PathParams& params) : p_(params) {
    VisionTuningParams tuning;
    tuning.threshold_floor = params.threshold_floor;
    tuning.color_filter_enabled = params.vision_color_filter;
    tuning.saturation_penalty = params.vision_saturation_penalty;
    tuning.blue_reject_enabled = params.vision_blue_reject;
    tuning.blue_hue_low = params.vision_blue_hue_low;
    tuning.blue_hue_high = params.vision_blue_hue_high;
    tuning.blue_saturation_min = params.vision_blue_saturation_min;
    tuning.blue_value_min = params.vision_blue_value_min;
    tuning.blue_penalty = params.vision_blue_penalty;
    set_vision_tuning(tuning);
    load_calibration();
}

VisionTuningParams LegacyVisionPipeline::vision_tuning() const {
    VisionTuningParams tuning;
    tuning.threshold_floor = threshold_floor_.load();
    tuning.color_filter_enabled = color_filter_enabled_.load();
    tuning.saturation_penalty = saturation_penalty_.load();
    tuning.blue_reject_enabled = blue_reject_enabled_.load();
    tuning.blue_hue_low = blue_hue_low_.load();
    tuning.blue_hue_high = blue_hue_high_.load();
    tuning.blue_saturation_min = blue_saturation_min_.load();
    tuning.blue_value_min = blue_value_min_.load();
    tuning.blue_penalty = blue_penalty_.load();
    return tuning;
}

void LegacyVisionPipeline::set_vision_tuning(
    const VisionTuningParams& requested) {
    int hue_low = clamp_value(requested.blue_hue_low, 0, 179);
    int hue_high = clamp_value(requested.blue_hue_high, 0, 179);
    if (hue_high < hue_low) std::swap(hue_low, hue_high);
    threshold_floor_.store(clamp_value(requested.threshold_floor, 0, 255));
    color_filter_enabled_.store(requested.color_filter_enabled);
    saturation_penalty_.store(
        clamp_value(requested.saturation_penalty, 0, 200));
    blue_reject_enabled_.store(requested.blue_reject_enabled);
    blue_hue_low_.store(hue_low);
    blue_hue_high_.store(hue_high);
    blue_saturation_min_.store(
        clamp_value(requested.blue_saturation_min, 0, 255));
    blue_value_min_.store(clamp_value(requested.blue_value_min, 0, 255));
    blue_penalty_.store(clamp_value(requested.blue_penalty, 0, 255));
}

void LegacyVisionPipeline::load_calibration() {
    calibration_ = {
        {190, 20}, {158, 30}, {132, 40}, {113, 50},
        {100, 60}, {92, 70},  {85, 80},  {80, 90},
        {66, 120}, {55, 180}, {50, 240}, {46, 300},
        {44, 360},
    };
    horizon_image_row_ = 34.0;

    if (p_.calibration_path.empty()) {
        return;
    }
    std::ifstream in(p_.calibration_path);
    if (!in) {
        return;
    }

    std::vector<CalibrationPoint> loaded;
    double loaded_horizon = horizon_image_row_;
    std::string line;
    while (std::getline(in, line)) {
        std::istringstream ss(line);
        double image_row = 0.0;
        double distance = 0.0;
        if (!(ss >> image_row)) {
            continue;
        }
        if (ss >> distance) {
            loaded.push_back(CalibrationPoint{image_row, distance});
        } else {
            loaded_horizon = image_row;
        }
    }
    if (loaded.size() >= 2) {
        calibration_ = loaded;
        horizon_image_row_ = loaded_horizon;
    }
}

int LegacyVisionPipeline::image_row_to_binary_row(double image_row) const {
    const int calib_h = std::max(1, p_.calibration_image_height);
    const double y = image_row * static_cast<double>(kBinaryHeight) /
                     static_cast<double>(calib_h);
    return safe_row(static_cast<int>(std::lround(y)));
}

int LegacyVisionPipeline::horizon_row() const {
    if (p_.horizon_row >= 0) {
        return safe_row(p_.horizon_row);
    }
    const double image_row =
        p_.horizon_image_row >= 0.0 ? p_.horizon_image_row : horizon_image_row_;
    return image_row_to_binary_row(image_row);
}

int LegacyVisionPipeline::row_for_distance(double distance_cm) const {
    if (calibration_.empty()) {
        return safe_row(p_.forward_row >= 0 ? p_.forward_row : kBinaryHeight / 2);
    }

    std::vector<CalibrationPoint> points = calibration_;
    std::sort(points.begin(), points.end(), [](const auto& a, const auto& b) {
        return a.distance_cm < b.distance_cm;
    });

    if (distance_cm <= points.front().distance_cm) {
        return image_row_to_binary_row(points.front().image_row);
    }
    if (distance_cm >= points.back().distance_cm) {
        return image_row_to_binary_row(points.back().image_row);
    }

    for (std::size_t i = 1; i < points.size(); ++i) {
        const CalibrationPoint& lo = points[i - 1];
        const CalibrationPoint& hi = points[i];
        if (distance_cm <= hi.distance_cm) {
            const double denom = std::max(1e-6, hi.distance_cm - lo.distance_cm);
            const double t = (distance_cm - lo.distance_cm) / denom;
            const double image_row = lo.image_row + (hi.image_row - lo.image_row) * t;
            return image_row_to_binary_row(image_row);
        }
    }
    return image_row_to_binary_row(points.back().image_row);
}

RoadEstimateLite LegacyVisionPipeline::process_gray(const std::uint8_t* gray,
                                                    int width,
                                                    int height,
                                                    int step) {
    resize_to_binary_grid(gray, width, height, step);
    last_blue_mask_pixels_ = 0;
    return process_loaded_grid();
}

RoadEstimateLite LegacyVisionPipeline::process_loaded_grid() {
    RoadEstimateLite out;
    const VisionTuningParams tuning = vision_tuning();
    std::uint8_t threshold = otsu_threshold(tuning.threshold_floor);
    threshold = static_cast<std::uint8_t>(std::max<int>(
        threshold, tuning.threshold_floor));
    binarize(threshold);
    initialize_wheel_box();
    apply_wheel_mask();
    find_longest_white_column(&out);
    find_sidelines(&out);
    find_corners(&out);
    fit_missing_edges_from_bottom(&out);
    find_midline(&out);
    classify_geometry(&out);
    out.vision_threshold = threshold;
    out.vision_blue_mask_pixels = last_blue_mask_pixels_;
    out.vision_color_filter_enabled = tuning.color_filter_enabled ? 1 : 0;
    return out;
}

#ifndef PATH_FOLLOW_NO_OPENCV
RoadEstimateLite LegacyVisionPipeline::process_bgr(const cv::Mat& bgr) {
    if (bgr.empty()) {
        return RoadEstimateLite{};
    }
    if (bgr.channels() == 1) {
        return process_gray(bgr.ptr<std::uint8_t>(0), bgr.cols, bgr.rows,
                            static_cast<int>(bgr.step));
    }

    cv::Mat source_bgr;
    if (bgr.channels() == 3) {
        source_bgr = bgr;
    } else if (bgr.channels() == 4) {
        cv::cvtColor(bgr, source_bgr, cv::COLOR_BGRA2BGR);
    } else {
        return RoadEstimateLite{};
    }

    cv::Mat small;
    cv::resize(source_bgr, small, cv::Size(kBinaryWidth, kBinaryHeight),
               0.0, 0.0, cv::INTER_AREA);
    cv::Mat gray;
    cv::cvtColor(small, gray, cv::COLOR_BGR2GRAY);

    const VisionTuningParams tuning = vision_tuning();
    last_blue_mask_pixels_ = 0;
    if (!tuning.color_filter_enabled) {
        for (int y = 0; y < kBinaryHeight; ++y) {
            const std::uint8_t* gray_row = gray.ptr<std::uint8_t>(y);
            std::copy(gray_row, gray_row + kBinaryWidth, frame_.gray[y].begin());
        }
        return process_loaded_grid();
    }

    cv::Mat hsv;
    cv::cvtColor(small, hsv, cv::COLOR_BGR2HSV);
    for (int y = 0; y < kBinaryHeight; ++y) {
        const std::uint8_t* gray_row = gray.ptr<std::uint8_t>(y);
        const cv::Vec3b* hsv_row = hsv.ptr<cv::Vec3b>(y);
        for (int x = 0; x < kBinaryWidth; ++x) {
            const int hue = hsv_row[x][0];
            const int saturation = hsv_row[x][1];
            const int value = hsv_row[x][2];
            int road_score = gray_row[x] -
                saturation * tuning.saturation_penalty / 100;
            const bool blue =
                tuning.blue_reject_enabled &&
                hue >= tuning.blue_hue_low && hue <= tuning.blue_hue_high &&
                saturation >= tuning.blue_saturation_min &&
                value >= tuning.blue_value_min;
            if (blue) {
                road_score -= tuning.blue_penalty;
                ++last_blue_mask_pixels_;
            }
            frame_.gray[y][x] = static_cast<std::uint8_t>(
                clamp_value(road_score, 0, 255));
        }
    }
    return process_loaded_grid();
}

cv::Mat LegacyVisionPipeline::draw_overlay(const cv::Mat& bgr,
                                            const RoadEstimateLite& road,
                                            const NavigationCommand& control) const {
    cv::Mat out;
    if (bgr.empty()) {
        return out;
    }
    if (bgr.channels() == 1) {
        cv::cvtColor(bgr, out, cv::COLOR_GRAY2BGR);
    } else {
        out = bgr.clone();
    }

    const double sx = static_cast<double>(out.cols) / kBinaryWidth;
    const double sy = static_cast<double>(out.rows) / kBinaryHeight;
    auto map_point = [&](int x, int y) {
        return cv::Point(static_cast<int>(std::lround(x * sx)),
                         static_cast<int>(std::lround(y * sy)));
    };

    for (int y = road.info.bottom; y >= road.info.top; --y) {
        if (road.left_valid[y]) {
            cv::circle(out, map_point(road.left[y], y), 2,
                       cv::Scalar(0, 255, 255), -1);
        }
        if (road.right_valid[y]) {
            cv::circle(out, map_point(road.right[y], y), 2,
                       cv::Scalar(255, 0, 255), -1);
        }
        cv::circle(out, map_point(road.mid[y], y), 2,
                   cv::Scalar(0, 255, 0), -1);
    }

    auto mark_corner = [&](const CornerPoint& c, const cv::Scalar& color) {
        if (!c.valid) return;
        cv::circle(out, map_point(c.col, c.row), 4, color, 1);
    };
    mark_corner(road.left_lower, cv::Scalar(255, 0, 0));
    mark_corner(road.left_upper, cv::Scalar(255, 0, 0));
    mark_corner(road.right_lower, cv::Scalar(0, 0, 255));
    mark_corner(road.right_upper, cv::Scalar(0, 0, 255));
    mark_corner(road.roundabout_track_point, cv::Scalar(0, 255, 255));
    mark_corner(road.stable_track_point, cv::Scalar(255, 255, 0));
    mark_corner(road.roundabout_recovery_point, cv::Scalar(255, 0, 255));

    if (p_.wheel_mask_enable) {
        int wl = 0, wr = 0, wt = 0, wb = 0;
        wheel_box(&wl, &wr, &wt, &wb);
        cv::rectangle(out, map_point(wl, wt), map_point(wr, wb),
                      cv::Scalar(255, 128, 0), 1);
    }

    char line[240];
    std::snprintf(line, sizeof(line),
                  "%s/%s err=%.2f wheel=%.2f conf=%.2f cross=%d "
                  "zebra=%d/%d%s stable=%d br=%d/%d round=%s",
                  drive_state_name(control.state),
                  roundabout_stage_name(road.elements.roundabout_stage),
                  road.line_error,
                  road.vehicle_center_error,
                  road.line_confidence,
                  road.elements.cross ? 1 : 0,
                  road.elements.zebra ? 1 : 0,
                  control.zebra_encounter_count,
                  control.zebra_active ? "*" : "",
                  road.elements.two_side_stable ? 1 : 0,
                  road.elements.left_branch_count,
                  road.elements.right_branch_count,
                  feature_side_name(road.elements.roundabout));
    cv::putText(out, line, cv::Point(8, 22), cv::FONT_HERSHEY_SIMPLEX,
                0.55, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    std::snprintf(line, sizeof(line),
                  "road-score thr=%d color=%d blue-mask=%d",
                  road.vision_threshold,
                  road.vision_color_filter_enabled,
                  road.vision_blue_mask_pixels);
    cv::putText(out, line, cv::Point(8, 44), cv::FONT_HERSHEY_SIMPLEX,
                0.50, cv::Scalar(0, 220, 255), 1, cv::LINE_AA);
    return out;
}

cv::Mat LegacyVisionPipeline::draw_longest_white_debug(
    const VisionFrame& vision_frame,
    const RoadEstimateLite& road,
    const NavigationCommand& control,
    const cv::Size& output_size) const {
    cv::Mat grid(kBinaryHeight, kBinaryWidth, CV_8UC3);
    for (int y = 0; y < kBinaryHeight; ++y) {
        for (int x = 0; x < kBinaryWidth; ++x) {
            const std::uint8_t binary = vision_frame.binary[y][x];
            const std::uint8_t gray = vision_frame.gray[y][x];
            const std::uint8_t value = binary != 0
                ? static_cast<std::uint8_t>(
                    205 + static_cast<int>(gray) * 50 / 255)
                : static_cast<std::uint8_t>(
                    8 + static_cast<int>(gray) * 28 / 255);
            grid.at<cv::Vec3b>(y, x) = cv::Vec3b(value, value, value);
        }
    }

    int wheel_left = 0;
    int wheel_right = 0;
    int wheel_top = 0;
    int wheel_bottom = 0;
    wheel_box(&wheel_left, &wheel_right, &wheel_top, &wheel_bottom);
    const int search_row = p_.wheel_mask_enable
        ? clamp_value(wheel_top - 1, road.info.top, road.info.bottom - 1)
        : road.info.bottom - 1;
    const int column_top = clamp_value(
        search_row - road.info.white_num + 1, 0, search_row);
    cv::line(grid,
             cv::Point(road.info.max_column, search_row),
             cv::Point(road.info.max_column, column_top),
             cv::Scalar(0, 128, 255), 1, cv::LINE_8);
    cv::line(grid, cv::Point(0, search_row),
             cv::Point(kBinaryWidth - 1, search_row),
             cv::Scalar(64, 64, 255), 1, cv::LINE_8);
    cv::line(grid, cv::Point(0, road.info.control_row),
             cv::Point(kBinaryWidth - 1, road.info.control_row),
             cv::Scalar(255, 128, 0), 1, cv::LINE_8);

    for (int y = road.info.bottom; y >= road.info.top; --y) {
        if (road.left_valid[y]) {
            grid.at<cv::Vec3b>(y, safe_col(road.left[y])) =
                cv::Vec3b(0, 255, 255);
        }
        if (road.right_valid[y]) {
            grid.at<cv::Vec3b>(y, safe_col(road.right[y])) =
                cv::Vec3b(255, 0, 255);
        }
        grid.at<cv::Vec3b>(y, safe_col(road.mid[y])) =
            cv::Vec3b(0, 255, 0);
    }

    auto mark_corner = [&](const CornerPoint& point,
                           const cv::Scalar& color) {
        if (!point.valid) return;
        cv::circle(grid, cv::Point(safe_col(point.col), safe_row(point.row)),
                   3, color, 1, cv::LINE_8);
    };
    mark_corner(road.left_lower, cv::Scalar(255, 64, 0));
    mark_corner(road.left_upper, cv::Scalar(255, 64, 0));
    mark_corner(road.right_lower, cv::Scalar(0, 64, 255));
    mark_corner(road.right_upper, cv::Scalar(0, 64, 255));
    mark_corner(road.roundabout_track_point, cv::Scalar(0, 255, 255));
    mark_corner(road.stable_track_point, cv::Scalar(255, 255, 0));
    mark_corner(road.roundabout_recovery_point, cv::Scalar(255, 0, 255));

    const cv::Size target_size(
        output_size.width > 0 ? output_size.width : kBinaryWidth * 4,
        output_size.height > 0 ? output_size.height : kBinaryHeight * 4);
    cv::Mat out;
    cv::resize(grid, out, target_size, 0.0, 0.0, cv::INTER_NEAREST);

    char line[256];
    std::snprintf(
        line, sizeof(line),
        "LONGEST-WHITE col=%d len=%d search=%d top=%d control=%d",
        road.info.max_column, road.info.white_num, search_row,
        road.info.top, road.info.control_row);
    cv::rectangle(out, cv::Rect(0, 0, out.cols, std::min(52, out.rows)),
                  cv::Scalar(0, 0, 0), -1);
    cv::putText(out, line, cv::Point(8, 20), cv::FONT_HERSHEY_SIMPLEX,
                0.48, cv::Scalar(0, 220, 255), 1, cv::LINE_AA);
    std::snprintf(
        line, sizeof(line),
        "%s/%s err=%.3f far=%.3f conf=%.2f L/R=%d/%d cross=%d "
        "zebra=%d/%d%s round=%s",
        drive_state_name(control.state),
        roundabout_stage_name(road.elements.roundabout_stage),
        road.line_error, road.far_error, road.line_confidence,
        road.elements.left_branch_count,
        road.elements.right_branch_count,
        road.elements.cross ? 1 : 0,
        road.elements.zebra ? 1 : 0,
        control.zebra_encounter_count,
        control.zebra_active ? "*" : "",
        feature_side_name(road.elements.roundabout));
    cv::putText(out, line, cv::Point(8, 42), cv::FONT_HERSHEY_SIMPLEX,
                0.44, cv::Scalar(128, 255, 128), 1, cv::LINE_AA);
    return out;
}
#endif

void LegacyVisionPipeline::resize_to_binary_grid(const std::uint8_t* gray,
                                                 int width,
                                                 int height,
                                                 int step) {
    if (!gray || width <= 0 || height <= 0 || step <= 0) {
        for (auto& row : frame_.gray) row.fill(0);
        return;
    }

    for (int y = 0; y < kBinaryHeight; ++y) {
        const int src_y0 = y * height / kBinaryHeight;
        const int src_y1 = std::max(src_y0 + 1, (y + 1) * height / kBinaryHeight);
        for (int x = 0; x < kBinaryWidth; ++x) {
            const int src_x0 = x * width / kBinaryWidth;
            const int src_x1 = std::max(src_x0 + 1, (x + 1) * width / kBinaryWidth);
            int sum = 0;
            int count = 0;
            for (int sy = src_y0; sy < src_y1 && sy < height; ++sy) {
                const std::uint8_t* src = gray + sy * step;
                for (int sx = src_x0; sx < src_x1 && sx < width; ++sx) {
                    sum += src[sx];
                    ++count;
                }
            }
            frame_.gray[y][x] =
                static_cast<std::uint8_t>(count > 0 ? sum / count : 0);
        }
    }
}

std::uint8_t LegacyVisionPipeline::otsu_threshold(int threshold_floor) const {
    std::array<int, 256> hist{};
    for (const auto& row : frame_.gray) {
        for (std::uint8_t v : row) {
            ++hist[v];
        }
    }

    const int total = kBinaryWidth * kBinaryHeight;
    double sum_all = 0.0;
    for (int i = 0; i < 256; ++i) {
        sum_all += i * hist[i];
    }

    double bg_weight = 0.0;
    double bg_sum = 0.0;
    double best_var = -1.0;
    int best_threshold = threshold_floor;
    const int upper = 180;  // Matches the original project; avoids highlights.
    for (int t = 0; t < upper; ++t) {
        bg_weight += hist[t];
        if (bg_weight <= 0.0) continue;
        const double fg_weight = total - bg_weight;
        if (fg_weight <= 0.0) break;

        bg_sum += t * hist[t];
        const double bg_mean = bg_sum / bg_weight;
        const double fg_mean = (sum_all - bg_sum) / fg_weight;
        const double delta = bg_mean - fg_mean;
        const double between = bg_weight * fg_weight * delta * delta;
        if (between > best_var) {
            best_var = between;
            best_threshold = t;
        }
    }
    return static_cast<std::uint8_t>(clamp_value(best_threshold, 0, 255));
}

void LegacyVisionPipeline::binarize(std::uint8_t threshold) {
    for (int y = 0; y < kBinaryHeight; ++y) {
        for (int x = 0; x < kBinaryWidth; ++x) {
            int local = threshold;
            if (x <= 18 || x >= 76) {
                local -= 10;
            }
            frame_.binary[y][x] = frame_.gray[y][x] > local ? 255 : 0;
        }
    }
}

WheelMaskBox LegacyVisionPipeline::configured_wheel_box() const {
    WheelMaskBox box;
    const int box_w = std::max(
        2, static_cast<int>(std::lround(p_.wheel_box_width_ratio * kBinaryWidth)));
    const int cx = static_cast<int>(
        std::lround(p_.wheel_box_center_ratio * (kBinaryWidth - 1)));
    box.left = clamp_value(cx - box_w / 2, 1, kBinaryWidth - 2);
    box.right = clamp_value(box.left + box_w - 1,
                            box.left + 1, kBinaryWidth - 2);
    box.top = clamp_value(static_cast<int>(std::lround(
                              p_.wheel_box_top_ratio * (kBinaryHeight - 1))),
                          0, kBinaryHeight - 1);
    box.bottom = clamp_value(static_cast<int>(std::lround(
                                 p_.wheel_box_bottom_ratio * (kBinaryHeight - 1))),
                             box.top, kBinaryHeight - 1);
    return box;
}

bool LegacyVisionPipeline::detect_wheel_box(WheelMaskBox* result) const {
    if (!result) return false;

    // The visible wheel is expected to be a compact black component that
    // reaches the bottom of the central part of the image. Restricting the
    // flood fill to this ROI prevents the two black areas outside the white
    // track from being selected as the wheel.
    constexpr int roi_left = kBinaryWidth * 18 / 100;
    constexpr int roi_right = kBinaryWidth * 82 / 100;
    constexpr int roi_top = kBinaryHeight * 55 / 100;
    std::array<std::array<std::uint8_t, kBinaryWidth>, kBinaryHeight> seen{};
    WheelMaskBox best;
    double best_score = -1.0;

    for (int sy = roi_top; sy < kBinaryHeight; ++sy) {
        for (int sx = roi_left; sx <= roi_right; ++sx) {
            if (seen[sy][sx] || frame_.binary[sy][sx] != 0) continue;
            std::queue<std::pair<int, int>> pending;
            pending.push({sx, sy});
            seen[sy][sx] = 1;
            int left = sx;
            int right = sx;
            int top = sy;
            int bottom = sy;
            int area = 0;
            while (!pending.empty()) {
                const int x = pending.front().first;
                const int y = pending.front().second;
                pending.pop();
                ++area;
                left = std::min(left, x);
                right = std::max(right, x);
                top = std::min(top, y);
                bottom = std::max(bottom, y);
                const int nx[4] = {x - 1, x + 1, x, x};
                const int ny[4] = {y, y, y - 1, y + 1};
                for (int i = 0; i < 4; ++i) {
                    if (nx[i] < roi_left || nx[i] > roi_right ||
                        ny[i] < roi_top || ny[i] >= kBinaryHeight ||
                        seen[ny[i]][nx[i]] ||
                        frame_.binary[ny[i]][nx[i]] != 0) {
                        continue;
                    }
                    seen[ny[i]][nx[i]] = 1;
                    pending.push({nx[i], ny[i]});
                }
            }

            const int width = right - left + 1;
            const int height = bottom - top + 1;
            const double center = (left + right) * 0.5;
            const double center_error = std::abs(
                center - (kBinaryWidth - 1) * 0.5);
            const double density = static_cast<double>(area) /
                std::max(1, width * height);
            if (bottom < kBinaryHeight - 2 || width < 4 ||
                width > kBinaryWidth * 45 / 100 || height < 4 ||
                area < 12 || density < 0.30 ||
                center < kBinaryWidth * 35 / 100 ||
                center > kBinaryWidth * 65 / 100) {
                continue;
            }
            const double score = area * density - center_error * 3.0;
            if (score <= best_score) continue;
            best_score = score;
            best.left = clamp_value(left - 1, 1, kBinaryWidth - 2);
            best.right = clamp_value(right + 1, best.left + 1,
                                     kBinaryWidth - 2);
            best.top = clamp_value(top - 1, 0, kBinaryHeight - 1);
            best.bottom = kBinaryHeight - 1;
            best.auto_detected = true;
        }
    }
    if (best_score < 0.0) return false;
    *result = best;
    return true;
}

void LegacyVisionPipeline::initialize_wheel_box() {
    if (!p_.wheel_mask_enable) return;
    std::lock_guard<std::mutex> lock(wheel_box_mutex_);
    if (wheel_mask_box_.initialized) return;
    WheelMaskBox detected;
    wheel_mask_box_ = detect_wheel_box(&detected)
        ? detected : configured_wheel_box();
    wheel_mask_box_.initialized = true;
    std::printf("[VISION] wheel box initialized: x=%d..%d y=%d..%d (%s)\n",
                wheel_mask_box_.left, wheel_mask_box_.right,
                wheel_mask_box_.top, wheel_mask_box_.bottom,
                wheel_mask_box_.auto_detected ? "auto" : "configured fallback");
}

WheelMaskBox LegacyVisionPipeline::wheel_mask_box() const {
    std::lock_guard<std::mutex> lock(wheel_box_mutex_);
    if (wheel_mask_box_.initialized) return wheel_mask_box_;
    return configured_wheel_box();
}

bool LegacyVisionPipeline::set_wheel_mask_box(double left_ratio,
                                               double top_ratio,
                                               double right_ratio,
                                               double bottom_ratio) {
    if (!std::isfinite(left_ratio) || !std::isfinite(top_ratio) ||
        !std::isfinite(right_ratio) || !std::isfinite(bottom_ratio) ||
        left_ratio < 0.0 || left_ratio > 1.0 ||
        top_ratio < 0.0 || top_ratio > 1.0 ||
        right_ratio < 0.0 || right_ratio > 1.0 ||
        bottom_ratio < 0.0 || bottom_ratio > 1.0) {
        return false;
    }
    left_ratio = clamp_value(left_ratio, 0.0, 1.0);
    right_ratio = clamp_value(right_ratio, 0.0, 1.0);
    top_ratio = clamp_value(top_ratio, 0.0, 1.0);
    bottom_ratio = clamp_value(bottom_ratio, 0.0, 1.0);
    if (right_ratio - left_ratio < 0.02 ||
        bottom_ratio - top_ratio < 0.02) {
        return false;
    }
    WheelMaskBox box;
    box.left = clamp_value(static_cast<int>(std::lround(
                               left_ratio * (kBinaryWidth - 1))),
                           1, kBinaryWidth - 3);
    box.right = clamp_value(static_cast<int>(std::lround(
                                right_ratio * (kBinaryWidth - 1))),
                            box.left + 1, kBinaryWidth - 2);
    box.top = clamp_value(static_cast<int>(std::lround(
                              top_ratio * (kBinaryHeight - 1))),
                          0, kBinaryHeight - 2);
    box.bottom = clamp_value(static_cast<int>(std::lround(
                                 bottom_ratio * (kBinaryHeight - 1))),
                             box.top + 1, kBinaryHeight - 1);
    box.initialized = true;
    box.auto_detected = false;
    {
        std::lock_guard<std::mutex> lock(wheel_box_mutex_);
        wheel_mask_box_ = box;
    }
    std::printf("[VISION] wheel box manually corrected: x=%d..%d y=%d..%d\n",
                box.left, box.right, box.top, box.bottom);
    return true;
}

void LegacyVisionPipeline::wheel_box(int* left,
                                     int* right,
                                     int* top,
                                     int* bottom) const {
    const WheelMaskBox box = wheel_mask_box();
    if (left) *left = box.left;
    if (right) *right = box.right;
    if (top) *top = box.top;
    if (bottom) *bottom = box.bottom;
}

bool LegacyVisionPipeline::row_in_wheel_box(int row) const {
    if (!p_.wheel_mask_enable) return false;
    int l = 0, r = 0, t = 0, b = 0;
    wheel_box(&l, &r, &t, &b);
    (void)l;
    (void)r;
    return row >= t && row <= b;
}

void LegacyVisionPipeline::apply_wheel_mask() {
    if (!p_.wheel_mask_enable) return;
    int l = 0, r = 0, t = 0, b = 0;
    wheel_box(&l, &r, &t, &b);
    for (int y = t; y <= b; ++y) {
        for (int x = l; x <= r; ++x) {
            frame_.binary[y][x] = 255;
        }
    }
}

bool LegacyVisionPipeline::find_edge_from_anchor(int row,
                                                 int start_col,
                                                 int direction,
                                                 int* edge_col) const {
    row = safe_row(row);
    direction = direction < 0 ? -1 : 1;
    int x = safe_col(start_col);

    while (x > 1 && x < kBinaryWidth - 2 && !white(row, x)) {
        x += direction;
    }
    if (x <= 1 || x >= kBinaryWidth - 2 || !white(row, x)) {
        return false;
    }

    int last_white = x;
    for (; x > 1 && x < kBinaryWidth - 2; x += direction) {
        if (white(row, x)) {
            last_white = x;
            continue;
        }
        const int next = safe_col(x + direction);
        if (!white(row, next)) {
            if (edge_col) *edge_col = last_white;
            return true;
        }
    }
    if (edge_col) *edge_col = last_white;
    return true;
}

void LegacyVisionPipeline::find_longest_white_column(RoadEstimateLite* out) {
    RoadImageInfo& info = out->info;
    info.bottom = kBinaryHeight - 1;
    info.top = p_.far_search_enable ? horizon_row() : 0;
    info.last_mid = last_mid_col_;
    info.white_num = 0;
    info.max_column = clamp_value(info.last_mid, 2, kBinaryWidth - 3);

    const int bottom = info.bottom - 1;
    int wl = 0, wr = 0, wt = 0, wb = 0;
    wheel_box(&wl, &wr, &wt, &wb);
    (void)wl;
    (void)wr;
    (void)wb;
    const int search_row = p_.wheel_mask_enable
        ? clamp_value(wt - 1, info.top, bottom)
        : bottom;

    int seed = clamp_value(info.last_mid, 2, kBinaryWidth - 3);
    if (!white(search_row, seed)) {
        int best_dist = kBinaryWidth;
        for (int x = 2; x < kBinaryWidth - 2; ++x) {
            if (!white(search_row, x)) continue;
            const int dist = std::abs(x - kBinaryWidth / 2);
            if (dist < best_dist) {
                best_dist = dist;
                seed = x;
            }
        }
    }

    int left = seed;
    int right = seed;
    for (int x = seed; x >= 2; --x) {
        if (white(search_row, x)) left = x;
        if (white(search_row, x) && !white(search_row, x - 1) &&
            !white(search_row, x - 2)) {
            break;
        }
    }
    for (int x = seed; x < kBinaryWidth - 2; ++x) {
        if (white(search_row, x)) right = x;
        if (white(search_row, x) && !white(search_row, x + 1) &&
            !white(search_row, x + 2)) {
            break;
        }
    }

    for (int x = left; x <= right; ++x) {
        int y = search_row;
        while (y >= 0 && white(y, x)) --y;
        const int len = search_row - y;
        if (len > info.white_num) {
            info.white_num = len;
            info.max_column = x;
        }
    }

    if (p_.far_search_enable) {
        info.top = horizon_row();
    } else {
        info.top = clamp_value(search_row - info.white_num + 1,
                               0, kBinaryHeight - 2);
        info.top = std::max(info.top, horizon_row());
    }
}

void LegacyVisionPipeline::find_sidelines(RoadEstimateLite* out) {
    RoadImageInfo& info = out->info;
    info.left_lost_count = 0;
    info.right_lost_count = 0;
    info.both_lost_count = 0;

    out->left.fill(1);
    out->right.fill(kBinaryWidth - 2);
    out->mid.fill(kBinaryWidth / 2);
    out->left_valid.fill(0);
    out->right_valid.fill(0);
    out->width.fill(0);

    int wl = 0, wr = 0, wt = 0, wb = 0;
    wheel_box(&wl, &wr, &wt, &wb);

    int last_left = 1;
    int last_right = kBinaryWidth - 2;
    bool have_last_pair = false;

    for (int y = info.bottom - 1; y >= info.top; --y) {
        const bool use_wheel_anchor =
            p_.wheel_mask_enable && y >= wt && y <= info.bottom - 1;
        const int center = use_wheel_anchor && have_last_pair
            ? clamp_value((last_left + last_right) / 2, 2, kBinaryWidth - 3)
            : clamp_value(info.max_column, 2, kBinaryWidth - 3);
        const int left_start = use_wheel_anchor
            ? clamp_value(wl - p_.wheel_box_margin_cols, 2, kBinaryWidth - 3)
            : center;
        const int right_start = use_wheel_anchor
            ? clamp_value(wr + p_.wheel_box_margin_cols, 2, kBinaryWidth - 3)
            : center;

        int lx = last_left;
        int rx = last_right;
        bool left_ok = find_edge_from_anchor(y, left_start, -1, &lx);
        bool right_ok = find_edge_from_anchor(y, right_start, 1, &rx);

        if ((!left_ok || !right_ok) && have_last_pair && !use_wheel_anchor) {
            if (!left_ok) left_ok = find_edge_from_anchor(y, center - 3, -1, &lx);
            if (!right_ok) right_ok = find_edge_from_anchor(y, center + 3, 1, &rx);
        }

        if (left_ok && right_ok) {
            const int w = rx - lx;
            if (w < 8 || w > kBinaryWidth - 4) {
                left_ok = false;
                right_ok = false;
            }
        }

        out->left[y] = left_ok ? lx : last_left;
        out->right[y] = right_ok ? rx : last_right;
        out->left_valid[y] = left_ok ? 1 : 0;
        out->right_valid[y] = right_ok ? 1 : 0;
        out->width[y] = (left_ok && right_ok) ? std::max(0, rx - lx) : 0;

        if (left_ok && right_ok) {
            last_left = lx;
            last_right = rx;
            have_last_pair = true;
        }
        if (!left_ok) ++info.left_lost_count;
        if (!right_ok) ++info.right_lost_count;
        if (!left_ok && !right_ok) ++info.both_lost_count;
    }
}

void LegacyVisionPipeline::fit_missing_edges_from_bottom(RoadEstimateLite* out) {
    const int top = out->info.top;
    const int bottom = out->info.bottom - 1;
    const int fit_rows = std::max(2, p_.bottom_fit_rows);

    const auto raw_left = out->left;
    const auto raw_right = out->right;
    const auto raw_left_valid = out->left_valid;
    const auto raw_right_valid = out->right_valid;
    const auto raw_width = out->width;

    int wheel_top = 0;
    wheel_box(nullptr, nullptr, &wheel_top, nullptr);
    const int vehicle_anchor_row = clamp_value(
        wheel_top - 2, top, bottom);
    out->bottom_pair_valid = true;
    for (int y = std::max(top, vehicle_anchor_row - 2);
         y <= std::min(bottom, vehicle_anchor_row + 2); ++y) {
        if (!raw_left_valid[y] || !raw_right_valid[y] ||
            raw_left[y] <= 2 || raw_right[y] >= kBinaryWidth - 3 ||
            raw_width[y] < 8) {
            out->bottom_pair_valid = false;
            break;
        }
    }

    auto smooth_prediction_fit = [&](const BottomLineFit& current,
                                     bool left_side) {
        bool& has = left_side ? has_filtered_left_fit_ : has_filtered_right_fit_;
        double& slope = left_side ? filtered_left_slope_ : filtered_right_slope_;
        double& intercept = left_side ? filtered_left_intercept_ : filtered_right_intercept_;
        if (!current.valid) {
            BottomLineFit previous;
            if (has) {
                previous.valid = true;
                previous.slope = slope;
                previous.intercept = intercept;
            }
            return previous;
        }
        BottomLineFit smoothed = current;
        if (has && p_.edge_smooth_window > 1) {
            const double alpha = clamp_value(2.0 / (p_.edge_smooth_window + 1.0),
                                             0.05, 1.0);
            smoothed.slope = slope * (1.0 - alpha) + current.slope * alpha;
            smoothed.intercept = intercept * (1.0 - alpha) +
                                 current.intercept * alpha;
        }
        has = true;
        slope = smoothed.slope;
        intercept = smoothed.intercept;
        return smoothed;
    };

    auto locked_fit = [](bool has, double slope, double intercept) {
        BottomLineFit fit;
        if (has) {
            fit.valid = true;
            fit.slope = slope;
            fit.intercept = intercept;
        }
        return fit;
    };

    BottomLineFit rough_left = prediction_cross_lock_ && has_locked_left_fit_
        ? locked_fit(has_locked_left_fit_, locked_left_slope_, locked_left_intercept_)
        : fit_quantile_prediction_line(raw_left, raw_left_valid, top, bottom,
                                       fit_rows, true);
    BottomLineFit rough_right = prediction_cross_lock_ && has_locked_right_fit_
        ? locked_fit(has_locked_right_fit_, locked_right_slope_, locked_right_intercept_)
        : fit_quantile_prediction_line(raw_right, raw_right_valid, top, bottom,
                                       fit_rows, false);

    SchmittResult left_seed_tags =
        schmitt_anomaly_tags(raw_left, raw_left_valid, rough_left,
                             top, bottom, true, p_);
    SchmittResult right_seed_tags =
        schmitt_anomaly_tags(raw_right, raw_right_valid, rough_right,
                             top, bottom, false, p_);

    BottomLineFit left_clean =
        fit_quantile_prediction_line(raw_left, raw_left_valid, top, bottom,
                                     fit_rows, true, &left_seed_tags.tags);
    if (!left_clean.valid) left_clean = rough_left;
    BottomLineFit right_clean =
        fit_quantile_prediction_line(raw_right, raw_right_valid, top, bottom,
                                     fit_rows, false, &right_seed_tags.tags);
    if (!right_clean.valid) right_clean = rough_right;

    const bool left_lock_slope_diverged =
        prediction_cross_lock_ && has_locked_left_fit_ && left_clean.valid &&
        std::abs(left_clean.slope - locked_left_slope_) > p_.lock_slope_tolerance;
    const bool right_lock_slope_diverged =
        prediction_cross_lock_ && has_locked_right_fit_ && right_clean.valid &&
        std::abs(right_clean.slope - locked_right_slope_) > p_.lock_slope_tolerance;
    const bool lock_slope_diverged =
        left_lock_slope_diverged || right_lock_slope_diverged;
    const bool lock_still_valid = prediction_cross_lock_ && !lock_slope_diverged;

    BottomLineFit left_fit = lock_still_valid && has_locked_left_fit_
        ? rough_left : smooth_prediction_fit(left_clean, true);
    BottomLineFit right_fit = lock_still_valid && has_locked_right_fit_
        ? rough_right : smooth_prediction_fit(right_clean, false);

    const auto fitted_path_error = [&](int row) {
        row = clamp_value(row, top, bottom);
        if (!left_fit.valid || !right_fit.valid) return 1.0;
        const double image_center = (kBinaryWidth - 1) * 0.5;
        const double path_center =
            (left_fit.predict(row) + right_fit.predict(row)) * 0.5;
        return std::abs(path_center - image_center) /
            std::max(1.0, image_center);
    };
    const int live_control_row = p_.forward_row >= 0
        ? clamp_value(p_.forward_row, top, bottom)
        : row_for_distance(p_.control_distance_cm);
    const int live_far_row = p_.far_row >= 0
        ? clamp_value(p_.far_row, top, bottom)
        : row_for_distance(p_.far_distance_cm);
    const double live_path_error = std::max(
        fitted_path_error(live_control_row),
        fitted_path_error(live_far_row));
    const bool live_path_centered_for_roundabout =
        live_path_error <= p_.round_enter_max_path_error;

    SchmittResult left_schmitt =
        schmitt_anomaly_tags(raw_left, raw_left_valid, left_fit,
                             top, bottom, true, p_);
    SchmittResult right_schmitt =
        schmitt_anomaly_tags(raw_right, raw_right_valid, right_fit,
                             top, bottom, false, p_);
    const int max_gap = std::max(p_.cross_corner_row_tolerance,
                                 p_.branch_window_rows);
    const bool symmetric_double_peak =
        left_schmitt.peak_valid && right_schmitt.peak_valid &&
        std::abs(left_schmitt.peak_row - right_schmitt.peak_row) <= max_gap;

    const auto average_raw_width = [&](int y0, int y1) {
        y0 = clamp_value(y0, top, bottom);
        y1 = clamp_value(y1, top, bottom);
        if (y0 > y1) std::swap(y0, y1);
        int sum = 0;
        int count = 0;
        for (int y = y0; y <= y1; ++y) {
            if (!raw_left_valid[y] || !raw_right_valid[y] ||
                raw_width[y] <= 0) {
                continue;
            }
            sum += raw_width[y];
            ++count;
        }
        return count > 0
            ? static_cast<double>(sum) / count : 0.0;
    };
    const double raw_near_width =
        average_raw_width(bottom - 8, bottom - 2);
    const double raw_mid_width =
        average_raw_width(
            (top + bottom) / 2 - 3,
            (top + bottom) / 2 + 3);
    const double raw_far_width =
        average_raw_width(top + 2, top + 9);
    const bool broad_two_side_opening =
        (raw_mid_width > raw_near_width * 1.30 &&
         raw_mid_width - raw_near_width > 8.0) ||
        (raw_far_width > raw_near_width * 1.25 &&
         raw_far_width - raw_near_width > 8.0);
    const auto paired_corner = [&](const CornerPoint& a,
                                   const CornerPoint& b) {
        return a.valid && b.valid &&
            std::abs(a.row - b.row) <=
                p_.cross_corner_row_tolerance;
    };
    const bool paired_cross_corners =
        paired_corner(out->left_lower, out->right_lower) ||
        paired_corner(out->left_upper, out->right_upper);
    const int preliminary_left_branches =
        count_branch_segments(raw_left, raw_left_valid, left_fit,
                              top, bottom, true, p_);
    const int preliminary_right_branches =
        count_branch_segments(raw_right, raw_right_valid, right_fit,
                              top, bottom, false, p_);
    const RoundaboutSideAnalysis current_left_analysis =
        analyze_roundabout_side(raw_left, raw_left_valid, left_fit,
                                top, bottom, true, p_);
    const RoundaboutSideAnalysis current_right_analysis =
        analyze_roundabout_side(raw_right, raw_right_valid, right_fit,
                                top, bottom, false, p_);
    const bool paired_two_side_branches =
        preliminary_left_branches > 0 &&
        preliminary_right_branches > 0;
    const bool one_sided_recovery =
        (current_left_analysis.segment_count > 0) !=
        (current_right_analysis.segment_count > 0);
    const bool symmetric_cross_opening =
        symmetric_double_peak &&
        (paired_cross_corners || paired_two_side_branches) &&
        broad_two_side_opening &&
        !one_sided_recovery;

    const bool roundabout_controls_path =
        roundabout_stage_ == RoundaboutVisionStage::Inside ||
        roundabout_stage_ == RoundaboutVisionStage::Exit;
    if (!roundabout_controls_path && symmetric_cross_opening &&
        !lock_slope_diverged) {
        if (!prediction_cross_lock_) {
            if (left_fit.valid) {
                has_locked_left_fit_ = true;
                locked_left_slope_ = left_fit.slope;
                locked_left_intercept_ = left_fit.intercept;
            }
            if (right_fit.valid) {
                has_locked_right_fit_ = true;
                locked_right_slope_ = right_fit.slope;
                locked_right_intercept_ = right_fit.intercept;
            }
        }
        prediction_cross_lock_ = true;
    } else {
        prediction_cross_lock_ = false;
        has_locked_left_fit_ = false;
        has_locked_right_fit_ = false;
    }

    BottomLineFit final_left = prediction_cross_lock_ && has_locked_left_fit_
        ? locked_fit(has_locked_left_fit_, locked_left_slope_, locked_left_intercept_)
        : left_fit;
    BottomLineFit final_right = prediction_cross_lock_ && has_locked_right_fit_
        ? locked_fit(has_locked_right_fit_, locked_right_slope_, locked_right_intercept_)
        : right_fit;

    auto clear_roundabout_tracking = [&] {
        roundabout_stage_ = RoundaboutVisionStage::None;
        roundabout_side_ = FeatureSide::None;
        roundabout_stable_side_ = FeatureSide::None;
        roundabout_stage_frames_ = 0;
        roundabout_reacquire_frames_ = 0;
        roundabout_inside_clear_frames_ = 0;
        roundabout_inside_clear_seen_ = false;
        roundabout_exit_candidate_count_ = 0;
        roundabout_exit_candidate_age_frames_ = 0;
        has_roundabout_locked_left_fit_ = false;
        has_roundabout_locked_right_fit_ = false;
        has_roundabout_override_left_fit_ = false;
        has_roundabout_override_right_fit_ = false;
        roundabout_entry_anchor_point_ = CornerPoint{};
        roundabout_track_point_ = CornerPoint{};
        stable_track_point_ = CornerPoint{};
        roundabout_recovery_point_ = CornerPoint{};
    };
    auto save_locked_fits = [&](const BottomLineFit& left,
                                const BottomLineFit& right) {
        has_roundabout_locked_left_fit_ = left.valid;
        roundabout_locked_left_slope_ = left.slope;
        roundabout_locked_left_intercept_ = left.intercept;
        has_roundabout_locked_right_fit_ = right.valid;
        roundabout_locked_right_slope_ = right.slope;
        roundabout_locked_right_intercept_ = right.intercept;
    };
    auto round_locked_left = [&] {
        return locked_fit(has_roundabout_locked_left_fit_,
                          roundabout_locked_left_slope_,
                          roundabout_locked_left_intercept_);
    };
    auto round_locked_right = [&] {
        return locked_fit(has_roundabout_locked_right_fit_,
                          roundabout_locked_right_slope_,
                          roundabout_locked_right_intercept_);
    };
    auto lower_recovery = [](const RoundaboutSideAnalysis& analysis) {
        CornerPoint point;
        for (int i = 0; i < analysis.segment_count; ++i) {
            const CornerPoint& candidate = analysis.segments[i].recovery;
            if (candidate.valid && (!point.valid || candidate.row > point.row)) {
                point = candidate;
            }
        }
        return point;
    };
    auto recovery_above = [](const RoundaboutSideAnalysis& analysis,
                             int row,
                             int gap) {
        CornerPoint point;
        for (int i = 0; i < analysis.segment_count; ++i) {
            const CornerPoint& candidate = analysis.segments[i].recovery;
            if (!candidate.valid || candidate.row > row - gap) continue;
            if (!point.valid || candidate.row > point.row) point = candidate;
        }
        return point;
    };
    auto set_roundabout_override = [&](FeatureSide side,
                                       const BottomLineFit& fit) {
        if (!fit.valid) return;
        if (side == FeatureSide::Left) {
            has_roundabout_override_left_fit_ = true;
            roundabout_override_left_slope_ = fit.slope;
            roundabout_override_left_intercept_ = fit.intercept;
        } else if (side == FeatureSide::Right) {
            has_roundabout_override_right_fit_ = true;
            roundabout_override_right_slope_ = fit.slope;
            roundabout_override_right_intercept_ = fit.intercept;
        }
    };

    const auto side_from_branches = [](int left_count, int right_count) {
        if (left_count > 0 && right_count == 0) {
            return FeatureSide::Left;
        }
        if (right_count > 0 && left_count == 0) {
            return FeatureSide::Right;
        }
        return FeatureSide::None;
    };
    const FeatureSide legacy_side_open = side_from_branches(
        preliminary_left_branches, preliminary_right_branches);
    const auto has_roundabout_deviation =
        [](const RoundaboutSideAnalysis& analysis) {
            return analysis.segment_count > 0 ||
                analysis.unfinished_deviation;
        };
    const auto dominates_stable_side =
        [&](const RoundaboutSideAnalysis& ring,
            const RoundaboutSideAnalysis& stable) {
            const double minimum_ring_error = std::max(
                p_.branch_min_deviation * 2.0,
                stable.near_max_abs_error + p_.branch_min_deviation);
            return !ring.near_stable &&
                ring.near_max_abs_error >= minimum_ring_error;
        };
    const bool right_roundabout =
        current_left_analysis.near_stable &&
        has_roundabout_deviation(current_right_analysis) &&
        (current_right_analysis.segment_count >
             current_left_analysis.segment_count ||
         dominates_stable_side(
             current_right_analysis, current_left_analysis));
    const bool left_roundabout =
        current_right_analysis.near_stable &&
        has_roundabout_deviation(current_left_analysis) &&
        (current_left_analysis.segment_count >
             current_right_analysis.segment_count ||
         dominates_stable_side(
             current_left_analysis, current_right_analysis));
    const FeatureSide dominant_branch_side =
        preliminary_right_branches >= 2 &&
        preliminary_right_branches > preliminary_left_branches
            ? FeatureSide::Right
            : (preliminary_left_branches >= 2 &&
               preliminary_left_branches > preliminary_right_branches
                ? FeatureSide::Left
                : FeatureSide::None);

    if (!p_.enable_side_road &&
        roundabout_stage_ != RoundaboutVisionStage::None) {
        clear_roundabout_tracking();
    }

    if (p_.enable_side_road &&
        roundabout_stage_ == RoundaboutVisionStage::None &&
        !prediction_cross_lock_ && left_fit.valid && right_fit.valid &&
        live_path_centered_for_roundabout) {
        const FeatureSide new_candidate =
            right_roundabout != left_roundabout
                ? (right_roundabout
                    ? FeatureSide::Right
                    : FeatureSide::Left)
                : FeatureSide::None;
        const bool candidate_has_branch_support =
            legacy_side_open == new_candidate ||
            dominant_branch_side == new_candidate;
        if (new_candidate != FeatureSide::None &&
            candidate_has_branch_support) {
            roundabout_side_ = right_roundabout
                ? FeatureSide::Right : FeatureSide::Left;
            roundabout_stable_side_ = right_roundabout
                ? FeatureSide::Left : FeatureSide::Right;
            save_locked_fits(left_fit, right_fit);
            const RoundaboutSideAnalysis& ring_analysis = right_roundabout
                ? current_right_analysis : current_left_analysis;
            const BottomLineFit ring_fit = right_roundabout
                ? round_locked_right() : round_locked_left();
            const BottomLineFit stable_fit = right_roundabout
                ? round_locked_left() : round_locked_right();
            roundabout_track_point_ = lower_recovery(ring_analysis);
            if (roundabout_track_point_.valid) {
                roundabout_track_point_.col =
                    ring_fit.predict(roundabout_track_point_.row);
                roundabout_entry_anchor_point_ =
                    roundabout_track_point_;
                stable_track_point_ = CornerPoint{
                    roundabout_track_point_.row,
                    stable_fit.predict(roundabout_track_point_.row),
                    true};
                roundabout_stage_ = RoundaboutVisionStage::Approach;
                roundabout_stage_frames_ = 0;
                roundabout_exit_candidate_count_ =
                    legacy_side_open == new_candidate ? 1 : 0;
            } else {
                clear_roundabout_tracking();
            }
        }
    } else if (p_.enable_side_road &&
               roundabout_stage_ == RoundaboutVisionStage::Approach) {
        const BottomLineFit locked_left = round_locked_left();
        const BottomLineFit locked_right = round_locked_right();
        const BottomLineFit ring_fit = roundabout_side_ == FeatureSide::Right
            ? locked_right : locked_left;
        const BottomLineFit stable_fit = roundabout_stable_side_ == FeatureSide::Left
            ? locked_left : locked_right;
        const RoundaboutSideAnalysis& ring_analysis =
            roundabout_side_ == FeatureSide::Right
                ? current_right_analysis : current_left_analysis;

        const bool new_evidence_matches =
            roundabout_side_ == FeatureSide::Right
                ? right_roundabout && !left_roundabout
                : left_roundabout && !right_roundabout;
        const bool conflicting_evidence =
            roundabout_side_ == FeatureSide::Right
                ? left_roundabout : right_roundabout;
        const bool legacy_evidence_matches =
            legacy_side_open == roundabout_side_ ||
            dominant_branch_side == roundabout_side_;
        const bool one_sided_evidence_matches =
            legacy_side_open == roundabout_side_;
        const bool legacy_evidence_conflicts =
            (legacy_side_open != FeatureSide::None ||
             dominant_branch_side != FeatureSide::None) &&
            !legacy_evidence_matches;
        if (!live_path_centered_for_roundabout ||
            prediction_cross_lock_ || conflicting_evidence ||
            (legacy_evidence_conflicts && !new_evidence_matches)) {
            clear_roundabout_tracking();
        } else if (new_evidence_matches &&
                   one_sided_evidence_matches) {
            ++roundabout_exit_candidate_count_;
            const CornerPoint moving_recovery = lower_recovery(ring_analysis);
            if (moving_recovery.valid &&
                moving_recovery.row >= roundabout_track_point_.row) {
                roundabout_track_point_.row = moving_recovery.row;
                roundabout_track_point_.col =
                    ring_fit.predict(moving_recovery.row);
                stable_track_point_ = CornerPoint{
                    moving_recovery.row,
                    stable_fit.predict(moving_recovery.row),
                    true};
            }

            const int point_gap = std::max(3, p_.branch_window_rows);
            const CornerPoint current_lower_recovery =
                lower_recovery(ring_analysis);
            const CornerPoint entry_track_point =
                ring_analysis.segment_count >= 2
                    ? current_lower_recovery
                    : roundabout_track_point_;
            const CornerPoint next_recovery = recovery_above(
                ring_analysis, entry_track_point.row, point_gap);
            const bool distinct_second_segment =
                ring_analysis.segment_count >= 2;
            const int tracked_progress_rows =
                current_lower_recovery.valid &&
                roundabout_entry_anchor_point_.valid
                    ? current_lower_recovery.row -
                        roundabout_entry_anchor_point_.row
                    : 0;
            const int minimum_tracked_frames = std::max(
                3, p_.round_enter_frames);
            const int round_enter_row =
                row_for_distance(p_.round_enter_distance_cm);
            const bool reached_round_enter_distance =
                current_lower_recovery.valid &&
                current_lower_recovery.row >= round_enter_row;
            const bool persistent_moving_recovery =
                ring_analysis.segment_count >= 1 &&
                roundabout_exit_candidate_count_ >=
                    minimum_tracked_frames &&
                tracked_progress_rows >= 2 &&
                reached_round_enter_distance;
            const bool confirmed =
                roundabout_exit_candidate_count_ >=
                    std::max(2, p_.round_enter_frames);
            const CornerPoint entry_anchor =
                distinct_second_segment
                    ? next_recovery
                    : roundabout_entry_anchor_point_;
            if (confirmed &&
                (distinct_second_segment ||
                 persistent_moving_recovery) &&
                entry_track_point.valid && entry_anchor.valid) {
                const BottomLineFit entry_fit =
                    line_through(entry_track_point, entry_anchor);
                if (entry_fit.valid) {
                    roundabout_track_point_ = entry_track_point;
                    stable_track_point_ = CornerPoint{
                        entry_track_point.row,
                        stable_fit.predict(entry_track_point.row),
                        true};
                    has_roundabout_override_left_fit_ = false;
                    has_roundabout_override_right_fit_ = false;
                    set_roundabout_override(roundabout_side_, entry_fit);
                    roundabout_recovery_point_ = entry_anchor;
                    has_roundabout_locked_left_fit_ = false;
                    has_roundabout_locked_right_fit_ = false;
                    roundabout_stage_ = RoundaboutVisionStage::Inside;
                    roundabout_stage_frames_ = 0;
                    roundabout_inside_clear_frames_ = 0;
                    roundabout_inside_clear_seen_ = false;
                    roundabout_exit_candidate_count_ = 0;
                    roundabout_exit_candidate_age_frames_ = 0;
                }
            }
        } else {
            roundabout_exit_candidate_count_ = 0;
        }

    } else if (roundabout_stage_ == RoundaboutVisionStage::Inside) {
        const RoundaboutSideAnalysis& ring_analysis =
            roundabout_side_ == FeatureSide::Right
                ? current_right_analysis : current_left_analysis;
        if (ring_analysis.segment_count == 0) {
            ++roundabout_inside_clear_frames_;
            if (roundabout_inside_clear_frames_ >=
                std::max(2, p_.round_exit_frames)) {
                roundabout_inside_clear_seen_ = true;
            }
        } else if (!roundabout_inside_clear_seen_) {
            roundabout_inside_clear_frames_ = 0;
        }
        const bool both_open =
            roundabout_inside_clear_seen_ &&
            current_left_analysis.segment_count > 0 &&
            current_right_analysis.segment_count > 0;
        const int exit_evidence_hold_frames = std::max(
            p_.round_exit_frames * 2,
            static_cast<int>(std::lround(std::max(1, p_.camera_fps) * 3.0)));
        if (both_open) {
            if (roundabout_exit_candidate_age_frames_ >
                exit_evidence_hold_frames) {
                roundabout_exit_candidate_count_ = 0;
            }
            ++roundabout_exit_candidate_count_;
            roundabout_exit_candidate_age_frames_ = 0;
        } else if (roundabout_exit_candidate_count_ > 0) {
            ++roundabout_exit_candidate_age_frames_;
            if (roundabout_exit_candidate_age_frames_ >
                exit_evidence_hold_frames) {
                roundabout_exit_candidate_count_ = 0;
                roundabout_exit_candidate_age_frames_ = 0;
            }
        }
        if (both_open && roundabout_exit_candidate_count_ >= 2) {
            const FeatureSide opposite_side = roundabout_stable_side_;
            const RoundaboutSideAnalysis& opposite_analysis =
                opposite_side == FeatureSide::Left
                    ? current_left_analysis : current_right_analysis;
            const BottomLineFit ring_fit = roundabout_side_ == FeatureSide::Left
                ? left_fit : right_fit;
            const CornerPoint opposite_corner = lower_recovery(opposite_analysis);
            if (ring_fit.valid && opposite_corner.valid) {
                const int opposite_sign =
                    opposite_side == FeatureSide::Right ? 1 : -1;
                const CornerPoint extension{
                    bottom,
                    clamp_value(ring_fit.predict(bottom) +
                                    opposite_sign * kNominalTrackWidth,
                                1, kBinaryWidth - 2),
                    true};
                const BottomLineFit exit_fit =
                    line_through(opposite_corner, extension);
                if (exit_fit.valid) {
                    has_roundabout_override_left_fit_ = false;
                    has_roundabout_override_right_fit_ = false;
                    set_roundabout_override(opposite_side, exit_fit);
                    roundabout_recovery_point_ = opposite_corner;
                    roundabout_stage_ = RoundaboutVisionStage::Exit;
                    roundabout_stage_frames_ = 0;
                    roundabout_reacquire_frames_ = 0;
                }
            }
        }
    } else if (roundabout_stage_ == RoundaboutVisionStage::Exit &&
               symmetric_cross_opening) {
        clear_roundabout_tracking();
    } else if (roundabout_stage_ == RoundaboutVisionStage::Exit) {
        const int far_sample = top + (bottom - top) / 3;
        const int near_width = right_fit.predict(bottom) - left_fit.predict(bottom);
        const int far_width = right_fit.predict(far_sample) - left_fit.predict(far_sample);
        const bool prediction_symmetric =
            left_fit.valid && right_fit.valid &&
            std::abs(left_fit.slope + right_fit.slope) <=
                p_.lock_slope_tolerance &&
            near_width >= 16 && near_width <= kBinaryWidth - 8 &&
            far_width >= 10 && far_width <= kBinaryWidth - 8;
        const bool recovered = prediction_symmetric;
        out->elements.roundabout_prediction_symmetric = prediction_symmetric;
        if (recovered) {
            ++roundabout_reacquire_frames_;
            if (roundabout_reacquire_frames_ >=
                std::max(2, p_.round_exit_frames)) {
                has_roundabout_override_left_fit_ = false;
                has_roundabout_override_right_fit_ = false;
                roundabout_stage_ = RoundaboutVisionStage::Reacquired;
                roundabout_stage_frames_ = 0;
            }
        } else {
            roundabout_reacquire_frames_ = 0;
        }
    } else if (roundabout_stage_ == RoundaboutVisionStage::Reacquired &&
               roundabout_stage_frames_ >= std::max(3, p_.round_exit_frames + 1)) {
        clear_roundabout_tracking();
    }

    if (roundabout_stage_ != RoundaboutVisionStage::None) {
        ++roundabout_stage_frames_;
    }
    if (roundabout_stage_ == RoundaboutVisionStage::Inside ||
        roundabout_stage_ == RoundaboutVisionStage::Exit) {
        prediction_cross_lock_ = false;
        has_locked_left_fit_ = false;
        has_locked_right_fit_ = false;
    }
    const double roundabout_timeout_scale =
        roundabout_stage_ == RoundaboutVisionStage::Exit ? 2.0 : 1.0;
    const int roundabout_timeout_frames = std::max(
        30, static_cast<int>(std::lround(
                std::max(1, p_.camera_fps) * p_.round_timeout_s *
                roundabout_timeout_scale)));
    if (roundabout_stage_ != RoundaboutVisionStage::None &&
        roundabout_stage_ != RoundaboutVisionStage::Reacquired &&
        roundabout_stage_frames_ > roundabout_timeout_frames) {
        clear_roundabout_tracking();
    }

    if (roundabout_stage_ == RoundaboutVisionStage::Inside ||
        roundabout_stage_ == RoundaboutVisionStage::Exit) {
        if (has_roundabout_override_left_fit_) {
            final_left = locked_fit(true, roundabout_override_left_slope_,
                                    roundabout_override_left_intercept_);
        }
        if (has_roundabout_override_right_fit_) {
            final_right = locked_fit(true, roundabout_override_right_slope_,
                                     roundabout_override_right_intercept_);
        }
    }

    out->elements.left_branch_count =
        count_branch_segments(raw_left, raw_left_valid, final_left,
                              top, bottom, true, p_);
    out->elements.right_branch_count =
        count_branch_segments(raw_right, raw_right_valid, final_right,
                              top, bottom, false, p_);
    out->elements.left_recovery_count = current_left_analysis.segment_count;
    out->elements.right_recovery_count = current_right_analysis.segment_count;
    out->elements.left_prediction_stable = current_left_analysis.near_stable;
    out->elements.right_prediction_stable = current_right_analysis.near_stable;
    out->elements.left_prediction_mean_error =
        current_left_analysis.near_mean_abs_error;
    out->elements.right_prediction_mean_error =
        current_right_analysis.near_mean_abs_error;
    out->elements.left_prediction_max_error =
        current_left_analysis.near_max_abs_error;
    out->elements.right_prediction_max_error =
        current_right_analysis.near_max_abs_error;

    int stable_rows = 0;
    const int stable_lo = top + (bottom - top) / 3;
    const int stable_hi = top + (bottom - top) * 2 / 3;
    for (int y = stable_lo; y <= stable_hi; ++y) {
        if (!raw_left_valid[y] || !raw_right_valid[y]) continue;
        if (raw_width[y] >= 16 && raw_width[y] <= kBinaryWidth - 8) {
            ++stable_rows;
        }
    }
    const bool left_lost = out->info.left_lost_count >
        (out->info.bottom - out->info.top) / 4;
    const bool right_lost = out->info.right_lost_count >
        (out->info.bottom - out->info.top) / 4;
    out->elements.two_side_stable =
        stable_rows >= p_.stable_two_side_min_rows && !left_lost && !right_lost;
    out->elements.cross = prediction_cross_lock_ &&
        symmetric_cross_opening &&
        roundabout_stage_ == RoundaboutVisionStage::None;
    out->elements.roundabout_stage = roundabout_stage_;
    out->elements.roundabout = roundabout_side_;
    out->elements.side_open = side_from_branches(
        out->elements.left_branch_count,
        out->elements.right_branch_count);
    if (out->elements.side_open == FeatureSide::None &&
        roundabout_stage_ != RoundaboutVisionStage::None) {
        out->elements.side_open = roundabout_side_;
    }
    out->elements.stable_side = roundabout_stable_side_;
    out->roundabout_track_point = roundabout_track_point_;
    out->stable_track_point = stable_track_point_;
    out->roundabout_recovery_point = roundabout_recovery_point_;

    for (int y = bottom; y >= top; --y) {
        if (final_left.valid) {
            out->left[y] = final_left.predict(y);
            out->left_valid[y] = 1;
        }
        if (final_right.valid) {
            out->right[y] = final_right.predict(y);
            out->right_valid[y] = 1;
        }
        if (out->left_valid[y] && out->right_valid[y] &&
            out->right[y] <= out->left[y] + 8) {
            const int center = (out->left[y] + out->right[y]) / 2;
            out->left[y] = clamp_value(center - 4, 1, kBinaryWidth - 10);
            out->right[y] = clamp_value(center + 4, out->left[y] + 8,
                                        kBinaryWidth - 2);
        }
        out->width[y] = (out->left_valid[y] && out->right_valid[y])
            ? std::max(0, out->right[y] - out->left[y]) : 0;
    }
}

void LegacyVisionPipeline::smooth_sidelines(RoadEstimateLite* out) {
    const int top = out->info.top;
    const int bottom = out->info.bottom - 1;
    const int half = std::max(0, p_.edge_smooth_window / 2);
    if (half <= 0) return;

    auto smooth_side = [&](const std::array<int, kBinaryHeight>& src,
                           const std::array<std::uint8_t, kBinaryHeight>& valid,
                           std::array<int, kBinaryHeight>* dst) {
        *dst = src;
        for (int y = top; y <= bottom; ++y) {
            if (!valid[y]) continue;
            int sum = 0;
            int count = 0;
            for (int yy = std::max(top, y - half);
                 yy <= std::min(bottom, y + half); ++yy) {
                if (!valid[yy]) continue;
                sum += src[yy];
                ++count;
            }
            if (count > 0) {
                (*dst)[y] = clamp_value(
                    static_cast<int>(std::lround(static_cast<double>(sum) / count)),
                    1, kBinaryWidth - 2);
            }
        }
    };

    std::array<int, kBinaryHeight> left_smoothed{};
    std::array<int, kBinaryHeight> right_smoothed{};
    smooth_side(out->left, out->left_valid, &left_smoothed);
    smooth_side(out->right, out->right_valid, &right_smoothed);
    out->left = left_smoothed;
    out->right = right_smoothed;
    for (int y = top; y <= bottom; ++y) {
        const bool both = out->left_valid[y] && out->right_valid[y];
        out->width[y] = both ? std::max(0, out->right[y] - out->left[y]) : 0;
    }
}
void LegacyVisionPipeline::find_corners(RoadEstimateLite* out) {
    const int top = out->info.top;
    const int bottom = out->info.bottom - 1;
    auto width_gain = [&](int y, int span) {
        const int a = safe_row(y - span);
        const int b = safe_row(y + span);
        return out->width[b] - out->width[a];
    };

    for (int y = top + 3; y <= bottom - 4; ++y) {
        const int left_delta = out->left[y + 1] - out->left[y - 1];
        const int right_delta = out->right[y - 1] - out->right[y + 1];
        if (!out->left_upper.valid &&
            out->left_valid[y] && width_gain(y, 2) > 5 && left_delta > 2) {
            out->left_upper = CornerPoint{y, out->left[y], true};
        }
        if (!out->right_upper.valid &&
            out->right_valid[y] && width_gain(y, 2) > 5 && right_delta > 2) {
            out->right_upper = CornerPoint{y, out->right[y], true};
        }
    }

    for (int y = bottom - 2; y >= top + 3; --y) {
        const int left_delta = out->left[y - 1] - out->left[y + 1];
        const int right_delta = out->right[y + 1] - out->right[y - 1];
        if (!out->left_lower.valid &&
            out->left_valid[y] && width_gain(y, 2) < -4 && left_delta > 2) {
            out->left_lower = CornerPoint{y, out->left[y], true};
        }
        if (!out->right_lower.valid &&
            out->right_valid[y] && width_gain(y, 2) < -4 && right_delta > 2) {
            out->right_lower = CornerPoint{y, out->right[y], true};
        }
    }
}

void LegacyVisionPipeline::find_midline(RoadEstimateLite* out) {
    int last_mid = kBinaryWidth / 2;
    double valid_rows = 0.0;

    for (int y = out->info.bottom - 1; y >= out->info.top; --y) {
        const bool left_ok = out->left_valid[y] != 0;
        const bool right_ok = out->right_valid[y] != 0;
        int mid = last_mid;
        if (left_ok && right_ok) {
            mid = (out->left[y] + out->right[y]) / 2;
            valid_rows += 1.0;
        } else if (left_ok) {
            mid = out->left[y] + kNominalTrackWidth / 2;
            valid_rows += 0.55;
        } else if (right_ok) {
            mid = out->right[y] - kNominalTrackWidth / 2;
            valid_rows += 0.55;
        }
        mid = clamp_value(mid, 1, kBinaryWidth - 2);
        out->mid[y] = mid;
        last_mid = mid;
    }

    const int control_row = p_.forward_row >= 0
        ? p_.forward_row
        : row_for_distance(p_.control_distance_cm);
    const int far_row = p_.far_row >= 0
        ? p_.far_row
        : row_for_distance(p_.far_distance_cm);
    out->info.control_row =
        clamp_value(control_row, out->info.top, out->info.bottom - 1);
    const double preview_error =
        calc_error_at_row(*out, out->info.control_row);
    out->far_error = calc_error_at_row(
        *out, clamp_value(far_row, out->info.top, out->info.bottom - 1));

    int wheel_left = 0;
    int wheel_right = 0;
    int wheel_top = 0;
    int wheel_bottom = 0;
    wheel_box(&wheel_left, &wheel_right, &wheel_top, &wheel_bottom);
    (void)wheel_bottom;
    const int vehicle_anchor_row = clamp_value(
        wheel_top - 2,
        out->info.control_row,
        out->info.bottom - 1);
    const double image_center = (kBinaryWidth - 1) * 0.5;
    const double vehicle_center =
        (wheel_left + wheel_right) * 0.5;
    const double vehicle_center_offset =
        (vehicle_center - image_center) / image_center;
    const double max_error_step =
        p_.error_step_limit / (kBinaryWidth * 0.5);
    const double vehicle_center_target = out->bottom_pair_valid
        ? clamp_value(
            calc_error_at_row(*out, vehicle_anchor_row) -
                vehicle_center_offset,
            -1.0,
            1.0)
        : 0.0;
    out->vehicle_center_error = last_vehicle_center_error_ +
        clamp_value(
            vehicle_center_target - last_vehicle_center_error_,
            -max_error_step,
            max_error_step);
    last_vehicle_center_error_ = out->vehicle_center_error;
    const double centering_weight =
        std::abs(out->vehicle_center_error) > 1e-6
        ? clamp_value(
            0.35 + 0.35 * std::abs(out->vehicle_center_error),
            0.35,
            0.60)
        : 0.0;
    out->line_error =
        preview_error * (1.0 - centering_weight) +
        out->vehicle_center_error * centering_weight;

    const int scan_rows = std::max(1, out->info.bottom - out->info.top);
    const double visibility =
        valid_rows / scan_rows;
    const double raw_edge_visibility = clamp_value(
        1.0 -
            static_cast<double>(
                out->info.left_lost_count + out->info.right_lost_count) /
                (2.0 * scan_rows),
        0.0,
        1.0);
    const double top_bonus =
        clamp_value((kBinaryHeight - out->info.top) / 45.0, 0.0, 1.0);
    out->line_confidence = clamp_value(
        0.50 * visibility +
            0.40 * raw_edge_visibility +
            0.10 * top_bonus,
        0.0,
        1.0);
    out->line_lost =
        out->line_confidence < 0.18 ||
        raw_edge_visibility < 0.08 ||
        valid_rows < 4.0;

    const double near_delta = out->line_error - last_error_;
    out->line_error = last_error_ +
        clamp_value(near_delta, -max_error_step, max_error_step);
    const double far_delta = out->far_error - last_far_error_;
    out->far_error = last_far_error_ +
        clamp_value(far_delta, -max_error_step, max_error_step);
    last_error_ = out->line_error;
    last_far_error_ = out->far_error;
    last_mid_col_ = out->mid[out->info.bottom - 1];
}

void LegacyVisionPipeline::classify_geometry(RoadEstimateLite* out) {
    const int top = out->info.top;
    const int bottom = out->info.bottom - 1;
    if (bottom <= top + 8) return;

    auto avg_width = [&](int y0, int y1) {
        y0 = safe_row(y0);
        y1 = safe_row(y1);
        if (y0 > y1) std::swap(y0, y1);
        int sum = 0;
        int count = 0;
        for (int y = y0; y <= y1; ++y) {
            if (out->width[y] > 0) {
                sum += out->width[y];
                ++count;
            }
        }
        return count > 0 ? static_cast<double>(sum) / count : 0.0;
    };

    const double near_w = avg_width(bottom - 8, bottom - 2);
    const double mid_w = avg_width((top + bottom) / 2 - 3,
                                   (top + bottom) / 2 + 3);
    const double far_w = avg_width(top + 2, top + 9);
    const bool broad_far = far_w > near_w * 1.25 && far_w - near_w > 8.0;
    const bool broad_mid = mid_w > near_w * 1.30 && mid_w - near_w > 8.0;

    const auto paired = [&](const CornerPoint& a, const CornerPoint& b) {
        return a.valid && b.valid &&
            std::abs(a.row - b.row) <= p_.cross_corner_row_tolerance;
    };
    const bool paired_lower = paired(out->left_lower, out->right_lower);
    const bool paired_upper = paired(out->left_upper, out->right_upper);
    const bool paired_cross_corners = paired_lower || paired_upper;

    (void)broad_mid;
    (void)broad_far;
    (void)paired_cross_corners;
    // Cross is now driven by prediction-line symmetric double-peak lock.
    // Keep the value produced by fit_missing_edges_from_bottom().
    out->elements.side_open = FeatureSide::None;
    if (!out->elements.cross) {
        const bool left_branch =
            out->elements.left_branch_count > 0;
        const bool right_branch =
            out->elements.right_branch_count > 0;
        if (left_branch && !right_branch) {
            out->elements.side_open = FeatureSide::Left;
        } else if (right_branch && !left_branch) {
            out->elements.side_open = FeatureSide::Right;
        }
    }

    out->elements.ramp =
        out->info.top < 10 &&
        std::abs(out->far_error - out->line_error) < 0.18 &&
        near_w > 20.0;

    int stripe_jumps = 0;
    for (int y = bottom - 18; y <= bottom - 5; ++y) {
        if (y <= 1 || y >= kBinaryHeight) continue;
        int transitions = 0;
        for (int x = 2; x < kBinaryWidth - 2; ++x) {
            if (frame_.binary[y][x] != frame_.binary[y][x - 1]) {
                ++transitions;
            }
        }
        if (transitions > 18) ++stripe_jumps;
    }
    out->elements.zebra = stripe_jumps >= 5;
}

double LegacyVisionPipeline::calc_error_at_row(const RoadEstimateLite& out,
                                               int row) const {
    const double center = (kBinaryWidth - 1) * 0.5;
    row = clamp_value(row, out.info.top, out.info.bottom - 1);
    double weighted_error = 0.0;
    double total_weight = 0.0;
    for (int offset = -2; offset <= 2; ++offset) {
        const int sample_row = row + offset;
        if (sample_row < out.info.top ||
            sample_row >= out.info.bottom) {
            continue;
        }
        const bool left_ok = out.left_valid[sample_row] != 0;
        const bool right_ok = out.right_valid[sample_row] != 0;
        if (!left_ok && !right_ok) continue;

        const double row_weight = 3.0 - std::abs(offset);
        const double evidence_weight =
            left_ok && right_ok ? 1.0 : 0.55;
        const double weight = row_weight * evidence_weight;
        weighted_error +=
            (out.mid[sample_row] - center) / center * weight;
        total_weight += weight;
    }
    if (total_weight <= 1e-6) {
        return clamp_value((out.mid[row] - center) / center, -1.0, 1.0);
    }
    return clamp_value(weighted_error / total_weight, -1.0, 1.0);
}

bool LegacyVisionPipeline::white(int row, int col) const {
    row = safe_row(row);
    col = safe_col(col);
    return frame_.binary[row][col] != 0;
}

int LegacyVisionPipeline::safe_row(int row) const {
    return clamp_value(row, 0, kBinaryHeight - 1);
}

int LegacyVisionPipeline::safe_col(int col) const {
    return clamp_value(col, 0, kBinaryWidth - 1);
}

double LegacyVisionPipeline::line_slope(int row1, int col1,
                                        int row2, int col2) const {
    const int dx = col2 - col1;
    if (dx == 0) return 0.0;
    return static_cast<double>(row2 - row1) / dx;
}

}  // namespace rewrite_path
