#include "target_recognizer.hpp"

#ifndef PATH_FOLLOW_NO_OPENCV
#include <opencv2/imgproc.hpp>
#endif

#if !defined(PATH_FOLLOW_NO_OPENCV) && defined(REWRITE_WITH_NCNN)
#include <opencv2/imgcodecs.hpp>
#endif

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <utility>
#include <vector>

namespace rewrite_path {

#if !defined(PATH_FOLLOW_NO_OPENCV) && defined(REWRITE_WITH_NCNN)
namespace {

constexpr double kPageWidthCm = 12.0;
constexpr double kImageHeightCm = 12.0;
constexpr double kRedHeightCm = 5.0;
constexpr int kDirectionStableFrames = 4;
constexpr double kDirectionResetDeg = 20.0;
constexpr int kClassVoteWindow = 7;
constexpr int kClassVoteRequired = 4;
constexpr int kClassVoteMargin = 2;
constexpr int kMissingResetFrames = 5;
constexpr int kWarpSize = 96;
constexpr double kPi = 3.14159265358979323846;

// Defaults copied from runhorse/recognize_printed_marker_tkinter_config.json.
constexpr int kRedHLow1 = 3;
constexpr int kRedHHigh1 = 5;
constexpr int kRedHLow2 = 170;
constexpr int kRedHHigh2 = 180;
constexpr int kRedSMin = 120;
constexpr int kRedVMin = 162;
constexpr double kMinRedArea = 300.0;
constexpr double kMinRedAspect = 1.4;
constexpr int kMorphKernel = 3;
constexpr double kCropScale = 1.0813148788927336;
constexpr double kCropExposure = 0.9799307958477508;
constexpr double kCropContrast = 3.0;
constexpr double kCropBlur = 0.0;
constexpr double kNearGeometryFix = -0.13;
constexpr double kRoiTop = 0.2;
constexpr double kRoiBottom = 0.78;
constexpr double kRoiLeft = 0.1;
constexpr double kRoiRight = 0.9;

constexpr double kCalibrationWidth = 320.0;
constexpr double kCalibrationHeight = 240.0;
constexpr double kCalibrationCenterX = kCalibrationWidth * 0.5;
constexpr double kCameraHeightCm = 28.0;
constexpr double kCameraForwardFromRearAxleCm = 15.0;
constexpr double kCosPitch45 = 0.7071067811865476;
constexpr double kSinPitch45 = 0.7071067811865476;
constexpr double kCameraFocalX =
    kCalibrationWidth / (2.0 * 1.7320508075688772);

constexpr std::array<double, 14> kCalibrationRows = {{
    34.0, 44.0, 46.0, 50.0, 55.0, 66.0, 80.0,
    85.0, 92.0, 100.0, 113.0, 132.0, 158.0, 190.0,
}};

constexpr std::array<double, 14> kCalibrationInvDistances = {{
    0.0, 1.0 / 360.0, 1.0 / 300.0, 1.0 / 240.0,
    1.0 / 180.0, 1.0 / 120.0, 1.0 / 90.0, 1.0 / 80.0,
    1.0 / 70.0, 1.0 / 60.0, 1.0 / 50.0, 1.0 / 40.0,
    1.0 / 30.0, 1.0 / 20.0,
}};

double interpolate_calibration(const std::array<double, 14>& xs,
                               const std::array<double, 14>& ys,
                               double x) {
    if (x <= xs.front()) return ys.front();
    for (std::size_t i = 1; i < xs.size(); ++i) {
        if (x <= xs[i]) {
            const double denom = xs[i] - xs[i - 1];
            const double t = std::abs(denom) <= 1e-9
                ? 0.0 : (x - xs[i - 1]) / denom;
            return ys[i - 1] + (ys[i] - ys[i - 1]) * t;
        }
    }
    return ys.back();
}

double row_to_distance_cm(double row, int frame_height) {
    const double calibration_row =
        row * kCalibrationHeight / std::max(1, frame_height);
    const double inv_distance = interpolate_calibration(
        kCalibrationRows, kCalibrationInvDistances, calibration_row);
    if (inv_distance <= 1e-9) {
        return std::numeric_limits<double>::infinity();
    }
    return 1.0 / inv_distance;
}

double distance_to_row_px(double distance_cm, int frame_height) {
    const double inv_distance =
        (std::isfinite(distance_cm) && distance_cm > 0.0)
            ? 1.0 / distance_cm : 0.0;
    const double calibration_row = interpolate_calibration(
        kCalibrationInvDistances, kCalibrationRows, inv_distance);
    return calibration_row * std::max(1, frame_height) / kCalibrationHeight;
}

double camera_depth_cm(double distance_from_rear_axle_cm) {
    const double forward_from_camera =
        distance_from_rear_axle_cm - kCameraForwardFromRearAxleCm;
    return forward_from_camera * kCosPitch45 + kCameraHeightCm * kSinPitch45;
}

bool finite_point(const cv::Point2d& p) {
    return std::isfinite(p.x) && std::isfinite(p.y);
}

bool image_point_to_ground(const cv::Point2d& point,
                           const cv::Size& frame_size,
                           cv::Point2d* ground) {
    if (!ground || frame_size.width <= 0 || frame_size.height <= 0) {
        return false;
    }
    const double calibration_x =
        point.x * kCalibrationWidth / frame_size.width;
    const double distance = row_to_distance_cm(point.y, frame_size.height);
    if (!std::isfinite(distance)) return false;
    const double depth = camera_depth_cm(distance);
    if (!std::isfinite(depth) || std::abs(depth) <= 1e-6) return false;
    ground->x = (calibration_x - kCalibrationCenterX) * depth / kCameraFocalX;
    ground->y = distance;
    return finite_point(*ground);
}

bool ground_point_to_image(const cv::Point2d& ground,
                           const cv::Size& frame_size,
                           cv::Point2f* image) {
    if (!image || frame_size.width <= 0 || frame_size.height <= 0 ||
        !finite_point(ground)) {
        return false;
    }
    const double depth = camera_depth_cm(ground.y);
    if (!std::isfinite(depth) || std::abs(depth) <= 1e-6) return false;
    const double calibration_x =
        kCalibrationCenterX + kCameraFocalX * ground.x / depth;
    const double row = distance_to_row_px(ground.y, frame_size.height);
    if (!std::isfinite(row)) return false;
    image->x = static_cast<float>(
        calibration_x * frame_size.width / kCalibrationWidth);
    image->y = static_cast<float>(row);
    return std::isfinite(image->x) && std::isfinite(image->y);
}

cv::Point2d normalize_axis(const cv::Point2d& axis) {
    const double length = std::sqrt(axis.x * axis.x + axis.y * axis.y);
    if (length <= 1e-9 || !std::isfinite(length)) {
        return cv::Point2d(0.0, 1.0);
    }
    return cv::Point2d(axis.x / length, axis.y / length);
}

cv::Point2d normalize_forward_axis(const cv::Point2d& axis) {
    cv::Point2d normalized = normalize_axis(axis);
    if (normalized.y < 0.0) normalized *= -1.0;
    return normalized;
}

cv::Point2d perpendicular_width_axis(const cv::Point2d& forward_axis) {
    cv::Point2d width(forward_axis.y, -forward_axis.x);
    width = normalize_axis(width);
    if (width.x < 0.0) width *= -1.0;
    return width;
}

double angle_between_degrees(const cv::Point2d& a, const cv::Point2d& b) {
    const double dot = clamp_value(a.x * b.x + a.y * b.y, -1.0, 1.0);
    return std::acos(dot) * 180.0 / kPi;
}

cv::Point2d weighted_direction_mean(const std::vector<cv::Point2d>& history) {
    if (history.empty()) return cv::Point2d(0.0, 1.0);
    cv::Point2d sum(0.0, 0.0);
    for (std::size_t i = 0; i < history.size(); ++i) {
        const double weight = static_cast<double>(i + 1);
        sum += history[i] * weight;
    }
    return normalize_forward_axis(sum);
}

bool fit_image_width_edge(const std::vector<cv::Point>& contour,
                          const cv::Point2f box[4],
                          cv::Point2d* edge_a,
                          cv::Point2d* edge_b) {
    if (!edge_a || !edge_b) return false;
    double max_edge_len = 0.0;
    cv::Point2d fallback_a(box[0].x, box[0].y);
    cv::Point2d fallback_b(box[1].x, box[1].y);
    for (int i = 0; i < 4; ++i) {
        const cv::Point2d a(box[i].x, box[i].y);
        const cv::Point2d b(box[(i + 1) % 4].x, box[(i + 1) % 4].y);
        const double len = cv::norm(a - b);
        if (len > max_edge_len) {
            max_edge_len = len;
            fallback_a = a;
            fallback_b = b;
        }
    }
    if (contour.size() < 4 || max_edge_len <= 1e-6) {
        *edge_a = fallback_a;
        *edge_b = fallback_b;
        return max_edge_len > 1e-6;
    }

    cv::Mat data(static_cast<int>(contour.size()), 2, CV_64F);
    for (std::size_t i = 0; i < contour.size(); ++i) {
        data.at<double>(static_cast<int>(i), 0) = contour[i].x;
        data.at<double>(static_cast<int>(i), 1) = contour[i].y;
    }
    cv::PCA pca(data, cv::Mat(), cv::PCA::DATA_AS_ROW);
    cv::Point2d center(pca.mean.at<double>(0, 0),
                       pca.mean.at<double>(0, 1));
    cv::Point2d axis(pca.eigenvectors.at<double>(0, 0),
                     pca.eigenvectors.at<double>(0, 1));
    axis = normalize_axis(axis);
    if (axis.x < 0.0) axis *= -1.0;
    const double half_length = max_edge_len * 0.5;
    *edge_a = center - axis * half_length;
    *edge_b = center + axis * half_length;
    return finite_point(*edge_a) && finite_point(*edge_b);
}

cv::Rect default_target_roi(const cv::Size& size) {
    const int x0 = clamp_value(
        static_cast<int>(std::round(kRoiLeft * size.width)), 0, size.width);
    const int x1 = clamp_value(
        static_cast<int>(std::round(kRoiRight * size.width)), x0, size.width);
    const int y0 = clamp_value(
        static_cast<int>(std::round(kRoiTop * size.height)), 0, size.height);
    const int y1 = clamp_value(
        static_cast<int>(std::round(kRoiBottom * size.height)), y0, size.height);
    return cv::Rect(x0, y0, std::max(0, x1 - x0), std::max(0, y1 - y0));
}

cv::Mat center_scale_crop(const cv::Mat& image, double scale) {
    if (image.empty() || std::abs(scale - 1.0) < 1e-6) return image.clone();
    scale = std::max(0.05, scale);
    const int width = image.cols;
    const int height = image.rows;
    const int scaled_width = std::max(1, static_cast<int>(std::round(width * scale)));
    const int scaled_height = std::max(1, static_cast<int>(std::round(height * scale)));

    cv::Mat resized;
    cv::resize(image, resized, cv::Size(scaled_width, scaled_height),
               0.0, 0.0, cv::INTER_LINEAR);
    if (scale > 1.0) {
        const int left = std::max(0, (scaled_width - width) / 2);
        const int top = std::max(0, (scaled_height - height) / 2);
        return resized(cv::Rect(left, top, width, height)).clone();
    }

    const int pad_left = std::max(0, (width - scaled_width) / 2);
    const int pad_top = std::max(0, (height - scaled_height) / 2);
    const int pad_right = std::max(0, width - scaled_width - pad_left);
    const int pad_bottom = std::max(0, height - scaled_height - pad_top);
    cv::Mat padded;
    cv::copyMakeBorder(resized, padded, pad_top, pad_bottom, pad_left,
                       pad_right, cv::BORDER_REPLICATE);
    return padded(cv::Rect(0, 0, width, height)).clone();
}

cv::Mat apply_default_crop_postprocess(const cv::Mat& crop) {
    cv::Mat processed = center_scale_crop(crop, kCropScale);
    if (std::abs(kCropExposure - 1.0) > 1e-6) {
        cv::convertScaleAbs(processed, processed, std::max(0.0, kCropExposure), 0.0);
    }
    if (std::abs(kCropContrast - 1.0) > 1e-6) {
        cv::Mat float_image;
        processed.convertTo(float_image, CV_32F);
        float_image = (float_image - 127.5) * std::max(0.0, kCropContrast) + 127.5;
        float_image.convertTo(processed, CV_8U);
    }
    if (kCropBlur > 0.0) {
        cv::GaussianBlur(processed, processed, cv::Size(0, 0), kCropBlur);
    }
    return processed;
}

}  // namespace
#endif

AsyncTargetRecognizer::AsyncTargetRecognizer(const PathParams& params) : p_(params) {}

AsyncTargetRecognizer::~AsyncTargetRecognizer() {
    stop();
}

bool AsyncTargetRecognizer::init() {
#if !defined(PATH_FOLLOW_NO_OPENCV) && defined(REWRITE_WITH_NCNN)
    const std::string param = p_.model_dir + "/best.ncnn.param";
    const std::string bin = p_.model_dir + "/best.ncnn.bin";
    net_.opt.use_vulkan_compute = false;
    net_.opt.num_threads = 2;
    if (net_.load_param(param.c_str()) != 0 ||
        net_.load_model(bin.c_str()) != 0) {
        std::fprintf(stderr, "[NCNN] failed to load model: %s\n", param.c_str());
        ready_ = false;
        return false;
    }
    ready_ = true;
    std::printf("[NCNN] model ready: %s input=%dx%d\n", param.c_str(),
                p_.target_input_size, p_.target_input_size);
    return true;
#else
    return false;
#endif
}

void AsyncTargetRecognizer::start() {
#if !defined(PATH_FOLLOW_NO_OPENCV) && defined(REWRITE_WITH_NCNN)
    stop_ = false;
    worker_ = std::thread([this] { loop(); });
#endif
}

void AsyncTargetRecognizer::stop() {
#if !defined(PATH_FOLLOW_NO_OPENCV) && defined(REWRITE_WITH_NCNN)
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        stop_ = true;
    }
    frame_cv_.notify_all();
    if (worker_.joinable()) worker_.join();
#endif
}

#ifndef PATH_FOLLOW_NO_OPENCV
void AsyncTargetRecognizer::submit(const cv::Mat& frame) {
#if defined(REWRITE_WITH_NCNN)
    if (frame.empty()) return;
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        pending_ = frame.clone();
        has_pending_ = true;
    }
    frame_cv_.notify_one();
#else
    (void)frame;
#endif
}
#endif

TargetObservation AsyncTargetRecognizer::latest() const {
    std::lock_guard<std::mutex> lock(result_mutex_);
    return latest_;
}

#if !defined(PATH_FOLLOW_NO_OPENCV) && defined(REWRITE_WITH_NCNN)
void AsyncTargetRecognizer::loop() {
    while (true) {
        cv::Mat frame;
        {
            std::unique_lock<std::mutex> lock(frame_mutex_);
            frame_cv_.wait(lock, [&] { return stop_ || has_pending_; });
            if (stop_) break;
            frame = pending_;
            has_pending_ = false;
        }
        classify(frame);
    }
}

void AsyncTargetRecognizer::classify(const cv::Mat& frame) {
    TargetObservation obs;
    cv::Mat crop;
    if (!extract_red_crop(frame, &crop, &obs)) {
        ++missing_count_;
        publish_none(missing_count_ >= kMissingResetFrames);
        return;
    }
    missing_count_ = 0;

    if (!ready_) {
        publish_none(false);
        return;
    }
    const int input_size = p_.target_input_size;
    cv::Mat resized;
    const cv::Mat processed_crop = apply_default_crop_postprocess(crop);
    cv::resize(processed_crop, resized, cv::Size(input_size, input_size));
    ncnn::Mat input = ncnn::Mat::from_pixels(
        resized.data, ncnn::Mat::PIXEL_BGR2RGB, input_size, input_size);
    const float mean[3] = {0.0f, 0.0f, 0.0f};
    const float norm[3] = {1.0f / 255.0f, 1.0f / 255.0f, 1.0f / 255.0f};
    input.substract_mean_normalize(mean, norm);

    ncnn::Extractor ex = net_.create_extractor();
    if (ex.input("in0", input) != 0) {
        publish_candidate(obs, TargetKind::None, 0.0f);
        return;
    }
    ncnn::Mat output;
    if (ex.extract("out0", output) != 0 || output.w != 6) {
        std::fprintf(stderr, "[NCNN] invalid classifier output width: %d\n",
                     output.w);
        publish_candidate(obs, TargetKind::None, 0.0f);
        return;
    }

    std::array<float, 6> probabilities{};
    float probability_sum = 0.0f;
    bool already_probabilities = true;
    for (int i = 0; i < 6; ++i) {
        probabilities[i] = output[i];
        probability_sum += output[i];
        already_probabilities = already_probabilities &&
            std::isfinite(output[i]) && output[i] >= 0.0f && output[i] <= 1.0f;
    }
    already_probabilities = already_probabilities &&
        std::abs(probability_sum - 1.0f) <= 1e-3f;
    if (!already_probabilities) {
        const float maximum = *std::max_element(
            probabilities.begin(), probabilities.end());
        float denominator = 0.0f;
        for (float& probability : probabilities) {
            probability = std::exp(probability - maximum);
            denominator += probability;
        }
        if (!std::isfinite(denominator) || denominator <= 0.0f) {
            publish_candidate(obs, TargetKind::None, 0.0f);
            return;
        }
        for (float& probability : probabilities) probability /= denominator;
    }

    float best_score = 0.0f;
    const TargetKind best_kind = target_kind_from_probabilities(
        probabilities, &best_score);
    const TargetKind kind = best_score >= p_.target_enter_confidence
        ? best_kind : TargetKind::None;
    publish_candidate(obs, kind, best_score);
}

void AsyncTargetRecognizer::publish_none(bool reset_tracking) {
    if (reset_tracking) {
        class_history_.clear();
        latched_kind_ = TargetKind::None;
        latched_confidence_ = 0.0f;
        latched_was_close_ = false;
        far_after_close_count_ = 0;
    }
    std::lock_guard<std::mutex> lock(result_mutex_);
    latest_ = TargetObservation{};
}

void AsyncTargetRecognizer::publish_candidate(TargetObservation observation,
                                              TargetKind kind,
                                              float confidence) {
    if (latched_kind_ != TargetKind::None) {
        if (observation.size >= p_.target_close_size) {
            latched_was_close_ = true;
            far_after_close_count_ = 0;
        } else if (latched_was_close_ &&
                   observation.size <= p_.target_close_size * 0.30) {
            ++far_after_close_count_;
            if (far_after_close_count_ >= 3) {
                class_history_.clear();
                latched_kind_ = TargetKind::None;
                latched_confidence_ = 0.0f;
                latched_was_close_ = false;
                far_after_close_count_ = 0;
            }
        } else {
            far_after_close_count_ = 0;
        }
    }

    if (latched_kind_ == TargetKind::None && kind != TargetKind::None) {
        class_history_.push_back(kind);
        if (class_history_.size() > static_cast<std::size_t>(kClassVoteWindow)) {
            class_history_.erase(class_history_.begin());
        }

        int weapon_count = 0;
        int supply_count = 0;
        int vehicle_count = 0;
        for (TargetKind vote : class_history_) {
            weapon_count += vote == TargetKind::Weapon ? 1 : 0;
            supply_count += vote == TargetKind::Supply ? 1 : 0;
            vehicle_count += vote == TargetKind::Vehicle ? 1 : 0;
        }
        const std::array<std::pair<int, TargetKind>, 3> votes = {{
            {weapon_count, TargetKind::Weapon},
            {supply_count, TargetKind::Supply},
            {vehicle_count, TargetKind::Vehicle},
        }};
        int best_count = 0;
        int runner_up_count = 0;
        TargetKind voted_kind = TargetKind::None;
        for (const auto& vote : votes) {
            if (vote.first > best_count) {
                runner_up_count = best_count;
                best_count = vote.first;
                voted_kind = vote.second;
            } else if (vote.first > runner_up_count) {
                runner_up_count = vote.first;
            }
        }
        if (best_count >= kClassVoteRequired &&
            best_count - runner_up_count >= kClassVoteMargin) {
            latched_kind_ = voted_kind;
            latched_confidence_ = confidence;
        }
    }

    if (latched_kind_ == TargetKind::None) {
        std::lock_guard<std::mutex> lock(result_mutex_);
        latest_ = TargetObservation{};
        return;
    }
    if (kind == latched_kind_ && confidence >= p_.target_enter_confidence) {
        latched_confidence_ = std::max(latched_confidence_, confidence);
    }
    observation.kind = latched_kind_;
    observation.confidence = latched_confidence_;
    observation.valid = true;
    std::lock_guard<std::mutex> lock(result_mutex_);
    latest_ = observation;
}

void AsyncTargetRecognizer::reset_direction_history() {
    direction_history_.clear();
}

cv::Point2d AsyncTargetRecognizer::stabilize_direction(
    const cv::Point2d& forward_axis) {
    const cv::Point2d axis = normalize_forward_axis(forward_axis);
    if (!direction_history_.empty()) {
        const cv::Point2d previous = weighted_direction_mean(direction_history_);
        if (angle_between_degrees(previous, axis) > kDirectionResetDeg) {
            direction_history_.clear();
            direction_history_.push_back(axis);
            return axis;
        }
    }

    direction_history_.push_back(axis);
    if (direction_history_.size() >
        static_cast<std::size_t>(kDirectionStableFrames)) {
        direction_history_.erase(direction_history_.begin());
    }
    return weighted_direction_mean(direction_history_);
}

bool AsyncTargetRecognizer::build_geometry_crop(
    const cv::Mat& frame,
    const std::vector<cv::Point>& contour,
    double red_area,
    cv::Mat* crop,
    TargetObservation* obs) {
    if (frame.empty() || contour.size() < 4 || !crop || !obs) return false;
    const cv::Size frame_size(frame.cols, frame.rows);

    std::vector<cv::Point2d> ground_points;
    ground_points.reserve(contour.size());
    for (const cv::Point& point : contour) {
        cv::Point2d ground;
        if (image_point_to_ground(
                cv::Point2d(point.x, point.y), frame_size, &ground)) {
            ground_points.push_back(ground);
        }
    }
    if (ground_points.size() < 4) return false;

    cv::RotatedRect image_rect = cv::minAreaRect(contour);
    cv::Point2f box[4];
    image_rect.points(box);

    cv::Point2d width_a_image;
    cv::Point2d width_b_image;
    if (!fit_image_width_edge(contour, box, &width_a_image, &width_b_image)) {
        return false;
    }

    cv::Point2d width_a_ground;
    cv::Point2d width_b_ground;
    if (!image_point_to_ground(width_a_image, frame_size, &width_a_ground) ||
        !image_point_to_ground(width_b_image, frame_size, &width_b_ground)) {
        return false;
    }

    cv::Point2d width_axis = normalize_axis(width_b_ground - width_a_ground);
    if (width_axis.x < 0.0) width_axis *= -1.0;
    cv::Point2d forward_axis(-width_axis.y, width_axis.x);
    if (forward_axis.y < 0.0) forward_axis *= -1.0;

    cv::Moments moments = cv::moments(contour);
    cv::Point2d center_image(image_rect.center.x, image_rect.center.y);
    if (std::abs(moments.m00) > 1e-6) {
        center_image.x = moments.m10 / moments.m00;
        center_image.y = moments.m01 / moments.m00;
    }
    obs->center_y_ratio = clamp_value(
        center_image.y / std::max(1, frame.rows), 0.0, 1.0);
    cv::Point2d red_center;
    if (!image_point_to_ground(center_image, frame_size, &red_center)) {
        return false;
    }

    const double near_weight =
        clamp_value((90.0 - red_center.y) / 70.0, 0.0, 1.0);
    const double near_fix =
        clamp_value(kNearGeometryFix * near_weight, -0.5, 0.6);
    const double width_scale =
        clamp_value(1.0 - 0.5 * near_fix, 0.65, 1.35);
    const double height_scale =
        clamp_value(1.0 + near_fix, 0.65, 1.65);
    const double red_height_cm = kRedHeightCm * height_scale;
    const double image_height_cm = kImageHeightCm * height_scale;
    const double half_width_cm = kPageWidthCm * 0.5 * width_scale;

    forward_axis = stabilize_direction(forward_axis);
    width_axis = perpendicular_width_axis(forward_axis);

    const auto projected_image_center_y = [&](const cv::Point2d& axis,
                                              double* out_y) {
        cv::Point2f image_center;
        const cv::Point2d ground_center =
            red_center + axis * (red_height_cm * 0.5 + image_height_cm * 0.5);
        if (!ground_point_to_image(ground_center, frame_size, &image_center)) {
            return false;
        }
        *out_y = image_center.y;
        return std::isfinite(*out_y);
    };

    double current_image_y = 0.0;
    double flipped_image_y = 0.0;
    const bool current_ok =
        projected_image_center_y(forward_axis, &current_image_y);
    const bool flipped_ok =
        projected_image_center_y(forward_axis * -1.0, &flipped_image_y);
    if (flipped_ok &&
        (!current_ok ||
         (current_image_y >= center_image.y - 0.5 &&
          flipped_image_y < current_image_y))) {
        forward_axis *= -1.0;
    }
    width_axis = perpendicular_width_axis(forward_axis);

    const cv::Point2d red_far_center =
        red_center + forward_axis * (red_height_cm * 0.5);
    const cv::Point2d image_far_center =
        red_far_center + forward_axis * image_height_cm;

    std::vector<cv::Point2d> image_ground = {
        image_far_center - width_axis * half_width_cm,
        image_far_center + width_axis * half_width_cm,
        red_far_center + width_axis * half_width_cm,
        red_far_center - width_axis * half_width_cm,
    };

    std::vector<cv::Point2f> image_quad;
    image_quad.reserve(4);
    for (const cv::Point2d& ground : image_ground) {
        cv::Point2f image_point;
        if (!ground_point_to_image(ground, frame_size, &image_point)) {
            return false;
        }
        image_quad.push_back(image_point);
    }

    const double top_width = cv::norm(image_quad[1] - image_quad[0]);
    const double bottom_width = cv::norm(image_quad[2] - image_quad[3]);
    const double left_height = cv::norm(image_quad[3] - image_quad[0]);
    const double right_height = cv::norm(image_quad[2] - image_quad[1]);
    if (std::min({top_width, bottom_width, left_height, right_height}) < 4.0) {
        return false;
    }

    const std::vector<cv::Point2f> dst = {
        cv::Point2f(0.0f, 0.0f),
        cv::Point2f(static_cast<float>(kWarpSize - 1), 0.0f),
        cv::Point2f(static_cast<float>(kWarpSize - 1),
                    static_cast<float>(kWarpSize - 1)),
        cv::Point2f(0.0f, static_cast<float>(kWarpSize - 1)),
    };
    const cv::Mat matrix = cv::getPerspectiveTransform(image_quad, dst);
    cv::warpPerspective(frame, *crop, matrix, cv::Size(kWarpSize, kWarpSize),
                        cv::INTER_LINEAR, cv::BORDER_REPLICATE);
    if (crop->empty()) return false;

    double min_x = image_quad[0].x;
    double max_x = image_quad[0].x;
    double min_y = image_quad[0].y;
    double max_y = image_quad[0].y;
    for (const cv::Point2f& point : image_quad) {
        min_x = std::min(min_x, static_cast<double>(point.x));
        max_x = std::max(max_x, static_cast<double>(point.x));
        min_y = std::min(min_y, static_cast<double>(point.y));
        max_y = std::max(max_y, static_cast<double>(point.y));
    }
    const cv::Rect frame_rect(0, 0, frame.cols, frame.rows);
    cv::Rect bounds(
        static_cast<int>(std::floor(min_x)),
        static_cast<int>(std::floor(min_y)),
        static_cast<int>(std::ceil(max_x - min_x)),
        static_cast<int>(std::ceil(max_y - min_y)));
    bounds &= frame_rect;
    obs->x = bounds.x;
    obs->y = bounds.y;
    obs->w = bounds.width;
    obs->h = bounds.height;
    obs->size =
        clamp_value(red_area / (frame.cols * frame.rows * 0.10), 0.0, 1.0);
    return true;
}

bool AsyncTargetRecognizer::extract_red_crop(const cv::Mat& frame,
                                             cv::Mat* crop,
                                             TargetObservation* obs) {
    if (frame.empty() || !crop || !obs) return false;
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask1, mask2, mask;
    cv::inRange(hsv, cv::Scalar(kRedHLow1, kRedSMin, kRedVMin),
                cv::Scalar(kRedHHigh1, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(kRedHLow2, kRedSMin, kRedVMin),
                cv::Scalar(kRedHHigh2, 255, 255), mask2);
    cv::bitwise_or(mask1, mask2, mask);
    const cv::Rect roi = default_target_roi(frame.size());
    if (roi.width <= 0 || roi.height <= 0) return false;
    cv::Mat roi_mask = cv::Mat::zeros(mask.size(), CV_8UC1);
    roi_mask(roi).setTo(cv::Scalar(255));
    cv::bitwise_and(mask, roi_mask, mask);
    const cv::Mat kernel =
        cv::getStructuringElement(cv::MORPH_RECT,
                                  cv::Size(kMorphKernel, kMorphKernel));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel,
                     cv::Point(-1, -1), 1);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    const double min_area = kMinRedArea;
    struct RedCandidate {
        std::vector<cv::Point> contour;
        cv::RotatedRect rect;
        cv::Rect bounds;
        cv::Scalar mean_hsv;
        double area = 0.0;
        double fill_ratio = 0.0;
    };
    std::vector<RedCandidate> candidates;
    double best_score = 0.0;
    double best_area = 0.0;
    std::vector<cv::Point> best_contour;
    for (const auto& c : contours) {
        const double area = cv::contourArea(c);
        if (area < min_area) continue;

        const cv::Rect r = cv::boundingRect(c);
        const int bottom = r.y + r.height - 1;
        const int roi_bottom = roi.y + roi.height;
        if ((roi_bottom < frame.rows && bottom >= roi_bottom - 2) ||
            bottom >= frame.rows - 2) {
            continue;
        }

        const cv::RotatedRect rect = cv::minAreaRect(c);
        const double rw = rect.size.width;
        const double rh = rect.size.height;
        if (rw <= 1.0 || rh <= 1.0) continue;
        const double aspect = std::max(rw, rh) / std::min(rw, rh);
        if (aspect < kMinRedAspect || aspect > 6.0) continue;

        const double rect_area = rw * rh;
        const double fill_ratio = rect_area > 1e-6 ? area / rect_area : 0.0;
        if (fill_ratio < 0.25) continue;

        cv::Mat contour_mask = cv::Mat::zeros(mask.size(), CV_8UC1);
        std::vector<std::vector<cv::Point>> one_contour = {c};
        cv::drawContours(contour_mask, one_contour, -1, cv::Scalar(255), -1);
        const cv::Scalar mean_hsv = cv::mean(hsv, contour_mask);
        RedCandidate candidate;
        candidate.contour = c;
        candidate.rect = rect;
        candidate.bounds = r;
        candidate.mean_hsv = mean_hsv;
        candidate.area = area;
        candidate.fill_ratio = fill_ratio;
        candidates.push_back(std::move(candidate));
    }

    for (const RedCandidate& candidate : candidates) {
        // Printed red objects sit above and overlap the reference strip.
        bool upper_image_red = false;
        for (const RedCandidate& lower : candidates) {
            if (&lower == &candidate) continue;
            const double vertical_delta =
                lower.rect.center.y - candidate.rect.center.y;
            const int overlap = std::max(
                0,
                std::min(candidate.bounds.x + candidate.bounds.width,
                         lower.bounds.x + lower.bounds.width) -
                    std::max(candidate.bounds.x, lower.bounds.x));
            const double overlap_ratio =
                static_cast<double>(overlap) /
                std::max(1, std::min(candidate.bounds.width,
                                     lower.bounds.width));
            const bool close_rows = vertical_delta <=
                3.0 * std::max(candidate.bounds.height, lower.bounds.height);
            const bool meaningful_lower =
                lower.area >= 0.15 * candidate.area;
            if (vertical_delta > 0.0 && close_rows &&
                overlap_ratio >= 0.25 && meaningful_lower) {
                upper_image_red = true;
                break;
            }
        }
        if (upper_image_red) continue;

        const double y_weight =
            candidate.rect.center.y / std::max(1, frame.rows);
        const double score =
            candidate.area * std::max(candidate.fill_ratio, 0.01) *
            (1.0 + candidate.mean_hsv[1] / 255.0) *
            (1.0 + 0.25 * y_weight);
        if (score > best_score) {
            best_score = score;
            best_area = candidate.area;
            best_contour = candidate.contour;
        }
    }
    if (best_contour.empty()) {
        reset_direction_history();
        return false;
    }

    if (!build_geometry_crop(frame, best_contour, best_area, crop, obs)) {
        reset_direction_history();
        return false;
    }
    return true;
}
#endif

}  // namespace rewrite_path
