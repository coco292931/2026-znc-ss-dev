#pragma once

#include "path_params.hpp"
#include "path_types.hpp"

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#ifndef PATH_FOLLOW_NO_OPENCV
#include <opencv2/core.hpp>
#endif

#if !defined(PATH_FOLLOW_NO_OPENCV) && defined(REWRITE_WITH_NCNN)
#include <ncnn/net.h>
#endif

namespace rewrite_path {

struct TargetObservation {
    bool valid = false;
    TargetKind kind = TargetKind::None;
    double confidence = 0.0;
    double size = 0.0;
    double center_y_ratio = 0.0;
    int x = 0;
    int y = 0;
    int w = 0;
    int h = 0;
};

class AsyncTargetRecognizer {
public:
    explicit AsyncTargetRecognizer(const PathParams& params);
    ~AsyncTargetRecognizer();

    bool init();
    void start();
    void stop();

#ifndef PATH_FOLLOW_NO_OPENCV
    void submit(const cv::Mat& frame);
#endif

    TargetObservation latest() const;

private:
#if !defined(PATH_FOLLOW_NO_OPENCV) && defined(REWRITE_WITH_NCNN)
    void loop();
    void classify(const cv::Mat& frame);
    bool extract_red_crop(const cv::Mat& frame, cv::Mat* crop, TargetObservation* obs);
    bool build_geometry_crop(const cv::Mat& frame,
                             const std::vector<cv::Point>& contour,
                             double red_area,
                             cv::Mat* crop,
                             TargetObservation* obs);
    cv::Point2d stabilize_direction(const cv::Point2d& forward_axis);
    void reset_direction_history();
    void publish_none(bool reset_tracking);
    void publish_candidate(TargetObservation observation,
                           TargetKind kind,
                           float confidence);
#endif

    const PathParams& p_;
    mutable std::mutex result_mutex_;
    TargetObservation latest_;

#if !defined(PATH_FOLLOW_NO_OPENCV) && defined(REWRITE_WITH_NCNN)
    std::thread worker_;
    ncnn::Net net_;
    std::mutex frame_mutex_;
    std::condition_variable frame_cv_;
    cv::Mat pending_;
    std::vector<cv::Point2d> direction_history_;
    std::vector<TargetKind> class_history_;
    TargetKind latched_kind_ = TargetKind::None;
    float latched_confidence_ = 0.0f;
    int missing_count_ = 0;
    bool latched_was_close_ = false;
    int far_after_close_count_ = 0;
    bool has_pending_ = false;
    bool stop_ = false;
    bool ready_ = false;
#endif
};

}  // namespace rewrite_path
