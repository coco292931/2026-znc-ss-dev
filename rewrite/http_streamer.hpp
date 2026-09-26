#pragma once

#include "path_params.hpp"
#include "path_types.hpp"
#include "vision_pipeline.hpp"

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifndef PATH_FOLLOW_NO_OPENCV
#include <opencv2/core.hpp>
#endif

namespace rewrite_path {

class HttpMjpegStreamer {
public:
    explicit HttpMjpegStreamer(const PathParams& params,
                               LegacyVisionPipeline* vision = nullptr);
    ~HttpMjpegStreamer();

    bool start();
    void stop();
    bool running() const { return running_.load(); }

#ifndef PATH_FOLLOW_NO_OPENCV
    void publish(const cv::Mat& frame,
                 const RoadEstimateLite& road,
                 const NavigationCommand& navigation,
                 const VisionFrame& vision_frame);
#endif
    void publish_telemetry(const TelemetrySample& sample);

    int active_clients() const {
        return stream_clients_.load() + telemetry_clients_.load();
    }

private:
#ifndef PATH_FOLLOW_NO_OPENCV
    struct RenderSnapshot {
        cv::Mat frame;
        RoadEstimateLite road;
        NavigationCommand navigation;
        VisionFrame vision_frame;
        std::uint64_t seq = 0;
        bool valid = false;
    };

    enum class StreamView : std::size_t {
        Overlay = 0,
        Gray,
        Saturation,
        LongestWhite,
        Count,
    };
    static constexpr std::size_t kStreamViewCount =
        static_cast<std::size_t>(StreamView::Count);

    void accept_loop();
    void encoder_loop();
    void handle_client(int fd);
    void stream_client(int fd, StreamView view);
    void telemetry_client(int fd);
    void send_index(int fd);
    void send_stats(int fd);
    void send_vision_params(int fd);
    void update_vision_params(int fd, const std::string& body);
    void send_wheel_box(int fd);
    void update_wheel_box(int fd, const std::string& body);
    void send_json(int fd, const std::string& body,
                   const char* status = "200 OK");
    std::string telemetry_json(const TelemetrySample& sample) const;
    bool send_jpeg_part(int fd, const std::vector<unsigned char>& jpeg);
    bool write_all(int fd, const void* data, std::size_t size);
#endif

    const PathParams& p_;
    std::atomic<bool> running_{false};
    std::atomic<int> stream_clients_{0};
    std::atomic<int> telemetry_clients_{0};
    int server_fd_ = -1;
    std::thread accept_thread_;
    std::thread encoder_thread_;

#ifndef PATH_FOLLOW_NO_OPENCV
    LegacyVisionPipeline renderer_;
    LegacyVisionPipeline* vision_ = nullptr;
    mutable std::mutex snapshot_mutex_;
    std::condition_variable snapshot_cv_;
    RenderSnapshot snapshot_;

    std::mutex jpeg_mutex_;
    std::condition_variable jpeg_cv_;
    std::array<std::vector<unsigned char>, kStreamViewCount> latest_jpegs_;
    std::array<std::uint64_t, kStreamViewCount> latest_jpeg_seqs_{};
    std::array<std::atomic<int>, kStreamViewCount> stream_view_clients_;
    std::uint64_t published_seq_ = 0;
    double encoded_fps_ = 0.0;
#endif

    std::mutex telemetry_mutex_;
    std::condition_variable telemetry_cv_;
    std::string latest_telemetry_;
    std::uint64_t telemetry_seq_ = 0;
    double last_telemetry_elapsed_s_ = -1.0;
};

}  // namespace rewrite_path
