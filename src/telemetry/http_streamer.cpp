#include "http_streamer.hpp"

#ifndef PATH_FOLLOW_NO_OPENCV

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace rewrite_path {

namespace {

const char kIndexHtml[] =
    "<!doctype html><html><head><meta charset='utf-8'>"
    "<title>rewrite path follow</title>"
    "<style>body{margin:0;background:#111;color:#ddd;font-family:sans-serif}"
    "main{max-width:960px;margin:20px auto}img{width:100%;background:#000}"
    "code{color:#9ee}</style></head><body><main>"
    "<h2>rewrite path follow</h2><img src='/stream'>"
    "<p><code>/stream</code> overlay, <code>/stream/gray</code> gray, "
    "<code>/stream/path</code> longest-white path, "
    "<code>/stream/saturation</code> saturation, "
    "<code>/telemetry</code> NDJSON, <code>/stats</code> JSON, "
    "<code>/vision-params</code> live tuning, "
    "<code>/wheel-box</code> wheel-mask correction</p>"
    "</main></body></html>";

}  // namespace

HttpMjpegStreamer::HttpMjpegStreamer(const PathParams& params,
                                     LegacyVisionPipeline* vision)
    : p_(params), renderer_(params), vision_(vision) {
    for (auto& clients : stream_view_clients_) clients.store(0);
}

HttpMjpegStreamer::~HttpMjpegStreamer() {
    stop();
}

bool HttpMjpegStreamer::start() {
    if (running_.exchange(true)) return true;

    server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
        running_ = false;
        return false;
    }

    int opt = 1;
    ::setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(p_.http_port);
    if (::bind(server_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(server_fd_);
        server_fd_ = -1;
        running_ = false;
        return false;
    }
    if (::listen(server_fd_, 8) < 0) {
        ::close(server_fd_);
        server_fd_ = -1;
        running_ = false;
        return false;
    }

    accept_thread_ = std::thread([this] { accept_loop(); });
    encoder_thread_ = std::thread([this] { encoder_loop(); });
    return true;
}

void HttpMjpegStreamer::stop() {
    if (!running_.exchange(false)) return;
    if (server_fd_ >= 0) {
        ::shutdown(server_fd_, SHUT_RDWR);
        ::close(server_fd_);
        server_fd_ = -1;
    }
    snapshot_cv_.notify_all();
    jpeg_cv_.notify_all();
    telemetry_cv_.notify_all();
    if (accept_thread_.joinable()) accept_thread_.join();
    if (encoder_thread_.joinable()) encoder_thread_.join();
}

void HttpMjpegStreamer::publish(const cv::Mat& frame,
                                const RoadEstimateLite& road,
                                const NavigationCommand& navigation,
                                const VisionFrame& vision_frame) {
    if (!running_.load() || stream_clients_.load() <= 0 || frame.empty()) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        snapshot_.frame = frame.clone();
        snapshot_.road = road;
        snapshot_.navigation = navigation;
        snapshot_.vision_frame = vision_frame;
        snapshot_.seq = ++published_seq_;
        snapshot_.valid = true;
    }
    snapshot_cv_.notify_one();
}

void HttpMjpegStreamer::publish_telemetry(const TelemetrySample& sample) {
    if (!running_.load() || telemetry_clients_.load() <= 0) return;
    const double min_period = 1.0 / std::max(1, p_.telemetry_hz);
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        if (last_telemetry_elapsed_s_ >= 0.0 &&
            sample.elapsed_s - last_telemetry_elapsed_s_ < min_period) {
            return;
        }
        latest_telemetry_ = telemetry_json(sample);
        last_telemetry_elapsed_s_ = sample.elapsed_s;
        ++telemetry_seq_;
    }
    telemetry_cv_.notify_all();
}

void HttpMjpegStreamer::accept_loop() {
    while (running_.load()) {
        sockaddr_in client{};
        socklen_t len = sizeof(client);
        int fd = ::accept(server_fd_, reinterpret_cast<sockaddr*>(&client), &len);
        if (fd < 0) {
            if (!running_.load()) break;
            continue;
        }

        int flag = 1;
        ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
        timeval timeout{};
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
        ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        std::thread([this, fd] {
            handle_client(fd);
            ::close(fd);
        }).detach();
    }
}

void HttpMjpegStreamer::encoder_loop() {
    std::uint64_t encoded_snapshot_seq = 0;
    auto last_encode = std::chrono::steady_clock::now();
    while (running_.load()) {
        {
            std::unique_lock<std::mutex> lock(snapshot_mutex_);
            snapshot_cv_.wait_for(lock, std::chrono::milliseconds(100), [&] {
                return !running_.load() ||
                       (stream_clients_.load() > 0 &&
                        snapshot_.valid &&
                        snapshot_.seq != encoded_snapshot_seq);
            });
            if (!running_.load()) break;
            if (stream_clients_.load() <= 0 ||
                !snapshot_.valid ||
                snapshot_.seq == encoded_snapshot_seq) {
                continue;
            }
        }

        const auto now = std::chrono::steady_clock::now();
        const double min_period = 1.0 / std::max(1, p_.http_fps_limit);
        const double elapsed =
            std::chrono::duration<double>(now - last_encode).count();
        if (encoded_snapshot_seq != 0 && elapsed < min_period) {
            std::this_thread::sleep_for(
                std::chrono::duration<double>(min_period - elapsed));
        }

        RenderSnapshot snap;
        {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            if (!snapshot_.valid ||
                snapshot_.seq == encoded_snapshot_seq) {
                continue;
            }
            snap = snapshot_;
            encoded_snapshot_seq = snapshot_.seq;
        }

        std::array<bool, kStreamViewCount> requested{};
        bool any_requested = false;
        for (std::size_t i = 0; i < kStreamViewCount; ++i) {
            requested[i] = stream_view_clients_[i].load() > 0;
            any_requested = any_requested || requested[i];
        }
        if (!any_requested) continue;

        std::array<cv::Mat, kStreamViewCount> rendered;
        const std::size_t overlay_index =
            static_cast<std::size_t>(StreamView::Overlay);
        const std::size_t gray_index =
            static_cast<std::size_t>(StreamView::Gray);
        const std::size_t saturation_index =
            static_cast<std::size_t>(StreamView::Saturation);
        const std::size_t path_index =
            static_cast<std::size_t>(StreamView::LongestWhite);

        LegacyVisionPipeline& renderer = vision_ ? *vision_ : renderer_;
        if (requested[overlay_index]) {
            rendered[overlay_index] = renderer.draw_overlay(
                snap.frame, snap.road, snap.navigation);
        }
        if (requested[gray_index]) {
            cv::Mat gray;
            if (snap.frame.channels() == 1) {
                gray = snap.frame;
            } else {
                cv::cvtColor(snap.frame, gray, cv::COLOR_BGR2GRAY);
            }
            cv::cvtColor(gray, rendered[gray_index], cv::COLOR_GRAY2BGR);
            cv::putText(rendered[gray_index], "GRAYSCALE",
                        cv::Point(8, 22), cv::FONT_HERSHEY_SIMPLEX,
                        0.58, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        }
        if (requested[saturation_index]) {
            cv::Mat saturation;
            if (snap.frame.channels() == 1) {
                saturation = cv::Mat::zeros(
                    snap.frame.rows, snap.frame.cols, CV_8UC1);
            } else {
                cv::Mat hsv;
                cv::cvtColor(snap.frame, hsv, cv::COLOR_BGR2HSV);
                cv::extractChannel(hsv, saturation, 1);
            }
            cv::cvtColor(
                saturation, rendered[saturation_index], cv::COLOR_GRAY2BGR);
            const double mean_saturation = cv::mean(saturation)[0];
            char label[96];
            std::snprintf(label, sizeof(label),
                          "SATURATION mean=%.1f", mean_saturation);
            cv::putText(rendered[saturation_index], label,
                        cv::Point(8, 22), cv::FONT_HERSHEY_SIMPLEX,
                        0.58, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        }
        if (requested[path_index]) {
            rendered[path_index] = renderer.draw_longest_white_debug(
                snap.vision_frame, snap.road, snap.navigation,
                snap.frame.size());
        }

        std::array<std::vector<unsigned char>, kStreamViewCount> encoded;
        std::array<bool, kStreamViewCount> encoded_ok{};
        bool any_encoded = false;
        for (std::size_t i = 0; i < kStreamViewCount; ++i) {
            if (!requested[i] || rendered[i].empty()) continue;
            encoded_ok[i] = cv::imencode(
                ".jpg", rendered[i], encoded[i],
                {cv::IMWRITE_JPEG_QUALITY, p_.http_jpeg_quality});
            any_encoded = any_encoded || encoded_ok[i];
        }
        if (any_encoded) {
            const auto encoded_at = std::chrono::steady_clock::now();
            const double dt =
                std::chrono::duration<double>(encoded_at - last_encode).count();
            last_encode = encoded_at;
            {
                std::lock_guard<std::mutex> lock(jpeg_mutex_);
                for (std::size_t i = 0; i < kStreamViewCount; ++i) {
                    if (!encoded_ok[i]) continue;
                    latest_jpegs_[i].swap(encoded[i]);
                    ++latest_jpeg_seqs_[i];
                }
                if (dt > 1e-6) {
                    const double instant = 1.0 / dt;
                    encoded_fps_ =
                        encoded_fps_ <= 0.0 ? instant : encoded_fps_ * 0.85 + instant * 0.15;
                }
            }
            jpeg_cv_.notify_all();
        }
    }
}

void HttpMjpegStreamer::handle_client(int fd) {
    std::string req;
    req.reserve(2048);
    std::size_t header_end = std::string::npos;
    std::size_t content_length = 0;
    while (req.size() < 8192) {
        char buffer[2048];
        const ssize_t n = ::recv(fd, buffer, sizeof(buffer), 0);
        if (n <= 0) return;
        req.append(buffer, static_cast<std::size_t>(n));
        if (header_end == std::string::npos) {
            header_end = req.find("\r\n\r\n");
            if (header_end != std::string::npos) {
                std::string lower = req.substr(0, header_end);
                std::transform(lower.begin(), lower.end(), lower.begin(),
                               [](unsigned char c) {
                                   return static_cast<char>(std::tolower(c));
                               });
                const std::string key = "content-length:";
                const std::size_t pos = lower.find(key);
                if (pos != std::string::npos) {
                    const char* value = lower.c_str() + pos + key.size();
                    content_length = static_cast<std::size_t>(
                        std::max<long>(0, std::strtol(value, nullptr, 10)));
                }
            }
        }
        if (header_end != std::string::npos &&
            req.size() >= header_end + 4 + content_length) {
            break;
        }
    }
    if (req.find("GET /stream/gray ") == 0) {
        stream_client(fd, StreamView::Gray);
    } else if (req.find("GET /stream/saturation ") == 0) {
        stream_client(fd, StreamView::Saturation);
    } else if (req.find("GET /stream/path ") == 0) {
        stream_client(fd, StreamView::LongestWhite);
    } else if (req.find("GET /stream/overlay ") == 0 ||
               req.find("GET /stream ") == 0) {
        stream_client(fd, StreamView::Overlay);
    } else if (req.find("GET /telemetry") == 0) {
        telemetry_client(fd);
    } else if (req.find("GET /stats") == 0) {
        send_stats(fd);
    } else if (req.find("GET /vision-dump") == 0) {
        send_vision_dump(fd);
    } else if (req.find("GET /vision-params") == 0) {
        send_vision_params(fd);
    } else if (req.find("POST /vision-params") == 0) {
        const std::size_t body_pos = req.find("\r\n\r\n");
        update_vision_params(
            fd,
            body_pos == std::string::npos ? std::string() :
                req.substr(body_pos + 4, content_length));
    } else if (req.find("GET /wheel-box") == 0) {
        send_wheel_box(fd);
    } else if (req.find("POST /wheel-box") == 0) {
        const std::size_t body_pos = req.find("\r\n\r\n");
        update_wheel_box(
            fd,
            body_pos == std::string::npos ? std::string() :
                req.substr(body_pos + 4, content_length));
    } else {
        send_index(fd);
    }
}

void HttpMjpegStreamer::send_index(int fd) {
    char header[256];
    const int n = std::snprintf(
        header, sizeof(header),
        "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n"
        "Content-Length: %zu\r\nConnection: close\r\n\r\n",
        std::strlen(kIndexHtml));
    write_all(fd, header, static_cast<std::size_t>(n));
    write_all(fd, kIndexHtml, std::strlen(kIndexHtml));
}

void HttpMjpegStreamer::send_stats(int fd) {
    char body[256];
    std::uint64_t seq = 0;
    std::uint64_t telemetry_seq = 0;
    double fps = 0.0;
    std::size_t bytes = 0;
    {
        std::lock_guard<std::mutex> lock(jpeg_mutex_);
        for (std::size_t i = 0; i < kStreamViewCount; ++i) {
            seq = std::max(seq, latest_jpeg_seqs_[i]);
            bytes += latest_jpegs_[i].size();
        }
        fps = encoded_fps_;
    }
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        telemetry_seq = telemetry_seq_;
    }
    const int bn = std::snprintf(
        body, sizeof(body),
        "{\"clients\":%d,\"streamClients\":%d,\"telemetryClients\":%d,"
        "\"frameSeq\":%llu,\"telemetrySeq\":%llu,"
        "\"encodedFps\":%.2f,\"jpegBytes\":%zu}",
        active_clients(),
        stream_clients_.load(),
        telemetry_clients_.load(),
        static_cast<unsigned long long>(seq),
        static_cast<unsigned long long>(telemetry_seq),
        fps,
        bytes);
    char header[256];
    const int hn = std::snprintf(
        header, sizeof(header),
        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
        "Content-Length: %d\r\nConnection: close\r\n\r\n",
        bn);
    write_all(fd, header, static_cast<std::size_t>(hn));
    write_all(fd, body, static_cast<std::size_t>(bn));
}

void HttpMjpegStreamer::send_vision_dump(int fd) {
    RenderSnapshot snap;
    {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        snap = snapshot_;
    }

    std::string body;
    body.reserve(24000);
    char line[512];

    if (!snap.valid) {
        body = "snapshot invalid\n";
    } else {
        const RoadEstimateLite& r = snap.road;
        const RoadImageInfo& i = r.info;

        // 主线程真实的轮子遮罩框（寻线搜索的锚点）。
        const LegacyVisionPipeline& v = vision_ ? *vision_ : renderer_;
        const WheelMaskBox box = v.wheel_mask_box();
        std::snprintf(line, sizeof(line),
                      "wheel_box %d %d %d %d auto=%d top_minus_1=%d\n",
                      box.left, box.right, box.top, box.bottom,
                      box.auto_detected ? 1 : 0, box.top - 1);
        body += line;

        std::snprintf(line, sizeof(line),
                      "info top=%d bottom=%d max_column=%d white_num=%d "
                      "last_mid=%d control_row=%d far_row=%d anchor_row=%d\n",
                      i.top, i.bottom, i.max_column, i.white_num, i.last_mid,
                      i.control_row, i.far_row, i.vehicle_anchor_row);
        body += line;

        std::snprintf(line, sizeof(line),
                      "losts left=%d right=%d both=%d scan_rows=%d\n",
                      i.left_lost_count, i.right_lost_count,
                      i.both_lost_count, i.bottom - i.top);
        body += line;

        std::snprintf(line, sizeof(line),
                      "errors line=%.6f far=%.6f vehicle=%.6f conf=%.4f "
                      "lost=%d pair_valid=%d\n",
                      r.line_error, r.far_error, r.vehicle_center_error,
                      r.line_confidence, r.line_lost ? 1 : 0,
                      r.bottom_pair_valid ? 1 : 0);
        body += line;

        std::snprintf(line, sizeof(line),
                      "cm valid=%d per_col_control=%.4f per_col_far=%.4f "
                      "line_cm=%.2f far_cm=%.2f vehicle_cm=%.2f\n",
                      r.cm_scale_valid ? 1 : 0, r.cm_per_col_control,
                      r.cm_per_col_far, r.line_error_cm, r.far_error_cm,
                      r.vehicle_center_error_cm);
        body += line;

        std::snprintf(line, sizeof(line),
                      "misc threshold=%d blue_pixels=%d cross=%d zebra=%d "
                      "L_branch=%d R_branch=%d\n",
                      r.vision_threshold, r.vision_blue_mask_pixels,
                      r.elements.cross ? 1 : 0, r.elements.zebra ? 1 : 0,
                      r.elements.left_branch_count,
                      r.elements.right_branch_count);
        body += line;

        const TopologyReport& t = r.topology;
        std::snprintf(line, sizeof(line),
                      "topology valid=%d track_area=%d far=%d near=%d "
                      "seed=%d,%d L(border=%d area=%d) R(border=%d area=%d)\n",
                      t.valid ? 1 : 0, t.track_area, t.track_far_row,
                      t.track_near_row, t.seed_col, t.seed_row,
                      t.left.border_white_rows, t.left.area,
                      t.right.border_white_rows, t.right.area);
        body += line;

        // 二值网格：'#' 白（赛道），'.' 黑。row 0 = 远，row 59 = 车头。
        body += "GRID binary (# = white)\n";
        for (int y = 0; y < kBinaryHeight; ++y) {
            line[0] = static_cast<char>('0' + (y / 10));
            line[1] = static_cast<char>('0' + (y % 10));
            line[2] = ' ';
            body.append(line, 3);
            for (int x = 0; x < kBinaryWidth; ++x) {
                body += snap.vision_frame.binary[y][x] ? '#' : '.';
            }
            body += '\n';
        }

        // 灰度网格：二值化之前的原始亮度，两位十六进制。用来判断阈值是否合适。
        body += "GRID gray (hex)\n";
        static const char kHex[] = "0123456789abcdef";
        for (int y = 0; y < kBinaryHeight; ++y) {
            line[0] = static_cast<char>('0' + (y / 10));
            line[1] = static_cast<char>('0' + (y % 10));
            line[2] = ' ';
            body.append(line, 3);
            for (int x = 0; x < kBinaryWidth; ++x) {
                const std::uint8_t g = snap.vision_frame.gray[y][x];
                body += kHex[g >> 4];
                body += kHex[g & 0x0F];
            }
            body += '\n';
        }

        body += "EDGES row left lv right rv width mid\n";
        for (int y = 0; y < kBinaryHeight; ++y) {
            std::snprintf(line, sizeof(line), "%2d %3d %d %3d %d %3d %3d\n",
                          y, r.left[y], r.left_valid[y] ? 1 : 0, r.right[y],
                          r.right_valid[y] ? 1 : 0, r.width[y], r.mid[y]);
            body += line;
        }
    }

    char header[256];
    const int hn = std::snprintf(
        header, sizeof(header),
        "HTTP/1.1 200 OK\r\nContent-Type: text/plain; charset=utf-8\r\n"
        "Content-Length: %zu\r\nCache-Control: no-store\r\n"
        "Connection: close\r\n\r\n",
        body.size());
    write_all(fd, header, static_cast<std::size_t>(hn));
    write_all(fd, body.data(), body.size());
}

void HttpMjpegStreamer::send_json(int fd,
                                  const std::string& body,
                                  const char* status) {
    char header[256];
    const int hn = std::snprintf(
        header, sizeof(header),
        "HTTP/1.1 %s\r\nContent-Type: application/json; charset=utf-8\r\n"
        "Content-Length: %zu\r\nCache-Control: no-store\r\n"
        "Connection: close\r\n\r\n",
        status, body.size());
    write_all(fd, header, static_cast<std::size_t>(hn));
    write_all(fd, body.data(), body.size());
}

void HttpMjpegStreamer::send_vision_params(int fd) {
    if (!vision_) {
        send_json(fd, "{\"error\":\"vision tuning unavailable\"}",
                  "503 Service Unavailable");
        return;
    }
    const VisionTuningParams tuning = vision_->vision_tuning();
    char body[640];
    const int n = std::snprintf(
        body, sizeof(body),
        "{\"threshold_floor\":%d,\"color_filter_enabled\":%s,"
        "\"saturation_penalty\":%d,\"blue_reject_enabled\":%s,"
        "\"blue_hue_low\":%d,\"blue_hue_high\":%d,"
        "\"blue_saturation_min\":%d,\"blue_value_min\":%d,"
        "\"blue_penalty\":%d}",
        tuning.threshold_floor,
        tuning.color_filter_enabled ? "true" : "false",
        tuning.saturation_penalty,
        tuning.blue_reject_enabled ? "true" : "false",
        tuning.blue_hue_low,
        tuning.blue_hue_high,
        tuning.blue_saturation_min,
        tuning.blue_value_min,
        tuning.blue_penalty);
    send_json(fd, std::string(body, static_cast<std::size_t>(n)));
}

void HttpMjpegStreamer::update_vision_params(
    int fd, const std::string& body) {
    if (!vision_) {
        send_json(fd, "{\"error\":\"vision tuning unavailable\"}",
                  "503 Service Unavailable");
        return;
    }
    VisionTuningParams tuning = vision_->vision_tuning();
    bool changed = false;
    std::size_t begin = 0;
    while (begin <= body.size()) {
        const std::size_t end = body.find('&', begin);
        const std::string item = body.substr(
            begin, end == std::string::npos ? std::string::npos : end - begin);
        const std::size_t equals = item.find('=');
        if (equals != std::string::npos) {
            const std::string key = item.substr(0, equals);
            const std::string text = item.substr(equals + 1);
            char* value_end = nullptr;
            const long value = std::strtol(text.c_str(), &value_end, 10);
            if (value_end && *value_end == '\0') {
                if (key == "threshold_floor") {
                    tuning.threshold_floor = static_cast<int>(value);
                } else if (key == "color_filter_enabled") {
                    tuning.color_filter_enabled = value != 0;
                } else if (key == "saturation_penalty") {
                    tuning.saturation_penalty = static_cast<int>(value);
                } else if (key == "blue_reject_enabled") {
                    tuning.blue_reject_enabled = value != 0;
                } else if (key == "blue_hue_low") {
                    tuning.blue_hue_low = static_cast<int>(value);
                } else if (key == "blue_hue_high") {
                    tuning.blue_hue_high = static_cast<int>(value);
                } else if (key == "blue_saturation_min") {
                    tuning.blue_saturation_min = static_cast<int>(value);
                } else if (key == "blue_value_min") {
                    tuning.blue_value_min = static_cast<int>(value);
                } else if (key == "blue_penalty") {
                    tuning.blue_penalty = static_cast<int>(value);
                } else {
                    begin = end == std::string::npos ? body.size() + 1 : end + 1;
                    continue;
                }
                changed = true;
            }
        }
        if (end == std::string::npos) break;
        begin = end + 1;
    }
    if (!changed) {
        send_json(fd, "{\"error\":\"no valid vision parameters\"}",
                  "400 Bad Request");
        return;
    }
    vision_->set_vision_tuning(tuning);
    send_vision_params(fd);
}

void HttpMjpegStreamer::send_wheel_box(int fd) {
    if (!vision_) {
        send_json(fd, "{\"error\":\"wheel box unavailable\"}",
                  "503 Service Unavailable");
        return;
    }
    const WheelMaskBox box = vision_->wheel_mask_box();
    char body[384];
    const int n = std::snprintf(
        body, sizeof(body),
        "{\"left\":%.6f,\"top\":%.6f,\"right\":%.6f,"
        "\"bottom\":%.6f,\"left_px\":%d,\"top_px\":%d,"
        "\"right_px\":%d,\"bottom_px\":%d,\"initialized\":%s,"
        "\"source\":\"%s\"}",
        static_cast<double>(box.left) / (kBinaryWidth - 1),
        static_cast<double>(box.top) / (kBinaryHeight - 1),
        static_cast<double>(box.right) / (kBinaryWidth - 1),
        static_cast<double>(box.bottom) / (kBinaryHeight - 1),
        box.left, box.top, box.right, box.bottom,
        box.initialized ? "true" : "false",
        box.auto_detected ? "auto" : "manual_or_fallback");
    send_json(fd, std::string(body, static_cast<std::size_t>(n)));
}

void HttpMjpegStreamer::update_wheel_box(
    int fd, const std::string& body) {
    if (!vision_) {
        send_json(fd, "{\"error\":\"wheel box unavailable\"}",
                  "503 Service Unavailable");
        return;
    }
    double left = -1.0;
    double top = -1.0;
    double right = -1.0;
    double bottom = -1.0;
    std::size_t begin = 0;
    while (begin <= body.size()) {
        const std::size_t end = body.find('&', begin);
        const std::string item = body.substr(
            begin, end == std::string::npos ? std::string::npos : end - begin);
        const std::size_t equals = item.find('=');
        if (equals != std::string::npos) {
            const std::string key = item.substr(0, equals);
            const std::string text = item.substr(equals + 1);
            char* value_end = nullptr;
            const double value = std::strtod(text.c_str(), &value_end);
            if (value_end && *value_end == '\0') {
                if (key == "left") left = value;
                else if (key == "top") top = value;
                else if (key == "right") right = value;
                else if (key == "bottom") bottom = value;
            }
        }
        if (end == std::string::npos) break;
        begin = end + 1;
    }
    if (!vision_->set_wheel_mask_box(left, top, right, bottom)) {
        send_json(fd,
                  "{\"error\":\"wheel box requires ordered 0..1 ratios\"}",
                  "400 Bad Request");
        return;
    }
    send_wheel_box(fd);
}

void HttpMjpegStreamer::stream_client(int fd, StreamView view) {
    const char header[] =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: close\r\n\r\n";
    if (!write_all(fd, header, sizeof(header) - 1)) return;

    const std::size_t view_index = static_cast<std::size_t>(view);
    stream_clients_.fetch_add(1);
    stream_view_clients_[view_index].fetch_add(1);
    snapshot_cv_.notify_one();
    std::uint64_t sent_seq = 0;
    while (running_.load()) {
        std::vector<unsigned char> jpeg;
        {
            std::unique_lock<std::mutex> lock(jpeg_mutex_);
            jpeg_cv_.wait_for(lock, std::chrono::milliseconds(500), [&] {
                return !running_.load() ||
                       (latest_jpeg_seqs_[view_index] != sent_seq &&
                        !latest_jpegs_[view_index].empty());
            });
            if (!running_.load()) break;
            if (latest_jpeg_seqs_[view_index] == sent_seq ||
                latest_jpegs_[view_index].empty()) {
                continue;
            }
            sent_seq = latest_jpeg_seqs_[view_index];
            jpeg = latest_jpegs_[view_index];
        }
        if (!send_jpeg_part(fd, jpeg)) break;
    }
    stream_view_clients_[view_index].fetch_sub(1);
    stream_clients_.fetch_sub(1);
}

void HttpMjpegStreamer::telemetry_client(int fd) {
    const char header[] =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: application/x-ndjson; charset=utf-8\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: close\r\n\r\n";
    if (!write_all(fd, header, sizeof(header) - 1)) return;

    telemetry_clients_.fetch_add(1);
    std::uint64_t sent_seq = 0;
    while (running_.load()) {
        std::string line;
        {
            std::unique_lock<std::mutex> lock(telemetry_mutex_);
            telemetry_cv_.wait_for(lock, std::chrono::milliseconds(1000), [&] {
                return !running_.load() ||
                    (telemetry_seq_ != sent_seq && !latest_telemetry_.empty());
            });
            if (!running_.load()) break;
            if (telemetry_seq_ == sent_seq || latest_telemetry_.empty()) continue;
            sent_seq = telemetry_seq_;
            line = latest_telemetry_;
        }
        line.push_back('\n');
        if (!write_all(fd, line.data(), line.size())) break;
    }
    telemetry_clients_.fetch_sub(1);
}

std::string HttpMjpegStreamer::telemetry_json(
    const TelemetrySample& sample) const {
    char body[16384];
    const NavigationCommand& nav = sample.navigation;
    const ControlDiagnostics& diag = sample.control.diagnostics;
    const MotorFeedbackLite& motor = sample.motor;
    const ImuFeedback& imu = sample.imu;
    const TofSlopeFeedback& tof = sample.tof;
    const OdometrySample& odom = sample.odometry;
    const InertialNavigationStatus& inertial =
        sample.inertial_navigation;
    const double clamped_vision_yaw = std::max(
        -p_.max_yaw_rate_dps,
        std::min(p_.max_yaw_rate_dps,
                 nav.vision_unclamped_yaw_rate_dps));
    const double state_yaw_adjustment =
        diag.requested_yaw_rate_dps - clamped_vision_yaw;
    FeatureSide control_side_open = sample.road.elements.side_open;
    if (p_.vision_yaw_sign < 0.0) {
        if (control_side_open == FeatureSide::Left) {
            control_side_open = FeatureSide::Right;
        } else if (control_side_open == FeatureSide::Right) {
            control_side_open = FeatureSide::Left;
        }
    }
    const int n = std::snprintf(
        body, sizeof(body),
        "{\"elapsed_s\":%.6f,"
        "\"state\":\"%s\",\"side_phase\":\"%s\",\"stop_reason\":\"%s\","
        "\"action\":\"%s\","
        "\"target_recognition_enabled\":%s,"
        "\"target_recognition_valid\":%s,"
        "\"target_recognition_kind\":\"%s\","
        "\"target_recognition_confidence\":%.6f,"
        "\"target_recognition_size\":%.6f,"
        "\"target_recognition_center_y_ratio\":%.6f,"
        "\"target_recognition_x\":%d,\"target_recognition_y\":%d,"
        "\"target_recognition_w\":%d,\"target_recognition_h\":%d,"
        "\"recognition_valid\":%s,\"recognition\":\"%s\","
        "\"recognition_confidence\":%.6f,"
        "\"recognition_size\":%.6f,"
        "\"recognition_center_y_ratio\":%.6f,"
        "\"target_encounter\":%d,\"target_route_active\":%s,"
        "\"sensor_mode\":\"%s\","
        "\"line_error\":%.6f,\"far_error\":%.6f,"
        "\"vehicle_center_error\":%.6f,"
        "\"bottom_pair_valid\":%s,"
        "\"vision_yaw_sign\":%.0f,"
        "\"control_line_error\":%.6f,\"control_far_error\":%.6f,"
        "\"line_confidence\":%.6f,"
        "\"line_lost\":%s,\"cross\":%s,\"zebra\":%s,\"side_open\":\"%s\","
        "\"control_side_open\":\"%s\","
        "\"roundabout_stage\":\"%s\",\"roundabout_side\":\"%s\","
        "\"stable_side\":\"%s\","
        "\"left_branches\":%d,\"right_branches\":%d,"
        "\"left_recoveries\":%d,\"right_recoveries\":%d,"
        "\"left_prediction_stable\":%s,\"right_prediction_stable\":%s,"
        "\"prediction_symmetric\":%s,"
        "\"left_prediction_mean_error\":%.6f,"
        "\"right_prediction_mean_error\":%.6f,"
        "\"left_prediction_max_error\":%.6f,"
        "\"right_prediction_max_error\":%.6f,"
        "\"round_track_row\":%d,\"round_recovery_row\":%d,"
        "\"two_side_stable\":%s,\"cross_score\":%d,"
        "\"zebra_active\":%s,\"zebra_encounter\":%d,\"round_score\":%d,"
        "\"topo_valid\":%s,\"topo_track_area\":%d,"
        "\"topo_track_far_row\":%d,\"topo_track_near_row\":%d,"
        "\"topo_seed_col\":%d,\"topo_seed_row\":%d,"
        "\"topo_left_border_rows\":%d,\"topo_left_border_is_track\":%s,"
        "\"topo_left_area\":%d,\"topo_left_far_row\":%d,"
        "\"topo_left_max_col\":%d,"
        "\"topo_right_border_rows\":%d,\"topo_right_border_is_track\":%s,"
        "\"topo_right_area\":%d,\"topo_right_far_row\":%d,"
        "\"topo_right_max_col\":%d,"
        "\"cm_scale_valid\":%s,\"cm_per_col_control\":%.6f,"
        "\"cm_per_col_far\":%.6f,\"line_error_cm\":%.6f,"
        "\"far_error_cm\":%.6f,\"vehicle_center_error_cm\":%.6f,"
        "\"vision_integral_error_s\":%.6f,"
        "\"vision_integral_yaw_rate_dps\":%.6f,"
        "\"vision_derivative_error_per_s\":%.6f,"
        "\"vision_derivative_yaw_rate_dps\":%.6f,"
        "\"vision_near_yaw_rate_dps\":%.6f,"
        "\"vision_far_yaw_rate_dps\":%.6f,"
        "\"vision_center_yaw_rate_dps\":%.6f,"
        "\"vision_curve_gain\":%.6f,"
        "\"vision_unclamped_yaw_rate_dps\":%.6f,"
        "\"state_yaw_adjustment_dps\":%.6f,"
        "\"side_require_tof\":%s,\"side_entry_gate_ready\":%s,"
        "\"target_speed_cmps\":%.6f,\"limited_speed_cmps\":%.6f,"
        "\"target_yaw_rate_dps\":%.6f,\"measured_yaw_rate_dps\":%.6f,"
        "\"inertial_direction_guard_active\":%s,"
        "\"inertial_takeover_yaw_rate_dps\":%.6f,"
        "\"raw_encoder_yaw_rate_dps\":%.6f,"
        "\"filtered_encoder_yaw_rate_dps\":%.6f,"
        "\"yaw_sensor_disagreement_frames\":%d,"
        "\"imu_yaw_rejected\":%s,\"yaw_direction_guard_active\":%s,"
        "\"heading_hold\":%s,\"target_heading_deg\":%.6f,"
        "\"power_boost_percent\":%.6f,"
        "\"imu_valid\":%s,\"imu_age_s\":%.6f,\"imu_heading_deg\":%.6f,"
        "\"imu_stationary\":%s,"
        "\"imu_raw_accel_x_g\":%.6f,\"imu_raw_accel_y_g\":%.6f,"
        "\"imu_raw_accel_z_g\":%.6f,"
        "\"imu_forward_accel_mps2\":%.6f,"
        "\"imu_right_accel_mps2\":%.6f,"
        "\"imu_velocity_x_mps\":%.6f,\"imu_velocity_y_mps\":%.6f,"
        "\"imu_position_x_m\":%.6f,\"imu_position_y_m\":%.6f,"
        "\"tof_started\":%s,\"tof_valid\":%s,\"tof_age_s\":%.6f,"
        "\"tof_distance_mm\":%.6f,\"tof_filtered_mm\":%.6f,"
        "\"tof_baseline_mm\":%.6f,\"tof_delta_mm\":%.6f,"
        "\"tof_baseline_ready\":%s,\"ramp_detected\":%s,"
        "\"left_encoder_valid\":%s,\"right_encoder_valid\":%s,"
        "\"left_rpm\":%.6f,\"right_rpm\":%.6f,"
        "\"left_speed_cmps\":%.6f,\"right_speed_cmps\":%.6f,"
        "\"left_distance_cm\":%.6f,\"right_distance_cm\":%.6f,"
        "\"encoder_heading_deg\":%.6f,"
        "\"left_target_cmps\":%.6f,\"right_target_cmps\":%.6f,"
        "\"target_steering_utilization\":%.6f,"
        "\"pwm_steering_utilization\":%.6f,"
        "\"left_pwm\":%.6f,\"right_pwm\":%.6f,\"saturated\":%s,"
        "\"map_y_positive\":\"right\","
        "\"x_cm\":%.6f,\"y_cm\":%.6f,\"heading_deg\":%.6f,"
        "\"distance_cm\":%.6f,\"odometry_valid\":%s,"
        "\"inertial_state\":\"%s\","
        "\"inertial_waypoint_index\":%zu,"
        "\"inertial_waypoint_count\":%zu,"
        "\"inertial_takeover_reanchored\":%s,"
        "\"inertial_progress_cm\":%.6f,"
        "\"inertial_path_length_cm\":%.6f,"
        "\"inertial_cross_track_error_cm\":%.6f,"
        "\"inertial_heading_error_deg\":%.6f,"
        "\"inertial_target_x_cm\":%.6f,"
        "\"inertial_target_y_cm\":%.6f}",
        sample.elapsed_s,
        drive_state_name(nav.state),
        nav.state == DriveState::Roundabout ? "ARC" :
            (nav.state == DriveState::RoundaboutExit ?
                "REACQUIRE" : "NONE"),
        stop_reason_name(nav.stop_reason),
        target_kind_name(nav.action_kind),
        sample.target_recognition_enabled ? "true" : "false",
        sample.target_recognition_valid ? "true" : "false",
        target_kind_name(sample.target_recognition_kind),
        sample.target_recognition_confidence,
        sample.target_recognition_size,
        sample.target_recognition_center_y_ratio,
        sample.target_recognition_x,
        sample.target_recognition_y,
        sample.target_recognition_w,
        sample.target_recognition_h,
        sample.recognition_valid ? "true" : "false",
        target_kind_name(sample.recognition_kind),
        sample.recognition_confidence,
        sample.recognition_size,
        sample.recognition_center_y_ratio,
        sample.target_encounter,
        sample.target_route_active ? "true" : "false",
        sensor_mode_name(diag.sensor_mode),
        sample.road.line_error,
        sample.road.far_error,
        sample.road.vehicle_center_error,
        sample.road.bottom_pair_valid ? "true" : "false",
        p_.vision_yaw_sign,
        p_.vision_yaw_sign * sample.road.line_error,
        p_.vision_yaw_sign * sample.road.far_error,
        sample.road.line_confidence,
        sample.road.line_lost ? "true" : "false",
        sample.road.elements.cross ? "true" : "false",
        sample.road.elements.zebra ? "true" : "false",
        feature_side_name(sample.road.elements.side_open),
        feature_side_name(control_side_open),
        roundabout_stage_name(sample.road.elements.roundabout_stage),
        feature_side_name(sample.road.elements.roundabout),
        feature_side_name(sample.road.elements.stable_side),
        sample.road.elements.left_branch_count,
        sample.road.elements.right_branch_count,
        sample.road.elements.left_recovery_count,
        sample.road.elements.right_recovery_count,
        sample.road.elements.left_prediction_stable ? "true" : "false",
        sample.road.elements.right_prediction_stable ? "true" : "false",
        sample.road.elements.roundabout_prediction_symmetric ? "true" : "false",
        sample.road.elements.left_prediction_mean_error,
        sample.road.elements.right_prediction_mean_error,
        sample.road.elements.left_prediction_max_error,
        sample.road.elements.right_prediction_max_error,
        sample.road.roundabout_track_point.valid
            ? sample.road.roundabout_track_point.row : -1,
        sample.road.roundabout_recovery_point.valid
            ? sample.road.roundabout_recovery_point.row : -1,
        sample.road.elements.two_side_stable ? "true" : "false",
        nav.cross_score,
        nav.zebra_active ? "true" : "false",
        nav.zebra_encounter_count,
        sample.road.elements.roundabout != FeatureSide::None &&
        sample.road.elements.roundabout_stage !=
            RoundaboutVisionStage::None &&
        sample.road.elements.roundabout_stage !=
            RoundaboutVisionStage::Approach
                ? p_.round_enter_frames : 0,
        sample.road.topology.valid ? "true" : "false",
        sample.road.topology.track_area,
        sample.road.topology.track_far_row,
        sample.road.topology.track_near_row,
        sample.road.topology.seed_col,
        sample.road.topology.seed_row,
        sample.road.topology.left.border_white_rows,
        sample.road.topology.left.border_is_track ? "true" : "false",
        sample.road.topology.left.area,
        sample.road.topology.left.far_row,
        sample.road.topology.left.max_col,
        sample.road.topology.right.border_white_rows,
        sample.road.topology.right.border_is_track ? "true" : "false",
        sample.road.topology.right.area,
        sample.road.topology.right.far_row,
        sample.road.topology.right.max_col,
        sample.road.cm_scale_valid ? "true" : "false",
        sample.road.cm_per_col_control,
        sample.road.cm_per_col_far,
        sample.road.line_error_cm,
        sample.road.far_error_cm,
        sample.road.vehicle_center_error_cm,
        nav.vision_integral_error_s,
        nav.vision_integral_yaw_rate_dps,
        nav.vision_derivative_error_per_s,
        nav.vision_derivative_yaw_rate_dps,
        nav.vision_near_yaw_rate_dps,
        nav.vision_far_yaw_rate_dps,
        nav.vision_center_yaw_rate_dps,
        nav.vision_curve_gain,
        nav.vision_unclamped_yaw_rate_dps,
        state_yaw_adjustment,
        p_.side_require_tof ? "true" : "false",
        !sample.tof.ramp_detected &&
        (!p_.side_require_tof ||
         (sample.tof.valid && sample.tof.baseline_ready))
            ? "true" : "false",
        nav.target_speed_cmps,
        diag.limited_speed_cmps,
        diag.requested_yaw_rate_dps,
        diag.measured_yaw_rate_dps,
        nav.inertial_direction_guard_active ? "true" : "false",
        nav.inertial_takeover_yaw_rate_dps,
        diag.raw_encoder_yaw_rate_dps,
        diag.filtered_encoder_yaw_rate_dps,
        diag.yaw_sensor_disagreement_frames,
        diag.imu_yaw_rejected ? "true" : "false",
        diag.yaw_direction_guard_active ? "true" : "false",
        nav.heading_hold ? "true" : "false",
        nav.target_heading_deg,
        diag.power_boost_percent,
        imu.valid ? "true" : "false",
        imu.age_s,
        imu.heading_deg,
        imu.stationary ? "true" : "false",
        imu.raw_accel_x_g,
        imu.raw_accel_y_g,
        imu.raw_accel_z_g,
        imu.forward_accel_mps2,
        imu.right_accel_mps2,
        imu.velocity_x_mps,
        imu.velocity_y_mps,
        imu.position_x_m,
        imu.position_y_m,
        tof.sensor_started ? "true" : "false",
        tof.valid ? "true" : "false",
        tof.age_s,
        tof.distance_mm,
        tof.filtered_distance_mm,
        tof.baseline_distance_mm,
        tof.signed_delta_mm,
        tof.baseline_ready ? "true" : "false",
        tof.ramp_detected ? "true" : "false",
        motor.left_valid ? "true" : "false",
        motor.right_valid ? "true" : "false",
        motor.left_rpm,
        motor.right_rpm,
        motor.left_speed_cmps,
        motor.right_speed_cmps,
        motor.left_distance_cm,
        motor.right_distance_cm,
        (motor.left_distance_cm - motor.right_distance_cm) /
            std::max(1.0, p_.wheel_base_cm) * 57.2957795130823208768,
        diag.left_target_cmps,
        diag.right_target_cmps,
        diag.target_steering_utilization,
        diag.pwm_steering_utilization,
        sample.control.command.left_percent,
        sample.control.command.right_percent,
        diag.saturated ? "true" : "false",
        odom.x_cm,
        odom.y_cm,
        odom.heading_deg,
        odom.distance_cm,
        odom.valid ? "true" : "false",
        inertial_nav_state_name(inertial.state),
        inertial.waypoint_index,
        inertial.waypoint_count,
        inertial.takeover_reanchored ? "true" : "false",
        inertial.progress_cm,
        inertial.path_length_cm,
        inertial.cross_track_error_cm,
        inertial.heading_error_deg,
        inertial.target_x_cm,
        inertial.target_y_cm);
    return n > 0 ? std::string(body, static_cast<std::size_t>(
        std::min(n, static_cast<int>(sizeof(body) - 1)))) : std::string("{}");
}

bool HttpMjpegStreamer::send_jpeg_part(
    int fd,
    const std::vector<unsigned char>& jpeg) {
    char part[160];
    const int n = std::snprintf(
        part, sizeof(part),
        "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n",
        jpeg.size());
    return write_all(fd, part, static_cast<std::size_t>(n)) &&
           write_all(fd, jpeg.data(), jpeg.size()) &&
           write_all(fd, "\r\n", 2);
}

bool HttpMjpegStreamer::write_all(int fd, const void* data, std::size_t size) {
    const char* p = static_cast<const char*>(data);
    while (size > 0) {
        ssize_t n = ::send(fd, p, size, MSG_NOSIGNAL);
        if (n < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        if (n == 0) return false;
        p += n;
        size -= static_cast<std::size_t>(n);
    }
    return true;
}

}  // namespace rewrite_path

#else

namespace rewrite_path {

HttpMjpegStreamer::HttpMjpegStreamer(const PathParams& params,
                                     LegacyVisionPipeline*) : p_(params) {}
HttpMjpegStreamer::~HttpMjpegStreamer() = default;
bool HttpMjpegStreamer::start() { return false; }
void HttpMjpegStreamer::stop() {}
void HttpMjpegStreamer::publish_telemetry(const TelemetrySample&) {}

}  // namespace rewrite_path

#endif
