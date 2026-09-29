// 板上单帧视觉探针。
//
// 用途：把一张静态图片喂进 LegacyVisionPipeline，把算法内部状态（94x60 二值网格、
// 逐行左右边缘、中线、最长白列、轮子框、cm 换算、拓扑）全部 dump 出来，并输出
// 与板端 /stream/overlay、/stream/path 完全相同的可视化图。
//
// 这样场地外的任何一张照片都能"原地复现"识别结果，不用反复移车看数字。
//
// 设计要点：
//   · 复用板端同一份 vision_pipeline.cpp，结果与真实运行完全一致。
//   · 同一帧重复喂 settle 次，让 last_mid / last_error 这类跨帧状态收敛到
//     "车静止时"的稳态，否则第一帧的结果不代表实际行为。
//   · 交叉编译到 LoongArch 后在板上跑（板上才有匹配的 OpenCV）。
//
// Usage:
//   frame_probe IMAGE [options]
//
// Options:
//   --rotate                 输入图按摄像头原始方向，先旋转 180（与 --rotate-180 一致）
//   --threshold-floor N      二值化下限（默认取 PathParams 默认值）
//   --calibration FILE       标定表路径
//   --control-cm F           控制行距离厘米（默认 60）
//   --far-cm F               远端行距离厘米（默认 120）
//   --track-width-cm F       物理赛道宽度厘米（配合 --cm-error）
//   --cm-error               打开归一化误差 -> 厘米换算
//   --topology               打开 BOOM 拓扑观测层
//   --no-wheel-mask          关闭轮子遮罩
//   --settle N               重复喂同一帧的次数（默认 8）
//   --out PREFIX             输出文件前缀（默认 probe）
//   --no-images              不写 PNG
//   --grid                   打印 94x60 二值网格（含行/列标尺）
//   --edges                  打印逐行 left/right/mid/width
//   --quiet                  只打印摘要

#include "path_params.hpp"
#include "vision_pipeline.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

using namespace rewrite_path;

namespace {

void print_grid(const VisionFrame& f) {
    std::printf("\nBINARY GRID 94x60  (row 0 = far, row 59 = car front)\n");
    std::printf("     ");
    for (int x = 0; x < kBinaryWidth; ++x) {
        std::putchar((x % 10 == 0) ? ('0' + ((x / 10) % 10)) : ' ');
    }
    std::putchar('\n');
    std::printf("     ");
    for (int x = 0; x < kBinaryWidth; ++x) {
        std::putchar('0' + (x % 10));
    }
    std::putchar('\n');
    for (int y = 0; y < kBinaryHeight; ++y) {
        std::printf("%3d  ", y);
        for (int x = 0; x < kBinaryWidth; ++x) {
            std::putchar(f.binary[y][x] ? '#' : '.');
        }
        std::putchar('\n');
    }
}

void print_row(const VisionFrame& f, int row, const char* label) {
    if (row < 0 || row >= kBinaryHeight) return;
    std::printf("\n%s row %d:\n     ", label, row);
    for (int x = 0; x < kBinaryWidth; ++x) {
        std::putchar(f.binary[row][x] ? '#' : '.');
    }
    std::putchar('\n');
    std::printf("     white runs:");
    int start = -1;
    for (int x = 0; x < kBinaryWidth; ++x) {
        const bool w = f.binary[row][x] != 0;
        if (w && start < 0) start = x;
        if (!w && start >= 0) {
            std::printf(" [%d..%d](%d)", start, x - 1, x - start);
            start = -1;
        }
    }
    if (start >= 0) {
        std::printf(" [%d..%d](%d)", start, kBinaryWidth - 1,
                    kBinaryWidth - start);
    }
    std::putchar('\n');
}

void print_edges(const RoadEstimateLite& r) {
    std::printf("\nROW  left  lv  right  rv  width   mid    note\n");
    for (int y = 0; y < kBinaryHeight; ++y) {
        const bool lv = r.left_valid[y] != 0;
        const bool rv = r.right_valid[y] != 0;
        if (!lv && !rv && r.width[y] == 0) continue;
        const char* note = "";
        if (lv && !rv) note = "left-only";
        else if (!lv && rv) note = "right-only";
        std::printf("%3d  %4d   %d   %4d   %d   %4d  %4d    %s\n",
                    y, r.left[y], lv ? 1 : 0, r.right[y], rv ? 1 : 0,
                    r.width[y], r.mid[y], note);
    }
}

void print_columns(const VisionFrame& f) {
    std::printf("\ncolumn white-pixel counts (>= 6 shown):\n");
    for (int x = 0; x < kBinaryWidth; ++x) {
        int n = 0;
        for (int y = 0; y < kBinaryHeight; ++y) {
            if (f.binary[y][x]) ++n;
        }
        if (n >= 6) {
            std::printf("   col %2d : %2d\n", x, n);
        }
    }
}

void print_summary(const RoadEstimateLite& r, const WheelMaskBox& box,
                   int settle) {
    const RoadImageInfo& i = r.info;
    std::printf("\n=== summary after %d settle frames ===\n", settle);
    std::printf("wheel box      : x=%d..%d  y=%d..%d  (%s)\n",
                box.left, box.right, box.top, box.bottom,
                box.auto_detected ? "auto" : "configured");
    std::printf("longest-white  : search_row=%d  seed_col(last_mid)=%d  "
                "max_column=%d  white_num=%d\n",
                box.top - 1, i.last_mid, i.max_column, i.white_num);
    std::printf("row window     : top=%d  bottom=%d  control_row=%d  "
                "far_row=%d  vehicle_anchor_row=%d\n",
                i.top, i.bottom, i.control_row, i.far_row,
                i.vehicle_anchor_row);
    std::printf("edge losses    : left=%d  right=%d  both=%d  "
                "(scan rows=%d)\n",
                i.left_lost_count, i.right_lost_count, i.both_lost_count,
                i.bottom - i.top);
    std::printf("errors         : line_error=%.4f  far_error=%.4f  "
                "vehicle_center_error=%.4f\n",
                r.line_error, r.far_error, r.vehicle_center_error);
    std::printf("quality        : confidence=%.3f  line_lost=%d  "
                "bottom_pair_valid=%d\n",
                r.line_confidence, r.line_lost ? 1 : 0,
                r.bottom_pair_valid ? 1 : 0);
    std::printf("threshold      : %d  blue_mask_pixels=%d\n",
                r.vision_threshold, r.vision_blue_mask_pixels);
    std::printf("cm scale       : valid=%d  per_col_control=%.4f  "
                "per_col_far=%.4f\n",
                r.cm_scale_valid ? 1 : 0, r.cm_per_col_control,
                r.cm_per_col_far);
    std::printf("cm errors      : line=%.2f  far=%.2f  vehicle=%.2f\n",
                r.line_error_cm, r.far_error_cm, r.vehicle_center_error_cm);
    std::printf("elements       : cross=%d  zebra=%d  side_open=%d  "
                "L_branch=%d  R_branch=%d\n",
                r.elements.cross ? 1 : 0, r.elements.zebra ? 1 : 0,
                static_cast<int>(r.elements.side_open),
                r.elements.left_branch_count, r.elements.right_branch_count);
    const TopologyReport& t = r.topology;
    std::printf("topology       : valid=%d  track_area=%d  far=%d near=%d  "
                "seed=(%d,%d)\n",
                t.valid ? 1 : 0, t.track_area, t.track_far_row,
                t.track_near_row, t.seed_col, t.seed_row);
    std::printf("topology L/R   : L border_rows=%d area=%d is_track=%d | "
                "R border_rows=%d area=%d is_track=%d\n",
                t.left.border_white_rows, t.left.area,
                t.left.border_is_track ? 1 : 0, t.right.border_white_rows,
                t.right.area, t.right.border_is_track ? 1 : 0);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr,
                     "Usage: %s IMAGE [--rotate] [--threshold-floor N] "
                     "[--calibration FILE]\n"
                     "       [--control-cm F] [--far-cm F] [--track-width-cm F]\n"
                     "       [--cm-error] [--topology] [--no-wheel-mask]\n"
                     "       [--settle N] [--out PREFIX] [--no-images]\n"
                     "       [--grid] [--edges] [--quiet]\n",
                     argv[0]);
        return 2;
    }

    const std::string image_path = argv[1];
    PathParams p;
    p.rotate_180 = false;
    p.use_http = false;
    p.dry_run = true;
    p.calibration_path = "/home/root/rewrite/calib.txt";

    int settle = 8;
    std::string out_prefix = "probe";
    bool write_images = true;
    bool show_grid = false;
    bool show_edges = false;
    bool show_columns = false;
    bool quiet = false;

    for (int a = 2; a < argc; ++a) {
        const std::string arg = argv[a];
        const char* next = (a + 1 < argc) ? argv[a + 1] : nullptr;
        auto need = [&](const char* what) -> bool {
            if (!next) {
                std::fprintf(stderr, "%s needs a value\n", what);
                return false;
            }
            ++a;
            return true;
        };
        if (arg == "--rotate" || arg == "--rotate-180") {
            p.rotate_180 = true;
        } else if (arg == "--no-rotate") {
            p.rotate_180 = false;
        } else if (arg == "--threshold-floor") {
            if (!need("--threshold-floor")) return 2;
            p.threshold_floor = std::atoi(next);
        } else if (arg == "--calibration") {
            if (!need("--calibration")) return 2;
            p.calibration_path = next;
        } else if (arg == "--control-cm") {
            if (!need("--control-cm")) return 2;
            p.control_distance_cm = std::atof(next);
        } else if (arg == "--far-cm") {
            if (!need("--far-cm")) return 2;
            p.far_distance_cm = std::atof(next);
        } else if (arg == "--track-width-cm") {
            if (!need("--track-width-cm")) return 2;
            p.track_width_cm = std::atof(next);
        } else if (arg == "--cm-error") {
            p.cm_error_enable = true;
        } else if (arg == "--topology") {
            p.enable_topology = true;
        } else if (arg == "--no-wheel-mask") {
            p.wheel_mask_enable = false;
        } else if (arg == "--settle") {
            if (!need("--settle")) return 2;
            settle = std::atoi(next);
            if (settle < 1) settle = 1;
        } else if (arg == "--out") {
            if (!need("--out")) return 2;
            out_prefix = next;
        } else if (arg == "--no-images") {
            write_images = false;
        } else if (arg == "--grid") {
            show_grid = true;
        } else if (arg == "--edges") {
            show_edges = true;
        } else if (arg == "--columns") {
            show_columns = true;
        } else if (arg == "--all") {
            show_grid = true;
            show_edges = true;
            show_columns = true;
        } else if (arg == "--quiet") {
            quiet = true;
        } else {
            std::fprintf(stderr, "unknown option: %s\n", arg.c_str());
            return 2;
        }
    }

    cv::Mat bgr = cv::imread(image_path, cv::IMREAD_COLOR);
    if (bgr.empty()) {
        std::fprintf(stderr, "failed to read image: %s\n", image_path.c_str());
        return 1;
    }
    if (p.rotate_180) {
        cv::rotate(bgr, bgr, cv::ROTATE_180);
    }
    std::printf("image  : %s  (%dx%d, rotate=%d)\n", image_path.c_str(),
                bgr.cols, bgr.rows, p.rotate_180 ? 1 : 0);

    LegacyVisionPipeline vision(p);
    RoadEstimateLite road;
    for (int i = 0; i < settle; ++i) {
        road = vision.process_bgr(bgr);
    }

    const WheelMaskBox box = vision.wheel_mask_box();
    print_summary(road, box, settle);

    if (!quiet) {
        print_row(vision.debug_frame(), box.top - 1, "search-row (wheel_top-1)");
        print_row(vision.debug_frame(), box.top, "wheel-top row");
        print_row(vision.debug_frame(), kBinaryHeight - 1, "bottom (car front)");
    }
    if (show_columns) print_columns(vision.debug_frame());
    if (show_edges) print_edges(road);
    if (show_grid) print_grid(vision.debug_frame());

    if (write_images) {
        NavigationCommand command;
        const cv::Mat overlay = vision.draw_overlay(bgr, road, command);
        const cv::Mat path = vision.draw_longest_white_debug(
            vision.debug_frame(), road, command, cv::Size(320, 240));
        if (!overlay.empty()) {
            const std::string f = out_prefix + "_overlay.png";
            std::printf("wrote %s\n", cv::imwrite(f, overlay) ? f.c_str()
                                                             : "(failed)");
        }
        if (!path.empty()) {
            const std::string f = out_prefix + "_path.png";
            std::printf("wrote %s\n", cv::imwrite(f, path) ? f.c_str()
                                                          : "(failed)");
        }
    }
    return 0;
}
