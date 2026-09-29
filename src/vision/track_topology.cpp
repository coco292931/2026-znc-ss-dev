#include "track_topology.hpp"

#include <algorithm>
#include <cstdlib>

namespace rewrite_path {

namespace {

constexpr std::uint8_t kTagTrack = 1;
constexpr std::uint8_t kTagLeft = 2;
constexpr std::uint8_t kTagRight = 3;

constexpr std::size_t kCellCount =
    static_cast<std::size_t>(kBinaryWidth) * kBinaryHeight;

}  // namespace

TrackTopology::TrackTopology(const PathParams& params)
    : p_(params), white_(kCellCount, 0), region_(kCellCount, 0), stack_() {
    // 显式栈，避免递归（BOOM 的 searchimg/searchleftmap 同样使用显式栈）。
    stack_.reserve(kCellCount / 4);
}

void TrackTopology::load_grid(const std::uint8_t* binary,
                              int width,
                              int height) {
    std::fill(white_.begin(), white_.end(), 0);
    std::fill(region_.begin(), region_.end(), 0);
    if (binary == nullptr || width <= 0 || height <= 0) {
        return;
    }
    // 最近邻取样。调用方通常直接给出原生 kBinaryWidth x kBinaryHeight 网格，
    // 此时是恒等映射；其余尺寸也能工作，便于合成用例。
    for (int y = 0; y < kBinaryHeight; ++y) {
        const int src_y = std::min(height - 1, y * height / kBinaryHeight);
        const std::uint8_t* src_row =
            binary + static_cast<std::size_t>(src_y) * width;
        for (int x = 0; x < kBinaryWidth; ++x) {
            const int src_x = std::min(width - 1, x * width / kBinaryWidth);
            white_[index(y, x)] = src_row[src_x] != 0 ? 1 : 0;
        }
    }
}

bool TrackTopology::is_white(int row, int col) const {
    if (row < 0 || row >= kBinaryHeight || col < 0 || col >= kBinaryWidth) {
        return false;
    }
    return white_[index(row, col)] != 0;
}

TrackTopology::Seed TrackTopology::pick_seed(int seed_col,
                                             int seed_row) const {
    Seed seed;
    if (seed_row < 0 || seed_row >= kBinaryHeight) {
        return seed;
    }
    const int center = clamp_value(seed_col, 0, kBinaryWidth - 1);
    // 对应 BOOM 在 go() 里的做法：在最靠车头的一行从中心向两侧找第一个白点。
    // 轮罩开启时底部中央会被挖空，因此允许先向更靠车头的方向逐行回退，
    // 再向远端方向回退。
    const int max_back = std::max(0, p_.topology_seed_search_rows);
    for (int back = 0; back <= max_back; ++back) {
        int row = seed_row;
        if (back > 0) {
            const int toward_car = seed_row + back;
            const int toward_far = seed_row - back;
            if (toward_car < kBinaryHeight) {
                row = toward_car;
            } else if (toward_far >= 0) {
                row = toward_far;
            } else {
                continue;
            }
        }
        for (int step = 0; step < kBinaryWidth; ++step) {
            if (step == 0) {
                if (is_white(row, center)) {
                    seed.col = center;
                    seed.row = row;
                    return seed;
                }
                continue;
            }
            const int right = center + step;
            const int left = center - step;
            if (right < kBinaryWidth && is_white(row, right)) {
                seed.col = right;
                seed.row = row;
                return seed;
            }
            if (left >= 0 && is_white(row, left)) {
                seed.col = left;
                seed.row = row;
                return seed;
            }
        }
    }
    return seed;
}

int TrackTopology::flood_fill(int start_col,
                              int start_row,
                              std::uint8_t tag,
                              int row_limit_lo,
                              int row_limit_hi) {
    stack_.clear();
    stack_.push_back(Point2i{start_col, start_row});
    int area = 0;
    while (!stack_.empty()) {
        const Point2i cell = stack_.back();
        stack_.pop_back();
        if (cell.x < 0 || cell.x >= kBinaryWidth || cell.y < 0 ||
            cell.y >= kBinaryHeight) {
            continue;
        }
        if (cell.y < row_limit_lo || cell.y > row_limit_hi) {
            continue;
        }
        const std::size_t idx = index(cell.y, cell.x);
        if (white_[idx] == 0 || region_[idx] != 0) {
            continue;
        }
        region_[idx] = tag;
        ++area;
        stack_.push_back(Point2i{cell.x - 1, cell.y});
        stack_.push_back(Point2i{cell.x + 1, cell.y});
        stack_.push_back(Point2i{cell.x, cell.y - 1});
        stack_.push_back(Point2i{cell.x, cell.y + 1});
    }
    return area;
}

void TrackTopology::summarize_region(std::uint8_t tag,
                                     TopologyRegion* region) const {
    int area = 0;
    int near_row = -1;
    int far_row = kBinaryHeight;
    int min_col = kBinaryWidth;
    int max_col = -1;
    double sum_col = 0.0;
    double sum_row = 0.0;
    for (int y = 0; y < kBinaryHeight; ++y) {
        for (int x = 0; x < kBinaryWidth; ++x) {
            if (region_[index(y, x)] != tag) {
                continue;
            }
            ++area;
            near_row = std::max(near_row, y);
            far_row = std::min(far_row, y);
            min_col = std::min(min_col, x);
            max_col = std::max(max_col, x);
            sum_col += x;
            sum_row += y;
        }
    }
    region->area = area;
    if (area == 0) {
        // 保留 seed_count，便于判断"有种子但面积不足"这种边界情况。
        return;
    }
    region->near_row = near_row;
    region->far_row = far_row;
    region->min_col = min_col;
    region->max_col = max_col;
    region->centroid_col = sum_col / area;
    region->centroid_row = sum_row / area;
}

TopologyReport TrackTopology::analyze(const std::uint8_t* binary,
                                      int width,
                                      int height,
                                      int seed_col,
                                      int seed_row) {
    load_grid(binary, width, height);

    TopologyReport report;
    report.width = kBinaryWidth;
    report.height = kBinaryHeight;

    const Seed seed = pick_seed(seed_col, seed_row);
    if (!seed.valid()) {
        return report;
    }
    report.seed_col = seed.col;
    report.seed_row = seed.row;

    // (1) basemap 等价物：从车头中点洪水填充，得到赛道连通域。
    report.track_area = flood_fill(seed.col, seed.row, kTagTrack, 0,
                                   kBinaryHeight - 1);
    if (report.track_area <= 0) {
        return report;
    }
    for (int y = 0; y < kBinaryHeight; ++y) {
        for (int x = 0; x < kBinaryWidth; ++x) {
            if (region_[index(y, x)] != kTagTrack) {
                continue;
            }
            report.track_far_row = report.track_far_row < 0
                ? y
                : std::min(report.track_far_row, y);
            report.track_near_row = std::max(report.track_near_row, y);
        }
    }

    // (2) 侧向开口证据。
    // BOOM 的 leftmap/rightmap 从图像边界生长，但其区域包含黑色背景，
    // 语义上更接近"赛道左侧的非赛道连通区域"。本模块刻意改成更适合
    // rewrite 二值网格的等价判据：
    //   在近端 topology_side_rows 行内，检查图像最左/最右列是否为白色。
    //   正常直道的边界列一定是背景黑；边界出现白色说明该侧的白色区域
    //   超出了主赛道（岔口 / 环岛入口 / 车库）。
    //   若边界白属于赛道连通域，用 border_is_track 标记
    //   （赛道本身顶到了该侧边界）；否则从边界白出发、只在白色上生长
    //   且不越过赛道，得到侧路露出的那部分。
    //   注意：不能只在"未归类"的边界白上生长——与主赛道相连的侧路
    //   会先被赛道填充吞掉，那正是最常见的情况。
    const int side_rows =
        clamp_value(p_.topology_side_rows, 1, kBinaryHeight);
    const int scan_lo = std::max(0, kBinaryHeight - side_rows);
    const int far_limit = std::max(0, report.track_far_row);
    const int left_col = 0;
    const int right_col = kBinaryWidth - 1;
    for (int y = kBinaryHeight - 1; y >= scan_lo; --y) {
        if (is_white(y, left_col)) {
            ++report.left.border_white_rows;
            report.left.border_near_row =
                std::max(report.left.border_near_row, y);
            report.left.border_far_row = report.left.border_far_row < 0
                ? y
                : std::min(report.left.border_far_row, y);
            const std::uint8_t tag = region_[index(y, left_col)];
            if (tag == kTagTrack) {
                report.left.border_is_track = true;
            } else if (tag == 0) {
                ++report.left.seed_count;
                flood_fill(left_col, y, kTagLeft, far_limit,
                           kBinaryHeight - 1);
            }
        }
        if (is_white(y, right_col)) {
            ++report.right.border_white_rows;
            report.right.border_near_row =
                std::max(report.right.border_near_row, y);
            report.right.border_far_row = report.right.border_far_row < 0
                ? y
                : std::min(report.right.border_far_row, y);
            const std::uint8_t tag = region_[index(y, right_col)];
            if (tag == kTagTrack) {
                report.right.border_is_track = true;
            } else if (tag == 0) {
                ++report.right.seed_count;
                flood_fill(right_col, y, kTagRight, far_limit,
                           kBinaryHeight - 1);
            }
        }
    }

    summarize_region(kTagLeft, &report.left);
    summarize_region(kTagRight, &report.right);

    // 面积门限只用于剔除噪点；border_* 字段不受影响。
    const auto drop_noise = [&](TopologyRegion* region) {
        const int min_area = std::max(1, p_.topology_min_region_area);
        if (region->area >= min_area) {
            return;
        }
        const int seed_count = region->seed_count;
        const int border_white_rows = region->border_white_rows;
        const int border_near_row = region->border_near_row;
        const int border_far_row = region->border_far_row;
        const bool border_is_track = region->border_is_track;
        *region = TopologyRegion{};
        region->seed_count = seed_count;
        region->border_white_rows = border_white_rows;
        region->border_near_row = border_near_row;
        region->border_far_row = border_far_row;
        region->border_is_track = border_is_track;
    };
    drop_noise(&report.left);
    drop_noise(&report.right);

    report.valid = true;
    return report;
}

}  // namespace rewrite_path
