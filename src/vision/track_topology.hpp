#pragma once

#include "path_params.hpp"
#include "path_types.hpp"

#include <cstdint>
#include <vector>

namespace rewrite_path {

// BOOM 连通域拓扑观测层。
//
// 对应关系（BOOM 名 -> 本模块名）：
//   basemap           -> cell tag 1
//                         从车头中点做洪水填充得到的赛道连通域
//   leftmap/rightmap  -> TopologyRegion left / right
//                         侧向开口证据。BOOM 从图像边界生长，但它的区域
//                         包含黑色背景；本模块改用等价且可判读的判据：
//                         近端范围内图像边界列出现白色，即说明该侧白色区域
//                         超出了主赛道。详细语义见 TopologyRegion 注释。
//   II.step (STEP1)   -> Params::topology_side_rows
//                         只在这些"近端"行里检查边界白度
//
// 与 BOOM 刻意保持的三点差异：
//   1) 不缩小分辨率。rewrite 帧率达标，直接在原生 kBinaryWidth x kBinaryHeight
//      网格上运算，因此横向阈值无需按 47/94 折算（本模块的种子固定在图像边界列，
//      横向本就没有阈值；纵向量纲与 BOOM 一致，topology_side_rows 可直接沿用）。
//   2) 不改写巡线。本模块只产出观测结果 TopologyReport，不修改 line_error /
//      far_error / 任何导航状态，符合 docs/logic-invariants.md
//      "新视觉算法只能补充环岛判断" 的约束。
//   3) 不含 BOOM 的摄像头标定数值（standardK/standardB/k1/k2/40cm 赛道宽）。
//      安装位置与角度不同，标定必须用 rewrite 自己的数据重新测量。
//
// 行序约定：沿用 rewrite 原生网格的行序 —— row 0 = 远端（图像上方），
// row (kBinaryHeight - 1) = 车头（图像下方）。
// BOOM 在二值化后做过整图上下翻转，其 row r 等价于本模块的
// row (kBinaryHeight - 1 - r)。
class TrackTopology {
public:
    using Grid = std::vector<std::uint8_t>;

    explicit TrackTopology(const PathParams& params);

    // binary: 行主序，stride == width，非 0 视为白色（赛道）。
    // seed_col: 车头中点列，通常取 RoadImageInfo::max_column。
    // seed_row: 允许用作起点的最靠车头的一行；轮罩开启时应传 wheel_top - 1。
    TopologyReport analyze(const std::uint8_t* binary,
                           int width,
                           int height,
                           int seed_col,
                           int seed_row);

    // 调试用：0 = 未归类，1 = 赛道，2 = 左侧区域，3 = 右侧区域。
    const Grid& region_map() const { return region_; }

private:
    struct Seed {
        int col = -1;
        int row = -1;
        bool valid() const { return col >= 0 && row >= 0; }
    };

    void load_grid(const std::uint8_t* binary, int width, int height);
    bool is_white(int row, int col) const;
    Seed pick_seed(int seed_col, int seed_row) const;
    int flood_fill(int start_col,
                   int start_row,
                   std::uint8_t tag,
                   int row_limit_lo,
                   int row_limit_hi);
    void summarize_region(std::uint8_t tag, TopologyRegion* region) const;

    std::size_t index(int row, int col) const {
        return static_cast<std::size_t>(row) * kBinaryWidth + col;
    }

    const PathParams& p_;
    Grid white_;   // 1 = 白色
    Grid region_;  // 0 = 未归类，1/2/3 = 赛道/左区域/右区域
    std::vector<Point2i> stack_;
};

}  // namespace rewrite_path
