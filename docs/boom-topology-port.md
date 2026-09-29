# BOOM 拓扑原语移植到 rewrite（阶段 1 + cm 标定）

本文记录把 BOOM（浙工大鸿泉叁江队 `MENU0731`）的**寻路感知原语**移植到 rewrite 的第一阶段成果：
新增一个 opt-in 的连通域拓扑观测层，以及"归一化误差 → 物理厘米"的标定。

对照与分析背景见 [`boom-vs-rewrite.md`](boom-vs-rewrite.md)。

- 引入版本：`2026-znc-ss-dev`（在 `2948c08` 之后）
- 约束遵守：`docs/logic-invariants.md` 要求"新视觉算法只能补充环岛判断；环岛未确认时不得改写普通巡线误差"。
  本阶段**只新增观测输出**，不修改 `line_error` / `far_error` / 任何导航状态，也不改变既有默认行为。

---

## 1. 移植了什么、没移植什么

| BOOM 原语 | 处理方式 |
|---|---|
| `basemap`（从车头中点洪水填充的赛道连通域） | ✅ 移植为 `TopologyReport::track_*` |
| `leftmap` / `rightmap`（侧向区域） | ⚠️ **改写成等价判据**，见 §3 |
| `II.step`（`STEP1 = 33`，侧区域只在近端行搜索） | ✅ 移植为 `--topology-side-rows`（默认 33） |
| `searchimg` 的显式栈洪水填充 | ✅ 移植（同样用显式栈，不用递归） |
| `getLineInfoLeft/Right` 边线分段 | ❌ 本阶段未做（阶段 2 候选） |
| `chooseControlLine*` 选控制线 | ❌ 不移植：它服务于"舵机打哪条线"，三轮差速车无对应概念 |
| `kk/bb/dd` 偏差加权 + 舵机 PD | ❌ 不移植：输出量是舵机打脚，与 rewrite 的"目标偏航角速度"不同纲 |
| 20+ 元素状态机 | ❌ 不移植：阈值绑死叁江队的赛道尺度与阿克曼几何 |
| `standardK/standardB/k1/k2` 标定数值 | ❌ **不复用**：安装位置与角度不同，必须重测（见 §4） |

原则：**迁移感知原语，不迁移决策状态机**。

---

## 2. 新增模块

```
src/vision/track_topology.hpp
src/vision/track_topology.cpp        （已加入 scripts/build/build_rewrite.sh 的 CORE_SRC）
```

```cpp
class TrackTopology {
    explicit TrackTopology(const PathParams& params);
    TopologyReport analyze(const std::uint8_t* binary, int width, int height,
                           int seed_col, int seed_row);
    const Grid& region_map() const;   // 调试：1=赛道 2=左侧 3=右侧
};
```

- 输入是 `LegacyVisionPipeline` 已有的二值网格（`frame_.binary`），尺寸 **94×60 原生**，**不做分辨率缩放**
  （本车帧率达标，无需为性能降采样）。
- 内部网格放在堆上（`std::vector`），避免每个 `LegacyVisionPipeline` 实例膨胀 —— 这一点很重要：
  `LegacyVisionPipeline` 在主机自测的 `main()` 里被构造了几十次，成员若按值内嵌会显著推高栈占用。
- 行序沿用 rewrite 原生约定：**row 0 = 远端，row 59 = 车头**。
  BOOM 在二值化后做过整图上下翻转，其 `row r` 等价于本模块的 `row (60 - 1 - r)`。**移植时这是最容易错的一点。**

---

## 3. 侧向开口判据（对 BOOM 的刻意改写）

### 3.1 为什么不能照搬

BOOM 的 `leftmap` 生长条件是 `basemap[i][0]` 为真，而 `basemap` 的取值是
`0 = 赛道 / 1 = 白色但非赛道 / 2 = 黑色`，所以**该条件对黑色背景同样成立** ——
`leftmap` 实际是"赛道左侧的非赛道连通区域"，包含黑色背景，其用途是通过
`searchleftmapPoint()` 的角点提取反推赛道左边界，而不是"检测一块独立的白色区域"。

如果按字面理解成"与赛道不相连的白色区域"，会漏掉**最常见的情况**：
岔口 / 环岛入口 / 车库的白色在图像里通常是与主赛道**连通**的，
会被赛道的洪水填充先吸收掉，边界上不会留下未归类的种子。

### 3.2 改用的判据

在近端 `topology_side_rows` 行内，检查**图像最左/最右列是否为白色**：

- 正常直道的边界列一定是背景黑 → 无信号；
- 边界列出现白色 → 该侧的白色区域**超出了主赛道**（岔口 / 环岛入口 / 车库 / 反光噪点）。

并进一步区分两种情形：

| 字段 | 含义 |
|---|---|
| `border_white_rows` | 边界列上为白的行数（主信号） |
| `border_near_row` / `border_far_row` | 这些行里最靠车头 / 最远的行 |
| `border_is_track` | 边界白**属于赛道连通域**（赛道本身顶到了该侧边界 → 左侧很开阔） |
| `area` | 边界白中"不属于赛道"的部分的面积（独立白色区域的面积；`border_is_track` 时为 0） |
| `seed_count` | 独立生长起点个数 |
| `near_row` / `far_row` / `min_col` / `max_col` / `centroid_*` | 该独立区域的几何范围 |

**能力边界（务必清楚）**

- 能稳定识别：近端行范围内该侧边界出现白色（开阔 / 开口 / 明显反光）。
- 只给出"面积"的情形：边界白与赛道不相连（被黑缝隔开的侧路、车库）。
- 与赛道相连的开口：只给 `border_white_rows` 与 `border_is_track`，不给面积
  （因为面积会被赛道填充吞掉）。这不是缺陷，而是该判据的明确语义。
- 不会检测：主赛道内部的分支（两侧边界都不触及），这类仍由既有的
  `branch_count` / 预测线判据负责。

---

## 4. 厘米标定（`--cm-error`）

### 4.1 不需要新的摄像头标定

rewrite 已有 `config/标定数据.txt`（行 → 距离 cm）。本模块**只新增一项实测标量**：

```
--track-width-cm <W>     赛道物理宽度（实测一次，默认 45）
```

逐行横向比例直接由**该行的双边实测列宽**得到：

```
cm_per_col(row) = track_width_cm / width[ row ]        （双边有效的行）
cm_per_col(row) = cm_per_col(最近有效行) * d(row) / d(最近有效行)   （单边行外推）
```

其中 `d(row)` 由既有标定表插值得到（透视下横向比例正比于距离）。
这是一个"自标定"方案：赛道在图像里变宽变窄时比例自动跟随，比 BOOM 用固定标准线更贴合实际赛道。

### 4.2 输出

`line_error` / `far_error` / `vehicle_center_error` 都是"以半宽归一化"的横向偏移，因此：

```
偏移(cm) = 归一化误差 × (kBinaryWidth - 1) / 2 × cm_per_col(对应行)
```

| 新字段 | 含义 |
|---|---|
| `cm_scale_valid` | 是否成功得到比例 |
| `cm_per_col_control` / `cm_per_col_far` | 近端 / 远端行每列多少厘米 |
| `line_error_cm` / `far_error_cm` / `vehicle_center_error_cm` | 厘米偏移 |

> 注意：`*_cm` 与遥测里既有的 `line_error` / `far_error` 一样，是**图像原始观测**（未经 `vision_yaw_sign` 换算），
> 与 `control_line_error` / `control_far_error` 的约定区分保持一致。

---

## 5. 使用方法

```bash
# 拓扑观测（默认关）
./lq_path_follow_rewrite --http 8080 --dry-run --topology
./lq_path_follow_rewrite --http 8080 --dry-run --topology --topology-side-rows 33 --topology-min-area 8

# 厘米误差（默认关）
./lq_path_follow_rewrite --http 8080 --dry-run --cm-error --track-width-cm 45

# 两者同时（推荐的首次上车对比方式）
./lq_path_follow_rewrite --http 8080 --dry-run --topology --cm-error --track-width-cm 45
```

banner 会打印一行确认：

```
  topology=on side_rows=33 min_area=8 seed_rows=4 | cm_error=on track_width=45.0cm
```

### 5.1 上车验证步骤

1. `--dry-run --topology` 跑一段**正常直道与弯道**，看遥测里
   `topo_left_border_rows` / `topo_right_border_rows` 是否恒为 0。
   - 若非 0，说明阈值或二值化边缘有问题（例如两侧反光），先调 `--topology-min-area` 或二值化参数。
2. 跑一段**已知有侧向开口的赛道**（岔口 / 环岛入口 / 车库），确认进入该区域时
   对应侧的 `topo_*_border_rows` 显著上升、`topo_*_border_is_track` 置位。
3. 与既有 `left_branches` / `right_branches` **并排对比**：记录两者在同一段赛道上是否一致，
   找出新判据独有的、以及漏判的情形。这一步是决定阶段 3 是否值得做的依据。
4. `--cm-error` 部分：把车摆在赛道中央，确认 `line_error_cm` ≈ 0；
   再把车横向平移一个已知距离（用尺量），确认 `line_error_cm` 与实测接近。
   若系统性偏大/偏小，修正 `--track-width-cm`。

### 5.2 新增遥测字段

```
topo_valid, topo_track_area, topo_track_far_row, topo_track_near_row,
topo_seed_col, topo_seed_row,
topo_left_border_rows, topo_left_border_is_track, topo_left_area,
topo_left_far_row, topo_left_max_col,
topo_right_border_rows, topo_right_border_is_track, topo_right_area,
topo_right_far_row, topo_right_max_col,
cm_scale_valid, cm_per_col_control, cm_per_col_far,
line_error_cm, far_error_cm, vehicle_center_error_cm
```

> 顺带修掉一个隐患：遥测 JSON 的 `body` 缓冲原为 `char body[8192]`，而 `snprintf` 之后是
> `std::min(n, sizeof(body)-1)` —— **超出即静默截断**，会产生非法 JSON。已扩到 16384。

---

## 6. 兼容性与回归

- **默认关闭**，两个开关都不开时输出与改动前完全一致（自测中有专门断言）。
- 新增的 `RoadEstimateLite` / `RoadImageInfo` 字段全部是**新增**，未改动任何既有字段的语义。
- `RoadImageInfo` 新增 `far_row` 与 `vehicle_anchor_row`：原来这两个行号是 `find_midline()` 的局部变量，
  现在记录下来供 cm 换算与遥测使用。

### 6.1 自测

新增 14 项断言，覆盖：

| 用例 | 断言 |
|---|---|
| CLI | `--topology*` / `--cm-error` / `--track-width-cm` 可解析 |
| 直道 | 赛道连通域有效；远端行 < 近端行；两侧 `border_white_rows == 0`；无侧区域 |
| 左侧独立白块 | `border_white_rows >= 8` 且 `!border_is_track`；`area >= 100`、`min_col == 0`；右侧不受影响 |
| 赛道顶到左边界 | `border_white_rows > 0` 且 `border_is_track` |
| 边界小反光 | `border_white_rows > 0` 但 `area == 0`（被面积门限滤掉） |
| cm 标定 | `cm_scale_valid`；`cm_per_col_far > cm_per_col_control`（透视不变量）；居中的合成赛道 `abs(line_error_cm) < 5` |
| 默认关闭 | `!topology.valid && !cm_scale_valid && track_area == 0 && line_error_cm == 0` |

结果：**163 项通过**（改动前为 149），失败 1 项，且该失败是**改动前就存在**的
`recorded full-steer case drives only the outer wheel with feedback headroom`，
与本改动无关。

### 6.2 在 Windows/MinGW 上跑自测的注意事项

`build_rewrite.sh selftest` 的编译命令在 **MinGW 下会静默崩溃**：`path_selftest.cpp` 的
`main()` 有约 3000 行、大量 `{ }` 作用域里各自构造 `LegacyVisionPipeline`，
MinGW 默认栈只有 2 MB，会直接 access violation 且**没有任何输出**。
需自行加大栈：

```powershell
g++ -std=c++17 -O0 -g "-Wl,--stack,16777216" ... -pthread -o selftest.exe
```

Linux 下默认栈 8 MB，`build_rewrite.sh` 无需改动（`--stack` 是 PE/COFF 专有链接选项，
**不要**加进 `build_rewrite.sh`）。

---

## 7. 后续阶段（未实施）

| 阶段 | 内容 | 风险 |
|---|---|---|
| 2 | 把 `TopologyReport` 接入 `StepInput`，只做遥测对比 | 低 |
| 3 | 用 `border_white_rows` / `border_is_track` 作环岛 APPROACH 的准入旁证，替换或增强现有的 `branch_count` 路径 | 中：会改环岛/十字判据，需同步更新 `logic-invariants.md` 并补自测 |
| 4 | 移植 `getLineInfoLeft/Right` 的**边线分段**，用段拓扑描述岔口/环岛，进一步产出"选哪条边算误差" | 高：会触及 `line_error` 的定义 |
| 支线 | 用 `cm_per_col` 把 `near_yaw_gain` / `far_yaw_gain` 换算成物理单位（dps/cm），使增益跨场地可迁移 | 低 |

阶段 3 是否值得做，取决于 §5.1 第 3 步的实测对比结果：如果新判据没有带来
`branch_count` 之外的信息，就不应该动控制准入。
