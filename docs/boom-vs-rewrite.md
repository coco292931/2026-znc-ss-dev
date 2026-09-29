# BOOM（浙工大鸿泉叁江队）vs rewrite 逐项差异对照

本文对比两套龙芯 2K0300 智能车代码的**视觉实现、控制思路与参数体系**，并记录双方可互相借鉴之处。

- 对比对象
  - **A 方（下称 BOOM）**：`极速光电龙芯-浙工大鸿泉叁江队/国赛代码/MENU0731/`（无 git，目录名即版本 `MENU0731`）
  - **B 方（下称 rewrite）**：本仓库 `2026-znc-ss-dev`（HEAD `2948c08`）
- 整理日期：2026-09-27
- 所有数值均取自代码原文，关键出处见文末「附录 A：证据索引」。

> ⚠️ 阅读提示：`MENU0731` 中 `project/code/control.h`（92 行）与 `go.h`（15 行）是 **GBK** 编码，其余为 UTF-8；用 UTF-8 打开这两个文件会出现乱码，但其中注释信息（如"属于阿克曼模型"）需要切编码后才能读到。

---

## 1. 最关键的前提：两车根本不是一类车

这一条是所有其他差异的根源，务必先看清。

| | **BOOM / 叁江队** | **rewrite** |
|---|---|---|
| 车体 | **四轮阿克曼（B 型）** | **三轮 F 型差速** |
| 转向执行器 | **舵机 S3010 前轮机械转向** | 无舵机，**左右轮速差** |
| 驱动 | 后轮双电机独立驱动 | 后轮双电机 |
| 下压力 | **无刷负压风机** | 无 |
| 转向控制量类型 | **位置型**（前轮转角） | **速度型**（轮速差） |
| 横摆来源 | 前轮侧向力（机械几何） | 轮胎纵向力差（无几何约束） |
| 横摆模型 | `omega = v * tan(delta) / L`（L = 轴距） | `omega = (v_r - v_l) / W`（W = 轮距） |
| 横摆增益量级 | 由舵机行程决定，物理上限明确 | `1 / W = 1 / 0.153 ≈ 6.5 m^-1`，极大 |
| 内外轮速度关系 | **有几何约束**（阿克曼要求内轮慢） | 无约束，纯靠控制器分配 |
| 打滑时 | 前轮几何仍在，仍能转向 | 轮差完全失效，必然失控 |
| 能否不依赖 IMU 稳住 | 能（打脚量 ≈ 已知转角，半开环可信） | **不能**，必须靠 IMU 才知道车转没转 |

**代码级证据**

- BOOM 有舵机：`servo.h` 中 `S3010_PWM_CH = "/dev/zf_device_pwm_servo"`、`S3010_HZ = 50`、`SERVO_PWM_MID = 780`。
- BOOM 作者自述阿克曼：`motor.cpp:317` 注释原文 —— *"其实就是 舵机输出的量归一化之后乘一个系数，**属于阿克曼模型**"*。
- BOOM 有负压：`PWM_1/PWM_2 = /dev/zf_device_pwm_esc_1/2` + `brushless_flag` + `menu.cpp:368` "负压状态"。
- rewrite 无舵机：全仓检索 `servo|舵机|steering_angle|轴距|前轮|万向` 结果为空。
- rewrite 的 `wheel_base_cm = 15.3` 是**两驱动轮中心距（轮距）**，不是轴距。

### 1.1 由车体差异导出的三个直接结论

**结论 1：rewrite 的横摆增益大到必须先闭环。**
`yaw_rate_correction_limit_cmps = 22`，`wheel_base_cm = 15.3` → 22 / 15.3 = **1.44 rad/s ≈ 82 dps**。
而 `max_yaw_rate_dps = 120`（= 2.09 rad/s）只需 32 cm/s 轮差 —— 即 `base = 35` 时两轮变成 19 / 51 cm/s，**内轮几乎停住**。这正好对应文档里"inner wheel may be reduced to zero"的描述。
反观舵机车：`±80 / 780 ≈ ±10%` 打脚量即可获得全部转向权限。两者尺度不在同一量级。

**结论 2：rewrite 那一批"抗蛇形"参数是被车体逼出来的。**
`error_step_limit = 2.5`（每帧误差限幅）、`center_yaw_weight = 0.75` 在弯道自动淡出、`curve_strength` 抑制 I/D 累积、`forbid_reverse`、`min_move_percent` 死区兜底 —— 根因都是"三轮差速横摆增益过大且无几何约束"。
三轮车还有隐性特点：低速时轮差产生的横摆力矩小（打不动），高速时又容易甩尾，因此还需要 `curvature_slowdown` + 硬弯速度地板 + 失控保护这一整套。

**结论 3：BOOM 的"元素级关掉 D"也讲得通了。**
`servo.cpp`：`if ((IF.ramp && IF.ramp != 3) || IF.annulus == AL5 || IF.annulus == AR5 || IF.annulusDelay) kd = 0;`
四轮车在坡道上车身俯仰会改变前轮有效转角（阿克曼几何被破坏），环岛圆弧上前轮角度持续变化 —— 此时微分项会把**几何扰动**当成误差变化去放大，于是干脆关掉 D。这是四轮车特有的问题，rewrite 不存在。

---

## 2. BOOM 的世界观（理解它的钥匙）

BOOM 不是官方库，而是智能车圈内代际传递的开源代码框架。本仓库注释里可见血统：`BOOM5 HLZ` → `BOOM7 SAMURAI_WRY` → `ZJUT BOOM7_WRY`。核心思想是 **"标准线 + 逐行标定"**，用解析方式替代真正的逆透视变换。

| 概念 | 代码 | 含义 |
|---|---|---|
| **标准线** | `standard()`: `leftline[i] = (standardK*i + standardB)/4`；`rightline[i] = XM - leftline[i]`；`standardK = 1.15, standardB = 17.0` | 预先算出的"标准直道下第 i 行左/右边线应在第几列"，是**所有偏差计算的参照物** |
| **k1[i] 横向比例** | `k1[i] = 40.0 / (rightline[i] - leftline[i] + 1)` | 假设标准赛道宽 **40 cm**，反推"第 i 行每列多少 cm"。底部 `k1[0] = 40/40 = 1.0 cm/列`，顶部 `k1[59] = 40/6 ≈ 6.7 cm/列` → **透视标定** |
| **k2[i] 纵向距离表** | `float k2[YM] = {25.1, 25.4, 25.8, ...}` | 手工标定的**行 → 距离(cm) 查表**，`k2[0] = 25.1 cm`。代码中还留有 10 组被注释的备选表（对应不同摄像头高度/俯角） |
| **车身线** | `bodyworkLine_left[i] = (bodyworkK*i + bodyworkB)/4`；`bodyworkK = 0.8` | 车身轮廓对应的两条线，判"车身会不会蹭路肩" |

`k1` 给横向 cm、`k2` 给纵向 cm，两者合起来使 BOOM 的偏差天然带物理单位 —— 这一点在控制部分会用到。

---

## 3. 视觉实现对比

### 3.1 总览

| 维度 | BOOM / 叁江队 | rewrite |
|---|---|---|
| 原始分辨率 | **188 × 60**（UVC 直出，极小） | 320 × 240 |
| 处理网格 | **47 × 60**（`XM=47, YM=60`），横向 4:1 压缩、纵向不压缩 | **94 × 60** |
| 二值化 | 大津法（阈值上下限钳位）后按阈值二值化 | `gray - saturation*penalty`（+ 蓝色 HSV 门内额外惩罚），Otsu 下限 `threshold_floor = 70` |
| 压缩规则 | 每 4 个像素中 **≥2 黑判黑**（黑优先，抑制反光假白） | 无此机制 |
| 坐标系 | 整图**上下翻转**（摄像头倒装） | `rotate_180 = true` + `vision_yaw_sign = -1` |
| **赛道提取范式** | **连通域洪水填充**：`basemap`(赛道) / `insidemap`(赛道外) / `leftmap`·`rightmap`(左右岔口证据) / `deletemap`(噪声) | **逐行找边**：底部行白种子 → 向左右找连续白 → `find_edge_from_anchor` |
| 车头中点 | `getSearchLineColMid`：在 **±range(3/15/20/25)** 内找"白色延续最长"的列 | `find_longest_white_column`：在**全图**（有轮罩遮蔽时从 `wheel_top-1` 行起）找最长白列 |
| 边线判据 | **连续 2 个黑点** | `find_edge_from_anchor`（连续性 + 宽度护栏 `8..90` 列） |
| 断线修补 | 分段 → 删短段 → `mergeAndFix` → `deleteline` → 纵向扫线补 | `fit_missing_edges_from_bottom`（底部拟合外推）；`smooth_sidelines` 已废弃未调用 |
| **误差形式** | **三路物理量**：控制线拟合的 **斜率 K**、**截距 B**、底部平均距离 `dist_bottom`(cm)；环岛/坡道另有 `getAveDeviation`(cm) | **三路归一化量**：`line_error`(近端)、`far_error`(远端预瞄)、`vehicle_center_error`(轮位对中)，均 ∈ [-1, 1] |
| 像素 → 物理 | **k1[i] 逐行横向 cm/列** + **k2[i] 行 → cm 查表** | 用 `control_distance_cm = 60` / `far_distance_cm = 120` **反查行号**（读 `config/标定数据.txt`），误差**不换算物理单位** |
| 误差平滑 | 无（单帧直接算） | `calc_error_at_row` 5 行加权 `(3 - \|offset\|)`，单边证据降权 0.55；每帧限幅 `error_step_limit = 2.5` |
| 置信度 | 无（用点数阈值隐式判断） | `0.50*可见率 + 0.40*边线可见率 + 0.10*顶部奖励`；`<0.18` 或可见率 `<0.08` 或有效行 `<4` → 丢线 |
| 方向决策 | **行车记录仪**：`drivingRecorder_record()` 记录上一圈，`getUcrossDirByDrivingRecorder()` 下圈据此决定转向 | `--inertial-path` / `--target-*-left\|right.csv` 录像复现（按目标分类选左右） |
| 单文件规模 | `deal_img.cpp` **10178 行** + `deal_img.h` 667 行 | `vision_pipeline.cpp` 93 KB |

**范式差异一句话**：

- BOOM 是 **"区域 → 线段 → 线参数"**：先圈出赛道，再切成段，最后把选中的线拟合成 `y = K*x + B`，**用 K、B 这两个线参数转向**。
- rewrite 是 **"锚点 → 逐行边线 → 采点误差"**：找到车头中点与最长白列，逐行拉出左右边线，在特定距离对应的行上采一个**归一化横向误差**（近端 60 cm、远端 120 cm），再混入轮位对中误差。

### 3.2 BOOM 视觉链分解

```
imgGray 188x60
  -> myNewOstuThreshold   大津法（阈值钳位到 [OSTU_MIN, OSTU_MAX]）
  -> Ostu_Robert          按阈值二值化 255/0   ← th_edge/sobel_threshold 参数实际未使用
  -> horizonCompress      4 像素中 >=2 黑判黑 -> 47x60
  -> 上下翻转
  -> allmap 47x60
  -> getSearchLineColMid  在 ±range 内找"白点延续最高"的列 = 车头中点
  -> searchimg            从该点 4 邻域洪水填充 -> basemap(赛道) + insidemap(赛道外)
  -> searchLeftAndRightMap 在 basemap 外向左/右生长 -> leftmap / rightmap
  -> searchLines          从底部中心逐行向左右找边线（连续 2 个黑点为边线）
  -> getLineInfoLeft/Right 按"连续 + 斜率突变>5"分段，点数<5 的段删除
  -> 角点 / 高线 / 底线 / 纵向扫线
  -> 元素状态机（IF.crossroad / annulus / ramp / garage / Fork / smallObstacle）
  -> directionControl     选控制线 -> 拟合 -> 偏差 -> 舵机 PD
```

**要点说明**

1. **横向 4:1 压缩是 BOOM 最聪明的一招**。规则为"每 4 个像素中 ≥2 黑则该点判黑"，选黑优先是因为赛道为白、反光容易产生假白，宁可让赛道稍细也不让噪声进来。运算量降到 1/4，这是在龙芯上跑出高帧率的关键；代价是横向分辨率变粗。

2. **洪水填充（signature 设计）**。不是直接"找边线"，而是先找**赛道区域**：
   - `basemap`：从车头中点 4 邻域 DFS 生长出的白色连通域（隧道白色）
   - `insidemap`：`basemap` 补集
   - `leftmap` / `rightmap`：在 `basemap` 之外分别向左/右生长的连通域 → **判断左侧/右侧有无岔口或圆弧的硬几何证据**
   - `deletemap` / `noDown`：噪声点，从线里剔除
   - `searchimg()` 使用显式栈（`Stack_Size = 3000`），避免递归
   - `searchimg_accelerated()` 是隔点加速版，注释写明*"会导致 basemap 中边界出问题…今年不打算使用"* → **未启用的死代码**

3. **扫线判据"连续两个黑点"**（`searchLines`）：
   ```c
   if (basemap[j][i] != 0 && basemap[j][i+1] != 0) leftline[j] = i + 2;
   ```
   这是抗单像素噪声的手段。

4. **分段 `getLineInfoLeft/Right` 是 BOOM 最有辨识度的设计**。把边线点按两条件切段：
   - 扫描高度上不连续 → 断段
   - 相邻两点横向差 `|leftline[j+1] - leftline[j]| > 5` → 断段
   - 段点数 `< 5` → 整段删除
   于是每个元素都有自己的"线段指纹"：十字 = 中段断开 + 上下各一段；环岛 = 单侧多一段圆弧；三岔 = 某侧多一段。**所有拓扑判断都建立在此之上**。

5. **元素状态机 20+ 个**，函数命名 `twentyCm*`（表示进入该元素后约 20 cm 内的判别流程）。特点是：**纯特征驱动**（角点位置、段起点行 `L_START_Y(0) > 30`、段点数 `lnum_all > 50` 等），距离阈值用 **k2 表换算的 cm**（如 `drawLine_distance_annulus = 100` 即 100 cm），而非像素。

6. **行车记录仪**：`drivingRecorder_record()` 记录上一圈路径，`getUcrossDirByDrivingRecorder()` 下一圈据此决定 U 型十字/环岛的转向。这是很实用的工程手段。

### 3.3 rewrite 视觉链要点

- 二值化分数 = `gray - saturation * penalty`，仅在配置的蓝色 HSV 门内再叠加额外惩罚，用于抑制"亮蓝色赛道反光变成白色像素"。
- `find_longest_white_column`：以 `last_mid_col_` 为种子，先在种子行向左右扩展出连续白段，再在该段内逐列向上数白点长度，取最长者作为 `max_column`（新寻路锚点）。
- `find_sidelines`：自底向上逐行调用 `find_edge_from_anchor`；行宽超出 `8..90` 列即判定双边无效；带"取上次双边位置为基准"的连续性护栏。
- `find_midline`：双边取平均；仅单边时用 `kNominalTrackWidth / 2` 外推；`control_row` / `far_row` 由 `control_distance_cm`（60 cm）与 `far_distance_cm`（120 cm）经 `row_for_distance()` 查标定表得到。
- `vehicle_center_error`：以轮罩上沿附近的行为锚点，减去"车身视觉中心 vs 图像中心"的偏移，得到"车身相对车道的横向偏移"。
- `line_error = preview_error * (1 - w) + vehicle_center_error * w`，`w` 取 `clamp(0.35 + 0.35*|vce|, 0.35, 0.60)`（即偏移越大越信轮位项）。
- 每帧对 `line_error` / `far_error` 做 `error_step_limit / (kBinaryWidth*0.5)` 的增量限幅。

---

## 4. 控制实现对比

### 4.1 两个转向律并排

**BOOM（实际运行）**

```
拟合控制线 -> K（斜率）、B（截距）
偏差 dev = K * kk + B * bb + dd * dist_bottom
其中 bb = baseB/100 = 2.2
     kk = bb * proportion * 10 = 2.2 * 4.0 * 10 = 88（默认）
     dd = 0（平时）; 往返十字/单边十字 dd = bb*80/100 ; 坡道 dd = bb*100/100

舵机 PWM = 780 - (kp * dev + kd * d_dev)
kp = 0.62, kd = 2.0（入弯/出弯同值），限幅 ±80
d_dev 为不完全微分（taw = 5 → 50/50 低通）
```

**rewrite**

```
归一化误差 e_near, e_far, e_center  (-1..1)
目标偏航角速度 yaw* = curve_gain_near * 58 * e_near
                    + curve_gain_far  * 82 * e_far
                    + (1 - curve_strength) * 0.75 * 58 * e_center
                    + I 项 + D 项
                    -> clamp ±120 dps

yaw 修正 = 0.045 * (yaw* - IMU 实测 yaw)  -> clamp ±22 cm/s
左轮 = base - 修正/2,  右轮 = base + 修正/2
每轮 PWM = 前馈 0.70 + PI(kp=0.35, ki=0.90, i_limit=20) -> clamp max_percent=30
```

### 4.2 五个本质差别

**① 闭环层级：BOOM 用"前轮几何"替代了角速度闭环**

| | 反馈量 | 环路 |
|---|---|---|
| BOOM | 图像偏差 | 视觉 → 偏差 → **舵机打脚（位置）**，另加"舵机打脚量 → 内轮减速"的阿克曼补偿，最后每轮速度 PID |
| rewrite | IMU 偏航角速度 | 视觉 → **目标角速度** → IMU 闭环 → **目标轮速** → 编码器 PI → PWM |

关键点：BOOM 不是"少做了一层"，而是**那一层由硬件几何做掉了** —— 舵机打脚量 ≈ 已知前轮转角，所以横摆是"可信开环"；rewrite 没有可用的几何，只能把横摆外包给 IMU 闭环。

**② 控制量的物理意义不同**

- BOOM：`dev` 是"斜率 × 88 + 截距 × 2.2"的**混合手感量**，量纲不纯，但**参数极其直观**（调 `kk` 就是调转向预瞄强度）。
- rewrite：中间量是**物理角速度 (dps)**，`near_yaw_gain` 单位是 dps/单位误差，物理意义清楚但直觉不如 BOOM 直接。

**③ "往哪看"的取向其实一致（重要共性）**

- BOOM：斜率项系数 88 vs 截距项 2.2 → **更信任车道方向**
- rewrite：`far_yaw_gain = 82 > near_yaw_gain = 58` → **更信任远端预瞄**

两者都选择"看得远一点"，但实现完全不同：BOOM 靠"整条线的斜率"（等价于对全图做一次直线拟合，天然平滑），rewrite 靠"远端某一行采一个点"（需靠 5 行加权与 `error_step_limit` 抑噪）。**这是两个独立技术栈上得出同一条工程直觉的例子。**

**④ 参数组织：写死表 vs 可调体系**

- BOOM：`DeviationParamKB` 一张表管所有元素（`proportion_enterAnnulus[5]` 支持多环岛），但**全是 int×100 写死在代码里**，改一个数要重编译重烧。
- rewrite：约 190 个 CLI 参数 + HTTP `/vision-params` 可在跑车中修改二值化阈值/饱和惩罚/蓝色范围；参数全为 double 且有物理单位。

**⑤ 元素处理：特征驱动 + 精细 vs 里程驱动 + 不变量**

- BOOM：每个元素一套"角点/段特征"判据链（20+ 状态机），精细但耦合极重（全靠全局 `II`/`IF`/`LI`/... 通信）。
- rewrite：`docs/logic-invariants.md` 明确约束"环岛未确认时不得改写普通巡线误差"，状态只有 3 类，每类有明确进入/退出条件，可验证性强。

### 4.3 BOOM 的电子差速不是"差速转向"，而是阿克曼补偿

```c
differential = SI.differential / 100.0;        // 负压车 70 -> 0.7
PWM     = SERVO_PWM_MID - servoPWM;            // 用的是舵机打脚量
MAX_PWM = (SERVO_PWM_MAX + SERVO_PWM_MIN) / 2;
if (PWM < 0 && |nowDeviation| > 5) SI.aimSpeedL = aimSpeed + PWM/MAX_PWM * differential * aimSpeed;
else if (PWM > 0 && |nowDeviation| > 5) SI.aimSpeedR = aimSpeed - PWM/MAX_PWM * differential * aimSpeed;
else SI.aimSpeedL = SI.aimSpeedR = aimSpeed;
```

注意它用**舵机输出**（而非视觉误差）决定内轮减速量，注释也写着"单纯内轮减速"。即：**先由舵机确定几何转角，再按转角反推内轮该慢多少**。主体转向是机械的，电子差速只做补偿。这与 rewrite 的"轮差即转向本身"是两回事。

### 4.4 电机闭环（纠正一个常见误解）

BOOM 的电机**不是开环**，`setMotorPWM()` 中调用了叁江队自研的完整 PID：

```c
SI.motorPWML = calcMotorPID_3JIANG(&zjutPIDLeft,  &zjutPIDParam);
SI.motorPWMR = calcMotorPID_3JIANG(&zjutPIDRight, &zjutPIDParam);
```

`calcMotorPID_3JIANG` 特性（在浙工大版基础上又改过）：

| 特性 | 实现 |
|---|---|
| 参数 | `kp = 80/10 = 8.0`，`kd = 300/10 = 30.0` |
| **变积分** | 同向：`ki = 1.0 - 1.0/(1 + e^(4 - 0.2*\|e\|))` → `\|e\|=0` 时 ≈0.98，`\|e\|=20` 时 0.5，`\|e\|=50` 时 0.0025（**大误差不积分**）；反向：`ki = 0.9`（标准积分加速收敛） |
| 不完全微分 | `derivative = 0.5*(e - e_prev) + 0.5*derivative`（与舵机 `taw=5` 同一手法） |
| 抗积分饱和 | 未饱和才累加，越界回扣 |
| **误差符号翻转清零积分** | `if ((e>0 && e_prev<0) \|\| (e<0 && e_prev>0)) integral = 0` |

对比 rewrite 的轮速环：固定 `ki = 0.90` + `i_limit = 20`，无变积分、无清积分。历史上 rewrite 曾因"积分饱和振荡"把 `ki` 从 0.9 降到 0.15、`i_limit` 从 20 降到 6 —— BOOM 这套是从原理上解决的。

### 4.5 刹车

```c
#define MOTOR_PWM_MAX       999     // 电机限幅
#define MOTOR_PWM_MIN      -500
#define REV_PWM_MAX_ABS     200     // 仅超速时允许的最大反向占空(20%)
#define SPEED_MARGIN_RPS    5.0f    // 超出目标 5 才给负扭矩
#define REV_PWM_SLOPE       2.0f
#define BRAKE_REVERSE_SPEED -800    #define BRAKE_REVERSE_TIME 270  #define BRAKE_STEP 8
// 状态机: BRAKE_START(反冲 -800) -> BRAKE_DECEL(每周期 +8 回零) -> BRAKE_ZERO
```

无刷负压电机：`pwm_set_duty(PWM_1/2, 600 -> 650 -> 700)` 阶梯启动；关闭时先 500 再 0。

---

## 5. 参数对照表（同语义不同实现）

| 语义 | BOOM / 叁江队 | rewrite |
|---|---|---|
| 转向比例增益 | `kp = 62`（0.62） | `near_yaw_gain = 58` / `far_yaw_gain = 82`（dps/单位误差） |
| 转向微分增益 | `kd = 200`（2.0），不完全微分 `taw = 5` | `vision_d_gain = 0`（**默认关闭**）+ `vision_d_filter_tau_s = 0.10` |
| 转向积分 | 无 | `vision_i_gain = 0`（**默认关闭**） |
| 输出限幅 | `SERVO_PWM_MAX = 80`（相对中位 780） | `max_yaw_rate_dps = 120` |
| 元素级增益 | `KbParam.proportion_*`（3.0 ~ 4.5） | 各状态用固定 gain（环岛/十字不用分段增益） |
| 元素级微分抑制 | 环岛 AL5/AR5、坡道 `kd = 0` | `vision_d_gain` 全局关闭；另用 curve 状态抑制 I/D 累积 |
| 车身安全 | `bodyworkSafetyAssess()`：车身边线与地图交点距离 `k2 > 200` → VERY_SAFE，`> 20` → 危险（7 档） | 无对应物（仅 `RunawayStopGuard` 传感器级保护） |
| 速度上限 | `maxSpeed = 135`（单位见下） | `max_speed_cmps = 40`（板端脚本给 95；文档提及 144 高速档） |
| 速度单位 | `SI.nowSpeedL = 5 * encoder_get_count(4ms)` → "5×计数"；电机 PID 反馈用 `varL[0]/20` | cm/s（物理单位） |
| 弯道降速 | `calcAimSpeed(DK) = ref - (ref-min)*DK^2/speedK2^2`（**按车道斜率 DK = K*100**） | `preview_slowdown` 按 **far_error** + `curvature_slowdown = 0.52`，硬弯地板 0.38×base |
| 加速规划 | 无斜坡，靠 `NORMAL_SHIFT` 换挡 | `target_accel_cmps2 = 25` / `target_decel_cmps2 = 60` + `startup_ramp_s = 0.65` |
| 电机闭环 | 变积分 PID（kp=8.0, kd=30.0, 变 ki） | 前馈 0.70 + PI（kp=0.35, ki=0.90, i_limit=20） |
| 反转策略 | `limit_reverse_pwm`（仅超速时给反向，限 20%） | `forbid_reverse = true` 默认禁反转，`min_move_percent = 4` 死区兜底 |
| 控制周期 | 4 ms 定时器中断（`pit_ms_init(4, pit_callback)`） | `control_hz = 50`（20 ms），另有 `kMaxControlDt = 0.05` 抗卡顿 |

### 5.1 BOOM 元素级增益表（`DeviationParamKB`）

| 元素 | 原始值 | 折算 `proportion` | 折算 `kk = 2.2 * proportion * 10` |
|---|---|---|---|
| 默认（`proportion_min`） | 400 | 4.0 | **88** |
| `proportion_max`（未启用） | 450 | 4.5 | 99 |
| 环岛进入 `enterAnnulus[0..4]` | 380 | 3.8 | 83.6 |
| 环岛中 `isAnnulus[0..4]` | 380 | 3.8 | 83.6 |
| 环岛出 `outAnnulus[0..4]` | 380 | 3.8 | 83.6 |
| 坡道 `proportion_ramp` | 300 | 3.0 | 66 |
| 坡道中 `proportion_onRamp` | 250 | 2.5 | 55 |
| 三岔 `proportion_fork` | 100 | 1.0 | 乘法叠加 |
| 进库 `proportion_enterGarage` | 110 | 1.1 | 96.8 |
| 出库 `proportion_outGarage` | 95 | 0.95 | 83.6 |

`baseB = 220`，另有 `proportion_dd_Ramp = 100`、`proportion_dd_ucross = 80`（用于 `dd`，即"底部距离"项的权重）。

> **解读**：`dev = 88*K + 2.2*B`。系数上斜率项是截距项的 **40 倍**。K 是"车道方向"，B 是"当前横向位置" —— 也就是说 BOOM 的转向**以"顺着车道方向"为主、以"贴中线"为辅**，本质是一种"朝车道走向对准"的前视控制。

### 5.2 BOOM 速度参数（负压组，注释称"国赛最好成绩的速度"）

| 参数 | 值 | | 参数 | 值 |
|---|---|---|---|---|
| `maxSpeed` 直道 | 135 | | `annulusSpeed` | 165 |
| `minSpeed` | 110 | | `annulusMinSpeed` | 160 |
| `normalSpeed` | 130 | | `switchSpeedTop` | 90 |
| `curveSpeed` | 125 | | `rampUpSpeed` | 100 |
| `speedK2` 入弯系数 | 50 | | `rampOnSpeed` | -100 |
| `annulusSpeedK2` | 77 | | `rampDownSpeed` | -100 |
| `SI.differential` | 70 | | `garageSpeed` 出库 | 180 |
| `constantSpeed`（未用） | 130 | | `avoidObstacleSpeed` | 190 |
| | | | `outObstacleSpeed` | 200 |

无负压版本 `SI.differential = 45`。

**`getAimSpeed_variableSpeed()` 的杀手锏**：

```c
uint8 speedTop_TFMINI = getSpeedTopByTFMINI();          // 测距值换算成图像行
SI.realSpeedTop = min(II.speedTop, speedTop_TFMINI);    // 取小
```

`II.speedTop` = 视觉能看到的赛道最远行，`speedTop_TFMINI` = 测距能看到的距离。**"视觉看多远 / 测距看多远，取更保守的那个"决定能冲多快** —— 这是"基于可预见距离变速"，与 rewrite 的"误差大就减速"是不同哲学。

---

## 6. 元素 / 拓扑处理对比

| | BOOM | rewrite |
|---|---|---|
| 元素种类 | 环岛 `AL1-6/AR1-6/AA`、十字 `CL1-2/CR1-2/CM1-2/UL1-3/UR1-3`、往返十字、三岔 `FL/FR`、车库 `GL/GR1-3`、出库 `OA/OL1-3/OR1-3`、坡道 `ramp/rampDelay`、小障碍 `smallObstacle`、斑马线、起跑线 | `cross`（预测线对称双峰锁定）、`roundabout`（APPROACH → INSIDE → EXIT → REACQUIRED）、`zebra`；坡道**靠 TOF 不靠图像** |
| 判据来源 | 角点 / 线段分布 / `basemap` 宽度 / `leftmap`·`rightmap` | 预测线对称性 / 分支计数 / 斑马线跳变计数 / 图像顶部行 |
| 状态数量 | 20+ | 3 |
| 方向决策 | 行车记录仪（上一圈记录） | 目标分类选 CSV 路线复现 |
| 距离度量 | k2 表换算 cm | 编码器里程 cm |
| 环岛实现 | 6 段状态机（进/中/出各多段） | APPROACH 锁存恢复点 + INSIDE 重建中线 + IMU/编码器同向 360° 后进入 EXIT 航向锁 |

rewrite 在这块的**能力明显弱于 BOOM**（没有三岔/车库/往返十字/小障碍/斑马线二段），但**可验证性明显强于 BOOM**（每类有明确进入/保持/退出条件，且有 `logic-invariants.md` 约束）。

---

## 7. 死代码与工程性问题对比

两套代码都有同一类问题：**为了稳，只敢注释不敢删**。

### 7.1 BOOM / MENU0731

| 内容 | 位置 | 状态 |
|---|---|---|
| `Ostu_Robert` 的 `th_edge` / `sobel_threshold` | `deal_img.cpp` `binaryAlgorithm` | **完全未使用**（函数名是历史残留） |
| `searchimg_accelerated()` | `deal_img.cpp:2318` | 注释明说"今年不打算使用" |
| `adjustParamByAimspeed()` | `control.cpp:798` | 调用处被注释；作者自评"思路有问题…d 的 max 和 min 是一样的值" |
| `getAimSpeed_constantSpeed()` | `speed.cpp:414` | 被 `variableSpeed` 取代 |
| `kd_in` / `kd_out` 非对称微分 | `servo.cpp` | 框架在，但 `differential = 0` 未启用 |
| `Fork` 三岔状态机 | `go.cpp` | 调用被注释 |
| `detect_slope_with_angle()` 图像判坡 | `deal_img.cpp` | 被注释，改用 STP23 测距 |
| `detect_second_zebra_and_stop()` | `go.cpp` | 被注释 |
| `k2[]` 另有 10 组备选标定表 | `deal_img.cpp:209-272` | 全部注释 |
| `screenLCD.cpp` 重复一份 `standardK/standardB` | `screenLCD.cpp:741` | 与 `deal_img.cpp` 重复定义 |

其他工程问题：

- `deal_img.cpp` **10178 行单文件**，20+ 状态机 + 十几个全局结构体（`II/IF/LI/RI/HL/HR/BL/BR/PL/PR/BSI/LAST_*` 及一堆 `*_INIT`），强耦合。
- `statusReset()` 每帧执行 `LAST_II = II; II = II_INIT;`，即 3 次全结构体拷贝。
- `project/out/` **提交了构建产物**（`.o`、`CMakeCache.txt`、416 KB 可执行文件 `MENU`），且路径显示构建于队友机器 `/home/tetfn/桌面/MENU0731`。
- 绝对路径硬编码：OpenCV `/home/tetfn/桌面/opencv/install`、工具链 `/opt/loongson-gnu-toolchain-8.3-...`、板卡图片 `/home/root/pictures/P2.jpg`、串口 `/dev/ttyS1`。
- `control.h`、`go.h` 为 GBK 编码（其余 UTF-8）。
- `CMakeLists.txt` 用 `aux_source_directory` 收集目录下全部源文件 → **加文件即编，无白名单**。

### 7.2 rewrite

| 类别 | 内容 |
|---|---|
| 零引用头文件（12 个，~920 行） | `drivers/loongson/inc/` 下 `LQ_GTIM_PWM / LQ_HW_PWM / LQ_HW_SPI / LQ_I2C_DEV / LQ_PWM_ENCODER / LQ_SOFT_I2C / LQ_TCP_Client / LQ_UDP_Client / LQ_Uart / LQ_YOLO / LQ_module_loader.hpp` + `sensors/imu/lq_lsm6dsr_spi.hpp`（后者已被 `lsm6dsr_spi1.*` 取代） |
| 编进 `target` 但无调用者（477 行） | `drivers/loongson/LQ_ATIM_PWM.cpp`（`success_motor.cpp` 已改为手写 ATIM 寄存器）、`drivers/loongson/LQ_HW_ADC.cpp`（唯一引用者是被废弃的 `lq_lsm6dsr_spi.hpp`） |
| 活文件中的死函数 | `vision_pipeline.cpp`：`smooth_sidelines()`(L1950)、`row_in_wheel_box()`(L1047)、`line_slope()`(L2252) |
| 死字段 | `path_types.hpp`：`RoadImageInfo.left_straight/right_straight`、`ElementFlags.small_obstacle/red_block`（从不赋值也不读） |
| 死常量 | `path_types.hpp`：`kDisplayWidth = 160` / `kDisplayHeight = 120`（实际屏幕 128×160） |
| 重复实现 | `hal.cpp` 与 `success_motor.cpp` 各存一份 `MappedPage` + `configure_pin_mux()`（~80 行逐字重复）；两套软 I2C（`lsm6dsr_i2c` / `vl53l0x_i2c`）；两套 LSM6DSR 驱动（`lq_lsm6dsr.cpp` I2C / `lsm6dsr_spi1.cpp` SPI，仅总线 3 个函数不同）；`monotonic_ns()` 两份；`path_controller.cpp` 中 heading-hold 块出现 3 次 |
| 文档问题 | `docs/rewrite.md` 第 11 行链接写作 `LOGIC_INVARIANTS.md`，实际文件名为 `logic-invariants.md` → 坏链（Windows 可开，Linux 断） |

> 注：rewrite 侧 **没有**死 CLI 参数、未调用自由函数、`#if 0` 块或未引用枚举值（已用 GCC `-Wunused-*` 交叉验证）。`imu_feedback.cpp` 的 `kGravityMps2`/`kRadiansPerDegree` 仅在 `PATH_FOLLOW_NO_HW` 编译下未使用，板端真实构建在用，不算死代码。

---

## 8. 结论

### 8.1 各自优劣

**BOOM / 叁江队**

- 优点
  1. `k1/k2` 双标定表把透视问题解析化，偏差天然是 cm，PD 参数有物理意义、跨场地迁移性好
  2. 多层地图（`basemap`/`leftmap`/`rightmap`/`deletemap`）让"单边/岔口"有硬几何证据，能识别 10+ 种元素
  3. 横向 4:1 压缩（188×60 → 47×60）运算量极小，是龙芯上高帧率的关键
  4. 元素级参数表（`DeviationParamKB`）调"每个元素的手感"非常直观
  5. 行车记录仪：记住上一圈路径来决策下一圈转向
  6. `speedTop` 视觉/测距取小来规划速度，逻辑朴素有效
  7. 电机 PID 含变积分 + 抗饱和 + 符号翻转清零，比 rewrite 的轮速环更完善
- 缺点
  1. `deal_img.cpp` 10178 行单文件 + 20+ 状态机 + 大量全局结构体，强耦合
  2. 参数写死在代码里、`×100` 整数、命名混乱（`kk/bb/dd/KbParam/proportion_*/k1/k2`）
  3. 大量注释掉的功能块（`go.cpp` 近半是注释）
  4. 转向是单帧开环直接驱动舵机，无角速度/时间抽象
  5. 构建产物与绝对路径入库

**rewrite**

- 优点：分层清晰（视觉 → 目标角速度 → IMU 闭环 → 轮速 PI → PWM）、单位物理化、参数全部可调可热改、状态只有 3 类、有 `logic-invariants.md` 约束修改边界
- 代价：拓扑识别能力远弱于 BOOM；环岛仍属"实时误差 + 偏置"雏形；没有"记住上一圈"的能力

### 8.2 互相可借鉴之处

**BOOM → rewrite（推荐）**

> 其中第 1 条与第 5 条已实施，见 [`boom-topology-port.md`](boom-topology-port.md)。

1. **`k1[i]` 逐行横向比例 + `k2[i]` 行→cm 表**：把 rewrite 的归一化误差换成物理 cm，`near_yaw_gain`/`far_yaw_gain` 立刻有物理意义，换场地不必重调。**此条不受车体差异影响，是最高价值借鉴。**
2. **`leftmap`/`rightmap` 双侧连通域**：给 rewrite 的 `side_open`/`roundabout` 一个硬几何证据（当前只靠 `left_branch_count > 0` 这类计数），环岛识别会稳得多。
3. **行车记录仪**：与已有的 `--inertial-path` / target route 合并成"第一圈记录、第二圈复现"。
4. **横向 4:1 "膨胀黑"压缩**：治远处边线断续，同时省算力。
5. **`calcMotorPID_3JIANG` 的变积分 / 符号翻转清积分 / 抗饱和**：移植到 `MotionController::wheel_output()`，正好治 rewrite 历史上"闭环负载下断断续续"的病根。

**rewrite → BOOM（推荐）**

1. **把 `servoPID.error` 的口径统一到"目标偏航角速度"并用 IMU 闭环**：BOOM 现在是纯前馈 PD，颠簸/打滑/坡道上一样会偏；他们已经在坡道上一刀切关掉 D，那正是缺角速度反馈的表现。
2. **把 `KbParam` 抽成外部配置**：现在改一个数要重编译重烧。
3. **把 20+ 状态机抽象成"进入条件 / 保持条件 / 退出条件"三件套**，减少 `II` 全局变量耦合。
4. **`line_confidence` + 丢线判定**可作为 BOOM "点数阈值"的更连续替代。

**不要照搬的（重要）**

- ❌ **不要把 BOOM 的电子差速搬给 rewrite**。BOOM 的差速是"在舵机已确定阿克曼几何之后的内轮补偿"；三轮车没有主转向，直接照搬等于把横摆再叠加一层，只会更不可控。
- ❌ **不要把 rewrite 的差速轮速分配搬给 BOOM**，同理。

### 8.3 一句话总结

> 三轮差速（rewrite）看似"少一个舵机、少两个转向轮"，结构更简单，**但对控制的要求更高**：必须闭环、必须处理打滑、必须自己解决内轮反转。
> 四轮阿克曼（BOOM）是**硬件替控制做掉了一半活**：舵机给出确定角度、阿克曼几何给出确定的内外轮关系、负压给出抓地力 —— 所以它可以用"斜率 × 88 + 截距 × 2.2"这种手感参数直接打舵机，还能跑得很快。
>
> 两套代码的气质差异由此而来：**rewrite 是"用干净的闭环体系去弥补车体缺陷"，BOOM 是"靠调好的手感榨干车体的物理优势"。**

---

## 附录 A：证据索引

### BOOM / MENU0731

| 主题 | 文件:位置 |
|---|---|
| 舵机与中位/限幅 | `project/code/servo.h`、`servo.cpp:9-11, 45-70, 84-104` |
| 阿克曼注释 | `project/code/motor.cpp:317-327` |
| 电子差速（内轮减速） | `project/code/motor.cpp:289-342` |
| 电机 PID（3JIANG 版） | `project/code/motorControlAlgorithm.cpp:343-405`；参数 `:137-141` |
| 图像尺寸与地图常量 | `project/code/deal_img.h:6-25` |
| 标准线 / k1 / 车身线 | `project/code/deal_img.cpp:821-835`（`standardK/B` 在 `:281-283`） |
| k2 距离表 | `project/code/deal_img.cpp:272`（备选表 `:209-265`） |
| 二值化 / 压缩 | `project/code/deal_img.cpp:1071-1095`、`:1167-1196`、`:1199-1223` |
| 扫线 / 中点 | `project/code/deal_img.cpp:1225-1352` |
| 分段 | `project/code/deal_img.cpp:1577-1900+`（左）、`:1901-2220+`（右） |
| 洪水填充 | `project/code/deal_img.cpp:2275-2316`（`searchimg`）、`:2318-2424`（`_accelerated`，未用） |
| 元素状态机示例 | `project/code/deal_img.cpp:5248+`（十字）、`:7297+`（环岛）、`:6706+`（坡道） |
| 选控制线 | `project/code/control.cpp:366-446` |
| 偏差计算 `dev` | `project/code/control.cpp:196-300` |
| `getDevParam` / `KbParam` | `project/code/control.cpp:58-95`、`:752-796` |
| 速度参数（负压） | `project/code/speed.cpp:96-150` |
| 变速公式与 speedTop | `project/code/speed.cpp:458-515`、`:516+` |
| 刹车状态机 | `project/code/motor.cpp:126-136, 138-210` |
| 4 ms 中断 | `project/code/main.cpp:118-135`、`common.cpp:8-19` |

### rewrite

| 主题 | 文件:位置 |
|---|---|
| 默认参数 | `src/navigation/path_params.hpp` |
| 转向律 | `src/navigation/path_controller.cpp:654-696`（`vision_yaw_breakdown` / `follow_yaw_rate`） |
| 视觉误差 | `src/vision/vision_pipeline.cpp:2025-2135`（`find_midline`）、`:2136-2220`（`classify_geometry`）、`:2208-2250`（`calc_error_at_row`） |
| 最长白列 / 边线 | `src/vision/vision_pipeline.cpp:1098-1232` |
| 轮速环 | `src/motion/motion_control.cpp`（`wheel_output`）、`motion_control.hpp` |
| 标定数据 | `config/标定数据.txt` |
| 控制不变量 | `docs/logic-invariants.md` |

### 车体事实来源

| 事实 | 来源 |
|---|---|
| BOOM = 四轮阿克曼 | `servo.h` 存在 S3010 舵机 + `motor.cpp:317` 注释"属于阿克曼模型" + 负压无刷 |
| rewrite = 三轮 F 型差速 | 全仓无 `servo/舵机/轴距/前轮` 标识；仅左右两电机；`wheel_base_cm = 15.3` 为轮距 |
