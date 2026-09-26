# rewrite 独立开发目录

这是从 `2026-znc-ss` 提取并重新组织的 rewrite 路径跟随开发目录。目录只保留当前目标程序、板端驱动、模型、标定数据和配套工具；文件按功能模块重新分配，不再保留原仓库的 `rewrite`、`SmartCar`、`success-new2`、`target`、`tools` 等历史目录层级。

## 项目定位

当前主程序是 LoongArch 板端运行的 `lq_path_follow_rewrite`。控制主链路为：

```text
摄像头帧 -> 视觉边线/误差 -> 路径控制 -> IMU 偏航反馈
-> 左右轮目标速度 -> 编码器前馈与 PI -> PWM
```

`scripts/build/build_rewrite.sh target` 是目标程序的真实编译入口。Windows GUI、PowerShell 部署脚本和 WSL 后端最终都调用这条链路。

入口调用关系如下：

```text
scripts/gui/start_rewrite_gui.bat
  -> scripts/gui/rewrite_control_gui.py
  -> scripts/deploy/deploy_rewrite.ps1
  -> scripts/deploy/deploy_rewrite.sh
  -> scripts/build/build_rewrite.sh target
```

`target` 目标由 `src/app/lq_path_follow.cpp`、视觉/导航/运动/传感器/遥测核心源码、ST7735S 显示、平台 HAL、电机适配和 `src/drivers/loongson/` 驱动共同链接；它不是把 `src/` 下所有 `.cpp` 无条件编译进去。`selftest`、`replay`、显示示例和电机测试分别使用各自的最小源码集合。

## 目录结构

```text
src/
  app/                 可执行程序入口、自测、回放和硬件示例
  vision/              视觉预处理、边线/目标识别
  navigation/          路径参数、路径控制、里程计、惯性导航
  motion/              运动控制、电机适配、轮速闭环
  sensors/imu/         LSM6DSR 驱动和 IMU 反馈
  sensors/tof/         VL53L0X 和坡道检测
  telemetry/            HTTP 视频/遥测和状态快照
  display/              ST7735S 显示和 SPI1 共享总线
  platform/             HAL、编码器、电机平台适配
  platform/safety/      电机进程锁和控制安全约束
  drivers/loongson/     Loongson GPIO/ADC/PWM/地址映射驱动
  drivers/loongson/inc/ Loongson 驱动头文件

scripts/
  build/                主机自测、目标交叉编译和回放构建
  deploy/               PowerShell/WSL 构建、上传和板端启动
  run/                  板端运行、遥测记录和绘图编排
  gui/                  Windows 控制 GUI 及启动批处理
  board/                板端 SPI 配置、IMU 记录辅助脚本
  telemetry/            CSV 记录、绘图、调试和惯性路径工具

models/target/          best.ncnn.param、best.ncnn.bin、模型元数据
config/                 标定数据（config/标定数据.txt）
docs/                   控制逻辑、模块说明和不变量文档
build/                  构建和运行输出（首次构建时生成）
```

## 环境要求

- Windows PowerShell：运行部署脚本和 GUI。
- WSL，默认发行版为 `Debian`：执行板端交叉构建和部署。
- C++17 GCC/G++：`selftest`、模拟电机测试和本机工具使用。
- LoongArch GCC 8.3：`target` 交叉编译必须使用 `loongarch64-linux-gnu-g++` 8.3。
- OpenCV 4：`target` 和 `replay` 需要 OpenCV；可通过 `OPENCV_DIR` 指定安装目录，或让 `pkg-config opencv4` 可用。
- NCNN：启用目标识别并执行部署时需要 `ncnn_install`；构建脚本通过 `NCNN_DIR` 指定。
- Python 3 和 `matplotlib`：遥测记录、轨迹绘图和 GUI 的 Python 后端需要。
- 板端 SSH：默认连接 `root@192.168.43.220`，并需要可用的 SSH 密钥或其他 BatchMode 认证方式。

部署脚本默认从 Windows 目录 `$USERPROFILE\Downloads\lq环境配置 (2)` 读取 LoongArch 工具链和依赖，WSL 中对应为 `/mnt/c/Users/.../Downloads/lq环境配置 (2)`。可用 `-EnvironmentRoot` 覆盖。

## 构建

在 WSL 或 Bash 中从仓库根目录执行：

```bash
scripts/build/build_rewrite.sh selftest
scripts/build/build_rewrite.sh target
scripts/build/build_rewrite.sh replay
scripts/build/build_rewrite.sh display-example
scripts/build/build_rewrite.sh motor-test
scripts/build/build_rewrite.sh motor-test-sim
```

各模式用途：

- `selftest`：不依赖 OpenCV 和硬件的路径控制回归测试。
- `target`：生成板端主程序 `build/rewrite/lq_path_follow_rewrite`。
- `replay`：生成离线视频回放程序，需要 OpenCV。
- `display-example`：生成 ST7735S 显示示例，通常需要 Linux 的 `sys/mman.h` 等头文件。
- `motor-test`：使用 Loongson GPIO 的电机抖动分层测试。
- `motor-test-sim`：不访问真实 GPIO 的电机测试变体。

`target` 支持以下环境变量：

```bash
OPENCV_DIR=/path/to/opencv_install \
NCNN_DIR=/path/to/ncnn_install \
CXX=/path/to/loongarch64-linux-gnu-g++ \
BUILD_JOBS=6 FORCE_REBUILD=1 \
scripts/build/build_rewrite.sh target
```

未设置 `FORCE_REBUILD=1` 时，目标构建会根据源文件、头文件和编译选项哈希增量编译；`BUILD_JOBS` 控制并行编译数量。

## 交叉编译与部署

Windows 端推荐使用 PowerShell 入口：

```powershell
.\scripts\deploy\deploy_rewrite.ps1
.\scripts\deploy\deploy_rewrite.ps1 -BuildOnly
.\scripts\deploy\deploy_rewrite.ps1 -RunMode Dry
.\scripts\deploy\deploy_rewrite.ps1 -RunMode Motors -ConfirmWheelsLifted
.\scripts\deploy\deploy_rewrite.ps1 -ForceRebuild
.\scripts\deploy\deploy_rewrite.ps1 -SkipModels
```

- 默认只构建、上传并安装，不自动启动车辆。
- `-BuildOnly` 只执行 GCC 8.3 构建和 LoongArch ELF 校验，不上传、不启动电机。
- `-RunMode Dry` 启动板端程序但保持 `--dry-run`，适合检查摄像头、IMU、遥测和显示。
- `-RunMode Motors` 会启用电机，必须同时传入 `-ConfirmWheelsLifted`，并先将车轮抬离地面。
- `-SkipModels` 跳过 NCNN 模型同步；部署脚本仍会检查本地 NCNN 依赖目录。
- 上传前后都会计算 SHA-256；旧程序默认保留为 `.bak`，可用 `-NoBackup` 关闭。

部署产物为板端 `/home/root/lq_path_follow_rewrite`，同时同步：

- `/home/root/rewrite/标定数据.txt`
- `/home/root/models/best.ncnn.param`、`best.ncnn.bin`、`model_metadata.json`
- `/home/root/setup_st7735s_spi.sh` 和 `/home/root/record_imu_path.sh`

启用运行模式后，板端 HTTP 地址默认是 `http://192.168.43.220:8080`，遥测接口为 `/telemetry`，视频接口为 `/stream`。

底层 Bash 后端也可直接调用：

```bash
scripts/deploy/deploy_rewrite.sh --help
scripts/deploy/deploy_rewrite.sh --build-only
scripts/deploy/deploy_rewrite.sh --run dry
```

## 运行与遥测

```powershell
.\scripts\run\run_rewrite_track.ps1
.\scripts\run\run_rewrite_track.ps1 -DryRun
.\scripts\run\run_rewrite_track.ps1 -TargetPathDir /home/root -DryRun
```

运行脚本会启动板端程序、记录 `/telemetry` 为 CSV，并在结束后生成轨迹 PNG 和摘要。常用输出目录为：

- `build/trajectory/`：遥测 CSV、轨迹图和摘要。
- `build/paths/`：录制的惯性/目标路径 CSV。
- `build/rewrite/`：可执行文件、目标构建对象和 GUI 设置。

单独使用遥测工具：

```bash
python scripts/telemetry/record_rewrite_trajectory.py
python scripts/telemetry/plot_rewrite_trajectory.py build/trajectory/rewrite_*.csv
python scripts/telemetry/debug_rewrite_telemetry.py
python scripts/telemetry/record_inertial_path.py --output build/paths/course.csv
python scripts/telemetry/record_imu_path.py --output build/paths/imu_course.csv
```

## GUI

双击或在 PowerShell 中运行：

```text
scripts/gui/start_rewrite_gui.bat
```

GUI 提供构建/上传、Dry Run 或实车启动、停止/紧急停止、实时 MJPEG 视频、遥测状态、CSV 选择和轨迹 PNG 查看。GUI 的仓库根目录固定为本目录，不再回退到旧的 `tools` 或 `SmartCar/tools`。

## 板端辅助脚本

- `scripts/board/setup_st7735s_spi.sh`：配置 SPI1 引脚复用并绑定 ST7735S 的 spidev 设备；板端重启后需要重新执行。
- `scripts/board/record_imu_path.sh`：使用已部署程序直接记录板端 IMU 路径 CSV。

## 模型与标定

- 模型文件位于 `models/target/`，目标分类器默认使用 32x32 RGB 输入。
- 标定文件位于 `config/标定数据.txt`，程序默认从该路径读取，也可通过命令行参数覆盖。
- 目标路径、遥测和显示功能都是独立模块；不需要目标识别的本机 `selftest` 不加载 NCNN。

## 验证

执行静态语法检查：

```bash
bash -n scripts/build/build_rewrite.sh
bash -n scripts/deploy/deploy_rewrite.sh
python -m py_compile scripts/gui/*.py scripts/telemetry/*.py
```

`selftest` 是运行时测试，不应仅根据编译成功就宣称全绿：

```bash
scripts/build/build_rewrite.sh selftest
```

本机已验证 Bash/PowerShell/Python 入口和核心 C++ 语法。此前本机 `selftest` 以 `-O0` 编译成功并通过大部分测试，但在既有的 `recorded full-steer case drives only the outer wheel with feedback headroom` 行为测试处失败；该失败没有被隐藏，也不代表目标板端构建失败。Windows 本机缺少 OpenCV 时，`replay`/`target` 会按预期拒绝构建；缺少 Linux `sys/mman.h` 时，`display-example` 也无法在 Windows 原生环境编译。

## 已知限制与范围

- Windows 原生没有 Bash 时，使用 WSL 或 Git Bash；部署脚本本身默认通过 WSL Debian 执行。
- 目标板构建必须使用 GCC 8.3 LoongArch 工具链，不能用主机的 MinGW 或普通 x86 GCC 代替。
- `target` 需要 OpenCV；部署和目标识别还需要 NCNN 依赖及三个模型文件。
- 电机模式具有物理风险，必须抬轮并显式确认；运行中的失控保护仍应视为最后一道保护。
- 本目录没有复制旧版 `expt`、历史巡线资料、旧测试/回放示例和不参与当前目标程序的文件。
- `docs/rewrite.md` 和 `docs/logic-invariants.md` 记录控制流程、符号约定和修改边界；调整导航或电机逻辑前应先阅读。
