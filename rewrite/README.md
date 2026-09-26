# rewrite path follow

`rewrite/` is an independent path-following program built around the existing
94x60 longest-white-column vision pipeline. It does not reuse the
`success-new2` navigation or steering state machine.

## Control flow

长期维护和合并代码前先阅读
[`LOGIC_INVARIANTS.md`](LOGIC_INVARIANTS.md)。其中固定了巡线权威基线、符号约定、状态优先级和新环岛的接管边界。

```text
camera evidence
  -> near/far path error and topology
  -> target speed + target yaw rate
  -> IMU yaw-rate/heading feedback
  -> left/right wheel speed targets
  -> encoder feed-forward + PI
  -> PWM
```

Inside the controller, visual error, yaw rate, and heading are all
right-positive. The replacement vehicle's mounted camera produces the
opposite raw horizontal error, so its launcher passes
`--vision-yaw-sign -1`. Telemetry preserves `line_error`/`far_error` as raw
image observations and adds `control_line_error`/`control_far_error` after
the physical sign conversion.

The track launcher enables bounded visual PID shaping. Integral correction
only accumulates on a stable paired line with no cross or side-opening
evidence, small near error, and small near/far disagreement. A filtered,
rate-limited derivative adds early correction while error grows and opposes
the command while the car recenters. Curves and topology events suppress or
reset controller memory so feedback does not carry windup or derivative kick
through a bend.

Side-road handling is prediction-line driven and opt-in. With `--side-road`
disabled, the roundabout tracker is inactive and cannot change the normal
line fit. An `APPROACH` candidate now uses the new stable-side and
deviation-return analysis directly; legacy `side_open` remains telemetry-only
and is not an entry gate. `APPROACH` is observation-only: normal live fits
continue to produce `line_error` and `far_error`. Three consecutive new
algorithm confirmations plus a second return enter `INSIDE`, where the
reconstructed boundary is allowed to steer the vehicle. After the entry
opening clears, two confirmed observations of
openings on both sides reconstruct the opposite exit boundary and enter
`ROUNDABOUT_EXIT`. Stable near-field boundaries plus approximately symmetric
left/right predictions produce `REACQUIRED` and restore `FOLLOW`.

The reference side-road video `2026-07-18 02-50-19.mp4` produces the calibrated
sequence `APPROACH` at 7.00 s, `INSIDE` at 11.00 s, `EXIT` at 18.00 s, and
`REACQUIRED` at 33.75 s when replayed at 4 fps with the saved tuning profile.
The verified rewrite vehicle mapping keeps logical left/right motor channels
unchanged by default. Use `--swap-motors` only if a different wiring harness
physically exchanges the two wheels.

The board-side motor pairing is left `PWM82/CH2 + DIR21` and right
`PWM81/CH1 + DIR22`. The corresponding encoder mapping is left `PIN67/CH3`
and right `PIN65/CH1`; the launcher therefore uses `--no-swap-encoders`.
The capture input measures pulse frequency only, so encoder direction is
inferred from the corresponding motor command rather than measured from an
A/B quadrature pair. A PWM channel must therefore never be paired with the
opposite wheel's direction pin.

The telemetry map frame uses the startup direction as `+X` and the vehicle's
right side as `+Y`. Heading is also right-positive, so a right turn moves
toward `+Y` and a left turn moves toward `-Y`.

The default cruise speed is 35 cm/s with a 40 cm/s hard limit. Dry-run remains
the default. Side-road and target actions are opt-in:

```bash
./lq_path_follow_rewrite --http 8080 --dry-run
./lq_path_follow_rewrite --http 8080 --side-road --dry-run
./lq_path_follow_rewrite --http 8080 --target-actions --dry-run
```

The target classifier in `target/model` uses a 32x32 RGB input. The class
groups are 0-1 weapon/left bypass, 2-3 supply/right bypass, and 4-5
vehicle/straight over. Use `--target-input-size 32` when overriding defaults.
The reference-video red-area trigger is `--target-close-size 0.04`; target
groups are locked by a seven-result vote before this close-range trigger.
The reference-strip mask uses high-saturation red plus lower-overlapping
candidate selection, so a red first-aid image cannot replace the strip below.
Action confidence sums each two-class group, and the vote rearms after a
confirmed close marker recedes so consecutive targets remain independent.

Recorded inertial routes can replace the geometric left/right bypass after
the red reference strip reaches the middle of the camera image:

```bash
./lq_path_follow_rewrite --target-path-dir /home/root --dry-run
```

This single optional parameter loads `straight_left/right.csv` for target
encounters 1 and 2, `cross_1_left/right.csv` for encounter 3,
`cross_2_left/right.csv` for encounter 4, and `chicane_left/right.csv` for
encounter 5. Weapon targets select the left file, supply targets select the
right file, and vehicle targets retain the existing straight-over action.
The default trigger requires the red-strip center at or below image row ratio
`0.50`; tune it with `--target-path-trigger-y`. Every file can be overridden
individually with `--target-straight-left/right`, `--target-cross1-left/right`,
`--target-cross2-left/right`, and `--target-chicane-left/right`. Target routes
cannot be combined with the whole-run `--inertial-path` mode.

### On-board status display

Add `--display` to mirror the live control state onto the TFT18 (ST7735S)
for on-track debugging without a laptop or Wi-Fi. It reuses the display
driver's SPI1/GPIO48/GPIO49 wiring and runs on its own render thread:

```bash
./lq_path_follow_rewrite --display --dry-run
./lq_path_follow_rewrite --http 8080 --display --display-hz 5
```

The page shows a large colored drive-state header (green FOLLOW, yellow
cross/roundabout, red STOPPED) followed by sensor mode, near/far error,
confidence and line-loss, target vs limited speed, requested vs measured yaw
rate, left/right PWM, left/right wheel speed, IMU heading and validity, ToF
distance and ramp flag, power boost and distance, any stop reason, and elapsed
time. Warnings (line lost, IMU invalid, PWM saturated, stop latched) turn red
or yellow. Only changed rows repaint, so the 8 MHz SPI refresh stays off
the control loop. Overrides: `--display-spi`, `--display-spi-speed`,
`--imu-spi-speed`,
`--display-dc`, `--display-rst`, `--display-rotation`, `--display-scale`,
`--display-hz`.

## Modules

- `vision_pipeline.*`: 94x60 thresholding, longest-column anchor, sidelines,
  centerline, cross evidence, roundabout return-point tracking, and prediction
  reconstruction for entry and exit.
- `path_controller.*`: physical navigation commands for following, cross
  heading lock, visual `ROUNDABOUT`/`ROUNDABOUT_EXIT` phases, distance-driven
  target actions, and line-loss stop/recovery.
- `imu_feedback.*`: threaded 104 Hz LSM6DSR reader with selectable I2C/SPI
  transport, startup bias calibration, freshness checks, heading integration,
  and stationary bias adaptation.
- `spi1_shared.*`: serialized SPI1 access with GPIO63 TFT CS and GPIO25 IMU
  CS. GPIO60/61/62 stay in hardware SPI mode. TFT transfers temporarily mux
  GPIO63 to the controller CS; IMU transfers hold GPIO63 high and select only
  GPIO25, which supports the board's older controller without `SPI_NO_CS`.
- `lsm6dsr_spi1.*`: 8 MHz hardware-SPI LSM6DSR transport used by
  `imu_feedback.*` when `--imu-transport spi` is selected.
- `tof_slope_sensor.*`: XSHUT-reset VL53L0X adapter using the reference
  `lq_vl53l0x::read_result()` single-shot path, stationary flat-road baseline,
  confirmed ramp entry/exit, and bounded climb power boost.
- `motion_control.*`: yaw feedback, differential wheel targets, independent
  wheel PI loops, startup/acceleration ramps, saturation protection, and
  sensor degradation.
- `motor_adapter.*`: HAL adapter, wheel-speed/distance conversion, encoder
  health checks, motor direction and channel mapping.
- `odometry.*`: midpoint integration using encoder distance and IMU heading.
- `st7735s.*`: 128x160 RGB565 display driver using Linux spidev with GPIO63
  chip select, GPIO48 for DC, and GPIO49 for reset. Includes a compact
  5x7 ASCII font (`draw_char`/`draw_text`) for on-board text.
- `status_display.*`: opt-in TFT18 live status page. It copies a compact
  scalar snapshot of each telemetry frame and renders on its own thread, so
  the slow SPI refresh never blocks the control loop. Enabling it never
  changes navigation or motor behavior.
- `http_streamer.*`: MJPEG `/stream`, JSON `/stats`, and 20 Hz NDJSON
  `/telemetry`. Telemetry clients do not cause frame cloning.

For layered motor-stutter diagnosis, build the `motor-test` target and follow
[`tools/MOTOR_STUTTER_TEST.md`](../tools/MOTOR_STUTTER_TEST.md). The test compares
one-shot PWM, repeated PWM, and the current synchronous closed-loop path while
recording per-cycle timing, encoder capture values, RPM, and PWM commands.

The near-field zebra detector is connected to the controller as an encounter
latch. The first confirmed zebra at launch remains in normal `FOLLOW`; after
the stripes clear it rearms for the finish. The second confirmed encounter
uses `ZEBRA_PASS`, which runs the same visual following control, then latches
`STOPPED/ZEBRA_COMPLETE` only after the zebra has stayed clear for the
configured exit-frame count. This final stop does not auto-resume on a good
line; restart the process for the next run. Tune the debounce with
`--zebra-enter` and `--zebra-exit` (defaults 3 and 10 frames).

Sensor degradation is continuous:

- IMU unavailable: filtered and gain-limited encoder differential yaw, limited
  to 25 cm/s.
- One encoder unavailable: healthy wheel PI plus failed-wheel feed-forward,
  limited to 20 cm/s.
- Both encoders unavailable: feed-forward wheels with IMU, limited to 15 cm/s.
- IMU and encoders unavailable: visual open loop, limited to 12 cm/s.
- Persistent visual loss still stops the vehicle after 1.2 seconds.

For a confirmed hard turn, the existing yaw loop may reduce the inner wheel
target to zero and transfer the bounded yaw correction to the outer wheel.
The inner PWM releases four times faster than normal acceleration, while
`forbid_reverse` still prevents negative PWM. Telemetry exposes
`target_steering_utilization` and `pwm_steering_utilization`; `1.0` means the
available forward-only differential is fully used. As yaw demand rises, the
outer-wheel target increases continuously using the existing
`yaw-rate-limit` wheel-differential setting. This can temporarily exceed the
body-speed limit without jumping to a fixed maximum, so no separate steering
controller or tuning parameter is introduced. Outer-wheel effort fades as
that wheel reaches its target speed and is removed on overspeed, so a fast
entry cannot keep accelerating the outer wheel through an S-curve reversal.

Normal following gives the wheel-position-to-fitted-midline offset more
weight than preview alone. Yaw feedback is scaled continuously by requested
turn strength: small corrections use reduced differential authority to avoid
left/right hunting, while confirmed hard turns retain full yaw feedback and
may stop the inner wheel. The outer-wheel speed rises continuously with turn
strength but remains within the configured vehicle speed limit, so curvature
slowdown cannot be cancelled by an oversized differential target.
The fitted-midline repair term now fades continuously with curve strength and
reaches zero in a confirmed hard curve. Straight and mild-curve centering are
unchanged, while a transient wheel-position fit cannot cancel same-direction
near/far evidence halfway through an S bend.
Near, far, and wheel-to-midline errors share a bounded per-frame visual step.
The wheel-to-midline term also fades in and out when the complete lower edge
pair appears or disappears, so one incomplete camera frame cannot abruptly
reverse steering or swap the driven outer wheel.
Roundabout entry accepts a near-stable opposite edge when the ring-side
recovery segment is one-sided. A cross still has priority only when its branch
and recovery evidence remain symmetric, preventing a ring entrance from being
consumed by a short `CROSS_LOCK`.
The same existing curve boost uses an adjustable response exponent from 1.0
to 5.0, defaulting to 3.0. Higher values keep small corrections smooth while
making gain rise much faster near the track edge. Vehicle centering is applied
only when both raw lower edges are complete across the wheel-anchor sample
band; telemetry reports this as `bottom_pair_valid`.

`--center-yaw-weight` scales that existing wheel-position-to-fitted-midline
repair term without changing near or far preview. A larger value holds the
vehicle closer to the fitted center, while zero disables only this repair.

The transition into hard-turn wheel targets and outer-wheel effort is also
continuous, so commands near the former threshold cannot alternate between
coasting and turning. Center offset contributes a direct bounded repair term.
Cross visual evidence accumulates independently of the IMU gate; a fast entry
may hand over to heading lock only when it is already converging on the
nearest configured grid heading.

## Build and deployment

Host regression test is optional and independent from deployment:

```bash
./rewrite/build_rewrite.sh selftest
```

Build the LoongArch ST7735S hardware-SPI example:

```bash
CXX=/path/to/loongarch64-linux-gnu-g++ \
  ./rewrite/build_rewrite.sh display-example
```

The shared bus wiring is SPI1 MOSI GPIO62, SPI1 MISO GPIO61, SPI1 CLK GPIO60,
TFT CS GPIO63, IMU CS GPIO25, TFT DC GPIO48, and TFT reset GPIO49. GPIO63 is
hardware-controlled only during TFT transfers and held high during IMU
transfers; GPIO25 is low only during IMU transfers. The
example opens `/dev/spidev1.0`; override it
when the board exposes the same SPI chip select under another node:

```bash
./build/rewrite/rewrite_st7735s_example --spi /dev/spidev1.0 --rotation 0
```

The default TFT and IMU SPI clocks are 8 MHz. Override them with `--speed`
for the example, `--display-spi-speed`, and `--imu-spi-speed` if needed.
This 128x160 panel has been calibrated with zero column and row offsets.

After each board reboot, configure the SPI1 pinmux and bind the device-tree
`spi1.0` ST7735 device to spidev before starting the example:

```bash
/home/root/setup_st7735s_spi.sh
```

Offline video replay writes per-frame recognition and state events:

```bash
./rewrite/build_rewrite.sh replay
./build/rewrite/rewrite_video_replay input.mov events.csv
```

Canonical Windows deployment keeps `success-new2` untouched:

```powershell
.\deploy_rewrite.ps1
.\deploy_rewrite.ps1 -RunMode Dry
.\deploy_rewrite.ps1 -RunMode Motors -ConfirmWheelsLifted
```

The deployment uses the fixed WSL Ubuntu environment and old-world LoongArch
GCC 8.3, verifies checksums, installs `/home/root/lq_path_follow_rewrite`
atomically, and keeps a `.bak`.

The default board is `root@192.168.43.220`. Builds are incremental and compile
independent translation units in parallel. Unchanged binaries, calibration,
and model files are skipped by SHA-256, while SSH connection multiplexing
avoids repeated handshakes. Useful overrides:

```powershell
.\deploy_rewrite.ps1 -Jobs 6
.\deploy_rewrite.ps1 -BuildOnly
.\deploy_rewrite.ps1 -ForceRebuild
.\deploy_rewrite.ps1 -BoardIP 192.168.43.220 -SkipModels
```

## Telemetry and trajectory

Start a board run and automatically record, stop, and plot its trajectory:

```powershell
.\tools\run_rewrite_track.ps1
.\tools\run_rewrite_track.ps1 -DryRun
.\tools\run_rewrite_track.ps1 -TargetPathDir /home/root -DryRun
```

The track runner uses the independently deployed
`/home/root/lq_path_follow_imu`, runs `setup_st7735s_spi.sh` before every
start, and enables the TFT status page by default. In target-path mode the TFT
shows the drive state plus `REC` (recognized group/confidence), `RED`
(red-strip size and vertical position), and `ACT` (encounter number and active
behavior). Target-path mode enables NCNN and does not append the older
whole-course `--line-lost-inertial` fallback.

The runner auto-detects Python with matplotlib (override it with
`-PythonExe`) and starts a terminal echo by default. It prints a
`DATA` row for recognition, line, speed, yaw, wheels and PWM, followed by a
`NAV` row for inertial state, waypoint progress, cross-track/heading error and
the current pose. Disable it with `-NoTelemetryEcho` or change the rate with
`-TelemetryEchoInterval 0.25`.

To inspect an already-running board without starting or stopping it, use the
standalone local debugger:

```powershell
.\tools\debug_rewrite_local.ps1
.\tools\debug_rewrite_local.ps1 -DurationSeconds 10
```

This entry auto-detects Python and uses the regular Windows OpenSSH client for
the optional board-process probe. Pass `-SkipSshProbe` when only HTTP telemetry
is needed.

For normal use, double-click `tools\start_rewrite_gui.bat`. The GUI saves
connection, speed-loop, visual-yaw, and ramp settings under
`build/rewrite_gui_settings.json`. It provides build/upload, live start,
safe stop, emergency SSH stop, CSV selection, and in-window trajectory and
summary display. `启动并记录` also opens the board `/stream` MJPEG feed in a
live tab; this is the board-rendered overlay with yellow/magenta road edges,
the green guidance centerline, corners, and current navigation state. The
viewer reconnects automatically and keeps the final frame after a run. Its
`实时二值化参数` panel controls the running vision process through
`GET/POST /vision-params`: minimum threshold, saturation penalty, blue HSV
range, and the extra blue-highlight penalty can be changed without restarting
the vehicle. Saved values are automatically applied after the next stream
connection. The road detector uses `gray - saturation * penalty` as its base
score and applies the extra blue penalty only inside the configured HSV gate,
preventing bright blue track reflections from becoming white-road pixels.
The stream overlay reports the applied threshold and blue-mask pixel count.
The
`舵量波形` tab reads the same `/telemetry` stream and separates near preview,
far preview, fitted-center repair, visual I/D, and navigation-state steering.
It also compares target/measured yaw rate and target/actual left-right wheel
speed difference over the latest 20 seconds. Selecting an existing CSV loads
the channels available in that recording; full component curves require a
binary that publishes the component telemetry fields.
The
cross panel exposes entry/exit confirmation, minimum
distance, timeout, speed scale, IMU heading-grid snapping/tolerance,
entry yaw-rate gating, paired-corner tolerance, and heading-hold gain/rate.
Upload installs the program without starting the motors;
`启动并记录` starts telemetry recording and generates the final plot.

The launcher defaults to live motors. Use `-DryRun` for a no-motion test.
Press Enter in its PowerShell window to stop the vehicle; the recorder then
closes the CSV and generates the PNG and summary automatically. Basic curve
testing enables `CROSS_LOCK` after five consecutive frames with paired corners,
a symmetric two-sided opening, a widened raw road profile, and an acceptable
IMU heading/rate. Cross lock snaps to the nearest configured orthogonal heading
until the paired main line is centered again.

The current track launcher uses an encoder gear ratio of `0.40`, fitted from
valid turning samples against the IMU yaw rate. The GUI exposes this ratio for
later calibration. IMU remains the primary yaw feedback; encoder yaw is the
fallback.

The current high-speed profile uses 108 cm/s cruise, a 144 cm/s reference
maximum, 110/180 cm/s2 target acceleration/deceleration, a 0.65 second startup
ramp, 0.70 wheel-speed
feed-forward, wheel-speed PI gains of 1.80/0.30, and an 80 cm/s yaw-feedback
wheel correction limit. Far-preview speed reduction begins at 0.08 normalized
error, curvature slowdown is 0.88, and the hard-curve floor is 0.38 of cruise.
At full forward-only steering, the inner target remains zero while the outer
target rises on a bounded gradient controlled by `yaw-rate-limit`. This closes
the large measured yaw-rate deficit without reversing the inner wheel or
jumping directly to a fixed maximum speed. Motor direction is never switched
during normal track following.

The track profile uses maximum
forward-only steering feedback: the inner wheel may be reduced to zero but is
never commanded to reverse. It also enables VL53L0X ramp detection on
SCL/SDA pins 84/85. Ten stationary valid samples establish the flat-road
baseline. Three consecutive valid distances at or below 400 mm confirm a ramp;
distances above 435 mm release it with the existing exit confirmation. A
confirmed ramp ensures at least a 62.4 cm/s target, adds 5% feed-forward, and
limits ramp PWM to 28% for at most
2.5 seconds. The speed floor, common power boost, wheel synchronization, and
steering restriction fade continuously as visual yaw demand rises. Normal
visual steering therefore regains full authority before a ramp curve, while
the final PWM slew limiter prevents a one-frame jump when boost ends. Encoder
speed also fades the extra feed-forward to zero at or above the target speed,
so a downhill section cannot continue applying climb power.

The motion safety guard latches `STOPPED/RUNAWAY_DETECTED` after two
consecutive samples with either an average wheel speed of at least 160 cm/s,
or at least 220 deg/s body rotation while a valid wheel exceeds 80 cm/s.
Restart the process after checking that the vehicle is back on the track.

The replacement board uses the LSM6DSR software-SPI connection from the
reference driver: SCK `PIN_60`, MOSI `PIN_62`, MISO `PIN_61`, and CS `PIN_25`.
Run with `--imu-transport spi`. The older I2C wiring remains available on SCL
`PIN_84`, SDA `PIN_85`, address `0x6A`; `--imu-transport auto` probes I2C then
SPI. Wiring does not determine yaw sign. With the motors disabled, rotate the
vehicle right and verify that `imu_heading_deg` increases; otherwise invert
`--imu-yaw-sign`.

The default `--tof-ramp-sign -1` assumes that entering the ramp shortens the
measured range. If the installed sensor points in the opposite direction and
the range increases on the ramp, use `--tof-ramp-sign 1`. Telemetry exposes
`tof_distance_mm`, `tof_baseline_mm`, `tof_delta_mm`, and `ramp_detected` for
verification.

The track launcher runs `/home/root/setup_st7735s_spi.sh` before every start.
This is required after a board reboot: without the SPI1 pinmux and spidev
binding, the SPI IMU is unavailable and motion deliberately falls back to the
25 cm/s encoder-yaw limit.

For a stationary ramp detection validation run, add `--tof-stop-test` to the
board command. It latches `STOPPED/RAMP_DETECTED` and clears both PWM outputs
as soon as the confirmed ramp event occurs.

The trajectory CSV always includes `tof_started`, `tof_valid`, raw
`tof_distance_mm`, filtered distance, baseline, and delta. At shutdown the
plotter independently scans valid range samples in both directions and writes
`*_tof_events.csv`; the text summary reports distance coverage, min/max range,
maximum increase/decrease, and a concrete sensor-init status when no valid
distance was recorded.

Create a self-contained ZIP containing the launcher, recorder, and plotter:

```powershell
.\tools\package_rewrite_track.ps1
```

The package is written to `build/packages/rewrite_track_tools.zip`. The
extracted launcher automatically uses the Python tools beside it.

Record the board-fused trajectory:

```bash
python SmartCar/tools/record_rewrite_trajectory.py
```

Render an equal-scale XY plot and summary after the run:

```bash
python SmartCar/tools/plot_rewrite_trajectory.py build/trajectory/rewrite_*.csv
```

The Python tools never open the IMU. Position comes from encoder travel plus
gyro heading; acceleration double integration is intentionally not used.

### Recorded-path inertial navigation

Record a compact path while the normal visual follower (or another manual
control process that publishes rewrite telemetry) drives the course:

```bash
python SmartCar/tools/record_inertial_path.py \
  --url http://192.168.43.220:8080/telemetry \
  --output build/paths/course.csv
```

Stop with Ctrl+C. The recorder keeps only samples with valid IMU and encoder
odometry, adds a waypoint every 5 cm or 4 degrees, and normalizes the first
pose to `x=0, y=0, heading=0`. An existing full telemetry recording can be
converted without connecting to the board:

`--dry-run` can validate the telemetry connection and IMU, but cannot record
a path because motor/encoder odometry is intentionally invalid in dry-run.
Path recording requires a real moving run with valid encoder feedback.

To record velocity and path directly from the initialized IMU without using
encoders, keep the vehicle completely still during `[IMU] stationary
calibration`, then start rewrite with high-rate telemetry:

```bash
./lq_path_follow_rewrite --http 8080 --telemetry-hz 100 \
  --imu-transport spi --dry-run
```

After `[IMU] ready` appears, manually move the vehicle while recording:

```bash
python SmartCar/tools/record_imu_path.py \
  --url http://192.168.43.220:8080/telemetry \
  --output build/paths/imu_course.csv
```

The board can write the same CSV directly, without HTTP or Python. This is the
preferred mode when the PC cannot reach port 8080:

```bash
./lq_path_follow_rewrite --imu-transport spi --dry-run \
  --imu-csv /home/root/imu_course.csv --imu-csv-hz 50
```

Stop the process with SIGINT or SIGTERM so the final buffered rows are flushed.
Copy `/home/root/imu_course.csv` back to the PC after recording.

The deployed board launcher wraps this command and creates a timestamped file:

```bash
/home/root/record_imu_path.sh
# or choose the output name and override mounting options:
/home/root/record_imu_path.sh /home/root/course.csv \
  --imu-accel-forward-sign -1
```

The launcher uses the independently deployed `/home/root/lq_path_follow_imu`
binary so other board workflows can continue managing the canonical
`lq_path_follow_rewrite` path without replacing the recorder build.

The IMU worker removes the stationary X/Y acceleration bias, maps body
acceleration into the startup frame using integrated gyro heading, and
integrates velocity and position at the sensor's 104 Hz sample rate. The CSV
contains time, `x_cm/y_cm`, heading, scalar speed, X/Y velocity, body-frame
acceleration, and the stationary flag; it can also be loaded by
`--inertial-path`. The default mounting is sensor X forward and Y right. Use
`--imu-accel-swap-xy`, `--imu-accel-forward-sign -1`, or
`--imu-accel-right-sign -1` when the physical mounting differs.

Pure accelerometer integration accumulates bias twice in position and cannot
distinguish rest from ideal constant-velocity motion. Use this mode only for
short recordings, start and end at rest, and inspect the final velocity drift
before replay. Encoder-fused `record_inertial_path.py` remains the accurate
choice for longer routes.

```bash
python SmartCar/tools/record_inertial_path.py \
  --input build/trajectory/rewrite_20260719_120000.csv \
  --output build/paths/course.csv
```

Copy the generated CSV to the board and start route replay from the same pose
and orientation used at the beginning of recording:

```bash
./lq_path_follow_rewrite \
  --inertial-path /home/root/course.csv \
  --imu-transport spi \
  --http 8080 \
  --enable-motors
```

`--inertial-path` bypasses visual navigation commands but keeps motor speed
feedback, yaw-rate feedback, ToF stop testing, and the runaway guard. The
state machine waits for fresh IMU plus encoder odometry, aligns to the first
segment, follows distance-indexed lookahead points, and stops on completion.
Add `--line-lost-inertial` to keep normal visual following at first and latch
the same recorded route only when persistent visual line loss would otherwise
stop the car. While vision is active, the inertial navigator passively tracks
route progress; after takeover, sensor-loss and cross-track deviation faults
remain active.
Persistent sensor loss or excessive cross-track error latches a fault and
requires a process restart. The camera is optional in this mode. Tune with
`--inertial-lookahead`, `--inertial-speed`, `--inertial-min-speed`,
`--inertial-heading-kp`, `--inertial-finish-tol`, and
`--inertial-max-error`. Sensor-loss timing is controlled by
`--inertial-sensor-timeout`. Always validate a new path in `--dry-run` first, then
run the first live test with the wheels lifted.

## Calibration

`标定数据.txt` maps source image rows to distance from the rear axle:

- horizon: source row 34, binary row 9;
- 60 cm control lookahead: source row 100, binary row 25;
- 120 cm far preview: source row 66, binary row 17.

`--forward-row` and `--far-row` remain available as explicit overrides.
