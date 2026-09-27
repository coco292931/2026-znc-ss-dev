#!/usr/bin/env python3
"""Windows control panel for rewrite deployment, track runs, and plotting."""

from __future__ import annotations

import csv
import json
import io
import math
import os
import queue
import re
import shutil
import subprocess
import sys
import threading
import time
import tkinter as tk
import urllib.error
import urllib.parse
import urllib.request
from collections import deque
from pathlib import Path
from tkinter import filedialog, messagebox, scrolledtext, ttk

try:
    from PIL import Image, ImageDraw, ImageTk
except ImportError:
    Image = None
    ImageDraw = None
    ImageTk = None


WINDOW_TITLE = "Rewrite 智能车控制台"
SETTINGS_VERSION = 11

DEFAULT_ENVIRONMENT_ROOT = str(
    Path.home() / "Downloads" / "lq环境配置 (2)"
)
TOOLCHAIN_NAME = (
    "loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6"
)

VIDEO_VIEWS = (
    ("overlay", "实时叠加", "/stream"),
    ("gray", "灰度图", "/stream/gray"),
    ("path", "最长白线寻路", "/stream/path"),
    ("saturation", "饱和度图", "/stream/saturation"),
)

DEFAULTS: dict[str, object] = {
    "board_ip": "192.168.43.220",
    "board_user": "root",
    "identity_file": "",
    "wsl_distribution": "Debian",
    "environment_root": DEFAULT_ENVIRONMENT_ROOT,
    "startup_timeout": 20,
    "control_hz": 50,
    "jobs": 6,
    "base_speed": 65.0,
    "max_speed": 75.0,
    "hard_turn_outer_speed_scale": 1.00,
    "max_percent": 32.0,
    "target_accel": 80.0,
    "target_decel": 180.0,
    "pwm_slew": 180.0,
    "pwm_frequency_hz": 1000.0,
    "speed_ff": 0.60,
    "speed_kp": 0.45,
    "speed_ki": 0.90,
    "encoder_ratio": 0.40,
    "encoder_filter_alpha": 0.35,
    "near_yaw_gain": 100.0,
    "center_yaw_weight": 0.75,
    "far_yaw_gain": 60.0,
    "curve_yaw_boost": 0.5,
    "curve_yaw_shape": 2.0,
    "curvature_slowdown": 0.0,
    "min_follow_speed": 1.0,
    "min_follow_yaw_scale": 0.80,
    "max_yaw_rate": 180.0,
    "target_yaw_slew": 600.0,
    "yaw_rate_kp": 0.20,
    "yaw_rate_limit": 12.0,
    "encoder_yaw_kp_scale": 0.10,
    "imu_accel_deadband": 0.002,
    "imu_accel_forward_sign": 1,
    "imu_accel_right_sign": 1,
    "imu_stationary_accel": 0.02,
    "imu_stationary_yaw": 2.5,
    "imu_stationary_hold": 0.25,
    "imu_accel_swap_xy": False,
    "imu_stationary_zero": True,
    "swap_motors": False,
    "swap_encoders": False,
    "vision_i_gain": 2.0,
    "vision_d_gain": 2.0,
    "vision_error_step": 2.0,
    "cross_enter_frames": 5,
    "cross_exit_frames": 3,
    "cross_min_distance": 35.0,
    "cross_min_time": 0.8,
    "cross_timeout": 1.5,
    "cross_speed_scale": 0.70,
    "cross_heading_grid": 90.0,
    "cross_heading_tolerance": 45.0,
    "cross_enter_max_yaw_rate": 45.0,
    "heading_hold_kp": 3.0,
    "heading_hold_max_rate": 90.0,
    "cross_corner_tolerance": 3,
    "round_enter_distance": 35.0,
    "round_enter_max_error": 0.25,
    "ramp_boost": 12.0,
    "ramp_speed": 70.0,
    "ramp_max_seconds": 2.5,
    "tof_ramp_delta": 400.0,
    "dry_run": False,
    "disable_video": False,
    "disable_line_lost_stop": False,
    "side_road": True,
    "tof_slope": True,
    "force_rebuild": False,
    "skip_models": False,
    "vision_threshold_floor": 70,
    "vision_color_filter_enabled": True,
    "vision_saturation_penalty": 60,
    "vision_blue_reject_enabled": True,
    "vision_blue_hue_low": 85,
    "vision_blue_hue_high": 135,
    "vision_blue_saturation_min": 35,
    "vision_blue_value_min": 55,
    "vision_blue_penalty": 80,
}

FIELD_GROUPS = [
    (
        "连接与部署",
        [
            ("board_ip", "主板 IP", str, None, None),
            ("board_user", "SSH 用户", str, None, None),
            ("identity_file", "SSH 私钥", str, None, None),
            ("wsl_distribution", "WSL 发行版", str, None, None),
            ("environment_root", "LoongArch 环境目录", str, None, None),
            ("startup_timeout", "启动等待/秒", int, 5, 120),
            ("control_hz", "控制环频率/Hz", int, 20, 100),
            ("jobs", "编译线程", int, 1, 32),
        ],
    ),
    (
        "速度闭环",
        [
            ("base_speed", "巡航速度 cm/s", float, 5, 180),
            ("max_speed", "最大速度（不限制输入）", float, 5, None),
            (
                "hard_turn_outer_speed_scale",
                "满舵外轮上限倍率（最大速度×倍率）",
                float,
                1.0,
                2.5,
            ),
            ("max_percent", "全局 PWM 上限/%", float, 5, 100),
            ("target_accel", "加速度 cm/s²", float, 1, 200),
            ("target_decel", "减速度 cm/s²", float, 1, 300),
            ("pwm_slew", "PWM 响应速度 %/s（↑延迟更小）", float, 10, 1000),
            ("pwm_frequency_hz", "PWM 频率/Hz", float, 100, 1000000),
            ("speed_ff", "速度前馈 FF", float, 0, 3),
            ("speed_kp", "轮速 Kp", float, 0, 5),
            ("speed_ki", "轮速 Ki", float, 0, 5),
            ("encoder_ratio", "编码器传动比例", float, 0.05, 2.0),
            (
                "encoder_filter_alpha",
                "编码器响应系数（↑延迟更小）",
                float,
                0.05,
                1.0,
            ),
        ],
    ),
    (
        "转弯方向与强度",
        [
            ("near_yaw_gain", "近端方向增益（↑跟随近端）", float, 0, None),
            ("center_yaw_weight", "中线回正权重（↑更咬中线）", float, 0, None),
            ("far_yaw_gain", "远端预瞄增益（↑提前转）", float, 0, None),
            ("curve_yaw_boost", "急弯曲率增益（↑弯更急）", float, 0, None),
            (
                "curve_yaw_shape",
                "视觉函数波形（1平滑→5激进）",
                float,
                1,
                5,
            ),
            (
                "min_follow_yaw_scale",
                "低速转向保留比例（↑低速舵效更强）",
                float,
                0.3,
                1.0,
            ),
            ("max_yaw_rate", "目标偏航上限（↑弯更急）°/s", float, 0, None),
            (
                "target_yaw_slew",
                "目标偏航变化率（↑转向变化更快）°/s²",
                float,
                1,
                5000,
            ),
            ("yaw_rate_kp", "偏航反馈 Kp（↑追向更强）", float, 0, None),
            (
                "yaw_rate_limit",
                "急弯额外轮差（↑外轮更快）cm/s",
                float,
                0,
                None,
            ),
            (
                "encoder_yaw_kp_scale",
                "编码器偏航权重（IMU失效时）",
                float,
                0,
                1,
            ),
        ],
    ),
    (
        "弯道速度与稳定",
        [
            ("curvature_slowdown", "弯道减速量（↑减速更多）", float, 0, 0.9),
            ("min_follow_speed", "弯道最低速度（↑速度更高）", float, 0.1, 1.0),
            ("vision_i_gain", "贴边积分 I（↑回中更强）", float, 0, 100),
            ("vision_d_gain", "变化反馈 D（↑抑制冲出）", float, 0, 100),
            (
                "vision_error_step",
                "视觉平滑度（←平滑 / 响应快→）",
                float,
                0.5,
                5.0,
            ),
        ],
    ),
    (
        "IMU 加速度与静止归零",
        [
            ("imu_accel_deadband", "积分死区/g", float, 0, 0.1),
            ("imu_accel_forward_sign", "前向轴符号（-1 或 1）", int, -1, 1),
            ("imu_accel_right_sign", "右向轴符号（-1 或 1）", int, -1, 1),
            ("imu_stationary_accel", "静止加速度阈值/g", float, 0.001, 0.25),
            ("imu_stationary_yaw", "静止角速度阈值/°s", float, 0.1, 50),
            ("imu_stationary_hold", "静止确认时间/秒", float, 0, 5),
        ],
    ),
    (
        "十字与回正",
        [
            ("cross_enter_frames", "进入确认帧", int, 1, 60),
            ("cross_exit_frames", "退出确认帧", int, 1, 30),
            ("cross_min_distance", "最小穿越距离/cm", float, 0, 200),
            ("cross_min_time", "最短航向锁时间/秒", float, 0.1, 10),
            ("cross_timeout", "航向锁超时/秒", float, 0.2, 10),
            ("cross_speed_scale", "十字速度比例", float, 0.1, 1.0),
            ("cross_heading_grid", "正交航向间隔/°", float, 0, 180),
            ("cross_heading_tolerance", "航向吸附容差/°", float, 0, 90),
            ("cross_enter_max_yaw_rate", "进入最大角速度/°s", float, 1, 360),
            ("heading_hold_kp", "航向保持 Kp", float, 0, 20),
            ("heading_hold_max_rate", "最大回正 °/s", float, 1, 360),
            ("cross_corner_tolerance", "左右拐点行差", int, 0, 20),
            (
                "round_enter_distance",
                "环岛提前进入距离/cm（↑更早）",
                float,
                10,
                300,
            ),
            (
                "round_enter_max_error",
                "环岛进入最大巡线误差（越小越拒绝急弯）",
                float,
                0.05,
                1.0,
            ),
        ],
    ),
    (
        "坡道",
        [
            ("ramp_boost", "额外动力/%", float, 0, 50),
            ("ramp_speed", "坡道速度 cm/s", float, 5, 180),
            ("ramp_max_seconds", "最长加力/秒", float, 0.2, 10),
            ("tof_ramp_delta", "坡道距离阈值/mm（≤触发）", float, 20, 1000),
        ],
    ),
]

GROUP_NOTES = {
    "转弯方向与强度": (
        "方向固定：正偏航=右转、负偏航=左转；这里的数值只改变强弱。"
        "本组不设人工上限，PWM、轮速与失控停车保护仍然有效。"
        "调节顺序：未打满先调近端/曲率/波形；已打满但实际偏航不足，"
        "先调偏航反馈 Kp，再调急弯额外轮差。额外轮差直接提高外轮目标，"
        "内轮仍保持不反转。视觉波形越靠右，贴近边界时增益上升越陡。"
    ),
    "弯道速度与稳定": (
        "内轮目标已经为 0 但仍滑行时，提高弯道减速量并降低弯道最低速度；"
        "蛇形时先向左调视觉平滑度；转弯响应迟缓时向右调。"
    ),
    "IMU 加速度与静止归零": (
        "死区只用于速度积分，不会清除加速度遥测。达到加速度和角速度阈值并"
        "持续超过确认时间后，静止归零会将惯导速度置零并暂停位置积分。"
    ),
}

RUN_ARGUMENTS = [
    ("control_hz", "-ControlHz"),
    ("base_speed", "-BaseSpeed"),
    ("max_speed", "-MaxSpeed"),
    ("hard_turn_outer_speed_scale", "-HardTurnOuterSpeedScale"),
    ("max_percent", "-MaxPercent"),
    ("target_accel", "-TargetAccel"),
    ("target_decel", "-TargetDecel"),
    ("pwm_slew", "-PwmSlew"),
    ("pwm_frequency_hz", "-PwmFrequencyHz"),
    ("speed_ff", "-SpeedFf"),
    ("speed_kp", "-SpeedKp"),
    ("speed_ki", "-SpeedKi"),
    ("encoder_ratio", "-EncoderRatio"),
    ("encoder_filter_alpha", "-EncoderFilterAlpha"),
    ("near_yaw_gain", "-NearYawGain"),
    ("center_yaw_weight", "-CenterYawWeight"),
    ("far_yaw_gain", "-FarYawGain"),
    ("curve_yaw_boost", "-CurveYawBoost"),
    ("curve_yaw_shape", "-CurveYawShape"),
    ("curvature_slowdown", "-CurvatureSlowdown"),
    ("min_follow_speed", "-MinFollowSpeed"),
    ("min_follow_yaw_scale", "-MinFollowYawScale"),
    ("max_yaw_rate", "-MaxYawRate"),
    ("target_yaw_slew", "-TargetYawSlew"),
    ("yaw_rate_kp", "-YawRateKp"),
    ("yaw_rate_limit", "-YawRateLimit"),
    ("encoder_yaw_kp_scale", "-EncoderYawKpScale"),
    ("imu_accel_deadband", "-ImuAccelDeadband"),
    ("imu_accel_forward_sign", "-ImuAccelForwardSign"),
    ("imu_accel_right_sign", "-ImuAccelRightSign"),
    ("imu_stationary_accel", "-ImuStationaryAccel"),
    ("imu_stationary_yaw", "-ImuStationaryYaw"),
    ("imu_stationary_hold", "-ImuStationaryHold"),
    ("vision_i_gain", "-VisionIGain"),
    ("vision_d_gain", "-VisionDGain"),
    ("vision_error_step", "-VisionErrorStep"),
    ("cross_enter_frames", "-CrossEnterFrames"),
    ("cross_exit_frames", "-CrossExitFrames"),
    ("cross_min_distance", "-CrossMinDistance"),
    ("cross_min_time", "-CrossMinTime"),
    ("cross_timeout", "-CrossTimeout"),
    ("cross_speed_scale", "-CrossSpeedScale"),
    ("cross_heading_grid", "-CrossHeadingGrid"),
    ("cross_heading_tolerance", "-CrossHeadingTolerance"),
    ("cross_enter_max_yaw_rate", "-CrossEnterMaxYawRate"),
    ("heading_hold_kp", "-HeadingHoldKp"),
    ("heading_hold_max_rate", "-HeadingHoldMaxRate"),
    ("cross_corner_tolerance", "-CrossCornerTolerance"),
    ("round_enter_distance", "-RoundEnterDistance"),
    ("round_enter_max_error", "-RoundEnterMaxError"),
    ("ramp_boost", "-RampBoost"),
    ("ramp_speed", "-RampSpeed"),
    ("ramp_max_seconds", "-RampMaxSeconds"),
    ("tof_ramp_delta", "-TofRampDelta"),
    ("startup_timeout", "-StartupTimeoutSeconds"),
]


def find_repo_root(script_dir: Path) -> Path:
    candidate = script_dir.parent.parent
    if (candidate / "scripts" / "deploy" / "deploy_rewrite.ps1").is_file():
        return candidate
    return script_dir


def decode_output(raw: bytes) -> str:
    for encoding in ("utf-8", "gb18030"):
        try:
            return raw.decode(encoding).rstrip("\r\n")
        except UnicodeDecodeError:
            pass
    return raw.decode("utf-8", errors="replace").rstrip("\r\n")


class RewriteControlGui:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.script_dir = Path(__file__).resolve().parent
        self.repo_root = find_repo_root(self.script_dir)
        self.deploy_script = self.repo_root / "scripts" / "deploy" / "deploy_rewrite.ps1"
        self.run_script = self.repo_root / "scripts" / "run" / "run_rewrite_track.ps1"
        self.plot_script = self.repo_root / "scripts" / "telemetry" / "plot_rewrite_trajectory.py"
        self.output_dir = self.repo_root / "build" / "trajectory"
        self.settings_path = (
            self.repo_root / "build" / "rewrite_gui_settings.json"
        )

        self.events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.process: subprocess.Popen[bytes] | None = None
        self.process_kind = ""
        self.run_started_at = 0.0
        self.close_when_done = False
        self.last_csv: Path | None = None
        self.last_png: Path | None = None
        # The navigation path uses encoder distance with IMU heading. Raw
        # accelerometer double integration is retained as a diagnostic only.
        self.trajectory_source_var = tk.StringVar(value="odometry")
        self.path_rows: list[dict[str, str]] = []
        self.path_name = ""
        self.photo = None
        self.video_photo = None
        self.video_frame = None
        self.video_stop: threading.Event | None = None
        self.video_board_ip = ""
        self.wheel_box_ratio: tuple[float, float, float, float] | None = None
        self.wheel_selecting = False
        self.wheel_drag_start: tuple[int, int] | None = None
        self.wheel_drag_current: tuple[int, int] | None = None
        self.wheel_box_synced = False
        self.steering_stop: threading.Event | None = None
        self.steering_samples: deque[dict[str, object]] = deque(maxlen=500)
        self.autosave_after_id: str | None = None
        self.vision_auto_applied = False

        self.vars: dict[str, tk.Variable] = {}
        self.entries: dict[str, ttk.Entry] = {}
        self.vision_vars: dict[str, tk.Variable] = {}
        self.status_var = tk.StringVar(value="就绪")
        self.output_var = tk.StringVar(value="尚无轨迹")
        self.video_status_var = tk.StringVar(value="图传未连接")
        self.video_view_var = tk.StringVar(value=VIDEO_VIEWS[0][1])
        self.vision_status_var = tk.StringVar(value="视觉参数尚未同步")
        self.wheel_status_var = tk.StringVar(value="车轮框等待自动初始化")
        self.steering_status_var = tk.StringVar(value="舵量遥测未连接")
        self.outer_speed_limit_var = tk.StringVar(value="")

        self._configure_window()
        self._build_ui()
        self._load_settings()
        self._bind_autosave()
        self._update_outer_speed_limit_preview()
        self._refresh_button_state()
        self.root.after(100, self._poll_events)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _configure_window(self) -> None:
        self.root.title(WINDOW_TITLE)
        self.root.geometry("1420x860")
        self.root.minsize(1120, 700)
        style = ttk.Style(self.root)
        if "vista" in style.theme_names():
            style.theme_use("vista")
        style.configure("Action.TButton", font=("Microsoft YaHei UI", 10, "bold"))
        style.configure("Danger.TButton", foreground="#a00000")
        style.configure("Status.TLabel", padding=(8, 5))

    @staticmethod
    def _range_hint(lo: object, hi: object) -> str:
        if lo is None and hi is None:
            return ""
        if lo is None:
            return f"提示：≤ {hi}"
        if hi is None:
            return f"提示：≥ {lo}"
        return f"提示：{lo}～{hi}"

    def _build_ui(self) -> None:
        outer = ttk.Panedwindow(self.root, orient=tk.HORIZONTAL)
        outer.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        left_holder = ttk.Frame(outer, width=410)
        right = ttk.Frame(outer)
        outer.add(left_holder, weight=0)
        outer.add(right, weight=1)

        canvas = tk.Canvas(left_holder, highlightthickness=0, width=390)
        scrollbar = ttk.Scrollbar(
            left_holder, orient=tk.VERTICAL, command=canvas.yview
        )
        self.settings_frame = ttk.Frame(canvas)
        self.settings_frame.bind(
            "<Configure>",
            lambda _event: canvas.configure(scrollregion=canvas.bbox("all")),
        )
        canvas.create_window((0, 0), window=self.settings_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        canvas.bind_all(
            "<MouseWheel>",
            lambda event: canvas.yview_scroll(
                int(-1 * (event.delta / 120)), "units"
            ),
        )

        row = 0
        for title, fields in FIELD_GROUPS:
            frame = ttk.LabelFrame(self.settings_frame, text=title, padding=8)
            frame.grid(row=row, column=0, sticky="ew", padx=6, pady=5)
            frame.columnconfigure(1, weight=1)
            row += 1
            field_offset = 0
            note = GROUP_NOTES.get(title)
            if note:
                ttk.Label(
                    frame,
                    text=note,
                    foreground="#444444",
                    wraplength=350,
                    justify=tk.LEFT,
                ).grid(
                    row=0,
                    column=0,
                    columnspan=3,
                    sticky="ew",
                    pady=(0, 6),
                )
                field_offset = 1
            for field_row, (key, label, _kind, _lo, _hi) in enumerate(fields):
                ttk.Label(frame, text=label).grid(
                    row=field_row + field_offset,
                    column=0,
                    sticky="w",
                    padx=(0, 8),
                    pady=3,
                )
                var = tk.StringVar(value=str(DEFAULTS[key]))
                entry = ttk.Entry(frame, textvariable=var, width=20)
                entry.grid(
                    row=field_row + field_offset,
                    column=1,
                    sticky="ew",
                    pady=3,
                )
                self.vars[key] = var
                self.entries[key] = entry
                if key == "identity_file":
                    ttk.Button(
                        frame,
                        text="选择",
                        command=self._choose_identity,
                        width=6,
                    ).grid(
                        row=field_row + field_offset,
                        column=2,
                        padx=(5, 0),
                    )
                elif key == "environment_root":
                    ttk.Button(
                        frame,
                        text="选择",
                        command=self._choose_environment_root,
                        width=6,
                    ).grid(
                        row=field_row + field_offset,
                        column=2,
                        padx=(5, 0),
                    )
                elif key == "hard_turn_outer_speed_scale":
                    ttk.Label(
                        frame,
                        textvariable=self.outer_speed_limit_var,
                        foreground="#005a9c",
                    ).grid(
                        row=field_row + field_offset,
                        column=2,
                        sticky="w",
                        padx=(5, 0),
                    )
                else:
                    hint = self._range_hint(_lo, _hi)
                    if hint:
                        ttk.Label(
                            frame,
                            text=hint,
                            foreground="#666666",
                        ).grid(
                            row=field_row + field_offset,
                            column=2,
                            sticky="w",
                            padx=(5, 0),
                        )

        options = ttk.LabelFrame(
            self.settings_frame, text="运行选项", padding=8
        )
        options.grid(row=row, column=0, sticky="ew", padx=6, pady=5)
        row += 1
        self.option_buttons: dict[str, ttk.Checkbutton] = {}
        option_fields = [
                ("dry_run", "Dry-run（不驱动电机）"),
                ("disable_video", "禁用图传（保留遥测与轨迹记录）"),
                ("disable_line_lost_stop", "关闭丢线 STOP（继续尝试巡线）"),
                ("side_road", "启用环岛/侧路"),
                ("tof_slope", "启用 TOF 坡道检测"),
                ("imu_enabled", "启用 IMU（关闭后使用编码器降级）"),
                ("swap_motors", "交换左右电机输出（按实车接线选择）"),
                ("swap_encoders", "交换左右编码器反馈"),
                ("imu_accel_swap_xy", "交换 IMU 前向/右向加速度轴"),
                ("imu_stationary_zero", "启用 IMU 静止速度归零（ZUPT）"),
                ("force_rebuild", "部署时强制重新编译"),
                ("skip_models", "部署时跳过模型同步"),
            ]
        for index, (key, label) in enumerate(
            item for item in option_fields if item[0] in DEFAULTS
        ):
            var = tk.BooleanVar(value=bool(DEFAULTS[key]))
            self.vars[key] = var
            button = ttk.Checkbutton(options, text=label, variable=var)
            button.grid(
                row=index, column=0, sticky="w", pady=2
            )
            self.option_buttons[key] = button

        actions = ttk.LabelFrame(self.settings_frame, text="操作", padding=8)
        actions.grid(row=row, column=0, sticky="ew", padx=6, pady=5)
        actions.columnconfigure((0, 1), weight=1)
        self.deploy_button = ttk.Button(
            actions,
            text="上传部署",
            style="Action.TButton",
            command=self._deploy,
        )
        self.deploy_button.grid(row=0, column=0, sticky="ew", padx=3, pady=4)
        self.start_button = ttk.Button(
            actions,
            text="启动并记录",
            style="Action.TButton",
            command=self._start_run,
        )
        self.start_button.grid(row=0, column=1, sticky="ew", padx=3, pady=4)
        self.stop_button = ttk.Button(
            actions, text="安全停车", command=self._request_stop
        )
        self.stop_button.grid(row=1, column=0, sticky="ew", padx=3, pady=4)
        self.emergency_button = ttk.Button(
            actions,
            text="紧急停车",
            style="Danger.TButton",
            command=self._emergency_stop,
        )
        self.emergency_button.grid(row=1, column=1, sticky="ew", padx=3, pady=4)
        ttk.Button(
            actions, text="保存参数", command=self._save_settings
        ).grid(row=2, column=0, sticky="ew", padx=3, pady=4)
        ttk.Button(
            actions, text="恢复安全默认值", command=self._restore_defaults
        ).grid(row=2, column=1, sticky="ew", padx=3, pady=4)
        ttk.Button(
            actions, text="选择路径 CSV", command=self._choose_csv
        ).grid(row=3, column=0, sticky="ew", padx=3, pady=4)
        ttk.Button(
            actions, text="打开轨迹目录", command=self._open_output_dir
        ).grid(row=3, column=1, sticky="ew", padx=3, pady=4)

        ttk.Label(
            self.settings_frame,
            textvariable=self.status_var,
            style="Status.TLabel",
            wraplength=360,
        ).grid(row=row + 1, column=0, sticky="ew", padx=6, pady=5)

        notebook = ttk.Notebook(right)
        notebook.pack(fill=tk.BOTH, expand=True)
        log_tab = ttk.Frame(notebook)
        video_tab = ttk.Frame(notebook)
        steering_tab = ttk.Frame(notebook)
        trajectory_tab = ttk.Frame(notebook)
        notebook.add(video_tab, text="实时图传")
        notebook.add(steering_tab, text="舵量波形")
        notebook.add(log_tab, text="运行日志")
        notebook.add(trajectory_tab, text="轨迹图")
        self.notebook = notebook
        self.video_tab = video_tab
        self.steering_tab = steering_tab
        self.trajectory_tab = trajectory_tab

        video_tab.rowconfigure(1, weight=1)
        video_tab.columnconfigure(0, weight=1)
        ttk.Label(
            video_tab,
            textvariable=self.video_status_var,
            anchor="w",
            padding=(8, 6),
        ).grid(row=0, column=0, sticky="ew")
        ttk.Label(video_tab, text="画面：").grid(
            row=0, column=1, sticky="e", padx=(8, 2)
        )
        self.video_view_box = ttk.Combobox(
            video_tab,
            textvariable=self.video_view_var,
            values=[label for _key, label, _path in VIDEO_VIEWS],
            state="readonly",
            width=16,
        )
        self.video_view_box.grid(row=0, column=2, sticky="e", padx=(2, 8))
        self.video_view_box.bind(
            "<<ComboboxSelected>>", self._on_video_view_changed
        )

        video_body = ttk.Panedwindow(video_tab, orient=tk.HORIZONTAL)
        video_body.grid(
            row=1, column=0, columnspan=3, sticky="nsew", padx=8, pady=8
        )
        video_view = ttk.Frame(video_body)
        vision_panel = ttk.LabelFrame(
            video_body, text="实时二值化参数", padding=8
        )
        video_body.add(video_view, weight=1)
        video_body.add(vision_panel, weight=0)
        video_view.rowconfigure(0, weight=1)
        video_view.columnconfigure(0, weight=1)
        self.video_label = ttk.Label(
            video_view,
            text="启动车辆后可切换实时叠加、灰度、最长白线寻路和饱和度画面",
            anchor="center",
        )
        self.video_label.grid(row=0, column=0, sticky="nsew")
        self.video_label.bind("<ButtonPress-1>", self._wheel_select_press)
        self.video_label.bind("<B1-Motion>", self._wheel_select_motion)
        self.video_label.bind("<ButtonRelease-1>", self._wheel_select_release)

        self.vision_vars["vision_color_filter_enabled"] = tk.BooleanVar(
            value=bool(DEFAULTS["vision_color_filter_enabled"])
        )
        self.vision_vars["vision_blue_reject_enabled"] = tk.BooleanVar(
            value=bool(DEFAULTS["vision_blue_reject_enabled"])
        )
        ttk.Checkbutton(
            vision_panel,
            text="启用颜色赛道得分",
            variable=self.vision_vars["vision_color_filter_enabled"],
        ).grid(row=0, column=0, sticky="w", pady=2)
        ttk.Checkbutton(
            vision_panel,
            text="启用蓝色高光抑制",
            variable=self.vision_vars["vision_blue_reject_enabled"],
        ).grid(row=1, column=0, sticky="w", pady=(2, 5))

        vision_scales = [
            ("vision_threshold_floor", "最低阈值", 0, 255),
            ("vision_saturation_penalty", "饱和度惩罚/%", 0, 200),
            ("vision_blue_hue_low", "蓝色 H 下限", 0, 179),
            ("vision_blue_hue_high", "蓝色 H 上限", 0, 179),
            ("vision_blue_saturation_min", "蓝色 S 下限", 0, 255),
            ("vision_blue_value_min", "蓝色 V 下限", 0, 255),
            ("vision_blue_penalty", "蓝色附加惩罚", 0, 255),
        ]
        for index, (key, label, low, high) in enumerate(vision_scales, start=2):
            var = tk.StringVar(value=str(DEFAULTS[key]))
            self.vision_vars[key] = var
            row_frame = ttk.Frame(vision_panel)
            row_frame.grid(row=index, column=0, sticky="ew", pady=1)
            row_frame.columnconfigure(1, weight=1)
            ttk.Label(row_frame, text=label).grid(
                row=0,
                column=0,
                sticky="w",
                padx=(0, 6),
            )
            ttk.Entry(row_frame, textvariable=var, width=8).grid(
                row=0,
                column=1,
                sticky="ew",
            )
            ttk.Label(
                row_frame,
                text=self._range_hint(low, high),
                foreground="#666666",
            ).grid(row=0, column=2, sticky="w", padx=(6, 0))

        button_row = 2 + len(vision_scales)
        buttons = ttk.Frame(vision_panel)
        buttons.grid(row=button_row, column=0, sticky="ew", pady=(8, 4))
        buttons.columnconfigure((0, 1), weight=1)
        ttk.Button(
            buttons, text="连接图传", command=self._connect_video_tuning
        ).grid(row=0, column=0, sticky="ew", padx=(0, 3), pady=2)
        ttk.Button(
            buttons, text="读取车端", command=self._refresh_vision_params
        ).grid(row=0, column=1, sticky="ew", padx=(3, 0), pady=2)
        ttk.Button(
            buttons, text="应用到车端", command=self._apply_vision_params
        ).grid(row=1, column=0, columnspan=2, sticky="ew", pady=2)
        self.wheel_select_button = ttk.Button(
            buttons, text="手动框选车轮", command=self._start_wheel_selection
        )
        self.wheel_select_button.grid(
            row=2, column=0, columnspan=2, sticky="ew", pady=(6, 2)
        )
        ttk.Label(
            vision_panel,
            textvariable=self.vision_status_var,
            wraplength=240,
            foreground="#555555",
        ).grid(row=button_row + 1, column=0, sticky="ew", pady=(4, 0))
        ttk.Label(
            vision_panel,
            textvariable=self.wheel_status_var,
            wraplength=240,
            foreground="#555555",
        ).grid(row=button_row + 2, column=0, sticky="ew", pady=(4, 0))
        ttk.Label(
            vision_panel,
            text=(
                "建议先保持默认值；蓝色仍变白时提高饱和度/蓝色惩罚，"
                "白赛道断裂时降低惩罚或提高蓝色 S 下限。"
            ),
            wraplength=240,
            justify=tk.LEFT,
            foreground="#555555",
        ).grid(row=button_row + 3, column=0, sticky="ew", pady=(8, 0))

        steering_tab.columnconfigure(0, weight=1)
        for graph_row in range(1, 4):
            steering_tab.rowconfigure(graph_row, weight=1)
        ttk.Label(
            steering_tab,
            textvariable=self.steering_status_var,
            anchor="w",
            padding=(8, 6),
        ).grid(row=0, column=0, sticky="ew")
        self.steering_factor_canvas = tk.Canvas(
            steering_tab, background="#11151a", highlightthickness=0
        )
        self.steering_yaw_canvas = tk.Canvas(
            steering_tab, background="#11151a", highlightthickness=0
        )
        self.steering_wheel_canvas = tk.Canvas(
            steering_tab, background="#11151a", highlightthickness=0
        )
        self.steering_factor_canvas.grid(
            row=1, column=0, sticky="nsew", padx=8, pady=(8, 3)
        )
        self.steering_yaw_canvas.grid(
            row=2, column=0, sticky="nsew", padx=8, pady=3
        )
        self.steering_wheel_canvas.grid(
            row=3, column=0, sticky="nsew", padx=8, pady=(3, 8)
        )
        for graph in (
            self.steering_factor_canvas,
            self.steering_yaw_canvas,
            self.steering_wheel_canvas,
        ):
            graph.bind("<Configure>", lambda _event: self._draw_steering_waveforms())

        self.log = scrolledtext.ScrolledText(
            log_tab,
            wrap=tk.WORD,
            font=("Consolas", 10),
            background="#11151a",
            foreground="#d9e2ec",
            insertbackground="white",
        )
        self.log.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.log.configure(state=tk.DISABLED)

        trajectory_tab.rowconfigure(1, weight=1)
        trajectory_tab.columnconfigure(0, weight=1)
        trajectory_toolbar = ttk.Frame(trajectory_tab)
        trajectory_toolbar.grid(row=0, column=0, sticky="ew")
        trajectory_toolbar.columnconfigure(0, weight=1)
        ttk.Label(
            trajectory_toolbar,
            textvariable=self.output_var,
            anchor="w",
            padding=(8, 6),
        ).grid(row=0, column=0, sticky="ew")
        ttk.Label(trajectory_toolbar, text="路径视图").grid(
            row=0, column=1, padx=(6, 2)
        )
        for column, (value, label) in enumerate(
            (
                ("odometry", "IMU+编码器"),
                ("imu", "纯IMU诊断"),
                ("overlay", "叠加诊断"),
            ),
            start=2,
        ):
            ttk.Radiobutton(
                trajectory_toolbar,
                text=label,
                value=value,
                variable=self.trajectory_source_var,
                command=self._trajectory_source_changed,
            ).grid(row=0, column=column, padx=2)
        self.image_label = ttk.Label(
            trajectory_tab,
            text="运行结束后将在这里显示轨迹图",
            anchor="center",
        )
        self.image_label.grid(row=1, column=0, sticky="nsew", padx=8, pady=5)
        self.path_canvas = tk.Canvas(
            trajectory_tab,
            background="#fafbfc",
            highlightthickness=1,
            highlightbackground="#c8cdd2",
        )
        self.path_canvas.bind(
            "<Configure>", lambda _event: self._draw_path_canvas()
        )
        self.summary = scrolledtext.ScrolledText(
            trajectory_tab, height=8, wrap=tk.WORD, font=("Microsoft YaHei UI", 9)
        )
        self.summary.grid(row=2, column=0, sticky="ew", padx=8, pady=(0, 8))
        self.summary.configure(state=tk.DISABLED)

    def _choose_identity(self) -> None:
        path = filedialog.askopenfilename(title="选择 SSH 私钥")
        if path:
            self.vars["identity_file"].set(path)

    def _choose_environment_root(self) -> None:
        current = str(self.vars["environment_root"].get()).strip()
        initial_dir = current if Path(current).is_dir() else str(Path.home())
        path = filedialog.askdirectory(
            title="选择 LoongArch 工具链与依赖目录",
            initialdir=initial_dir,
        )
        if path:
            self.vars["environment_root"].set(path)

    def _load_settings(self) -> None:
        values = dict(DEFAULTS)
        try:
            if self.settings_path.is_file():
                saved = json.loads(
                    self.settings_path.read_text(encoding="utf-8")
                )
                values.update(saved)
                legacy_aggressive = (
                    float(saved.get("speed_ff", 1.0)) <= 0.45
                    and float(saved.get("speed_kp", 0.0)) >= 3.0
                    and float(saved.get("center_yaw_weight", 0.0)) >= 5.0
                    and float(saved.get("yaw_rate_limit", 0.0)) >= 120.0
                    and float(saved.get("vision_i_gain", 0.0)) >= 12.0
                    and float(saved.get("vision_error_step", 5.0)) <= 1.6
                )
                saved_version = int(saved.get("_settings_version", 0))
                if saved_version < 2:
                    for key in (
                        "base_speed",
                        "max_speed",
                        "max_percent",
                        "speed_ff",
                        "speed_kp",
                        "speed_ki",
                        "near_yaw_gain",
                        "center_yaw_weight",
                        "far_yaw_gain",
                        "curve_yaw_boost",
                        "curve_yaw_shape",
                        "curvature_slowdown",
                        "min_follow_speed",
                        "max_yaw_rate",
                        "target_yaw_slew",
                        "yaw_rate_kp",
                        "yaw_rate_limit",
                        "encoder_yaw_kp_scale",
                        "vision_i_gain",
                        "vision_d_gain",
                        "vision_error_step",
                        "cross_heading_tolerance",
                        "ramp_boost",
                        "tof_ramp_delta",
                    ):
                        values[key] = DEFAULTS[key]
                    self._append_log(
                        "[设置] 已取消转弯减速，并将旧参数迁移为连续控制参数"
                    )
                if saved_version < 3:
                    for key in (
                        "imu_accel_deadband",
                        "imu_accel_forward_sign",
                        "imu_accel_right_sign",
                        "imu_stationary_accel",
                        "imu_stationary_yaw",
                        "imu_stationary_hold",
                        "imu_accel_swap_xy",
                        "imu_stationary_zero",
                    ):
                        values[key] = DEFAULTS[key]
                    self._append_log(
                        "[设置] 已加入 IMU 加速度与静止归零的安全默认参数"
                    )
                if saved_version < 5:
                    for key in (
                        "pwm_slew",
                        "encoder_filter_alpha",
                        "cross_min_time",
                    ):
                        values[key] = DEFAULTS[key]
                    if float(saved.get("cross_timeout", 0.8)) <= 0.8:
                        values["cross_timeout"] = DEFAULTS["cross_timeout"]
                    self._append_log(
                        "[设置] 已加入低延迟驱动和十字航向锁安全参数"
                    )
                if saved_version < 6:
                    values["control_hz"] = DEFAULTS["control_hz"]
                    values["encoder_filter_alpha"] = DEFAULTS[
                        "encoder_filter_alpha"
                    ]
                    self._append_log(
                        "[设置] 已将驱动闭环分离为 50Hz 并恢复稳定编码器滤波"
                    )
                if saved_version < 7:
                    values["wsl_distribution"] = DEFAULTS[
                        "wsl_distribution"
                    ]
                    values["environment_root"] = DEFAULTS[
                        "environment_root"
                    ]
                    self._append_log(
                        "[设置] 已切换为本机 Debian WSL 与 LoongArch 环境目录"
                    )
                if saved_version < 8:
                    values["swap_motors"] = DEFAULTS["swap_motors"]
                    values["swap_encoders"] = DEFAULTS["swap_encoders"]
                    self._append_log(
                        "[设置] 已重置左右电机交换参数为实车接线配置"
                    )
                if saved_version < 9:
                    values["disable_line_lost_stop"] = DEFAULTS[
                        "disable_line_lost_stop"
                    ]
                if saved_version < 11:
                    values["swap_motors"] = DEFAULTS["swap_motors"]
                    values["swap_encoders"] = DEFAULTS["swap_encoders"]
                    self._append_log(
                        "[设置] 已按实车接线关闭左右电机交换；编码器保持原通道"
                    )
                if legacy_aggressive:
                    self._append_log(
                        "[设置] 检测到激进参数；可点击“恢复安全默认参数”"
                    )
        except (OSError, ValueError) as exc:
            self._append_log(f"[设置] 无法读取历史设置：{exc}")
        for key, value in values.items():
            if key in self.vars:
                self.vars[key].set(value)
            if key in self.vision_vars:
                self.vision_vars[key].set(value)

    def _collect_vision_params(self) -> dict[str, object]:
        values = {
            key: bool(var.get()) if isinstance(var, tk.BooleanVar) else int(var.get())
            for key, var in self.vision_vars.items()
        }
        return values

    def _collect_settings(self) -> dict[str, object]:
        values: dict[str, object] = {}
        specs = {
            key: (label, kind, lo, hi)
            for _group, fields in FIELD_GROUPS
            for key, label, kind, lo, hi in fields
        }
        for key, var in self.vars.items():
            if isinstance(var, tk.BooleanVar):
                values[key] = bool(var.get())
                continue
            text = str(var.get()).strip()
            label, kind, lo, hi = specs[key]
            if kind is str:
                if key in (
                    "board_ip",
                    "board_user",
                    "wsl_distribution",
                    "environment_root",
                ) and not text:
                    raise ValueError(f"“{label}”不能为空")
                values[key] = text
                continue
            try:
                value = kind(text)
            except ValueError as exc:
                raise ValueError(
                    f"“{label}”输入的“{text}”不是有效数字"
                ) from exc
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError(f"“{label}”必须是有限数字")
            values[key] = value
        identity = str(values["identity_file"])
        if identity and not Path(identity).is_file():
            raise ValueError("SSH 私钥文件不存在")
        values.update(self._collect_vision_params())
        return values

    @staticmethod
    def _validate_environment_root(environment_root: str) -> None:
        root = Path(environment_root)
        if not root.is_dir():
            raise ValueError(f"LoongArch 环境目录不存在：{root}")
        required_paths = (
            root / TOOLCHAIN_NAME / "bin" / "loongarch64-linux-gnu-g++",
            root / "LQ_Dep_libs" / "opencv_install",
            root / "LQ_Dep_libs" / "ncnn_install",
        )
        missing = next((path for path in required_paths if not path.exists()), None)
        if missing is not None:
            raise ValueError(f"LoongArch 编译环境不完整，缺少：{missing}")

    def _bind_autosave(self) -> None:
        for var in self.vars.values():
            var.trace_add("write", self._schedule_autosave)

    def _update_outer_speed_limit_preview(self) -> None:
        try:
            max_speed = float(self.vars["max_speed"].get())
            scale = float(
                self.vars["hard_turn_outer_speed_scale"].get()
            )
            if not math.isfinite(max_speed) or not math.isfinite(scale):
                raise ValueError
            self.outer_speed_limit_var.set(
                f"⇒ {max_speed * scale:.1f} cm/s"
            )
        except (TypeError, ValueError, tk.TclError):
            self.outer_speed_limit_var.set("⇒ 输入完成后计算")

    def _schedule_autosave(self, *_args: object) -> None:
        self._update_outer_speed_limit_preview()
        if self.autosave_after_id is not None:
            self.root.after_cancel(self.autosave_after_id)
        self.autosave_after_id = self.root.after(
            600, self._autosave_settings
        )

    def _autosave_settings(self) -> None:
        self.autosave_after_id = None
        values = self._save_settings(quiet=True, show_errors=False)
        if values is not None:
            self.status_var.set("参数已自动保存")

    def _save_settings(
        self,
        quiet: bool = False,
        show_errors: bool = True,
    ) -> dict[str, object] | None:
        try:
            values = self._collect_settings()
            values["_settings_version"] = SETTINGS_VERSION
            self.settings_path.parent.mkdir(parents=True, exist_ok=True)
            self.settings_path.write_text(
                json.dumps(values, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
        except (OSError, ValueError) as exc:
            if show_errors:
                messagebox.showerror("参数错误", str(exc), parent=self.root)
            elif not quiet:
                self.status_var.set(f"参数尚未保存：{exc}")
            return None
        if not quiet:
            self.status_var.set(f"参数已保存：{self.settings_path}")
        return values

    def _restore_defaults(self) -> None:
        for key, value in DEFAULTS.items():
            if key in self.vars:
                self.vars[key].set(value)
            if key in self.vision_vars:
                self.vision_vars[key].set(value)
        self._save_settings(quiet=True)
        self.status_var.set("已恢复安全默认参数")

    def _powershell(self) -> str:
        return (
            shutil.which("powershell.exe")
            or shutil.which("pwsh.exe")
            or "powershell.exe"
        )

    def _common_connection_args(self, values: dict[str, object]) -> list[str]:
        args = [
            "-BoardIP",
            str(values["board_ip"]),
            "-BoardUser",
            str(values["board_user"]),
        ]
        identity = str(values["identity_file"])
        if identity:
            args.extend(["-IdentityFile", identity])
        return args

    def _deploy(self) -> None:
        values = self._save_settings(quiet=True)
        if values is None:
            return
        # 原生 Windows 构建链（build_rewrite.ps1）自动解析工具链与依赖，
        # 不再依赖 lq环境配置 目录，因此不做环境目录阻断校验。
        if not self.deploy_script.is_file():
            messagebox.showerror(
                "无法部署",
                f"未找到部署脚本：{self.deploy_script}",
                parent=self.root,
            )
            return
        command = [
            self._powershell(),
            "-NoProfile",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            str(self.deploy_script),
            *self._common_connection_args(values),
            "-WslDistribution",
            str(values["wsl_distribution"]),
            "-EnvironmentRoot",
            str(values["environment_root"]),
            "-RunMode",
            "None",
            "-Jobs",
            str(values["jobs"]),
        ]
        if values["force_rebuild"]:
            command.append("-ForceRebuild")
        if values["skip_models"]:
            command.append("-SkipModels")
        self._launch_process("deploy", command)

    def _start_run(self) -> None:
        values = self._save_settings(quiet=True)
        if values is None:
            return
        if not self.run_script.is_file():
            messagebox.showerror(
                "无法启动",
                f"未找到运行脚本：{self.run_script}",
                parent=self.root,
            )
            return
        # if not values["dry_run"] and not messagebox.askyesno(
        #     "确认启动电机",
        #     "确认车辆已放在赛道起点、前方无人且可随时按紧急停车？",
        #     icon=messagebox.WARNING,
        #     parent=self.root,
        # ):
        #     return
        command = [
            self._powershell(),
            "-NoProfile",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            str(self.run_script),
            *self._common_connection_args(values),
        ]
        for key, argument in RUN_ARGUMENTS:
            command.extend([argument, str(values[key])])
        if values["dry_run"]:
            command.append("-DryRun")
        if not values["side_road"]:
            command.append("-NoSideRoad")
        if not values["tof_slope"]:
            command.append("-NoTofSlope")
        if values["disable_line_lost_stop"]:
            command.append("-NoStop")
        if values["swap_motors"]:
            command.append("-SwapMotors")
        if values["swap_encoders"]:
            command.append("-SwapEncoders")
        if values["imu_accel_swap_xy"]:
            command.append("-ImuAccelSwapXY")
        if not values["imu_stationary_zero"]:
            command.append("-NoImuStationaryZero")
        self.run_started_at = time.time()
        self.last_csv = None
        self.last_png = None
        self._launch_process("run", command, interactive=True)
        if self.process is not None and self.process.poll() is None:
            if values["disable_video"]:
                self._stop_video(clear=True)
                self.video_status_var.set("图传已禁用；遥测与轨迹记录仍在运行")
                self.video_label.configure(
                    image="",
                    text="本次运行已禁用实时图传",
                )
            else:
                self._start_video(str(values["board_ip"]))
            self._start_steering_stream(str(values["board_ip"]))

    def _launch_process(
        self, kind: str, command: list[str], interactive: bool = False
    ) -> None:
        if self.process is not None and self.process.poll() is None:
            messagebox.showwarning(
                "任务进行中", "请先等待当前任务结束", parent=self.root
            )
            return
        self.process_kind = kind
        self.status_var.set(
            {"deploy": "正在编译并上传…", "run": "正在启动并记录…", "plot": "正在绘图…"}[
                kind
            ]
        )
        self._append_log(
            f"\n===== {self.status_var.get()} =====\n"
            + " ".join(f'"{part}"' if " " in part else part for part in command)
        )
        creation_flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
        try:
            self.process = subprocess.Popen(
                command,
                cwd=self.repo_root,
                stdin=subprocess.PIPE if interactive else subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                creationflags=creation_flags,
            )
        except OSError as exc:
            self.process = None
            messagebox.showerror("启动失败", str(exc), parent=self.root)
            self.status_var.set("启动失败")
            return
        self._refresh_button_state()
        thread = threading.Thread(
            target=self._read_process_output,
            args=(self.process, kind),
            daemon=True,
        )
        thread.start()

    def _read_process_output(
        self, process: subprocess.Popen[bytes], kind: str
    ) -> None:
        assert process.stdout is not None
        for raw in iter(process.stdout.readline, b""):
            line = decode_output(raw)
            self.events.put(("log", line))
            self._capture_output_path(line)
        return_code = process.wait()
        self.events.put(("finished", (kind, return_code, process)))

    def _capture_output_path(self, line: str) -> None:
        clean = re.sub(r"\x1b\[[0-9;]*m", "", line).strip()
        for prefix, event_name in (("CSV:", "csv"), ("PNG:", "png")):
            if clean.startswith(prefix):
                path = Path(clean[len(prefix) :].strip())
                self.events.put((event_name, path))

    def _vision_endpoint(self) -> str:
        board_ip = str(self.vars["board_ip"].get()).strip()
        if not board_ip:
            raise ValueError("主板 IP 不能为空")
        return f"http://{board_ip}:8080/vision-params"

    def _wheel_endpoint(self) -> str:
        board_ip = str(self.vars["board_ip"].get()).strip()
        if not board_ip:
            raise ValueError("主板 IP 不能为空")
        return f"http://{board_ip}:8080/wheel-box"

    def _connect_video_tuning(self) -> None:
        try:
            board_ip = str(self.vars["board_ip"].get()).strip()
            if not board_ip:
                raise ValueError("主板 IP 不能为空")
        except ValueError as exc:
            messagebox.showerror("连接失败", str(exc), parent=self.root)
            return
        self._start_video(board_ip)

    def _refresh_vision_params(self) -> None:
        try:
            url = self._vision_endpoint()
        except ValueError as exc:
            messagebox.showerror("参数错误", str(exc), parent=self.root)
            return
        self.vision_status_var.set("正在读取车端视觉参数…")
        threading.Thread(
            target=self._vision_request_worker,
            args=(url, None, False),
            daemon=True,
        ).start()

    def _apply_vision_params(self, silent: bool = False) -> None:
        try:
            url = self._vision_endpoint()
            local = self._collect_vision_params()
        except (ValueError, tk.TclError) as exc:
            if not silent:
                messagebox.showerror("视觉参数错误", str(exc), parent=self.root)
            return
        payload = {
            key.removeprefix("vision_"): int(value)
            if isinstance(value, bool)
            else value
            for key, value in local.items()
        }
        self.vision_status_var.set("正在应用视觉参数…")
        threading.Thread(
            target=self._vision_request_worker,
            args=(url, payload, silent),
            daemon=True,
        ).start()

    def _vision_request_worker(
        self,
        url: str,
        payload: dict[str, object] | None,
        silent: bool,
    ) -> None:
        try:
            data = None
            method = "GET"
            if payload is not None:
                data = urllib.parse.urlencode(payload).encode("ascii")
                method = "POST"
            request = urllib.request.Request(
                url,
                data=data,
                method=method,
                headers={
                    "User-Agent": "rewrite-control-gui",
                    "Content-Type": "application/x-www-form-urlencoded",
                },
            )
            with urllib.request.urlopen(request, timeout=3.0) as response:
                result = json.loads(response.read().decode("utf-8"))
            if not isinstance(result, dict):
                raise ValueError("车端返回的参数格式无效")
            self.events.put(("vision_params", result))
            self.events.put(
                (
                    "vision_status",
                    "视觉参数已应用并由车端确认"
                    if payload is not None
                    else "已读取车端视觉参数",
                )
            )
        except (OSError, ValueError, json.JSONDecodeError, urllib.error.URLError) as exc:
            prefix = "自动应用失败" if silent else "视觉参数通信失败"
            self.events.put(("vision_status", f"{prefix}：{exc}"))

    def _start_video(self, board_ip: str) -> None:
        self._stop_video(clear=False)
        self.video_board_ip = board_ip
        stop = threading.Event()
        self.video_stop = stop
        view_key, view_label, endpoint = self._selected_video_view()
        url = f"http://{board_ip}:8080{endpoint}"
        self.vision_auto_applied = False
        self.wheel_box_synced = False
        self.video_status_var.set(f"正在连接 {url}")
        self.notebook.select(self.video_tab)
        threading.Thread(
            target=self._video_loop,
            args=(url, stop, view_key, view_label),
            daemon=True,
        ).start()

    def _refresh_wheel_box(self) -> None:
        try:
            url = self._wheel_endpoint()
        except ValueError as exc:
            self.wheel_status_var.set(str(exc))
            return
        threading.Thread(
            target=self._wheel_box_request_worker,
            args=(url, None),
            daemon=True,
        ).start()

    def _start_wheel_selection(self) -> None:
        if self.wheel_selecting:
            self.wheel_selecting = False
            self.wheel_drag_start = None
            self.wheel_drag_current = None
            self.wheel_select_button.configure(text="手动框选车轮")
            self.video_label.configure(cursor="")
            self.wheel_status_var.set("已取消手动框选")
            self._render_video_frame()
            return
        if self.video_frame is None:
            messagebox.showinfo(
                "手动框选车轮",
                "请先连接图传并等待画面出现。",
                parent=self.root,
            )
            return
        self.wheel_selecting = True
        self.wheel_drag_start = None
        self.wheel_drag_current = None
        self.wheel_select_button.configure(text="取消框选")
        self.video_label.configure(cursor="crosshair")
        self.wheel_status_var.set("请在实时画面中按住左键拖框，完整包住车轮")

    def _video_image_point(self, event: tk.Event) -> tuple[int, int] | None:
        if self.video_frame is None:
            return None
        image_width, image_height = self.video_frame.size
        label_width = self.video_label.winfo_width()
        label_height = self.video_label.winfo_height()
        offset_x = (label_width - image_width) // 2
        offset_y = (label_height - image_height) // 2
        x = int(event.x) - offset_x
        y = int(event.y) - offset_y
        if x < 0 or y < 0 or x >= image_width or y >= image_height:
            return None
        return x, y

    def _wheel_select_press(self, event: tk.Event) -> None:
        if not self.wheel_selecting:
            return
        point = self._video_image_point(event)
        if point is None:
            return
        self.wheel_drag_start = point
        self.wheel_drag_current = point

    def _wheel_select_motion(self, event: tk.Event) -> None:
        if not self.wheel_selecting or self.wheel_drag_start is None:
            return
        point = self._video_image_point(event)
        if point is None:
            return
        self.wheel_drag_current = point
        self._render_video_frame()

    def _wheel_select_release(self, event: tk.Event) -> None:
        if not self.wheel_selecting or self.wheel_drag_start is None:
            return
        point = self._video_image_point(event)
        if point is not None:
            self.wheel_drag_current = point
        if self.wheel_drag_current is None or self.video_frame is None:
            return
        x0, y0 = self.wheel_drag_start
        x1, y1 = self.wheel_drag_current
        left, right = sorted((x0, x1))
        top, bottom = sorted((y0, y1))
        width, height = self.video_frame.size
        if right - left < max(4, int(width * 0.02)) or \
                bottom - top < max(4, int(height * 0.02)):
            self.wheel_status_var.set("框选范围太小，请重新拖框")
            self.wheel_drag_start = None
            self.wheel_drag_current = None
            self._render_video_frame()
            return
        ratios = (
            left / max(1, width - 1),
            top / max(1, height - 1),
            right / max(1, width - 1),
            bottom / max(1, height - 1),
        )
        self.wheel_box_ratio = ratios
        self.wheel_selecting = False
        self.wheel_drag_start = None
        self.wheel_drag_current = None
        self.wheel_select_button.configure(text="手动框选车轮")
        self.video_label.configure(cursor="")
        self.wheel_status_var.set("正在把手动车轮框应用到车端…")
        self._render_video_frame()
        try:
            url = self._wheel_endpoint()
        except ValueError as exc:
            self.wheel_status_var.set(str(exc))
            return
        payload = dict(zip(("left", "top", "right", "bottom"), ratios))
        threading.Thread(
            target=self._wheel_box_request_worker,
            args=(url, payload),
            daemon=True,
        ).start()

    def _wheel_box_request_worker(
        self,
        url: str,
        payload: dict[str, float] | None,
    ) -> None:
        try:
            data = None
            method = "GET"
            if payload is not None:
                data = urllib.parse.urlencode(payload).encode("ascii")
                method = "POST"
            request = urllib.request.Request(
                url,
                data=data,
                method=method,
                headers={
                    "User-Agent": "rewrite-control-gui",
                    "Content-Type": "application/x-www-form-urlencoded",
                },
            )
            with urllib.request.urlopen(request, timeout=3.0) as response:
                result = json.loads(response.read().decode("utf-8"))
            if not isinstance(result, dict):
                raise ValueError("车端返回的车轮框格式无效")
            self.events.put(("wheel_box", result))
        except (OSError, ValueError, json.JSONDecodeError, urllib.error.URLError) as exc:
            self.events.put(("wheel_status", f"车轮框通信失败：{exc}"))

    def _selected_video_view(self) -> tuple[str, str, str]:
        selected = self.video_view_var.get()
        for view in VIDEO_VIEWS:
            if view[1] == selected:
                return view
        return VIDEO_VIEWS[0]

    def _on_video_view_changed(self, _event: object = None) -> None:
        board_ip = self.video_board_ip
        if board_ip:
            self._start_video(board_ip)

    def _stop_video(self, clear: bool = False) -> None:
        if self.video_stop is not None:
            self.video_stop.set()
            self.video_stop = None
        self.video_board_ip = ""
        self.video_status_var.set(
            "图传未连接" if clear else "图传已停止（保留最后一帧）"
        )
        if clear:
            self.video_photo = None
            self.video_frame = None
            self.video_label.configure(
                image="",
                text="启动车辆后可切换实时叠加、灰度、最长白线寻路和饱和度画面",
            )

    def _video_loop(
        self,
        url: str,
        stop: threading.Event,
        view_key: str,
        view_label: str,
    ) -> None:
        if Image is None:
            self.events.put(
                ("video_error", (view_key, "缺少 Pillow，无法解码 MJPEG 图传"))
            )
            return
        while not stop.is_set():
            try:
                request = urllib.request.Request(
                    url,
                    headers={"User-Agent": "rewrite-control-gui"},
                )
                with urllib.request.urlopen(request, timeout=3.0) as response:
                    self.events.put(
                        ("video_status", (view_key, f"{view_label}：{url}"))
                    )
                    self.events.put(("vision_connected", url))
                    buffer = bytearray()
                    while not stop.is_set():
                        chunk = response.read(8192)
                        if not chunk:
                            raise ConnectionError("图传连接已关闭")
                        buffer.extend(chunk)
                        start = buffer.find(b"\xff\xd8")
                        end = buffer.find(b"\xff\xd9", start + 2)
                        if start >= 0 and end > start:
                            jpeg = bytes(buffer[start : end + 2])
                            del buffer[: end + 2]
                            with Image.open(io.BytesIO(jpeg)) as decoded:
                                frame = decoded.convert("RGB")
                            frame.thumbnail((960, 720), Image.Resampling.LANCZOS)
                            self.events.put(("video_frame", (view_key, frame)))
                        elif len(buffer) > 2 * 1024 * 1024:
                            buffer.clear()
            except (OSError, ValueError) as exc:
                if stop.is_set():
                    break
                self.events.put(
                    ("video_status", (view_key, f"{view_label}重连中：{exc}"))
                )
                stop.wait(0.5)

    def _start_steering_stream(self, board_ip: str) -> None:
        self._stop_steering_stream(clear=True)
        stop = threading.Event()
        self.steering_stop = stop
        url = f"http://{board_ip}:8080/telemetry"
        self.steering_status_var.set(f"正在连接 {url}")
        threading.Thread(
            target=self._steering_loop,
            args=(url, stop),
            daemon=True,
        ).start()

    def _stop_steering_stream(self, clear: bool = False) -> None:
        if self.steering_stop is not None:
            self.steering_stop.set()
            self.steering_stop = None
        if clear:
            self.steering_samples.clear()
            self._draw_steering_waveforms()
        self.steering_status_var.set(
            "舵量遥测未连接" if clear else "舵量遥测已停止（保留最后波形）"
        )

    def _steering_loop(self, url: str, stop: threading.Event) -> None:
        while not stop.is_set():
            try:
                request = urllib.request.Request(
                    url,
                    headers={"User-Agent": "rewrite-control-gui"},
                )
                with urllib.request.urlopen(request, timeout=3.0) as response:
                    self.events.put(("steering_status", (stop, f"实时舵量：{url}")))
                    while not stop.is_set():
                        raw = response.readline()
                        if not raw:
                            raise ConnectionError("舵量遥测连接已关闭")
                        sample = json.loads(raw.decode("utf-8"))
                        sample["target_wheel_diff_cmps"] = (
                            float(sample.get("left_target_cmps", 0.0))
                            - float(sample.get("right_target_cmps", 0.0))
                        )
                        sample["actual_wheel_diff_cmps"] = (
                            float(sample.get("left_speed_cmps", 0.0))
                            - float(sample.get("right_speed_cmps", 0.0))
                        )
                        self.events.put(("steering_sample", (stop, time.monotonic(), sample)))
            except (OSError, ValueError, json.JSONDecodeError) as exc:
                if stop.is_set():
                    break
                self.events.put(("steering_status", (stop, f"舵量遥测重连中：{exc}")))
                stop.wait(0.5)

    def _on_live_telemetry(self, sample: dict[str, object], received_at: float) -> None:
        """Allow panels to consume live samples without using historical CSV data."""

    @staticmethod
    def _sample_number(sample: dict[str, object], key: str) -> float | None:
        value = sample.get(key)
        if value in (None, ""):
            return None
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        return number if math.isfinite(number) else None

    def _draw_waveform(
        self,
        canvas: tk.Canvas,
        title: str,
        unit: str,
        series: list[tuple[str, str, str, tuple[int, ...] | None]],
    ) -> None:
        canvas.delete("all")
        width = max(320, canvas.winfo_width())
        height = max(120, canvas.winfo_height())
        left, right, top, bottom = 54, 12, 26, 24
        plot_width = max(1, width - left - right)
        plot_height = max(1, height - top - bottom)
        samples = list(self.steering_samples)
        canvas.create_text(
            8, 8, anchor="nw", fill="#e6edf3",
            font=("Microsoft YaHei UI", 9, "bold"), text=f"{title} ({unit})"
        )
        if len(samples) < 2:
            canvas.create_text(
                width / 2, height / 2, fill="#8b949e",
                text="等待遥测数据"
            )
            return
        times = [
            self._sample_number(sample, "elapsed_s") or 0.0
            for sample in samples
        ]
        end_time = max(times)
        start_time = max(min(times), end_time - 20.0)
        visible = [
            sample for sample, sample_time in zip(samples, times)
            if sample_time >= start_time
        ]
        values = [
            abs(value)
            for sample in visible
            for key, _label, _color, _dash in series
            for value in [self._sample_number(sample, key)]
            if value is not None
        ]
        scale = max(1.0, max(values, default=1.0) * 1.08)
        zero_y = top + plot_height / 2
        canvas.create_line(left, zero_y, width - right, zero_y, fill="#59636e")
        canvas.create_line(left, top, left, height - bottom, fill="#59636e")
        canvas.create_text(
            left - 5, top, anchor="e", fill="#8b949e", text=f"{scale:.0f}"
        )
        canvas.create_text(
            left - 5, zero_y, anchor="e", fill="#8b949e", text="0"
        )
        canvas.create_text(
            left - 5, height - bottom, anchor="e",
            fill="#8b949e", text=f"{-scale:.0f}"
        )
        duration = max(0.001, end_time - start_time)
        for key, label, color, dash in series:
            points: list[float] = []
            for sample in visible:
                sample_time = self._sample_number(sample, "elapsed_s")
                value = self._sample_number(sample, key)
                if sample_time is None or value is None:
                    continue
                x = left + (sample_time - start_time) / duration * plot_width
                y = zero_y - value / scale * plot_height / 2
                points.extend((x, y))
            if len(points) >= 4:
                canvas.create_line(
                    points, fill=color, width=2, smooth=False,
                    dash=dash or ()
                )
        legend_x = left + 8
        for _key, label, color, dash in series:
            canvas.create_line(
                legend_x, 13, legend_x + 18, 13,
                fill=color, width=2, dash=dash or ()
            )
            canvas.create_text(
                legend_x + 22, 13, anchor="w",
                fill="#c9d1d9", text=label
            )
            legend_x += 28 + len(label) * 12
        canvas.create_text(
            width - right, height - 4, anchor="se",
            fill="#8b949e", text=f"最近 {duration:.1f}s"
        )

    def _draw_steering_waveforms(self) -> None:
        if not hasattr(self, "steering_factor_canvas"):
            return
        self._draw_waveform(
            self.steering_factor_canvas,
            "视觉与状态机打舵分量",
            "°/s",
            [
                ("vision_near_yaw_rate_dps", "近端", "#ff7b72", None),
                ("vision_far_yaw_rate_dps", "远端", "#79c0ff", (6, 3)),
                ("vision_center_yaw_rate_dps", "归中", "#7ee787", None),
                ("vision_integral_yaw_rate_dps", "积分I", "#d2a8ff", (2, 3)),
                ("vision_derivative_yaw_rate_dps", "微分D", "#ffa657", (8, 3)),
                ("state_yaw_adjustment_dps", "后级调整", "#f2cc60", (3, 2)),
            ],
        )
        self._draw_waveform(
            self.steering_yaw_canvas,
            "目标与实际偏航",
            "°/s",
            [
                ("target_yaw_rate_dps", "目标舵量", "#ff7b72", None),
                ("measured_yaw_rate_dps", "实测偏航", "#79c0ff", (6, 3)),
            ],
        )
        self._draw_waveform(
            self.steering_wheel_canvas,
            "目标与实际左右轮差（左减右）",
            "cm/s",
            [
                ("target_wheel_diff_cmps", "目标轮差", "#f2cc60", None),
                ("actual_wheel_diff_cmps", "实际轮差", "#7ee787", (6, 3)),
            ],
        )

    def _request_stop(self) -> None:
        process = self.process
        if (
            process is None
            or process.poll() is not None
            or self.process_kind != "run"
        ):
            self.status_var.set("当前没有由 GUI 启动的运行任务")
            return
        try:
            if process.stdin is not None:
                process.stdin.write(b"\n")
                process.stdin.flush()
            self.status_var.set("正在安全停车、保存 CSV 并绘图…")
            self._append_log("[GUI] 已发送安全停车请求")
        except OSError as exc:
            self._append_log(f"[GUI] 安全停车请求失败：{exc}")
            self._emergency_stop()

    def _emergency_stop(self) -> None:
        values = self._save_settings(quiet=True)
        if values is None:
            return
        self._request_stop_if_possible()
        thread = threading.Thread(
            target=self._run_emergency_stop, args=(values,), daemon=True
        )
        thread.start()
        self.status_var.set("正在执行紧急停车…")

    def _request_stop_if_possible(self) -> None:
        process = self.process
        if (
            process is not None
            and process.poll() is None
            and self.process_kind == "run"
            and process.stdin is not None
        ):
            try:
                process.stdin.write(b"\n")
                process.stdin.flush()
            except OSError:
                pass

    def _run_emergency_stop(self, values: dict[str, object]) -> None:
        remote = f"{values['board_user']}@{values['board_ip']}"
        command = [
            shutil.which("ssh.exe") or "ssh.exe",
            "-o",
            "BatchMode=yes",
            "-o",
            "ConnectTimeout=8",
            "-o",
            "StrictHostKeyChecking=accept-new",
            "-o",
            "HostKeyAlgorithms=+ssh-rsa",
            "-o",
            "PubkeyAcceptedAlgorithms=+ssh-rsa",
        ]
        identity = str(values["identity_file"])
        if identity:
            command.extend(["-i", identity, "-o", "IdentitiesOnly=yes"])
        command.extend(
            [
                remote,
                "killall -INT lq_path_follow_rewrite 2>/dev/null || true; "
                "sleep 1; "
                "killall -TERM lq_path_follow_rewrite 2>/dev/null || true",
            ]
        )
        creation_flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
        result = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            creationflags=creation_flags,
            check=False,
        )
        output = decode_output(result.stdout)
        self.events.put(
            (
                "emergency_finished",
                (result.returncode, output),
            )
        )

    def _choose_csv(self) -> None:
        path = filedialog.askopenfilename(
            title="选择轨迹 CSV",
            initialdir=self.repo_root / "build",
            filetypes=[("CSV", "*.csv"), ("所有文件", "*.*")],
        )
        if not path:
            return
        csv_path = Path(path)
        self.last_csv = csv_path
        self.last_png = None
        self._load_steering_csv(csv_path)
        self._show_csv_path(csv_path)

    @staticmethod
    def _read_csv_rows(csv_path: Path) -> list[dict[str, str]]:
        with csv_path.open("r", newline="", encoding="utf-8-sig") as handle:
            return list(
                csv.DictReader(
                    line
                    for line in handle
                    if line.strip() and not line.lstrip().startswith("#")
                )
            )

    def _load_steering_csv(self, csv_path: Path) -> None:
        try:
            rows = self._read_csv_rows(csv_path)
        except OSError as exc:
            self._append_log(f"[GUI] 无法读取舵量 CSV：{exc}")
            return
        self.steering_samples.clear()
        if not rows or "left_target_cmps" not in rows[0]:
            self.steering_status_var.set(
                f"已载入 IMU 路径：{csv_path.name}（无舵量数据）"
            )
            self._draw_steering_waveforms()
            return
        for row in rows[-self.steering_samples.maxlen :]:
            row["target_wheel_diff_cmps"] = (
                float(row.get("left_target_cmps") or 0.0)
                - float(row.get("right_target_cmps") or 0.0)
            )
            row["actual_wheel_diff_cmps"] = (
                float(row.get("left_speed_cmps") or 0.0)
                - float(row.get("right_speed_cmps") or 0.0)
            )
            self.steering_samples.append(row)
        self.steering_status_var.set(
            f"已载入 {len(self.steering_samples)} 行：{csv_path.name}"
        )
        self._draw_steering_waveforms()

    def _show_csv_path(self, csv_path: Path) -> None:
        try:
            rows = self._read_csv_rows(csv_path)
        except OSError as exc:
            messagebox.showerror(
                "无法读取路径", str(exc), parent=self.root
            )
            return
        if not rows or not all(key in rows[0] for key in ("x_cm", "y_cm")):
            messagebox.showerror(
                "无法显示路径",
                "CSV 缺少 x_cm 或 y_cm 路径坐标。",
                parent=self.root,
            )
            return

        self.path_rows = rows
        self.path_name = csv_path.name
        self.output_var.set(str(csv_path))
        self.image_label.grid_remove()
        self.path_canvas.grid(
            row=1, column=0, sticky="nsew", padx=8, pady=5
        )
        self._update_path_summary(rows)
        self.notebook.select(self.trajectory_tab)
        self.root.after_idle(self._draw_path_canvas)

    def _trajectory_source_changed(self) -> None:
        if self.path_rows:
            self._update_path_summary(self.path_rows)
            self.root.after_idle(self._draw_path_canvas)

    @staticmethod
    def _path_number(
        row: dict[str, str], key: str, default: float = 0.0
    ) -> float:
        try:
            value = float(row.get(key, default))
        except (TypeError, ValueError):
            return default
        return value if math.isfinite(value) else default

    def _update_path_summary(self, rows: list[dict[str, str]]) -> None:
        direct_imu = "route_distance_cm" in rows[0]
        distance_key = "route_distance_cm" if direct_imu else "distance_cm"
        speed_key = "speed_cmps" if direct_imu else "limited_speed_cmps"
        first = rows[0]
        last = rows[-1]
        duration = max(
            0.0,
            self._path_number(last, "elapsed_s")
            - self._path_number(first, "elapsed_s"),
        )
        distance = max(
            0.0,
            self._path_number(last, distance_key)
            - self._path_number(first, distance_key),
        )
        source = self.trajectory_source_var.get()
        selected = self._trajectory_points(rows, source)
        if len(selected) >= 2:
            displacement = math.hypot(
                selected[-1][0] - selected[0][0],
                selected[-1][1] - selected[0][1],
            )
        else:
            displacement = 0.0
        max_speed = max(
            (abs(self._path_number(row, speed_key)) for row in rows),
            default=0.0,
        )
        lines = [
            f"数据源：{'直接 IMU 惯导' if direct_imu else 'IMU 航向 + 编码器里程融合'}",
            f"路径视图：{ {'overlay': '融合路径 + 纯 IMU 诊断', 'odometry': 'IMU/编码器融合路径', 'imu': '纯 IMU 加速度积分诊断'} .get(source, source)}",
            f"采样点：{len(rows)}",
            f"持续时间：{duration:.3f} s",
            f"累计路径：{distance:.3f} cm",
            f"起终点位移：{displacement:.3f} cm",
            f"最大速度：{max_speed:.3f} cm/s",
        ]
        if direct_imu:
            stationary = sum(
                str(row.get("imu_stationary", "")).lower()
                in {"1", "true"}
                for row in rows
            )
            max_accel = max(
                (
                    math.hypot(
                        self._path_number(row, "forward_accel_mps2"),
                        self._path_number(row, "right_accel_mps2"),
                    )
                    for row in rows
                ),
                default=0.0,
            )
            lines.extend(
                [
                    f"静止归零：{stationary}/{len(rows)} "
                    f"({stationary / len(rows) * 100:.1f}%)",
                    f"最大平面加速度：{max_accel:.4f} m/s²",
                ]
            )
        else:
            imu_samples = self._local_imu_samples(rows)
            if len(imu_samples) >= 2:
                imu_extent = max(
                    math.hypot(point[0], point[1])
                    for _, point in imu_samples
                )
                fused_distance = max(distance, 1.0)
                lines.append(f"纯 IMU 最大漂移半径：{imu_extent:.3f} cm")
                if imu_extent > max(500.0, fused_distance * 8.0):
                    lines.append(
                        "诊断：纯 IMU 双积分漂移过大，不可作为赛道路径；"
                        "请使用 IMU/编码器融合路径"
                    )
        self.summary.configure(state=tk.NORMAL)
        self.summary.delete("1.0", tk.END)
        self.summary.insert(tk.END, "\n".join(lines))
        self.summary.configure(state=tk.DISABLED)

    @staticmethod
    def _speed_color(speed: float, maximum: float) -> str:
        ratio = 0.0 if maximum <= 1e-9 else min(1.0, speed / maximum)
        start = (38, 111, 160)
        end = (204, 73, 58)
        rgb = tuple(
            round(low + (high - low) * ratio)
            for low, high in zip(start, end)
        )
        return "#{:02x}{:02x}{:02x}".format(*rgb)

    def _trajectory_points(
        self,
        rows: list[dict[str, str]],
        source: str,
    ) -> list[tuple[float, float]]:
        return [point for _, point in self._trajectory_samples(rows, source)]

    def _trajectory_samples(
        self,
        rows: list[dict[str, str]],
        source: str,
    ) -> list[tuple[dict[str, str], tuple[float, float]]]:
        direct_imu = bool(rows) and "route_distance_cm" in rows[0]
        if source == "imu" and not direct_imu:
            return self._local_imu_samples(rows)
        y_scale = 1.0 if direct_imu or str(
            rows[0].get("map_y_positive", "")
        ).lower() == "right" else -1.0
        return [
            (
                row,
                (
                    self._path_number(row, "x_cm"),
                    y_scale * self._path_number(row, "y_cm"),
                ),
            )
            for row in rows
        ]

    def _local_imu_samples(
        self,
        rows: list[dict[str, str]],
    ) -> list[tuple[dict[str, str], tuple[float, float]]]:
        """Convert world-frame IMU integration to the recording-start frame."""
        valid_rows = [
            row for row in rows
            if str(row.get("imu_valid", "")).lower() in {"1", "true"}
        ]
        if not valid_rows:
            return []

        origin = valid_rows[0]
        origin_x_m = self._path_number(origin, "imu_position_x_m")
        origin_y_m = self._path_number(origin, "imu_position_y_m")
        origin_vx_mps = self._path_number(origin, "imu_velocity_x_mps")
        origin_vy_mps = self._path_number(origin, "imu_velocity_y_mps")
        origin_heading_deg = self._path_number(origin, "imu_heading_deg")
        angle = math.radians(origin_heading_deg)
        cosine = math.cos(angle)
        sine = math.sin(angle)

        samples = []
        for row in valid_rows:
            dx_m = self._path_number(row, "imu_position_x_m") - origin_x_m
            dy_m = self._path_number(row, "imu_position_y_m") - origin_y_m
            dvx_mps = self._path_number(row, "imu_velocity_x_mps") - origin_vx_mps
            dvy_mps = self._path_number(row, "imu_velocity_y_mps") - origin_vy_mps
            local_row = dict(row)
            local_row["_imu_local_heading_deg"] = str(
                self._wrap_degrees(
                    self._path_number(row, "imu_heading_deg")
                    - origin_heading_deg
                )
            )
            local_row["_imu_local_speed_cmps"] = str(
                100.0 * math.hypot(dvx_mps, dvy_mps)
            )
            samples.append((
                local_row,
                (
                    100.0 * (cosine * dx_m + sine * dy_m),
                    100.0 * (-sine * dx_m + cosine * dy_m),
                ),
            ))
        return samples

    @staticmethod
    def _wrap_degrees(angle: float) -> float:
        return (angle + 180.0) % 360.0 - 180.0

    def _trajectory_speed(self, row: dict[str, str], source: str) -> float:
        direct_imu = "route_distance_cm" in row
        if source == "imu" and not direct_imu:
            return self._path_number(row, "_imu_local_speed_cmps")
        return abs(self._path_number(
            row, "speed_cmps" if direct_imu else "limited_speed_cmps"
        ))

    def _draw_path_canvas(self) -> None:
        canvas = self.path_canvas
        canvas.delete("all")
        rows = self.path_rows
        width = max(2, canvas.winfo_width())
        height = max(2, canvas.winfo_height())
        if not rows:
            canvas.create_text(
                width / 2,
                height / 2,
                text="尚未载入路径 CSV",
                fill="#59636e",
                font=("Microsoft YaHei UI", 11),
            )
            return

        source = self.trajectory_source_var.get()
        odometry_samples = self._trajectory_samples(rows, "odometry")
        imu_samples = self._trajectory_samples(rows, "imu")
        primary_samples = (
            imu_samples if source == "imu" and len(imu_samples) >= 2
            else odometry_samples
        )
        if source == "imu" and len(imu_samples) < 2:
            source = "odometry"
            self.trajectory_source_var.set(source)
        odometry_points = [point for _, point in odometry_samples]
        imu_points = [point for _, point in imu_samples]
        if source == "overlay" and len(imu_samples) >= 2:
            all_points = odometry_points + imu_points
        else:
            all_points = [point for _, point in primary_samples]
        if len(all_points) < 2:
            canvas.create_text(
                width / 2, height / 2,
                text="CSV 没有可用的 IMU 路径坐标",
                fill="#59636e",
                font=("Microsoft YaHei UI", 11),
            )
            return
        primary_rows = [row for row, _ in primary_samples]
        points = [point for _, point in primary_samples]
        speeds = [
            self._trajectory_speed(row, source) for row in primary_rows
        ]
        max_speed = max(speeds, default=0.0)
        xs = [point[0] for point in all_points]
        ys = [point[1] for point in all_points]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        span_x = max_x - min_x
        span_y = max_y - min_y
        if span_x < 1e-6:
            min_x -= 5.0
            max_x += 5.0
            span_x = 10.0
        if span_y < 1e-6:
            min_y -= 5.0
            max_y += 5.0
            span_y = 10.0
        margin = 58.0
        scale = max(
            0.01,
            min(
                max(1.0, width - 2 * margin) / span_x,
                max(1.0, height - 2 * margin) / span_y,
            ),
        )
        plot_width = span_x * scale
        plot_height = span_y * scale
        left = (width - plot_width) / 2
        top = (height - plot_height) / 2

        def screen(point: tuple[float, float]) -> tuple[float, float]:
            return (
                left + (point[0] - min_x) * scale,
                top + plot_height - (point[1] - min_y) * scale,
            )

        for index in range(6):
            fraction = index / 5
            x = left + plot_width * fraction
            y = top + plot_height * fraction
            canvas.create_line(
                x, top, x, top + plot_height, fill="#e1e5e9"
            )
            canvas.create_line(
                left, y, left + plot_width, y, fill="#e1e5e9"
            )
            canvas.create_text(
                x,
                top + plot_height + 18,
                text=f"{min_x + span_x * fraction:.1f}",
                fill="#68727d",
                font=("Segoe UI", 8),
            )
            canvas.create_text(
                left - 28,
                top + plot_height - plot_height * fraction,
                text=f"{min_y + span_y * fraction:.1f}",
                fill="#68727d",
                font=("Segoe UI", 8),
            )
        canvas.create_rectangle(
            left,
            top,
            left + plot_width,
            top + plot_height,
            outline="#aeb6bf",
        )
        canvas.create_text(
            left + plot_width / 2,
            height - 14,
            text="X / cm",
            fill="#39434d",
            font=("Microsoft YaHei UI", 9),
        )
        canvas.create_text(
            15,
            top + plot_height / 2,
            text="Y\n/\ncm",
            fill="#39434d",
            font=("Microsoft YaHei UI", 9),
        )

        screen_points = [screen(point) for point in points]
        for index in range(len(screen_points) - 1):
            canvas.create_line(
                *screen_points[index],
                *screen_points[index + 1],
                fill=self._speed_color(speeds[index], max_speed),
                width=3,
            )
        if source == "overlay" and len(imu_points) >= 2:
            imu_screen = [screen(point) for point in imu_points]
            for index in range(len(imu_screen) - 1):
                canvas.create_line(
                    *imu_screen[index],
                    *imu_screen[index + 1],
                    fill="#8e44ad",
                    width=2,
                    dash=(7, 4),
                )
        arrow_step = max(1, len(screen_points) // 20)
        for index in range(0, len(screen_points), arrow_step):
            x, y = screen_points[index]
            heading_key = (
                "_imu_local_heading_deg"
                if source == "imu"
                and "route_distance_cm" not in primary_rows[index]
                else "heading_deg"
            )
            heading = math.radians(
                self._path_number(primary_rows[index], heading_key)
            )
            canvas.create_line(
                x,
                y,
                x + math.cos(heading) * 13,
                y - math.sin(heading) * 13,
                fill="#17202a",
                width=1,
                arrow=tk.LAST,
                arrowshape=(5, 6, 2),
            )
        start_x, start_y = screen_points[0]
        end_x, end_y = screen_points[-1]
        canvas.create_oval(
            start_x - 6,
            start_y - 6,
            start_x + 6,
            start_y + 6,
            fill="#2d9b59",
            outline="white",
            width=2,
        )
        canvas.create_rectangle(
            end_x - 6,
            end_y - 6,
            end_x + 6,
            end_y + 6,
            fill="#cf4034",
            outline="white",
            width=2,
        )
        canvas.create_text(
            left,
            max(12, top - 22),
            anchor="w",
            text=(
                f"{self.path_name}   {len(rows)} 点   "
                f"{source}   最大速度 {max_speed:.2f} cm/s"
            ),
            fill="#26313b",
            font=("Microsoft YaHei UI", 10, "bold"),
        )

    def _open_output_dir(self) -> None:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        if os.name == "nt":
            os.startfile(self.output_dir)  # type: ignore[attr-defined]
        else:
            subprocess.Popen(["xdg-open", str(self.output_dir)])

    def _append_log(self, line: str) -> None:
        self.log.configure(state=tk.NORMAL)
        self.log.insert(tk.END, line + "\n")
        self.log.see(tk.END)
        self.log.configure(state=tk.DISABLED)

    def _poll_events(self) -> None:
        latest_video_frame = None
        steering_changed = False
        try:
            while True:
                event, payload = self.events.get_nowait()
                if event == "log":
                    self._append_log(str(payload))
                elif event == "csv":
                    self.last_csv = Path(payload)
                elif event == "png":
                    self.last_png = Path(payload)
                elif event == "finished":
                    kind, code, process = payload  # type: ignore[misc]
                    self._handle_finished(str(kind), int(code), process)
                elif event == "emergency_finished":
                    code, output = payload  # type: ignore[misc]
                    if output:
                        self._append_log(str(output))
                    self.status_var.set(
                        "紧急停车命令已发送"
                        if int(code) == 0
                        else f"紧急停车 SSH 失败，退出码 {code}"
                    )
                elif event == "video_status":
                    view_key, text = payload  # type: ignore[misc]
                    if view_key == self._selected_video_view()[0]:
                        self.video_status_var.set(str(text))
                elif event == "video_error":
                    view_key, text = payload  # type: ignore[misc]
                    if view_key == self._selected_video_view()[0]:
                        self.video_status_var.set(str(text))
                elif event == "video_frame":
                    view_key, frame = payload  # type: ignore[misc]
                    if view_key == self._selected_video_view()[0]:
                        latest_video_frame = frame
                elif event == "vision_connected":
                    if not self.vision_auto_applied:
                        self.vision_auto_applied = True
                        self._apply_vision_params(silent=True)
                    if not self.wheel_box_synced:
                        self.wheel_box_synced = True
                        self._refresh_wheel_box()
                elif event == "vision_status":
                    self.vision_status_var.set(str(payload))
                elif event == "vision_params":
                    if isinstance(payload, dict):
                        for key, value in payload.items():
                            gui_key = f"vision_{key}"
                            if gui_key in self.vision_vars:
                                self.vision_vars[gui_key].set(value)
                elif event == "wheel_status":
                    self.wheel_status_var.set(str(payload))
                elif event == "wheel_box":
                    if isinstance(payload, dict):
                        try:
                            self.wheel_box_ratio = tuple(
                                float(payload[key])
                                for key in ("left", "top", "right", "bottom")
                            )  # type: ignore[assignment]
                            source = str(payload.get("source", "unknown"))
                            source_text = {
                                "auto": "启动自动识别",
                                "manual_or_fallback": "手动框或静态回退",
                            }.get(source, source)
                            self.wheel_status_var.set(
                                f"车轮框已同步（{source_text}）："
                                f"x={int(payload.get('left_px', 0))}.."
                                f"{int(payload.get('right_px', 0))}，"
                                f"y={int(payload.get('top_px', 0))}.."
                                f"{int(payload.get('bottom_px', 0))}"
                            )
                            self._render_video_frame()
                        except (KeyError, TypeError, ValueError):
                            self.wheel_status_var.set("车端返回的车轮框无效")
                elif event == "steering_status":
                    source, text = payload
                    if source is self.steering_stop and not source.is_set():
                        self.steering_status_var.set(str(text))
                elif event == "steering_sample":
                    source, received_at, sample = payload
                    if source is not self.steering_stop or source.is_set():
                        continue
                    if isinstance(sample, dict):
                        self._on_live_telemetry(sample, received_at)
                        self.steering_samples.append(sample)
                        steering_changed = True
                        target_util = self._sample_number(
                            sample, "target_steering_utilization"
                        ) or 0.0
                        pwm_util = self._sample_number(
                            sample, "pwm_steering_utilization"
                        ) or 0.0
                        state = str(sample.get("state", "?"))
                        steering_text = (
                            f"状态 {state}｜目标舵量 {target_util * 100:.0f}%"
                            f"{'（已打满）' if target_util >= 0.98 else ''}"
                            f"｜PWM轮差 {pwm_util * 100:.0f}%"
                        )
                        self.steering_status_var.set(steering_text)
        except queue.Empty:
            pass
        if latest_video_frame is not None:
            self._show_video_frame(latest_video_frame)
        if steering_changed:
            self._draw_steering_waveforms()
        self.root.after(100, self._poll_events)

    def _show_video_frame(self, frame: object) -> None:
        if ImageTk is None or Image is None or not isinstance(frame, Image.Image):
            return
        self.video_frame = frame
        self._render_video_frame()

    def _render_video_frame(self) -> None:
        if (ImageTk is None or ImageDraw is None or
                not isinstance(self.video_frame, Image.Image)):
            return
        frame = self.video_frame.copy()
        draw = ImageDraw.Draw(frame)
        width, height = frame.size
        if self.wheel_box_ratio is not None:
            left, top, right, bottom = self.wheel_box_ratio
            draw.rectangle(
                (
                    int(left * (width - 1)),
                    int(top * (height - 1)),
                    int(right * (width - 1)),
                    int(bottom * (height - 1)),
                ),
                outline=(0, 255, 80),
                width=3,
            )
        if self.wheel_drag_start is not None and self.wheel_drag_current is not None:
            x0, y0 = self.wheel_drag_start
            x1, y1 = self.wheel_drag_current
            draw.rectangle(
                (min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)),
                outline=(255, 220, 0),
                width=3,
            )
        self.video_photo = ImageTk.PhotoImage(frame)
        self.video_label.configure(image=self.video_photo, text="")

    def _handle_finished(
        self,
        kind: str,
        return_code: int,
        process: subprocess.Popen[bytes],
    ) -> None:
        if process is self.process:
            self.process = None
            self.process_kind = ""
        if kind == "run":
            self._stop_video(clear=False)
            self._stop_steering_stream(clear=False)
            if self.last_csv is not None and self.last_csv.is_file():
                self._load_steering_csv(self.last_csv)
        self._refresh_button_state()
        if return_code == 0:
            messages = {
                "deploy": "部署完成，板端程序未自动启动",
                "run": "运行结束，CSV 与轨迹图已保存",
                "plot": "轨迹图生成完成",
            }
            self.status_var.set(messages[kind])
            if kind in ("run", "plot"):
                self._show_latest_trajectory()
        else:
            self.status_var.set(f"{kind} 失败，退出码 {return_code}")
            messagebox.showerror(
                "任务失败",
                f"{kind} 退出码：{return_code}\n请查看运行日志。",
                parent=self.root,
            )
        if self.close_when_done:
            self.root.destroy()

    def _show_latest_trajectory(self) -> None:
        png = self.last_png
        if png is None or not png.is_file():
            candidates = sorted(
                self.output_dir.glob("rewrite_*.png"),
                key=lambda item: item.stat().st_mtime,
                reverse=True,
            )
            if candidates:
                png = candidates[0]
        if png is None or not png.is_file():
            self._append_log("[GUI] 未找到生成的轨迹 PNG")
            return
        self.last_png = png
        self.output_var.set(str(png))
        self.path_rows = []
        self.path_canvas.grid_remove()
        self.image_label.grid(
            row=1, column=0, sticky="nsew", padx=8, pady=5
        )
        if Image is not None and ImageTk is not None:
            try:
                image = Image.open(png)
                image.thumbnail((940, 600), Image.Resampling.LANCZOS)
                self.photo = ImageTk.PhotoImage(image)
                self.image_label.configure(image=self.photo, text="")
            except OSError as exc:
                self.image_label.configure(text=f"无法读取轨迹图：{exc}", image="")
        else:
            try:
                self.photo = tk.PhotoImage(file=str(png))
                self.image_label.configure(image=self.photo, text="")
            except tk.TclError:
                self.image_label.configure(
                    text="缺少 Pillow，无法在界面缩放显示；PNG 已保存。", image=""
                )
        summary_path = png.with_name(png.stem + "_summary.txt")
        summary_text = ""
        try:
            if summary_path.is_file():
                summary_text = summary_path.read_text(
                    encoding="utf-8", errors="replace"
                )
        except OSError as exc:
            summary_text = f"读取摘要失败：{exc}"
        self.summary.configure(state=tk.NORMAL)
        self.summary.delete("1.0", tk.END)
        self.summary.insert(tk.END, summary_text)
        self.summary.configure(state=tk.DISABLED)
        self.notebook.select(self.trajectory_tab)

    def _refresh_button_state(self) -> None:
        busy = self.process is not None and self.process.poll() is None
        run_busy = busy and self.process_kind == "run"
        self.deploy_button.configure(state=tk.DISABLED if busy else tk.NORMAL)
        self.start_button.configure(state=tk.DISABLED if busy else tk.NORMAL)
        self.stop_button.configure(state=tk.NORMAL if run_busy else tk.DISABLED)

    def _on_close(self) -> None:
        if (
            self.process is not None
            and self.process.poll() is None
            and self.process_kind == "run"
        ):
            if not messagebox.askyesno(
                "车辆仍在运行",
                "关闭界面前将先安全停车。是否继续？",
                icon=messagebox.WARNING,
                parent=self.root,
            ):
                return
            self.close_when_done = True
            self._request_stop()
            return
        if self.autosave_after_id is not None:
            self.root.after_cancel(self.autosave_after_id)
            self.autosave_after_id = None
        self._save_settings(quiet=True, show_errors=False)
        self._stop_video(clear=True)
        self._stop_steering_stream(clear=True)
        self.root.destroy()


def main() -> int:
    root = tk.Tk()
    app = RewriteControlGui(root)
    if len(sys.argv) > 1:
        csv_path = Path(sys.argv[1]).resolve()
        if csv_path.is_file():
            root.after(0, lambda: app._show_csv_path(csv_path))
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
