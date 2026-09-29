#!/usr/bin/env python3
"""Print compact DATA/NAV rows from the rewrite NDJSON telemetry stream."""

from __future__ import annotations

import argparse
import json
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any


def number(sample: dict[str, Any], key: str, default: float = 0.0) -> float:
    value = sample.get(key, default)
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def integer(sample: dict[str, Any], key: str, default: int = 0) -> int:
    value = sample.get(key, default)
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def text(sample: dict[str, Any], key: str, default: str = "-") -> str:
    value = sample.get(key, default)
    return str(value) if value is not None else default


def format_rows(sample: dict[str, Any]) -> tuple[str, str]:
    recognition = text(
        sample,
        "recognition",
        text(sample, "target_recognition_kind", "NONE"),
    )
    confidence = number(
        sample,
        "recognition_confidence",
        number(sample, "target_recognition_confidence"),
    )
    data = (
        f"DATA t={number(sample, 'elapsed_s'):7.2f} "
        f"state={text(sample, 'state'):>15} "
        f"rec={recognition}:{confidence:.2f} "
        f"line={number(sample, 'control_line_error'):+.3f}/"
        f"{number(sample, 'control_far_error'):+.3f} "
        f"speed={number(sample, 'limited_speed_cmps'):5.1f} "
        f"yaw={number(sample, 'target_yaw_rate_dps'):+6.1f}/"
        f"{number(sample, 'measured_yaw_rate_dps'):+6.1f} "
        f"wheel={number(sample, 'left_target_cmps'):5.1f}/"
        f"{number(sample, 'left_speed_cmps'):5.1f}|"
        f"{number(sample, 'right_target_cmps'):5.1f}/"
        f"{number(sample, 'right_speed_cmps'):5.1f} "
        f"pwm={number(sample, 'left_pwm'):+5.1f}/"
        f"{number(sample, 'right_pwm'):+5.1f}"
    )
    nav = (
        f"NAV  inertial={text(sample, 'inertial_state'):>9} "
        f"wp={integer(sample, 'inertial_waypoint_index')}/"
        f"{integer(sample, 'inertial_waypoint_count')} "
        f"progress={number(sample, 'inertial_progress_cm'):6.1f}/"
        f"{number(sample, 'inertial_path_length_cm'):6.1f}cm "
        f"xtrack={number(sample, 'inertial_cross_track_error_cm'):+6.1f}cm "
        f"heading_err={number(sample, 'inertial_heading_error_deg'):+6.1f}deg "
        f"pose=({number(sample, 'x_cm'):+6.1f},"
        f"{number(sample, 'y_cm'):+6.1f},"
        f"{number(sample, 'heading_deg'):+6.1f})"
    )
    return data, nav


def should_stop(stop_file: Path | None, deadline: float | None) -> bool:
    if stop_file is not None and stop_file.exists():
        return True
    return deadline is not None and time.monotonic() >= deadline


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="http://192.168.43.178:8080/telemetry")
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--stop-file", type=Path)
    args = parser.parse_args()

    interval = max(0.05, args.interval)
    deadline = (
        time.monotonic() + args.duration if args.duration > 0.0 else None
    )
    last_print = -1.0

    while not should_stop(args.stop_file, deadline):
        try:
            request = urllib.request.Request(
                args.url,
                headers={"User-Agent": "rewrite-telemetry-debugger"},
            )
            with urllib.request.urlopen(request, timeout=3.0) as response:
                for raw in response:
                    if should_stop(args.stop_file, deadline):
                        return 0
                    try:
                        sample = json.loads(raw.decode("utf-8"))
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        continue
                    now = time.monotonic()
                    if last_print >= 0.0 and now - last_print < interval:
                        continue
                    last_print = now
                    for row in format_rows(sample):
                        print(row, flush=True)
        except (OSError, urllib.error.URLError) as exc:
            if should_stop(args.stop_file, deadline):
                break
            print(f"telemetry reconnect: {exc}", flush=True)
            time.sleep(0.5)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


