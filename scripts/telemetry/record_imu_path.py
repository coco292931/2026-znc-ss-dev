#!/usr/bin/env python3
"""Record the rewrite IMU-integrated velocity and path as CSV.

The board initializes and samples the LSM6DSR. This tool only records the
already calibrated and 104 Hz integrated IMU state exposed by /telemetry, so
it works without encoder odometry and never opens the IMU from the PC.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import signal
import sys
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, Iterator, List, Union


FIELDS = [
    "elapsed_s",
    "route_distance_cm",
    "x_cm",
    "y_cm",
    "heading_deg",
    "speed_cmps",
    "velocity_x_cmps",
    "velocity_y_cmps",
    "raw_accel_x_g",
    "raw_accel_y_g",
    "raw_accel_z_g",
    "forward_accel_mps2",
    "right_accel_mps2",
    "imu_stationary",
]

REQUIRED_IMU_FIELDS = (
    "imu_position_x_m",
    "imu_position_y_m",
    "imu_velocity_x_mps",
    "imu_velocity_y_mps",
    "imu_raw_accel_x_g",
    "imu_raw_accel_y_g",
    "imu_raw_accel_z_g",
    "imu_forward_accel_mps2",
    "imu_right_accel_mps2",
)


def as_bool(value):
    # type: (Any) -> bool
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def as_float(value, default=0.0):
    # type: (Any, float) -> float
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def wrap_degrees(angle):
    # type: (float) -> float
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def telemetry_lines(url, timeout):
    # type: (str, float) -> Iterator[Dict[str, Any]]
    request = urllib.request.Request(
        url,
        headers={"Accept": "application/x-ndjson", "Cache-Control": "no-cache"},
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        for raw in response:
            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            sample = json.loads(line)
            if isinstance(sample, dict):
                yield sample


@dataclass
class ImuPathPoint:
    elapsed_s: float
    position_x_m: float
    position_y_m: float
    velocity_x_mps: float
    velocity_y_mps: float
    heading_deg: float
    raw_accel_x_g: float
    raw_accel_y_g: float
    raw_accel_z_g: float
    forward_accel_mps2: float
    right_accel_mps2: float
    stationary: bool


class ImuPathRecorder:
    def __init__(
        self,
        minimum_period_s: float = 0.0,
        maximum_start_gap_s: float = 1.0,
    ) -> None:
        self.minimum_period_s = max(0.0, minimum_period_s)
        self.maximum_start_gap_s = max(0.1, maximum_start_gap_s)
        self.points = []  # type: List[ImuPathPoint]
        self.total_samples = 0
        self.imu_invalid_samples = 0
        self.missing_field_samples = 0

    def add(self, sample):
        # type: (Dict[str, Any]) -> bool
        self.total_samples += 1
        if not as_bool(sample.get("imu_valid")):
            self.imu_invalid_samples += 1
            return False
        if any(field not in sample for field in REQUIRED_IMU_FIELDS):
            self.missing_field_samples += 1
            return False

        point = ImuPathPoint(
            elapsed_s=as_float(sample.get("elapsed_s")),
            position_x_m=as_float(sample.get("imu_position_x_m")),
            position_y_m=as_float(sample.get("imu_position_y_m")),
            velocity_x_mps=as_float(sample.get("imu_velocity_x_mps")),
            velocity_y_mps=as_float(sample.get("imu_velocity_y_mps")),
            heading_deg=as_float(sample.get("imu_heading_deg")),
            raw_accel_x_g=as_float(sample.get("imu_raw_accel_x_g")),
            raw_accel_y_g=as_float(sample.get("imu_raw_accel_y_g")),
            raw_accel_z_g=as_float(sample.get("imu_raw_accel_z_g")),
            forward_accel_mps2=as_float(
                sample.get("imu_forward_accel_mps2")
            ),
            right_accel_mps2=as_float(
                sample.get("imu_right_accel_mps2")
            ),
            stationary=as_bool(sample.get("imu_stationary")),
        )
        if (
            len(self.points) == 1
            and point.elapsed_s - self.points[0].elapsed_s
            > self.maximum_start_gap_s
        ):
            # /telemetry may send its cached last frame before the first live
            # frame when a client reconnects after an idle period.
            self.points.clear()
        if self.points and point.elapsed_s <= self.points[-1].elapsed_s:
            return False
        if self.points and (
            point.elapsed_s - self.points[-1].elapsed_s
            < self.minimum_period_s
        ):
            return False
        self.points.append(point)
        return True

    def rows(self):
        # type: () -> List[Dict[str, Union[float, int]]]
        if len(self.points) < 2:
            return []
        origin = self.points[0]
        angle = math.radians(origin.heading_deg)
        cosine = math.cos(angle)
        sine = math.sin(angle)
        route_distance_cm = 0.0
        previous_x_cm = 0.0
        previous_y_cm = 0.0
        rows = []  # type: List[Dict[str, Union[float, int]]]
        for index, point in enumerate(self.points):
            dx_m = point.position_x_m - origin.position_x_m
            dy_m = point.position_y_m - origin.position_y_m
            x_cm = 100.0 * (cosine * dx_m + sine * dy_m)
            y_cm = 100.0 * (-sine * dx_m + cosine * dy_m)
            dvx_mps = point.velocity_x_mps - origin.velocity_x_mps
            dvy_mps = point.velocity_y_mps - origin.velocity_y_mps
            velocity_x_cmps = 100.0 * (
                cosine * dvx_mps + sine * dvy_mps
            )
            velocity_y_cmps = 100.0 * (
                -sine * dvx_mps + cosine * dvy_mps
            )
            if index:
                route_distance_cm += math.hypot(
                    x_cm - previous_x_cm, y_cm - previous_y_cm
                )
            rows.append(
                {
                    "elapsed_s": point.elapsed_s - origin.elapsed_s,
                    "route_distance_cm": route_distance_cm,
                    "x_cm": x_cm,
                    "y_cm": y_cm,
                    "heading_deg": wrap_degrees(
                        point.heading_deg - origin.heading_deg
                    ),
                    "speed_cmps": math.hypot(
                        velocity_x_cmps, velocity_y_cmps
                    ),
                    "velocity_x_cmps": velocity_x_cmps,
                    "velocity_y_cmps": velocity_y_cmps,
                    "raw_accel_x_g": point.raw_accel_x_g,
                    "raw_accel_y_g": point.raw_accel_y_g,
                    "raw_accel_z_g": point.raw_accel_z_g,
                    "forward_accel_mps2": point.forward_accel_mps2,
                    "right_accel_mps2": point.right_accel_mps2,
                    "imu_stationary": int(point.stationary),
                }
            )
            previous_x_cm = x_cm
            previous_y_cm = y_cm
        return rows


def default_output() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    root = Path(__file__).resolve().parents[2]
    return root / "build" / "paths" / f"imu_path_{stamp}.csv"


def write_rows(output, rows):
    # type: (Path, List[Dict[str, Union[float, int]]]) -> None
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(output.name + ".tmp")
    with temporary.open("w", newline="", encoding="utf-8") as handle:
        handle.write("# rewrite direct IMU velocity/path v1\n")
        handle.write("# frame: +X initial forward, +Y initial right\n")
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    field: row[field]
                    if isinstance(row[field], int)
                    else f"{row[field]:.6f}"
                    for field in FIELDS
                }
            )
        handle.flush()
    temporary.replace(output)


def csv_samples(path):
    # type: (Path) -> Iterable[Dict[str, Any]]
    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        yield from csv.DictReader(
            line for line in handle if not line.startswith("#")
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record velocity and path directly from initialized IMU"
    )
    parser.add_argument(
        "--url",
        default="http://192.168.43.220:8080/telemetry",
        help="rewrite NDJSON telemetry endpoint",
    )
    parser.add_argument(
        "--input",
        type=Path,
        help="convert a telemetry CSV containing direct IMU fields",
    )
    parser.add_argument("--output", type=Path, help="output IMU path CSV")
    parser.add_argument(
        "--sample-period",
        type=float,
        default=0.0,
        help="minimum seconds between CSV rows; default keeps every sample",
    )
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--reconnect-delay", type=float, default=1.0)
    parser.add_argument("--stop-after-disconnect", action="store_true")
    parser.add_argument("--stop-file", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output = args.output or default_output()
    recorder = ImuPathRecorder(args.sample_period)

    if args.input:
        for sample in csv_samples(args.input):
            recorder.add(sample)
    else:
        running = True

        def stop_handler(_signum: int, _frame: object) -> None:
            nonlocal running
            running = False

        signal.signal(signal.SIGINT, stop_handler)
        if hasattr(signal, "SIGTERM"):
            signal.signal(signal.SIGTERM, stop_handler)
        connected_once = False
        print(f"Recording direct IMU path {args.url} -> {output}")
        while running:
            if args.stop_file and args.stop_file.exists():
                break
            try:
                for sample in telemetry_lines(args.url, args.timeout):
                    if not running or (
                        args.stop_file and args.stop_file.exists()
                    ):
                        running = False
                        break
                    connected_once = True
                    if recorder.add(sample) and len(recorder.points) % 20 == 0:
                        print(
                            f"\rimu_samples={len(recorder.points):5d}",
                            end="",
                            flush=True,
                        )
                if running:
                    raise ConnectionError("telemetry stream ended")
            except (
                OSError,
                ConnectionError,
                json.JSONDecodeError,
                urllib.error.URLError,
            ) as exc:
                if args.stop_after_disconnect and connected_once:
                    break
                print(f"\n[WARN] {exc}; reconnecting", file=sys.stderr)
                deadline = time.monotonic() + max(0.1, args.reconnect_delay)
                while running and time.monotonic() < deadline:
                    time.sleep(0.05)

    rows = recorder.rows()
    if len(rows) < 2:
        print(
            "No usable direct-IMU path. "
            f"received={recorder.total_samples}, "
            f"imu_invalid={recorder.imu_invalid_samples}, "
            f"missing_new_imu_fields={recorder.missing_field_samples}. "
            "Deploy the rebuilt rewrite binary and wait for [IMU] ready.",
            file=sys.stderr,
        )
        return 1
    write_rows(output, rows)
    print(
        f"\nSaved {len(rows)} IMU samples, "
        f"duration={rows[-1]['elapsed_s']:.2f}s, "
        f"path={rows[-1]['route_distance_cm']:.1f}cm -> {output}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


