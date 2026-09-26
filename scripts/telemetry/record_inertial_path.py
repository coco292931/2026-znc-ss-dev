#!/usr/bin/env python3
"""Record a replayable inertial path from rewrite telemetry.

The board remains the single owner of the LSM6DSR. This tool consumes the
board-fused IMU/encoder odometry stream and writes the compact CSV understood
by the navigation/inertial-navigation module in this tree.
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
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable

from record_rewrite_trajectory import telemetry_lines


FIELDS = ["route_distance_cm", "x_cm", "y_cm", "heading_deg", "speed_cmps"]


def as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def as_float(value: Any, default: float = 0.0) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def wrap_degrees(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


@dataclass
class RoutePoint:
    x_cm: float
    y_cm: float
    heading_deg: float
    speed_cmps: float


class RouteRecorder:
    def __init__(
        self,
        spacing_cm: float,
        heading_spacing_deg: float,
        speed_override_cmps: float,
        min_speed_cmps: float,
        max_speed_cmps: float,
    ) -> None:
        self.spacing_cm = max(0.5, spacing_cm)
        self.heading_spacing_deg = max(0.5, heading_spacing_deg)
        self.speed_override_cmps = max(0.0, speed_override_cmps)
        self.min_speed_cmps = max(1.0, min_speed_cmps)
        self.max_speed_cmps = max(self.min_speed_cmps, max_speed_cmps)
        self._raw_points: list[RoutePoint] = []
        self._pending: RoutePoint | None = None
        self.total_samples = 0
        self.rejected_samples = 0
        self.imu_invalid_samples = 0
        self.odometry_invalid_samples = 0

    @property
    def waypoint_count(self) -> int:
        return len(self._raw_points)

    def add(self, sample: dict[str, Any]) -> bool:
        self.total_samples += 1
        imu_valid = as_bool(sample.get("imu_valid"))
        odometry_valid = as_bool(sample.get("odometry_valid"))
        if not imu_valid:
            self.imu_invalid_samples += 1
        if not odometry_valid:
            self.odometry_invalid_samples += 1
        if not imu_valid or not odometry_valid:
            self.rejected_samples += 1
            return False

        point = RoutePoint(
            x_cm=as_float(sample.get("x_cm")),
            y_cm=as_float(sample.get("y_cm")),
            heading_deg=as_float(
                sample.get("heading_deg", sample.get("imu_heading_deg"))
            ),
            speed_cmps=self._sample_speed(sample),
        )
        self._pending = point
        if not self._raw_points:
            self._raw_points.append(point)
            return True

        previous = self._raw_points[-1]
        distance = math.hypot(
            point.x_cm - previous.x_cm, point.y_cm - previous.y_cm
        )
        heading_change = abs(
            wrap_degrees(point.heading_deg - previous.heading_deg)
        )
        if (
            distance >= self.spacing_cm
            or heading_change >= self.heading_spacing_deg
        ):
            self._raw_points.append(point)
            return True
        return False

    def _sample_speed(self, sample: dict[str, Any]) -> float:
        speed = self.speed_override_cmps
        if speed <= 0.0:
            speed = as_float(
                sample.get(
                    "target_speed_cmps", sample.get("limited_speed_cmps")
                )
            )
        if speed <= 0.0:
            speed = self.min_speed_cmps
        return min(self.max_speed_cmps, max(self.min_speed_cmps, speed))

    def points(self) -> list[dict[str, float]]:
        raw = list(self._raw_points)
        if self._pending is not None:
            if not raw or math.hypot(
                self._pending.x_cm - raw[-1].x_cm,
                self._pending.y_cm - raw[-1].y_cm,
            ) >= 0.1:
                raw.append(self._pending)
        if len(raw) < 2:
            return []

        origin = raw[0]
        angle = math.radians(origin.heading_deg)
        cosine = math.cos(angle)
        sine = math.sin(angle)
        route_distance = 0.0
        output: list[dict[str, float]] = []
        previous_x = 0.0
        previous_y = 0.0
        for index, point in enumerate(raw):
            dx = point.x_cm - origin.x_cm
            dy = point.y_cm - origin.y_cm
            x_cm = cosine * dx + sine * dy
            y_cm = -sine * dx + cosine * dy
            if index:
                route_distance += math.hypot(
                    x_cm - previous_x, y_cm - previous_y
                )
            output.append(
                {
                    "route_distance_cm": route_distance,
                    "x_cm": x_cm,
                    "y_cm": y_cm,
                    "heading_deg": wrap_degrees(
                        point.heading_deg - origin.heading_deg
                    ),
                    "speed_cmps": point.speed_cmps,
                }
            )
            previous_x = x_cm
            previous_y = y_cm
        output[-1]["speed_cmps"] = 0.0
        return output


def default_output() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    root = Path(__file__).resolve().parents[2]
    return root / "build" / "paths" / f"inertial_path_{stamp}.csv"


def write_path(output: Path, points: list[dict[str, float]]) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(output.name + ".tmp")
    with temporary.open("w", newline="", encoding="utf-8") as handle:
        handle.write("# rewrite inertial path v1\n")
        handle.write("# frame: +X forward, +Y right, heading right-positive\n")
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        for point in points:
            writer.writerow(
                {key: f"{point[key]:.6f}" for key in FIELDS}
            )
        handle.flush()
    temporary.replace(output)


def csv_samples(path: Path) -> Iterable[dict[str, Any]]:
    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        yield from csv.DictReader(
            line for line in handle if not line.startswith("#")
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record a replayable IMU/odometry path from rewrite"
    )
    parser.add_argument(
        "--url",
        default="http://192.168.43.220:8080/telemetry",
        help="rewrite NDJSON telemetry endpoint",
    )
    parser.add_argument(
        "--input",
        type=Path,
        help="convert an existing telemetry CSV instead of recording live",
    )
    parser.add_argument("--output", type=Path, help="output route CSV")
    parser.add_argument("--spacing", type=float, default=5.0)
    parser.add_argument("--heading-spacing", type=float, default=4.0)
    parser.add_argument(
        "--speed", type=float, default=0.0,
        help="fixed replay speed; 0 records telemetry target speed",
    )
    parser.add_argument("--min-speed", type=float, default=10.0)
    parser.add_argument("--max-speed", type=float, default=80.0)
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--reconnect-delay", type=float, default=1.0)
    parser.add_argument("--stop-after-disconnect", action="store_true")
    parser.add_argument("--stop-file", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output = args.output or default_output()
    recorder = RouteRecorder(
        args.spacing,
        args.heading_spacing,
        args.speed,
        args.min_speed,
        args.max_speed,
    )

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
        print(f"Recording {args.url} -> {output}")
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
                    if recorder.add(sample) and recorder.waypoint_count % 10 == 0:
                        print(
                            f"\rwaypoints={recorder.waypoint_count:5d} "
                            f"rejected={recorder.rejected_samples:4d}",
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

    points = recorder.points()
    if len(points) < 2:
        print(
            "No usable path: at least two valid IMU/odometry points are "
            "required. "
            f"received={recorder.total_samples}, "
            f"imu_invalid={recorder.imu_invalid_samples}, "
            f"odometry_invalid={recorder.odometry_invalid_samples}. "
            "A --dry-run session cannot record a moving path.",
            file=sys.stderr,
        )
        return 1
    write_path(output, points)
    print(
        f"\nSaved {len(points)} waypoints, "
        f"length={points[-1]['route_distance_cm']:.1f}cm -> {output}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

