#!/usr/bin/env python3
"""Render a recorded rewrite trajectory and a text summary."""

from __future__ import annotations

import argparse
import csv
import math
from collections import Counter
from pathlib import Path
from statistics import median
from typing import Any


def number(row: dict[str, str], key: str, default: float = 0.0) -> float:
    try:
        return float(row.get(key, default))
    except (TypeError, ValueError):
        return default


def heading_vector(heading_deg: float) -> tuple[float, float]:
    """Map right-positive heading into the +X forward, +Y right frame."""
    heading = math.radians(heading_deg)
    return math.cos(heading), math.sin(heading)


def load_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as handle:
        return list(
            csv.DictReader(
                line
                for line in handle
                if line.strip() and not line.lstrip().startswith("#")
            )
        )


def is_direct_imu_path(rows: list[dict[str, str]]) -> bool:
    return bool(rows) and all(
        field in rows[0]
        for field in ("route_distance_cm", "speed_cmps", "imu_stationary")
    )


def imu_points(rows: list[dict[str, str]]) -> list[tuple[float, float]]:
    """Return the IMU path in the recording-start vehicle frame."""
    if not rows or is_direct_imu_path(rows):
        return []
    valid_rows = [row for row in rows if truthy(row.get("imu_valid", ""))]
    if not valid_rows:
        return []
    origin = valid_rows[0]
    origin_x_m = number(origin, "imu_position_x_m")
    origin_y_m = number(origin, "imu_position_y_m")
    angle = math.radians(number(origin, "imu_heading_deg"))
    cosine = math.cos(angle)
    sine = math.sin(angle)
    points = []
    for row in valid_rows:
        dx_m = number(row, "imu_position_x_m") - origin_x_m
        dy_m = number(row, "imu_position_y_m") - origin_y_m
        points.append((
            100.0 * (cosine * dx_m + sine * dy_m),
            100.0 * (-sine * dx_m + cosine * dy_m),
        ))
    if len(points) < 2:
        return []
    if max(math.hypot(x, y) for x, y in points) < 1e-6:
        return []
    return points


def truthy(value: Any) -> bool:
    return str(value).lower() in {"true", "1"}


def analyze_tof(
    rows: list[dict[str, str]],
    ramp_delta_mm: float = 70.0,
    enter_frames: int = 3,
    baseline_samples: int = 10,
) -> dict[str, Any]:
    columns_present = bool(rows) and all(
        field in rows[0]
        for field in ("tof_valid", "tof_distance_mm")
    )
    started_samples = sum(truthy(row.get("tof_started", "")) for row in rows)
    valid: list[tuple[dict[str, str], float]] = []
    for row in rows:
        distance = number(row, "tof_distance_mm")
        if truthy(row.get("tof_valid", "")) and 20.0 <= distance <= 4000.0:
            valid.append((row, distance))

    result: dict[str, Any] = {
        "tof_columns_present": columns_present,
        "tof_started_samples": started_samples,
        "tof_valid_samples": len(valid),
        "tof_valid_rate": len(valid) / len(rows) if rows else 0.0,
        "tof_distance_min_mm": 0.0,
        "tof_distance_max_mm": 0.0,
        "offline_tof_status": "NO_ROWS",
        "offline_tof_baseline_mm": 0.0,
        "offline_max_range_decrease_mm": 0.0,
        "offline_max_range_increase_mm": 0.0,
        "offline_ramp_candidates": 0,
        "offline_first_ramp_candidate": "NONE",
        "tof_events": [],
    }
    if not rows:
        return result
    if not columns_present:
        result["offline_tof_status"] = "MISSING_DISTANCE_COLUMNS"
        return result
    if not valid:
        result["offline_tof_status"] = (
            "SENSOR_STARTED_BUT_NO_VALID_DISTANCE"
            if started_samples
            else "SENSOR_NOT_STARTED_OR_INIT_FAILED"
        )
        return result

    distances = [distance for _, distance in valid]
    result["tof_distance_min_mm"] = min(distances)
    result["tof_distance_max_mm"] = max(distances)
    board_baselines = [
        number(row, "tof_baseline_mm")
        for row, _ in valid
        if number(row, "tof_baseline_mm") > 20.0
    ]
    required_baseline = max(3, baseline_samples)
    if board_baselines:
        baseline = median(board_baselines)
    elif len(valid) >= required_baseline:
        baseline = median(
            distance for _, distance in valid[:required_baseline]
        )
    else:
        result["offline_tof_status"] = "INSUFFICIENT_BASELINE_SAMPLES"
        return result

    threshold = max(5.0, ramp_delta_mm)
    release = threshold * 0.5
    required_enter = max(1, enter_frames)
    filtered = valid[0][1]
    decrease_count = 0
    increase_count = 0
    decrease_latched = False
    increase_latched = False
    max_decrease = 0.0
    max_increase = 0.0
    events: list[dict[str, Any]] = []

    for row, distance in valid:
        board_filtered = number(row, "tof_filtered_mm")
        if board_filtered > 20.0:
            filtered = board_filtered
        else:
            filtered += (distance - filtered) * 0.35
        decrease = baseline - filtered
        increase = filtered - baseline
        max_decrease = max(max_decrease, decrease)
        max_increase = max(max_increase, increase)

        decrease_count = decrease_count + 1 if decrease >= threshold else 0
        increase_count = increase_count + 1 if increase >= threshold else 0
        if decrease <= release:
            decrease_latched = False
        if increase <= release:
            increase_latched = False

        direction = ""
        delta = 0.0
        if decrease_count >= required_enter and not decrease_latched:
            direction = "RANGE_DECREASE"
            delta = decrease
            decrease_latched = True
        elif increase_count >= required_enter and not increase_latched:
            direction = "RANGE_INCREASE"
            delta = increase
            increase_latched = True
        if direction:
            events.append(
                {
                    "elapsed_s": number(row, "elapsed_s"),
                    "distance_cm": number(row, "distance_cm"),
                    "direction": direction,
                    "delta_mm": delta,
                    "tof_distance_mm": distance,
                    "tof_filtered_mm": filtered,
                    "tof_baseline_mm": baseline,
                }
            )

    result.update(
        {
            "offline_tof_status": "OK",
            "offline_tof_baseline_mm": baseline,
            "offline_max_range_decrease_mm": max_decrease,
            "offline_max_range_increase_mm": max_increase,
            "offline_ramp_candidates": len(events),
            "offline_first_ramp_candidate": (
                f"{events[0]['direction']}@{events[0]['elapsed_s']:.3f}s"
                if events else "NONE"
            ),
            "tof_events": events,
        }
    )
    return result


def summarize(
    rows: list[dict[str, str]],
    tof_ramp_delta_mm: float = 70.0,
    tof_enter_frames: int = 3,
    tof_baseline_samples: int = 10,
) -> dict[str, Any]:
    tof_analysis = analyze_tof(
        rows,
        ramp_delta_mm=tof_ramp_delta_mm,
        enter_frames=tof_enter_frames,
        baseline_samples=tof_baseline_samples,
    )
    if not rows:
        summary = {
            "path_source": "unknown",
            "samples": 0,
            "duration_s": 0.0,
            "distance_cm": 0.0,
            "displacement_cm": 0.0,
            "average_speed_cmps": 0.0,
            "max_speed_cmps": 0.0,
            "link_gaps": 0,
            "imu_invalid_samples": 0,
            "left_encoder_invalid_samples": 0,
            "right_encoder_invalid_samples": 0,
            "tof_available": False,
            "tof_invalid_samples": 0,
            "ramp_detected_samples": 0,
            "sensor_modes": {},
            "states": {},
            "stop_reasons": {},
            "stationary_samples": 0,
            "stationary_rate": 0.0,
            "max_planar_accel_mps2": 0.0,
        }
        summary.update(tof_analysis)
        return summary
    first = rows[0]
    last = rows[-1]
    direct_imu = is_direct_imu_path(rows)
    duration = max(
        0.0, number(last, "elapsed_s") - number(first, "elapsed_s")
    )
    distance = max(
        0.0,
        number(
            last,
            "route_distance_cm" if direct_imu else "distance_cm",
        )
        - number(
            first,
            "route_distance_cm" if direct_imu else "distance_cm",
        ),
    )
    displacement = math.hypot(
        number(last, "x_cm") - number(first, "x_cm"),
        number(last, "y_cm") - number(first, "y_cm"),
    )
    speed_field = "speed_cmps" if direct_imu else "limited_speed_cmps"
    speeds = [abs(number(row, speed_field)) for row in rows]
    stationary_samples = sum(
        truthy(row.get("imu_stationary", "")) for row in rows
    )
    summary = {
        "path_source": "direct_imu" if direct_imu else "fused_odometry",
        "samples": len(rows),
        "duration_s": duration,
        "distance_cm": distance,
        "displacement_cm": displacement,
        "average_speed_cmps": distance / duration if duration > 0 else 0.0,
        "max_speed_cmps": max(speeds, default=0.0),
        "link_gaps": sum(int(number(row, "link_gap")) for row in rows),
        "imu_invalid_samples": 0 if direct_imu else sum(
            row.get("imu_valid", "").lower() not in {"true", "1"}
            for row in rows
        ),
        "left_encoder_invalid_samples": 0 if direct_imu else sum(
            row.get("left_encoder_valid", "").lower() not in {"true", "1"}
            for row in rows
        ),
        "right_encoder_invalid_samples": 0 if direct_imu else sum(
            row.get("right_encoder_valid", "").lower() not in {"true", "1"}
            for row in rows
        ),
        "tof_available": any("tof_valid" in row for row in rows),
        "tof_invalid_samples": sum(
            row.get("tof_valid", "").lower() not in {"true", "1"}
            for row in rows
            if "tof_valid" in row
        ),
        "ramp_detected_samples": sum(
            row.get("ramp_detected", "").lower() in {"true", "1"}
            for row in rows
        ),
        "sensor_modes": dict(Counter(row.get("sensor_mode", "?") for row in rows)),
        "states": dict(Counter(row.get("state", "?") for row in rows)),
        "stop_reasons": dict(
            Counter(row.get("stop_reason", "?") for row in rows)
        ) if not direct_imu else {},
        "stationary_samples": stationary_samples,
        "stationary_rate": stationary_samples / len(rows),
        "max_planar_accel_mps2": max(
            (
                math.hypot(
                    number(row, "forward_accel_mps2"),
                    number(row, "right_accel_mps2"),
                )
                for row in rows
            ),
            default=0.0,
        ),
    }
    summary.update(tof_analysis)
    return summary


def write_summary(path: Path, summary: dict[str, Any]) -> None:
    lines = [
        f"path_source: {summary['path_source']}",
        f"samples: {summary['samples']}",
        f"duration_s: {summary['duration_s']:.3f}",
        f"distance_cm: {summary['distance_cm']:.3f}",
        f"displacement_cm: {summary['displacement_cm']:.3f}",
        f"average_speed_cmps: {summary['average_speed_cmps']:.3f}",
        f"max_speed_cmps: {summary['max_speed_cmps']:.3f}",
    ]
    if summary["path_source"] == "direct_imu":
        lines.extend(
            [
                f"stationary_samples: {summary['stationary_samples']}",
                f"stationary_rate: {summary['stationary_rate']:.3f}",
                "max_planar_accel_mps2: "
                f"{summary['max_planar_accel_mps2']:.4f}",
            ]
        )
    else:
        lines.extend(
            [
                f"link_gaps: {summary['link_gaps']}",
                f"imu_invalid_samples: {summary['imu_invalid_samples']}",
                "left_encoder_invalid_samples: "
                f"{summary['left_encoder_invalid_samples']}",
                "right_encoder_invalid_samples: "
                f"{summary['right_encoder_invalid_samples']}",
                f"tof_available: {summary['tof_available']}",
                f"tof_invalid_samples: {summary['tof_invalid_samples']}",
                f"ramp_detected_samples: {summary['ramp_detected_samples']}",
                f"tof_started_samples: {summary['tof_started_samples']}",
                f"tof_valid_samples: {summary['tof_valid_samples']}",
                f"tof_valid_rate: {summary['tof_valid_rate']:.3f}",
                f"tof_distance_min_mm: {summary['tof_distance_min_mm']:.3f}",
                f"tof_distance_max_mm: {summary['tof_distance_max_mm']:.3f}",
                f"offline_tof_status: {summary['offline_tof_status']}",
                "offline_tof_baseline_mm: "
                f"{summary['offline_tof_baseline_mm']:.3f}",
                "offline_max_range_decrease_mm: "
                f"{summary['offline_max_range_decrease_mm']:.3f}",
                "offline_max_range_increase_mm: "
                f"{summary['offline_max_range_increase_mm']:.3f}",
                "offline_ramp_candidates: "
                f"{summary['offline_ramp_candidates']}",
                "offline_first_ramp_candidate: "
                f"{summary['offline_first_ramp_candidate']}",
                f"sensor_modes: {summary['sensor_modes']}",
                f"states: {summary['states']}",
                f"stop_reasons: {summary['stop_reasons']}",
            ]
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_tof_events(path: Path, events: list[dict[str, Any]]) -> None:
    fields = [
        "elapsed_s",
        "distance_cm",
        "direction",
        "delta_mm",
        "tof_distance_mm",
        "tof_filtered_mm",
        "tof_baseline_mm",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(events)


def render(rows: list[dict[str, str]], output: Path, arrow_every: int) -> None:
    try:
        import matplotlib.pyplot as plt
        from matplotlib.collections import LineCollection
        from matplotlib.colors import Normalize
    except ImportError as exc:
        raise RuntimeError(
            "绘图需要 matplotlib：python -m pip install matplotlib"
        ) from exc

    direct_imu = is_direct_imu_path(rows)
    independent_imu = imu_points(rows)
    xs = [number(row, "x_cm") for row in rows]
    # Files recorded before the right-positive map convention do not contain
    # map_y_positive and stored +Y toward the vehicle's left. Normalize them
    # while plotting so every generated figure presents +Y to the right.
    y_scale = (
        1.0
        if direct_imu or (
            rows and rows[0].get("map_y_positive", "").lower() == "right"
        )
        else -1.0
    )
    ys = [y_scale * number(row, "y_cm") for row in rows]
    speed_field = "speed_cmps" if direct_imu else "limited_speed_cmps"
    speeds = [abs(number(row, speed_field)) for row in rows]
    heading_vectors = [
        heading_vector(number(row, "heading_deg")) for row in rows
    ]

    fig, (ax_path, ax_speed) = plt.subplots(
        2, 1, figsize=(11, 10), gridspec_kw={"height_ratios": [3, 1]}
    )
    if len(rows) >= 2:
        points = list(zip(xs, ys))
        segments = [[points[index], points[index + 1]]
                    for index in range(len(points) - 1)]
        collection = LineCollection(
            segments,
            cmap="viridis",
            norm=Normalize(0.0, max(1.0, max(speeds))),
            linewidth=2.0,
        )
        collection.set_array(speeds[:-1])
        ax_path.add_collection(collection)
        fig.colorbar(collection, ax=ax_path, label="speed (cm/s)")
    else:
        ax_path.plot(xs, ys)

    if independent_imu:
        ax_path.plot(
            [point[0] for point in independent_imu],
            [point[1] for point in independent_imu],
            color="#8e44ad",
            linewidth=1.8,
            linestyle="--",
            label="independent IMU path",
        )

    step = max(1, arrow_every)
    arrow_indices = list(range(0, len(rows), step))
    path_extent = max(max(xs) - min(xs), max(ys) - min(ys))
    if path_extent < 1e-6:
        arrow_indices = arrow_indices[:1]
        path_extent = 1.0
    arrow_x = [xs[index] for index in arrow_indices]
    arrow_y = [ys[index] for index in arrow_indices]
    arrow_vectors = [heading_vectors[index] for index in arrow_indices]
    ax_path.quiver(
        arrow_x,
        arrow_y,
        [value[0] for value in arrow_vectors],
        [value[1] for value in arrow_vectors],
        angles="xy",
        scale_units="xy",
        scale=1.0 / (path_extent * 0.04),
        width=0.003,
        color="black",
        alpha=0.55,
    )
    ax_path.scatter(xs[:1], ys[:1], color="green", s=70, label="start")
    ax_path.scatter(xs[-1:], ys[-1:], color="red", s=70, label="end")
    ax_path.set_title(
        ("direct IMU path" if direct_imu else "rewrite fused trajectory")
        + " (+X forward, +Y right)"
    )
    ax_path.set_xlabel("X (cm)")
    ax_path.set_ylabel("Y (cm)")
    ax_path.axis("equal")
    ax_path.grid(True, alpha=0.3)
    ax_path.legend()

    elapsed = [number(row, "elapsed_s") for row in rows]
    ax_speed.plot(
        elapsed,
        speeds,
        label="IMU integrated speed" if direct_imu else "limited target speed",
    )
    if direct_imu:
        stationary = [truthy(row.get("imu_stationary", "")) for row in rows]
        top = max(max(speeds, default=0.0), 1.0)
        ax_speed.fill_between(
            elapsed,
            0.0,
            top,
            where=stationary,
            color="#d7e8d2",
            alpha=0.55,
            label="stationary / ZUPT",
        )
    else:
        ax_speed.plot(
            elapsed,
            [
                0.5
                * (
                    abs(number(row, "left_speed_cmps"))
                    + abs(number(row, "right_speed_cmps"))
                )
                for row in rows
            ],
            label="wheel speed",
            alpha=0.8,
        )
    ax_speed.set_xlabel("vehicle elapsed time (s)")
    ax_speed.set_ylabel("cm/s")
    ax_speed.grid(True, alpha=0.3)
    ax_speed.legend()
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=160)
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="绘制 rewrite 车辆运行轨迹")
    parser.add_argument("csv", type=Path, help="record_rewrite_trajectory 输出")
    parser.add_argument("--output", type=Path, default=None, help="PNG 输出")
    parser.add_argument(
        "--summary", type=Path, default=None, help="文本摘要输出"
    )
    parser.add_argument(
        "--arrow-every", type=int, default=25, help="航向箭头采样间隔"
    )
    parser.add_argument(
        "--tof-events", type=Path, default=None, help="TOF 候选事件 CSV"
    )
    parser.add_argument("--tof-ramp-delta", type=float, default=70.0)
    parser.add_argument("--tof-ramp-enter", type=int, default=3)
    parser.add_argument("--tof-baseline-samples", type=int, default=10)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rows = load_rows(args.csv)
    if not rows:
        raise SystemExit("CSV 没有数据")
    output = args.output or args.csv.with_suffix(".png")
    summary_path = args.summary or args.csv.with_name(
        args.csv.stem + "_summary.txt"
    )
    tof_events_path = args.tof_events or args.csv.with_name(
        args.csv.stem + "_tof_events.csv"
    )
    summary = summarize(
        rows,
        tof_ramp_delta_mm=args.tof_ramp_delta,
        tof_enter_frames=args.tof_ramp_enter,
        tof_baseline_samples=args.tof_baseline_samples,
    )
    write_summary(summary_path, summary)
    write_tof_events(tof_events_path, summary["tof_events"])
    render(rows, output, args.arrow_every)
    print(f"轨迹图：{output}")
    print(f"摘要：{summary_path}")
    print(f"TOF events: {tof_events_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


