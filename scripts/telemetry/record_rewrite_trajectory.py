#!/usr/bin/env python3
"""Record rewrite path-follow telemetry as CSV.

The vehicle owns sensor fusion and odometry. This tool only records the
newline-delimited JSON stream, so it never competes with the board process for
the software-I2C IMU.
"""

from __future__ import annotations

import argparse
import csv
import json
import signal
import sys
import time
import urllib.error
import urllib.request
from datetime import datetime
from pathlib import Path
from typing import Any, Iterator


HOST_FIELDS = [
    "host_received_s",
    "host_gap_s",
    "reconnect_count",
    "link_gap",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="记录 rewrite /telemetry NDJSON 轨迹数据"
    )
    parser.add_argument(
        "--url",
        default="http://192.168.43.178:8080/telemetry",
        help="车辆遥测地址",
    )
    parser.add_argument(
        "--output",
        default="",
        help="CSV 输出；默认 build/trajectory/rewrite_时间.csv",
    )
    parser.add_argument(
        "--reconnect-delay",
        type=float,
        default=1.0,
        help="连接中断后的重试间隔",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=8.0,
        help="HTTP 连接/读取超时",
    )
    parser.add_argument(
        "--gap-threshold",
        type=float,
        default=0.25,
        help="主机收包间隔超过该值时标记 link_gap",
    )
    parser.add_argument(
        "--flush-every",
        type=int,
        default=20,
        help="每多少行刷新一次磁盘",
    )
    parser.add_argument(
        "--stop-after-disconnect",
        action="store_true",
        help="Exit cleanly when an established telemetry stream disconnects",
    )
    parser.add_argument(
        "--stop-file",
        default="",
        help="Exit cleanly when this local marker file appears",
    )
    return parser.parse_args()


def default_output() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    root = Path(__file__).resolve().parents[2]
    return root / "build" / "trajectory" / f"rewrite_{stamp}.csv"


def telemetry_lines(url: str, timeout: float) -> Iterator[dict[str, Any]]:
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


def normalize_sample(
    sample: dict[str, Any],
    host_received_s: float,
    host_gap_s: float,
    reconnect_count: int,
    gap_threshold: float,
) -> dict[str, Any]:
    row = dict(sample)
    row.update(
        {
            "host_received_s": f"{host_received_s:.6f}",
            "host_gap_s": f"{host_gap_s:.6f}",
            "reconnect_count": reconnect_count,
            "link_gap": int(host_gap_s > gap_threshold),
        }
    )
    return row


def main() -> int:
    args = parse_args()
    output = Path(args.output) if args.output else default_output()
    stop_file = Path(args.stop_file) if args.stop_file else None
    output.parent.mkdir(parents=True, exist_ok=True)
    running = True

    def stop_handler(_signum: int, _frame: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop_handler)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, stop_handler)

    writer: csv.DictWriter[str] | None = None
    handle = output.open("w", newline="", encoding="utf-8")
    start = time.monotonic()
    previous_received: float | None = None
    reconnect_count = 0
    row_count = 0
    connected_once = False
    print(f"记录 {args.url} -> {output}")

    try:
        while running:
            if stop_file is not None and stop_file.exists():
                break
            try:
                for sample in telemetry_lines(args.url, args.timeout):
                    if not running:
                        break
                    if stop_file is not None and stop_file.exists():
                        running = False
                        break
                    connected_once = True
                    received = time.monotonic()
                    gap = (
                        0.0
                        if previous_received is None
                        else received - previous_received
                    )
                    previous_received = received
                    row = normalize_sample(
                        sample,
                        received - start,
                        gap,
                        reconnect_count,
                        args.gap_threshold,
                    )
                    if writer is None:
                        fields = HOST_FIELDS + [
                            key for key in row if key not in HOST_FIELDS
                        ]
                        writer = csv.DictWriter(
                            handle,
                            fieldnames=fields,
                            extrasaction="ignore",
                        )
                        writer.writeheader()
                    writer.writerow(row)
                    row_count += 1
                    if row_count % max(1, args.flush_every) == 0:
                        handle.flush()
                        elapsed = float(sample.get("elapsed_s", 0.0))
                        state = sample.get("state", "?")
                        mode = sample.get("sensor_mode", "?")
                        print(
                            f"\rrows={row_count} vehicle={elapsed:7.2f}s "
                            f"state={state:<14} mode={mode:<18}",
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
                if stop_file is not None and stop_file.exists():
                    break
                reconnect_count += 1
                if args.stop_after_disconnect and connected_once:
                    print(
                        f"\nTelemetry stopped after recording {row_count} rows."
                    )
                    break
                print(
                    f"\n[WARN] 遥测中断：{exc}；"
                    f"{args.reconnect_delay:.1f}s 后重连",
                    file=sys.stderr,
                )
                deadline = time.monotonic() + max(0.1, args.reconnect_delay)
                while running and time.monotonic() < deadline:
                    time.sleep(0.05)
    finally:
        handle.flush()
        handle.close()

    print(f"\n完成：{row_count} 行，重连 {reconnect_count} 次，文件 {output}")
    return 0 if row_count > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())


