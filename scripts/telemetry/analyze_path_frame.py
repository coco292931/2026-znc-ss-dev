"""Analyze a /stream/path frame: strip overlay colors, recover the binary grid.

Usage:
    python analyze_path_frame.py <frame.jpg>

The path stream renders the 94x60 binary road grid scaled to the video frame
(320x240 by default) with colored overlay lines on top.  We keep only pixels
that are (near) pure gray-white, then downsample back to the grid and print
per-row white runs so the geometry seen by the vision pipeline is readable
without a GUI.
"""

from __future__ import annotations

import sys

import numpy as np
from PIL import Image

GRID_W = 94
GRID_H = 60


def load_binary_mask(path: str) -> np.ndarray:
    img = np.asarray(Image.open(path).convert("RGB"), dtype=np.int16)
    r, g, b = img[:, :, 0], img[:, :, 1], img[:, :, 2]
    # Pure gray/white: all channels bright and close together.  This rejects
    # every overlay color (yellow/green/magenta/blue/red/orange) and the black
    # background.
    bright = (r > 180) & (g > 180) & (b > 180)
    neutral = (np.abs(r - g) < 40) & (np.abs(g - b) < 40)
    return (bright & neutral)


def runs_of(row: np.ndarray) -> list[tuple[int, int]]:
    out: list[tuple[int, int]] = []
    start = -1
    for i, v in enumerate(row):
        if v and start < 0:
            start = i
        elif not v and start >= 0:
            out.append((start, i - 1))
            start = -1
    if start >= 0:
        out.append((start, len(row) - 1))
    return out


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    mask = load_binary_mask(sys.argv[1])
    h, w = mask.shape
    print(f"frame {w}x{h} -> grid {GRID_W}x{GRID_H}")

    # Downsample: a grid cell is on when the majority of its pixels are on.
    ys = (np.arange(GRID_H) * h / GRID_H).astype(int)
    xs = (np.arange(GRID_W) * w / GRID_W).astype(int)
    grid = np.zeros((GRID_H, GRID_W), dtype=bool)
    for gy in range(GRID_H):
        y0, y1 = ys[gy], ys[gy + 1] if gy + 1 < GRID_H else h
        for gx in range(GRID_W):
            x0, x1 = xs[gx], xs[gx + 1] if gx + 1 < GRID_W else w
            block = mask[y0:y1, x0:x1]
            grid[gy, gx] = block.mean() > 0.5

    print("row: white runs (col ranges)  [row 0 = far, row 59 = car front]")
    for gy in range(GRID_H):
        rr = runs_of(grid[gy])
        if not rr:
            continue
        txt = " ".join(f"{a}-{b}" for a, b in rr)
        total = int(grid[gy].sum())
        print(f"{gy:3d}: {txt}   (n={total})")

    # Column histogram of the whole grid, to see where mass sits.
    colsum = grid.sum(axis=0)
    occupied = np.nonzero(colsum)[0]
    if len(occupied):
        print(f"grid col range with white: {occupied.min()}..{occupied.max()}")
        weighted = float((colsum * np.arange(GRID_W)).sum() / colsum.sum())
        print(f"area-weighted centroid col: {weighted:.1f}  (image center = {GRID_W/2:.1f})")

    # ASCII picture, useful when no GUI is available.
    print("\nASCII (0=black, #=white), row 0 top = far:")
    for gy in range(0, GRID_H, 1):
        line = "".join("#" if v else "." for v in grid[gy])
        print(f"{gy:2d} {line}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
