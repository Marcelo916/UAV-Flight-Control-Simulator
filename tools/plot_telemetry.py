#!/usr/bin/env python3
"""Plot UAV telemetry CSV into a PNG figure using only Python stdlib.

Usage:
    python3 tools/plot_telemetry.py telemetry.csv plots/nominal_response.png
"""

from __future__ import annotations

import csv
import math
import struct
import sys
import zlib
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

REQUIRED_COLUMNS = {
    "time_seconds",
    "target_altitude",
    "true_altitude",
    "vertical_velocity",
    "actual_thrust",
}

Color = Tuple[int, int, int]

WHITE: Color = (255, 255, 255)
BLACK: Color = (30, 30, 30)
GRAY: Color = (225, 225, 225)
BLUE: Color = (35, 105, 210)
ORANGE: Color = (245, 130, 32)
GREEN: Color = (30, 150, 80)
RED: Color = (200, 50, 50)


def read_telemetry(csv_path: Path) -> Dict[str, List[float]]:
    with csv_path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError("CSV is missing a header row.")

        missing = REQUIRED_COLUMNS.difference(reader.fieldnames)
        if missing:
            missing_list = ", ".join(sorted(missing))
            raise ValueError(f"CSV is missing required columns: {missing_list}")

        data: Dict[str, List[float]] = {
            "time_seconds": [],
            "target_altitude": [],
            "true_altitude": [],
            "vertical_velocity": [],
            "actual_thrust": [],
        }

        for row in reader:
            data["time_seconds"].append(float(row["time_seconds"]))
            data["target_altitude"].append(float(row["target_altitude"]))
            data["true_altitude"].append(float(row["true_altitude"]))
            data["vertical_velocity"].append(float(row["vertical_velocity"]))
            data["actual_thrust"].append(float(row["actual_thrust"]))

    if not data["time_seconds"]:
        raise ValueError("CSV contains no telemetry rows.")

    return data


def make_canvas(width: int, height: int, color: Color = WHITE) -> List[List[Color]]:
    return [[color for _ in range(width)] for _ in range(height)]


def set_pixel(canvas: List[List[Color]], x: int, y: int, color: Color) -> None:
    h = len(canvas)
    w = len(canvas[0])
    if 0 <= x < w and 0 <= y < h:
        canvas[y][x] = color


def draw_line(canvas: List[List[Color]], x0: int, y0: int, x1: int, y1: int, color: Color) -> None:
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    err = dx + dy

    while True:
        set_pixel(canvas, x0, y0, color)
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy


def draw_rect_outline(canvas: List[List[Color]], x0: int, y0: int, x1: int, y1: int, color: Color) -> None:
    draw_line(canvas, x0, y0, x1, y0, color)
    draw_line(canvas, x1, y0, x1, y1, color)
    draw_line(canvas, x1, y1, x0, y1, color)
    draw_line(canvas, x0, y1, x0, y0, color)


def draw_grid(canvas: List[List[Color]], x0: int, y0: int, x1: int, y1: int, color: Color, steps: int = 4) -> None:
    for i in range(1, steps):
        x = x0 + (x1 - x0) * i // steps
        y = y0 + (y1 - y0) * i // steps
        draw_line(canvas, x, y0, x, y1, color)
        draw_line(canvas, x0, y, x1, y, color)


def normalize(values: Iterable[float]) -> Tuple[float, float]:
    vals = list(values)
    vmin = min(vals)
    vmax = max(vals)
    if math.isclose(vmin, vmax):
        pad = 1.0 if math.isclose(vmin, 0.0) else abs(vmin) * 0.05
        return vmin - pad, vmax + pad
    pad = (vmax - vmin) * 0.05
    return vmin - pad, vmax + pad


def map_to_pixel(
    t: float,
    y: float,
    t_min: float,
    t_max: float,
    y_min: float,
    y_max: float,
    x0: int,
    y0: int,
    x1: int,
    y1: int,
) -> Tuple[int, int]:
    tx = 0.0 if math.isclose(t_max, t_min) else (t - t_min) / (t_max - t_min)
    ty = 0.0 if math.isclose(y_max, y_min) else (y - y_min) / (y_max - y_min)
    px = int(round(x0 + tx * (x1 - x0)))
    py = int(round(y1 - ty * (y1 - y0)))
    return px, py


def plot_series(
    canvas: List[List[Color]],
    times: List[float],
    values: List[float],
    y_min: float,
    y_max: float,
    bounds: Tuple[int, int, int, int],
    color: Color,
) -> None:
    x0, y0, x1, y1 = bounds
    t_min = times[0]
    t_max = times[-1]

    prev = None
    for t, v in zip(times, values):
        point = map_to_pixel(t, v, t_min, t_max, y_min, y_max, x0, y0, x1, y1)
        if prev is not None:
            draw_line(canvas, prev[0], prev[1], point[0], point[1], color)
        prev = point


def write_png(path: Path, canvas: List[List[Color]]) -> None:
    height = len(canvas)
    width = len(canvas[0])

    raw = bytearray()
    for row in canvas:
        raw.append(0)
        for r, g, b in row:
            raw.extend((r, g, b))

    def chunk(chunk_type: bytes, data: bytes) -> bytes:
        return (
            struct.pack("!I", len(data))
            + chunk_type
            + data
            + struct.pack("!I", zlib.crc32(chunk_type + data) & 0xFFFFFFFF)
        )

    ihdr = struct.pack("!IIBBBBB", width, height, 8, 2, 0, 0, 0)
    idat = zlib.compress(bytes(raw), level=9)

    png_data = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr) + chunk(b"IDAT", idat) + chunk(b"IEND", b"")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(png_data)


def make_plot(data: Dict[str, List[float]], output_path: Path) -> None:
    width, height = 1200, 820
    canvas = make_canvas(width, height)

    left_margin = 90
    right_margin = 40
    top_margin = 40
    panel_height = 220
    panel_gap = 35

    panels = []
    for i in range(3):
        y0 = top_margin + i * (panel_height + panel_gap)
        y1 = y0 + panel_height
        panels.append((left_margin, y0, width - right_margin, y1))

    times = data["time_seconds"]

    # Panel 1: target vs true altitude
    alt_min, alt_max = normalize(data["target_altitude"] + data["true_altitude"])
    draw_grid(canvas, *panels[0], GRAY)
    draw_rect_outline(canvas, *panels[0], BLACK)
    plot_series(canvas, times, data["target_altitude"], alt_min, alt_max, panels[0], ORANGE)
    plot_series(canvas, times, data["true_altitude"], alt_min, alt_max, panels[0], BLUE)

    # Panel 2: vertical velocity
    vz_min, vz_max = normalize(data["vertical_velocity"])
    draw_grid(canvas, *panels[1], GRAY)
    draw_rect_outline(canvas, *panels[1], BLACK)
    plot_series(canvas, times, data["vertical_velocity"], vz_min, vz_max, panels[1], GREEN)

    # Panel 3: actual thrust
    thrust_min, thrust_max = normalize(data["actual_thrust"])
    draw_grid(canvas, *panels[2], GRAY)
    draw_rect_outline(canvas, *panels[2], BLACK)
    plot_series(canvas, times, data["actual_thrust"], thrust_min, thrust_max, panels[2], RED)

    # Minimal legend swatches in top panel
    lx, ly = left_margin + 10, top_margin + 12
    draw_line(canvas, lx, ly, lx + 45, ly, ORANGE)
    draw_line(canvas, lx, ly + 12, lx + 45, ly + 12, BLUE)

    write_png(output_path, canvas)


def main(argv: List[str]) -> int:
    if len(argv) != 3:
        print("Usage: python3 tools/plot_telemetry.py <input_csv> <output_png>", file=sys.stderr)
        return 1

    input_csv = Path(argv[1])
    output_png = Path(argv[2])

    if not input_csv.exists():
        print(f"Input CSV not found: {input_csv}", file=sys.stderr)
        return 2

    try:
        data = read_telemetry(input_csv)
        make_plot(data, output_png)
    except Exception as exc:
        print(f"Failed to generate plot: {exc}", file=sys.stderr)
        return 3

    print(f"Saved plot: {output_png}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
