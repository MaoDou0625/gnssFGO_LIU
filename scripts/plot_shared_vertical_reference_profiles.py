#!/usr/bin/env python

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt


EARTH_RADIUS_M = 6378137.0


@dataclass
class ReferenceLinePoint:
    s_m: float
    east_m: float
    north_m: float
    origin_lat_rad: float
    origin_lon_rad: float
    origin_h_m: float


@dataclass
class Track:
    label: str
    s_m: list[float]
    up_m: list[float]
    time_s: list[float]


def safe_float(value: str | None) -> float:
    try:
        return float(value) if value not in (None, "") else float("nan")
    except ValueError:
        return float("nan")


def parse_labeled_path(value: str) -> tuple[str, Path]:
    if "=" not in value:
        path = Path(value)
        return path.stem, path
    label, raw_path = value.split("=", 1)
    if not label:
        raise ValueError(f"empty label in {value!r}")
    return label, Path(raw_path)


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as file:
        return list(csv.DictReader(file))


def read_reference(path: Path) -> tuple[list[float], list[float]]:
    rows = read_csv_rows(path)
    return (
        [safe_float(row.get("s_m")) for row in rows],
        [safe_float(row.get("reference_up_m")) for row in rows],
    )


def read_reference_line(path: Path) -> list[ReferenceLinePoint]:
    points: list[ReferenceLinePoint] = []
    for row in read_csv_rows(path):
        points.append(
            ReferenceLinePoint(
                s_m=safe_float(row.get("s_m")),
                east_m=safe_float(row.get("east_m")),
                north_m=safe_float(row.get("north_m")),
                origin_lat_rad=safe_float(row.get("origin_lat_rad")),
                origin_lon_rad=safe_float(row.get("origin_lon_rad")),
                origin_h_m=safe_float(row.get("origin_h_m")),
            )
        )
    if len(points) < 2:
        raise ValueError("shared reference line must contain at least two points")
    if not math.isfinite(points[0].origin_lat_rad) or not math.isfinite(points[0].origin_lon_rad):
        raise ValueError("shared_reference_line.csv must contain origin_lat_rad/origin_lon_rad")
    return points


def geodetic_to_local_xy(lat_rad: float, lon_rad: float, line: list[ReferenceLinePoint]) -> tuple[float, float]:
    origin = line[0]
    mean_lat = 0.5 * (lat_rad + origin.origin_lat_rad)
    east_m = (lon_rad - origin.origin_lon_rad) * math.cos(mean_lat) * EARTH_RADIUS_M
    north_m = (lat_rad - origin.origin_lat_rad) * EARTH_RADIUS_M
    return east_m, north_m


def project_xy_to_line(east_m: float, north_m: float, line: list[ReferenceLinePoint]) -> float:
    best_s = float("nan")
    best_dist2 = float("inf")
    for index in range(len(line) - 1):
        left = line[index]
        right = line[index + 1]
        dx = right.east_m - left.east_m
        dy = right.north_m - left.north_m
        segment_len2 = dx * dx + dy * dy
        if segment_len2 <= 0.0:
            continue
        alpha = ((east_m - left.east_m) * dx + (north_m - left.north_m) * dy) / segment_len2
        alpha = max(0.0, min(1.0, alpha))
        projected_e = left.east_m + alpha * dx
        projected_n = left.north_m + alpha * dy
        dist2 = (east_m - projected_e) ** 2 + (north_m - projected_n) ** 2
        if dist2 < best_dist2:
            best_dist2 = dist2
            best_s = left.s_m + alpha * (right.s_m - left.s_m)
    return best_s


def read_track(label: str, path: Path, line: list[ReferenceLinePoint]) -> Track:
    s_values: list[float] = []
    up_values: list[float] = []
    time_values: list[float] = []
    for row in read_csv_rows(path):
        lat_rad = safe_float(row.get("lat_rad"))
        lon_rad = safe_float(row.get("lon_rad"))
        up_m = safe_float(row.get("up_m"))
        time_s = safe_float(row.get("time_s"))
        if not (math.isfinite(lat_rad) and math.isfinite(lon_rad) and math.isfinite(up_m)):
            continue
        east_m, north_m = geodetic_to_local_xy(lat_rad, lon_rad, line)
        s_m = project_xy_to_line(east_m, north_m, line)
        if math.isfinite(s_m):
            s_values.append(s_m)
            up_values.append(up_m)
            time_values.append(time_s)
    combined = sorted(zip(s_values, up_values, time_values), key=lambda item: item[0])
    if not combined:
        raise ValueError(f"trajectory has no projectable rows: {path}")
    s_sorted, up_sorted, time_sorted = zip(*combined)
    return Track(label=label, s_m=list(s_sorted), up_m=list(up_sorted), time_s=list(time_sorted))


def interp(x: list[float], y: list[float], target: float) -> float:
    if target <= x[0]:
        return y[0]
    if target >= x[-1]:
        return y[-1]
    lo = 0
    hi = len(x) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if x[mid] <= target:
            lo = mid
        else:
            hi = mid
    span = x[hi] - x[lo]
    alpha = (target - x[lo]) / span if span > 0.0 else 0.0
    return y[lo] + alpha * (y[hi] - y[lo])


def read_projection_diagnostics(path: Path | None) -> tuple[list[float], list[float], list[float], list[float]]:
    rtk_s: list[float] = []
    rtk_up: list[float] = []
    nav_s: list[float] = []
    nav_up: list[float] = []
    if path is None:
        return rtk_s, rtk_up, nav_s, nav_up
    for row in read_csv_rows(path):
        if row.get("used") != "1":
            continue
        sample_kind = row.get("sample_kind", "")
        source = row.get("source", "")
        s_m = safe_float(row.get("s_m"))
        if sample_kind == "RTKFIX" and source == "RTK":
            up_m = safe_float(row.get("raw_up_m"))
            if math.isfinite(s_m) and math.isfinite(up_m):
                rtk_s.append(s_m)
                rtk_up.append(up_m)
        elif sample_kind == "STAGE2_NAV" and source == "NAV_BRIDGE":
            up_m = safe_float(row.get("corrected_up_m"))
            if math.isfinite(s_m) and math.isfinite(up_m):
                nav_s.append(s_m)
                nav_up.append(up_m)
    return rtk_s, rtk_up, nav_s, nav_up


def time_to_s(track: Track, time_s: float) -> float:
    pairs = sorted(
        ((time_value, s_value) for time_value, s_value in zip(track.time_s, track.s_m) if math.isfinite(time_value)),
        key=lambda item: item[0],
    )
    if not pairs:
        return float("nan")
    times, distances = zip(*pairs)
    return interp(list(times), list(distances), time_s)


def read_jump_spans(values: list[str], tracks: dict[str, Track]) -> list[tuple[float, float]]:
    spans: list[tuple[float, float]] = []
    for value in values:
        label, path = parse_labeled_path(value)
        if label not in tracks:
            raise ValueError(f"jump window label {label!r} has no matching trajectory label")
        track = tracks[label]
        for row in read_csv_rows(path):
            start_s = time_to_s(track, safe_float(row.get("start_time_s")))
            end_s = time_to_s(track, safe_float(row.get("end_time_s")))
            if math.isfinite(start_s) and math.isfinite(end_s):
                spans.append((min(start_s, end_s), max(start_s, end_s)))
    return spans


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot shared vertical reference, projected trajectories, RTK points, and jump windows."
    )
    parser.add_argument("--shared-reference", required=True, help="shared_vertical_reference.csv")
    parser.add_argument("--shared-reference-line", required=True, help="shared_reference_line.csv")
    parser.add_argument(
        "--trajectory",
        action="append",
        default=[],
        help="Projected trajectory as label=trajectory.csv. May be repeated.",
    )
    parser.add_argument(
        "--projection-diagnostics",
        default="",
        help="Optional shared_vertical_reference_projection_diagnostics.csv",
    )
    parser.add_argument(
        "--jump-windows",
        action="append",
        default=[],
        help="Optional label=body_z_seed_jump_windows.csv. Label must match a trajectory.",
    )
    parser.add_argument("--show-nav-bridge", action="store_true", help="Also scatter corrected Stage2 nav bridge samples.")
    parser.add_argument("--output", required=True, help="Output image path.")
    parser.add_argument("--title", default="Shared vertical reference profiles", help="Figure title.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    ref_s, ref_up = read_reference(Path(args.shared_reference))
    line = read_reference_line(Path(args.shared_reference_line))
    tracks = {
        label: read_track(label, path, line)
        for label, path in (parse_labeled_path(value) for value in args.trajectory)
    }
    jump_spans = read_jump_spans(args.jump_windows, tracks)
    rtk_s, rtk_up, nav_s, nav_up = read_projection_diagnostics(
        Path(args.projection_diagnostics) if args.projection_diagnostics else None
    )

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig.suptitle(args.title)
    for axis in axes:
        for start_s, end_s in jump_spans:
            axis.axvspan(start_s, end_s, color="tab:red", alpha=0.10)
        axis.grid(True, alpha=0.3)

    axes[0].plot(ref_s, ref_up, color="black", linewidth=1.8, label="shared reference")
    if rtk_s:
        axes[0].scatter(rtk_s, rtk_up, s=7, alpha=0.32, color="tab:green", label="used RTKFIX")
    if args.show_nav_bridge and nav_s:
        axes[0].scatter(nav_s, nav_up, s=4, alpha=0.16, color="tab:gray", label="nav bridge")
    for track in tracks.values():
        axes[0].plot(track.s_m, track.up_m, linewidth=1.0, label=track.label)
    axes[0].set_ylabel("height (m)")
    axes[0].legend(loc="best", fontsize=9)

    for track in tracks.values():
        residual = [up - interp(ref_s, ref_up, s_m) for s_m, up in zip(track.s_m, track.up_m)]
        axes[1].plot(track.s_m, residual, linewidth=1.0, label=f"{track.label} - shared")
    axes[1].axhline(0.0, color="black", linewidth=0.8)
    axes[1].set_xlabel("shared distance s (m)")
    axes[1].set_ylabel("height residual (m)")
    axes[1].legend(loc="best", fontsize=9)

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(output, dpi=170)
    print(f"plot_saved={output}")
    print(f"trajectory_count={len(tracks)}")
    print(f"used_rtk_point_count={len(rtk_s)}")
    print(f"jump_span_count={len(jump_spans)}")


if __name__ == "__main__":
    main()
