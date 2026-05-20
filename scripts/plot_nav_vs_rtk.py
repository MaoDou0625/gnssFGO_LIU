#!/usr/bin/env python

from __future__ import annotations

import argparse
import csv
import math
import statistics
from bisect import bisect_left
from pathlib import Path

try:
    import matplotlib.pyplot as plt
except ImportError as exc:  # pragma: no cover - runtime dependency guard
    raise SystemExit(
        "matplotlib is required to generate plots. "
        "Use a Python environment with matplotlib installed."
    ) from exc


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)
DEFAULT_ERROR_DEADBAND_M = 0.02


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot offline navigation results against RTKFIX GNSS values."
    )
    parser.add_argument("--trajectory", required=True, help="Path to trajectory.csv")
    parser.add_argument(
        "--gnss",
        required=True,
        help="Path to gnss_solution_gnss_fgo.txt",
    )
    parser.add_argument(
        "--summary",
        default="",
        help="Optional path to summary.txt for exact origin reuse",
    )
    parser.add_argument(
        "--output",
        required=True,
        help="Output image path, for example nav_vs_rtk.png",
    )
    parser.add_argument(
        "--time-tolerance-s",
        type=float,
        default=0.12,
        help="Maximum allowed timestamp mismatch when pairing RTK and navigation points",
    )
    parser.add_argument(
        "--error-deadband-m",
        type=float,
        default=DEFAULT_ERROR_DEADBAND_M,
        help=(
            "Evaluation deadband in meters. Navigation/RTK differences within this "
            "distance are counted as zero in eval metrics. Use 0 for raw metrics only."
        ),
    )
    parser.add_argument(
        "--title",
        default="Offline navigation vs RTKFIX",
        help="Figure title",
    )
    return parser.parse_args()


def read_summary(path: Path) -> dict[str, float]:
    summary: dict[str, float] = {}
    if not path or not path.exists():
        return summary
    for line in path.read_text(encoding="utf-8").splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        try:
            summary[key.strip()] = float(value.strip())
        except ValueError:
            continue
    return summary


def read_trajectory(path: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        for raw in reader:
            rows.append(
                {
                    "time_s": float(raw["time_s"]),
                    "east_m": float(raw["east_m"]),
                    "north_m": float(raw["north_m"]),
                    "up_m": float(raw["up_m"]),
                    "lat_rad": float(raw["lat_rad"]),
                    "lon_rad": float(raw["lon_rad"]),
                    "h_m": float(raw["h_m"]),
                    "gnss_fix_type": raw["gnss_fix_type"],
                }
            )
    if not rows:
        raise ValueError(f"No trajectory rows found in {path}")
    return rows


def safe_float(value: str) -> float:
    parsed = float(value)
    if math.isnan(parsed) or math.isinf(parsed):
        raise ValueError("non-finite numeric value")
    return parsed


def iter_gnss_rows(path: Path):
    with path.open("r", encoding="utf-8", newline="") as file:
        for line in file:
            raw = line.strip().split()
            if raw:
                yield raw


def ecef_from_llh(lat_rad: float, lon_rad: float, h_m: float) -> tuple[float, float, float]:
    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    sin_lon = math.sin(lon_rad)
    cos_lon = math.cos(lon_rad)
    radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (radius + h_m) * cos_lat * cos_lon
    y = (radius + h_m) * cos_lat * sin_lon
    z = (radius * (1.0 - WGS84_E2) + h_m) * sin_lat
    return x, y, z


def enu_from_llh(
    lat_rad: float,
    lon_rad: float,
    h_m: float,
    origin_lat_rad: float,
    origin_lon_rad: float,
    origin_h_m: float,
) -> tuple[float, float, float]:
    x, y, z = ecef_from_llh(lat_rad, lon_rad, h_m)
    x0, y0, z0 = ecef_from_llh(origin_lat_rad, origin_lon_rad, origin_h_m)
    dx = x - x0
    dy = y - y0
    dz = z - z0

    sin_lat0 = math.sin(origin_lat_rad)
    cos_lat0 = math.cos(origin_lat_rad)
    sin_lon0 = math.sin(origin_lon_rad)
    cos_lon0 = math.cos(origin_lon_rad)

    east = -sin_lon0 * dx + cos_lon0 * dy
    north = (
        -sin_lat0 * cos_lon0 * dx
        - sin_lat0 * sin_lon0 * dy
        + cos_lat0 * dz
    )
    up = (
        cos_lat0 * cos_lon0 * dx
        + cos_lat0 * sin_lon0 * dy
        + sin_lat0 * dz
    )
    return east, north, up


def read_rtk_points(
    gnss_path: Path,
    origin_lat_rad: float,
    origin_lon_rad: float,
    origin_h_m: float,
) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    for raw in iter_gnss_rows(gnss_path):
        try:
            gnssfgo_type_code = int(float(raw[12]))
            if gnssfgo_type_code != 1:
                continue
            time_s = safe_float(raw[0])
            lat_rad = safe_float(raw[1])
            lon_rad = safe_float(raw[2])
            h_m = safe_float(raw[3])
        except (IndexError, ValueError):
            continue

        east_m, north_m, up_m = enu_from_llh(
            lat_rad,
            lon_rad,
            h_m,
            origin_lat_rad,
            origin_lon_rad,
            origin_h_m,
        )
        rows.append(
            {
                "time_s": time_s,
                "east_m": east_m,
                "north_m": north_m,
                "up_m": up_m,
            }
        )
    if not rows:
        raise ValueError(f"No RTKFIX rows found in {gnss_path}")
    return rows


def choose_origin(
    summary: dict[str, float],
    trajectory_rows: list[dict[str, object]],
    gnss_path: Path,
) -> tuple[float, float, float]:
    for raw in iter_gnss_rows(gnss_path):
        try:
            return safe_float(raw[1]), safe_float(raw[2]), safe_float(raw[3])
        except (IndexError, ValueError):
            continue

    lat = summary.get("origin_lat_rad")
    lon = summary.get("origin_lon_rad")
    h_m = summary.get("origin_h_m")
    if lat is not None and lon is not None and h_m is not None:
        return lat, lon, h_m

    first = trajectory_rows[0]
    if all(key in first for key in ("lat_rad", "lon_rad", "h_m")):
        return float(first["lat_rad"]), float(first["lon_rad"]), float(first["h_m"])

    raise ValueError("Unable to determine ENU origin")


def match_pairs(
    trajectory_rows: list[dict[str, object]],
    rtk_rows: list[dict[str, float]],
    tolerance_s: float,
    error_deadband_m: float = 0.0,
) -> list[dict[str, float]]:
    if error_deadband_m < 0.0:
        raise ValueError("error_deadband_m must be non-negative")

    trajectory_times = [float(row["time_s"]) for row in trajectory_rows]
    pairs: list[dict[str, float]] = []
    for rtk in rtk_rows:
        time_s = rtk["time_s"]
        index = bisect_left(trajectory_times, time_s)
        candidates: list[int] = []
        if index < len(trajectory_times):
            candidates.append(index)
        if index > 0:
            candidates.append(index - 1)
        if not candidates:
            continue
        best_index = min(candidates, key=lambda idx: abs(trajectory_times[idx] - time_s))
        dt = abs(trajectory_times[best_index] - time_s)
        if dt > tolerance_s:
            continue
        nav = trajectory_rows[best_index]
        east_error = float(nav["east_m"]) - rtk["east_m"]
        north_error = float(nav["north_m"]) - rtk["north_m"]
        up_error = float(nav["up_m"]) - rtk["up_m"]
        horizontal_error = math.hypot(east_error, north_error)
        position_error = math.sqrt(
            east_error * east_error
            + north_error * north_error
            + up_error * up_error
        )
        east_error_eval = apply_signed_deadband(east_error, error_deadband_m)
        north_error_eval = apply_signed_deadband(north_error, error_deadband_m)
        up_error_eval = apply_signed_deadband(up_error, error_deadband_m)
        horizontal_error_eval = apply_magnitude_deadband(
            horizontal_error, error_deadband_m
        )
        position_error_eval = apply_magnitude_deadband(position_error, error_deadband_m)
        pairs.append(
            {
                "time_s": time_s,
                "nav_east_m": float(nav["east_m"]),
                "nav_north_m": float(nav["north_m"]),
                "nav_up_m": float(nav["up_m"]),
                "rtk_east_m": rtk["east_m"],
                "rtk_north_m": rtk["north_m"],
                "rtk_up_m": rtk["up_m"],
                "east_error_m": east_error,
                "north_error_m": north_error,
                "up_error_m": up_error,
                "horizontal_error_m": horizontal_error,
                "position_error_m": position_error,
                "east_error_eval_m": east_error_eval,
                "north_error_eval_m": north_error_eval,
                "up_error_eval_m": up_error_eval,
                "horizontal_error_eval_m": horizontal_error_eval,
                "position_error_eval_m": position_error_eval,
            }
        )
    if not pairs:
        raise ValueError("No matched RTK/navigation samples found")
    return pairs


def rms(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values))


def apply_signed_deadband(value: float, deadband_m: float) -> float:
    if deadband_m <= 0.0:
        return value
    magnitude = max(abs(value) - deadband_m, 0.0)
    if magnitude == 0.0:
        return 0.0
    return math.copysign(magnitude, value)


def apply_magnitude_deadband(value: float, deadband_m: float) -> float:
    if deadband_m <= 0.0:
        return value
    return max(value - deadband_m, 0.0)


def eval_value(
    row: dict[str, float],
    eval_key: str,
    raw_key: str,
    error_deadband_m: float,
    *,
    signed: bool,
) -> float:
    if eval_key in row:
        return row[eval_key]
    raw_value = row[raw_key]
    if signed:
        return apply_signed_deadband(raw_value, error_deadband_m)
    return apply_magnitude_deadband(raw_value, error_deadband_m)


def build_stats_text(
    pairs: list[dict[str, float]],
    error_deadband_m: float = 0.0,
) -> str:
    horizontal = [row["horizontal_error_m"] for row in pairs]
    position = [row["position_error_m"] for row in pairs]
    up = [row["up_error_m"] for row in pairs]
    lines = [
        f"matched={len(pairs)}",
        f"raw horiz mean={statistics.mean(horizontal):.3f} m",
        f"raw horiz rms={rms(horizontal):.3f} m",
        f"raw horiz max={max(horizontal):.3f} m",
        f"raw 3d mean={statistics.mean(position):.3f} m",
        f"raw 3d rms={rms(position):.3f} m",
        f"raw 3d max={max(position):.3f} m",
        f"raw up rms={rms(up):.3f} m",
    ]

    if error_deadband_m > 0.0:
        horizontal_eval = [
            eval_value(
                row,
                "horizontal_error_eval_m",
                "horizontal_error_m",
                error_deadband_m,
                signed=False,
            )
            for row in pairs
        ]
        position_eval = [
            eval_value(
                row,
                "position_error_eval_m",
                "position_error_m",
                error_deadband_m,
                signed=False,
            )
            for row in pairs
        ]
        up_eval = [
            eval_value(
                row,
                "up_error_eval_m",
                "up_error_m",
                error_deadband_m,
                signed=True,
            )
            for row in pairs
        ]
        lines.extend(
            [
                f"eval deadband={error_deadband_m:.3f} m",
                f"eval horiz mean={statistics.mean(horizontal_eval):.3f} m",
                f"eval horiz rms={rms(horizontal_eval):.3f} m",
                f"eval horiz max={max(horizontal_eval):.3f} m",
                f"eval 3d mean={statistics.mean(position_eval):.3f} m",
                f"eval 3d rms={rms(position_eval):.3f} m",
                f"eval 3d max={max(position_eval):.3f} m",
                f"eval up rms={rms(up_eval):.3f} m",
            ]
        )

    return "\n".join(lines)


def make_plot(
    trajectory_rows: list[dict[str, object]],
    rtk_rows: list[dict[str, float]],
    pairs: list[dict[str, float]],
    output_path: Path,
    title: str,
    error_deadband_m: float = 0.0,
) -> None:
    nav_time = [float(row["time_s"]) for row in trajectory_rows]
    nav_east = [float(row["east_m"]) for row in trajectory_rows]
    nav_north = [float(row["north_m"]) for row in trajectory_rows]
    nav_up = [float(row["up_m"]) for row in trajectory_rows]
    rtk_time = [row["time_s"] for row in rtk_rows]
    rtk_east = [row["east_m"] for row in rtk_rows]
    rtk_north = [row["north_m"] for row in rtk_rows]
    rtk_up = [row["up_m"] for row in rtk_rows]

    pair_time = [row["time_s"] for row in pairs]
    east_error = [row["east_error_m"] for row in pairs]
    north_error = [row["north_error_m"] for row in pairs]
    up_error = [row["up_error_m"] for row in pairs]
    horizontal_error = [row["horizontal_error_m"] for row in pairs]
    position_error = [row["position_error_m"] for row in pairs]
    horizontal_error_eval = [
        eval_value(
            row,
            "horizontal_error_eval_m",
            "horizontal_error_m",
            error_deadband_m,
            signed=False,
        )
        for row in pairs
    ]
    position_error_eval = [
        eval_value(
            row,
            "position_error_eval_m",
            "position_error_m",
            error_deadband_m,
            signed=False,
        )
        for row in pairs
    ]

    fig, axes = plt.subplots(2, 2, figsize=(16, 10), constrained_layout=True)
    fig.suptitle(title, fontsize=16)

    ax_traj = axes[0][0]
    ax_traj.plot(nav_east, nav_north, color="#1f77b4", linewidth=1.4, label="offline nav")
    ax_traj.scatter(
        rtk_east,
        rtk_north,
        color="#ff7f0e",
        s=8,
        alpha=0.7,
        label="RTKFIX",
    )
    ax_traj.set_title("East-North trajectory")
    ax_traj.set_xlabel("East [m]")
    ax_traj.set_ylabel("North [m]")
    ax_traj.axis("equal")
    ax_traj.grid(True, alpha=0.3)
    ax_traj.legend()

    ax_up = axes[0][1]
    ax_up.plot(nav_time, nav_up, color="#1f77b4", linewidth=1.2, label="offline nav up")
    ax_up.scatter(rtk_time, rtk_up, color="#ff7f0e", s=8, alpha=0.7, label="RTKFIX up")
    ax_up.set_title("Up component vs time")
    ax_up.set_xlabel("Time [s]")
    ax_up.set_ylabel("Up [m]")
    ax_up.grid(True, alpha=0.3)
    ax_up.legend()

    ax_error = axes[1][0]
    ax_error.plot(pair_time, east_error, linewidth=1.0, label="east error")
    ax_error.plot(pair_time, north_error, linewidth=1.0, label="north error")
    ax_error.plot(pair_time, up_error, linewidth=1.0, label="up error")
    if error_deadband_m > 0.0:
        ax_error.axhspan(
            -error_deadband_m,
            error_deadband_m,
            color="#7f7f7f",
            alpha=0.12,
            label="ignored band",
            zorder=0,
        )
    ax_error.set_title("Navigation minus RTKFIX")
    ax_error.set_xlabel("Time [s]")
    ax_error.set_ylabel("Error [m]")
    ax_error.grid(True, alpha=0.3)
    ax_error.legend()

    ax_norm = axes[1][1]
    ax_norm.plot(pair_time, horizontal_error, color="#2ca02c", linewidth=1.2, label="horizontal error")
    ax_norm.plot(pair_time, position_error, color="#d62728", linewidth=1.2, label="3D error")
    if error_deadband_m > 0.0:
        ax_norm.axhspan(
            0.0,
            error_deadband_m,
            color="#7f7f7f",
            alpha=0.12,
            label="ignored band",
            zorder=0,
        )
        ax_norm.plot(
            pair_time,
            horizontal_error_eval,
            color="#2ca02c",
            linewidth=1.0,
            linestyle="--",
            label="horizontal eval",
        )
        ax_norm.plot(
            pair_time,
            position_error_eval,
            color="#d62728",
            linewidth=1.0,
            linestyle="--",
            label="3D eval",
        )
    ax_norm.set_title("Error magnitude")
    ax_norm.set_xlabel("Time [s]")
    ax_norm.set_ylabel("Magnitude [m]")
    ax_norm.grid(True, alpha=0.3)
    ax_norm.legend(loc="upper right")
    ax_norm.text(
        0.02,
        0.98,
        build_stats_text(pairs, error_deadband_m),
        transform=ax_norm.transAxes,
        va="top",
        ha="left",
        fontsize=10,
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.85},
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def main() -> int:
    args = parse_args()
    if args.error_deadband_m < 0.0:
        raise ValueError("--error-deadband-m must be non-negative")

    trajectory_path = Path(args.trajectory)
    gnss_path = Path(args.gnss)
    output_path = Path(args.output)

    trajectory_rows = read_trajectory(trajectory_path)
    summary = read_summary(Path(args.summary)) if args.summary else {}
    origin_lat_rad, origin_lon_rad, origin_h_m = choose_origin(
        summary, trajectory_rows, gnss_path
    )
    rtk_rows = read_rtk_points(gnss_path, origin_lat_rad, origin_lon_rad, origin_h_m)
    pairs = match_pairs(
        trajectory_rows,
        rtk_rows,
        args.time_tolerance_s,
        args.error_deadband_m,
    )
    make_plot(
        trajectory_rows,
        rtk_rows,
        pairs,
        output_path,
        args.title,
        args.error_deadband_m,
    )

    stats_text = build_stats_text(pairs, args.error_deadband_m)
    print(f"plot_saved={output_path}")
    print(stats_text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
