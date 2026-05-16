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
TIME_EPSILON_S = 1e-9


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot navigation speed against RTK position-difference speed."
    )
    parser.add_argument("--trajectory", required=True, help="Path to trajectory.csv")
    parser.add_argument("--gnss", required=True, help="Path to gnss_solution_gnss_fgo.txt")
    parser.add_argument("--output", required=True, help="Output image path")
    parser.add_argument(
        "--time-tolerance-s",
        type=float,
        default=0.12,
        help="Maximum allowed timestamp mismatch when pairing RTK and navigation speeds",
    )
    parser.add_argument(
        "--speed-window-s",
        type=float,
        default=1.0,
        help="Centered RTK position-difference window in seconds",
    )
    parser.add_argument(
        "--title",
        default="Navigation speed vs RTK position-difference speed",
        help="Figure title",
    )
    return parser.parse_args()


def safe_float(value: str) -> float:
    parsed = float(value)
    if math.isnan(parsed) or math.isinf(parsed):
        raise ValueError("non-finite numeric value")
    return parsed


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


def choose_origin(gnss_path: Path, trajectory_path: Path) -> tuple[float, float, float]:
    with gnss_path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.reader(file, delimiter="\t")
        for raw in reader:
            if not raw:
                continue
            try:
                return safe_float(raw[1]), safe_float(raw[2]), safe_float(raw[3])
            except (IndexError, ValueError):
                continue

    with trajectory_path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        for raw in reader:
            try:
                return (
                    safe_float(raw["lat_rad"]),
                    safe_float(raw["lon_rad"]),
                    safe_float(raw["h_m"]),
                )
            except (KeyError, ValueError):
                continue

    raise ValueError("Unable to determine ENU origin")


def read_nav_speed_rows(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        for raw in reader:
            vx_mps = safe_float(raw["vx_mps"])
            vy_mps = safe_float(raw["vy_mps"])
            vz_mps = safe_float(raw["vz_mps"])
            rows.append(
                {
                    "time_s": safe_float(raw["time_s"]),
                    "vx_mps": vx_mps,
                    "vy_mps": vy_mps,
                    "vz_mps": vz_mps,
                    "speed_mps": math.sqrt(vx_mps * vx_mps + vy_mps * vy_mps + vz_mps * vz_mps),
                }
            )
    if not rows:
        raise ValueError(f"No trajectory rows found in {path}")
    rows.sort(key=lambda row: row["time_s"])
    return rows


def read_rtkfix_rows(
    gnss_path: Path,
    origin_lat_rad: float,
    origin_lon_rad: float,
    origin_h_m: float,
) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with gnss_path.open("r", encoding="utf-8") as file:
        for line in file:
            raw = line.split()
            if not raw:
                continue
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
    rows.sort(key=lambda row: row["time_s"])
    return rows


def find_nearest_index(times: list[float], target_time_s: float) -> int:
    index = bisect_left(times, target_time_s)
    candidates: list[int] = []
    if index < len(times):
        candidates.append(index)
    if index > 0:
        candidates.append(index - 1)
    if not candidates:
        raise ValueError("No timestamps available for nearest-neighbor lookup")
    return min(candidates, key=lambda candidate: abs(times[candidate] - target_time_s))


def find_index_at_or_before(times: list[float], target_time_s: float) -> int:
    index = bisect_left(times, target_time_s)
    if index < len(times) and times[index] == target_time_s:
        return index
    return index - 1


def find_index_at_or_after(times: list[float], target_time_s: float) -> int:
    return bisect_left(times, target_time_s)


def build_rtk_speed_rows(
    rtk_rows: list[dict[str, float]],
    speed_window_s: float,
) -> list[dict[str, float | bool | str]]:
    if speed_window_s <= 0.0:
        raise ValueError("speed_window_s must be positive")

    half_window_s = 0.5 * speed_window_s
    times = [row["time_s"] for row in rtk_rows]
    speed_rows: list[dict[str, float | bool | str]] = []

    for center_index, center_row in enumerate(rtk_rows):
        center_time_s = center_row["time_s"]
        row: dict[str, float | bool | str] = {
            "time_s": center_time_s,
            "window_dt_s": math.nan,
            "vx_mps": math.nan,
            "vy_mps": math.nan,
            "vz_mps": math.nan,
            "speed_mps": math.nan,
            "valid_speed": False,
            "invalid_reason": "boundary",
        }

        target_left_s = center_time_s - half_window_s
        target_right_s = center_time_s + half_window_s
        if target_left_s < times[0] or target_right_s > times[-1]:
            speed_rows.append(row)
            continue

        left_index = find_index_at_or_before(times, target_left_s)
        right_index = find_index_at_or_after(times, target_right_s)
        if left_index >= center_index or right_index <= center_index or left_index >= right_index:
            speed_rows.append(row)
            continue

        left_gap_s = target_left_s - times[left_index]
        right_gap_s = times[right_index] - target_right_s
        if left_gap_s > half_window_s + TIME_EPSILON_S or right_gap_s > half_window_s + TIME_EPSILON_S:
            row["invalid_reason"] = "large_gap"
            speed_rows.append(row)
            continue

        dt_s = times[right_index] - times[left_index]
        if dt_s <= 0.0:
            row["invalid_reason"] = "nonpositive_dt"
            speed_rows.append(row)
            continue

        delta_east_m = rtk_rows[right_index]["east_m"] - rtk_rows[left_index]["east_m"]
        delta_north_m = rtk_rows[right_index]["north_m"] - rtk_rows[left_index]["north_m"]
        delta_up_m = rtk_rows[right_index]["up_m"] - rtk_rows[left_index]["up_m"]

        vx_mps = delta_east_m / dt_s
        vy_mps = delta_north_m / dt_s
        vz_mps = delta_up_m / dt_s

        row["window_dt_s"] = dt_s
        row["vx_mps"] = vx_mps
        row["vy_mps"] = vy_mps
        row["vz_mps"] = vz_mps
        row["speed_mps"] = math.sqrt(vx_mps * vx_mps + vy_mps * vy_mps + vz_mps * vz_mps)
        row["valid_speed"] = True
        row["invalid_reason"] = "valid"
        speed_rows.append(row)

    return speed_rows


def match_speed_pairs(
    nav_rows: list[dict[str, float]],
    rtk_speed_rows: list[dict[str, float | bool | str]],
    tolerance_s: float,
) -> list[dict[str, float]]:
    nav_times = [row["time_s"] for row in nav_rows]
    pairs: list[dict[str, float]] = []
    for rtk_row in rtk_speed_rows:
        if not rtk_row["valid_speed"]:
            continue

        rtk_time_s = float(rtk_row["time_s"])
        nav_index = find_nearest_index(nav_times, rtk_time_s)
        dt_s = abs(nav_times[nav_index] - rtk_time_s)
        if dt_s > tolerance_s:
            continue

        nav_row = nav_rows[nav_index]
        vx_error_mps = nav_row["vx_mps"] - float(rtk_row["vx_mps"])
        vy_error_mps = nav_row["vy_mps"] - float(rtk_row["vy_mps"])
        vz_error_mps = nav_row["vz_mps"] - float(rtk_row["vz_mps"])
        speed_error_mps = nav_row["speed_mps"] - float(rtk_row["speed_mps"])
        pairs.append(
            {
                "time_s": rtk_time_s,
                "nav_vx_mps": nav_row["vx_mps"],
                "nav_vy_mps": nav_row["vy_mps"],
                "nav_vz_mps": nav_row["vz_mps"],
                "nav_speed_mps": nav_row["speed_mps"],
                "rtk_vx_mps": float(rtk_row["vx_mps"]),
                "rtk_vy_mps": float(rtk_row["vy_mps"]),
                "rtk_vz_mps": float(rtk_row["vz_mps"]),
                "rtk_speed_mps": float(rtk_row["speed_mps"]),
                "vx_error_mps": vx_error_mps,
                "vy_error_mps": vy_error_mps,
                "vz_error_mps": vz_error_mps,
                "speed_error_mps": speed_error_mps,
                "window_dt_s": float(rtk_row["window_dt_s"]),
            }
        )

    if not pairs:
        raise ValueError("No matched navigation/RTK speed samples found")
    return pairs


def rms(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values))


def build_stats_text(pairs: list[dict[str, float]]) -> str:
    speed_error = [pair["speed_error_mps"] for pair in pairs]
    vx_error = [pair["vx_error_mps"] for pair in pairs]
    vy_error = [pair["vy_error_mps"] for pair in pairs]
    vz_error = [pair["vz_error_mps"] for pair in pairs]
    abs_speed_error = [abs(value) for value in speed_error]
    return "\n".join(
        [
            f"matched={len(pairs)}",
            f"speed mean abs={statistics.mean(abs_speed_error):.3f} m/s",
            f"speed rms={rms(speed_error):.3f} m/s",
            f"vx rms={rms(vx_error):.3f} m/s",
            f"vy rms={rms(vy_error):.3f} m/s",
            f"vz rms={rms(vz_error):.3f} m/s",
        ]
    )


def make_plot(
    pairs: list[dict[str, float]],
    output_path: Path,
    title: str,
) -> None:
    time_s = [pair["time_s"] for pair in pairs]

    fig, axes = plt.subplots(4, 1, figsize=(16, 13), sharex=True, constrained_layout=True)
    fig.suptitle(title, fontsize=16)

    series = [
        ("Speed magnitude", "nav_speed_mps", "rtk_speed_mps", "Speed [m/s]"),
        ("East velocity", "nav_vx_mps", "rtk_vx_mps", "Vx [m/s]"),
        ("North velocity", "nav_vy_mps", "rtk_vy_mps", "Vy [m/s]"),
        ("Up velocity", "nav_vz_mps", "rtk_vz_mps", "Vz [m/s]"),
    ]

    for axis, (subtitle, nav_key, rtk_key, ylabel) in zip(axes, series):
        axis.plot(
            time_s,
            [pair[nav_key] for pair in pairs],
            color="#1f77b4",
            linewidth=1.2,
            label="navigation",
        )
        axis.plot(
            time_s,
            [pair[rtk_key] for pair in pairs],
            color="#ff7f0e",
            linewidth=1.0,
            label="RTK diff",
        )
        axis.set_ylabel(ylabel)
        axis.set_title(subtitle)
        axis.grid(True, alpha=0.3)
        axis.legend(loc="upper right")

    axes[0].text(
        0.01,
        0.97,
        build_stats_text(pairs),
        transform=axes[0].transAxes,
        va="top",
        ha="left",
        fontsize=10,
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.85},
    )

    axes[-1].set_xlabel("Time [s]")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def main() -> int:
    args = parse_args()
    trajectory_path = Path(args.trajectory)
    gnss_path = Path(args.gnss)
    output_path = Path(args.output)

    nav_rows = read_nav_speed_rows(trajectory_path)
    origin_lat_rad, origin_lon_rad, origin_h_m = choose_origin(gnss_path, trajectory_path)
    rtk_rows = read_rtkfix_rows(gnss_path, origin_lat_rad, origin_lon_rad, origin_h_m)
    rtk_speed_rows = build_rtk_speed_rows(rtk_rows, args.speed_window_s)
    pairs = match_speed_pairs(nav_rows, rtk_speed_rows, args.time_tolerance_s)
    make_plot(pairs, output_path, args.title)

    valid_speed_count = sum(1 for row in rtk_speed_rows if row["valid_speed"])
    invalid_speed_count = len(rtk_speed_rows) - valid_speed_count
    print(f"plot_saved={output_path}")
    print(f"rtkfix_count={len(rtk_rows)}")
    print(f"valid_rtk_speed_count={valid_speed_count}")
    print(f"invalid_rtk_speed_count={invalid_speed_count}")
    print(build_stats_text(pairs))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
