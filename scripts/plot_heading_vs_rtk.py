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
MIN_HEADING_DISPLACEMENT_M = 0.2
TIME_EPSILON_S = 1e-9


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot navigation heading against RTK position-difference heading."
    )
    parser.add_argument("--trajectory", required=True, help="Path to trajectory.csv")
    parser.add_argument(
        "--gnss",
        required=True,
        help="Path to gnss_solution_gnss_fgo.txt",
    )
    parser.add_argument(
        "--output",
        required=True,
        help="Output image path, for example heading_vs_rtk_heading.png",
    )
    parser.add_argument(
        "--time-tolerance-s",
        type=float,
        default=0.12,
        help="Maximum allowed timestamp mismatch when pairing RTK and navigation headings",
    )
    parser.add_argument(
        "--heading-window-s",
        type=float,
        default=1.0,
        help="Centered RTK position-difference window in seconds",
    )
    parser.add_argument(
        "--title",
        default="Navigation heading vs RTK position-difference heading",
        help="Figure title",
    )
    return parser.parse_args()


def safe_float(value: str) -> float:
    parsed = float(value)
    if math.isnan(parsed) or math.isinf(parsed):
        raise ValueError("non-finite numeric value")
    return parsed


def wrap_angle_deg(angle_deg: float) -> float:
    wrapped = (angle_deg + 180.0) % 360.0 - 180.0
    if wrapped == -180.0:
        return 180.0
    return wrapped


def unwrap_angles_deg(angles_deg: list[float]) -> list[float]:
    if not angles_deg:
        return []
    unwrapped = [angles_deg[0]]
    for angle_deg in angles_deg[1:]:
        candidate = angle_deg
        delta = candidate - unwrapped[-1]
        while delta > 180.0:
            candidate -= 360.0
            delta = candidate - unwrapped[-1]
        while delta < -180.0:
            candidate += 360.0
            delta = candidate - unwrapped[-1]
        unwrapped.append(candidate)
    return unwrapped


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


def read_nav_heading_rows(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        for raw in reader:
            rows.append(
                {
                    "time_s": safe_float(raw["time_s"]),
                    "yaw_deg": math.degrees(safe_float(raw["yaw_rad"])),
                }
            )
    if not rows:
        raise ValueError(f"No trajectory rows found in {path}")

    rows.sort(key=lambda row: row["time_s"])
    unwrapped = unwrap_angles_deg([row["yaw_deg"] for row in rows])
    for row, yaw_unwrapped_deg in zip(rows, unwrapped):
        row["yaw_unwrapped_deg"] = yaw_unwrapped_deg
    return rows


def read_rtkfix_rows(
    gnss_path: Path,
    origin_lat_rad: float,
    origin_lon_rad: float,
    origin_h_m: float,
) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with gnss_path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.reader(file, delimiter="\t")
        for raw in reader:
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


def build_rtk_heading_rows(
    rtk_rows: list[dict[str, float]],
    heading_window_s: float,
) -> list[dict[str, float | bool | str]]:
    if heading_window_s <= 0.0:
        raise ValueError("heading_window_s must be positive")

    half_window_s = 0.5 * heading_window_s
    times = [row["time_s"] for row in rtk_rows]
    heading_rows: list[dict[str, float | bool | str]] = []

    for center_index, center_row in enumerate(rtk_rows):
        center_time_s = center_row["time_s"]
        row: dict[str, float | bool | str] = {
            "time_s": center_time_s,
            "window_displacement_m": math.nan,
            "heading_deg": math.nan,
            "valid_heading": False,
            "invalid_reason": "boundary",
        }

        target_left_s = center_time_s - half_window_s
        target_right_s = center_time_s + half_window_s
        if target_left_s < times[0] or target_right_s > times[-1]:
            heading_rows.append(row)
            continue

        left_index = find_index_at_or_before(times, target_left_s)
        right_index = find_index_at_or_after(times, target_right_s)
        if left_index >= center_index or right_index <= center_index or left_index >= right_index:
            heading_rows.append(row)
            continue

        left_gap_s = target_left_s - times[left_index]
        right_gap_s = times[right_index] - target_right_s
        if left_gap_s > half_window_s + TIME_EPSILON_S or right_gap_s > half_window_s + TIME_EPSILON_S:
            row["invalid_reason"] = "large_gap"
            heading_rows.append(row)
            continue

        delta_east_m = rtk_rows[right_index]["east_m"] - rtk_rows[left_index]["east_m"]
        delta_north_m = rtk_rows[right_index]["north_m"] - rtk_rows[left_index]["north_m"]
        displacement_m = math.hypot(delta_east_m, delta_north_m)
        row["window_displacement_m"] = displacement_m

        if displacement_m < MIN_HEADING_DISPLACEMENT_M:
            row["invalid_reason"] = "low_displacement"
            heading_rows.append(row)
            continue

        row["heading_deg"] = math.degrees(math.atan2(delta_north_m, delta_east_m))
        row["valid_heading"] = True
        row["invalid_reason"] = "valid"
        heading_rows.append(row)

    valid_heading_angles_deg = [
        float(row["heading_deg"])
        for row in heading_rows
        if row["valid_heading"]
    ]
    valid_heading_unwrapped_deg = unwrap_angles_deg(valid_heading_angles_deg)
    valid_index = 0
    for row in heading_rows:
        if not row["valid_heading"]:
            row["heading_unwrapped_deg"] = math.nan
            continue
        row["heading_unwrapped_deg"] = valid_heading_unwrapped_deg[valid_index]
        valid_index += 1

    return heading_rows


def match_heading_pairs(
    nav_rows: list[dict[str, float]],
    rtk_heading_rows: list[dict[str, float | bool | str]],
    tolerance_s: float,
) -> list[dict[str, float]]:
    nav_times = [row["time_s"] for row in nav_rows]
    pairs: list[dict[str, float]] = []
    for rtk_row in rtk_heading_rows:
        if not rtk_row["valid_heading"]:
            continue

        rtk_time_s = float(rtk_row["time_s"])
        nav_index = find_nearest_index(nav_times, rtk_time_s)
        dt_s = abs(nav_times[nav_index] - rtk_time_s)
        if dt_s > tolerance_s:
            continue

        nav_row = nav_rows[nav_index]
        nav_heading_deg = float(nav_row["yaw_deg"])
        rtk_heading_deg = float(rtk_row["heading_deg"])
        error_deg = wrap_angle_deg(nav_heading_deg - rtk_heading_deg)
        pairs.append(
            {
                "time_s": rtk_time_s,
                "nav_heading_deg": nav_heading_deg,
                "nav_heading_unwrapped_deg": float(nav_row["yaw_unwrapped_deg"]),
                "rtk_heading_deg": rtk_heading_deg,
                "rtk_heading_unwrapped_deg": float(rtk_row["heading_unwrapped_deg"]),
                "heading_error_deg": error_deg,
                "window_displacement_m": float(rtk_row["window_displacement_m"]),
            }
        )
    if not pairs:
        raise ValueError("No matched navigation/RTK heading samples found")
    return pairs


def aligned_rtk_unwrapped_deg(pairs: list[dict[str, float]]) -> list[float]:
    offsets_deg = [
        pair["nav_heading_unwrapped_deg"] - pair["rtk_heading_unwrapped_deg"]
        for pair in pairs
    ]
    shift_deg = 360.0 * round(statistics.median(offsets_deg) / 360.0)
    return [pair["rtk_heading_unwrapped_deg"] + shift_deg for pair in pairs]


def rms(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values))


def build_stats_text(pairs: list[dict[str, float]]) -> str:
    error_deg = [pair["heading_error_deg"] for pair in pairs]
    abs_error_deg = [abs(value) for value in error_deg]
    return "\n".join(
        [
            f"matched={len(pairs)}",
            f"mean abs={statistics.mean(abs_error_deg):.3f} deg",
            f"rms={rms(error_deg):.3f} deg",
            f"max abs={max(abs_error_deg):.3f} deg",
        ]
    )


def make_plot(
    nav_rows: list[dict[str, float]],
    rtk_heading_rows: list[dict[str, float | bool | str]],
    pairs: list[dict[str, float]],
    output_path: Path,
    title: str,
) -> None:
    nav_time = [row["time_s"] for row in nav_rows]
    nav_heading_unwrapped_deg = [row["yaw_unwrapped_deg"] for row in nav_rows]

    pair_time = [pair["time_s"] for pair in pairs]
    pair_nav_heading_unwrapped_deg = [pair["nav_heading_unwrapped_deg"] for pair in pairs]
    pair_rtk_heading_unwrapped_deg = aligned_rtk_unwrapped_deg(pairs)
    pair_heading_error_deg = [pair["heading_error_deg"] for pair in pairs]

    valid_rtk_time = [
        float(row["time_s"])
        for row in rtk_heading_rows
        if row["valid_heading"]
    ]
    valid_rtk_displacement_m = [
        float(row["window_displacement_m"])
        for row in rtk_heading_rows
        if row["valid_heading"]
    ]
    invalid_rtk_time = [
        float(row["time_s"])
        for row in rtk_heading_rows
        if not row["valid_heading"]
    ]
    invalid_rtk_displacement_m = [
        float(row["window_displacement_m"])
        if math.isfinite(float(row["window_displacement_m"]))
        else 0.0
        for row in rtk_heading_rows
        if not row["valid_heading"]
    ]

    fig, axes = plt.subplots(3, 1, figsize=(16, 11), sharex=True, constrained_layout=True)
    fig.suptitle(title, fontsize=16)

    ax_heading = axes[0]
    ax_heading.plot(
        nav_time,
        nav_heading_unwrapped_deg,
        color="#1f77b4",
        linewidth=1.2,
        label="navigation heading",
    )
    ax_heading.plot(
        pair_time,
        pair_rtk_heading_unwrapped_deg,
        color="#ff7f0e",
        linewidth=1.0,
        marker="o",
        markersize=2.5,
        label="RTK diff heading",
    )
    ax_heading.set_ylabel("Heading [deg]")
    ax_heading.set_title("Heading comparison")
    ax_heading.grid(True, alpha=0.3)
    ax_heading.legend(loc="upper right")

    ax_error = axes[1]
    ax_error.plot(
        pair_time,
        pair_heading_error_deg,
        color="#d62728",
        linewidth=1.0,
        label="heading error",
    )
    ax_error.axhline(0.0, color="black", linewidth=0.8, alpha=0.6)
    ax_error.set_ylabel("Error [deg]")
    ax_error.set_title("Navigation heading minus RTK diff heading")
    ax_error.grid(True, alpha=0.3)
    ax_error.legend(loc="upper right")
    ax_error.text(
        0.01,
        0.97,
        build_stats_text(pairs),
        transform=ax_error.transAxes,
        va="top",
        ha="left",
        fontsize=10,
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.85},
    )

    ax_disp = axes[2]
    ax_disp.plot(
        valid_rtk_time,
        valid_rtk_displacement_m,
        color="#2ca02c",
        linewidth=1.0,
        label="valid RTK window displacement",
    )
    if invalid_rtk_time:
        ax_disp.scatter(
            invalid_rtk_time,
            invalid_rtk_displacement_m,
            color="#d62728",
            s=14,
            marker="x",
            label="invalid heading window",
        )
    ax_disp.axhline(
        MIN_HEADING_DISPLACEMENT_M,
        color="#9467bd",
        linewidth=0.9,
        linestyle="--",
        label="min displacement threshold",
    )
    ax_disp.set_xlabel("Time [s]")
    ax_disp.set_ylabel("Displacement [m]")
    ax_disp.set_title("RTK heading window displacement")
    ax_disp.grid(True, alpha=0.3)
    ax_disp.legend(loc="upper right")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def main() -> int:
    args = parse_args()
    trajectory_path = Path(args.trajectory)
    gnss_path = Path(args.gnss)
    output_path = Path(args.output)

    nav_rows = read_nav_heading_rows(trajectory_path)
    origin_lat_rad, origin_lon_rad, origin_h_m = choose_origin(gnss_path, trajectory_path)
    rtk_rows = read_rtkfix_rows(gnss_path, origin_lat_rad, origin_lon_rad, origin_h_m)
    rtk_heading_rows = build_rtk_heading_rows(rtk_rows, args.heading_window_s)
    pairs = match_heading_pairs(nav_rows, rtk_heading_rows, args.time_tolerance_s)
    make_plot(nav_rows, rtk_heading_rows, pairs, output_path, args.title)

    valid_heading_count = sum(1 for row in rtk_heading_rows if row["valid_heading"])
    invalid_heading_count = len(rtk_heading_rows) - valid_heading_count
    print(f"plot_saved={output_path}")
    print(f"rtkfix_count={len(rtk_rows)}")
    print(f"valid_rtk_heading_count={valid_heading_count}")
    print(f"invalid_rtk_heading_count={invalid_heading_count}")
    print(build_stats_text(pairs))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
