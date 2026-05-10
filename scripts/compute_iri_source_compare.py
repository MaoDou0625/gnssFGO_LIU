#!/usr/bin/env python

from __future__ import annotations

import argparse
import csv
import math
import subprocess
import sys
from bisect import bisect_left
from pathlib import Path


DEFAULT_IRI_SCRIPT = Path("D:/Code/michalsorel_iri_repo/python/iri.py")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare 50 m IRI from nav, raw RTK, low-pass/latent RTK, and nav sampled at RTK times."
    )
    parser.add_argument("--run-dir", required=True, help="Offline run output directory")
    parser.add_argument("--output-dir", default=None, help="Output directory")
    parser.add_argument("--iri-script", default=str(DEFAULT_IRI_SCRIPT), help="Michal Sorel iri.py path")
    parser.add_argument("--grid-step-m", type=float, default=0.25, help="Spatial resampling step")
    parser.add_argument("--segment-length-m", type=float, default=50.0, help="IRI segment length")
    parser.add_argument("--speed-threshold-mps", type=float, default=0.5, help="Trim stationary ends below this speed")
    return parser.parse_args()


def safe_float(value: str | None) -> float:
    if value is None or value == "":
        return math.nan
    try:
        return float(value)
    except ValueError:
        return math.nan


def read_trajectory(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        for raw in reader:
            time_s = safe_float(raw.get("time_s"))
            east_m = safe_float(raw.get("east_m"))
            north_m = safe_float(raw.get("north_m"))
            up_m = safe_float(raw.get("up_m"))
            vx_mps = safe_float(raw.get("vx_mps"))
            vy_mps = safe_float(raw.get("vy_mps"))
            if not all(math.isfinite(value) for value in (time_s, east_m, north_m, up_m)):
                continue
            rows.append(
                {
                    "time_s": time_s,
                    "east_m": east_m,
                    "north_m": north_m,
                    "up_m": up_m,
                    "speed_mps": math.hypot(vx_mps, vy_mps)
                    if math.isfinite(vx_mps) and math.isfinite(vy_mps)
                    else math.nan,
                }
            )
    rows.sort(key=lambda row: row["time_s"])
    if len(rows) < 2:
        raise ValueError(f"trajectory has too few rows: {path}")
    distance = 0.0
    rows[0]["distance_m"] = 0.0
    for index in range(1, len(rows)):
        distance += math.hypot(
            rows[index]["east_m"] - rows[index - 1]["east_m"],
            rows[index]["north_m"] - rows[index - 1]["north_m"],
        )
        rows[index]["distance_m"] = distance
    return rows


def trim_by_speed(rows: list[dict[str, float]], threshold_mps: float) -> list[dict[str, float]]:
    moving_indices = [
        index
        for index, row in enumerate(rows)
        if math.isfinite(row.get("speed_mps", math.nan)) and row["speed_mps"] >= threshold_mps
    ]
    if not moving_indices:
        return rows
    return rows[moving_indices[0] : moving_indices[-1] + 1]


def read_envelope_rows(path: Path) -> list[dict[str, float]]:
    if not path.exists():
        return []
    rows: list[dict[str, float]] = []
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        for raw in reader:
            if raw.get("factor_used", "1") == "0":
                continue
            time_s = safe_float(raw.get("corrected_time_s"))
            up_m = safe_float(raw.get("rtk_up_m"))
            if math.isfinite(time_s) and math.isfinite(up_m):
                rows.append({"time_s": time_s, "up_m": up_m})
    rows.sort(key=lambda row: row["time_s"])
    return rows


def read_lowpass_rows(path: Path) -> list[dict[str, float]]:
    if not path.exists():
        return []
    rows: list[dict[str, float]] = []
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        for raw in reader:
            if raw.get("lowpass_valid", "0") != "1":
                continue
            time_s = safe_float(raw.get("time_s"))
            up_m = safe_float(raw.get("lowpass_up_m"))
            if math.isfinite(time_s) and math.isfinite(up_m):
                rows.append({"time_s": time_s, "up_m": up_m})
    rows.sort(key=lambda row: row["time_s"])
    return rows


def read_latent_reference_rows(path: Path) -> list[dict[str, float]]:
    if not path.exists():
        return []
    rows: list[dict[str, float]] = []
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        for raw in reader:
            time_s = safe_float(raw.get("bin_center_time_s"))
            up_m = safe_float(raw.get("optimized_reference_up_m"))
            if not math.isfinite(up_m):
                up_m = safe_float(raw.get("initial_reference_up_m"))
            if math.isfinite(time_s) and math.isfinite(up_m):
                rows.append({"time_s": time_s, "up_m": up_m})
    rows.sort(key=lambda row: row["time_s"])
    return rows


def interpolate_series(rows: list[dict[str, float]], key: str, time_s: float) -> float:
    times = [row["time_s"] for row in rows]
    index = bisect_left(times, time_s)
    if index == 0:
        return rows[0][key]
    if index >= len(rows):
        return rows[-1][key]
    before = rows[index - 1]
    after = rows[index]
    span = after["time_s"] - before["time_s"]
    if span <= 0.0:
        return before[key]
    alpha = (time_s - before["time_s"]) / span
    return before[key] + alpha * (after[key] - before[key])


def build_time_sampled_profile(
    source_rows: list[dict[str, float]],
    trajectory_rows: list[dict[str, float]],
    trim_start_time_s: float,
    trim_end_time_s: float,
) -> tuple[list[float], list[float]]:
    stations: list[float] = []
    heights: list[float] = []
    for row in source_rows:
        time_s = row["time_s"]
        if time_s < trim_start_time_s or time_s > trim_end_time_s:
            continue
        station_m = interpolate_series(trajectory_rows, "distance_m", time_s)
        stations.append(station_m)
        heights.append(row["up_m"])
    if stations:
        station0 = stations[0]
        height0 = heights[0]
        stations = [station - station0 for station in stations]
        heights = [height - height0 for height in heights]
    return stations, heights


def resample_regular_grid(
    stations: list[float],
    heights: list[float],
    grid_step_m: float,
) -> tuple[list[float], list[float]]:
    if len(stations) < 2:
        return [], []
    unique_stations: list[float] = []
    unique_heights: list[float] = []
    last_station: float | None = None
    for station, height in sorted(zip(stations, heights), key=lambda pair: pair[0]):
        if last_station is not None and abs(station - last_station) <= 1e-9:
            continue
        unique_stations.append(station)
        unique_heights.append(height)
        last_station = station
    if len(unique_stations) < 2 or unique_stations[-1] < grid_step_m:
        return [], []
    grid_count = int(math.floor(unique_stations[-1] / grid_step_m)) + 1
    grid_stations = [index * grid_step_m for index in range(grid_count)]
    grid_heights = [
        interpolate_by_station(unique_stations, unique_heights, station)
        for station in grid_stations
    ]
    return grid_stations, grid_heights


def interpolate_by_station(stations: list[float], heights: list[float], station_m: float) -> float:
    index = bisect_left(stations, station_m)
    if index == 0:
        return heights[0]
    if index >= len(stations):
        return heights[-1]
    span = stations[index] - stations[index - 1]
    if span <= 0.0:
        return heights[index - 1]
    alpha = (station_m - stations[index - 1]) / span
    return heights[index - 1] + alpha * (heights[index] - heights[index - 1])


def write_profile(path: Path, stations: list[float], heights: list[float]) -> None:
    with path.open("w", encoding="utf-8", newline="") as file:
        for station, height in zip(stations, heights):
            file.write(f"{station:.6f} {height:.12f}\n")


def run_iri(iri_script: Path, profile_path: Path, output_path: Path, segment_length_m: float) -> None:
    wrapper = (
        "import runpy, sys, numpy as np; "
        "script=sys.argv[1]; "
        "np.in1d=getattr(np,'in1d',np.isin); "
        "sys.argv=[script]+sys.argv[2:]; "
        "runpy.run_path(script, run_name='__main__')"
    )
    subprocess.run(
        [
            sys.executable,
            "-c",
            wrapper,
            str(iri_script),
            str(profile_path),
            str(output_path),
            "-segment_length",
            str(segment_length_m),
            "-start_pos",
            "0",
            "-method",
            "2",
        ],
        check=True,
    )


def parse_iri(path: Path) -> tuple[float, float]:
    values: list[float] = []
    with path.open("r", encoding="utf-8") as file:
        for line in file:
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            tokens = stripped.split()
            if len(tokens) >= 3:
                value = safe_float(tokens[2])
                if math.isfinite(value):
                    values.append(value)
    if not values:
        return math.nan, math.nan
    return sum(values) / len(values), max(values)


def series_stats(values: list[float]) -> tuple[float, float]:
    if not values:
        return math.nan, math.nan
    value_range = max(values) - min(values)
    mean = sum(values) / len(values)
    variance = sum((value - mean) ** 2 for value in values) / len(values)
    return value_range, math.sqrt(variance)


def process_source(
    name: str,
    stations: list[float],
    heights: list[float],
    output_dir: Path,
    iri_script: Path,
    grid_step_m: float,
    segment_length_m: float,
) -> dict[str, float | str | int]:
    grid_stations, grid_heights = resample_regular_grid(stations, heights, grid_step_m)
    profile_path = output_dir / f"{name}_space_profile_0p25m.txt"
    iri_path = output_dir / f"{name}_iri_50m.txt"
    if iri_path.exists():
        iri_path.unlink()
    write_profile(profile_path, grid_stations, grid_heights)
    iri_ran = False
    if len(grid_stations) >= 2:
        run_iri(iri_script, profile_path, iri_path, segment_length_m)
        iri_ran = True
    raw_range, raw_std = series_stats(heights)
    grid_range, grid_std = series_stats(grid_heights)
    iri_mean, iri_max = parse_iri(iri_path) if iri_ran and iri_path.exists() else (math.nan, math.nan)
    return {
        "source": name,
        "sample_count": len(heights),
        "distance_m": stations[-1] - stations[0] if len(stations) >= 2 else math.nan,
        "resampled_distance_m": grid_stations[-1] if grid_stations else math.nan,
        "up_range_m": raw_range,
        "up_std_m": raw_std,
        "grid_up_range_m": grid_range,
        "grid_up_std_m": grid_std,
        "iri_mean_mm_per_m": iri_mean,
        "iri_max_mm_per_m": iri_max,
    }


def main() -> int:
    args = parse_args()
    run_dir = Path(args.run_dir)
    output_dir = Path(args.output_dir) if args.output_dir else run_dir / "iri_50m_source_compare"
    output_dir.mkdir(parents=True, exist_ok=True)
    iri_script = Path(args.iri_script)
    if not iri_script.exists():
        raise FileNotFoundError(f"IRI script not found: {iri_script}")

    trajectory_rows = read_trajectory(run_dir / "trajectory.csv")
    trimmed_nav = trim_by_speed(trajectory_rows, args.speed_threshold_mps)
    trim_start_time_s = trimmed_nav[0]["time_s"]
    trim_end_time_s = trimmed_nav[-1]["time_s"]
    nav_station0 = trimmed_nav[0]["distance_m"]
    nav_height0 = trimmed_nav[0]["up_m"]
    nav_stations = [row["distance_m"] - nav_station0 for row in trimmed_nav]
    nav_heights = [row["up_m"] - nav_height0 for row in trimmed_nav]

    raw_rtk_rows = read_envelope_rows(run_dir / "vertical_envelope_diagnostics.csv")
    lowpass_rtk_rows = read_lowpass_rows(run_dir / "rtk_vertical_lowpass_reference_diagnostics.csv")
    latent_rtk_rows = read_latent_reference_rows(run_dir / "rtk_vertical_latent_reference_diagnostics.csv")
    rtk_stations, rtk_heights = build_time_sampled_profile(
        raw_rtk_rows,
        trajectory_rows,
        trim_start_time_s,
        trim_end_time_s,
    )
    lowpass_stations, lowpass_heights = build_time_sampled_profile(
        lowpass_rtk_rows,
        trajectory_rows,
        trim_start_time_s,
        trim_end_time_s,
    )
    latent_stations, latent_heights = build_time_sampled_profile(
        latent_rtk_rows,
        trajectory_rows,
        trim_start_time_s,
        trim_end_time_s,
    )
    nav_at_rtk_stations, nav_at_rtk_heights = build_time_sampled_profile(
        [
            {
                "time_s": row["time_s"],
                "up_m": interpolate_series(trajectory_rows, "up_m", row["time_s"]),
            }
            for row in raw_rtk_rows
        ],
        trajectory_rows,
        trim_start_time_s,
        trim_end_time_s,
    )

    summaries = [
        process_source(
            "nav_highrate",
            nav_stations,
            nav_heights,
            output_dir,
            iri_script,
            args.grid_step_m,
            args.segment_length_m,
        ),
        process_source(
            "rtk_raw",
            rtk_stations,
            rtk_heights,
            output_dir,
            iri_script,
            args.grid_step_m,
            args.segment_length_m,
        ),
    ]
    if lowpass_stations and lowpass_heights:
        summaries.append(
            process_source(
                "rtk_lowpass",
                lowpass_stations,
                lowpass_heights,
                output_dir,
                iri_script,
                args.grid_step_m,
                args.segment_length_m,
            )
        )
    if latent_stations and latent_heights:
        summaries.append(
            process_source(
                "rtk_latent_reference",
                latent_stations,
                latent_heights,
                output_dir,
                iri_script,
                args.grid_step_m,
                args.segment_length_m,
            )
        )
    summaries.append(
        process_source(
            "nav_at_rtk_station",
            nav_at_rtk_stations,
            nav_at_rtk_heights,
            output_dir,
            iri_script,
            args.grid_step_m,
            args.segment_length_m,
        )
    )

    summary_path = output_dir / "source_iri_summary.csv"
    fieldnames = [
        "source",
        "sample_count",
        "distance_m",
        "resampled_distance_m",
        "up_range_m",
        "up_std_m",
        "grid_up_range_m",
        "grid_up_std_m",
        "iri_mean_mm_per_m",
        "iri_max_mm_per_m",
    ]
    with summary_path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(summaries)
    print(f"summary={summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
