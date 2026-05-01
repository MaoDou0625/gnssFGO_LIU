#!/usr/bin/env python

from __future__ import annotations

import argparse
import csv
import importlib.util
import math
import statistics
from pathlib import Path

try:
    import matplotlib.pyplot as plt
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "matplotlib is required to generate plots. "
        "Use a Python environment with matplotlib installed."
    ) from exc


SPEED_HELPER_PATH = Path(__file__).resolve().with_name("plot_speed_vs_rtk.py")
SPEED_HELPER_SPEC = importlib.util.spec_from_file_location("plot_speed_vs_rtk", SPEED_HELPER_PATH)
plot_speed_vs_rtk = importlib.util.module_from_spec(SPEED_HELPER_SPEC)
SPEED_HELPER_SPEC.loader.exec_module(plot_speed_vs_rtk)

TIME_EPSILON_S = 1e-9


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot optimized full-duration vertical position and vertical speed against RTK."
    )
    parser.add_argument("--trajectory", required=True, help="Path to trajectory.csv")
    parser.add_argument("--gnss", required=True, help="Path to gnss_solution_gnss_fgo.txt")
    parser.add_argument("--output", required=True, help="Output image path")
    parser.add_argument("--csv-output", default="", help="Optional aligned CSV output path")
    parser.add_argument(
        "--envelope-diagnostics",
        default="",
        help="Optional vertical_envelope_diagnostics.csv path; defaults to the trajectory directory when present",
    )
    parser.add_argument("--speed-window-s", type=float, default=1.0, help="Centered RTK speed-difference window in seconds")
    parser.add_argument("--title", default="Optimized vertical position and velocity over full duration", help="Figure title")
    return parser.parse_args()


def read_trajectory_rows(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        for raw in reader:
            rows.append(
                {
                    "time_s": plot_speed_vs_rtk.safe_float(raw["time_s"]),
                    "up_m": plot_speed_vs_rtk.safe_float(raw["up_m"]),
                    "vz_mps": plot_speed_vs_rtk.safe_float(raw["vz_mps"]),
                }
            )
    if not rows:
        raise ValueError(f"No trajectory rows found in {path}")
    rows.sort(key=lambda row: row["time_s"])
    return rows


def find_nearest_index(times: list[float], target_time_s: float) -> int:
    return plot_speed_vs_rtk.find_nearest_index(times, target_time_s)


def filter_valid_rtk_speed_rows(rows: list[dict[str, float | bool | str]]) -> list[dict[str, float]]:
    valid_rows: list[dict[str, float]] = []
    for row in rows:
        if not row["valid_speed"]:
            continue
        valid_rows.append(
            {
                "time_s": float(row["time_s"]),
                "vz_mps": float(row["vz_mps"]),
            }
        )
    return valid_rows


def filter_rows_from_time(
    rows: list[dict[str, float]],
    start_time_s: float,
) -> list[dict[str, float]]:
    return [row for row in rows if row["time_s"] + TIME_EPSILON_S >= start_time_s]


def read_envelope_rows(path: Path) -> list[dict[str, float]]:
    if not path.exists():
        return []

    rows: list[dict[str, float]] = []
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        for raw in reader:
            try:
                if raw.get("factor_used", "1") == "0":
                    continue
                rows.append(
                    {
                        "time_s": plot_speed_vs_rtk.safe_float(raw["corrected_time_s"]),
                        "rtk_up_m": plot_speed_vs_rtk.safe_float(raw["rtk_up_m"]),
                        "half_width_m": plot_speed_vs_rtk.safe_float(raw["half_width_m"]),
                    }
                )
            except (KeyError, ValueError):
                continue
    rows.sort(key=lambda row: row["time_s"])
    return rows


def read_key_value_file(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    if not path.exists():
        return result
    for line in path.read_text(encoding="utf-8").splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        result[key.strip()] = value.strip()
    return result


def resolve_dynamic_start_time(trajectory_path: Path) -> float | None:
    summary_values = read_key_value_file(trajectory_path.with_name("summary.txt"))
    dynamic_start_raw = summary_values.get("dynamic_start_time_s", "")
    if dynamic_start_raw:
        return float(dynamic_start_raw)
    return None


def compute_delta_up(
    rows: list[dict[str, float]],
    key: str = "up_m",
    reference_time_s: float | None = None,
) -> list[float]:
    if not rows:
        return []

    if reference_time_s is None:
        up0 = rows[0][key]
    else:
        reference_index = find_nearest_index(
            [row["time_s"] for row in rows],
            reference_time_s,
        )
        up0 = rows[reference_index][key]
    return [row[key] - up0 for row in rows]


def compute_vertical_stats(rows: list[dict[str, float]], include_velocity: bool = True) -> dict[str, float]:
    delta_up = compute_delta_up(rows)
    stats = {
        "delta_up_range_m": max(delta_up) - min(delta_up),
        "delta_up_std_m": statistics.pstdev(delta_up),
    }
    if include_velocity:
        vz_values = [row["vz_mps"] for row in rows]
        stats["vz_range_mps"] = max(vz_values) - min(vz_values)
        stats["vz_std_mps"] = statistics.pstdev(vz_values)
    return stats


def plot_envelope_gate(
    axis,
    envelope_rows: list[dict[str, float]],
    reference_time_s: float,
) -> None:
    envelope_rows_from_start = filter_rows_from_time(envelope_rows, reference_time_s)
    if not envelope_rows_from_start:
        return

    reference_index = find_nearest_index(
        [row["time_s"] for row in envelope_rows_from_start],
        reference_time_s,
    )
    reference_up_m = envelope_rows_from_start[reference_index]["rtk_up_m"]
    time_s = [row["time_s"] - reference_time_s for row in envelope_rows_from_start]
    center_delta_up = [row["rtk_up_m"] - reference_up_m for row in envelope_rows_from_start]
    upper_delta_up = [
        center + row["half_width_m"]
        for center, row in zip(center_delta_up, envelope_rows_from_start)
    ]
    lower_delta_up = [
        center - row["half_width_m"]
        for center, row in zip(center_delta_up, envelope_rows_from_start)
    ]

    axis.plot(
        time_s,
        upper_delta_up,
        color="#9467bd",
        linewidth=0.8,
        linestyle="--",
        alpha=0.85,
        label="RTK envelope gate",
    )
    axis.plot(
        time_s,
        lower_delta_up,
        color="#9467bd",
        linewidth=0.8,
        linestyle="--",
        alpha=0.85,
    )
    axis.fill_between(
        time_s,
        lower_delta_up,
        upper_delta_up,
        color="#9467bd",
        alpha=0.08,
        linewidth=0.0,
    )


def write_csv(
    path: Path,
    trajectory_rows: list[dict[str, float]],
    rtk_rows: list[dict[str, float]],
    rtk_speed_rows: list[dict[str, float]],
) -> None:
    reference_time_s = trajectory_rows[0]["time_s"]
    rtk_times = [row["time_s"] for row in rtk_rows]
    rtk_speed_times = [row["time_s"] for row in rtk_speed_rows]
    optimized_delta_up = compute_delta_up(
        trajectory_rows,
        reference_time_s=reference_time_s,
    )
    rtk_delta_up = compute_delta_up(
        rtk_rows,
        reference_time_s=reference_time_s,
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(
            [
                "time_s",
                "optimized_up_m",
                "optimized_delta_up_m",
                "optimized_vz_mps",
                "rtk_time_s",
                "rtk_up_m",
                "rtk_delta_up_m",
                "rtk_speed_time_s",
                "rtk_vz_mps",
            ]
        )
        for row_index, row in enumerate(trajectory_rows):
            rtk_index = find_nearest_index(rtk_times, row["time_s"])
            rtk_speed_index = find_nearest_index(rtk_speed_times, row["time_s"])
            writer.writerow(
                [
                    row["time_s"],
                    row["up_m"],
                    optimized_delta_up[row_index],
                    row["vz_mps"],
                    rtk_rows[rtk_index]["time_s"],
                    rtk_rows[rtk_index]["up_m"],
                    rtk_delta_up[rtk_index],
                    rtk_speed_rows[rtk_speed_index]["time_s"],
                    rtk_speed_rows[rtk_speed_index]["vz_mps"],
                ]
            )


def make_plot(
    trajectory_rows: list[dict[str, float]],
    rtk_rows: list[dict[str, float]],
    rtk_speed_rows: list[dict[str, float]],
    envelope_rows: list[dict[str, float]],
    output_path: Path,
    title: str,
) -> None:
    t0 = trajectory_rows[0]["time_s"]
    rtk_rows_from_start = filter_rows_from_time(rtk_rows, t0)
    rtk_speed_rows_from_start = filter_rows_from_time(rtk_speed_rows, t0)

    def relative_time(rows: list[dict[str, float]]) -> list[float]:
        return [row["time_s"] - t0 for row in rows]

    fig, axes = plt.subplots(3, 1, figsize=(16, 12), sharex=True, constrained_layout=True)
    fig.suptitle(title, fontsize=16)

    axes[0].plot(relative_time(trajectory_rows), [row["up_m"] for row in trajectory_rows], color="#1f77b4", linewidth=1.1, label="optimized")
    axes[0].plot(relative_time(rtk_rows_from_start), [row["up_m"] for row in rtk_rows_from_start], color="#ff7f0e", linewidth=0.9, alpha=0.9, label="RTKFIX")
    axes[0].set_ylabel("Up [m]")
    axes[0].set_title("Absolute up position")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="upper right")

    axes[1].plot(relative_time(trajectory_rows), compute_delta_up(trajectory_rows, reference_time_s=t0), color="#1f77b4", linewidth=1.1, label="optimized")
    axes[1].plot(relative_time(rtk_rows_from_start), compute_delta_up(rtk_rows_from_start, reference_time_s=t0), color="#ff7f0e", linewidth=0.9, alpha=0.9, label="RTKFIX")
    plot_envelope_gate(axes[1], envelope_rows, t0)
    axes[1].set_ylabel("Delta Up [m]")
    axes[1].set_title("Relative up position with envelope gate")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="upper right")

    axes[2].plot(relative_time(trajectory_rows), [row["vz_mps"] for row in trajectory_rows], color="#1f77b4", linewidth=1.1, label="optimized")
    axes[2].plot(relative_time(rtk_speed_rows_from_start), [row["vz_mps"] for row in rtk_speed_rows_from_start], color="#ff7f0e", linewidth=0.9, alpha=0.9, label="RTKFIX diff")
    axes[2].set_ylabel("Vz [m/s]")
    axes[2].set_title("Vertical velocity")
    axes[2].set_xlabel("Time since navigation start [s]")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc="upper right")
    axes[2].set_xlim(left=0.0)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def main() -> int:
    args = parse_args()
    trajectory_path = Path(args.trajectory)
    gnss_path = Path(args.gnss)
    output_path = Path(args.output)
    csv_output_path = (
        Path(args.csv_output)
        if args.csv_output
        else output_path.with_name(output_path.stem + ".csv")
    )

    trajectory_rows_all = read_trajectory_rows(trajectory_path)
    envelope_path = (
        Path(args.envelope_diagnostics)
        if args.envelope_diagnostics
        else trajectory_path.with_name("vertical_envelope_diagnostics.csv")
    )
    dynamic_start_time_s = resolve_dynamic_start_time(trajectory_path)
    trajectory_rows = (
        filter_rows_from_time(trajectory_rows_all, dynamic_start_time_s)
        if dynamic_start_time_s is not None
        else trajectory_rows_all
    )
    navigation_start_time_s = trajectory_rows[0]["time_s"]
    origin_lat_rad, origin_lon_rad, origin_h_m = plot_speed_vs_rtk.choose_origin(gnss_path, trajectory_path)
    rtk_rows = plot_speed_vs_rtk.read_rtkfix_rows(gnss_path, origin_lat_rad, origin_lon_rad, origin_h_m)
    rtk_speed_rows = filter_valid_rtk_speed_rows(
        plot_speed_vs_rtk.build_rtk_speed_rows(rtk_rows, args.speed_window_s)
    )
    envelope_rows = read_envelope_rows(envelope_path)
    rtk_rows_from_start = filter_rows_from_time(rtk_rows, navigation_start_time_s)
    rtk_speed_rows_from_start = filter_rows_from_time(rtk_speed_rows, navigation_start_time_s)

    make_plot(trajectory_rows, rtk_rows, rtk_speed_rows, envelope_rows, output_path, args.title)
    write_csv(csv_output_path, trajectory_rows, rtk_rows, rtk_speed_rows)

    optimized_stats = compute_vertical_stats(trajectory_rows)
    rtk_stats = compute_vertical_stats(
        rtk_rows_from_start,
        include_velocity=False,
    )
    rtk_speed_stats = compute_vertical_stats(
        [{"time_s": row["time_s"], "up_m": 0.0, "vz_mps": row["vz_mps"]} for row in rtk_speed_rows_from_start]
    )
    print(f"plot_saved={output_path}")
    print(f"csv_saved={csv_output_path}")
    print(
        "optimized: "
        f"delta_up_range_m={optimized_stats['delta_up_range_m']:.6f}, "
        f"delta_up_std_m={optimized_stats['delta_up_std_m']:.6f}, "
        f"vz_range_mps={optimized_stats['vz_range_mps']:.6f}, "
        f"vz_std_mps={optimized_stats['vz_std_mps']:.6f}"
    )
    print(
        "rtk: "
        f"delta_up_range_m={rtk_stats['delta_up_range_m']:.6f}, "
        f"delta_up_std_m={rtk_stats['delta_up_std_m']:.6f}, "
        f"vz_range_mps={rtk_speed_stats['vz_range_mps']:.6f}, "
        f"vz_std_mps={rtk_speed_stats['vz_std_mps']:.6f}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
