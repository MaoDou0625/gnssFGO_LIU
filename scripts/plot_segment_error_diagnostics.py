#!/usr/bin/env python

from __future__ import annotations

import argparse
import csv
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


GROUPS = [
    ("dtheta", "Attitude Error [deg]", ["dtheta_x_rad", "dtheta_y_rad", "dtheta_z_rad"]),
    ("dv", "Velocity Error [m/s]", ["dv_x_mps", "dv_y_mps", "dv_z_mps"]),
    ("dp", "Position Error [m]", ["dp_x_m", "dp_y_m", "dp_z_m"]),
    ("dbg", "Gyro Bias Error [rad/s]", ["dbg_x_radps", "dbg_y_radps", "dbg_z_radps"]),
    ("dba", "Accel Bias Error [m/s^2]", ["dba_x_mps2", "dba_y_mps2", "dba_z_mps2"]),
]

COMPONENT_LABELS = {"x": "x", "y": "y", "z": "z"}
COLORS = {"x": "#1f77b4", "y": "#ff7f0e", "z": "#2ca02c"}
TIME_EPSILON_S = 1e-9
DTHETA_COLUMNS = {"dtheta_x_rad", "dtheta_y_rad", "dtheta_z_rad"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot segment-local estimated error diagnostics and print summary stats."
    )
    parser.add_argument("--segment-error", required=True, help="Path to segment_error_diagnostics.csv")
    parser.add_argument(
        "--trajectory",
        default="",
        help="Optional path to trajectory.csv. Defaults to the sibling file next to segment_error_diagnostics.csv.",
    )
    parser.add_argument("--output", required=True, help="Output image path")
    parser.add_argument("--csv-output", default="", help="Optional summary CSV output path")
    parser.add_argument("--title", default="Segment-local estimated error diagnostics", help="Figure title")
    return parser.parse_args()


def read_rows(path: Path) -> list[dict[str, float]]:
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        rows = []
        for raw in reader:
            row = {
                "segment_index": float(raw["segment_index"]),
                "start_time_s": float(raw["start_time_s"]),
                "end_time_s": float(raw["end_time_s"]),
                "mid_time_s": 0.5 * (float(raw["start_time_s"]) + float(raw["end_time_s"])),
                "gnss_factor_count": float(raw["gnss_factor_count"]),
                "mean_prefit_nis": float(raw["mean_prefit_nis"]),
                "mean_postfit_nis": float(raw["mean_postfit_nis"]),
                "mean_covariance_scale": float(raw["mean_covariance_scale"]),
            }
            for _, _, columns in GROUPS:
                for column in columns:
                    row[column] = float(raw[column])
            rows.append(row)
    if not rows:
        raise ValueError(f"No rows found in {path}")
    rows.sort(key=lambda row: row["mid_time_s"])
    return rows


def resolve_trajectory_path(segment_error_path: Path, trajectory_argument: str) -> Path:
    if trajectory_argument:
        return Path(trajectory_argument)
    return segment_error_path.with_name("trajectory.csv")


def read_dynamic_start_time(path: Path) -> float | None:
    if not path.exists():
        return None
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        for raw in reader:
            return float(raw["time_s"])
    return None


def filter_dynamic_rows(
    rows: list[dict[str, float]],
    dynamic_start_time_s: float | None,
) -> list[dict[str, float]]:
    if dynamic_start_time_s is None:
        return rows
    filtered = [row for row in rows if row["start_time_s"] + TIME_EPSILON_S >= dynamic_start_time_s]
    if filtered:
        return filtered
    return rows


def convert_rows_to_display_units(rows: list[dict[str, float]]) -> list[dict[str, float]]:
    converted_rows: list[dict[str, float]] = []
    for row in rows:
        converted = dict(row)
        for column in DTHETA_COLUMNS:
            converted[column] = math.degrees(row[column])
        converted_rows.append(converted)
    return converted_rows


def total_variation(values: list[float]) -> float:
    return sum(abs(values[index + 1] - values[index]) for index in range(len(values) - 1))


def summarize(rows: list[dict[str, float]]) -> list[dict[str, float | str]]:
    stats: list[dict[str, float | str]] = []
    for group_name, _, columns in GROUPS:
        for column in columns:
            values = [row[column] for row in rows]
            component = column.split("_")[1]
            stats.append(
                {
                    "group": group_name,
                    "component": component,
                    "min": min(values),
                    "max": max(values),
                    "range": max(values) - min(values),
                    "mean": statistics.mean(values),
                    "std": statistics.pstdev(values),
                    "end_minus_start": values[-1] - values[0],
                    "total_variation": total_variation(values),
                }
            )
    return stats


def write_summary_csv(path: Path, stats: list[dict[str, float | str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=[
                "group",
                "component",
                "min",
                "max",
                "range",
                "mean",
                "std",
                "end_minus_start",
                "total_variation",
            ],
        )
        writer.writeheader()
        for row in stats:
            writer.writerow(row)


def make_plot(
    rows: list[dict[str, float]],
    output_path: Path,
    title: str,
    dynamic_start_time_s: float | None,
) -> None:
    if dynamic_start_time_s is None:
        time0 = rows[0]["mid_time_s"]
        xlabel = "Time since first segment [s]"
    else:
        time0 = dynamic_start_time_s
        xlabel = "Time since dynamic start [s]"
    times = [row["mid_time_s"] - time0 for row in rows]

    fig, axes = plt.subplots(len(GROUPS) + 1, 1, figsize=(16, 15), sharex=True, constrained_layout=True)
    fig.suptitle(title, fontsize=16)

    for axis, (group_name, ylabel, columns) in zip(axes[:-1], GROUPS):
        for column in columns:
            component = column.split("_")[1]
            axis.plot(
                times,
                [row[column] for row in rows],
                label=COMPONENT_LABELS[component],
                color=COLORS[component],
                linewidth=1.1,
            )
        axis.set_ylabel(ylabel)
        axis.set_title(group_name)
        axis.grid(True, alpha=0.3)
        axis.legend(loc="upper right")

    nis_axis = axes[-1]
    nis_axis.plot(times, [row["mean_prefit_nis"] for row in rows], color="#d62728", linewidth=1.0, label="prefit NIS")
    nis_axis.plot(times, [row["mean_postfit_nis"] for row in rows], color="#9467bd", linewidth=1.0, label="postfit NIS")
    nis_axis.plot(
        times,
        [row["mean_covariance_scale"] for row in rows],
        color="#8c564b",
        linewidth=1.0,
        label="covariance scale",
    )
    nis_axis.set_ylabel("Consistency")
    nis_axis.set_title("GNSS consistency by segment")
    nis_axis.set_xlabel(xlabel)
    nis_axis.grid(True, alpha=0.3)
    nis_axis.legend(loc="upper right")
    if dynamic_start_time_s is not None:
        nis_axis.set_xlim(left=0.0)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def print_stats(stats: list[dict[str, float | str]]) -> None:
    for row in stats:
        print(
            f"{row['group']}_{row['component']}: "
            f"range={float(row['range']):.6e}, "
            f"std={float(row['std']):.6e}, "
            f"end_minus_start={float(row['end_minus_start']):.6e}, "
            f"total_variation={float(row['total_variation']):.6e}"
        )


def main() -> int:
    args = parse_args()
    segment_error_path = Path(args.segment_error)
    output_path = Path(args.output)
    csv_output_path = (
        Path(args.csv_output)
        if args.csv_output
        else output_path.with_name(output_path.stem + "_summary.csv")
    )

    rows = read_rows(segment_error_path)
    trajectory_path = resolve_trajectory_path(segment_error_path, args.trajectory)
    dynamic_start_time_s = read_dynamic_start_time(trajectory_path)
    rows = filter_dynamic_rows(rows, dynamic_start_time_s)
    display_rows = convert_rows_to_display_units(rows)
    stats = summarize(display_rows)
    make_plot(display_rows, output_path, args.title, dynamic_start_time_s)
    write_summary_csv(csv_output_path, stats)

    print(f"plot_saved={output_path}")
    print(f"csv_saved={csv_output_path}")
    if dynamic_start_time_s is not None:
        print(f"dynamic_start_time_s={dynamic_start_time_s}")
    print_stats(stats)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
