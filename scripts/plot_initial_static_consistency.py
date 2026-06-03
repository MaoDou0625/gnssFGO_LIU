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
        "matplotlib is required to generate plots. Use a Python environment with matplotlib installed."
    ) from exc


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot optimized initial-static-subgraph consistency against the ideal static reference."
    )
    parser.add_argument("--static-trajectory", required=True, help="Path to initial_static_trajectory.csv")
    parser.add_argument("--summary", required=True, help="Path to summary.txt")
    parser.add_argument("--output", required=True, help="Output image path")
    parser.add_argument("--csv-output", default="", help="Optional CSV output path")
    parser.add_argument(
        "--title",
        default="Initial static consistency: optimized static subgraph",
        help="Figure title",
    )
    return parser.parse_args()


def read_static_rows(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        for raw in reader:
            rows.append(
                {
                    "time_s": float(raw["time_s"]),
                    "east_m": float(raw["east_m"]),
                    "north_m": float(raw["north_m"]),
                    "up_m": float(raw["up_m"]),
                    "vx_mps": float(raw["vx_mps"]),
                    "vy_mps": float(raw["vy_mps"]),
                    "vz_mps": float(raw["vz_mps"]),
                }
            )
    if not rows:
        raise ValueError(f"No rows found in {path}")
    rows.sort(key=lambda row: row["time_s"])
    return rows


def compute_metrics(rows: list[dict[str, float]]) -> dict[str, float]:
    reference = rows[0]
    velocity_norms = []
    horizontal_drifts = []
    up_drifts = []
    drifts_3d = []
    for row in rows:
        de = row["east_m"] - reference["east_m"]
        dn = row["north_m"] - reference["north_m"]
        du = row["up_m"] - reference["up_m"]
        horizontal_drifts.append(math.hypot(de, dn))
        up_drifts.append(abs(du))
        drifts_3d.append(math.sqrt(de * de + dn * dn + du * du))
        velocity_norms.append(math.sqrt(row["vx_mps"] ** 2 + row["vy_mps"] ** 2 + row["vz_mps"] ** 2))

    return {
        "velocity_norm_mean_mps": statistics.fmean(velocity_norms),
        "velocity_norm_std_mps": statistics.pstdev(velocity_norms),
        "velocity_norm_max_mps": max(velocity_norms),
        "horizontal_drift_max_m": max(horizontal_drifts),
        "up_drift_max_m": max(up_drifts),
        "drift_3d_max_m": max(drifts_3d),
    }


def write_csv(path: Path, optimized_rows: list[dict[str, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    east0 = optimized_rows[0]["east_m"]
    north0 = optimized_rows[0]["north_m"]
    up0 = optimized_rows[0]["up_m"]
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(
            [
                "time_s",
                "optimized_east_m",
                "optimized_north_m",
                "optimized_up_m",
                "optimized_vx_mps",
                "optimized_vy_mps",
                "optimized_vz_mps",
                "optimized_horizontal_drift_m",
                "optimized_up_drift_m",
                "optimized_velocity_norm_mps",
            ]
        )
        for optimized_row in optimized_rows:
            horizontal_drift = math.hypot(optimized_row["east_m"] - east0, optimized_row["north_m"] - north0)
            up_drift = optimized_row["up_m"] - up0
            velocity_norm = math.sqrt(
                optimized_row["vx_mps"] ** 2 + optimized_row["vy_mps"] ** 2 + optimized_row["vz_mps"] ** 2
            )
            writer.writerow(
                [
                    optimized_row["time_s"],
                    optimized_row["east_m"],
                    optimized_row["north_m"],
                    optimized_row["up_m"],
                    optimized_row["vx_mps"],
                    optimized_row["vy_mps"],
                    optimized_row["vz_mps"],
                    horizontal_drift,
                    up_drift,
                    velocity_norm,
                ]
            )


def make_plot(optimized_rows: list[dict[str, float]], output_path: Path, title: str) -> None:
    start_time_s = optimized_rows[0]["time_s"]
    output_path.parent.mkdir(parents=True, exist_ok=True)

    rel_time = [row["time_s"] - start_time_s for row in optimized_rows]
    east0 = optimized_rows[0]["east_m"]
    north0 = optimized_rows[0]["north_m"]
    up0 = optimized_rows[0]["up_m"]
    horizontal_drift = [math.hypot(row["east_m"] - east0, row["north_m"] - north0) for row in optimized_rows]
    up_drift = [row["up_m"] - up0 for row in optimized_rows]
    velocity_norm = [
        math.sqrt(row["vx_mps"] ** 2 + row["vy_mps"] ** 2 + row["vz_mps"] ** 2)
        for row in optimized_rows
    ]

    fig, axes = plt.subplots(3, 1, figsize=(15, 11), sharex=True, constrained_layout=True)
    fig.suptitle(title, fontsize=16)

    axes[0].axhline(0.0, color="#999999", linestyle="--", linewidth=1.0, label="ideal static")
    axes[0].plot(rel_time, velocity_norm, color="#1f77b4", linewidth=1.1, label="optimized static subgraph")
    axes[0].set_ylabel("|v| [m/s]")
    axes[0].set_title("Velocity norm during static 100 s")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="upper right")

    axes[1].axhline(0.0, color="#999999", linestyle="--", linewidth=1.0, label="ideal static")
    axes[1].plot(rel_time, horizontal_drift, color="#1f77b4", linewidth=1.1, label="optimized static subgraph")
    axes[1].set_ylabel("Horizontal drift [m]")
    axes[1].set_title("Horizontal drift from static start")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="upper left")

    axes[2].axhline(0.0, color="#999999", linestyle="--", linewidth=1.0, label="ideal static")
    axes[2].plot(rel_time, up_drift, color="#1f77b4", linewidth=1.1, label="optimized static subgraph")
    axes[2].set_ylabel("Up drift [m]")
    axes[2].set_xlabel("Time since static alignment start [s]")
    axes[2].set_title("Vertical drift from static start")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc="upper left")

    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def print_metrics(prefix: str, metrics: dict[str, float]) -> None:
    for key, value in metrics.items():
        print(f"{prefix}_{key}={value:.9f}")


def main() -> int:
    args = parse_args()
    static_trajectory_path = Path(args.static_trajectory)
    summary_path = Path(args.summary)
    output_path = Path(args.output)
    csv_output_path = Path(args.csv_output) if args.csv_output else output_path.with_suffix(".csv")

    _ = summary_path.read_text(encoding="utf-8")
    optimized_rows = read_static_rows(static_trajectory_path)
    optimized_metrics = compute_metrics(optimized_rows)

    make_plot(optimized_rows, output_path, args.title)
    write_csv(csv_output_path, optimized_rows)

    print(f"plot_saved={output_path}")
    print(f"csv_saved={csv_output_path}")
    print_metrics("optimized", optimized_metrics)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
