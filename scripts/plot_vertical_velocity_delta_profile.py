#!/usr/bin/env python

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


def safe_float(value: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float("nan")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot vertical velocity delta diagnostics.")
    parser.add_argument("--trajectory", required=True, help="Path to trajectory.csv")
    parser.add_argument("--diagnostics", required=True, help="Path to vertical_velocity_delta_diagnostics.csv")
    parser.add_argument("--jump-windows", default="", help="Optional body_z_seed_jump_windows.csv")
    parser.add_argument("--output", required=True, help="Output image path")
    parser.add_argument("--title", default="Vertical velocity delta diagnostics", help="Figure title")
    return parser.parse_args()


def read_trajectory(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        for raw in reader:
            rows.append({"time_s": safe_float(raw["time_s"]), "vz_mps": safe_float(raw["vz_mps"])})
    rows.sort(key=lambda row: row["time_s"])
    return rows


def read_diagnostics(path: Path) -> list[dict[str, float | bool | str]]:
    rows: list[dict[str, float | bool | str]] = []
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        for raw in reader:
            rows.append(
                {
                    "start_time_s": safe_float(raw["start_time_s"]),
                    "end_time_s": safe_float(raw["end_time_s"]),
                    "factor_added": raw["factor_added"] == "1",
                    "in_jump_padding": raw["in_jump_padding"] == "1",
                    "skip_reason": raw["skip_reason"],
                    "target_delta_vz_mps": safe_float(raw["target_delta_vz_mps"]),
                    "optimized_delta_vz_mps": safe_float(raw["optimized_delta_vz_mps"]),
                    "residual_mps": safe_float(raw["residual_mps"]),
                    "sigma_mps": safe_float(raw["sigma_mps"]),
                }
            )
    return rows


def read_jump_windows(path: Path) -> list[tuple[float, float]]:
    if not path.exists():
        return []
    windows: list[tuple[float, float]] = []
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        for raw in reader:
            windows.append((safe_float(raw["start_time_s"]), safe_float(raw["end_time_s"])))
    return windows


def compute_dvz_dt(rows: list[dict[str, float]]) -> tuple[list[float], list[float]]:
    times: list[float] = []
    values: list[float] = []
    for left, right in zip(rows, rows[1:]):
        dt_s = right["time_s"] - left["time_s"]
        if dt_s <= 0.0:
            continue
        times.append(0.5 * (left["time_s"] + right["time_s"]))
        values.append((right["vz_mps"] - left["vz_mps"]) / dt_s)
    return times, values


def main() -> None:
    args = parse_args()
    trajectory = read_trajectory(Path(args.trajectory))
    diagnostics = read_diagnostics(Path(args.diagnostics))
    jump_windows = read_jump_windows(Path(args.jump_windows)) if args.jump_windows else []

    time_s = [row["time_s"] for row in trajectory]
    vz_mps = [row["vz_mps"] for row in trajectory]
    dvz_time_s, dvz_dt_mps2 = compute_dvz_dt(trajectory)

    added_times = [
        0.5 * (float(row["start_time_s"]) + float(row["end_time_s"]))
        for row in diagnostics
        if bool(row["factor_added"])
    ]
    added_residuals = [float(row["residual_mps"]) for row in diagnostics if bool(row["factor_added"])]
    skipped_jump_times = [
        0.5 * (float(row["start_time_s"]) + float(row["end_time_s"]))
        for row in diagnostics
        if bool(row["in_jump_padding"])
    ]
    skipped_jump_intervals = [
        (float(row["start_time_s"]), float(row["end_time_s"]))
        for row in diagnostics
        if bool(row["in_jump_padding"])
    ]

    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    fig.suptitle(args.title)

    for axis in axes:
        for start_time_s, end_time_s in jump_windows:
            axis.axvspan(start_time_s, end_time_s, color="tab:red", alpha=0.12)

    axes[0].plot(time_s, vz_mps, linewidth=1.0, color="tab:blue")
    axes[0].set_ylabel("v_z (m/s)")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(dvz_time_s, dvz_dt_mps2, linewidth=0.9, color="tab:orange")
    axes[1].scatter(skipped_jump_times, [0.0] * len(skipped_jump_times), s=8, color="tab:red", alpha=0.45, label="jump skipped")
    axes[1].set_ylabel("dv_z/dt (m/s^2)")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.3)

    axes[2].scatter(added_times, added_residuals, s=8, color="tab:green", alpha=0.65, label="added factor residual")
    axes[2].axhline(0.0, color="black", linewidth=0.8)
    axes[2].set_ylabel("residual (m/s)")
    axes[2].set_xlabel("time_s")
    axes[2].legend(loc="upper right")
    axes[2].grid(True, alpha=0.3)

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(output, dpi=160)

    max_abs_dvz_dt = max((abs(value) for value in dvz_dt_mps2), default=float("nan"))
    max_abs_non_jump_dvz_dt = max(
        (
            abs(value)
            for time_s, value in zip(dvz_time_s, dvz_dt_mps2)
            if not any(start_time_s <= time_s <= end_time_s for start_time_s, end_time_s in skipped_jump_intervals)
        ),
        default=float("nan"),
    )
    max_abs_added_residual = max((abs(value) for value in added_residuals), default=float("nan"))
    print(f"plot_saved={output}")
    print(f"max_abs_dvz_dt_mps2={max_abs_dvz_dt:.6f}")
    print(f"max_abs_non_jump_dvz_dt_mps2={max_abs_non_jump_dvz_dt:.6f}")
    print(f"added_factor_count={len(added_times)}")
    print(f"jump_skipped_interval_count={len(skipped_jump_times)}")
    print(f"max_abs_added_residual_mps={max_abs_added_residual:.6f}")


if __name__ == "__main__":
    main()
