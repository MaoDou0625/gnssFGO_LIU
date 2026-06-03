#!/usr/bin/env python

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

try:
    import matplotlib.pyplot as plt
except ImportError as exc:  # pragma: no cover - runtime dependency guard
    raise SystemExit(
        "matplotlib is required to generate plots. "
        "Use a Python environment with matplotlib installed."
    ) from exc


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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot navigation yaw/pitch and gyro z bias versus time."
    )
    parser.add_argument("--trajectory", required=True, help="Path to trajectory.csv")
    parser.add_argument("--output", required=True, help="Output image path")
    parser.add_argument(
        "--title",
        default="Navigation attitude and gyro z bias vs time",
        help="Figure title",
    )
    return parser.parse_args()


def read_attitude_rows(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        for raw in reader:
            rows.append(
                {
                    "time_s": float(raw["time_s"]),
                    "yaw_deg": math.degrees(float(raw["yaw_rad"])),
                    "pitch_deg": math.degrees(float(raw["pitch_rad"])),
                    "roll_deg": math.degrees(float(raw["roll_rad"])),
                    "bgz_radps": float(raw["bgz"]),
                }
            )
    if not rows:
        raise ValueError(f"No trajectory rows found in {path}")
    yaw_unwrapped_deg = unwrap_angles_deg([row["yaw_deg"] for row in rows])
    for row, yaw_value_deg in zip(rows, yaw_unwrapped_deg):
        row["yaw_unwrapped_deg"] = yaw_value_deg
    return rows


def make_plot(rows: list[dict[str, float]], output_path: Path, title: str) -> None:
    time_s = [row["time_s"] for row in rows]
    yaw_deg = [row["yaw_unwrapped_deg"] for row in rows]
    pitch_deg = [row["pitch_deg"] for row in rows]
    bgz_radps = [row["bgz_radps"] for row in rows]

    fig, axes = plt.subplots(3, 1, figsize=(16, 10), sharex=True, constrained_layout=True)
    fig.suptitle(title, fontsize=16)

    axes[0].plot(time_s, yaw_deg, color="#1f77b4", linewidth=1.2)
    axes[0].set_title("Yaw")
    axes[0].set_ylabel("Yaw [deg]")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(time_s, pitch_deg, color="#ff7f0e", linewidth=1.2)
    axes[1].set_title("Pitch")
    axes[1].set_ylabel("Pitch [deg]")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(time_s, bgz_radps, color="#2ca02c", linewidth=1.2)
    axes[2].set_title("Gyro Z bias")
    axes[2].set_xlabel("Time [s]")
    axes[2].set_ylabel("bgz [rad/s]")
    axes[2].grid(True, alpha=0.3)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def main() -> int:
    args = parse_args()
    trajectory_path = Path(args.trajectory)
    output_path = Path(args.output)
    rows = read_attitude_rows(trajectory_path)
    make_plot(rows, output_path, args.title)
    print(f"plot_saved={output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
