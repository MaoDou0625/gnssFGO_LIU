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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot Stage 2 body/vehicle NHC velocity diagnostics."
    )
    parser.add_argument(
        "--diagnostics",
        required=True,
        help="Path to stage2_vehicle_nhc_state_diagnostics.csv",
    )
    parser.add_argument(
        "--output",
        required=True,
        help="Output image path, for example stage2_vehicle_nhc_velocity.png",
    )
    parser.add_argument(
        "--title",
        default="Stage 2 body and vehicle NHC velocities",
        help="Figure title",
    )
    return parser.parse_args()


def safe_float(value: str) -> float:
    try:
        parsed = float(value)
    except ValueError:
        return math.nan
    return parsed if math.isfinite(parsed) else math.nan


def read_rows(path: Path) -> list[dict[str, float | str]]:
    rows: list[dict[str, float | str]] = []
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        for raw in reader:
            row: dict[str, float | str] = {
                "region": raw.get("nhc_region_type", "UNSET"),
            }
            for key in [
                "time_s",
                "v_body_x_mps",
                "v_body_y_mps",
                "v_body_z_mps",
                "v_vehicle_y_mps",
                "v_vehicle_z_mps",
                "vehicle_y_correction_mps",
                "vehicle_z_correction_from_x_mps",
                "vehicle_z_correction_from_y_mps",
            ]:
                row[key] = safe_float(raw.get(key, "nan"))
            if math.isfinite(row["time_s"]):  # type: ignore[arg-type]
                rows.append(row)
    rows.sort(key=lambda row: row["time_s"])  # type: ignore[index, return-value]
    return rows


def finite_series(rows: list[dict[str, float | str]], key: str) -> tuple[list[float], list[float]]:
    if not rows:
        return [], []
    t0 = rows[0]["time_s"]
    times: list[float] = []
    values: list[float] = []
    for row in rows:
        time_s = row["time_s"]
        value = row[key]
        if isinstance(time_s, float) and isinstance(value, float) and math.isfinite(value):
            times.append(time_s - t0)  # type: ignore[operator]
            values.append(value)
    return times, values


def main() -> None:
    args = parse_args()
    rows = read_rows(Path(args.diagnostics))
    if not rows:
        raise SystemExit("no finite Stage 2 vehicle NHC diagnostics found")

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    fig.suptitle(args.title)

    for key, label in [
        ("v_body_y_mps", "body_y"),
        ("v_vehicle_y_mps", "vehicle_y"),
    ]:
        times, values = finite_series(rows, key)
        axes[0].plot(times, values, linewidth=1.0, label=label)
    axes[0].axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
    axes[0].set_ylabel("lateral velocity (m/s)")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="best")

    for key, label in [
        ("v_body_z_mps", "body_z"),
        ("v_vehicle_z_mps", "vehicle_z"),
    ]:
        times, values = finite_series(rows, key)
        axes[1].plot(times, values, linewidth=1.0, label=label)
    axes[1].axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
    axes[1].set_ylabel("vertical velocity (m/s)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="best")

    for key, label in [
        ("vehicle_y_correction_mps", "y correction"),
        ("vehicle_z_correction_from_x_mps", "z correction from x"),
        ("vehicle_z_correction_from_y_mps", "z correction from y"),
    ]:
        times, values = finite_series(rows, key)
        axes[2].plot(times, values, linewidth=1.0, label=label)
    axes[2].axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
    axes[2].set_ylabel("correction (m/s)")
    axes[2].set_xlabel("relative time (s)")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc="best")

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(output, dpi=160)


if __name__ == "__main__":
    main()
