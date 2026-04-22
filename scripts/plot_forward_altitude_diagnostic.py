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
except ImportError as exc:  # pragma: no cover - runtime dependency guard
    raise SystemExit(
        "matplotlib is required to generate plots. "
        "Use a Python environment with matplotlib installed."
    ) from exc


HELPER_PATH = Path(__file__).resolve().with_name("plot_forward_heading_diagnostic.py")
HELPER_SPEC = importlib.util.spec_from_file_location("plot_forward_heading_diagnostic", HELPER_PATH)
heading_diagnostic = importlib.util.module_from_spec(HELPER_SPEC)
HELPER_SPEC.loader.exec_module(heading_diagnostic)

TIME_EPSILON_S = 1e-9


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Diagnose early altitude behavior by comparing optimized navigation altitude "
            "against pure inertial forward propagation."
        )
    )
    parser.add_argument("--trajectory", required=True, help="Path to trajectory.csv")
    parser.add_argument("--imu", required=True, help="Path to imu_gnss_fgo.txt")
    parser.add_argument(
        "--gnss",
        default="",
        help="Optional path to gnss_solution_gnss_fgo.txt for overlaying RTKFIX altitude",
    )
    parser.add_argument("--summary", required=True, help="Path to summary.txt")
    parser.add_argument("--config", required=True, help="Path to config_snapshot.cfg")
    parser.add_argument("--output", required=True, help="Output image path")
    parser.add_argument(
        "--csv-output",
        default="",
        help="Optional CSV output path for the forward-altitude diagnostic series",
    )
    parser.add_argument(
        "--duration-s",
        type=float,
        default=30.0,
        help="Duration after navigation start to include in the diagnostic",
    )
    parser.add_argument(
        "--focus-window-s",
        type=float,
        default=10.0,
        help="Early window used for smoothness statistics",
    )
    parser.add_argument(
        "--title",
        default="Forward altitude diagnostic vs optimized navigation",
        help="Figure title",
    )
    return parser.parse_args()


def read_trajectory_rows(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        for raw in reader:
            rows.append(
                {
                    "time_s": heading_diagnostic.safe_float(raw["time_s"]),
                    "east_m": heading_diagnostic.safe_float(raw["east_m"]),
                    "north_m": heading_diagnostic.safe_float(raw["north_m"]),
                    "up_m": heading_diagnostic.safe_float(raw["up_m"]),
                    "vx_mps": heading_diagnostic.safe_float(raw["vx_mps"]),
                    "vy_mps": heading_diagnostic.safe_float(raw["vy_mps"]),
                    "vz_mps": heading_diagnostic.safe_float(raw["vz_mps"]),
                    "bax": heading_diagnostic.safe_float(raw["bax"]),
                    "bay": heading_diagnostic.safe_float(raw["bay"]),
                    "baz": heading_diagnostic.safe_float(raw["baz"]),
                    "bgx": heading_diagnostic.safe_float(raw["bgx"]),
                    "bgy": heading_diagnostic.safe_float(raw["bgy"]),
                    "bgz": heading_diagnostic.safe_float(raw["bgz"]),
                }
            )
    if not rows:
        raise ValueError(f"No trajectory rows found in {path}")
    rows.sort(key=lambda row: row["time_s"])
    return rows


def matvec(matrix: list[list[float]], vector: list[float]) -> list[float]:
    return [sum(matrix[row][col] * vector[col] for col in range(3)) for row in range(3)]


def propagate_forward_altitude_rows(
    imu_rows: list[dict[str, float]],
    start_time_s: float,
    end_time_s: float,
    initial_world_from_body: list[list[float]],
    initial_acc_bias: list[float],
    initial_gyro_bias: list[float],
    initial_position_enu_m: list[float],
    initial_velocity_enu_mps: list[float],
    gravity_mps2: float,
) -> list[dict[str, float]]:
    if end_time_s <= start_time_s:
        raise ValueError("end_time_s must be greater than start_time_s")

    current_world_from_body = [[value for value in row] for row in initial_world_from_body]
    current_position = [value for value in initial_position_enu_m]
    current_velocity = [value for value in initial_velocity_enu_mps]
    gravity_enu = [0.0, 0.0, gravity_mps2]

    rows = [
        {
            "time_s": start_time_s,
            "east_m": current_position[0],
            "north_m": current_position[1],
            "up_m": current_position[2],
            "vx_mps": current_velocity[0],
            "vy_mps": current_velocity[1],
            "vz_mps": current_velocity[2],
        }
    ]

    for imu_index in range(len(imu_rows) - 1):
        current_row = imu_rows[imu_index]
        next_row = imu_rows[imu_index + 1]
        interval_start_s = max(current_row["time_s"], start_time_s)
        interval_end_s = min(next_row["time_s"], end_time_s)
        if interval_end_s <= interval_start_s + TIME_EPSILON_S:
            continue

        delta_time_s = interval_end_s - interval_start_s
        corrected_gyro = [
            current_row["gyro_x"] - initial_gyro_bias[0],
            current_row["gyro_y"] - initial_gyro_bias[1],
            current_row["gyro_z"] - initial_gyro_bias[2],
        ]
        corrected_acc = [
            current_row["acc_x"] - initial_acc_bias[0],
            current_row["acc_y"] - initial_acc_bias[1],
            current_row["acc_z"] - initial_acc_bias[2],
        ]

        nav_specific_force = matvec(current_world_from_body, corrected_acc)
        nav_acc = [nav_specific_force[axis] - gravity_enu[axis] for axis in range(3)]
        current_position = [
            current_position[axis] + current_velocity[axis] * delta_time_s + 0.5 * nav_acc[axis] * delta_time_s * delta_time_s
            for axis in range(3)
        ]
        current_velocity = [
            current_velocity[axis] + nav_acc[axis] * delta_time_s
            for axis in range(3)
        ]

        delta_rotation = heading_diagnostic.exp_so3([value * delta_time_s for value in corrected_gyro])
        current_world_from_body = heading_diagnostic.matmul(current_world_from_body, delta_rotation)

        rows.append(
            {
                "time_s": interval_end_s,
                "east_m": current_position[0],
                "north_m": current_position[1],
                "up_m": current_position[2],
                "vx_mps": current_velocity[0],
                "vy_mps": current_velocity[1],
                "vz_mps": current_velocity[2],
            }
        )
        if interval_end_s >= end_time_s - TIME_EPSILON_S:
            break

    return rows


def compute_window_stats(rows: list[dict[str, float]], start_time_s: float, duration_s: float) -> dict[str, float]:
    if duration_s <= 0.0:
        raise ValueError("duration_s must be positive")
    end_time_s = start_time_s + duration_s
    selected_rows = [row for row in rows if start_time_s - TIME_EPSILON_S <= row["time_s"] <= end_time_s + TIME_EPSILON_S]
    if len(selected_rows) < 2:
        raise ValueError("not enough rows in the requested stats window")

    up0 = selected_rows[0]["up_m"]
    delta_up = [row["up_m"] - up0 for row in selected_rows]
    vz_values = [row["vz_mps"] for row in selected_rows]
    return {
        "delta_up_range_m": max(delta_up) - min(delta_up),
        "delta_up_std_m": statistics.pstdev(delta_up),
        "delta_up_end_m": delta_up[-1],
        "vz_range_mps": max(vz_values) - min(vz_values),
        "vz_std_mps": statistics.pstdev(vz_values),
        "sample_count": float(len(selected_rows)),
    }


def filter_rows_by_time_window(
    rows: list[dict[str, float]],
    start_time_s: float,
    end_time_s: float,
) -> list[dict[str, float]]:
    return [
        row
        for row in rows
        if start_time_s - TIME_EPSILON_S <= row["time_s"] <= end_time_s + TIME_EPSILON_S
    ]


def write_csv(
    path: Path,
    forward_rows: list[dict[str, float]],
    feedback_rows: list[dict[str, float]],
    optimized_rows: list[dict[str, float]],
    rtk_rows: list[dict[str, float]] | None = None,
) -> None:
    optimized_times = [row["time_s"] for row in optimized_rows]
    feedback_times = [row["time_s"] for row in feedback_rows]
    rtk_times = [row["time_s"] for row in rtk_rows] if rtk_rows else []
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(
            [
                "time_s",
                "forward_up_m",
                "forward_vz_mps",
                "feedback_up_m",
                "feedback_vz_mps",
                "optimized_time_s",
                "optimized_up_m",
                "optimized_vz_mps",
                "rtk_time_s",
                "rtk_up_m",
            ]
        )
        for row in forward_rows:
            feedback_index = heading_diagnostic.find_nearest_index(feedback_times, row["time_s"])
            feedback_row = feedback_rows[feedback_index]
            optimized_index = heading_diagnostic.find_nearest_index(optimized_times, row["time_s"])
            optimized_row = optimized_rows[optimized_index]
            if rtk_rows:
                rtk_index = heading_diagnostic.find_nearest_index(rtk_times, row["time_s"])
                rtk_row = rtk_rows[rtk_index]
                rtk_time_s = rtk_row["time_s"]
                rtk_up_m = rtk_row["up_m"]
            else:
                rtk_time_s = ""
                rtk_up_m = ""
            writer.writerow(
                [
                    row["time_s"],
                    row["up_m"],
                    row["vz_mps"],
                    feedback_row["up_m"],
                    feedback_row["vz_mps"],
                    optimized_row["time_s"],
                    optimized_row["up_m"],
                    optimized_row["vz_mps"],
                    rtk_time_s,
                    rtk_up_m,
                ]
            )


def make_plot(
    forward_rows: list[dict[str, float]],
    feedback_rows: list[dict[str, float]],
    optimized_rows: list[dict[str, float]],
    rtk_rows: list[dict[str, float]] | None,
    output_path: Path,
    title: str,
    focus_window_s: float,
) -> None:
    start_time_s = forward_rows[0]["time_s"]
    output_path.parent.mkdir(parents=True, exist_ok=True)

    def relative_time(rows: list[dict[str, float]]) -> list[float]:
        return [row["time_s"] - start_time_s for row in rows]

    def delta_up(rows: list[dict[str, float]]) -> list[float]:
        up0 = rows[0]["up_m"]
        return [row["up_m"] - up0 for row in rows]

    fig, axes = plt.subplots(3, 1, figsize=(15, 11), sharex=True, constrained_layout=True)
    fig.suptitle(title, fontsize=16)

    for axis in axes:
        axis.axvspan(0.0, focus_window_s, color="#eeeeee", alpha=0.7)

    axes[0].plot(relative_time(forward_rows), [row["up_m"] for row in forward_rows], color="#2ca02c", linewidth=1.0, label="forward INS")
    axes[0].plot(relative_time(feedback_rows), [row["up_m"] for row in feedback_rows], color="#9467bd", linewidth=1.0, label="forward INS (feedback bias)")
    axes[0].plot(relative_time(optimized_rows), [row["up_m"] for row in optimized_rows], color="#1f77b4", linewidth=1.1, label="optimized")
    if rtk_rows:
        axes[0].plot(
            relative_time(rtk_rows),
            [row["up_m"] for row in rtk_rows],
            color="#ff7f0e",
            linewidth=0.9,
            alpha=0.9,
            label="RTKFIX",
        )
    axes[0].set_ylabel("Up [m]")
    axes[0].set_title("Absolute up position")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="upper right")

    axes[1].plot(relative_time(forward_rows), delta_up(forward_rows), color="#2ca02c", linewidth=1.0, label="forward INS")
    axes[1].plot(relative_time(feedback_rows), delta_up(feedback_rows), color="#9467bd", linewidth=1.0, label="forward INS (feedback bias)")
    axes[1].plot(relative_time(optimized_rows), delta_up(optimized_rows), color="#1f77b4", linewidth=1.1, label="optimized")
    if rtk_rows:
        axes[1].plot(
            relative_time(rtk_rows),
            delta_up(rtk_rows),
            color="#ff7f0e",
            linewidth=0.9,
            alpha=0.9,
            label="RTKFIX",
        )
    axes[1].set_ylabel("ΔUp [m]")
    axes[1].set_ylabel("Delta Up [m]")
    axes[1].set_title("Up relative to navigation start")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="upper right")

    axes[2].plot(relative_time(forward_rows), [row["vz_mps"] for row in forward_rows], color="#2ca02c", linewidth=1.0, label="forward INS")
    axes[2].plot(relative_time(feedback_rows), [row["vz_mps"] for row in feedback_rows], color="#9467bd", linewidth=1.0, label="forward INS (feedback bias)")
    axes[2].plot(relative_time(optimized_rows), [row["vz_mps"] for row in optimized_rows], color="#1f77b4", linewidth=1.1, label="optimized")
    axes[2].set_xlabel("Time since navigation start [s]")
    axes[2].set_ylabel("Vz [m/s]")
    axes[2].set_title("Vertical velocity")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc="upper right")

    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def print_stats(prefix: str, stats: dict[str, float]) -> None:
    print(f"{prefix}_sample_count={int(stats['sample_count'])}")
    print(f"{prefix}_delta_up_range_m={stats['delta_up_range_m']:.6f}")
    print(f"{prefix}_delta_up_std_m={stats['delta_up_std_m']:.6f}")
    print(f"{prefix}_delta_up_end_m={stats['delta_up_end_m']:.6f}")
    print(f"{prefix}_vz_range_mps={stats['vz_range_mps']:.6f}")
    print(f"{prefix}_vz_std_mps={stats['vz_std_mps']:.6f}")


def main() -> int:
    args = parse_args()
    trajectory_path = Path(args.trajectory)
    imu_path = Path(args.imu)
    gnss_path = Path(args.gnss) if args.gnss else None
    summary_path = Path(args.summary)
    config_path = Path(args.config)
    output_path = Path(args.output)
    csv_output_path = Path(args.csv_output) if args.csv_output else output_path.with_suffix(".csv")

    summary = heading_diagnostic.read_summary(summary_path)
    config = heading_diagnostic.read_config(config_path)
    optimized_rows_all = read_trajectory_rows(trajectory_path)
    optimized_first_state = optimized_rows_all[0]
    imu_rows = heading_diagnostic.read_imu_rows(imu_path)

    origin_lat_rad = summary["origin_lat_rad"]
    origin_lon_rad = summary["origin_lon_rad"]
    origin_h_m = summary["origin_h_m"]
    alignment_start_time_s = summary["alignment_start_time_s"]
    static_alignment_duration_s = summary.get("static_alignment_duration_s", 0.0)
    alignment_end_time_s = (
        alignment_start_time_s + static_alignment_duration_s
        if static_alignment_duration_s > 0.0
        else summary["navigation_start_time_s"]
    )
    start_time_s = optimized_first_state["time_s"]
    end_time_s = min(start_time_s + args.duration_s, optimized_rows_all[-1]["time_s"])

    initial_world_from_body, initial_acc_bias, initial_gyro_bias, stationary_count = heading_diagnostic.estimate_initial_alignment(
        imu_rows=imu_rows,
        alignment_start_time_s=alignment_start_time_s,
        navigation_start_time_s=alignment_end_time_s,
        origin_lat_rad=origin_lat_rad,
        gravity_mps2=float(config["gravity_mps2"]),
        stationary_gyro_threshold_radps=float(config["stationary_gyro_threshold_radps"]),
        stationary_acc_tolerance_mps2=float(config["stationary_acc_tolerance_mps2"]),
        min_sample_count=int(config["imu_dual_vector_min_sample_count"]),
        min_cross_norm=float(config["imu_dual_vector_min_cross_norm"]),
    )

    optimized_acc_bias = [
        optimized_first_state["bax"],
        optimized_first_state["bay"],
        optimized_first_state["baz"],
    ]
    optimized_gyro_bias = [
        optimized_first_state["bgx"],
        optimized_first_state["bgy"],
        optimized_first_state["bgz"],
    ]
    feedback_world_from_body, _, _, feedback_stationary_count = heading_diagnostic.estimate_alignment_from_feedback_bias(
        imu_rows=imu_rows,
        alignment_start_time_s=alignment_start_time_s,
        navigation_start_time_s=alignment_end_time_s,
        origin_lat_rad=origin_lat_rad,
        gravity_mps2=float(config["gravity_mps2"]),
        stationary_gyro_threshold_radps=float(config["stationary_gyro_threshold_radps"]),
        stationary_acc_tolerance_mps2=float(config["stationary_acc_tolerance_mps2"]),
        min_sample_count=int(config["imu_dual_vector_min_sample_count"]),
        min_cross_norm=float(config["imu_dual_vector_min_cross_norm"]),
        feedback_acc_bias=optimized_acc_bias,
        feedback_gyro_bias=optimized_gyro_bias,
    )

    initial_position_enu_m = [
        optimized_first_state["east_m"],
        optimized_first_state["north_m"],
        optimized_first_state["up_m"],
    ]
    initial_velocity_enu_mps = [
        optimized_first_state["vx_mps"],
        optimized_first_state["vy_mps"],
        optimized_first_state["vz_mps"],
    ]

    forward_rows_all = propagate_forward_altitude_rows(
        imu_rows=imu_rows,
        start_time_s=start_time_s,
        end_time_s=end_time_s,
        initial_world_from_body=initial_world_from_body,
        initial_acc_bias=initial_acc_bias,
        initial_gyro_bias=initial_gyro_bias,
        initial_position_enu_m=initial_position_enu_m,
        initial_velocity_enu_mps=initial_velocity_enu_mps,
        gravity_mps2=float(config["gravity_mps2"]),
    )
    feedback_rows_all = propagate_forward_altitude_rows(
        imu_rows=imu_rows,
        start_time_s=start_time_s,
        end_time_s=end_time_s,
        initial_world_from_body=feedback_world_from_body,
        initial_acc_bias=optimized_acc_bias,
        initial_gyro_bias=optimized_gyro_bias,
        initial_position_enu_m=initial_position_enu_m,
        initial_velocity_enu_mps=initial_velocity_enu_mps,
        gravity_mps2=float(config["gravity_mps2"]),
    )
    optimized_rows = [row for row in optimized_rows_all if row["time_s"] <= end_time_s + TIME_EPSILON_S]
    if gnss_path:
        rtk_rows_all = heading_diagnostic.read_rtkfix_rows(
            gnss_path,
            origin_lat_rad,
            origin_lon_rad,
            origin_h_m,
        )
        rtk_rows = filter_rows_by_time_window(rtk_rows_all, start_time_s, end_time_s)
    else:
        rtk_rows = []

    make_plot(
        forward_rows_all,
        feedback_rows_all,
        optimized_rows,
        rtk_rows,
        output_path,
        args.title,
        args.focus_window_s,
    )
    write_csv(csv_output_path, forward_rows_all, feedback_rows_all, optimized_rows, rtk_rows)

    print(f"plot_saved={output_path}")
    print(f"csv_saved={csv_output_path}")
    print(f"stationary_sample_count={stationary_count}")
    print(f"feedback_stationary_sample_count={feedback_stationary_count}")
    if rtk_rows:
        print(f"rtkfix_sample_count={len(rtk_rows)}")
    for label, rows in (
        ("forward_first10s", forward_rows_all),
        ("feedback_first10s", feedback_rows_all),
        ("optimized_first10s", optimized_rows),
    ):
        stats = compute_window_stats(rows, start_time_s, args.focus_window_s)
        print_stats(label, stats)
    for label, rows in (
        ("forward_first30s", forward_rows_all),
        ("feedback_first30s", feedback_rows_all),
        ("optimized_first30s", optimized_rows),
    ):
        stats = compute_window_stats(rows, start_time_s, min(args.duration_s, 30.0))
        print_stats(label, stats)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
