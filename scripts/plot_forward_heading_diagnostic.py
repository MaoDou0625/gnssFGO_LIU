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
EARTH_ROTATION_RATE_RADPS = 7.292115e-5
TIME_EPSILON_S = 1e-9
MIN_HEADING_DISPLACEMENT_M = 0.2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Diagnose heading drift by comparing optimized navigation heading, "
            "single-pass IMU forward-propagated heading, and RTK position-difference heading."
        )
    )
    parser.add_argument("--trajectory", required=True, help="Path to trajectory.csv")
    parser.add_argument("--imu", required=True, help="Path to imu_gnss_fgo.txt")
    parser.add_argument("--gnss", required=True, help="Path to gnss_solution_gnss_fgo.txt")
    parser.add_argument("--summary", required=True, help="Path to summary.txt")
    parser.add_argument("--config", required=True, help="Path to config_snapshot.cfg")
    parser.add_argument("--output", required=True, help="Output image path")
    parser.add_argument(
        "--csv-output",
        default="",
        help="Optional CSV output path for the forward-heading diagnostic series",
    )
    parser.add_argument(
        "--time-tolerance-s",
        type=float,
        default=0.12,
        help="Maximum allowed timestamp mismatch when pairing heading samples",
    )
    parser.add_argument(
        "--heading-window-s",
        type=float,
        default=1.0,
        help="Centered RTK position-difference window in seconds",
    )
    parser.add_argument(
        "--title",
        default="Forward heading diagnostic vs optimized navigation and RTK",
        help="Figure title",
    )
    return parser.parse_args()


def safe_float(value: str) -> float:
    parsed = float(value)
    if math.isnan(parsed) or math.isinf(parsed):
        raise ValueError("non-finite numeric value")
    return parsed


def read_key_value_file(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        result[key.strip()] = value.strip()
    return result


def read_summary(path: Path) -> dict[str, float]:
    values = read_key_value_file(path)
    parsed: dict[str, float] = {}
    for key, value in values.items():
        try:
            parsed[key] = float(value)
        except ValueError:
            continue
    return parsed


def read_config(path: Path) -> dict[str, str]:
    return read_key_value_file(path)


def parse_bool(value: str) -> bool:
    lowered = value.strip().lower()
    if lowered in {"true", "1", "yes", "on"}:
        return True
    if lowered in {"false", "0", "no", "off"}:
        return False
    raise ValueError(f"invalid boolean value: {value}")


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


def read_first_trajectory_state(path: Path, start_time_s: float | None = None) -> dict[str, float]:
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        raw = None
        for candidate in reader:
            if start_time_s is None or safe_float(candidate["time_s"]) + TIME_EPSILON_S >= start_time_s:
                raw = candidate
                break
        if raw is None:
            raise ValueError(f"No trajectory rows found in {path} at or after {start_time_s}")

    return {
        "time_s": safe_float(raw["time_s"]),
        "yaw_deg": math.degrees(safe_float(raw["yaw_rad"])),
        "pitch_deg": math.degrees(safe_float(raw["pitch_rad"])),
        "roll_deg": math.degrees(safe_float(raw["roll_rad"])),
        "bax": safe_float(raw["bax"]),
        "bay": safe_float(raw["bay"]),
        "baz": safe_float(raw["baz"]),
        "bgx": safe_float(raw["bgx"]),
        "bgy": safe_float(raw["bgy"]),
        "bgz": safe_float(raw["bgz"]),
    }


def read_imu_rows(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.reader(file, delimiter="\t")
        for raw in reader:
            if not raw:
                continue
            rows.append(
                {
                    "time_s": safe_float(raw[0]),
                    "gyro_x": safe_float(raw[1]),
                    "gyro_y": safe_float(raw[2]),
                    "gyro_z": safe_float(raw[3]),
                    "acc_x": safe_float(raw[4]),
                    "acc_y": safe_float(raw[5]),
                    "acc_z": safe_float(raw[6]),
                }
            )
    if not rows:
        raise ValueError(f"No IMU rows found in {path}")
    return rows


def filter_rows_from_time(rows: list[dict[str, float]], start_time_s: float) -> list[dict[str, float]]:
    filtered = [row for row in rows if row["time_s"] + TIME_EPSILON_S >= start_time_s]
    if filtered:
        return filtered
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


def mean_of_stationary_window(
    imu_rows: list[dict[str, float]],
    alignment_start_time_s: float,
    duration_s: float,
    gravity_mps2: float,
    stationary_gyro_threshold_radps: float,
    stationary_acc_tolerance_mps2: float,
) -> tuple[list[float], list[float], int]:
    end_time_s = alignment_start_time_s + duration_s
    stationary_acc_sum = [0.0, 0.0, 0.0]
    stationary_gyro_sum = [0.0, 0.0, 0.0]
    stationary_count = 0

    for row in imu_rows:
        time_s = row["time_s"]
        if time_s + TIME_EPSILON_S < alignment_start_time_s:
            continue
        if time_s > end_time_s + TIME_EPSILON_S:
            break

        gyro = [row["gyro_x"], row["gyro_y"], row["gyro_z"]]
        acc = [row["acc_x"], row["acc_y"], row["acc_z"]]
        gyro_norm = math.sqrt(sum(value * value for value in gyro))
        acc_norm = math.sqrt(sum(value * value for value in acc))
        if gyro_norm <= stationary_gyro_threshold_radps and abs(acc_norm - gravity_mps2) <= stationary_acc_tolerance_mps2:
            stationary_acc_sum = [lhs + rhs for lhs, rhs in zip(stationary_acc_sum, acc)]
            stationary_gyro_sum = [lhs + rhs for lhs, rhs in zip(stationary_gyro_sum, gyro)]
            stationary_count += 1

    if stationary_count == 0:
        raise ValueError("No stationary IMU samples found in the requested static alignment window")

    mean_acc = [value / stationary_count for value in stationary_acc_sum]
    mean_gyro = [value / stationary_count for value in stationary_gyro_sum]
    return mean_acc, mean_gyro, stationary_count


def vector_norm(vector: list[float]) -> float:
    return math.sqrt(sum(value * value for value in vector))


def safe_normalize(vector: list[float]) -> list[float]:
    norm = vector_norm(vector)
    if norm <= 1e-12 or not math.isfinite(norm):
        raise ValueError("cannot normalize a near-zero vector")
    return [value / norm for value in vector]


def cross(lhs: list[float], rhs: list[float]) -> list[float]:
    return [
        lhs[1] * rhs[2] - lhs[2] * rhs[1],
        lhs[2] * rhs[0] - lhs[0] * rhs[2],
        lhs[0] * rhs[1] - lhs[1] * rhs[0],
    ]


def dot(lhs: list[float], rhs: list[float]) -> float:
    return sum(left * right for left, right in zip(lhs, rhs))


def matmul(lhs: list[list[float]], rhs: list[list[float]]) -> list[list[float]]:
    return [
        [
            sum(lhs[row_index][column_index] * rhs[column_index][out_index] for column_index in range(3))
            for out_index in range(3)
        ]
        for row_index in range(3)
    ]


def matvec(matrix: list[list[float]], vector: list[float]) -> list[float]:
    return [sum(matrix[row][col] * vector[col] for col in range(3)) for row in range(3)]


def transpose(matrix: list[list[float]]) -> list[list[float]]:
    return [[matrix[col][row] for col in range(3)] for row in range(3)]


def build_triad_frame(primary: list[float], secondary: list[float]) -> list[list[float]]:
    axis_1 = safe_normalize(primary)
    axis_2 = safe_normalize(cross(axis_1, secondary))
    axis_3 = cross(axis_1, axis_2)
    return [
        [axis_1[0], axis_2[0], axis_3[0]],
        [axis_1[1], axis_2[1], axis_3[1]],
        [axis_1[2], axis_2[2], axis_3[2]],
    ]


def rot3_to_ypr_deg(world_from_body: list[list[float]]) -> tuple[float, float, float]:
    yaw = math.degrees(math.atan2(world_from_body[1][0], world_from_body[0][0]))
    pitch = math.degrees(
        math.atan2(-world_from_body[2][0], math.sqrt(world_from_body[2][1] ** 2 + world_from_body[2][2] ** 2))
    )
    roll = math.degrees(math.atan2(world_from_body[2][1], world_from_body[2][2]))
    return yaw, pitch, roll


def exp_so3(omega_dt: list[float]) -> list[list[float]]:
    theta = vector_norm(omega_dt)
    if theta <= 1e-12:
        return [
            [1.0, -omega_dt[2], omega_dt[1]],
            [omega_dt[2], 1.0, -omega_dt[0]],
            [-omega_dt[1], omega_dt[0], 1.0],
        ]

    axis = [value / theta for value in omega_dt]
    skew = [
        [0.0, -axis[2], axis[1]],
        [axis[2], 0.0, -axis[0]],
        [-axis[1], axis[0], 0.0],
    ]
    skew_squared = matmul(skew, skew)
    sin_theta = math.sin(theta)
    one_minus_cos_theta = 1.0 - math.cos(theta)
    identity = [[1.0 if row == col else 0.0 for col in range(3)] for row in range(3)]
    return [
        [
            identity[row][col] + sin_theta * skew[row][col] + one_minus_cos_theta * skew_squared[row][col]
            for col in range(3)
        ]
        for row in range(3)
    ]


def solve_dual_vector_alignment(
    mean_acc: list[float],
    mean_gyro: list[float],
    origin_lat_rad: float,
    gravity_mps2: float,
    min_cross_norm: float,
) -> tuple[list[list[float]], list[float], list[float]]:
    gravity_enu = [0.0, 0.0, gravity_mps2]
    earth_rate_enu = [
        0.0,
        EARTH_ROTATION_RATE_RADPS * math.cos(origin_lat_rad),
        EARTH_ROTATION_RATE_RADPS * math.sin(origin_lat_rad),
    ]
    if vector_norm(earth_rate_enu) <= 1e-12 or vector_norm(mean_acc) <= 1e-12 or vector_norm(mean_gyro) <= 1e-12:
        raise ValueError("dual-vector initialization received a near-zero alignment vector")

    reference_cross_norm = vector_norm(cross(safe_normalize(gravity_enu), safe_normalize(earth_rate_enu)))
    measurement_cross_norm = vector_norm(cross(safe_normalize(mean_acc), safe_normalize(mean_gyro)))
    if reference_cross_norm < min_cross_norm or measurement_cross_norm < min_cross_norm:
        raise ValueError("dual-vector initialization cross norm below threshold")

    navigation_frame = build_triad_frame(gravity_enu, earth_rate_enu)
    body_frame = build_triad_frame(mean_acc, mean_gyro)
    world_from_body = matmul(navigation_frame, transpose(body_frame))
    predicted_acc_body = matvec(transpose(world_from_body), gravity_enu)
    predicted_gyro_body = matvec(transpose(world_from_body), earth_rate_enu)
    return world_from_body, predicted_acc_body, predicted_gyro_body


def estimate_initial_alignment(
    imu_rows: list[dict[str, float]],
    alignment_start_time_s: float,
    navigation_start_time_s: float,
    origin_lat_rad: float,
    gravity_mps2: float,
    stationary_gyro_threshold_radps: float,
    stationary_acc_tolerance_mps2: float,
    min_sample_count: int,
    min_cross_norm: float,
) -> tuple[list[list[float]], list[float], list[float], int]:
    duration_s = navigation_start_time_s - alignment_start_time_s
    if duration_s <= 0.0:
        raise ValueError("navigation_start_time_s must be after alignment_start_time_s")

    mean_acc, mean_gyro, stationary_count = mean_of_stationary_window(
        imu_rows,
        alignment_start_time_s,
        duration_s,
        gravity_mps2,
        stationary_gyro_threshold_radps,
        stationary_acc_tolerance_mps2,
    )
    if stationary_count < max(min_sample_count, 1):
        raise ValueError("insufficient stationary IMU samples for dual-vector initialization")

    world_from_body, predicted_acc_body, predicted_gyro_body = solve_dual_vector_alignment(
        mean_acc=mean_acc,
        mean_gyro=mean_gyro,
        origin_lat_rad=origin_lat_rad,
        gravity_mps2=gravity_mps2,
        min_cross_norm=min_cross_norm,
    )
    acc_bias = [measured - predicted for measured, predicted in zip(mean_acc, predicted_acc_body)]
    gyro_bias = [measured - predicted for measured, predicted in zip(mean_gyro, predicted_gyro_body)]
    return world_from_body, acc_bias, gyro_bias, stationary_count


def estimate_alignment_from_feedback_bias(
    imu_rows: list[dict[str, float]],
    alignment_start_time_s: float,
    navigation_start_time_s: float,
    origin_lat_rad: float,
    gravity_mps2: float,
    stationary_gyro_threshold_radps: float,
    stationary_acc_tolerance_mps2: float,
    min_sample_count: int,
    min_cross_norm: float,
    feedback_acc_bias: list[float],
    feedback_gyro_bias: list[float],
) -> tuple[list[list[float]], list[float], list[float], int]:
    duration_s = navigation_start_time_s - alignment_start_time_s
    if duration_s <= 0.0:
        raise ValueError("navigation_start_time_s must be after alignment_start_time_s")

    mean_acc, mean_gyro, stationary_count = mean_of_stationary_window(
        imu_rows,
        alignment_start_time_s,
        duration_s,
        gravity_mps2,
        stationary_gyro_threshold_radps,
        stationary_acc_tolerance_mps2,
    )
    if stationary_count < max(min_sample_count, 1):
        raise ValueError("insufficient stationary IMU samples for feedback alignment")

    corrected_mean_acc = [measured - bias for measured, bias in zip(mean_acc, feedback_acc_bias)]
    corrected_mean_gyro = [measured - bias for measured, bias in zip(mean_gyro, feedback_gyro_bias)]
    world_from_body, predicted_acc_body, predicted_gyro_body = solve_dual_vector_alignment(
        mean_acc=corrected_mean_acc,
        mean_gyro=corrected_mean_gyro,
        origin_lat_rad=origin_lat_rad,
        gravity_mps2=gravity_mps2,
        min_cross_norm=min_cross_norm,
    )
    residual_acc_bias = [measured - predicted for measured, predicted in zip(corrected_mean_acc, predicted_acc_body)]
    residual_gyro_bias = [measured - predicted for measured, predicted in zip(corrected_mean_gyro, predicted_gyro_body)]
    return world_from_body, residual_acc_bias, residual_gyro_bias, stationary_count


def forward_propagate_heading_rows(
    imu_rows: list[dict[str, float]],
    navigation_start_time_s: float,
    end_time_s: float,
    initial_world_from_body: list[list[float]],
    initial_gyro_bias: list[float],
) -> list[dict[str, float]]:
    if end_time_s <= navigation_start_time_s:
        raise ValueError("end_time_s must be after navigation_start_time_s")

    rows: list[dict[str, float]] = []
    current_world_from_body = [row[:] for row in initial_world_from_body]
    yaw_deg, pitch_deg, roll_deg = rot3_to_ypr_deg(current_world_from_body)
    rows.append(
        {
            "time_s": navigation_start_time_s,
            "yaw_deg": yaw_deg,
            "pitch_deg": pitch_deg,
            "roll_deg": roll_deg,
            "corrected_gyro_z_dps": math.nan,
        }
    )

    for imu_index in range(len(imu_rows) - 1):
        current_row = imu_rows[imu_index]
        next_row = imu_rows[imu_index + 1]
        interval_start_s = max(current_row["time_s"], navigation_start_time_s)
        interval_end_s = min(next_row["time_s"], end_time_s)
        if interval_end_s <= interval_start_s + TIME_EPSILON_S:
            continue

        delta_time_s = interval_end_s - interval_start_s
        corrected_gyro = [
            current_row["gyro_x"] - initial_gyro_bias[0],
            current_row["gyro_y"] - initial_gyro_bias[1],
            current_row["gyro_z"] - initial_gyro_bias[2],
        ]
        delta_rotation = exp_so3([value * delta_time_s for value in corrected_gyro])
        current_world_from_body = matmul(current_world_from_body, delta_rotation)
        yaw_deg, pitch_deg, roll_deg = rot3_to_ypr_deg(current_world_from_body)
        rows.append(
            {
                "time_s": interval_end_s,
                "yaw_deg": yaw_deg,
                "pitch_deg": pitch_deg,
                "roll_deg": roll_deg,
                "corrected_gyro_z_dps": math.degrees(corrected_gyro[2]),
            }
        )

        if interval_end_s >= end_time_s - TIME_EPSILON_S:
            break

    unwrapped = unwrap_angles_deg([row["yaw_deg"] for row in rows])
    for row, yaw_unwrapped_deg in zip(rows, unwrapped):
        row["yaw_unwrapped_deg"] = yaw_unwrapped_deg
    return rows


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
        pairs.append(
            {
                "time_s": rtk_time_s,
                "nav_heading_deg": nav_heading_deg,
                "nav_heading_unwrapped_deg": float(nav_row["yaw_unwrapped_deg"]),
                "rtk_heading_deg": rtk_heading_deg,
                "rtk_heading_unwrapped_deg": float(rtk_row["heading_unwrapped_deg"]),
                "heading_error_deg": wrap_angle_deg(nav_heading_deg - rtk_heading_deg),
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


def build_stats(prefix: str, pairs: list[dict[str, float]]) -> dict[str, float]:
    errors_deg = [pair["heading_error_deg"] for pair in pairs]
    abs_errors_deg = [abs(value) for value in errors_deg]
    return {
        f"{prefix}_matched": float(len(pairs)),
        f"{prefix}_mean_abs_deg": statistics.mean(abs_errors_deg),
        f"{prefix}_rms_deg": rms(errors_deg),
        f"{prefix}_max_abs_deg": max(abs_errors_deg),
    }


def build_stats_text(
    forward_pairs: list[dict[str, float]],
    feedback_pairs: list[dict[str, float]],
    optimized_pairs: list[dict[str, float]],
    initial_gyro_bias_radps: list[float],
    optimized_gyro_bias_radps: list[float],
    feedback_residual_gyro_bias_radps: list[float],
) -> str:
    forward_stats = build_stats("forward", forward_pairs)
    feedback_stats = build_stats("feedback", feedback_pairs)
    optimized_stats = build_stats("optimized", optimized_pairs)
    return "\n".join(
        [
            f"forward matched={int(forward_stats['forward_matched'])}",
            f"forward mean abs={forward_stats['forward_mean_abs_deg']:.3f} deg",
            f"forward rms={forward_stats['forward_rms_deg']:.3f} deg",
            f"forward max abs={forward_stats['forward_max_abs_deg']:.3f} deg",
            f"feedback matched={int(feedback_stats['feedback_matched'])}",
            f"feedback mean abs={feedback_stats['feedback_mean_abs_deg']:.3f} deg",
            f"feedback rms={feedback_stats['feedback_rms_deg']:.3f} deg",
            f"feedback max abs={feedback_stats['feedback_max_abs_deg']:.3f} deg",
            f"optimized matched={int(optimized_stats['optimized_matched'])}",
            f"optimized mean abs={optimized_stats['optimized_mean_abs_deg']:.3f} deg",
            f"optimized rms={optimized_stats['optimized_rms_deg']:.3f} deg",
            f"optimized max abs={optimized_stats['optimized_max_abs_deg']:.3f} deg",
            f"init bgz={initial_gyro_bias_radps[2]:.6e} rad/s",
            f"opt bgz={optimized_gyro_bias_radps[2]:.6e} rad/s",
            f"feedback residual bgz={feedback_residual_gyro_bias_radps[2]:.6e} rad/s",
        ]
    )


def make_plot(
    forward_rows: list[dict[str, float]],
    feedback_rows: list[dict[str, float]],
    optimized_rows: list[dict[str, float]],
    rtk_heading_rows: list[dict[str, float | bool | str]],
    forward_pairs: list[dict[str, float]],
    feedback_pairs: list[dict[str, float]],
    optimized_pairs: list[dict[str, float]],
    output_path: Path,
    title: str,
    initial_gyro_bias_radps: list[float],
    optimized_gyro_bias_radps: list[float],
    feedback_residual_gyro_bias_radps: list[float],
) -> None:
    forward_time = [row["time_s"] for row in forward_rows]
    forward_yaw_unwrapped_deg = [row["yaw_unwrapped_deg"] for row in forward_rows]
    feedback_time = [row["time_s"] for row in feedback_rows]
    feedback_yaw_unwrapped_deg = [row["yaw_unwrapped_deg"] for row in feedback_rows]
    optimized_time = [row["time_s"] for row in optimized_rows]
    optimized_yaw_unwrapped_deg = [row["yaw_unwrapped_deg"] for row in optimized_rows]

    forward_pair_time = [pair["time_s"] for pair in forward_pairs]
    feedback_pair_time = [pair["time_s"] for pair in feedback_pairs]
    optimized_pair_time = [pair["time_s"] for pair in optimized_pairs]
    forward_error_deg = [pair["heading_error_deg"] for pair in forward_pairs]
    feedback_error_deg = [pair["heading_error_deg"] for pair in feedback_pairs]
    optimized_error_deg = [pair["heading_error_deg"] for pair in optimized_pairs]
    aligned_rtk_deg = aligned_rtk_unwrapped_deg(forward_pairs)

    corrected_gyro_rows = [row for row in forward_rows if math.isfinite(row["corrected_gyro_z_dps"])]
    corrected_gyro_time = [row["time_s"] for row in corrected_gyro_rows]
    corrected_gyro_z_dps = [row["corrected_gyro_z_dps"] for row in corrected_gyro_rows]
    feedback_corrected_gyro_rows = [row for row in feedback_rows if math.isfinite(row["corrected_gyro_z_dps"])]
    feedback_corrected_gyro_time = [row["time_s"] for row in feedback_corrected_gyro_rows]
    feedback_corrected_gyro_z_dps = [row["corrected_gyro_z_dps"] for row in feedback_corrected_gyro_rows]

    valid_rtk_rows = [row for row in rtk_heading_rows if row["valid_heading"]]
    invalid_rtk_rows = [row for row in rtk_heading_rows if not row["valid_heading"]]
    valid_rtk_time = [float(row["time_s"]) for row in valid_rtk_rows]
    valid_rtk_displacement_m = [float(row["window_displacement_m"]) for row in valid_rtk_rows]
    invalid_rtk_time = [float(row["time_s"]) for row in invalid_rtk_rows]
    invalid_rtk_displacement_m = [
        float(row["window_displacement_m"]) if math.isfinite(float(row["window_displacement_m"])) else 0.0
        for row in invalid_rtk_rows
    ]

    fig, axes = plt.subplots(3, 1, figsize=(16, 11), sharex=True, constrained_layout=True)
    fig.suptitle(title, fontsize=16)

    ax_heading = axes[0]
    ax_heading.plot(forward_time, forward_yaw_unwrapped_deg, color="#2ca02c", linewidth=1.1, label="forward yaw (initial dual-vector bias)")
    ax_heading.plot(feedback_time, feedback_yaw_unwrapped_deg, color="#9467bd", linewidth=1.1, label="forward yaw (optimized-bias feedback)")
    ax_heading.plot(optimized_time, optimized_yaw_unwrapped_deg, color="#1f77b4", linewidth=1.0, label="optimized yaw")
    ax_heading.plot(forward_pair_time, aligned_rtk_deg, color="#ff7f0e", linewidth=0.9, marker="o", markersize=2.4, label="RTK diff heading")
    ax_heading.set_ylabel("Heading [deg]")
    ax_heading.set_title("Heading comparison")
    ax_heading.grid(True, alpha=0.3)
    ax_heading.legend(loc="upper right")

    ax_error = axes[1]
    ax_error.plot(forward_pair_time, forward_error_deg, color="#2ca02c", linewidth=1.0, label="forward minus RTK")
    ax_error.plot(feedback_pair_time, feedback_error_deg, color="#9467bd", linewidth=1.0, label="feedback-forward minus RTK")
    ax_error.plot(optimized_pair_time, optimized_error_deg, color="#d62728", linewidth=1.0, label="optimized minus RTK")
    ax_error.axhline(0.0, color="black", linewidth=0.8, alpha=0.6)
    ax_error.set_ylabel("Error [deg]")
    ax_error.set_title("Heading error vs RTK")
    ax_error.grid(True, alpha=0.3)
    ax_error.legend(loc="upper right")
    ax_error.text(
        0.01,
        0.97,
        build_stats_text(
            forward_pairs,
            feedback_pairs,
            optimized_pairs,
            initial_gyro_bias_radps,
            optimized_gyro_bias_radps,
            feedback_residual_gyro_bias_radps,
        ),
        transform=ax_error.transAxes,
        va="top",
        ha="left",
        fontsize=9.5,
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.85},
    )

    ax_diag = axes[2]
    ax_diag.plot(corrected_gyro_time, corrected_gyro_z_dps, color="#2ca02c", linewidth=1.0, label="corrected gyro z (initial bias)")
    ax_diag.plot(feedback_corrected_gyro_time, feedback_corrected_gyro_z_dps, color="#9467bd", linewidth=1.0, label="corrected gyro z (optimized bias)")
    ax_diag.axhline(0.0, color="black", linewidth=0.8, alpha=0.6)
    ax_diag.scatter(invalid_rtk_time, invalid_rtk_displacement_m, color="#d62728", s=12, marker="x", label="invalid RTK window")
    ax_diag.plot(valid_rtk_time, valid_rtk_displacement_m, color="#8c564b", linewidth=0.9, alpha=0.9, label="RTK window displacement")
    ax_diag.axhline(MIN_HEADING_DISPLACEMENT_M, color="#7f7f7f", linewidth=0.9, linestyle="--", label="min displacement threshold")
    ax_diag.set_xlabel("Time [s]")
    ax_diag.set_ylabel("Gyro z [deg/s] / Displacement [m]")
    ax_diag.set_title("Forward propagation diagnostic")
    ax_diag.grid(True, alpha=0.3)
    ax_diag.legend(loc="upper right")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def write_csv(
    path: Path,
    forward_rows: list[dict[str, float]],
    feedback_rows: list[dict[str, float]],
    optimized_rows: list[dict[str, float]],
) -> None:
    optimized_times = [row["time_s"] for row in optimized_rows]
    feedback_times = [row["time_s"] for row in feedback_rows]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(
            [
                "time_s",
                "forward_yaw_deg",
                "forward_yaw_unwrapped_deg",
                "forward_pitch_deg",
                "forward_roll_deg",
                "corrected_gyro_z_dps",
                "feedback_yaw_deg",
                "feedback_yaw_unwrapped_deg",
                "feedback_pitch_deg",
                "feedback_roll_deg",
                "feedback_corrected_gyro_z_dps",
                "feedback_minus_forward_deg",
                "nearest_optimized_time_s",
                "optimized_yaw_deg",
                "optimized_yaw_unwrapped_deg",
                "optimized_minus_forward_deg",
                "optimized_minus_feedback_deg",
            ]
        )
        for row in forward_rows:
            feedback_index = find_nearest_index(feedback_times, row["time_s"])
            feedback_row = feedback_rows[feedback_index]
            optimized_index = find_nearest_index(optimized_times, row["time_s"])
            optimized_row = optimized_rows[optimized_index]
            writer.writerow(
                [
                    row["time_s"],
                    row["yaw_deg"],
                    row["yaw_unwrapped_deg"],
                    row["pitch_deg"],
                    row["roll_deg"],
                    row["corrected_gyro_z_dps"],
                    feedback_row["yaw_deg"],
                    feedback_row["yaw_unwrapped_deg"],
                    feedback_row["pitch_deg"],
                    feedback_row["roll_deg"],
                    feedback_row["corrected_gyro_z_dps"],
                    wrap_angle_deg(feedback_row["yaw_deg"] - row["yaw_deg"]),
                    optimized_row["time_s"],
                    optimized_row["yaw_deg"],
                    optimized_row["yaw_unwrapped_deg"],
                    wrap_angle_deg(optimized_row["yaw_deg"] - row["yaw_deg"]),
                    wrap_angle_deg(optimized_row["yaw_deg"] - feedback_row["yaw_deg"]),
                ]
            )


def main() -> int:
    args = parse_args()
    trajectory_path = Path(args.trajectory)
    imu_path = Path(args.imu)
    gnss_path = Path(args.gnss)
    summary_path = Path(args.summary)
    config_path = Path(args.config)
    output_path = Path(args.output)
    csv_output_path = Path(args.csv_output) if args.csv_output else output_path.with_suffix(".csv")

    summary = read_summary(summary_path)
    config = read_config(config_path)
    dynamic_start_time_s = summary.get("dynamic_start_time_s", summary["navigation_start_time_s"])
    optimized_first_state = read_first_trajectory_state(trajectory_path, dynamic_start_time_s)
    optimized_rows = filter_rows_from_time(read_nav_heading_rows(trajectory_path), dynamic_start_time_s)
    imu_rows = read_imu_rows(imu_path)

    origin_lat_rad = summary["origin_lat_rad"]
    origin_lon_rad = summary["origin_lon_rad"]
    origin_h_m = summary["origin_h_m"]
    alignment_start_time_s = summary["alignment_start_time_s"]
    static_alignment_duration_s = summary.get("static_alignment_duration_s", 0.0)
    alignment_end_time_s = (
        alignment_start_time_s + static_alignment_duration_s
        if static_alignment_duration_s > 0.0
        else summary.get("dynamic_start_time_s", summary["navigation_start_time_s"])
    )
    navigation_start_time_s = dynamic_start_time_s
    end_time_s = optimized_rows[-1]["time_s"]

    initial_world_from_body, initial_acc_bias, initial_gyro_bias, stationary_count = estimate_initial_alignment(
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
    feedback_world_from_body, feedback_residual_acc_bias, feedback_residual_gyro_bias, feedback_stationary_count = estimate_alignment_from_feedback_bias(
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

    forward_rows = forward_propagate_heading_rows(
        imu_rows=imu_rows,
        navigation_start_time_s=navigation_start_time_s,
        end_time_s=end_time_s,
        initial_world_from_body=initial_world_from_body,
        initial_gyro_bias=initial_gyro_bias,
    )
    feedback_rows = forward_propagate_heading_rows(
        imu_rows=imu_rows,
        navigation_start_time_s=navigation_start_time_s,
        end_time_s=end_time_s,
        initial_world_from_body=feedback_world_from_body,
        initial_gyro_bias=optimized_gyro_bias,
    )
    rtk_rows = read_rtkfix_rows(gnss_path, origin_lat_rad, origin_lon_rad, origin_h_m)
    rtk_heading_rows = build_rtk_heading_rows(rtk_rows, args.heading_window_s)
    forward_pairs = match_heading_pairs(forward_rows, rtk_heading_rows, args.time_tolerance_s)
    feedback_pairs = match_heading_pairs(feedback_rows, rtk_heading_rows, args.time_tolerance_s)
    optimized_pairs = match_heading_pairs(optimized_rows, rtk_heading_rows, args.time_tolerance_s)

    make_plot(
        forward_rows=forward_rows,
        feedback_rows=feedback_rows,
        optimized_rows=optimized_rows,
        rtk_heading_rows=rtk_heading_rows,
        forward_pairs=forward_pairs,
        feedback_pairs=feedback_pairs,
        optimized_pairs=optimized_pairs,
        output_path=output_path,
        title=args.title,
        initial_gyro_bias_radps=initial_gyro_bias,
        optimized_gyro_bias_radps=optimized_gyro_bias,
        feedback_residual_gyro_bias_radps=feedback_residual_gyro_bias,
    )
    write_csv(csv_output_path, forward_rows, feedback_rows, optimized_rows)

    print(f"plot_saved={output_path}")
    print(f"csv_saved={csv_output_path}")
    print(f"stationary_sample_count={stationary_count}")
    print(f"feedback_stationary_sample_count={feedback_stationary_count}")
    print(
        "initial_acc_bias_mps2="
        + ",".join(f"{value:.12e}" for value in initial_acc_bias)
    )
    print(
        "initial_gyro_bias_radps="
        + ",".join(f"{value:.12e}" for value in initial_gyro_bias)
    )
    print(
        "optimized_first_state_acc_bias_mps2="
        + ",".join(f"{value:.12e}" for value in optimized_acc_bias)
    )
    print(
        "optimized_first_state_gyro_bias_radps="
        + ",".join(f"{value:.12e}" for value in optimized_gyro_bias)
    )
    print(
        "feedback_residual_acc_bias_mps2="
        + ",".join(f"{value:.12e}" for value in feedback_residual_acc_bias)
    )
    print(
        "feedback_residual_gyro_bias_radps="
        + ",".join(f"{value:.12e}" for value in feedback_residual_gyro_bias)
    )
    for label, pairs in (("forward", forward_pairs), ("feedback", feedback_pairs), ("optimized", optimized_pairs)):
        stats = build_stats(label, pairs)
        print(f"{label}_matched={int(stats[f'{label}_matched'])}")
        print(f"{label}_mean_abs_deg={stats[f'{label}_mean_abs_deg']:.6f}")
        print(f"{label}_rms_deg={stats[f'{label}_rms_deg']:.6f}")
        print(f"{label}_max_abs_deg={stats[f'{label}_max_abs_deg']:.6f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
