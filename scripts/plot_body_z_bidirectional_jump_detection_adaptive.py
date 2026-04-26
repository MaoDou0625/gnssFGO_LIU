#!/usr/bin/env python3
"""Detect bidirectional body-z velocity jump windows from integrated acceleration.

The detector is intentionally diagnostic: it does not modify the navigation
solution.  It selects DOWN and UP jumps independently.  Within each direction,
the current strongest remaining peak defines the next level threshold; after a
level is selected, its windows are suppressed and the next level is recomputed
from the remaining signal.
"""

from __future__ import annotations

import argparse
import math
import os
import subprocess
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd


@dataclass
class DetectorConfig:
    pre_post_window_s: float = 0.25
    center_gap_s: float = 0.03
    velocity_smooth_s: float = 0.20
    threshold_ratio: float = 0.35
    support_ratio: float = 0.35
    min_score_mps: float = 0.008
    min_separation_s: float = 0.50
    max_window_duration_s: float = 0.55
    max_levels: int = 12
    dense_gap_s: float = 0.80
    dense_peak_count: int = 20
    dense_peak_floor_ratio: float = 4.0


@dataclass
class SelectedWindow:
    direction: str
    selection_level: int
    start_index: int
    center_index: int
    end_index: int
    start_rel_time_s: float
    center_rel_time_s: float
    end_rel_time_s: float
    duration_s: float
    pre_velocity_mps: float
    post_velocity_mps: float
    signed_delta_velocity_mps: float
    direction_score_mps: float
    signed_step_metric_mps: float
    level_threshold_mps: float
    level_max_peak_mps: float
    level_noise_floor_mps: float
    min_acc_mps2: float
    max_acc_mps2: float
    mean_acc_mps2: float


def odd_window_samples(window_s: float, dt_s: float) -> int:
    samples = max(1, int(round(window_s / max(dt_s, 1e-9))))
    if samples % 2 == 0:
        samples += 1
    return samples


def centered_mean(values: np.ndarray, window_s: float, dt_s: float) -> np.ndarray:
    count = odd_window_samples(window_s, dt_s)
    return (
        pd.Series(values)
        .rolling(window=count, center=True, min_periods=max(1, count // 4))
        .mean()
        .to_numpy()
    )


def centered_step_metric(values: np.ndarray, dt_s: float, cfg: DetectorConfig) -> np.ndarray:
    window_count = max(3, int(round(cfg.pre_post_window_s / max(dt_s, 1e-9))))
    gap_count = max(0, int(round(cfg.center_gap_s / max(dt_s, 1e-9))))
    min_periods = max(3, window_count // 3)
    series = pd.Series(values)
    left = series.shift(gap_count + 1).rolling(window_count, min_periods=min_periods).median()
    right = (
        pd.Series(values[::-1])
        .shift(gap_count + 1)
        .rolling(window_count, min_periods=min_periods)
        .median()
        .to_numpy()[::-1]
    )
    return right - left.to_numpy()


def robust_sigma(values: np.ndarray) -> float:
    finite_values = values[np.isfinite(values)]
    if finite_values.size == 0:
        return float("nan")
    median = float(np.median(finite_values))
    mad = float(np.median(np.abs(finite_values - median)))
    return 1.4826 * mad


def local_peak_indices(score: np.ndarray) -> list[int]:
    peaks: list[int] = []
    for index in range(1, len(score) - 1):
        value = score[index]
        if not math.isfinite(float(value)) or value <= 0.0:
            continue
        previous = score[index - 1] if math.isfinite(float(score[index - 1])) else -math.inf
        next_value = score[index + 1] if math.isfinite(float(score[index + 1])) else -math.inf
        if value + 1e-12 >= previous and value + 1e-12 >= next_value:
            peaks.append(index)
    return peaks


def build_window(
    center_index: int,
    score: np.ndarray,
    time_s: np.ndarray,
    threshold_mps: float,
    cfg: DetectorConfig,
) -> tuple[int, int]:
    center_score = float(score[center_index])
    support_threshold = max(cfg.min_score_mps, cfg.support_ratio * threshold_mps)
    max_half_duration_s = 0.5 * cfg.max_window_duration_s
    start_index = center_index
    end_index = center_index
    while start_index > 0:
        next_start = start_index - 1
        if time_s[center_index] - time_s[next_start] > max_half_duration_s:
            break
        if not math.isfinite(float(score[next_start])) or score[next_start] < support_threshold:
            break
        start_index = next_start
    while end_index + 1 < len(score):
        next_end = end_index + 1
        if time_s[next_end] - time_s[center_index] > max_half_duration_s:
            break
        if not math.isfinite(float(score[next_end])) or score[next_end] < support_threshold:
            break
        end_index = next_end
    if start_index == center_index and center_index > 0:
        start_index -= 1
    if end_index == center_index and center_index + 1 < len(score):
        end_index += 1
    if time_s[end_index] - time_s[start_index] > cfg.max_window_duration_s:
        while start_index < center_index and time_s[end_index] - time_s[start_index] > cfg.max_window_duration_s:
            start_index += 1
        while end_index > center_index and time_s[end_index] - time_s[start_index] > cfg.max_window_duration_s:
            end_index -= 1
    if center_score < support_threshold:
        return center_index, center_index
    return start_index, end_index


def select_direction(
    direction: str,
    sign: float,
    signed_step: np.ndarray,
    time_s: np.ndarray,
    velocity: np.ndarray,
    acceleration: np.ndarray,
    cfg: DetectorConfig,
) -> tuple[list[SelectedWindow], list[str]]:
    score = sign * signed_step
    score[~np.isfinite(score)] = np.nan
    peaks = local_peak_indices(score)
    suppressed = np.zeros(len(score), dtype=bool)
    windows: list[SelectedWindow] = []
    summary: list[str] = []
    direction_sigma = robust_sigma(score[np.isfinite(score) & (score > 0.0)])
    signed_sigma = robust_sigma(signed_step)
    base_noise_floor = max(
        cfg.min_score_mps,
        1.5 * signed_sigma if math.isfinite(signed_sigma) else cfg.min_score_mps,
        1.5 * direction_sigma if math.isfinite(direction_sigma) else cfg.min_score_mps,
    )

    for level in range(1, cfg.max_levels + 1):
        remaining_peaks = [index for index in peaks if not suppressed[index] and math.isfinite(float(score[index]))]
        if not remaining_peaks:
            summary.append(f"stop_{direction}=no separated peaks above direction noise floor")
            break
        peak_values = np.array([score[index] for index in remaining_peaks], dtype=float)
        level_max_peak = float(np.nanmax(peak_values))
        if not math.isfinite(level_max_peak) or level_max_peak < base_noise_floor:
            summary.append(f"stop_{direction}=strongest remaining peak below direction noise floor")
            break
        threshold = max(base_noise_floor, cfg.threshold_ratio * level_max_peak)
        level_peak_indices = [index for index in remaining_peaks if score[index] >= threshold]
        level_peak_indices.sort(key=lambda idx: float(score[idx]), reverse=True)
        ordered_times = np.sort(time_s[level_peak_indices]) if level_peak_indices else np.array([])
        median_gap_s = (
            float(np.median(np.diff(ordered_times))) if ordered_times.size > 1 else float("inf")
        )
        density_hz = float(len(level_peak_indices) / max(time_s[-1] - time_s[0], 1e-9))
        if (
            len(level_peak_indices) >= cfg.dense_peak_count
            and median_gap_s < cfg.dense_gap_s
            and level_max_peak < cfg.dense_peak_floor_ratio * base_noise_floor
        ):
            summary.append(
                f"stop_{direction}=remaining peaks are too dense to distinguish "
                f"(level={level},count={len(level_peak_indices)},median_gap_s={median_gap_s:.6g})"
            )
            break

        selected_this_level = 0
        for center_index in level_peak_indices:
            if suppressed[center_index]:
                continue
            start_index, end_index = build_window(center_index, score, time_s, threshold, cfg)
            center_time_s = time_s[center_index]
            if any(abs(center_time_s - window.center_rel_time_s) < cfg.min_separation_s for window in windows):
                continue
            signed_delta = float(velocity[end_index] - velocity[start_index])
            if direction == "DOWN" and signed_delta >= 0.0:
                continue
            if direction == "UP" and signed_delta <= 0.0:
                continue
            acc_slice = acceleration[start_index : end_index + 1]
            windows.append(
                SelectedWindow(
                    direction=direction,
                    selection_level=level,
                    start_index=start_index,
                    center_index=center_index,
                    end_index=end_index,
                    start_rel_time_s=float(time_s[start_index]),
                    center_rel_time_s=float(time_s[center_index]),
                    end_rel_time_s=float(time_s[end_index]),
                    duration_s=float(time_s[end_index] - time_s[start_index]),
                    pre_velocity_mps=float(velocity[start_index]),
                    post_velocity_mps=float(velocity[end_index]),
                    signed_delta_velocity_mps=signed_delta,
                    direction_score_mps=float(score[center_index]),
                    signed_step_metric_mps=float(signed_step[center_index]),
                    level_threshold_mps=threshold,
                    level_max_peak_mps=level_max_peak,
                    level_noise_floor_mps=base_noise_floor,
                    min_acc_mps2=float(np.nanmin(acc_slice)),
                    max_acc_mps2=float(np.nanmax(acc_slice)),
                    mean_acc_mps2=float(np.nanmean(acc_slice)),
                )
            )
            selected_this_level += 1
            suppress_start_time_s = time_s[start_index] - 0.5 * cfg.min_separation_s
            suppress_end_time_s = time_s[end_index] + 0.5 * cfg.min_separation_s
            suppressed |= (time_s >= suppress_start_time_s) & (time_s <= suppress_end_time_s)

        summary.append(
            f"direction={direction},level={level},threshold={threshold},"
            f"max_peak={level_max_peak},noise_floor={base_noise_floor},"
            f"peak_count={len(level_peak_indices)},selected_count={selected_this_level},"
            f"median_gap_s={median_gap_s},density_hz={density_hz}"
        )
        if selected_this_level == 0:
            summary.append(f"stop_{direction}=no sign-consistent windows selected at level {level}")
            break

    return windows, summary


def write_matlab_script(run_dir: Path, base_name: str, windows_csv_name: str) -> Path:
    script_path = run_dir / f"make_{base_name}_fig.m"
    content = f"""runDir = '{str(run_dir).replace("'", "''")}';
seriesPath = fullfile(runDir, '{base_name}_series.csv');
windowsPath = fullfile(runDir, '{windows_csv_name}');
figPath = fullfile(runDir, '{base_name}.fig');
pngPath = fullfile(runDir, '{base_name}.png');

T = readtable(seriesPath, 'VariableNamingRule', 'preserve');
W = readtable(windowsPath, 'VariableNamingRule', 'preserve');
x = T.relative_time_s;

fig = figure('Name', 'Adaptive bidirectional body-z jump detection', ...
    'Color', 'w', 'Position', [60, 45, 1580, 1040]);
tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

ax1 = nexttile;
hold(ax1, 'on');
plot(ax1, x, T.body_z_acc_mps2, 'Color', [0.74 0.74 0.74], 'LineWidth', 0.45, ...
    'DisplayName', 'body z acceleration');
plot(ax1, x, T.body_z_acc_1s_smooth_mps2, 'Color', [0.86 0.20 0.16], 'LineWidth', 1.1, ...
    'DisplayName', 'body z acceleration, 1 s smooth');
shadeWindows(ax1, W);
grid(ax1, 'on');
yline(ax1, 0.0, '-', 'Color', [0.55 0.55 0.55], 'HandleVisibility', 'off');
ylabel(ax1, 'a_{{bz}} (m/s^2)');
title(ax1, 'Adaptive bidirectional acceleration-window selection');
legend(ax1, 'Location', 'best');

ax2 = nexttile;
hold(ax2, 'on');
plot(ax2, x, T.integrated_body_z_velocity_mps, 'Color', [0.62 0.62 0.62], 'LineWidth', 0.7, ...
    'DisplayName', 'integrated v_{{bz}}');
plot(ax2, x, T.integrated_body_z_velocity_0p2s_smooth_mps, 'Color', [0.10 0.45 0.82], 'LineWidth', 1.1, ...
    'DisplayName', 'integrated v_{{bz}}, 0.2 s smooth');
plot(ax2, x, T.integrated_body_z_velocity_1s_smooth_mps, 'Color', [0.05 0.50 0.22], 'LineWidth', 1.2, ...
    'DisplayName', 'integrated v_{{bz}}, 1 s smooth');
scatterWindows(ax2, W, 'post_velocity_mps');
shadeWindows(ax2, W);
grid(ax2, 'on');
yline(ax2, 0.0, '-', 'Color', [0.55 0.55 0.55], 'HandleVisibility', 'off');
ylabel(ax2, 'v_{{bz}} (m/s)');
title(ax2, 'Integrated velocity and selected jump windows');
legend(ax2, 'Location', 'best');

ax3 = nexttile;
hold(ax3, 'on');
plot(ax3, x, T.signed_step_metric_mps, 'Color', [0.35 0.35 0.35], 'LineWidth', 0.85, ...
    'DisplayName', 'signed step metric: post - pre');
plot(ax3, x, T.downward_score_mps, 'Color', [0.86 0.20 0.16], 'LineWidth', 0.9, ...
    'DisplayName', 'DOWN score = -signed step');
plot(ax3, x, T.upward_score_mps, 'Color', [0.10 0.35 0.82], 'LineWidth', 0.9, ...
    'DisplayName', 'UP score = signed step');
scatterWindows(ax3, W, 'signed_step_metric_mps');
shadeWindows(ax3, W);
grid(ax3, 'on');
yline(ax3, 0.0, '-', 'Color', [0.55 0.55 0.55], 'HandleVisibility', 'off');
ylabel(ax3, 'step metric (m/s)');
xlabel(ax3, 'time since dynamic start (s)');
title(ax3, 'Separate DOWN/UP levels recomputed after selected windows are suppressed');
legend(ax3, 'Location', 'best');

linkaxes([ax1 ax2 ax3], 'x');
savefig(fig, figPath, 'compact');
exportgraphics(fig, pngPath, 'Resolution', 180);

function c = windowColor(direction, level)
if strcmp(direction, 'DOWN')
    base = [0.86 0.20 0.16];
else
    base = [0.10 0.35 0.82];
end
scale = max(0.45, 1.0 - 0.10 * double(level - 1));
c = base * scale + [1 1 1] * (1 - scale) * 0.25;
end

function shadeWindows(ax, W)
if isempty(W) || ~ismember('start_rel_time_s', W.Properties.VariableNames)
    return;
end
y = ylim(ax);
legendAdded = containers.Map('KeyType', 'char', 'ValueType', 'logical');
for i = 1:height(W)
    startT = W.start_rel_time_s(i);
    endT = W.end_rel_time_s(i);
    centerT = W.center_rel_time_s(i);
    direction = string(W.direction(i));
    level = W.selection_level(i);
    color = windowColor(char(direction), level);
    key = sprintf('%s L%d', char(direction), level);
    if isKey(legendAdded, key)
        visibility = 'off';
    else
        visibility = 'on';
        legendAdded(key) = true;
    end
    h = patch(ax, [startT endT endT startT], [y(1) y(1) y(2) y(2)], ...
        color, 'FaceAlpha', 0.16, 'EdgeColor', 'none', ...
        'DisplayName', key + " window", 'HandleVisibility', visibility);
    uistack(h, 'bottom');
    xline(ax, centerT, '--', 'Color', color * 0.75, 'HandleVisibility', 'off');
end
ylim(ax, y);
end

function scatterWindows(ax, W, yName)
if isempty(W) || ~ismember(yName, W.Properties.VariableNames)
    return;
end
for i = 1:height(W)
    direction = char(string(W.direction(i)));
    color = windowColor(direction, W.selection_level(i));
    if strcmp(direction, 'DOWN')
        marker = 'v';
    else
        marker = '^';
    end
    scatter(ax, W.center_rel_time_s(i), W.(yName)(i), 42, color, marker, 'filled', ...
        'HandleVisibility', 'off');
end
end
"""
    script_path.write_text(content, encoding="utf-8")
    return script_path


def run_matlab(script_path: Path, matlab_exe: str | None) -> None:
    if not matlab_exe:
        return
    subprocess.run(
        [matlab_exe, "-batch", f"run('{str(script_path).replace(chr(92), '/')}')"],
        check=True,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument(
        "--input",
        default="current_body_z_acc_integrated_smoothing.csv",
        help="input CSV path relative to run-dir unless absolute",
    )
    parser.add_argument(
        "--base-name",
        default="body_z_bidirectional_jump_detection_adaptive",
        help="output basename",
    )
    parser.add_argument(
        "--windows-csv",
        default="body_z_bidirectional_jump_windows_adaptive.csv",
        help="window output CSV path relative to run-dir unless absolute",
    )
    parser.add_argument("--matlab", default=os.environ.get("MATLAB_EXE"))
    args = parser.parse_args()

    run_dir = args.run_dir.resolve()
    input_path = Path(args.input)
    if not input_path.is_absolute():
        input_path = run_dir / input_path
    source = pd.read_csv(input_path)
    cfg = DetectorConfig()
    time_s = source["relative_time_s"].to_numpy(dtype=float)
    dt_s = float(np.nanmedian(np.diff(time_s)))
    acceleration = source["body_z_acc_mps2"].to_numpy(dtype=float)
    acceleration_1s = source["body_z_acc_1s_smooth_mps2"].to_numpy(dtype=float)
    velocity = source["integrated_body_z_velocity_mps"].to_numpy(dtype=float)
    velocity_1s = source["integrated_body_z_velocity_1s_smooth_mps"].to_numpy(dtype=float)
    velocity_0p2 = centered_mean(velocity, cfg.velocity_smooth_s, dt_s)
    signed_step = centered_step_metric(velocity_0p2, dt_s, cfg)
    downward_score = np.where(np.isfinite(signed_step), np.maximum(-signed_step, 0.0), np.nan)
    upward_score = np.where(np.isfinite(signed_step), np.maximum(signed_step, 0.0), np.nan)
    sigma_signed = robust_sigma(signed_step)

    down_windows, down_summary = select_direction(
        "DOWN",
        -1.0,
        signed_step.copy(),
        time_s,
        velocity,
        acceleration,
        cfg,
    )
    up_windows, up_summary = select_direction(
        "UP",
        1.0,
        signed_step.copy(),
        time_s,
        velocity,
        acceleration,
        cfg,
    )
    windows = sorted(down_windows + up_windows, key=lambda window: window.center_rel_time_s)

    # Downsample the dense 1 kHz series for plotting while keeping every 10 ms.
    stride = max(1, int(round(0.01 / max(dt_s, 1e-9))))
    series = pd.DataFrame(
        {
            "relative_time_s": time_s[::stride],
            "body_z_acc_mps2": acceleration[::stride],
            "body_z_acc_1s_smooth_mps2": acceleration_1s[::stride],
            "integrated_body_z_velocity_mps": velocity[::stride],
            "integrated_body_z_velocity_0p2s_smooth_mps": velocity_0p2[::stride],
            "integrated_body_z_velocity_1s_smooth_mps": velocity_1s[::stride],
            "signed_step_metric_mps": signed_step[::stride],
            "downward_score_mps": downward_score[::stride],
            "upward_score_mps": upward_score[::stride],
            "robust_sigma_signed_mps": np.full_like(time_s[::stride], sigma_signed),
        }
    )
    windows_table = pd.DataFrame([window.__dict__ for window in windows])

    series_path = run_dir / f"{args.base_name}_series.csv"
    windows_path = Path(args.windows_csv)
    if not windows_path.is_absolute():
        windows_path = run_dir / windows_path
    summary_path = run_dir / f"{args.base_name}_summary.txt"
    series.to_csv(series_path, index=False, float_format="%.12g")
    windows_table.to_csv(windows_path, index=False, float_format="%.12g")
    summary_lines = [
        f"dt_s={dt_s}",
        f"robust_sigma_signed_mps={sigma_signed}",
        f"detector=min_score_mps={cfg.min_score_mps},threshold_ratio={cfg.threshold_ratio},"
        f"support_ratio={cfg.support_ratio},min_separation_s={cfg.min_separation_s}",
        "process=DOWN and UP are detected independently; each level recomputes its threshold "
        "from the strongest remaining same-direction peak after selected windows are suppressed",
    ]
    summary_lines.extend(down_summary)
    summary_lines.extend(up_summary)
    summary_lines.append(f"selected_total={len(windows)}")
    summary_lines.append(f"selected_down={sum(1 for window in windows if window.direction == 'DOWN')}")
    summary_lines.append(f"selected_up={sum(1 for window in windows if window.direction == 'UP')}")
    summary_path.write_text("\n".join(summary_lines) + "\n", encoding="utf-8")

    matlab_windows_name = windows_path.name if windows_path.parent == run_dir else str(windows_path)
    matlab_script = write_matlab_script(run_dir, args.base_name, matlab_windows_name)
    run_matlab(matlab_script, args.matlab)

    print(f"wrote {series_path}")
    print(f"wrote {windows_path}")
    print(f"wrote {summary_path}")
    print(f"wrote {matlab_script}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
