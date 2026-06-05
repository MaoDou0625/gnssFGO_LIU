function output_paths = plot_attitude_comparison_no_bodyz_visible(run_dir, gnss_path, output_base, title_text)
%PLOT_ATTITUDE_COMPARISON_NO_BODYZ_VISIBLE Plot attitude references without body-z seed.
%
% output_paths = plot_attitude_comparison_no_bodyz_visible(run_dir, gnss_path)
% output_paths = plot_attitude_comparison_no_bodyz_visible(run_dir, gnss_path, output_base, title_text)
%
% The saved .fig intentionally keeps Visible='on' so reopening the artifact
% does not produce a hidden MATLAB figure.

if nargin < 3 || isempty(output_base)
    output_base = fullfile(run_dir, "attitude_comparison_no_bodyz_visible");
end
if nargin < 4 || isempty(title_text)
    title_text = "Attitude comparison without body-z seed";
end

run_dir = char(run_dir);
gnss_path = char(gnss_path);
output_base = char(output_base);
title_text = char(title_text);

debug_path = fullfile(run_dir, "stage_attitude_debug_trajectory.csv");
trajectory_path = fullfile(run_dir, "trajectory.csv");
outage_path = fullfile(run_dir, "rtk_outage_windows.csv");

debug_table = readtable(debug_path, "TextType", "string", "VariableNamingRule", "preserve");
trajectory_table = readtable(trajectory_path, "TextType", "string", "VariableNamingRule", "preserve");

sources = string(debug_table.("source"));
base_graph = make_series(debug_table(sources == "base_graph_optimized", :));
stage2_anchor = make_series(debug_table(sources == "stage2_anchor_imu_delta", :));
final_graph = make_series(trajectory_table);
outage_windows = read_outage_windows(outage_path);
rtk_heading = build_rtk_heading(gnss_path);

[rtk_time, rtk_yaw_deg] = aligned_rtk_heading(rtk_heading, base_graph);
[base_err_time, base_err_deg, base_stats] = yaw_error_stats(base_graph, rtk_heading, "base graph optimized");
[stage2_err_time, stage2_err_deg, stage2_stats] = yaw_error_stats(stage2_anchor, rtk_heading, "stage2 anchor imu delta");
[final_err_time, final_err_deg, final_stats] = yaw_error_stats(final_graph, rtk_heading, "graph with gnss final");

fig = figure("Visible", "on", "Color", "white", "Position", [80, 80, 1700, 1100]);
layout = tiledlayout(fig, 4, 1, "TileSpacing", "compact", "Padding", "compact");
title(layout, title_text, "Interpreter", "none");

colors.base = [0.1216, 0.4667, 0.7059];
colors.stage2 = [0.1725, 0.6275, 0.1725];
colors.final = [0.8392, 0.1529, 0.1569];
colors.rtk = [0.05, 0.05, 0.05];

axes_handles = gobjects(4, 1);

axes_handles(1) = nexttile(layout);
hold(axes_handles(1), "on");
plot(axes_handles(1), base_graph.time_s, base_graph.yaw_deg, "LineWidth", 1.15, "Color", colors.base, "DisplayName", "base graph optimized");
plot(axes_handles(1), stage2_anchor.time_s, stage2_anchor.yaw_deg, "LineWidth", 1.15, "Color", colors.stage2, "DisplayName", "stage2 anchor imu delta");
plot(axes_handles(1), final_graph.time_s, final_graph.yaw_deg, "LineWidth", 1.35, "Color", colors.final, "DisplayName", "graph with gnss final");
if ~isempty(rtk_time)
    scatter(axes_handles(1), rtk_time, rtk_yaw_deg, 10, colors.rtk, "filled", "MarkerFaceAlpha", 0.45, "DisplayName", "RTKFIX diff heading");
end
ylabel(axes_handles(1), "yaw [deg]");
grid(axes_handles(1), "on");
legend(axes_handles(1), "Location", "best", "FontSize", 8);

axes_handles(2) = nexttile(layout);
hold(axes_handles(2), "on");
plot(axes_handles(2), base_graph.time_s, base_graph.pitch_deg, "LineWidth", 1.15, "Color", colors.base, "DisplayName", "base graph optimized");
plot(axes_handles(2), stage2_anchor.time_s, stage2_anchor.pitch_deg, "LineWidth", 1.15, "Color", colors.stage2, "DisplayName", "stage2 anchor imu delta");
plot(axes_handles(2), final_graph.time_s, final_graph.pitch_deg, "LineWidth", 1.35, "Color", colors.final, "DisplayName", "graph with gnss final");
ylabel(axes_handles(2), "pitch [deg]");
grid(axes_handles(2), "on");

axes_handles(3) = nexttile(layout);
hold(axes_handles(3), "on");
plot(axes_handles(3), base_graph.time_s, base_graph.roll_deg, "LineWidth", 1.15, "Color", colors.base, "DisplayName", "base graph optimized");
plot(axes_handles(3), stage2_anchor.time_s, stage2_anchor.roll_deg, "LineWidth", 1.15, "Color", colors.stage2, "DisplayName", "stage2 anchor imu delta");
plot(axes_handles(3), final_graph.time_s, final_graph.roll_deg, "LineWidth", 1.35, "Color", colors.final, "DisplayName", "graph with gnss final");
ylabel(axes_handles(3), "roll [deg]");
grid(axes_handles(3), "on");

axes_handles(4) = nexttile(layout);
hold(axes_handles(4), "on");
plot(axes_handles(4), base_err_time, base_err_deg, "LineWidth", 1.0, "Color", colors.base, "DisplayName", "base graph optimized");
plot(axes_handles(4), stage2_err_time, stage2_err_deg, "LineWidth", 1.0, "Color", colors.stage2, "DisplayName", "stage2 anchor imu delta");
plot(axes_handles(4), final_err_time, final_err_deg, "LineWidth", 1.0, "Color", colors.final, "DisplayName", "graph with gnss final");
yline(axes_handles(4), 0.0, "Color", [0, 0, 0], "LineWidth", 0.8, "Alpha", 0.6, "HandleVisibility", "off");
ylabel(axes_handles(4), "yaw - RTK [deg]");
xlabel(axes_handles(4), "time [s]");
grid(axes_handles(4), "on");
legend(axes_handles(4), "Location", "best", "FontSize", 8);

stats_lines = [string(base_stats); string(stage2_stats); string(final_stats)];
stats_text = strjoin(stats_lines, newline);
text(axes_handles(4), 0.01, 0.98, stats_text, "Units", "normalized", ...
    "VerticalAlignment", "top", "HorizontalAlignment", "left", "FontSize", 8, ...
    "BackgroundColor", [1, 1, 1], "EdgeColor", [0.55, 0.55, 0.55], "Margin", 4);

for idx = 1:numel(axes_handles)
    add_outage_shading(axes_handles(idx), outage_windows);
end
linkaxes(axes_handles, "x");

fig_path = string(output_base) + ".fig";
png_path = string(output_base) + ".png";
set(fig, "Visible", "on");
drawnow;
savefig(fig, char(fig_path));
exportgraphics(fig, char(png_path), "Resolution", 180);

output_paths = struct("fig", char(fig_path), "png", char(png_path));
fprintf("fig_saved=%s\n", fig_path);
fprintf("png_saved=%s\n", png_path);
fprintf("%s\n", stats_text);

end

function series = make_series(data_table)
time_s = double(data_table.("time_s"));
yaw_rad = double(data_table.("yaw_rad"));
pitch_rad = double(data_table.("pitch_rad"));
roll_rad = double(data_table.("roll_rad"));
finite_mask = isfinite(time_s) & isfinite(yaw_rad) & isfinite(pitch_rad) & isfinite(roll_rad);
time_s = time_s(finite_mask);
yaw_rad = yaw_rad(finite_mask);
pitch_rad = pitch_rad(finite_mask);
roll_rad = roll_rad(finite_mask);
[time_s, order] = sort(time_s);
yaw_rad = yaw_rad(order);
pitch_rad = pitch_rad(order);
roll_rad = roll_rad(order);

series = struct();
series.time_s = time_s;
series.raw_yaw_deg = rad2deg(yaw_rad);
series.yaw_deg = rad2deg(unwrap(yaw_rad));
series.pitch_deg = rad2deg(pitch_rad);
series.roll_deg = rad2deg(roll_rad);
end

function windows = read_outage_windows(outage_path)
if ~isfile(outage_path)
    windows = zeros(0, 2);
    return;
end
data_table = readtable(outage_path, "TextType", "string", "VariableNamingRule", "preserve");
start_s = double(data_table.("start_time_s"));
end_s = double(data_table.("end_time_s"));
valid = isfinite(start_s) & isfinite(end_s) & end_s > start_s;
windows = [start_s(valid), end_s(valid)];
end

function rtk = build_rtk_heading(gnss_path)
raw = readmatrix(gnss_path, "FileType", "text", "Delimiter", "\t");
origin_mask = all(isfinite(raw(:, 1:4)), 2);
origin_index = find(origin_mask, 1, "first");
if isempty(origin_index)
    error("Unable to determine GNSS origin from %s", gnss_path);
end
origin_lat = raw(origin_index, 2);
origin_lon = raw(origin_index, 3);
origin_h = raw(origin_index, 4);

fix_mask = raw(:, 13) == 1 & all(isfinite(raw(:, 1:4)), 2);
fix_rows = raw(fix_mask, :);
if isempty(fix_rows)
    error("No RTKFIX rows found in %s", gnss_path);
end

[time_s, order] = sort(fix_rows(:, 1));
fix_rows = fix_rows(order, :);
[east_m, north_m] = llh_to_enu(fix_rows(:, 2), fix_rows(:, 3), fix_rows(:, 4), origin_lat, origin_lon, origin_h);

half_window_s = 0.5;
min_displacement_m = 0.2;
heading_deg = nan(size(time_s));
window_displacement_m = nan(size(time_s));
valid_heading = false(size(time_s));

for idx = 1:numel(time_s)
    left_target = time_s(idx) - half_window_s;
    right_target = time_s(idx) + half_window_s;
    if left_target < time_s(1) || right_target > time_s(end)
        continue;
    end
    left_idx = find(time_s <= left_target, 1, "last");
    right_idx = find(time_s >= right_target, 1, "first");
    if isempty(left_idx) || isempty(right_idx) || left_idx >= idx || right_idx <= idx || left_idx >= right_idx
        continue;
    end

    delta_east_m = east_m(right_idx) - east_m(left_idx);
    delta_north_m = north_m(right_idx) - north_m(left_idx);
    displacement_m = hypot(delta_east_m, delta_north_m);
    window_displacement_m(idx) = displacement_m;
    if displacement_m < min_displacement_m
        continue;
    end
    heading_deg(idx) = atan2d(delta_north_m, delta_east_m);
    valid_heading(idx) = true;
end

rtk = struct();
rtk.time_s = time_s;
rtk.heading_deg = heading_deg;
rtk.window_displacement_m = window_displacement_m;
rtk.valid_heading = valid_heading;
end

function [east_m, north_m] = llh_to_enu(lat_rad, lon_rad, h_m, origin_lat_rad, origin_lon_rad, origin_h_m)
[x, y, z] = ecef_from_llh(lat_rad, lon_rad, h_m);
[x0, y0, z0] = ecef_from_llh(origin_lat_rad, origin_lon_rad, origin_h_m);
dx = x - x0;
dy = y - y0;
dz = z - z0;

sin_lat0 = sin(origin_lat_rad);
cos_lat0 = cos(origin_lat_rad);
sin_lon0 = sin(origin_lon_rad);
cos_lon0 = cos(origin_lon_rad);

east_m = -sin_lon0 .* dx + cos_lon0 .* dy;
north_m = -sin_lat0 .* cos_lon0 .* dx - sin_lat0 .* sin_lon0 .* dy + cos_lat0 .* dz;
end

function [x, y, z] = ecef_from_llh(lat_rad, lon_rad, h_m)
wgs84_a = 6378137.0;
wgs84_f = 1.0 / 298.257223563;
wgs84_e2 = wgs84_f * (2.0 - wgs84_f);
sin_lat = sin(lat_rad);
cos_lat = cos(lat_rad);
sin_lon = sin(lon_rad);
cos_lon = cos(lon_rad);
radius = wgs84_a ./ sqrt(1.0 - wgs84_e2 .* sin_lat .* sin_lat);
x = (radius + h_m) .* cos_lat .* cos_lon;
y = (radius + h_m) .* cos_lat .* sin_lon;
z = (radius .* (1.0 - wgs84_e2) + h_m) .* sin_lat;
end

function [rtk_time, rtk_yaw_deg] = aligned_rtk_heading(rtk, reference_series)
valid = rtk.valid_heading & isfinite(rtk.heading_deg);
rtk_time = rtk.time_s(valid);
if isempty(rtk_time)
    rtk_yaw_deg = [];
    return;
end
rtk_yaw_deg = rad2deg(unwrap(deg2rad(rtk.heading_deg(valid))));
reference_yaw_deg = interp1(reference_series.time_s, reference_series.yaw_deg, rtk_time, "linear", "extrap");
offset = reference_yaw_deg - rtk_yaw_deg;
offset = offset(isfinite(offset));
if ~isempty(offset)
    rtk_yaw_deg = rtk_yaw_deg + 360.0 * round(median(offset) / 360.0);
end
end

function [err_time, err_deg, stats_line] = yaw_error_stats(series, rtk, label)
tolerance_s = 0.26;
valid_time = rtk.time_s(rtk.valid_heading & isfinite(rtk.heading_deg));
valid_heading = rtk.heading_deg(rtk.valid_heading & isfinite(rtk.heading_deg));
err_time = [];
err_deg = [];

for idx = 1:numel(valid_time)
    [delta_s, nearest_idx] = min(abs(series.time_s - valid_time(idx)));
    if isempty(nearest_idx) || delta_s > tolerance_s
        continue;
    end
    err_time(end + 1, 1) = valid_time(idx); %#ok<AGROW>
    err_deg(end + 1, 1) = wrap_deg(series.raw_yaw_deg(nearest_idx) - valid_heading(idx)); %#ok<AGROW>
end

if isempty(err_deg)
    stats_line = label + ": no RTK match";
    return;
end

abs_err = abs(err_deg);
sorted_abs = sort(abs_err);
p95_index = max(1, min(numel(sorted_abs), ceil(0.95 * numel(sorted_abs))));
p95 = sorted_abs(p95_index);
rms_value = sqrt(mean(err_deg .* err_deg));
stats_line = sprintf("%s: N=%d, mean_abs=%.3f deg, rms=%.3f, p95=%.3f, max=%.3f, median=%.3f", ...
    label, numel(err_deg), mean(abs_err), rms_value, p95, max(abs_err), median(err_deg));
end

function wrapped = wrap_deg(angle_deg)
wrapped = mod(angle_deg + 180.0, 360.0) - 180.0;
wrapped(wrapped == -180.0) = 180.0;
end

function add_outage_shading(axis_handle, windows)
if isempty(windows)
    return;
end
yl = ylim(axis_handle);
for idx = 1:size(windows, 1)
    patch_handle = patch(axis_handle, ...
        [windows(idx, 1), windows(idx, 2), windows(idx, 2), windows(idx, 1)], ...
        [yl(1), yl(1), yl(2), yl(2)], ...
        [0.88, 0.88, 0.88], ...
        "EdgeColor", "none", ...
        "FaceAlpha", 0.55, ...
        "HandleVisibility", "off");
    uistack(patch_handle, "bottom");
end
ylim(axis_handle, yl);
end
