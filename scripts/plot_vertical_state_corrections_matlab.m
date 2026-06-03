function plot_vertical_state_corrections_matlab(csv_path, fig_path, png_path)
%PLOT_VERTICAL_STATE_CORRECTIONS_MATLAB Plot per-node vertical corrections and save a MATLAB .fig.

if nargin < 1 || strlength(string(csv_path)) == 0
    error('csv_path is required');
end

csv_path = string(csv_path);
if nargin < 2 || strlength(string(fig_path)) == 0
    fig_path = replace(csv_path, ".csv", ".fig");
end
if nargin < 3 || strlength(string(png_path)) == 0
    png_path = replace(string(fig_path), ".fig", ".png");
end

fig_path = string(fig_path);
png_path = string(png_path);

rows = readtable(csv_path, 'PreserveVariableNames', true);
if isempty(rows)
    error('No rows found in %s', csv_path);
end

time_rel_s = rows.corrected_time_s - rows.corrected_time_s(1);
optimized_pitch_deg = rad2deg(rows.optimized_pitch_rad);
optimized_roll_deg = rad2deg(rows.optimized_roll_rad);

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80, 80, 1500, 980]);
tiled = tiledlayout(fig, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tiled, 'Vertical State Corrections', 'Interpreter', 'none');

nexttile;
plot(time_rel_s, rows.measurement_up_m, 'k-', 'LineWidth', 1.0); hold on;
plot(time_rel_s, rows.optimized_up_m, 'r-', 'LineWidth', 1.1);
grid on;
xlabel('Relative time (s)');
ylabel('Up (m)');
legend({'RTK up', 'Optimized up'}, 'Location', 'best');
title('Up State at GNSS measurement time');

nexttile;
plot(time_rel_s, rows.postfit_residual_u_m, 'Color', [0.49, 0.18, 0.56], 'LineWidth', 1.0);
grid on;
xlabel('Relative time (s)');
ylabel('Vertical position (m)');
legend({'Postfit residual u'}, 'Location', 'best');
title('Position Residual');

nexttile;
plot(time_rel_s, rows.optimized_vz_mps, 'r-', 'LineWidth', 1.1);
grid on;
xlabel('Relative time (s)');
ylabel('Vz (m/s)');
legend({'Optimized vz'}, 'Location', 'best');
title('Vertical Velocity');

nexttile;
plot(time_rel_s, optimized_pitch_deg, 'b-', 'LineWidth', 1.1); hold on;
plot(time_rel_s, optimized_roll_deg, 'r-', 'LineWidth', 1.1);
grid on;
xlabel('Relative time (s)');
ylabel('Angle (deg)');
legend({'Optimized pitch', 'Optimized roll'}, 'Location', 'best');
title('Attitude State');

nexttile;
plot(time_rel_s, rows.optimized_baz_ug, 'r-', 'LineWidth', 1.1);
ylabel('ba_z (ug)');
grid on;
xlabel('Relative time (s)');
legend({'Optimized ba_z'}, 'Location', 'best');
title('Accelerometer Bias');

nexttile;
stairs(time_rel_s, double(rows.vertical_direct_position_factor_used), 'r-', 'LineWidth', 1.1);
ylim([-0.1, 1.1]);
grid on;
xlabel('Relative time (s)');
ylabel('Flag');
legend({'Vertical direct position factor'}, 'Location', 'best');
title('Factor Mode');

savefig(fig, fig_path);
exportgraphics(fig, png_path, 'Resolution', 180);
close(fig);

fprintf('Saved figure to %s\n', fig_path);
fprintf('Saved preview to %s\n', png_path);
end
