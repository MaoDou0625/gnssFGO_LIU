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
delta_pitch_deg = rad2deg(rows.delta_pitch_rad);
delta_roll_deg = rad2deg(rows.delta_roll_rad);
reference_pitch_deg = rad2deg(rows.reference_pitch_rad);
optimized_pitch_deg = rad2deg(rows.optimized_pitch_rad);
reference_roll_deg = rad2deg(rows.reference_roll_rad);
optimized_roll_deg = rad2deg(rows.optimized_roll_rad);

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80, 80, 1500, 980]);
tiled = tiledlayout(fig, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tiled, 'Vertical State Corrections', 'Interpreter', 'none');

nexttile;
plot(time_rel_s, rows.measurement_up_m, 'k-', 'LineWidth', 1.0); hold on;
plot(time_rel_s, rows.reference_up_m, 'b--', 'LineWidth', 1.0);
plot(time_rel_s, rows.optimized_up_m, 'r-', 'LineWidth', 1.1);
grid on;
xlabel('Relative time (s)');
ylabel('Up (m)');
legend({'RTK up', 'Reference up', 'Optimized up'}, 'Location', 'best');
title('Up State at GNSS measurement time');

nexttile;
plot(time_rel_s, rows.delta_up_m, 'b-', 'LineWidth', 1.1); hold on;
plot(time_rel_s, rows.prefit_residual_u_m, 'Color', [0.85, 0.33, 0.10], 'LineStyle', '--', 'LineWidth', 1.0);
plot(time_rel_s, rows.postfit_residual_u_m, 'Color', [0.49, 0.18, 0.56], 'LineWidth', 1.0);
grid on;
xlabel('Relative time (s)');
ylabel('Vertical position (m)');
legend({'\Delta up', 'Prefit residual u', 'Postfit residual u'}, 'Location', 'best');
title('Position Correction');

nexttile;
plot(time_rel_s, rows.reference_vz_mps, 'b--', 'LineWidth', 1.0); hold on;
plot(time_rel_s, rows.optimized_vz_mps, 'r-', 'LineWidth', 1.1);
plot(time_rel_s, rows.delta_vz_mps, 'k-', 'LineWidth', 0.9);
grid on;
xlabel('Relative time (s)');
ylabel('Vz (m/s)');
legend({'Reference vz', 'Optimized vz', '\Delta vz'}, 'Location', 'best');
title('Velocity Correction');

nexttile;
plot(time_rel_s, reference_pitch_deg, 'b--', 'LineWidth', 1.0); hold on;
plot(time_rel_s, optimized_pitch_deg, 'b-', 'LineWidth', 1.1);
plot(time_rel_s, reference_roll_deg, 'r--', 'LineWidth', 1.0);
plot(time_rel_s, optimized_roll_deg, 'r-', 'LineWidth', 1.1);
grid on;
xlabel('Relative time (s)');
ylabel('Angle (deg)');
legend({'Reference pitch', 'Optimized pitch', 'Reference roll', 'Optimized roll'}, 'Location', 'best');
title('Attitude State');

nexttile;
yyaxis left;
plot(time_rel_s, rows.reference_baz_mps2, 'b--', 'LineWidth', 1.0); hold on;
plot(time_rel_s, rows.optimized_baz_mps2, 'r-', 'LineWidth', 1.1);
ylabel('ba_z (m/s^2)');
yyaxis right;
plot(time_rel_s, rows.delta_baz_mps2, 'k-', 'LineWidth', 1.0);
ylabel('\Delta ba_z (m/s^2)');
grid on;
xlabel('Relative time (s)');
legend({'Reference ba_z', 'Optimized ba_z', '\Delta ba_z'}, 'Location', 'best');
title('Accelerometer Bias Correction');

nexttile;
stairs(time_rel_s, rows.vertical_gate_inside, 'b-', 'LineWidth', 1.1); hold on;
stairs(time_rel_s, double(rows.vertical_direct_position_factor_used), 'r-', 'LineWidth', 1.1);
ylim([-0.1, 1.1]);
grid on;
xlabel('Relative time (s)');
ylabel('Flag');
legend({'Inside gate', 'Vertical direct position factor'}, 'Location', 'best');
title('Gate / Factor Mode');

savefig(fig, fig_path);
exportgraphics(fig, png_path, 'Resolution', 180);
close(fig);

fprintf('Saved figure to %s\n', fig_path);
fprintf('Saved preview to %s\n', png_path);
end
