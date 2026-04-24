function plot_vertical_state_acceleration_matlab(csv_path, fig_path, png_path)
%PLOT_VERTICAL_STATE_ACCELERATION_MATLAB Plot derived vertical acceleration from state corrections.

if nargin < 1 || strlength(string(csv_path)) == 0
    error('csv_path is required');
end

csv_path = string(csv_path);
if nargin < 2 || strlength(string(fig_path)) == 0
    fig_path = replace(csv_path, ".csv", "_az.fig");
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

time_s = rows.state_time_s;
time_rel_s = time_s - time_s(1);
reference_az_mps2 = gradient(rows.reference_vz_mps, time_s);
optimized_az_mps2 = gradient(rows.optimized_vz_mps, time_s);
delta_az_mps2 = optimized_az_mps2 - reference_az_mps2;

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100, 100, 1400, 900]);
tiled = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tiled, 'Vertical Acceleration Derived From State Corrections', 'Interpreter', 'none');

nexttile;
plot(time_rel_s, reference_az_mps2, 'b--', 'LineWidth', 1.0); hold on;
plot(time_rel_s, optimized_az_mps2, 'r-', 'LineWidth', 1.1);
grid on;
xlabel('Relative time (s)');
ylabel('a_z (m/s^2)');
legend({'Reference a_z', 'Optimized a_z'}, 'Location', 'best');
title('Derived Vertical Acceleration');

nexttile;
plot(time_rel_s, delta_az_mps2, 'k-', 'LineWidth', 1.0);
grid on;
xlabel('Relative time (s)');
ylabel('\Delta a_z (m/s^2)');
legend({'\Delta a_z = optimized - reference'}, 'Location', 'best');
title('Acceleration Correction');

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
fprintf('Note: a_z is derived from gradient(v_z, time), not raw IMU specific force.\n');
end
