function plot_raw_imu_nav_vertical_acceleration_matlab(run_dir, fig_path, png_path, csv_path)
%PLOT_RAW_IMU_NAV_VERTICAL_ACCELERATION_MATLAB
% Plot raw IMU transformed to navigation-frame vertical acceleration using
% optimized trajectory pitch/roll and accelerometer bias.

if nargin < 1 || strlength(string(run_dir)) == 0
    error('run_dir is required');
end

run_dir = string(run_dir);
if nargin < 2 || strlength(string(fig_path)) == 0
    fig_path = fullfile(run_dir, "raw_imu_nav_vertical_acceleration.fig");
end
if nargin < 3 || strlength(string(png_path)) == 0
    png_path = replace(string(fig_path), ".fig", ".png");
end
if nargin < 4 || strlength(string(csv_path)) == 0
    csv_path = replace(string(fig_path), ".fig", ".csv");
end

config_path = fullfile(run_dir, "config_snapshot.cfg");
trajectory_path = fullfile(run_dir, "trajectory.csv");

config_text = fileread(config_path);
imu_path = localConvertWslPath(localReadConfigValue(config_text, "imu_path"));
gravity_mps2 = str2double(localReadConfigValue(config_text, "gravity_mps2"));

trajectory = readtable(trajectory_path, 'PreserveVariableNames', true);
imu_data = readmatrix(imu_path, 'FileType', 'text');
if size(imu_data, 2) < 7
    error('Unexpected IMU format in %s', imu_path);
end

imu_time_s = imu_data(:, 1);
imu_accel_x = imu_data(:, 5);
imu_accel_y = imu_data(:, 6);
imu_accel_z = imu_data(:, 7);

time_min = trajectory.time_s(1);
time_max = trajectory.time_s(end);
valid = imu_time_s >= time_min & imu_time_s <= time_max;
imu_time_s = imu_time_s(valid);
imu_accel_x = imu_accel_x(valid);
imu_accel_y = imu_accel_y(valid);
imu_accel_z = imu_accel_z(valid);

pitch_rad = interp1(trajectory.time_s, trajectory.pitch_rad, imu_time_s, 'linear', 'extrap');
roll_rad = interp1(trajectory.time_s, trajectory.roll_rad, imu_time_s, 'linear', 'extrap');
bax = interp1(trajectory.time_s, trajectory.bax, imu_time_s, 'linear', 'extrap');
bay = interp1(trajectory.time_s, trajectory.bay, imu_time_s, 'linear', 'extrap');
baz = interp1(trajectory.time_s, trajectory.baz, imu_time_s, 'linear', 'extrap');

corrected_ax = imu_accel_x - bax;
corrected_ay = imu_accel_y - bay;
corrected_az = imu_accel_z - baz;

nav_specific_force_z = ...
    -sin(pitch_rad) .* corrected_ax + ...
    cos(pitch_rad) .* sin(roll_rad) .* corrected_ay + ...
    cos(pitch_rad) .* cos(roll_rad) .* corrected_az;
nav_acc_z = nav_specific_force_z - gravity_mps2;
integrated_nav_acc_z_mps = cumtrapz(imu_time_s, nav_acc_z);

time_rel_s = imu_time_s - imu_time_s(1);

output = table( ...
    imu_time_s, time_rel_s, ...
    imu_accel_x, imu_accel_y, imu_accel_z, ...
    bax, bay, baz, pitch_rad, roll_rad, ...
    corrected_ax, corrected_ay, corrected_az, ...
    nav_specific_force_z, nav_acc_z, integrated_nav_acc_z_mps, ...
    'VariableNames', { ...
        'time_s', 'relative_time_s', ...
        'raw_accel_x_mps2', 'raw_accel_y_mps2', 'raw_accel_z_mps2', ...
        'bax_mps2', 'bay_mps2', 'baz_mps2', 'pitch_rad', 'roll_rad', ...
        'corrected_accel_x_mps2', 'corrected_accel_y_mps2', 'corrected_accel_z_mps2', ...
        'nav_specific_force_z_mps2', 'nav_acc_z_mps2', 'integrated_nav_acc_z_mps'});
writetable(output, csv_path);

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100, 100, 1500, 980]);
tiled = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tiled, 'Raw IMU to Navigation-Frame Vertical Acceleration', 'Interpreter', 'none');

nexttile;
plot(time_rel_s, nav_specific_force_z, 'b-', 'LineWidth', 0.9); hold on;
yline(gravity_mps2, 'k--', 'LineWidth', 1.0);
grid on;
xlabel('Relative time (s)');
ylabel('Specific force z (m/s^2)');
legend({'nav specific force z', 'gravity'}, 'Location', 'best');
title('Full Duration Specific Force');

nexttile;
plot(time_rel_s, nav_acc_z, 'r-', 'LineWidth', 0.9);
grid on;
xlabel('Relative time (s)');
ylabel('a_z (m/s^2)');
legend({'nav acceleration z'}, 'Location', 'best');
title('Full Duration Vertical Acceleration');

nexttile;
plot(time_rel_s, integrated_nav_acc_z_mps, 'k-', 'LineWidth', 1.0);
grid on;
xlabel('Relative time (s)');
ylabel('\int a_z dt (m/s)');
legend({'integrated nav acceleration z'}, 'Location', 'best');
title('Integral of Vertical Acceleration');

savefig(fig, fig_path);
exportgraphics(fig, png_path, 'Resolution', 180);
close(fig);

fprintf('Saved figure to %s\n', fig_path);
fprintf('Saved preview to %s\n', png_path);
fprintf('Saved data to %s\n', csv_path);
fprintf('Formula: nav_acc_z = (R_nb * (acc_raw - b_a))_z - g\n');
end

function value = localReadConfigValue(config_text, key)
lines = splitlines(string(config_text));
needle = string(key) + "=";
value = "";
for index = 1:numel(lines)
    line = strtrim(lines(index));
    if startsWith(line, needle)
        value = strtrim(extractAfter(line, strlength(needle)));
        break;
    end
end
if strlength(value) == 0
    error("Missing key %s in config", key);
end
end

function path_out = localConvertWslPath(path_in)
path_in = string(path_in);
prefix = "/mnt/";
if startsWith(path_in, prefix)
    drive_letter = extractBetween(path_in, strlength(prefix) + 1, strlength(prefix) + 1);
    rest = extractAfter(path_in, strlength(prefix) + 2);
    path_out = upper(drive_letter) + ":/" + rest;
else
    path_out = path_in;
end
path_out = replace(path_out, "\", "/");
end
