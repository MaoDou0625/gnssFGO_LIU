runDir = 'D:\Code\offline_lc_minimal\runs\transformed1cut1_vertical_rtk_preintegration_feedback_z0200';

gnssPath = fullfile(runDir, 'gnss_consistency.csv');
alignPath = fullfile(runDir, 'gnss_alignment.csv');
refPath = fullfile(runDir, 'reference_node_trajectory.csv');
bodyZPath = fullfile(runDir, 'body_z_seed_jump_windows.csv');
iterPath = fullfile(runDir, 'vertical_local_recovery_iterations.csv');
stateCorrectionPath = fullfile(runDir, 'vertical_state_corrections.csv');
summaryPath = fullfile(runDir, 'summary.txt');
figPath = fullfile(runDir, 'current_full_local_body_z_windows.fig');
pngPath = fullfile(runDir, 'current_full_local_body_z_windows.png');

summaryText = fileread(summaryPath);
token = regexp(summaryText, 'dynamic_start_time_s=([^\r\n]+)', 'tokens', 'once');
dynamicStartTimeS = str2double(token{1});

G = readtable(gnssPath, 'VariableNamingRule', 'preserve');
A = readtable(alignPath, 'VariableNamingRule', 'preserve');
A = A(:, {'sample_index', 'meas_up_m'});
T = innerjoin(G, A, 'Keys', 'sample_index');
T = T(strcmp(string(T.fix_type), 'RTKFIX'), :);
Ref = readtable(refPath, 'VariableNamingRule', 'preserve');
B = readtable(bodyZPath, 'VariableNamingRule', 'preserve');
I = readtable(iterPath, 'VariableNamingRule', 'preserve');
C = readtable(stateCorrectionPath, 'VariableNamingRule', 'preserve');

relTime = T.corrected_time_s - dynamicStartTimeS;
correctionRelTime = C.corrected_time_s - dynamicStartTimeS;
firstRtkUp = T.meas_up_m(1);
rtkUp = T.meas_up_m - firstRtkUp;
prefitUp = T.meas_up_m + T.local_prefit_residual_u_m - firstRtkUp;
postfitUp = interp1(Ref.time_s, Ref.up_m, T.corrected_time_s, 'linear', NaN) - firstRtkUp;
postfitResidualU = postfitUp - rtkUp;
postfitVz = interp1(Ref.time_s, Ref.vz_mps, T.corrected_time_s, 'linear', NaN);
ugMps2 = 9.80665e-6;
referenceBazUg = C.reference_baz_mps2 ./ ugMps2;
optimizedBazUg = C.optimized_baz_mps2 ./ ugMps2;
accepted = BuildAcceptedWindows(I, Ref, dynamicStartTimeS);

fig = figure('Name', 'Full navigation with body-z seed windows', ...
    'Color', 'w', 'Position', [60, 40, 1700, 1150]);
tiledlayout(fig, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

ax1 = nexttile; hold(ax1, 'on');
plot(ax1, relTime, rtkUp, 'k.-', 'DisplayName', 'RTKFIX up');
plot(ax1, relTime, prefitUp, 'Color', [0.80 0.18 0.16], ...
    'LineWidth', 1.0, 'DisplayName', 'local prefit up');
plot(ax1, relTime, postfitUp, ':', 'Color', [0.05 0.50 0.25], ...
    'LineWidth', 1.6, 'DisplayName', 'local postfit up');
grid(ax1, 'on');
ylabel(ax1, 'up - first RTKFIX (m)');
title(ax1, sprintf('Full local RTK/prefit/postfit vertical profile with body-z seed windows, total %d windows', height(B)));
shadeBodyZWindows(ax1, B);
shadeAcceptedWindows(ax1, accepted);
legend(ax1, 'Location', 'best');

ax2 = nexttile; hold(ax2, 'on');
plot(ax2, relTime, T.vz_ref_global_smoothed_mps, 'k-', 'LineWidth', 1.0, ...
    'DisplayName', 'RTKFIX diff vz ref');
plot(ax2, relTime, T.vz_prefit_mps, 'Color', [0.80 0.18 0.16], ...
    'LineWidth', 1.0, 'DisplayName', 'local prefit vz');
plot(ax2, relTime, postfitVz, ':', 'Color', [0.05 0.50 0.25], ...
    'LineWidth', 1.5, 'DisplayName', 'local postfit vz');
scatterAcceptedDelta(ax2, accepted);
scatterBodyZDelta(ax2, B);
grid(ax2, 'on');
ylabel(ax2, 'vz / delta vz (m/s)');
title(ax2, 'Velocity reference, local prefit velocity, local postfit velocity, and accepted delta-v tail');
shadeBodyZWindows(ax2, B);
shadeAcceptedWindows(ax2, accepted);
legend(ax2, 'Location', 'best');

ax3 = nexttile; hold(ax3, 'on');
plot(ax3, relTime, T.local_prefit_residual_u_m, 'Color', [0.80 0.18 0.16], ...
    'LineWidth', 1.0, 'DisplayName', 'local prefit residual u');
plot(ax3, relTime, postfitResidualU, 'Color', [0.05 0.50 0.25], ...
    'LineWidth', 1.5, 'DisplayName', 'final reference residual u');
plot(ax3, relTime, T.vertical_gate_threshold_m, '--', 'Color', [0.45 0.45 0.45], ...
    'DisplayName', '+1D NIS gate');
plot(ax3, relTime, -T.vertical_gate_threshold_m, '--', 'Color', [0.45 0.45 0.45], ...
    'HandleVisibility', 'off');
grid(ax3, 'on');
ylabel(ax3, 'u residual (m)');
title(ax3, 'Local vertical residuals against RTKFIX');
shadeBodyZWindows(ax3, B);
shadeAcceptedWindows(ax3, accepted);
legend(ax3, 'Location', 'best');

ax4 = nexttile; hold(ax4, 'on');
plot(ax4, correctionRelTime, referenceBazUg, 'Color', [0.05 0.50 0.25], ...
    'LineWidth', 1.5, 'DisplayName', 'reference ba_z estimate');
plot(ax4, correctionRelTime, optimizedBazUg, 'Color', [0.10 0.35 0.82], ...
    'LineWidth', 1.2, 'DisplayName', 'optimized ba_z estimate');
yline(ax4, referenceBazUg(1), '--', 'Color', [0.45 0.45 0.45], ...
    'DisplayName', 'initial reference ba_z');
grid(ax4, 'on');
ylabel(ax4, 'ba_z (ug)');
xlabel(ax4, 'time since dynamic start (s)');
title(ax4, 'Vertical accelerometer bias estimate');
shadeBodyZWindows(ax4, B);
shadeAcceptedWindows(ax4, accepted);
legend(ax4, 'Location', 'best');

linkaxes([ax1 ax2 ax3 ax4], 'x');
xlim([min(relTime), max(relTime)]);
savefig(fig, figPath, 'compact');
exportgraphics(fig, pngPath, 'Resolution', 180);
close(fig);
fprintf('Saved %s\n', figPath);
fprintf('Saved %s\n', pngPath);

function accepted = BuildAcceptedWindows(I, Ref, dynamicStartTimeS)
accepted = table();
if isempty(I) || ~ismember('selected_jump_window_start_state_index', I.Properties.VariableNames)
    return;
end
mask = I.selected_jump_window_start_state_index >= 0 & ...
    I.selected_jump_window_center_state_index >= 0 & ...
    I.selected_jump_window_end_state_index >= 0;
I = I(mask, :);
if isempty(I)
    return;
end
keys = string(I.selected_jump_window_start_state_index) + "_" + ...
    string(I.selected_jump_window_center_state_index) + "_" + ...
    string(I.selected_jump_window_end_state_index) + "_" + string(I.corrected_time_s);
[~, uniqueIdx] = unique(keys, 'stable');
I = I(uniqueIdx, :);
startIdx = I.selected_jump_window_start_state_index + 1;
centerIdx = I.selected_jump_window_center_state_index + 1;
endIdx = I.selected_jump_window_end_state_index + 1;
valid = startIdx >= 1 & endIdx <= height(Ref) & centerIdx >= 1 & centerIdx <= height(Ref);
I = I(valid, :);
startIdx = startIdx(valid);
centerIdx = centerIdx(valid);
endIdx = endIdx(valid);
accepted = table();
accepted.start_rel_time_s = Ref.time_s(startIdx) - dynamicStartTimeS;
accepted.center_rel_time_s = Ref.time_s(centerIdx) - dynamicStartTimeS;
accepted.end_rel_time_s = Ref.time_s(endIdx) - dynamicStartTimeS;
accepted.selected_jump_delta_vz_tail_mps = I.selected_jump_delta_vz_tail_mps;
accepted.recovery_mode = string(I.recovery_mode);
end

function shadeBodyZWindows(ax, W)
if isempty(W) || ~ismember('start_relative_time_s', W.Properties.VariableNames)
    return;
end
y = ylim(ax);
legendAdded = false;
for i = 1:height(W)
    st = W.start_relative_time_s(i);
    et = W.end_relative_time_s(i);
    ct = W.center_relative_time_s(i);
    if ~isfinite(st) || ~isfinite(et)
        continue;
    end
    if string(W.direction(i)) == "DOWN"
        color = [0.86 0.20 0.16];
    else
        color = [0.10 0.35 0.82];
    end
    vis = 'off';
    if ~legendAdded
        vis = 'on';
        legendAdded = true;
    end
    h = patch(ax, [st et et st], [y(1) y(1) y(2) y(2)], color, ...
        'FaceAlpha', 0.10, 'EdgeColor', color, 'LineStyle', '--', ...
        'DisplayName', 'body-z seed jump window', 'HandleVisibility', vis);
    uistack(h, 'bottom');
    xline(ax, ct, ':', 'Color', color * 0.75, 'HandleVisibility', 'off');
end
ylim(ax, y);
end

function shadeAcceptedWindows(ax, W)
if isempty(W) || ~ismember('start_rel_time_s', W.Properties.VariableNames)
    return;
end
y = ylim(ax);
legendAdded = false;
for i = 1:height(W)
    st = W.start_rel_time_s(i);
    et = W.end_rel_time_s(i);
    ct = W.center_rel_time_s(i);
    if ~isfinite(st) || ~isfinite(et)
        continue;
    end
    color = [0.95 0.65 0.05];
    vis = 'off';
    if ~legendAdded
        vis = 'on';
        legendAdded = true;
    end
    h = patch(ax, [st et et st], [y(1) y(1) y(2) y(2)], color, ...
        'FaceAlpha', 0.18, 'EdgeColor', [0.55 0.36 0.02], ...
        'LineStyle', '-', 'DisplayName', 'accepted recovery window', ...
        'HandleVisibility', vis);
    uistack(h, 'bottom');
    xline(ax, ct, '--', 'Color', [0.55 0.36 0.02], 'HandleVisibility', 'off');
end
ylim(ax, y);
end

function scatterAcceptedDelta(ax, T)
if isempty(T) || ~ismember('selected_jump_delta_vz_tail_mps', T.Properties.VariableNames)
    return;
end
mask = isfinite(T.selected_jump_delta_vz_tail_mps) & ...
    abs(T.selected_jump_delta_vz_tail_mps) > 1e-9 & isfinite(T.center_rel_time_s);
if any(mask)
    scatter(ax, T.center_rel_time_s(mask), T.selected_jump_delta_vz_tail_mps(mask), ...
        36, [0.10 0.55 0.85], 'filled', 'DisplayName', 'accepted delta-v tail');
end
end

function scatterBodyZDelta(ax, W)
if isempty(W) || ~ismember('delta_vz_init_mps', W.Properties.VariableNames)
    return;
end
mask = isfinite(W.delta_vz_init_mps);
if any(mask)
    scatter(ax, W.center_relative_time_s(mask), W.delta_vz_init_mps(mask), ...
        42, [0.86 0.20 0.16], 'v', 'filled', 'DisplayName', 'body-z delta-v init');
end
end

function scatterBodyZStep(ax, W)
if isempty(W) || ~ismember('signed_delta_velocity_mps', W.Properties.VariableNames)
    return;
end
mask = isfinite(W.signed_delta_velocity_mps);
if any(mask)
    scatter(ax, W.center_relative_time_s(mask), W.signed_delta_velocity_mps(mask), ...
        42, [0.86 0.20 0.16], 'v', 'filled', 'DisplayName', 'body-z signed step');
end
end
