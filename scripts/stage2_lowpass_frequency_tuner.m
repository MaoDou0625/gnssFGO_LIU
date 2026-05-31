function app = stage2_lowpass_frequency_tuner(resultDir, varargin)
%STAGE2_LOWPASS_FREQUENCY_TUNER Interactive Stage2 vertical lowpass tuner.
%
%   stage2_lowpass_frequency_tuner()
%   stage2_lowpass_frequency_tuner(resultDir)
%   app = stage2_lowpass_frequency_tuner(resultDir, 'Visible', 'off')
%
% The tool reads the current vertical solution from resultDir. It prefers
% stage3_vertical_reference_diagnostics.csv when available, otherwise it falls
% back to trajectory.csv so Stage2-only runs can be smoothed directly.

if nargin < 1 || isempty(resultDir)
  resultDir = defaultResultDir();
end

options = parseOptions(varargin{:});
data = loadStage2LowpassData(resultDir);

state.rangeMinHz = options.MinCutoffHz;
state.rangeMaxHz = options.MaxCutoffHz;
state.cutoffHz = min(max(options.InitialCutoffHz, state.rangeMinHz), state.rangeMaxHz);
state.excludeStatic = true;
state.protectInitialDynamicStatic = true;
state.showRtkScatter = false;
state.showStage2Up = true;
state.showTunedReference = true;
state.showSolverReference = data.hasSolverReference;

fig = figure( ...
  'Name', 'Stage2 Lowpass Frequency Tuner', ...
  'NumberTitle', 'off', ...
  'Color', 'w', ...
  'Visible', options.Visible, ...
  'Position', [60 60 1500 900]);

axFull = axes('Parent', fig, 'Position', [0.06 0.57 0.88 0.26]);
axDelta = axes('Parent', fig, 'Position', [0.06 0.34 0.88 0.15]);
axStart = axes('Parent', fig, 'Position', [0.06 0.07 0.42 0.20]);
axEnd = axes('Parent', fig, 'Position', [0.52 0.07 0.42 0.20]);

uicontrol(fig, 'Style', 'text', 'String', 'Cutoff Hz', ...
  'Units', 'normalized', 'Position', [0.06 0.945 0.06 0.025], ...
  'BackgroundColor', 'w', 'HorizontalAlignment', 'left');
cutoffSlider = uicontrol(fig, 'Style', 'slider', ...
  'Units', 'normalized', 'Position', [0.12 0.947 0.25 0.025], ...
  'Min', log10(state.rangeMinHz), ...
  'Max', log10(state.rangeMaxHz), ...
  'Value', log10(state.cutoffHz), ...
  'Callback', @onCutoffSlider);
cutoffEdit = uicontrol(fig, 'Style', 'edit', ...
  'Units', 'normalized', 'Position', [0.38 0.944 0.06 0.032], ...
  'String', formatCutoff(state.cutoffHz), ...
  'Callback', @onCutoffEdit);
presetPopup = uicontrol(fig, 'Style', 'popupmenu', ...
  'Units', 'normalized', 'Position', [0.45 0.944 0.08 0.032], ...
  'String', {'0.005','0.01','0.02','0.03','0.05','0.08','0.10','0.20','0.50','1.0','2.0','5.0'}, ...
  'Value', 2, ...
  'Callback', @onPreset);
staticCheckbox = uicontrol(fig, 'Style', 'checkbox', ...
  'Units', 'normalized', 'Position', [0.55 0.947 0.15 0.025], ...
  'String', 'exclude static ranges', ...
  'Value', state.excludeStatic, ...
  'BackgroundColor', 'w', ...
  'Callback', @onStaticCheckbox);
initialWindowCheckbox = uicontrol(fig, 'Style', 'checkbox', ...
  'Units', 'normalized', 'Position', [0.70 0.947 0.17 0.025], ...
  'String', 'protect initial dynamic static', ...
  'Value', state.protectInitialDynamicStatic, ...
  'BackgroundColor', 'w', ...
  'Callback', @onInitialWindowCheckbox);
solverReferenceCheckbox = uicontrol(fig, 'Style', 'checkbox', ...
  'Units', 'normalized', 'Position', [0.87 0.947 0.10 0.025], ...
  'String', 'solver ref', ...
  'Value', state.showSolverReference, ...
  'Enable', onOff(data.hasSolverReference), ...
  'BackgroundColor', 'w', ...
  'Callback', @onSolverReferenceCheckbox);

uicontrol(fig, 'Style', 'text', 'String', 'Min Hz', ...
  'Units', 'normalized', 'Position', [0.06 0.907 0.045 0.025], ...
  'BackgroundColor', 'w', 'HorizontalAlignment', 'left');
rangeMinEdit = uicontrol(fig, 'Style', 'edit', ...
  'Units', 'normalized', 'Position', [0.105 0.904 0.055 0.032], ...
  'String', formatCutoff(state.rangeMinHz), ...
  'Callback', @onRangeEdit);
uicontrol(fig, 'Style', 'text', 'String', 'Max Hz', ...
  'Units', 'normalized', 'Position', [0.17 0.907 0.045 0.025], ...
  'BackgroundColor', 'w', 'HorizontalAlignment', 'left');
rangeMaxEdit = uicontrol(fig, 'Style', 'edit', ...
  'Units', 'normalized', 'Position', [0.215 0.904 0.055 0.032], ...
  'String', formatCutoff(state.rangeMaxHz), ...
  'Callback', @onRangeEdit);
uicontrol(fig, 'Style', 'pushbutton', 'String', 'Save PNG', ...
  'Units', 'normalized', 'Position', [0.29 0.904 0.07 0.032], ...
  'Callback', @onSavePng);
uicontrol(fig, 'Style', 'pushbutton', 'String', 'Save FIG', ...
  'Units', 'normalized', 'Position', [0.37 0.904 0.07 0.032], ...
  'Callback', @onSaveFig);
uicontrol(fig, 'Style', 'pushbutton', 'String', 'Export CSV', ...
  'Units', 'normalized', 'Position', [0.45 0.904 0.08 0.032], ...
  'Callback', @onExportCsv);
rtkScatterCheckbox = uicontrol(fig, 'Style', 'checkbox', ...
  'Units', 'normalized', 'Position', [0.55 0.910 0.09 0.025], ...
  'String', 'RTK scatter', ...
  'Value', state.showRtkScatter, ...
  'BackgroundColor', 'w', ...
  'Callback', @onRtkScatterCheckbox);
stage2UpCheckbox = uicontrol(fig, 'Style', 'checkbox', ...
  'Units', 'normalized', 'Position', [0.64 0.910 0.09 0.025], ...
  'String', 'Stage2 up', ...
  'Value', state.showStage2Up, ...
  'BackgroundColor', 'w', ...
  'Callback', @onStage2UpCheckbox);
tunedReferenceCheckbox = uicontrol(fig, 'Style', 'checkbox', ...
  'Units', 'normalized', 'Position', [0.73 0.910 0.11 0.025], ...
  'String', 'tuned smooth', ...
  'Value', state.showTunedReference, ...
  'BackgroundColor', 'w', ...
  'Callback', @onTunedReferenceCheckbox);
statusText = uicontrol(fig, 'Style', 'text', ...
  'Units', 'normalized', 'Position', [0.06 0.855 0.88 0.04], ...
  'String', '', ...
  'BackgroundColor', 'w', ...
  'HorizontalAlignment', 'left');

current = recomputeAndPlot();

app = struct();
app.figure = fig;
app.axes = struct('full', axFull, 'delta', axDelta, 'start', axStart, 'end', axEnd);
app.resultDir = resultDir;
app.data = data;
app.current = current;

if nargout == 0
  clear app;
end

  function onCutoffSlider(~, ~)
    state.cutoffHz = 10 .^ get(cutoffSlider, 'Value');
    set(cutoffEdit, 'String', formatCutoff(state.cutoffHz));
    current = recomputeAndPlot();
  end

  function onCutoffEdit(~, ~)
    value = str2double(get(cutoffEdit, 'String'));
    if ~isfinite(value) || value <= 0
      set(cutoffEdit, 'String', formatCutoff(state.cutoffHz));
      return;
    end
    applyCutoffValue(value, true);
    current = recomputeAndPlot();
  end

  function onPreset(~, ~)
    labels = get(presetPopup, 'String');
    applyCutoffValue(str2double(labels{get(presetPopup, 'Value')}), true);
    current = recomputeAndPlot();
  end

  function onRangeEdit(~, ~)
    newMinHz = str2double(get(rangeMinEdit, 'String'));
    newMaxHz = str2double(get(rangeMaxEdit, 'String'));
    if ~isfinite(newMinHz) || ~isfinite(newMaxHz) || ...
        newMinHz <= 0 || newMaxHz <= 0 || newMinHz >= newMaxHz
      refreshCutoffControls();
      return;
    end
    state.rangeMinHz = newMinHz;
    state.rangeMaxHz = newMaxHz;
    state.cutoffHz = min(max(state.cutoffHz, state.rangeMinHz), state.rangeMaxHz);
    refreshCutoffControls();
    current = recomputeAndPlot();
  end

  function onStaticCheckbox(~, ~)
    state.excludeStatic = logical(get(staticCheckbox, 'Value'));
    current = recomputeAndPlot();
  end

  function onInitialWindowCheckbox(~, ~)
    state.protectInitialDynamicStatic = logical(get(initialWindowCheckbox, 'Value'));
    current = recomputeAndPlot();
  end

  function onSolverReferenceCheckbox(~, ~)
    state.showSolverReference = logical(get(solverReferenceCheckbox, 'Value'));
    current = recomputeAndPlot();
  end

  function onRtkScatterCheckbox(~, ~)
    state.showRtkScatter = logical(get(rtkScatterCheckbox, 'Value'));
    current = recomputeAndPlot();
  end

  function onStage2UpCheckbox(~, ~)
    state.showStage2Up = logical(get(stage2UpCheckbox, 'Value'));
    current = recomputeAndPlot();
  end

  function onTunedReferenceCheckbox(~, ~)
    state.showTunedReference = logical(get(tunedReferenceCheckbox, 'Value'));
    current = recomputeAndPlot();
  end

  function onSavePng(~, ~)
    defaultName = fullfile(resultDir, sprintf('stage2_lowpass_tuned_%.4gHz.png', state.cutoffHz));
    [fileName, pathName] = uiputfile('*.png', 'Save PNG', defaultName);
    if isequal(fileName, 0)
      return;
    end
    exportgraphics(fig, fullfile(pathName, fileName), 'Resolution', 180);
  end

  function onSaveFig(~, ~)
    defaultName = fullfile(resultDir, sprintf('stage2_lowpass_tuned_%.4gHz.fig', state.cutoffHz));
    [fileName, pathName] = uiputfile('*.fig', 'Save FIG', defaultName);
    if isequal(fileName, 0)
      return;
    end
    savefig(fig, fullfile(pathName, fileName));
  end

  function onExportCsv(~, ~)
    defaultName = fullfile(resultDir, sprintf('stage2_lowpass_tuned_%.4gHz.csv', state.cutoffHz));
    [fileName, pathName] = uiputfile('*.csv', 'Export tuned reference CSV', defaultName);
    if isequal(fileName, 0)
      return;
    end
    out = table( ...
      data.timeS(:), ...
      data.stage2UpM(:), ...
      current.referenceUpM(:), ...
      current.deltaM(:), ...
      data.skipReason(:), ...
      'VariableNames', {'time_s','stage2_up_m','tuned_lowpass_up_m','tuned_lowpass_delta_m','skip_reason'});
    writetable(out, fullfile(pathName, fileName));
  end

  function result = recomputeAndPlot()
    result = computeTunedReference(data, state.cutoffHz, ...
      state.excludeStatic, state.protectInitialDynamicStatic);
    drawPlots(data, result, state, axFull, axDelta, axStart, axEnd);
    set(statusText, 'String', summaryString(data, result, state.cutoffHz));
    drawnow limitrate;
  end

  function applyCutoffValue(value, expandRange)
    if expandRange
      state.rangeMinHz = min(state.rangeMinHz, value);
      state.rangeMaxHz = max(state.rangeMaxHz, value);
    end
    state.cutoffHz = min(max(value, state.rangeMinHz), state.rangeMaxHz);
    refreshCutoffControls();
  end

  function refreshCutoffControls()
    set(cutoffSlider, 'Min', log10(state.rangeMinHz), ...
      'Max', log10(state.rangeMaxHz), ...
      'Value', log10(state.cutoffHz));
    set(cutoffEdit, 'String', formatCutoff(state.cutoffHz));
    set(rangeMinEdit, 'String', formatCutoff(state.rangeMinHz));
    set(rangeMaxEdit, 'String', formatCutoff(state.rangeMaxHz));
  end
end

function options = parseOptions(varargin)
options.Visible = 'on';
options.InitialCutoffHz = 0.01;
options.MinCutoffHz = 0.003;
options.MaxCutoffHz = 5.0;
if mod(numel(varargin), 2) ~= 0
  error('Options must be name-value pairs.');
end
for i = 1:2:numel(varargin)
  name = lower(string(varargin{i}));
  value = varargin{i + 1};
  switch name
    case "visible"
      options.Visible = char(value);
    case "initialcutoffhz"
      options.InitialCutoffHz = value;
    case "mincutoffhz"
      options.MinCutoffHz = value;
    case "maxcutoffhz"
      options.MaxCutoffHz = value;
    otherwise
      error('Unknown option: %s', name);
  end
end
end

function resultDir = defaultResultDir()
resultDir = ['D:\Code\dataset\BeiJingGongLuTuiChe\gnssFGO_use\' ...
  '20260323_124742\transformed1rtkjumpcut2\offline_lc_v2_0_result_stage2_center006'];
end

function data = loadStage2LowpassData(resultDir)
csvPath = fullfile(resultDir, 'stage3_vertical_reference_diagnostics.csv');
trajectoryPath = fullfile(resultDir, 'trajectory.csv');
if isfile(csvPath)
  data = loadStage3DiagnosticsData(resultDir, csvPath);
elseif isfile(trajectoryPath)
  data = loadTrajectoryData(resultDir, trajectoryPath);
else
  error('Missing stage3_vertical_reference_diagnostics.csv or trajectory.csv under: %s', resultDir);
end
data = finalizeLowpassData(data, resultDir);
end

function data = loadStage3DiagnosticsData(resultDir, csvPath)
T = readtable(csvPath, 'TextType', 'string');
data.resultDir = resultDir;
data.sourceName = 'stage3_vertical_reference_diagnostics.csv';
data.timeS = T.time_s;
data.stage2UpM = T.stage2_up_m;
data.solverReferenceUpM = T.stage2_lowpass_up_m;
data.optimizedUpM = optionalColumn(T, 'optimized_up_m', nan(height(T), 1));
data.skipReason = T.skip_reason;
data.initialMask = data.skipReason == "INITIAL_STATIC";
data.terminalMask = data.skipReason == "TERMINAL_STATIC";
data.currentFactorMask = logical(optionalColumn(T, 'factor_added', zeros(height(T), 1)));
end

function data = loadTrajectoryData(resultDir, trajectoryPath)
T = readtable(trajectoryPath, 'TextType', 'string');
requiredColumns = {'time_s','up_m'};
if ~all(ismember(requiredColumns, T.Properties.VariableNames))
  error('trajectory.csv must contain time_s and up_m columns: %s', trajectoryPath);
end
data.resultDir = resultDir;
data.sourceName = 'trajectory.csv';
data.timeS = T.time_s;
data.stage2UpM = T.up_m;
data.solverReferenceUpM = nan(height(T), 1);
data.optimizedUpM = nan(height(T), 1);
data.currentFactorMask = logical(optionalColumn(T, 'gnss_factor_used', zeros(height(T), 1)));
data.dynamicStartTimeS = readSummaryValue(resultDir, 'dynamic_start_time_s', nan);
if ~isfinite(data.dynamicStartTimeS)
  data.dynamicStartTimeS = data.timeS(1);
end
data.initialMask = data.timeS < data.dynamicStartTimeS;
data.terminalMask = readWindowMask(resultDir, data.timeS, 'late_static_windows.csv');
data.skipReason = buildSkipReason(data.initialMask, data.terminalMask);
end

function data = finalizeLowpassData(data, resultDir)
data.relativeTimeS = data.timeS - data.timeS(1);
if ~isfield(data, 'dynamicStartTimeS') || ~isfinite(data.dynamicStartTimeS)
  data.dynamicStartTimeS = firstNonInitialTime(data);
end
data.dynamicStartRelS = data.dynamicStartTimeS - data.timeS(1);
data.configBlendS = readConfigValue(resultDir, 'initial_dynamic_static_lowpass_blend_s', 2.0);
data.initialDynamicStaticWindows = readInitialDynamicStaticWindows(resultDir);
data.rtkScatter = readRtkScatter(resultDir, data.timeS(1));
data.hasSolverReference = any(isfinite(data.solverReferenceUpM));
end

function skipReason = buildSkipReason(initialMask, terminalMask)
skipReason = strings(numel(initialMask), 1);
skipReason(:) = "DYNAMIC";
skipReason(initialMask) = "INITIAL_STATIC";
skipReason(terminalMask) = "TERMINAL_STATIC";
end

function values = optionalColumn(T, name, defaultValue)
if ismember(name, T.Properties.VariableNames)
  values = T.(name);
else
  values = defaultValue;
end
end

function mask = readWindowMask(resultDir, timeS, fileName)
mask = false(size(timeS));
windowPath = fullfile(resultDir, fileName);
if ~isfile(windowPath)
  return;
end
T = readtable(windowPath, 'TextType', 'string');
requiredColumns = {'start_time_s','end_time_s'};
if ~all(ismember(requiredColumns, T.Properties.VariableNames))
  return;
end
valid = true(height(T), 1);
if ismember('valid', T.Properties.VariableNames)
  valid = T.valid ~= 0;
end
for i = 1:height(T)
  if ~valid(i)
    continue;
  end
  startS = T.start_time_s(i);
  endS = T.end_time_s(i);
  if isfinite(startS) && isfinite(endS)
    mask = mask | (timeS >= startS & timeS <= endS);
  end
end
end

function dynamicStartTimeS = firstNonInitialTime(data)
index = find(~data.initialMask & isfinite(data.timeS), 1, 'first');
if isempty(index)
  dynamicStartTimeS = data.timeS(1);
else
  dynamicStartTimeS = data.timeS(index);
end
end

function value = readConfigValue(resultDir, key, defaultValue)
value = defaultValue;
cfgPath = fullfile(resultDir, 'config_snapshot.cfg');
if ~isfile(cfgPath)
  return;
end
text = fileread(cfgPath);
pattern = "(?m)^" + regexptranslate('escape', key) + "=(?<value>[^\r\n]+)";
match = regexp(text, pattern, 'names', 'once');
if isempty(match)
  return;
end
parsed = str2double(match.value);
if isfinite(parsed)
  value = parsed;
end
end

function value = readSummaryValue(resultDir, key, defaultValue)
value = defaultValue;
summaryPath = fullfile(resultDir, 'summary.txt');
if ~isfile(summaryPath)
  return;
end
text = fileread(summaryPath);
pattern = "(?m)^" + regexptranslate('escape', key) + "=(?<value>[^\r\n]+)";
match = regexp(text, pattern, 'names', 'once');
if isempty(match)
  return;
end
parsed = str2double(match.value);
if isfinite(parsed)
  value = parsed;
end
end

function windows = readInitialDynamicStaticWindows(resultDir)
windows = table();
windowPath = fullfile(resultDir, 'initial_dynamic_static_windows.csv');
if isfile(windowPath)
  windows = readtable(windowPath, 'TextType', 'string');
end
end

function rtkScatter = readRtkScatter(resultDir, referenceTimeS)
dynamicRtk = readDynamicRtkScatter(resultDir);
staticRtk = readStaticAlignmentRtkScatter(resultDir);
rtkScatter = combineRtkScatter(staticRtk, dynamicRtk, referenceTimeS);
end

function rtkScatter = emptyRtkScatter()
rtkScatter = struct('timeS', [], 'relativeTimeS', [], 'upM', []);
end

function rtkScatter = readDynamicRtkScatter(resultDir)
rtkScatter = emptyRtkScatter();
alignmentPath = fullfile(resultDir, 'gnss_alignment.csv');
if ~isfile(alignmentPath)
  return;
end
T = readtable(alignmentPath, 'TextType', 'string');
requiredColumns = {'corrected_time_s','raw_rtk_up_m'};
if ~all(ismember(requiredColumns, T.Properties.VariableNames))
  return;
end
valid = isfinite(T.corrected_time_s) & isfinite(T.raw_rtk_up_m);
if ismember('factor_used', T.Properties.VariableNames)
  valid = valid & T.factor_used ~= 0;
end
if ismember('fix_type', T.Properties.VariableNames)
  valid = valid & T.fix_type == "RTKFIX";
end
rtkScatter.timeS = T.corrected_time_s(valid);
rtkScatter.upM = T.raw_rtk_up_m(valid);
end

function rtkScatter = readStaticAlignmentRtkScatter(resultDir)
rtkScatter = emptyRtkScatter();
validationPath = fullfile(resultDir, 'static_alignment_validation.csv');
gnssPath = findGnssSolutionPath(resultDir);
originHeightM = readSummaryValue(resultDir, 'origin_h_m', nan);
if ~isfile(validationPath) || ~isfile(gnssPath) || ~isfinite(originHeightM)
  return;
end
T = readtable(validationPath, 'TextType', 'string');
if ~ismember('time_s', T.Properties.VariableNames)
  return;
end
staticTimes = T.time_s(isfinite(T.time_s));
if isempty(staticTimes)
  return;
end
staticStartS = min(staticTimes);
staticEndS = max(staticTimes);
gnssMatrix = readmatrix(gnssPath, 'FileType', 'text');
if size(gnssMatrix, 2) < 13
  return;
end
timeOffsetS = readConfigValue(resultDir, 'gnss_time_offset_s', 0.0);
timeS = gnssMatrix(:, 1) - timeOffsetS;
heightM = gnssMatrix(:, 4);
fixType = gnssMatrix(:, 13);
valid = isfinite(timeS) & isfinite(heightM) & ...
  timeS >= staticStartS & timeS <= staticEndS & fixType == 1;
rtkScatter.timeS = timeS(valid);
rtkScatter.upM = heightM(valid) - originHeightM;
end

function gnssPath = findGnssSolutionPath(resultDir)
candidatePaths = {
  fullfile(resultDir, 'gnss_solution_gnss_fgo.txt')
  fullfile(resultDir, '..', 'gnss_solution_gnss_fgo.txt')
  };
gnssPath = '';
for i = 1:numel(candidatePaths)
  candidatePath = char(candidatePaths{i});
  if isfile(candidatePath)
    gnssPath = candidatePath;
    return;
  end
end
end

function rtkScatter = combineRtkScatter(staticRtk, dynamicRtk, referenceTimeS)
rtkScatter = emptyRtkScatter();
timeS = [staticRtk.timeS(:); dynamicRtk.timeS(:)];
upM = [staticRtk.upM(:); dynamicRtk.upM(:)];
valid = isfinite(timeS) & isfinite(upM);
if ~any(valid)
  return;
end
timeS = timeS(valid);
upM = upM(valid);
[timeS, order] = sort(timeS);
upM = upM(order);
rtkScatter.timeS = timeS;
rtkScatter.relativeTimeS = timeS - referenceTimeS;
rtkScatter.upM = upM;
end

function result = computeTunedReference(data, cutoffHz, excludeStatic, protectInitialDynamicStatic)
inputUpM = data.stage2UpM;
if protectInitialDynamicStatic
  inputUpM = applyInitialDynamicStaticWindows(data, inputUpM, false);
end

firstIndex = 1;
lastIndex = numel(data.timeS);
if excludeStatic
  firstDynamic = find(~data.initialMask, 1, 'first');
  firstTerminal = find(data.terminalMask, 1, 'first');
  if ~isempty(firstDynamic)
    firstIndex = firstDynamic;
  end
  if ~isempty(firstTerminal)
    lastIndex = firstTerminal - 1;
  end
end

referenceUpM = buildLowpassProfile(data.timeS, inputUpM, cutoffHz, firstIndex, lastIndex);
if protectInitialDynamicStatic
  referenceUpM = applyInitialDynamicStaticWindows(data, referenceUpM, true);
end

result.referenceUpM = referenceUpM;
result.deltaM = referenceUpM - data.stage2UpM;
result.firstFilterIndex = firstIndex;
result.lastFilterIndex = lastIndex;
end

function outputUpM = applyInitialDynamicStaticWindows(data, inputUpM, protectOutput)
outputUpM = inputUpM;
windows = data.initialDynamicStaticWindows;
if isempty(windows) || ~ismember('valid', windows.Properties.VariableNames)
  return;
end
for i = 1:height(windows)
  if windows.valid(i) == 0
    continue;
  end
  startS = windows.start_time_s(i);
  endS = windows.end_time_s(i);
  inWindow = data.timeS >= startS & data.timeS <= endS & isfinite(data.stage2UpM);
  if ~any(inWindow)
    continue;
  end
  referenceUpM = median(data.stage2UpM(inWindow), 'omitnan');
  outputUpM(inWindow) = referenceUpM;
  if ~protectOutput
    continue;
  end
  blendDurationS = max(0.0, data.configBlendS);
  if blendDurationS <= 0
    continue;
  end
  inBlend = data.timeS > endS & data.timeS < endS + blendDurationS & isfinite(outputUpM);
  blendFraction = (data.timeS(inBlend) - endS) ./ blendDurationS;
  outputUpM(inBlend) = (1.0 - blendFraction) .* referenceUpM + ...
    blendFraction .* outputUpM(inBlend);
end
end

function lowpassUpM = buildLowpassProfile(timeS, inputUpM, cutoffHz, firstIndex, lastIndex)
lowpassUpM = nan(size(inputUpM));
validInput = isfinite(timeS) & isfinite(inputUpM);
segment = [];
for index = 1:numel(timeS)
  if index < firstIndex || index > lastIndex
    if validInput(index)
      lowpassUpM(index) = inputUpM(index);
    end
    continue;
  end
  if ~validInput(index)
    lowpassUpM = filterSegment(timeS, inputUpM, lowpassUpM, segment, cutoffHz);
    segment = [];
    continue;
  end
  if ~isempty(segment) && timeS(index) <= timeS(segment(end))
    lowpassUpM = filterSegment(timeS, inputUpM, lowpassUpM, segment, cutoffHz);
    segment = [];
  end
  segment(end + 1) = index; %#ok<AGROW>
end
lowpassUpM = filterSegment(timeS, inputUpM, lowpassUpM, segment, cutoffHz);
end

function lowpassUpM = filterSegment(timeS, inputUpM, lowpassUpM, indices, cutoffHz)
if isempty(indices)
  return;
end
tauS = 1.0 / (2.0 * pi * cutoffHz);
forward = zeros(numel(indices), 1);
forward(1) = inputUpM(indices(1));
for i = 2:numel(indices)
  dtS = timeS(indices(i)) - timeS(indices(i - 1));
  alpha = lowpassAlpha(dtS, tauS);
  forward(i) = forward(i - 1) + alpha * (inputUpM(indices(i)) - forward(i - 1));
end
zeroPhase = forward;
for i = (numel(indices) - 1):-1:1
  dtS = timeS(indices(i + 1)) - timeS(indices(i));
  alpha = lowpassAlpha(dtS, tauS);
  zeroPhase(i) = zeroPhase(i + 1) + alpha * (forward(i) - zeroPhase(i + 1));
end
lowpassUpM(indices) = zeroPhase;
end

function alpha = lowpassAlpha(dtS, tauS)
if dtS <= 0 || ~isfinite(dtS)
  alpha = 0.0;
  return;
end
alpha = 1.0 - exp(-dtS / tauS);
end

function drawPlots(data, result, state, axFull, axDelta, axStart, axEnd)
drawReferenceAxes(axFull, data, result, state, 'Full Stage2 vertical reference');
drawDeltaAxes(axDelta, data, result, state, 'Lowpass delta');

drawReferenceAxes(axStart, data, result, state, 'Start boundary zoom');
xlim(axStart, [data.dynamicStartRelS - 5.0, data.dynamicStartRelS + 20.0]);

terminalIndex = find(data.terminalMask, 1, 'first');
if isempty(terminalIndex)
  endCenterS = data.relativeTimeS(end);
else
  endCenterS = data.relativeTimeS(terminalIndex);
end
drawReferenceAxes(axEnd, data, result, state, 'End boundary zoom');
xlim(axEnd, [endCenterS - 20.0, min(data.relativeTimeS(end), endCenterS + 15.0)]);
end

function drawReferenceAxes(ax, data, result, state, titleText)
cla(ax);
hold(ax, 'on');
legendHandles = gobjects(0);
legendItems = {};
if state.showRtkScatter && ~isempty(data.rtkScatter.relativeTimeS)
  hRtk = scatter(ax, data.rtkScatter.relativeTimeS, data.rtkScatter.upM, ...
    10, [0.95 0.55 0.10], 'filled', 'MarkerFaceAlpha', 0.35, ...
    'MarkerEdgeAlpha', 0.15);
  legendHandles(end + 1) = hRtk;
  legendItems{end + 1} = 'RTK up';
end
if state.showStage2Up
  hStage2 = plot(ax, data.relativeTimeS, data.stage2UpM, ...
    'Color', [0.10 0.55 0.42], 'LineWidth', 0.9);
  legendHandles(end + 1) = hStage2;
  legendItems{end + 1} = 'Stage2 up';
end
if state.showTunedReference
  hTuned = plot(ax, data.relativeTimeS, result.referenceUpM, ...
    'Color', [0.12 0.34 0.80], 'LineWidth', 1.3);
  legendHandles(end + 1) = hTuned;
  legendItems{end + 1} = 'tuned lowpass';
end
if state.showSolverReference && data.hasSolverReference
  hSolver = plot(ax, data.relativeTimeS, data.solverReferenceUpM, '--', ...
    'Color', [0.75 0.20 0.15], 'LineWidth', 0.9);
  legendHandles(end + 1) = hSolver;
  legendItems{end + 1} = 'solver lowpass';
end
decorateAxes(ax, data);
grid(ax, 'on');
title(ax, titleText);
ylabel(ax, 'Up (m)');
if ~isempty(legendHandles)
  legend(ax, legendHandles, legendItems, 'Location', 'best');
end
end

function drawDeltaAxes(ax, data, result, state, titleText)
cla(ax);
hold(ax, 'on');
legendHandles = gobjects(0);
legendItems = {};
if state.showTunedReference
  hTuned = plot(ax, data.relativeTimeS, result.deltaM, ...
    'Color', [0.15 0.15 0.15], 'LineWidth', 1.0);
  legendHandles(end + 1) = hTuned;
  legendItems{end + 1} = 'tuned delta';
end
if state.showSolverReference && data.hasSolverReference
  hSolver = plot(ax, data.relativeTimeS, data.solverReferenceUpM - data.stage2UpM, '--', ...
    'Color', [0.75 0.20 0.15], 'LineWidth', 0.8);
  legendHandles(end + 1) = hSolver;
  legendItems{end + 1} = 'solver delta';
end
yline(ax, 0.0, '-k');
decorateAxes(ax, data);
grid(ax, 'on');
title(ax, titleText);
ylabel(ax, 'Delta (m)');
xlabel(ax, 'Time since start (s)');
if ~isempty(legendHandles)
  legend(ax, legendHandles, legendItems, 'Location', 'best');
end
end

function decorateAxes(ax, data)
xline(ax, data.dynamicStartRelS, '--k', 'Dynamic start', ...
  'LabelVerticalAlignment', 'bottom', 'LabelOrientation', 'aligned');
shadeMask(ax, data.relativeTimeS, data.initialMask, [0.70 0.82 1.00], 0.16);
shadeMask(ax, data.relativeTimeS, data.terminalMask, [1.00 0.82 0.35], 0.20);
end

function shadeMask(ax, x, mask, color, alphaValue)
if ~any(mask)
  return;
end
yl = ylim(ax);
runs = maskRuns(mask);
for i = 1:size(runs, 1)
  x1 = x(runs(i, 1));
  x2 = x(runs(i, 2));
  patch(ax, [x1 x2 x2 x1], [yl(1) yl(1) yl(2) yl(2)], color, ...
    'FaceAlpha', alphaValue, 'EdgeColor', 'none', 'HandleVisibility', 'off');
end
uistack(findobj(ax, 'Type', 'line'), 'top');
uistack(findobj(ax, 'Type', 'scatter'), 'top');
ylim(ax, yl);
end

function runs = maskRuns(mask)
mask = logical(mask(:));
edges = diff([false; mask; false]);
starts = find(edges == 1);
ends = find(edges == -1) - 1;
runs = [starts, ends];
end

function text = summaryString(data, result, cutoffHz)
dynamicMask = ~data.initialMask & ~data.terminalMask & isfinite(result.deltaM);
startMask = data.timeS >= data.dynamicStartTimeS & data.timeS <= data.dynamicStartTimeS + 10.0;
terminalMask = data.terminalMask & isfinite(result.deltaM);
text = sprintf(['source=%s | cutoff=%.5g Hz | max abs delta dynamic=%.4g m | ' ...
  'first 10s=%.4g m | terminal=%.4g m | filter states %d..%d'], ...
  data.sourceName, ...
  cutoffHz, ...
  finiteMaxAbs(result.deltaM(dynamicMask)), ...
  finiteMaxAbs(result.deltaM(startMask)), ...
  finiteMaxAbs(result.deltaM(terminalMask)), ...
  result.firstFilterIndex, ...
  result.lastFilterIndex);
end

function value = finiteMaxAbs(values)
values = values(isfinite(values));
if isempty(values)
  value = nan;
else
  value = max(abs(values));
end
end

function text = formatCutoff(value)
text = sprintf('%.5g', value);
end

function value = onOff(condition)
if condition
  value = 'on';
else
  value = 'off';
end
end
