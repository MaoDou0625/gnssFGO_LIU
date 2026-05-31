function app = stage2_lowpass_frequency_tuner(resultDir, varargin)
%STAGE2_LOWPASS_FREQUENCY_TUNER Interactive Stage2 vertical smoothing tuner.
%
%   stage2_lowpass_frequency_tuner()
%   stage2_lowpass_frequency_tuner(resultDir)
%   app = stage2_lowpass_frequency_tuner(resultDir, 'Visible', 'off')
%
% The tool reads the current vertical solution from resultDir. It can compare
% time-domain lowpass smoothing, boundary-blended lowpass smoothing, spatial
% penalized least-squares baselines, and spline baselines. All methods export
% a reference sampled on the original time_s timeline for later Stage3 use.

if nargin < 1 || isempty(resultDir)
  resultDir = defaultResultDir();
end

options = parseOptions(varargin{:});
data = loadStage2LowpassData(resultDir);

state.rangeMinHz = options.MinCutoffHz;
state.rangeMaxHz = options.MaxCutoffHz;
state.cutoffHz = min(max(options.InitialCutoffHz, state.rangeMinHz), state.rangeMaxHz);
state.methodValues = ["time_lowpass","boundary_lowpass","spatial_pls","spline_baseline"];
state.methodLabels = {'time lowpass','boundary lowpass','spatial PLS','spline baseline'};
state.method = state.methodValues(1);
state.smoothLambda = options.InitialSmoothLambda;
state.anchorWeight = options.InitialAnchorWeight;
state.slopeWeight = options.InitialSlopeWeight;
state.transitionDurationS = options.InitialTransitionDurationS;
state.knotSpacing = options.InitialKnotSpacing;
state.lambdaRange = [1.0e2, 1.0e10];
state.anchorRange = [1.0e2, 1.0e8];
state.slopeRange = [1.0e-2, 1.0e7];
state.knotRange = [2.0, 100.0];
state.useStationDomain = true;
state.excludeStatic = true;
state.protectInitialDynamicStatic = true;
state.showRtkScatter = false;
state.showStage2Up = true;
state.showTunedReference = true;
state.showSolverReference = data.hasSolverReference;

fig = figure( ...
  'Name', 'Stage2 Smoothing Tuner', ...
  'NumberTitle', 'off', ...
  'Color', 'w', ...
  'Visible', options.Visible, ...
  'Position', [60 60 1500 900]);

axFull = axes('Parent', fig, 'Position', [0.06 0.52 0.88 0.25]);
axDelta = axes('Parent', fig, 'Position', [0.06 0.315 0.88 0.13]);
axStart = axes('Parent', fig, 'Position', [0.06 0.07 0.42 0.20]);
axEnd = axes('Parent', fig, 'Position', [0.52 0.07 0.42 0.20]);

uicontrol(fig, 'Style', 'text', 'String', 'Method', ...
  'Units', 'normalized', 'Position', [0.06 0.947 0.05 0.025], ...
  'BackgroundColor', 'w', 'HorizontalAlignment', 'left');
methodPopup = uicontrol(fig, 'Style', 'popupmenu', ...
  'Units', 'normalized', 'Position', [0.11 0.944 0.12 0.032], ...
  'String', state.methodLabels, ...
  'Value', 1, ...
  'Callback', @onMethodPopup);
uicontrol(fig, 'Style', 'text', 'String', 'Cutoff Hz', ...
  'Units', 'normalized', 'Position', [0.25 0.947 0.06 0.025], ...
  'BackgroundColor', 'w', 'HorizontalAlignment', 'left');
cutoffSlider = uicontrol(fig, 'Style', 'slider', ...
  'Units', 'normalized', 'Position', [0.31 0.947 0.20 0.025], ...
  'Min', log10(state.rangeMinHz), ...
  'Max', log10(state.rangeMaxHz), ...
  'Value', log10(state.cutoffHz), ...
  'Callback', @onCutoffSlider);
cutoffEdit = uicontrol(fig, 'Style', 'edit', ...
  'Units', 'normalized', 'Position', [0.52 0.944 0.06 0.032], ...
  'String', formatCutoff(state.cutoffHz), ...
  'Callback', @onCutoffEdit);
presetPopup = uicontrol(fig, 'Style', 'popupmenu', ...
  'Units', 'normalized', 'Position', [0.59 0.944 0.08 0.032], ...
  'String', {'0.005','0.01','0.02','0.03','0.05','0.08','0.10','0.20','0.50','1.0','2.0','5.0'}, ...
  'Value', 2, ...
  'Callback', @onPreset);
staticCheckbox = uicontrol(fig, 'Style', 'checkbox', ...
  'Units', 'normalized', 'Position', [0.69 0.947 0.13 0.025], ...
  'String', 'exclude static ranges', ...
  'Value', state.excludeStatic, ...
  'BackgroundColor', 'w', ...
  'Callback', @onStaticCheckbox);
initialWindowCheckbox = uicontrol(fig, 'Style', 'checkbox', ...
  'Units', 'normalized', 'Position', [0.82 0.947 0.15 0.025], ...
  'String', 'protect initial dynamic static', ...
  'Value', state.protectInitialDynamicStatic, ...
  'BackgroundColor', 'w', ...
  'Callback', @onInitialWindowCheckbox);

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
uicontrol(fig, 'Style', 'text', 'String', 'lambda', ...
  'Units', 'normalized', 'Position', [0.285 0.907 0.045 0.025], ...
  'BackgroundColor', 'w', 'HorizontalAlignment', 'left');
lambdaEdit = uicontrol(fig, 'Style', 'edit', ...
  'Units', 'normalized', 'Position', [0.33 0.904 0.06 0.032], ...
  'String', formatParameter(state.smoothLambda), ...
  'TooltipString', 'Baseline smoothness weight. Larger values reduce curvature and usually lower IRI.', ...
  'Callback', @onParameterEdit);
uicontrol(fig, 'Style', 'text', 'String', 'anchor', ...
  'Units', 'normalized', 'Position', [0.405 0.907 0.045 0.025], ...
  'BackgroundColor', 'w', 'HorizontalAlignment', 'left');
anchorEdit = uicontrol(fig, 'Style', 'edit', ...
  'Units', 'normalized', 'Position', [0.45 0.904 0.06 0.032], ...
  'String', formatParameter(state.anchorWeight), ...
  'TooltipString', 'Endpoint height anchor weight. Larger values hold start/end height more tightly.', ...
  'Callback', @onParameterEdit);
uicontrol(fig, 'Style', 'text', 'String', 'slope', ...
  'Units', 'normalized', 'Position', [0.525 0.907 0.04 0.025], ...
  'BackgroundColor', 'w', 'HorizontalAlignment', 'left');
slopeEdit = uicontrol(fig, 'Style', 'edit', ...
  'Units', 'normalized', 'Position', [0.565 0.904 0.055 0.032], ...
  'String', formatParameter(state.slopeWeight), ...
  'TooltipString', 'Endpoint slope weight. Larger values make the baseline enter/leave static ranges flatter.', ...
  'Callback', @onParameterEdit);
uicontrol(fig, 'Style', 'text', 'String', 'transition s', ...
  'Units', 'normalized', 'Position', [0.635 0.907 0.07 0.025], ...
  'BackgroundColor', 'w', 'HorizontalAlignment', 'left');
transitionEdit = uicontrol(fig, 'Style', 'edit', ...
  'Units', 'normalized', 'Position', [0.705 0.904 0.055 0.032], ...
  'String', formatParameter(state.transitionDurationS), ...
  'TooltipString', 'Boundary lowpass raised-cosine transition duration in seconds.', ...
  'Callback', @onParameterEdit);
uicontrol(fig, 'Style', 'text', 'String', 'knot x', ...
  'Units', 'normalized', 'Position', [0.775 0.907 0.06 0.025], ...
  'BackgroundColor', 'w', 'HorizontalAlignment', 'left');
knotEdit = uicontrol(fig, 'Style', 'edit', ...
  'Units', 'normalized', 'Position', [0.835 0.904 0.055 0.032], ...
  'String', formatParameter(state.knotSpacing), ...
  'TooltipString', 'Spline control-point spacing. Larger spacing gives a lower-frequency baseline.', ...
  'Callback', @onParameterEdit);
stationDomainCheckbox = uicontrol(fig, 'Style', 'checkbox', ...
  'Units', 'normalized', 'Position', [0.90 0.910 0.08 0.025], ...
  'String', 'station x', ...
  'Value', state.useStationDomain, ...
  'BackgroundColor', 'w', ...
  'Callback', @onStationDomainCheckbox);

uicontrol(fig, 'Style', 'pushbutton', 'String', 'Save PNG', ...
  'Units', 'normalized', 'Position', [0.06 0.864 0.07 0.032], ...
  'Callback', @onSavePng);
uicontrol(fig, 'Style', 'pushbutton', 'String', 'Save FIG', ...
  'Units', 'normalized', 'Position', [0.14 0.864 0.07 0.032], ...
  'Callback', @onSaveFig);
uicontrol(fig, 'Style', 'pushbutton', 'String', 'Export CSV', ...
  'Units', 'normalized', 'Position', [0.22 0.864 0.08 0.032], ...
  'Callback', @onExportCsv);
rtkScatterCheckbox = uicontrol(fig, 'Style', 'checkbox', ...
  'Units', 'normalized', 'Position', [0.32 0.870 0.09 0.025], ...
  'String', 'RTK scatter', ...
  'Value', state.showRtkScatter, ...
  'BackgroundColor', 'w', ...
  'Callback', @onRtkScatterCheckbox);
stage2UpCheckbox = uicontrol(fig, 'Style', 'checkbox', ...
  'Units', 'normalized', 'Position', [0.41 0.870 0.09 0.025], ...
  'String', 'Stage2 up', ...
  'Value', state.showStage2Up, ...
  'BackgroundColor', 'w', ...
  'Callback', @onStage2UpCheckbox);
tunedReferenceCheckbox = uicontrol(fig, 'Style', 'checkbox', ...
  'Units', 'normalized', 'Position', [0.50 0.870 0.11 0.025], ...
  'String', 'tuned smooth', ...
  'Value', state.showTunedReference, ...
  'BackgroundColor', 'w', ...
  'Callback', @onTunedReferenceCheckbox);
solverReferenceCheckbox = uicontrol(fig, 'Style', 'checkbox', ...
  'Units', 'normalized', 'Position', [0.61 0.870 0.10 0.025], ...
  'String', 'solver ref', ...
  'Value', state.showSolverReference, ...
  'Enable', onOff(data.hasSolverReference), ...
  'BackgroundColor', 'w', ...
  'Callback', @onSolverReferenceCheckbox);
uicontrol(fig, 'Style', 'text', 'String', 'Baseline preset', ...
  'Units', 'normalized', 'Position', [0.72 0.870 0.08 0.025], ...
  'BackgroundColor', 'w', 'HorizontalAlignment', 'left');
baselinePresetPopup = uicontrol(fig, 'Style', 'popupmenu', ...
  'Units', 'normalized', 'Position', [0.80 0.864 0.17 0.032], ...
  'String', {'custom','PLS balanced','PLS very smooth','PLS tight ends', ...
    'spline 10m knots','spline 20m knots','spline 50m knots'}, ...
  'Value', 1, ...
  'Callback', @onBaselinePreset);

uicontrol(fig, 'Style', 'text', 'String', 'lambda', ...
  'Units', 'normalized', 'Position', [0.06 0.830 0.045 0.022], ...
  'BackgroundColor', 'w', 'HorizontalAlignment', 'left');
lambdaSlider = uicontrol(fig, 'Style', 'slider', ...
  'Units', 'normalized', 'Position', [0.105 0.832 0.14 0.022], ...
  'Min', log10(state.lambdaRange(1)), ...
  'Max', log10(state.lambdaRange(2)), ...
  'Value', log10(state.smoothLambda), ...
  'TooltipString', 'Baseline smoothness. Move right for a lower-curvature baseline.', ...
  'Callback', @onBaselineSlider);
uicontrol(fig, 'Style', 'text', 'String', 'anchor', ...
  'Units', 'normalized', 'Position', [0.27 0.830 0.045 0.022], ...
  'BackgroundColor', 'w', 'HorizontalAlignment', 'left');
anchorSlider = uicontrol(fig, 'Style', 'slider', ...
  'Units', 'normalized', 'Position', [0.315 0.832 0.14 0.022], ...
  'Min', log10(state.anchorRange(1)), ...
  'Max', log10(state.anchorRange(2)), ...
  'Value', log10(state.anchorWeight), ...
  'TooltipString', 'Endpoint height anchoring. Move right to hold start/end height more tightly.', ...
  'Callback', @onBaselineSlider);
uicontrol(fig, 'Style', 'text', 'String', 'slope', ...
  'Units', 'normalized', 'Position', [0.48 0.830 0.04 0.022], ...
  'BackgroundColor', 'w', 'HorizontalAlignment', 'left');
slopeSlider = uicontrol(fig, 'Style', 'slider', ...
  'Units', 'normalized', 'Position', [0.52 0.832 0.14 0.022], ...
  'Min', log10(state.slopeRange(1)), ...
  'Max', log10(state.slopeRange(2)), ...
  'Value', log10(state.slopeWeight), ...
  'TooltipString', 'Endpoint slope anchoring. Move right to make the dynamic-static transition flatter.', ...
  'Callback', @onBaselineSlider);
uicontrol(fig, 'Style', 'text', 'String', 'knot', ...
  'Units', 'normalized', 'Position', [0.685 0.830 0.04 0.022], ...
  'BackgroundColor', 'w', 'HorizontalAlignment', 'left');
knotSlider = uicontrol(fig, 'Style', 'slider', ...
  'Units', 'normalized', 'Position', [0.725 0.832 0.14 0.022], ...
  'Min', state.knotRange(1), ...
  'Max', state.knotRange(2), ...
  'Value', state.knotSpacing, ...
  'TooltipString', 'Spline knot spacing in station meters or seconds. Move right for fewer control points.', ...
  'Callback', @onBaselineSlider);
statusText = uicontrol(fig, 'Style', 'text', ...
  'Units', 'normalized', 'Position', [0.06 0.775 0.88 0.04], ...
  'String', '', ...
  'BackgroundColor', 'w', ...
  'HorizontalAlignment', 'left');

refreshMethodControls();
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

  function onMethodPopup(~, ~)
    state.method = state.methodValues(get(methodPopup, 'Value'));
    set(baselinePresetPopup, 'Value', 1);
    refreshMethodControls();
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

  function onParameterEdit(~, ~)
    smoothLambda = str2double(get(lambdaEdit, 'String'));
    anchorWeight = str2double(get(anchorEdit, 'String'));
    slopeWeight = str2double(get(slopeEdit, 'String'));
    transitionDurationS = str2double(get(transitionEdit, 'String'));
    knotSpacing = str2double(get(knotEdit, 'String'));
    if isfinite(smoothLambda) && smoothLambda >= 0
      state.smoothLambda = smoothLambda;
    end
    if isfinite(anchorWeight) && anchorWeight >= 0
      state.anchorWeight = anchorWeight;
    end
    if isfinite(slopeWeight) && slopeWeight >= 0
      state.slopeWeight = slopeWeight;
    end
    if isfinite(transitionDurationS) && transitionDurationS >= 0
      state.transitionDurationS = transitionDurationS;
    end
    if isfinite(knotSpacing) && knotSpacing > 0
      state.knotSpacing = knotSpacing;
    end
    set(baselinePresetPopup, 'Value', 1);
    refreshParameterControls();
    current = recomputeAndPlot();
  end

  function onBaselineSlider(~, ~)
    state.smoothLambda = sliderLogValue(lambdaSlider, state.lambdaRange);
    state.anchorWeight = sliderLogValue(anchorSlider, state.anchorRange);
    state.slopeWeight = sliderLogValue(slopeSlider, state.slopeRange);
    state.knotSpacing = get(knotSlider, 'Value');
    set(baselinePresetPopup, 'Value', 1);
    refreshParameterControls();
    current = recomputeAndPlot();
  end

  function onBaselinePreset(~, ~)
    switch get(baselinePresetPopup, 'Value')
      case 2
        applyBaselinePreset("spatial_pls", 1.0e6, 1.0e5, 1.0e3, 20.0);
      case 3
        applyBaselinePreset("spatial_pls", 1.0e8, 1.0e5, 1.0e4, 20.0);
      case 4
        applyBaselinePreset("spatial_pls", 1.0e6, 1.0e7, 1.0e5, 20.0);
      case 5
        applyBaselinePreset("spline_baseline", 1.0e4, 1.0e5, 1.0e3, 10.0);
      case 6
        applyBaselinePreset("spline_baseline", 1.0e4, 1.0e5, 1.0e3, 20.0);
      case 7
        applyBaselinePreset("spline_baseline", 1.0e4, 1.0e5, 1.0e3, 50.0);
      otherwise
        refreshMethodControls();
        return;
    end
    current = recomputeAndPlot();
  end

  function applyBaselinePreset(method, smoothLambda, anchorWeight, slopeWeight, knotSpacing)
    state.method = method;
    state.smoothLambda = smoothLambda;
    state.anchorWeight = anchorWeight;
    state.slopeWeight = slopeWeight;
    state.knotSpacing = knotSpacing;
    methodIndex = find(state.methodValues == state.method, 1, 'first');
    if ~isempty(methodIndex)
      set(methodPopup, 'Value', methodIndex);
    end
    refreshMethodControls();
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

  function onStationDomainCheckbox(~, ~)
    state.useStationDomain = logical(get(stationDomainCheckbox, 'Value'));
    current = recomputeAndPlot();
  end

  function onSavePng(~, ~)
    defaultName = fullfile(resultDir, sprintf('stage2_%s_tuned.png', char(state.method)));
    [fileName, pathName] = uiputfile('*.png', 'Save PNG', defaultName);
    if isequal(fileName, 0)
      return;
    end
    exportgraphics(fig, fullfile(pathName, fileName), 'Resolution', 180);
  end

  function onSaveFig(~, ~)
    defaultName = fullfile(resultDir, sprintf('stage2_%s_tuned.fig', char(state.method)));
    [fileName, pathName] = uiputfile('*.fig', 'Save FIG', defaultName);
    if isequal(fileName, 0)
      return;
    end
    savefig(fig, fullfile(pathName, fileName));
  end

  function onExportCsv(~, ~)
    defaultName = fullfile(resultDir, sprintf('stage2_%s_reference.csv', char(state.method)));
    [fileName, pathName] = uiputfile('*.csv', 'Export tuned reference CSV', defaultName);
    if isequal(fileName, 0)
      return;
    end
    methodColumn = repmat(string(current.method), numel(data.timeS), 1);
    out = table( ...
      data.timeS(:), ...
      data.relativeTimeS(:), ...
      data.stationM(:), ...
      data.stage2UpM(:), ...
      current.referenceUpM(:), ...
      current.referenceUpM(:), ...
      current.deltaM(:), ...
      data.skipReason(:), ...
      methodColumn(:), ...
      'VariableNames', {'time_s','relative_time_s','station_m','stage2_up_m', ...
        'stage2_lowpass_up_m','smoothed_reference_up_m','lowpass_delta_m', ...
        'skip_reason','smoothing_method'});
    writetable(out, fullfile(pathName, fileName));
  end

  function result = recomputeAndPlot()
    result = stage2_vertical_smoothing_engine(data, smoothingParamsFromState());
    drawPlots(data, result, state, axFull, axDelta, axStart, axEnd);
    set(statusText, 'String', summaryString(data, result, state));
    drawnow limitrate;
  end

  function params = smoothingParamsFromState()
    params = struct();
    params.Method = state.method;
    params.CutoffHz = state.cutoffHz;
    params.SmoothLambda = state.smoothLambda;
    params.AnchorWeight = state.anchorWeight;
    params.SlopeWeight = state.slopeWeight;
    params.TransitionDurationS = state.transitionDurationS;
    params.KnotSpacing = state.knotSpacing;
    params.ExcludeStatic = state.excludeStatic;
    params.ProtectInitialDynamicStatic = state.protectInitialDynamicStatic;
    params.UseStationDomain = state.useStationDomain;
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

  function refreshParameterControls()
    set(lambdaEdit, 'String', formatParameter(state.smoothLambda));
    set(anchorEdit, 'String', formatParameter(state.anchorWeight));
    set(slopeEdit, 'String', formatParameter(state.slopeWeight));
    set(transitionEdit, 'String', formatParameter(state.transitionDurationS));
    set(knotEdit, 'String', formatParameter(state.knotSpacing));
    set(lambdaSlider, 'Value', logSliderValue(state.smoothLambda, state.lambdaRange));
    set(anchorSlider, 'Value', logSliderValue(state.anchorWeight, state.anchorRange));
    set(slopeSlider, 'Value', logSliderValue(state.slopeWeight, state.slopeRange));
    set(knotSlider, 'Value', min(max(state.knotSpacing, state.knotRange(1)), state.knotRange(2)));
    set(stationDomainCheckbox, 'Value', state.useStationDomain);
  end

  function refreshMethodControls()
    usesLowpass = state.method == "time_lowpass" || state.method == "boundary_lowpass";
    usesBoundaryBlend = state.method == "boundary_lowpass";
    usesBaseline = state.method == "spatial_pls" || state.method == "spline_baseline";
    usesSpline = state.method == "spline_baseline";
    set(cutoffSlider, 'Enable', onOff(usesLowpass));
    set(cutoffEdit, 'Enable', onOff(usesLowpass));
    set(presetPopup, 'Enable', onOff(usesLowpass));
    set(rangeMinEdit, 'Enable', onOff(usesLowpass));
    set(rangeMaxEdit, 'Enable', onOff(usesLowpass));
    set(lambdaEdit, 'Enable', onOff(usesBaseline));
    set(anchorEdit, 'Enable', onOff(usesBaseline));
    set(slopeEdit, 'Enable', onOff(usesBaseline));
    set(transitionEdit, 'Enable', onOff(usesBoundaryBlend));
    set(knotEdit, 'Enable', onOff(usesSpline));
    set(baselinePresetPopup, 'Enable', onOff(usesBaseline));
    set(lambdaSlider, 'Enable', onOff(usesBaseline));
    set(anchorSlider, 'Enable', onOff(usesBaseline));
    set(slopeSlider, 'Enable', onOff(usesBaseline));
    set(knotSlider, 'Enable', onOff(usesSpline));
    set(stationDomainCheckbox, 'Enable', onOff(usesBaseline));
    refreshParameterControls();
  end
end

function options = parseOptions(varargin)
options.Visible = 'on';
options.InitialCutoffHz = 0.01;
options.MinCutoffHz = 0.003;
options.MaxCutoffHz = 5.0;
options.InitialSmoothLambda = 1.0e4;
options.InitialAnchorWeight = 1.0e5;
options.InitialSlopeWeight = 1.0e3;
options.InitialTransitionDurationS = 5.0;
options.InitialKnotSpacing = 20.0;
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
    case "initialsmoothlambda"
      options.InitialSmoothLambda = value;
    case "initialanchorweight"
      options.InitialAnchorWeight = value;
    case "initialslopeweight"
      options.InitialSlopeWeight = value;
    case "initialtransitiondurations"
      options.InitialTransitionDurationS = value;
    case "initialknotspacing"
      options.InitialKnotSpacing = value;
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
data.stationM = nan(height(T), 1);
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
data.stationM = trajectoryStation(T);
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
if ~isfield(data, 'stationM') || numel(data.stationM) ~= numel(data.timeS) || ~any(isfinite(data.stationM))
  data.stationM = readTrajectoryStationForTimes(resultDir, data.timeS);
end
data.hasSolverReference = any(isfinite(data.solverReferenceUpM));
end

function stationM = trajectoryStation(T)
stationM = nan(height(T), 1);
if ~all(ismember({'east_m','north_m'}, T.Properties.VariableNames))
  return;
end
eastM = T.east_m;
northM = T.north_m;
valid = isfinite(eastM) & isfinite(northM);
if ~any(valid)
  return;
end
stepM = zeros(height(T), 1);
stepM(2:end) = hypot(diff(eastM), diff(northM));
stepM(~isfinite(stepM)) = 0.0;
stationM = cumsum(stepM);
if any(~valid)
  stationM(~valid) = nan;
end
end

function stationM = readTrajectoryStationForTimes(resultDir, targetTimeS)
stationM = nan(size(targetTimeS));
trajectoryPath = fullfile(resultDir, 'trajectory.csv');
if ~isfile(trajectoryPath)
  return;
end
T = readtable(trajectoryPath, 'TextType', 'string');
if ~all(ismember({'time_s','east_m','north_m'}, T.Properties.VariableNames))
  return;
end
sourceStationM = trajectoryStation(T);
valid = isfinite(T.time_s) & isfinite(sourceStationM);
if sum(valid) < 2
  return;
end
[sourceTimeS, uniqueIndex] = unique(T.time_s(valid), 'stable');
sourceStationM = sourceStationM(valid);
sourceStationM = sourceStationM(uniqueIndex);
stationM = interp1(sourceTimeS, sourceStationM, targetTimeS, 'linear', nan);
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

function drawPlots(data, result, state, axFull, axDelta, axStart, axEnd)
drawReferenceAxes(axFull, data, result, state, 'Full Stage2 vertical reference');
drawDeltaAxes(axDelta, data, result, state, 'Reference delta');

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
  legendItems{end + 1} = char(result.methodLabel);
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

function text = summaryString(data, result, state)
if result.params.Method == "time_lowpass"
  methodParam = sprintf('cutoff=%.5g Hz', state.cutoffHz);
elseif result.params.Method == "boundary_lowpass"
  methodParam = sprintf('cutoff=%.5g Hz, transition=%.4g s', ...
    state.cutoffHz, state.transitionDurationS);
elseif result.params.Method == "spatial_pls"
  methodParam = sprintf('lambda=%.4g, anchor=%.4g, slope=%.4g', ...
    state.smoothLambda, state.anchorWeight, state.slopeWeight);
else
  methodParam = sprintf('lambda=%.4g, anchor=%.4g, slope=%.4g, knot=%.4g', ...
    state.smoothLambda, state.anchorWeight, state.slopeWeight, state.knotSpacing);
end
text = sprintf(['source=%s | method=%s (%s, x=%s) | max abs delta dynamic=%.4g m | ' ...
  'first 10s=%.4g m | terminal=%.4g m | curv proxy=%.4g | states %d..%d'], ...
  data.sourceName, ...
  char(result.methodLabel), ...
  methodParam, ...
  char(result.domainName), ...
  result.metrics.maxAbsDeltaDynamicM, ...
  result.metrics.maxAbsDeltaStart10sM, ...
  result.metrics.maxAbsDeltaTerminalM, ...
  result.metrics.curvatureProxyM, ...
  result.firstFilterIndex, ...
  result.lastFilterIndex);
end

function text = formatCutoff(value)
text = sprintf('%.5g', value);
end

function text = formatParameter(value)
text = sprintf('%.5g', value);
end

function value = logSliderValue(parameterValue, range)
parameterValue = min(max(parameterValue, range(1)), range(2));
value = log10(parameterValue);
end

function value = sliderLogValue(slider, range)
value = 10 .^ get(slider, 'Value');
value = min(max(value, range(1)), range(2));
end

function value = onOff(condition)
if condition
  value = 'on';
else
  value = 'off';
end
end
