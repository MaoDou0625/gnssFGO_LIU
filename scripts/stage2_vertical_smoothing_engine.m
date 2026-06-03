function result = stage2_vertical_smoothing_engine(data, params)
%STAGE2_VERTICAL_SMOOTHING_ENGINE Build Stage2-derived vertical references.
%
% The output is always sampled on the original time_s timeline so the result
% can be exported as a Stage3 vertical reference candidate.
%
% Use stage2_lowpass_frequency_tuner() for the interactive GUI. Calling this
% helper without input arguments opens that GUI for convenience.

if nargin == 0
  if nargout > 0
    error(['stage2_vertical_smoothing_engine requires data and params when an output is requested. ' ...
      'Use stage2_lowpass_frequency_tuner() for the interactive GUI.']);
  end
  stage2_lowpass_frequency_tuner();
  return;
end

if nargin < 2 || isempty(params)
  params = struct();
end

params = normalizeParams(params);
timeS = data.timeS(:);
stage2UpM = data.stage2UpM(:);

inputUpM = stage2UpM;
if params.ProtectInitialDynamicStatic
  inputUpM = applyInitialDynamicStaticWindows(data, inputUpM, false);
end

[firstIndex, lastIndex] = smoothingRange(data, params.ExcludeStatic);
[domainX, domainName] = selectSmoothingDomain(data, firstIndex, lastIndex, params.UseStationDomain);

switch params.Method
  case "time_lowpass"
    domainName = "time";
    referenceUpM = buildLowpassProfile(timeS, inputUpM, params.CutoffHz, firstIndex, lastIndex);
  case "boundary_lowpass"
    domainName = "time";
    referenceUpM = buildLowpassProfile(timeS, inputUpM, params.CutoffHz, firstIndex, lastIndex);
    referenceUpM = applyBoundaryBlend(data, inputUpM, referenceUpM, firstIndex, lastIndex, params.TransitionDurationS);
  case "spatial_pls"
    referenceUpM = fitPenalizedBaseline(data, domainX, inputUpM, firstIndex, lastIndex, params);
  case "spline_baseline"
    referenceUpM = fitSplineBaseline(data, domainX, inputUpM, firstIndex, lastIndex, params);
  otherwise
    error('Unsupported smoothing method: %s', params.Method);
end

if params.ProtectInitialDynamicStatic
  referenceUpM = applyInitialDynamicStaticWindows(data, referenceUpM, true);
end

result = struct();
result.referenceUpM = referenceUpM;
result.deltaM = referenceUpM - stage2UpM;
result.firstFilterIndex = firstIndex;
result.lastFilterIndex = lastIndex;
result.method = params.Method;
result.methodLabel = methodLabel(params.Method);
result.domainName = domainName;
result.params = params;
result.metrics = computeMetrics(data, result);
end

function params = normalizeParams(params)
if nargin < 1 || isempty(params)
  params = struct();
end
params.Method = normalizeMethod(getParam(params, 'Method', "time_lowpass"));
params.CutoffHz = positiveParam(getParam(params, 'CutoffHz', 0.01), 0.01);
params.SmoothLambda = nonnegativeParam(getParam(params, 'SmoothLambda', 1.0e4), 1.0e4);
params.AnchorWeight = nonnegativeParam(getParam(params, 'AnchorWeight', 1.0e5), 1.0e5);
params.SlopeWeight = nonnegativeParam(getParam(params, 'SlopeWeight', 1.0e3), 1.0e3);
params.TransitionDurationS = nonnegativeParam(getParam(params, 'TransitionDurationS', 5.0), 5.0);
params.KnotSpacing = positiveParam(getParam(params, 'KnotSpacing', 20.0), 20.0);
params.ExcludeStatic = logical(getParam(params, 'ExcludeStatic', true));
params.ProtectInitialDynamicStatic = logical(getParam(params, 'ProtectInitialDynamicStatic', true));
params.UseStationDomain = logical(getParam(params, 'UseStationDomain', true));
end

function value = getParam(params, name, defaultValue)
if isfield(params, name)
  value = params.(name);
else
  value = defaultValue;
end
end

function method = normalizeMethod(value)
method = lower(strrep(string(value), ' ', '_'));
switch method
  case {"lowpass", "time_lowpass", "time_domain_lowpass"}
    method = "time_lowpass";
  case {"boundary_lowpass", "edge_lowpass", "edge_blended_lowpass"}
    method = "boundary_lowpass";
  case {"spatial_pls", "pls", "spatial_smooth_baseline", "whittaker"}
    method = "spatial_pls";
  case {"spline", "spline_baseline", "spatial_spline"}
    method = "spline_baseline";
  otherwise
    error('Unsupported smoothing method: %s', value);
end
end

function value = positiveParam(value, defaultValue)
value = double(value);
if ~isfinite(value) || value <= 0
  value = defaultValue;
end
end

function value = nonnegativeParam(value, defaultValue)
value = double(value);
if ~isfinite(value) || value < 0
  value = defaultValue;
end
end

function label = methodLabel(method)
switch string(method)
  case "time_lowpass"
    label = "time lowpass";
  case "boundary_lowpass"
    label = "boundary lowpass";
  case "spatial_pls"
    label = "spatial PLS baseline";
  case "spline_baseline"
    label = "spline baseline";
  otherwise
    label = string(method);
end
end

function [firstIndex, lastIndex] = smoothingRange(data, excludeStatic)
firstIndex = 1;
lastIndex = numel(data.timeS);
if ~excludeStatic
  return;
end
firstDynamic = find(~data.initialMask, 1, 'first');
firstTerminal = find(data.terminalMask, 1, 'first');
if ~isempty(firstDynamic)
  firstIndex = firstDynamic;
end
if ~isempty(firstTerminal)
  lastIndex = firstTerminal - 1;
end
lastIndex = max(firstIndex, lastIndex);
end

function [domainX, domainName] = selectSmoothingDomain(data, firstIndex, lastIndex, useStationDomain)
timeS = data.timeS(:);
domainX = timeS;
domainName = "time";
if ~useStationDomain || ~isfield(data, 'stationM')
  return;
end
stationM = data.stationM(:);
if numel(stationM) ~= numel(timeS)
  return;
end
stationM = fillMissingByTime(timeS, stationM);
segment = firstIndex:lastIndex;
if numel(segment) < 2 || sum(isfinite(stationM(segment))) < 2
  return;
end
segmentStation = stationM(segment);
if max(segmentStation, [], 'omitnan') - min(segmentStation, [], 'omitnan') <= 1.0e-6
  return;
end
domainX = stationM;
domainName = "station";
end

function values = fillMissingByTime(timeS, values)
valid = isfinite(timeS) & isfinite(values);
if all(valid) || sum(valid) < 2
  return;
end
[uniqueTimeS, uniqueIndex] = unique(timeS(valid), 'stable');
uniqueValues = values(valid);
uniqueValues = uniqueValues(uniqueIndex);
if numel(uniqueTimeS) < 2
  return;
end
filled = interp1(uniqueTimeS, uniqueValues, timeS, 'linear', nan);
replace = ~isfinite(values) & isfinite(filled);
values(replace) = filled(replace);
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

function referenceUpM = applyBoundaryBlend(data, inputUpM, referenceUpM, firstIndex, lastIndex, transitionDurationS)
if transitionDurationS <= 0 || firstIndex >= lastIndex
  return;
end
timeS = data.timeS(:);
startTimeS = timeS(firstIndex);
endTimeS = timeS(lastIndex);
startValue = startBoundaryValue(data, inputUpM, firstIndex);
endValue = endBoundaryValue(data, inputUpM, lastIndex);

startMask = (1:numel(timeS))' >= firstIndex & (1:numel(timeS))' <= lastIndex & ...
  timeS >= startTimeS & timeS <= startTimeS + transitionDurationS & isfinite(referenceUpM);
if any(startMask)
  fraction = (timeS(startMask) - startTimeS) ./ transitionDurationS;
  weight = raisedCosine01(fraction);
  referenceUpM(startMask) = (1.0 - weight) .* startValue + weight .* referenceUpM(startMask);
end

endMask = (1:numel(timeS))' >= firstIndex & (1:numel(timeS))' <= lastIndex & ...
  timeS >= endTimeS - transitionDurationS & timeS <= endTimeS & isfinite(referenceUpM);
if any(endMask)
  fraction = (timeS(endMask) - (endTimeS - transitionDurationS)) ./ transitionDurationS;
  weight = raisedCosine01(fraction);
  referenceUpM(endMask) = (1.0 - weight) .* referenceUpM(endMask) + weight .* endValue;
end
end

function weight = raisedCosine01(fraction)
fraction = min(max(fraction, 0.0), 1.0);
weight = 0.5 - 0.5 .* cos(pi .* fraction);
end

function referenceUpM = fitPenalizedBaseline(data, domainX, inputUpM, firstIndex, lastIndex, params)
referenceUpM = inputUpM;
indices = (firstIndex:lastIndex)';
if numel(indices) < 3
  return;
end
z = inputUpM(indices);
valid = isfinite(z);
if ~any(valid)
  return;
end
n = numel(indices);
w = double(valid);
A = spdiags(w, 0, n, n);
rhs = zeros(n, 1);
rhs(valid) = z(valid);

if params.SmoothLambda > 0 && n >= 3
  D2 = spdiags(repmat([1 -2 1], n - 2, 1), 0:2, n - 2, n);
  A = A + params.SmoothLambda .* (D2' * D2);
end

[A, rhs] = addBoundaryTerms(data, domainX, inputUpM, indices, A, rhs, params);
A = A + speye(n) .* 1.0e-12;
referenceUpM(indices) = A \ rhs;
end

function referenceUpM = fitSplineBaseline(data, domainX, inputUpM, firstIndex, lastIndex, params)
referenceUpM = inputUpM;
indices = (firstIndex:lastIndex)';
if numel(indices) < 4
  return;
end
x = domainX(indices);
z = inputUpM(indices);
valid = isfinite(x) & isfinite(z);
if sum(valid) < 4
  referenceUpM = fitPenalizedBaseline(data, domainX, inputUpM, firstIndex, lastIndex, params);
  return;
end
x0 = min(x(valid));
x = x - x0;
xRange = max(x(valid));
if xRange <= 1.0e-6
  referenceUpM = fitPenalizedBaseline(data, domainX, inputUpM, firstIndex, lastIndex, params);
  return;
end
knots = 0:params.KnotSpacing:xRange;
if isempty(knots) || knots(end) < xRange
  knots(end + 1) = xRange;
end
if numel(knots) < 4
  knots = linspace(0.0, xRange, 4);
end
B = splineBasis(knots, min(max(x, 0.0), xRange));
k = size(B, 2);
Bobs = B(valid, :);
zobs = z(valid);
A = Bobs' * Bobs;
rhs = Bobs' * zobs;

if params.SmoothLambda > 0 && k >= 3
  D2 = spdiags(repmat([1 -2 1], k - 2, 1), 0:2, k - 2, k);
  A = A + params.SmoothLambda .* (D2' * D2);
end

[A, rhs] = addSplineBoundaryTerms(data, inputUpM, indices, B, A, rhs, params);
A = A + eye(k) .* 1.0e-12;
coeff = A \ rhs;
referenceUpM(indices) = B * coeff;
end

function B = splineBasis(knots, queryX)
k = numel(knots);
B = zeros(numel(queryX), k);
for j = 1:k
  coeff = zeros(1, k);
  coeff(j) = 1.0;
  B(:, j) = interp1(knots, coeff, queryX, 'spline', 'extrap');
end
end

function [A, rhs] = addBoundaryTerms(data, domainX, inputUpM, indices, A, rhs, params)
n = numel(indices);
if params.AnchorWeight > 0
  startValue = startBoundaryValue(data, inputUpM, indices(1));
  endValue = endBoundaryValue(data, inputUpM, indices(end));
  A(1, 1) = A(1, 1) + params.AnchorWeight;
  rhs(1) = rhs(1) + params.AnchorWeight * startValue;
  A(n, n) = A(n, n) + params.AnchorWeight;
  rhs(n) = rhs(n) + params.AnchorWeight * endValue;
end
if params.SlopeWeight > 0 && n >= 2
  startDx = finiteSpacing(domainX(indices(2)) - domainX(indices(1)));
  endDx = finiteSpacing(domainX(indices(end)) - domainX(indices(end - 1)));
  ds = sparse(1, [1 2], [-1 1] ./ startDx, 1, n);
  de = sparse(1, [n - 1 n], [-1 1] ./ endDx, 1, n);
  A = A + params.SlopeWeight .* (ds' * ds + de' * de);
end
end

function dx = finiteSpacing(dx)
dx = abs(dx);
if ~isfinite(dx) || dx <= 1.0e-6
  dx = 1.0;
end
end

function [A, rhs] = addSplineBoundaryTerms(data, inputUpM, indices, B, A, rhs, params)
if params.AnchorWeight > 0
  startValue = startBoundaryValue(data, inputUpM, indices(1));
  endValue = endBoundaryValue(data, inputUpM, indices(end));
  startBasis = B(1, :)';
  endBasis = B(end, :)';
  A = A + params.AnchorWeight .* (startBasis * startBasis' + endBasis * endBasis');
  rhs = rhs + params.AnchorWeight .* (startBasis * startValue + endBasis * endValue);
end
if params.SlopeWeight > 0 && size(B, 1) >= 2
  startSlope = (B(2, :) - B(1, :))';
  endSlope = (B(end, :) - B(end - 1, :))';
  A = A + params.SlopeWeight .* (startSlope * startSlope' + endSlope * endSlope');
end
end

function value = startBoundaryValue(data, inputUpM, firstIndex)
value = medianFinite(inputUpM(data.initialMask));
if isfinite(value)
  return;
end
if firstIndex > 1
  value = nearestFinite(inputUpM, firstIndex - 1, -1);
end
if ~isfinite(value)
  value = inputUpM(firstIndex);
end
end

function value = endBoundaryValue(data, inputUpM, lastIndex)
value = medianFinite(inputUpM(data.terminalMask));
if isfinite(value)
  return;
end
if lastIndex < numel(inputUpM)
  value = nearestFinite(inputUpM, lastIndex + 1, 1);
end
if ~isfinite(value)
  value = inputUpM(lastIndex);
end
end

function value = medianFinite(values)
values = values(isfinite(values));
if isempty(values)
  value = nan;
else
  value = median(values);
end
end

function value = nearestFinite(values, startIndex, direction)
value = nan;
index = startIndex;
while index >= 1 && index <= numel(values)
  if isfinite(values(index))
    value = values(index);
    return;
  end
  index = index + direction;
end
end

function outputUpM = applyInitialDynamicStaticWindows(data, inputUpM, protectOutput)
outputUpM = inputUpM;
if ~isfield(data, 'initialDynamicStaticWindows')
  return;
end
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

function metrics = computeMetrics(data, result)
metrics = struct();
dynamicMask = ~data.initialMask & ~data.terminalMask & isfinite(result.deltaM);
startMask = data.timeS >= data.dynamicStartTimeS & data.timeS <= data.dynamicStartTimeS + 10.0 & ...
  isfinite(result.deltaM);
terminalMask = data.terminalMask & isfinite(result.deltaM);
metrics.maxAbsDeltaDynamicM = finiteMaxAbs(result.deltaM(dynamicMask));
metrics.maxAbsDeltaStart10sM = finiteMaxAbs(result.deltaM(startMask));
metrics.maxAbsDeltaTerminalM = finiteMaxAbs(result.deltaM(terminalMask));
metrics.curvatureProxyM = curvatureProxy(data, result.referenceUpM, result.firstFilterIndex, result.lastFilterIndex);
end

function value = finiteMaxAbs(values)
values = values(isfinite(values));
if isempty(values)
  value = nan;
else
  value = max(abs(values));
end
end

function value = curvatureProxy(data, referenceUpM, firstIndex, lastIndex)
value = nan;
if ~isfield(data, 'stationM')
  return;
end
indices = (firstIndex:lastIndex)';
stationM = data.stationM(indices);
upM = referenceUpM(indices);
valid = isfinite(stationM) & isfinite(upM);
if sum(valid) < 5
  return;
end
stationM = stationM(valid);
upM = upM(valid);
[stationM, uniqueIndex] = unique(stationM, 'stable');
upM = upM(uniqueIndex);
if numel(stationM) < 5 || stationM(end) - stationM(1) < 1.0
  return;
end
grid = stationM(1):0.25:stationM(end);
if numel(grid) < 5
  return;
end
upGrid = interp1(stationM, upM, grid, 'linear');
d2 = diff(upGrid, 2);
value = sqrt(mean(d2 .^ 2, 'omitnan'));
end
