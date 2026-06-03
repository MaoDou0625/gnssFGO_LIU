function iriResult = stage2_iri50_calculator(data, referenceUpM, varargin)
%STAGE2_IRI50_CALCULATOR Compute 50 m IRI for a time-aligned vertical reference.
%
% The input reference is sampled on data.timeS. The calculator converts it to
% a station-domain profile, resamples at 0.25 m, and calls the local Michal
% Sorel IRI implementation used by docs/iri_50m_workflow.md.

options = parseOptions(varargin{:});
validateInputs(data, referenceUpM);

trim = selectTrimRange(data, referenceUpM, options);
profile = buildSpatialProfile(data.stationM(:), referenceUpM(:), trim, options);

outputDir = buildOutputDir(options.ResultDir, options.MethodName);
if ~exist(outputDir, 'dir')
  mkdir(outputDir);
end

spaceProfilePath = fullfile(outputDir, 'space_profile_0p25m.txt');
writeSpatialProfile(spaceProfilePath, profile);

rawIriPath = fullfile(outputDir, 'iri_50m.txt');
plotPath = fullfile(outputDir, 'iri_50m_michalsorel_plot.png');
runMichalSorelIri(options.IriScriptPath, spaceProfilePath, rawIriPath, plotPath, ...
  options.SegmentLengthM);

iriMatrix = parseIriText(rawIriPath);
iriTable = array2table(iriMatrix, ...
  'VariableNames', {'start_m','end_m','iri_mm_per_m','iri_std_mm_per_m'});
csvPath = fullfile(outputDir, 'iri_50m.csv');
writetable(iriTable, csvPath);

summaryPath = fullfile(outputDir, 'processing_summary.txt');
writeSummary(summaryPath, data, trim, profile, iriTable, options, ...
  spaceProfilePath, rawIriPath, csvPath, plotPath);

iriResult = struct();
iriResult.table = iriTable;
iriResult.outputDir = outputDir;
iriResult.csvPath = csvPath;
iriResult.rawIriPath = rawIriPath;
iriResult.spaceProfilePath = spaceProfilePath;
iriResult.plotPath = plotPath;
iriResult.summaryPath = summaryPath;
iriResult.trim = trim;
iriResult.meanIriMmPerM = mean(iriTable.iri_mm_per_m, 'omitnan');
iriResult.minIriMmPerM = min(iriTable.iri_mm_per_m, [], 'omitnan');
iriResult.maxIriMmPerM = max(iriTable.iri_mm_per_m, [], 'omitnan');
end

function options = parseOptions(varargin)
options.ResultDir = pwd;
options.MethodName = 'current';
options.FirstIndex = 1;
options.LastIndex = inf;
options.SpeedThresholdMps = 0.5;
options.UseSpeedTrim = true;
options.ResampleDxM = 0.25;
options.SegmentLengthM = 50.0;
options.IriScriptPath = 'D:\Code\michalsorel_iri_repo\python\iri.py';
if mod(numel(varargin), 2) ~= 0
  error('Options must be name-value pairs.');
end
for i = 1:2:numel(varargin)
  name = lower(string(varargin{i}));
  value = varargin{i + 1};
  switch name
    case "resultdir"
      options.ResultDir = char(value);
    case "methodname"
      options.MethodName = char(value);
    case "firstindex"
      options.FirstIndex = value;
    case "lastindex"
      options.LastIndex = value;
    case "speedthresholdmps"
      options.SpeedThresholdMps = value;
    case "usespeedtrim"
      options.UseSpeedTrim = logical(value);
    case "resampledxm"
      options.ResampleDxM = value;
    case "segmentlengthm"
      options.SegmentLengthM = value;
    case "iriscriptpath"
      options.IriScriptPath = char(value);
    otherwise
      error('Unknown option: %s', name);
  end
end
end

function validateInputs(data, referenceUpM)
requiredFields = {'timeS','stationM'};
for i = 1:numel(requiredFields)
  if ~isfield(data, requiredFields{i})
    error('data.%s is required for IRI computation.', requiredFields{i});
  end
end
if numel(referenceUpM) ~= numel(data.timeS)
  error('referenceUpM must have the same length as data.timeS.');
end
if numel(data.stationM) ~= numel(data.timeS)
  error('data.stationM must have the same length as data.timeS.');
end
end

function trim = selectTrimRange(data, referenceUpM, options)
n = numel(data.timeS);
firstIndex = max(1, min(n, round(options.FirstIndex)));
lastIndex = max(firstIndex, min(n, round(options.LastIndex)));
candidateMask = false(n, 1);
candidateMask(firstIndex:lastIndex) = true;
candidateMask = candidateMask & isfinite(data.stationM(:)) & isfinite(referenceUpM(:));

speedTrimApplied = false;
if options.UseSpeedTrim && isfield(data, 'speedMps') && numel(data.speedMps) == n
  speedMps = data.speedMps(:);
  moving = find(candidateMask & isfinite(speedMps) & speedMps >= options.SpeedThresholdMps);
  if ~isempty(moving)
    firstIndex = moving(1);
    lastIndex = moving(end);
    speedTrimApplied = true;
  end
end

trim = struct();
trim.firstIndex = firstIndex;
trim.lastIndex = lastIndex;
trim.startTimeS = data.timeS(firstIndex);
trim.endTimeS = data.timeS(lastIndex);
trim.startRawStationM = data.stationM(firstIndex);
trim.endRawStationM = data.stationM(lastIndex);
trim.speedTrimApplied = speedTrimApplied;
trim.speedThresholdMps = options.SpeedThresholdMps;
end

function profile = buildSpatialProfile(stationM, referenceUpM, trim, options)
indices = (trim.firstIndex:trim.lastIndex)';
station = stationM(indices);
up = referenceUpM(indices);
valid = isfinite(station) & isfinite(up);
station = station(valid);
up = up(valid);
if numel(station) < 3
  error('Too few valid samples for IRI computation after trimming.');
end

station = station - station(1);
up = up - up(1);
[station, uniqueIndex] = unique(station, 'stable');
up = up(uniqueIndex);
validStep = [true; diff(station) > 1.0e-9];
station = station(validStep);
up = up(validStep);
if station(end) < options.SegmentLengthM
  error('Retained distance %.3f m is shorter than one %.3f m IRI segment.', ...
    station(end), options.SegmentLengthM);
end

maxGridM = floor(station(end) / options.ResampleDxM) * options.ResampleDxM;
gridM = (0.0:options.ResampleDxM:maxGridM)';
upGridM = interp1(station, up, gridM, 'linear');
profile = struct();
profile.stationM = gridM;
profile.upM = upGridM;
profile.retainedDistanceM = station(end);
profile.tailNotIncludedM = station(end) - floor(station(end) / options.SegmentLengthM) * options.SegmentLengthM;
end

function writeSpatialProfile(path, profile)
fid = fopen(path, 'w');
if fid < 0
  error('Unable to write spatial profile: %s', path);
end
cleanup = onCleanup(@() fclose(fid));
for i = 1:numel(profile.stationM)
  fprintf(fid, '%.6f %.9f\n', profile.stationM(i), profile.upM(i));
end
delete(cleanup);
end

function runMichalSorelIri(iriScriptPath, spaceProfilePath, rawIriPath, plotPath, segmentLengthM)
if ~isfile(iriScriptPath)
  error('Missing IRI script: %s', iriScriptPath);
end
wrapperPath = fullfile(fileparts(rawIriPath), 'run_michalsorel_iri.py');
writePythonWrapper(wrapperPath);
args = {
  wrapperPath
  iriScriptPath
  spaceProfilePath
  rawIriPath
  '-segment_length'
  sprintf('%.15g', segmentLengthM)
  '-start_pos'
  '0'
  '-method'
  '2'
  '-plot_file'
  plotPath
  };
[status, output] = runPython(args);
if status ~= 0
  error('IRI Python command failed:\n%s', output);
end
end

function writePythonWrapper(path)
fid = fopen(path, 'w');
if fid < 0
  error('Unable to write Python wrapper: %s', path);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, 'import runpy\n');
fprintf(fid, 'import sys\n');
fprintf(fid, 'import numpy as np\n');
fprintf(fid, 'script = sys.argv[1]\n');
fprintf(fid, 'np.in1d = getattr(np, "in1d", np.isin)\n');
fprintf(fid, 'sys.argv = [script] + sys.argv[2:]\n');
fprintf(fid, 'runpy.run_path(script, run_name="__main__")\n');
delete(cleanup);
end

function [status, output] = runPython(args)
candidates = {'python', 'py -3'};
output = '';
for i = 1:numel(candidates)
  command = candidates{i} + " " + joinQuotedArgs(args);
  [status, output] = system(command);
  if status == 0
    return;
  end
end
end

function text = joinQuotedArgs(args)
quoted = cell(size(args));
for i = 1:numel(args)
  quoted{i} = quoteCommandArg(args{i});
end
text = strjoin(quoted, ' ');
end

function text = quoteCommandArg(value)
text = ['"', strrep(char(value), '"', '\"'), '"'];
end

function iriMatrix = parseIriText(path)
lines = splitlines(string(fileread(path)));
rows = zeros(0, 4);
for i = 1:numel(lines)
  line = strtrim(lines(i));
  if strlength(line) == 0 || startsWith(line, "#")
    continue;
  end
  values = sscanf(char(line), '%f');
  if numel(values) >= 4
    rows(end + 1, :) = values(1:4)'; %#ok<AGROW>
  end
end
if isempty(rows)
  error('IRI output did not contain any segment rows: %s', path);
end
iriMatrix = rows;
end

function outputDir = buildOutputDir(resultDir, methodName)
timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
safeMethod = regexprep(char(methodName), '[^A-Za-z0-9_]+', '_');
outputDir = fullfile(resultDir, sprintf('stage2_smoothing_iri50_%s_%s', safeMethod, timestamp));
end

function writeSummary(path, data, trim, profile, iriTable, options, ...
    spaceProfilePath, rawIriPath, csvPath, plotPath)
fid = fopen(path, 'w');
if fid < 0
  error('Unable to write IRI summary: %s', path);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, 'Stage2 smoothed reference 50m IRI summary\n');
fprintf(fid, 'source_count=%d\n', numel(data.timeS));
fprintf(fid, 'method_name=%s\n', options.MethodName);
fprintf(fid, 'speed_threshold_mps=%.9g\n', options.SpeedThresholdMps);
fprintf(fid, 'speed_trim_applied=%d\n', trim.speedTrimApplied);
fprintf(fid, 'trim_first_index=%d\n', trim.firstIndex);
fprintf(fid, 'trim_last_index=%d\n', trim.lastIndex);
fprintf(fid, 'trim_start_time_s=%.9f\n', trim.startTimeS);
fprintf(fid, 'trim_end_time_s=%.9f\n', trim.endTimeS);
fprintf(fid, 'trim_start_raw_station_m=%.9f\n', trim.startRawStationM);
fprintf(fid, 'trim_end_raw_station_m=%.9f\n', trim.endRawStationM);
fprintf(fid, 'retained_distance_m=%.9f\n', profile.retainedDistanceM);
fprintf(fid, 'resample_dx_m=%.9f\n', options.ResampleDxM);
fprintf(fid, 'segment_length_m=%.9f\n', options.SegmentLengthM);
fprintf(fid, 'complete_segments=%d\n', height(iriTable));
fprintf(fid, 'tail_not_included_m=%.9f\n', profile.tailNotIncludedM);
fprintf(fid, 'iri_mean_mm_per_m=%.9f\n', mean(iriTable.iri_mm_per_m, 'omitnan'));
fprintf(fid, 'iri_min_mm_per_m=%.9f\n', min(iriTable.iri_mm_per_m, [], 'omitnan'));
fprintf(fid, 'iri_max_mm_per_m=%.9f\n', max(iriTable.iri_mm_per_m, [], 'omitnan'));
fprintf(fid, 'space_profile=%s\n', spaceProfilePath);
fprintf(fid, 'raw_iri=%s\n', rawIriPath);
fprintf(fid, 'csv=%s\n', csvPath);
fprintf(fid, 'plot=%s\n', plotPath);
delete(cleanup);
end
