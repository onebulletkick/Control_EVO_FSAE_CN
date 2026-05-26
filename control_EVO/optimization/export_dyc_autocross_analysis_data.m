function [cfg, artifacts] = export_dyc_autocross_analysis_data(cfg, runResults)
%EXPORT_DYC_AUTOCROSS_ANALYSIS_DATA 导出可复用的 Autocross 时序分析数据。

cfg.signalDataDir = fullfile(cfg.resultsDir, 'signal_data');
cfg.signalManifestPath = fullfile(cfg.signalDataDir, 'signal_manifest.csv');
cfg.alignedComparisonPath = fullfile(cfg.signalDataDir, 'aligned_dyc_comparison.csv');
cfg.analysisDataMatPath = fullfile(cfg.signalDataDir, 'analysis_data.mat');

if ~isfolder(cfg.signalDataDir)
    mkdir(cfg.signalDataDir);
end

timeSeriesTables = repmat(struct('caseId', "", 'repeatIndex', NaN, 'table', table(), ...
    'csvPath', "", 'relativePath', ""), 0, 1);
manifest = localEmptyManifest();

for idx = 1:numel(runResults)
    run = runResults(idx);
    signalTable = localSignalsToTable(localSignalsField(run));
    caseId = localStringField(run, 'caseId');
    repeatIndex = localNumericField(run, 'repeatIndex');
    fileName = sprintf('%s_repeat%d_timeseries.csv', char(caseId), repeatIndex);
    csvPath = fullfile(cfg.signalDataDir, fileName);
    relativePath = "signal_data/" + string(fileName);

    if height(signalTable) > 0
        localWriteCsv(signalTable, csvPath);
        timeSeriesTables(end+1, 1) = struct('caseId', caseId, ...
            'repeatIndex', repeatIndex, 'table', signalTable, ...
            'csvPath', string(csvPath), 'relativePath', relativePath); %#ok<AGROW>
    else
        csvPath = "";
        relativePath = "";
    end

    manifest = [manifest; table(caseId, repeatIndex, localStringField(run, 'status'), ...
        localSignalSource(run), height(signalTable), relativePath, ...
        'VariableNames', manifest.Properties.VariableNames)]; %#ok<AGROW>
end

alignedComparison = localAlignedComparisonTable(runResults);
if height(alignedComparison) > 0
    localWriteCsv(alignedComparison, cfg.alignedComparisonPath);
end
localWriteCsv(manifest, cfg.signalManifestPath);

artifacts = struct();
artifacts.signalDataDir = string(cfg.signalDataDir);
artifacts.manifestPath = string(cfg.signalManifestPath);
artifacts.manifestRelativePath = "signal_data/signal_manifest.csv";
artifacts.alignedComparisonPath = string(cfg.alignedComparisonPath);
artifacts.alignedComparisonRelativePath = "signal_data/aligned_dyc_comparison.csv";
artifacts.analysisDataMatPath = string(cfg.analysisDataMatPath);
artifacts.analysisDataMatRelativePath = "signal_data/analysis_data.mat";
artifacts.manifest = manifest;
artifacts.alignedComparison = alignedComparison;
artifacts.timeSeriesTables = timeSeriesTables;

save(cfg.analysisDataMatPath, 'manifest', 'alignedComparison', 'timeSeriesTables', 'artifacts');
end

function localWriteCsv(tbl, filePath)
try
    writetable(tbl, filePath, 'QuoteStrings', 'all');
catch
    writetable(tbl, filePath);
end
end

function manifest = localEmptyManifest()
manifest = table('Size', [0 6], ...
    'VariableTypes', {'string','double','string','string','double','string'}, ...
    'VariableNames', {'caseId','repeatIndex','status','source','rowCount','relativePath'});
end

function signalTable = localSignalsToTable(signals)
fields = localExportSignalFields();
n = localSignalLength(signals, fields);
if n < 1
    signalTable = table();
    return;
end

signalTable = table();
for idx = 1:numel(fields)
    name = fields{idx};
    signalTable.(name) = localPaddedVector(signals, name, n);
end

signalTable.speed_kph = signalTable.speed_mps * 3.6;
signalTable.yawRateError_radps = localPairDelta(signalTable, 'yawRate_radps', 'yawRateTarget_radps');
signalTable.lateralError_m = localPairDelta(signalTable, 'latVeh_m', 'latTarget_m');
signalTable.steerSW_deg = rad2deg(signalTable.steerSW_rad);
signalTable.tireUtilL1 = localTireUtil(signalTable, 'L1');
signalTable.tireUtilL2 = localTireUtil(signalTable, 'L2');
signalTable.tireUtilR1 = localTireUtil(signalTable, 'R1');
signalTable.tireUtilR2 = localTireUtil(signalTable, 'R2');
signalTable.tireUtilMax = localRowMax([signalTable.tireUtilL1, signalTable.tireUtilL2, ...
    signalTable.tireUtilR1, signalTable.tireUtilR2]);
torqueMatrix = [signalTable.myDrL1_Nm, signalTable.myDrL2_Nm, ...
    signalTable.myDrR1_Nm, signalTable.myDrR2_Nm];
signalTable.driveTorqueSum_Nm = localRowSum(torqueMatrix);
signalTable.wheelTorqueSpread_Nm = localRowMax(torqueMatrix) - localRowMin(torqueMatrix);
signalTable.leftRightDriveTorqueDelta_Nm = ...
    (signalTable.myDrR1_Nm + signalTable.myDrR2_Nm) - ...
    (signalTable.myDrL1_Nm + signalTable.myDrL2_Nm);
signalTable.frontRearDriveTorqueDelta_Nm = ...
    (signalTable.myDrL1_Nm + signalTable.myDrR1_Nm) - ...
    (signalTable.myDrL2_Nm + signalTable.myDrR2_Nm);
end

function fields = localExportSignalFields()
fields = {'time_s','station_m','speed_mps','yawRate_radps','yawRateTarget_radps', ...
    'latVeh_m','latTarget_m','ay_mps2','ax_mps2','mz_Nm','throttle','steerSW_rad', ...
    'myDrL1_Nm','myDrL2_Nm','myDrR1_Nm','myDrR2_Nm', ...
    'tireFxL1_N','tireFxL2_N','tireFxR1_N','tireFxR2_N', ...
    'tireFyL1_N','tireFyL2_N','tireFyR1_N','tireFyR2_N', ...
    'tireFzL1_N','tireFzL2_N','tireFzR1_N','tireFzR2_N'};
end

function n = localSignalLength(signals, fields)
n = 0;
if ~isstruct(signals)
    return;
end
for idx = 1:numel(fields)
    if isfield(signals, fields{idx})
        n = max(n, numel(signals.(fields{idx})));
    end
end
end

function value = localPaddedVector(signals, fieldName, n)
value = NaN(n, 1);
if ~isstruct(signals) || ~isfield(signals, fieldName)
    return;
end
raw = double(signals.(fieldName));
raw = raw(:);
count = min(n, numel(raw));
if count > 0
    value(1:count) = raw(1:count);
end
if strcmp(fieldName, 'time_s') && ~any(isfinite(value))
    value = (0:n-1)';
end
end

function value = localPairDelta(tbl, actualName, targetName)
if ~ismember(actualName, tbl.Properties.VariableNames) || ~ismember(targetName, tbl.Properties.VariableNames)
    value = NaN(height(tbl), 1);
    return;
end
value = tbl.(actualName) - tbl.(targetName);
end

function util = localTireUtil(tbl, suffix)
fxName = ['tireFx' suffix '_N'];
fyName = ['tireFy' suffix '_N'];
fzName = ['tireFz' suffix '_N'];
if ~all(ismember({fxName, fyName, fzName}, tbl.Properties.VariableNames))
    util = NaN(height(tbl), 1);
    return;
end
util = hypot(tbl.(fxName), tbl.(fyName)) ./ max(abs(tbl.(fzName)), 1e-6);
util(~isfinite(util)) = NaN;
end

function value = localRowMax(values)
value = NaN(size(values, 1), 1);
for rowIdx = 1:size(values, 1)
    row = values(rowIdx, :);
    row = row(isfinite(row));
    if ~isempty(row)
        value(rowIdx) = max(row);
    end
end
end

function value = localRowMin(values)
value = NaN(size(values, 1), 1);
for rowIdx = 1:size(values, 1)
    row = values(rowIdx, :);
    row = row(isfinite(row));
    if ~isempty(row)
        value(rowIdx) = min(row);
    end
end
end

function value = localRowSum(values)
value = NaN(size(values, 1), 1);
for rowIdx = 1:size(values, 1)
    row = values(rowIdx, :);
    row = row(isfinite(row));
    if ~isempty(row)
        value(rowIdx) = sum(row);
    end
end
end

function aligned = localAlignedComparisonTable(runResults)
offRun = localFirstSignalRun(runResults, "dyc_off");
onRun = localFirstSignalRun(runResults, "dyc_on");
if isempty(offRun) || isempty(onRun)
    aligned = table();
    return;
end

offTable = localSignalsToTable(offRun.signals);
onTable = localSignalsToTable(onRun.signals);
if height(offTable) < 1 || height(onTable) < 1 || ...
        ~ismember('time_s', offTable.Properties.VariableNames) || ...
        ~ismember('time_s', onTable.Properties.VariableNames)
    aligned = table();
    return;
end

time = localCommonTime(offTable.time_s, onTable.time_s);
if isempty(time)
    aligned = table();
    return;
end

aligned = table(time(:), 'VariableNames', {'time_s'});
variables = {'station_m','speed_mps','yawRate_radps','yawRateError_radps', ...
    'lateralError_m','ay_mps2','ax_mps2','mz_Nm','throttle','steerSW_rad', ...
    'tireUtilMax','wheelTorqueSpread_Nm','leftRightDriveTorqueDelta_Nm'};
for idx = 1:numel(variables)
    name = variables{idx};
    offValue = localInterpColumn(offTable, name, time);
    onValue = localInterpColumn(onTable, name, time);
    aligned.([name '_dyc_off']) = offValue;
    aligned.([name '_dyc_on']) = onValue;
    aligned.([name '_delta']) = onValue - offValue;
end
end

function time = localCommonTime(offTime, onTime)
offTime = offTime(isfinite(offTime));
onTime = onTime(isfinite(onTime));
if isempty(offTime) || isempty(onTime)
    time = [];
    return;
end
tMin = max(min(offTime), min(onTime));
tMax = min(max(offTime), max(onTime));
time = offTime(offTime >= tMin & offTime <= tMax);
time = unique(time(:));
end

function values = localInterpColumn(tbl, name, time)
values = NaN(numel(time), 1);
if ~ismember(name, tbl.Properties.VariableNames)
    return;
end
x = tbl.time_s;
y = tbl.(name);
valid = isfinite(x) & isfinite(y);
if sum(valid) < 2
    return;
end
x = x(valid);
y = y(valid);
[x, uniqueIdx] = unique(x);
y = y(uniqueIdx);
if numel(x) < 2
    return;
end
values = interp1(x, y, time, 'linear', NaN);
values = values(:);
end

function run = localFirstSignalRun(runResults, caseId)
run = [];
fallbackRun = [];
for idx = 1:numel(runResults)
    if string(localStringField(runResults(idx), 'caseId')) ~= string(caseId)
        continue;
    end
    if ~isfield(runResults(idx), 'signals') || ~isstruct(runResults(idx).signals)
        continue;
    end
    if localSignalLength(runResults(idx).signals, localExportSignalFields()) < 1
        continue;
    end
    if string(localStringField(runResults(idx), 'status')) == "valid"
        run = runResults(idx);
        return;
    end
    if isempty(fallbackRun)
        fallbackRun = runResults(idx);
    end
end
run = fallbackRun;
end

function signals = localSignalsField(run)
if isfield(run, 'signals') && isstruct(run.signals)
    signals = run.signals;
else
    signals = struct();
end
end

function value = localStringField(run, name)
if ~isfield(run, name)
    value = "";
    return;
end
value = string(run.(name));
if isempty(value)
    value = "";
else
    value = value(1);
end
end

function value = localNumericField(run, name)
if ~isfield(run, name)
    value = NaN;
    return;
end
value = double(run.(name));
if isempty(value)
    value = NaN;
else
    value = value(1);
end
end

function source = localSignalSource(run)
source = "";
if isfield(run, 'signals') && isstruct(run.signals) && isfield(run.signals, 'source')
    source = string(run.signals.source);
end
if isempty(source)
    source = "";
else
    source = source(1);
end
end
