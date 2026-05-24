function result = compute_dyc_autocross_metrics(runResults, cfg)
%COMPUTE_DYC_AUTOCROSS_METRICS 计算 Autocross DYC 开关对比指标。

perRun = localBuildPerRunTable(runResults, cfg);
summary = localBuildSummaryTable(perRun, cfg);
delta = localBuildDeltaTable(summary);

result = struct();
result.perRun = perRun;
result.summary = summary;
result.delta = delta;
result.comparison = localBuildComparison(delta);
end

function perRun = localBuildPerRunTable(runResults, cfg)
fields = localPerRunFields();
types = localPerRunTypes();
rowCount = numel(runResults);
perRun = table('Size', [rowCount numel(fields)], 'VariableTypes', types, 'VariableNames', fields);

for rowIdx = 1:rowCount
    run = runResults(rowIdx);
    perRun.caseId(rowIdx) = localStringField(run, 'caseId');
    perRun.displayName(rowIdx) = localStringField(run, 'displayName');
    perRun.repeatIndex(rowIdx) = localNumericField(run, 'repeatIndex');
    perRun.Kp(rowIdx) = localNumericField(run, 'Kp');
    perRun.Ki(rowIdx) = localNumericField(run, 'Ki');
    perRun.Kd(rowIdx) = localNumericField(run, 'Kd');
    perRun.status(rowIdx) = localStringField(run, 'status');
    perRun.lapTime_s(rowIdx) = localNumericField(run, 'lapTime_s');
    perRun.finishStation_m(rowIdx) = localNumericField(run, 'finishStation_m');
    perRun.svStation_m(rowIdx) = localNumericField(run, 'svStation_m');
    perRun.objective(rowIdx) = localNumericField(run, 'objective');
    perRun.penalty(rowIdx) = localNumericField(run, 'penalty');
    perRun.stopReason(rowIdx) = localStringField(run, 'stopReason');
    perRun.failureReason(rowIdx) = localStringField(run, 'failureReason');
    perRun.lastRunLogPath(rowIdx) = localStringField(run, 'lastRunLogPath');
    perRun.lastRunEndPath(rowIdx) = localStringField(run, 'lastRunEndPath');
    perRun.elapsedWallTime_s(rowIdx) = localNumericField(run, 'elapsedWallTime_s');

    metrics = localComputeSignalMetrics(localSignalsField(run), cfg);
    metricFields = localMetricFields();
    for metricIdx = 1:numel(metricFields)
        name = metricFields{metricIdx};
        perRun.(name)(rowIdx) = metrics.(name);
    end
end
end

function summary = localBuildSummaryTable(perRun, cfg)
fields = localSummaryFields();
types = localSummaryTypes();
caseCount = numel(cfg.cases);
summary = table('Size', [caseCount numel(fields)], 'VariableTypes', types, 'VariableNames', fields);
metricFields = localSummaryMetricFields();

for caseIdx = 1:caseCount
    caseCfg = cfg.cases(caseIdx);
    caseId = string(caseCfg.id);
    rows = perRun(perRun.caseId == caseId, :);
    validRows = rows(rows.status == "valid", :);

    summary.caseId(caseIdx) = caseId;
    summary.displayName(caseIdx) = string(caseCfg.displayName);
    summary.validRunCount(caseIdx) = height(validRows);
    summary.lapTime_s(caseIdx) = localMeanOmitNaN(validRows.lapTime_s);
    summary.lapTimeStd_s(caseIdx) = localStdIfEnough(validRows.lapTime_s);

    for metricIdx = 1:numel(metricFields)
        name = metricFields{metricIdx};
        summary.(name)(caseIdx) = localMeanOmitNaN(validRows.(name));
    end
end
end

function delta = localBuildDeltaTable(summary)
fields = localDeltaFields();
types = localDeltaTypes();
delta = table('Size', [1 numel(fields)], 'VariableTypes', types, 'VariableNames', fields);

delta.baselineCaseId(1) = "dyc_off";
delta.testCaseId(1) = "dyc_on";
delta.status(1) = "invalid";
delta.failureReason(1) = "dyc_off or dyc_on has no valid run";
numericFields = fields(5:end);
for idx = 1:numel(numericFields)
    delta.(numericFields{idx})(1) = NaN;
end

baseline = summary(summary.caseId == delta.baselineCaseId(1), :);
testCase = summary(summary.caseId == delta.testCaseId(1), :);
if height(baseline) ~= 1 || height(testCase) ~= 1
    return;
end
if baseline.validRunCount < 1 || testCase.validRunCount < 1
    return;
end
if ~isfinite(baseline.lapTime_s) || ~isfinite(testCase.lapTime_s)
    delta.failureReason(1) = "dyc_off or dyc_on has invalid or missing lap time";
    return;
end
if baseline.lapTime_s == 0
    delta.failureReason(1) = "dyc_off baseline lap time is zero";
    return;
end

delta.status(1) = "valid";
delta.failureReason(1) = "";
delta.lapTimeDelta_s(1) = testCase.lapTime_s - baseline.lapTime_s;
delta.lapTimeDelta_pct(1) = 100 * delta.lapTimeDelta_s(1) / baseline.lapTime_s;

metricMap = localDeltaMetricMap();
for idx = 1:size(metricMap, 1)
    sourceName = metricMap{idx, 1};
    deltaName = metricMap{idx, 2};
    delta.(deltaName)(1) = testCase.(sourceName) - baseline.(sourceName);
end
end

function comparison = localBuildComparison(delta)
comparison = struct();
comparison.baselineCaseId = delta.baselineCaseId(1);
comparison.testCaseId = delta.testCaseId(1);
comparison.status = delta.status(1);
comparison.failureReason = delta.failureReason(1);
comparison.lapTimeDelta_s = delta.lapTimeDelta_s(1);
comparison.lapTimeDelta_pct = delta.lapTimeDelta_pct(1);
end

function metrics = localComputeSignalMetrics(signals, cfg)
metricFields = localMetricFields();
for idx = 1:numel(metricFields)
    metrics.(metricFields{idx}) = NaN;
end

yawRate = localVectorSignal(signals, 'yawRate_radps');
yawTarget = localVectorSignal(signals, 'yawRateTarget_radps');
yawError = localPairDiff(yawRate, yawTarget);
metrics.yawRateRmse = localRms(yawError);
metrics.yawRateMae = localMeanAbs(yawError);
metrics.yawRatePeakError = localPeakAbs(yawError);

latVeh = localVectorSignal(signals, 'latVeh_m');
latTarget = localVectorSignal(signals, 'latTarget_m');
lateralError = localPairDiff(latVeh, latTarget);
metrics.lateralErrorRmse = localRms(lateralError);
metrics.lateralErrorPeak = localPeakAbs(lateralError);

ay = localVectorSignal(signals, 'ay_mps2');
metrics.ayPeakAbs = localPeakAbs(ay);
metrics.ayRms = localRms(ay);
metrics.ayStd = localStdIfEnough(ay);

speed = localVectorSignal(signals, 'speed_mps');
metrics.meanSpeed_mps = localMeanOmitNaN(speed);
metrics.minSpeed_mps = localMinOmitNaN(speed);
metrics.speedStd_mps = localStdIfEnough(speed);

mz = localVectorSignal(signals, 'mz_Nm');
time = localVectorSignal(signals, 'time_s');
metrics.mzRms_Nm = localRms(mz);
metrics.mzPeakAbs_Nm = localPeakAbs(mz);
metrics.mzAbsIntegral_Nms = localMzAbsIntegral(time, mz);
metrics.interventionRatio = localInterventionRatio(mz, cfg);
end

function value = localMzAbsIntegral(time, mz)
if isempty(mz)
    value = NaN;
    return;
end
if ~any(isfinite(mz))
    value = NaN;
    return;
end

if numel(time) >= 2 && numel(mz) >= 2
    count = min(numel(time), numel(mz));
    time = time(1:count);
    mz = mz(1:count);
    valid = ~isnan(time) & ~isnan(mz);
    if sum(valid) >= 2
        value = trapz(time(valid), abs(mz(valid)));
    else
        value = sum(abs(mz(~isnan(mz))));
    end
else
    value = sum(abs(mz(~isnan(mz))));
end
end

function value = localInterventionRatio(mz, cfg)
mz = mz(~isnan(mz));
if isempty(mz)
    value = NaN;
    return;
end
threshold = cfg.thresholds.mzActive_Nm;
value = mean(abs(mz) > threshold);
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

function values = localVectorSignal(signals, name)
if ~isfield(signals, name)
    values = [];
    return;
end
values = double(signals.(name));
values = values(:);
end

function values = localPairDiff(left, right)
count = min(numel(left), numel(right));
if count < 1
    values = [];
    return;
end
values = left(1:count) - right(1:count);
end

function value = localRms(values)
values = values(~isnan(values));
if isempty(values)
    value = NaN;
else
    value = sqrt(mean(values .^ 2));
end
end

function value = localMeanAbs(values)
values = values(~isnan(values));
if isempty(values)
    value = NaN;
else
    value = mean(abs(values));
end
end

function value = localPeakAbs(values)
values = values(~isnan(values));
if isempty(values)
    value = NaN;
else
    value = max(abs(values));
end
end

function value = localMeanOmitNaN(values)
values = values(~isnan(values));
if isempty(values)
    value = NaN;
else
    value = mean(values);
end
end

function value = localMinOmitNaN(values)
values = values(~isnan(values));
if isempty(values)
    value = NaN;
else
    value = min(values);
end
end

function value = localStdIfEnough(values)
values = values(~isnan(values));
if numel(values) < 2
    value = NaN;
else
    value = std(values);
end
end

function fields = localPerRunFields()
fields = [localRunFields(), localMetricFields()];
end

function types = localPerRunTypes()
types = [ ...
    "string", "string", "double", "double", "double", "double", ...
    "string", "double", "double", "double", "double", "double", ...
    "string", "string", "string", "string", "double", ...
    repmat("double", 1, numel(localMetricFields()))];
end

function fields = localRunFields()
fields = {'caseId','displayName','repeatIndex','Kp','Ki','Kd','status','lapTime_s','finishStation_m','svStation_m','objective','penalty','stopReason','failureReason','lastRunLogPath','lastRunEndPath','elapsedWallTime_s'};
end

function fields = localMetricFields()
fields = {'yawRateRmse','yawRateMae','yawRatePeakError','lateralErrorRmse','lateralErrorPeak','ayPeakAbs','ayRms','ayStd','meanSpeed_mps','minSpeed_mps','speedStd_mps','mzRms_Nm','mzPeakAbs_Nm','mzAbsIntegral_Nms','interventionRatio'};
end

function fields = localSummaryFields()
fields = [{'caseId','displayName','validRunCount','lapTime_s','lapTimeStd_s'}, localSummaryMetricFields()];
end

function types = localSummaryTypes()
types = ["string", "string", "double", "double", "double", repmat("double", 1, numel(localSummaryMetricFields()))];
end

function fields = localSummaryMetricFields()
fields = localMetricFields();
end

function fields = localDeltaFields()
fields = {'baselineCaseId','testCaseId','status','failureReason', ...
    'lapTimeDelta_s','lapTimeDelta_pct','yawRateRmseDelta','yawRateMaeDelta', ...
    'yawRatePeakErrorDelta','lateralErrorRmseDelta','lateralErrorPeakDelta', ...
    'ayPeakAbsDelta','ayRmsDelta','ayStdDelta','meanSpeedDelta_mps', ...
    'minSpeedDelta_mps','speedStdDelta_mps','mzRmsDelta_Nm','mzPeakAbsDelta_Nm', ...
    'mzAbsIntegralDelta_Nms','interventionRatioDelta'};
end

function types = localDeltaTypes()
types = ["string", "string", "string", "string", repmat("double", 1, numel(localDeltaFields()) - 4)];
end

function metricMap = localDeltaMetricMap()
metricMap = {
    'yawRateRmse', 'yawRateRmseDelta';
    'yawRateMae', 'yawRateMaeDelta';
    'yawRatePeakError', 'yawRatePeakErrorDelta';
    'lateralErrorRmse', 'lateralErrorRmseDelta';
    'lateralErrorPeak', 'lateralErrorPeakDelta';
    'ayPeakAbs', 'ayPeakAbsDelta';
    'ayRms', 'ayRmsDelta';
    'ayStd', 'ayStdDelta';
    'meanSpeed_mps', 'meanSpeedDelta_mps';
    'minSpeed_mps', 'minSpeedDelta_mps';
    'speedStd_mps', 'speedStdDelta_mps';
    'mzRms_Nm', 'mzRmsDelta_Nm';
    'mzPeakAbs_Nm', 'mzPeakAbsDelta_Nm';
    'mzAbsIntegral_Nms', 'mzAbsIntegralDelta_Nms';
    'interventionRatio', 'interventionRatioDelta'};
end
