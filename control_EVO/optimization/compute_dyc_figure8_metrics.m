function result = compute_dyc_figure8_metrics(runResults, cfg)
%COMPUTE_DYC_FIGURE8_METRICS 计算八字绕环 DYC 开关对比指标。

result = compute_dyc_autocross_metrics(runResults, cfg);
[segments, evidence] = localBuildSegmentTable(runResults, cfg);
result.figure8Segments = segments;
result.figure8SegmentDelta = localBuildSegmentDeltaTable(segments);
result.figure8Evidence = localCompleteEvidence(evidence, result);
end

function [segments, evidence] = localBuildSegmentTable(runResults, cfg)
segments = localEmptySegmentTable();
evidence = struct('status', "unavailable", 'failureReason', "分段证据不可用：缺少 ay_mps2 和 yawRate_radps。", ...
    'segmentSource', "", 'stabilityConclusion', "分段证据不可用");

sourceName = localSelectSegmentSource(runResults);
if strlength(sourceName) == 0
    return;
end

for idx = 1:numel(runResults)
    run = runResults(idx);
    if string(localStructField(run, 'status')) ~= "valid" || ...
            ~isfield(run, 'signals') || ~isstruct(run.signals)
        continue;
    end
    turnSignal = localVectorField(run.signals, char(sourceName));
    if isempty(turnSignal)
        continue;
    end
    segmentType = localClassifySegments(turnSignal, cfg, sourceName);
    for segmentName = ["left", "right", "transition"]
        mask = segmentType == segmentName;
        if sum(mask) < localMinSegmentSampleCount(cfg)
            continue;
        end
        segments = [segments; localSegmentRow(run, segmentName, sourceName, mask)]; %#ok<AGROW>
    end
end

if height(segments) > 0
    evidence.status = "valid";
    evidence.failureReason = "";
    evidence.segmentSource = sourceName;
    evidence.stabilityConclusion = "分段证据有效";
end
end

function sourceName = localSelectSegmentSource(runResults)
if localRunsHaveFiniteField(runResults, 'ay_mps2')
    sourceName = "ay_mps2";
elseif localRunsHaveFiniteField(runResults, 'yawRate_radps')
    sourceName = "yawRate_radps";
else
    sourceName = "";
end
end

function tf = localRunsHaveFiniteField(runResults, fieldName)
tf = false;
for idx = 1:numel(runResults)
    if ~isfield(runResults(idx), 'signals') || ~isstruct(runResults(idx).signals)
        continue;
    end
    value = localVectorField(runResults(idx).signals, fieldName);
    if ~isempty(value) && any(isfinite(value))
        tf = true;
        return;
    end
end
end

function segmentType = localClassifySegments(turnSignal, cfg, sourceName)
turnSignal = turnSignal(:);
segmentType = strings(size(turnSignal));
if sourceName == "ay_mps2"
    threshold = localFigure8Field(cfg, 'transitionAyAbsThreshold_mps2', 0.35);
else
    threshold = localFigure8Field(cfg, 'transitionYawRateAbsThreshold_radps', 0.03);
end
segmentType(abs(turnSignal) <= threshold) = "transition";
segmentType(turnSignal > threshold) = "left";
segmentType(turnSignal < -threshold) = "right";
end

function row = localSegmentRow(run, segmentName, sourceName, mask)
signals = run.signals;
latError = localPairDiff(localVectorField(signals, 'latVeh_m'), localVectorField(signals, 'latTarget_m'));
speed = localVectorField(signals, 'speed_mps');
tireUtil = localTireUtilizationMax(signals);
mz = localVectorField(signals, 'mz_Nm');
torqueSpread = localWheelTorqueSpread(signals);

row = table( ...
    string(localStructField(run, 'caseId')), ...
    string(segmentName), ...
    string(sourceName), ...
    double(sum(mask)), ...
    localRms(localMasked(latError, mask)), ...
    localPeakAbs(localMasked(latError, mask)), ...
    localMeanOmitNaN(localMasked(speed, mask)), ...
    localMinOmitNaN(localMasked(speed, mask)), ...
    localPeakAbs(localMasked(tireUtil, mask)), ...
    localMeanOmitNaN(localMasked(tireUtil, mask)), ...
    localPeakAbs(localMasked(mz, mask)), ...
    localRms(localMasked(mz, mask)), ...
    localPeakAbs(localMasked(torqueSpread, mask)), ...
    localRms(localMasked(torqueSpread, mask)), ...
    'VariableNames', localSegmentFields());
end

function delta = localBuildSegmentDeltaTable(segments)
delta = localEmptySegmentDeltaTable();
if height(segments) < 1
    return;
end

for segmentName = ["left", "right", "transition"]
    off = segments(segments.caseId == "dyc_off" & segments.segmentType == segmentName, :);
    on = segments(segments.caseId == "dyc_on" & segments.segmentType == segmentName, :);
    if height(off) ~= 1 || height(on) ~= 1
        continue;
    end
    row = table( ...
        segmentName, ...
        on.sampleCount - off.sampleCount, ...
        on.lateralErrorRmse - off.lateralErrorRmse, ...
        on.lateralErrorPeak - off.lateralErrorPeak, ...
        on.meanSpeed_mps - off.meanSpeed_mps, ...
        on.minSpeed_mps - off.minSpeed_mps, ...
        on.tireUtilPeak - off.tireUtilPeak, ...
        on.tireUtilMean - off.tireUtilMean, ...
        on.mzPeakAbs_Nm - off.mzPeakAbs_Nm, ...
        on.mzRms_Nm - off.mzRms_Nm, ...
        on.wheelTorqueSpreadPeak_Nm - off.wheelTorqueSpreadPeak_Nm, ...
        on.wheelTorqueSpreadRms_Nm - off.wheelTorqueSpreadRms_Nm, ...
        'VariableNames', localSegmentDeltaFields());
    delta = [delta; row]; %#ok<AGROW>
end
end

function evidence = localCompleteEvidence(evidence, result)
if string(evidence.status) ~= "valid" || height(result.figure8SegmentDelta) < 1
    return;
end

delta = result.figure8SegmentDelta;
latOk = localMeanOmitNaN(delta.lateralErrorRmseDelta) < 0;
tireOk = localMeanOmitNaN(delta.tireUtilPeakDelta) < 0;
timeDelta = result.comparison.lapTimeDelta_s;
timeOk = isfinite(timeDelta) && timeDelta < 0;
timeSmall = isfinite(timeDelta) && abs(timeDelta) < 0.2;

if latOk && tireOk && timeOk && timeSmall
    evidence.stabilityConclusion = "稳定性有效，圈速收益有限";
elseif latOk && tireOk
    evidence.stabilityConclusion = "稳定性有效";
elseif timeOk
    evidence.stabilityConclusion = "完成时间有效，稳定性证据不足";
else
    evidence.stabilityConclusion = "有效性证据不足";
end
end

function tbl = localEmptySegmentTable()
tbl = table('Size', [0 numel(localSegmentFields())], ...
    'VariableTypes', {'string','string','string','double','double','double', ...
    'double','double','double','double','double','double','double','double'}, ...
    'VariableNames', localSegmentFields());
end

function fields = localSegmentFields()
fields = {'caseId','segmentType','segmentSource','sampleCount', ...
    'lateralErrorRmse','lateralErrorPeak','meanSpeed_mps','minSpeed_mps', ...
    'tireUtilPeak','tireUtilMean','mzPeakAbs_Nm','mzRms_Nm', ...
    'wheelTorqueSpreadPeak_Nm','wheelTorqueSpreadRms_Nm'};
end

function tbl = localEmptySegmentDeltaTable()
tbl = table('Size', [0 numel(localSegmentDeltaFields())], ...
    'VariableTypes', {'string','double','double','double','double','double', ...
    'double','double','double','double','double','double'}, ...
    'VariableNames', localSegmentDeltaFields());
end

function fields = localSegmentDeltaFields()
fields = {'segmentType','sampleCountDelta','lateralErrorRmseDelta', ...
    'lateralErrorPeakDelta','meanSpeedDelta_mps','minSpeedDelta_mps', ...
    'tireUtilPeakDelta','tireUtilMeanDelta','mzPeakAbsDelta_Nm', ...
    'mzRmsDelta_Nm','wheelTorqueSpreadPeakDelta_Nm','wheelTorqueSpreadRmsDelta_Nm'};
end

function value = localFigure8Field(cfg, fieldName, defaultValue)
if isfield(cfg, 'figure8') && isstruct(cfg.figure8) && isfield(cfg.figure8, fieldName)
    value = double(cfg.figure8.(fieldName));
else
    value = defaultValue;
end
end

function value = localMinSegmentSampleCount(cfg)
value = localFigure8Field(cfg, 'minSegmentSampleCount', 2);
end

function value = localStructField(s, fieldName)
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
else
    value = "";
end
end

function value = localVectorField(signals, fieldName)
value = [];
if ~isstruct(signals) || ~isfield(signals, fieldName)
    return;
end
value = double(signals.(fieldName));
value = value(:);
end

function value = localPairDiff(left, right)
count = min(numel(left), numel(right));
if count < 1
    value = [];
else
    value = left(1:count) - right(1:count);
end
end

function value = localMasked(values, mask)
count = min(numel(values), numel(mask));
if count < 1
    value = [];
else
    value = values(1:count);
    value = value(mask(1:count));
end
end

function value = localRms(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = sqrt(mean(values .^ 2));
end
end

function value = localPeakAbs(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = max(abs(values));
end
end

function value = localMeanOmitNaN(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = mean(values);
end
end

function value = localMinOmitNaN(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = min(values);
end
end

function value = localTireUtilizationMax(signals)
wheelIds = {'L1','L2','R1','R2'};
value = [];
for idx = 1:numel(wheelIds)
    suffix = wheelIds{idx};
    fx = localVectorField(signals, ['tireFx' suffix '_N']);
    fy = localVectorField(signals, ['tireFy' suffix '_N']);
    fz = localVectorField(signals, ['tireFz' suffix '_N']);
    if isempty(fx) || isempty(fy) || isempty(fz)
        continue;
    end
    count = min([numel(fx), numel(fy), numel(fz)]);
    wheelUtil = hypot(fx(1:count), fy(1:count)) ./ max(abs(fz(1:count)), 1e-6);
    if isempty(value)
        value = wheelUtil;
    else
        commonCount = min(numel(value), numel(wheelUtil));
        value = max(value(1:commonCount), wheelUtil(1:commonCount));
    end
end
end

function value = localWheelTorqueSpread(signals)
fields = {'myDrL1_Nm','myDrL2_Nm','myDrR1_Nm','myDrR2_Nm'};
values = [];
for idx = 1:numel(fields)
    v = localVectorField(signals, fields{idx});
    if isempty(v)
        value = [];
        return;
    end
    values(:, idx) = v(:); %#ok<AGROW>
end
value = max(values, [], 2) - min(values, [], 2);
end
