function analysis = analyze_dyc_figure8_report_results(resultsDir, varargin)
%ANALYZE_DYC_FIGURE8_REPORT_RESULTS 从八字绕环报告结果目录生成“快在哪里”分析。

if nargin < 1 || strlength(string(resultsDir)) == 0
    error('dyc:figure8ReportAnalysis:MissingResultsDir', 'resultsDir is required.');
end

opts = struct('writeFiles', true);
if nargin > 1
    opts = localMergeStruct(opts, varargin{1});
end

resultsDir = char(string(resultsDir));
paths = struct();
paths.summary = fullfile(resultsDir, 'comparison_metrics.csv');
paths.segmentMetrics = fullfile(resultsDir, 'figure8_segment_metrics.csv');
paths.segmentDelta = fullfile(resultsDir, 'figure8_segment_delta.csv');
paths.whereFaster = fullfile(resultsDir, 'figure8_where_faster.csv');
paths.analysisText = fullfile(resultsDir, 'figure8_where_faster_analysis.txt');

localRequireFile(paths.summary);
localRequireFile(paths.segmentMetrics);
localRequireFile(paths.segmentDelta);

summary = readtable(paths.summary, 'TextType', 'string');
segmentMetrics = readtable(paths.segmentMetrics, 'TextType', 'string');
segmentDelta = readtable(paths.segmentDelta, 'TextType', 'string');

whereFasterTable = localBuildWhereFasterTable(summary, segmentMetrics);
analysisLines = localBuildAnalysisLines(summary, segmentDelta, whereFasterTable);

if opts.writeFiles
    writetable(whereFasterTable, paths.whereFaster);
    localWriteTextFile(paths.analysisText, analysisLines);
end

analysis = struct();
analysis.resultsDir = string(resultsDir);
analysis.whereFasterPath = string(paths.whereFaster);
analysis.analysisTextPath = string(paths.analysisText);
analysis.whereFasterTable = whereFasterTable;
analysis.analysisLines = analysisLines;
analysis.conclusion = localLineAfterPrefix(analysisLines, "总体判断：");
end

function out = localMergeStruct(base, overrides)
out = base;
names = fieldnames(overrides);
for idx = 1:numel(names)
    out.(names{idx}) = overrides.(names{idx});
end
end

function localRequireFile(filePath)
if ~isfile(filePath)
    error('dyc:figure8ReportAnalysis:MissingFile', ...
        'Required Figure-8 report file is missing: %s', char(string(filePath)));
end
end

function tbl = localBuildWhereFasterTable(summary, segmentMetrics)
tbl = localEmptyWhereFasterTable();

tbl = localAppendSummaryRow(tbl, summary, "overall", "完成时间", ...
    "lapTime_s", "s", "lower", "总完成时间");
tbl = localAppendSummaryRow(tbl, summary, "overall", "整体平均速度", ...
    "meanSpeed_mps", "m/s", "higher", "整体平均速度保持");
tbl = localAppendSummaryRow(tbl, summary, "overall", "整体最低速度", ...
    "minSpeed_mps", "m/s", "higher", "整体最低速度");
tbl = localAppendSummaryRow(tbl, summary, "overall", "整体路径误差 RMSE", ...
    "lateralErrorRmse", "m", "lower", "整体路径跟随");
tbl = localAppendSummaryRow(tbl, summary, "overall", "整体路径误差峰值", ...
    "lateralErrorPeak", "m", "lower", "整体峰值路径误差");
tbl = localAppendSummaryRow(tbl, summary, "overall", "整体轮胎峰值利用率", ...
    "tireUtilPeak", "", "lower", "整体轮胎峰值负荷");
tbl = localAppendSummaryRow(tbl, summary, "overall", "横摆力矩峰值", ...
    "mzPeakAbs_Nm", "Nm", "higher", "横摆力矩介入");
tbl = localAppendSummaryRow(tbl, summary, "overall", "控制介入占比", ...
    "interventionRatio", "", "higher", "控制介入占比");

for segmentName = ["left", "right", "transition"]
    label = localSegmentLabel(segmentName);
    tbl = localAppendSegmentRow(tbl, segmentMetrics, segmentName, label + "平均速度", ...
        "meanSpeed_mps", "m/s", "higher", label + "速度保持");
    tbl = localAppendSegmentRow(tbl, segmentMetrics, segmentName, label + "最低速度", ...
        "minSpeed_mps", "m/s", "higher", label + "低速段速度保持");
    tbl = localAppendSegmentRow(tbl, segmentMetrics, segmentName, label + "路径误差 RMSE", ...
        "lateralErrorRmse", "m", "lower", label + "路径跟随");
    tbl = localAppendSegmentRow(tbl, segmentMetrics, segmentName, label + "路径误差峰值", ...
        "lateralErrorPeak", "m", "lower", label + "峰值路径误差");
    tbl = localAppendSegmentRow(tbl, segmentMetrics, segmentName, label + "轮胎峰值利用率", ...
        "tireUtilPeak", "", "lower", label + "轮胎峰值负荷");
    tbl = localAppendSegmentRow(tbl, segmentMetrics, segmentName, label + "横摆力矩峰值", ...
        "mzPeakAbs_Nm", "Nm", "higher", label + "横摆力矩介入");
end
end

function tbl = localAppendSummaryRow(tbl, summary, scope, metricName, fieldName, unitText, betterDirection, label)
off = localSummaryValue(summary, "dyc_off", fieldName);
on = localSummaryValue(summary, "dyc_on", fieldName);
tbl = [tbl; localWhereRow(scope, metricName, off, on, unitText, betterDirection, label)]; %#ok<AGROW>
end

function tbl = localAppendSegmentRow(tbl, segmentMetrics, segmentName, metricName, fieldName, unitText, betterDirection, label)
off = localSegmentValue(segmentMetrics, "dyc_off", segmentName, fieldName);
on = localSegmentValue(segmentMetrics, "dyc_on", segmentName, fieldName);
tbl = [tbl; localWhereRow(segmentName, metricName, off, on, unitText, betterDirection, label)]; %#ok<AGROW>
end

function row = localWhereRow(scope, metricName, off, on, unitText, betterDirection, label)
delta = on - off;
supportsDyc = localSupportsDyc(delta, betterDirection);
row = table( ...
    string(scope), string(metricName), off, on, delta, string(unitText), ...
    string(betterDirection), supportsDyc, ...
    localInterpretation(label, delta, unitText, betterDirection, supportsDyc), ...
    'VariableNames', localWhereFasterFields());
end

function tbl = localEmptyWhereFasterTable()
tbl = table('Size', [0 numel(localWhereFasterFields())], ...
    'VariableTypes', {'string','string','double','double','double','string','string','logical','string'}, ...
    'VariableNames', localWhereFasterFields());
end

function fields = localWhereFasterFields()
fields = {'scope','metric','dycOffValue','dycOnValue','delta','unit','betterDirection','supportsDyc','interpretation'};
end

function tf = localSupportsDyc(delta, betterDirection)
if ~isfinite(delta)
    tf = false;
elseif string(betterDirection) == "lower"
    tf = delta < 0;
elseif string(betterDirection) == "higher"
    tf = delta > 0;
else
    tf = false;
end
end

function text = localInterpretation(label, delta, unitText, betterDirection, supportsDyc)
if ~isfinite(delta)
    text = string(label) + "数据不可用";
    return;
end
deltaText = localSignedValue(delta, unitText);
if supportsDyc
    if string(betterDirection) == "lower"
        text = string(label) + "降低 " + deltaText + "，支持 DYC 有效";
    else
        text = string(label) + "提升 " + deltaText + "，支持 DYC 有效";
    end
else
    if string(betterDirection) == "lower"
        text = string(label) + "增加 " + deltaText + "，需要谨慎解读";
    else
        text = string(label) + "降低 " + deltaText + "，需要谨慎解读";
    end
end
end

function lines = localBuildAnalysisLines(summary, segmentDelta, whereFasterTable)
lapDelta = localSummaryValue(summary, "dyc_on", "lapTime_s") - ...
    localSummaryValue(summary, "dyc_off", "lapTime_s");
lapPct = 100 * lapDelta / localSummaryValue(summary, "dyc_off", "lapTime_s");
meanSpeedDelta = localSummaryValue(summary, "dyc_on", "meanSpeed_mps") - ...
    localSummaryValue(summary, "dyc_off", "meanSpeed_mps");
rightMeanSpeedDelta = localSegmentDeltaValue(segmentDelta, "right", "meanSpeedDelta_mps");
rightMinSpeedDelta = localSegmentDeltaValue(segmentDelta, "right", "minSpeedDelta_mps");
transitionMeanSpeedDelta = localSegmentDeltaValue(segmentDelta, "transition", "meanSpeedDelta_mps");
transitionLatDelta = localSegmentDeltaValue(segmentDelta, "transition", "lateralErrorRmseDelta");
tirePeakDelta = localFiniteColumn(segmentDelta, "tireUtilPeakDelta");
timeConclusion = localTimeConclusion(lapDelta);
stabilityConclusion = localStabilityConclusion(whereFasterTable, tirePeakDelta);

lines = [
    "八字绕环 DYC 有效性分析"
    ""
    "1. 双主线结论"
    "完成时间：dyc_on - dyc_off = " + localSignedValue(lapDelta, " s") + "（" + localSignedValue(lapPct, " %") + "）。"
    "总体判断：" + timeConclusion + "，" + stabilityConclusion
    ""
    "2. 快在哪里"
    "- 整体平均速度变化：" + localSignedValue(meanSpeedDelta, " m/s") + "。"
    "- 右转段平均速度变化：" + localSignedValue(rightMeanSpeedDelta, " m/s") + "，最低速度变化：" + localSignedValue(rightMinSpeedDelta, " m/s") + "。"
    "- 换向过渡区平均速度变化：" + localSignedValue(transitionMeanSpeedDelta, " m/s") + "，路径误差 RMSE 变化：" + localSignedValue(transitionLatDelta, " m") + "。"
    ""
    "3. 数据解读"
    localBestSupportedRows(whereFasterTable)
    "- 若轮胎峰值利用率或最低速度局部变差，应写成混合证据，不写成全面稳定性提升。"
    ""
    "4. 验证边界"
    "本结论来自当前八字绕环 CarSim/Simulink 联合仿真，不代表 DIL、实时硬件或实车验证。"
];
end

function text = localTimeConclusion(lapDelta)
if isfinite(lapDelta) && lapDelta < 0
    text = "圈速有效";
elseif isfinite(lapDelta)
    text = "圈速未改善";
else
    text = "圈速不可用";
end
end

function text = localStabilityConclusion(whereFasterTable, tirePeakDelta)
pathRows = whereFasterTable(contains(whereFasterTable.metric, "路径误差"), :);
pathOk = any(pathRows.supportsDyc);
tireHasWorsePeak = any(tirePeakDelta > 0);
tireHasBetterPeak = any(tirePeakDelta < 0);
if pathOk && tireHasWorsePeak
    text = "稳定性证据混合/不足";
elseif pathOk && tireHasBetterPeak
    text = "稳定性证据支持 DYC";
else
    text = "稳定性证据不足";
end
end

function lines = localBestSupportedRows(whereFasterTable)
rows = whereFasterTable(whereFasterTable.supportsDyc, :);
if height(rows) == 0
    lines = "- 当前表格没有形成明确支持 DYC 的分项证据。";
    return;
end
priority = ["完成时间", "右转平均速度", "右转最低速度", "换向过渡区路径误差 RMSE", "整体平均速度"];
lines = strings(0, 1);
for idx = 1:numel(priority)
    match = rows(rows.metric == priority(idx), :);
    if height(match) > 0
        lines(end+1, 1) = "- " + match.interpretation(1); %#ok<AGROW>
    end
end
if isempty(lines)
    for idx = 1:min(height(rows), 3)
        lines(end+1, 1) = "- " + rows.interpretation(idx); %#ok<AGROW>
    end
end
end

function value = localSummaryValue(summary, caseId, fieldName)
value = NaN;
if ~ismember(fieldName, summary.Properties.VariableNames)
    return;
end
row = summary(string(summary.caseId) == string(caseId), :);
if height(row) ~= 1
    return;
end
value = double(row.(fieldName)(1));
end

function value = localSegmentValue(segmentMetrics, caseId, segmentName, fieldName)
value = NaN;
if ~ismember(fieldName, segmentMetrics.Properties.VariableNames)
    return;
end
row = segmentMetrics(string(segmentMetrics.caseId) == string(caseId) & ...
    string(segmentMetrics.segmentType) == string(segmentName), :);
if height(row) ~= 1
    return;
end
value = double(row.(fieldName)(1));
end

function value = localSegmentDeltaValue(segmentDelta, segmentName, fieldName)
value = NaN;
if ~ismember(fieldName, segmentDelta.Properties.VariableNames)
    return;
end
row = segmentDelta(string(segmentDelta.segmentType) == string(segmentName), :);
if height(row) ~= 1
    return;
end
value = double(row.(fieldName)(1));
end

function values = localFiniteColumn(tbl, fieldName)
if ~ismember(fieldName, tbl.Properties.VariableNames)
    values = [];
    return;
end
values = double(tbl.(fieldName));
values = values(isfinite(values));
end

function label = localSegmentLabel(segmentName)
switch string(segmentName)
    case "left"
        label = "左转";
    case "right"
        label = "右转";
    otherwise
        label = "换向过渡区";
end
end

function text = localSignedValue(value, unitText)
if ~isfinite(value)
    text = "unavailable";
else
    text = string(sprintf('%+.4f%s', value, char(unitText)));
end
end

function text = localLineAfterPrefix(lines, prefix)
text = "";
for idx = 1:numel(lines)
    line = string(lines(idx));
    if startsWith(line, prefix)
        text = extractAfter(line, strlength(prefix));
        return;
    end
end
end

function localWriteTextFile(filePath, lines)
folder = fileparts(filePath);
if ~isfolder(folder)
    mkdir(folder);
end
writelines(lines, filePath);
end
