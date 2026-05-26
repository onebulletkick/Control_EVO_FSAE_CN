function analysis = analyze_dyc_autocross_report_results(resultsDir, varargin)
%ANALYZE_DYC_AUTOCROSS_REPORT_RESULTS 从 Autocross 报告结果目录生成“快在哪里”分析。

if nargin < 1 || strlength(string(resultsDir)) == 0
    error('dyc:autocrossReportAnalysis:MissingResultsDir', 'resultsDir is required.');
end

opts = struct('writeFiles', true);
if nargin > 1
    opts = localMergeStruct(opts, varargin{1});
end

resultsDir = char(string(resultsDir));
paths = struct();
paths.summary = fullfile(resultsDir, 'comparison_metrics.csv');
paths.runResults = fullfile(resultsDir, 'run_results.csv');
paths.aligned = fullfile(resultsDir, 'signal_data', 'aligned_dyc_comparison.csv');
paths.whereFaster = fullfile(resultsDir, 'autocross_where_faster.csv');
paths.analysisText = fullfile(resultsDir, 'autocross_where_faster_analysis.txt');

localRequireFile(paths.summary);
localRequireFile(paths.runResults);
localRequireFile(paths.aligned);

summary = readtable(paths.summary, 'TextType', 'string');
runResults = readtable(paths.runResults, 'TextType', 'string');
aligned = readtable(paths.aligned, 'TextType', 'string');

whereFasterTable = localBuildWhereFasterTable(summary, aligned);
analysisLines = localBuildAnalysisLines(summary, runResults, whereFasterTable);

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
    error('dyc:autocrossReportAnalysis:MissingFile', ...
        'Required Autocross report file is missing: %s', char(string(filePath)));
end
end

function tbl = localBuildWhereFasterTable(summary, aligned)
tbl = localEmptyWhereFasterTable();
tbl = localAppendSummaryRow(tbl, summary, "overall", "完成时间", ...
    "lapTime_s", "s", "lower", "总完成时间");
tbl = localAppendSummaryRow(tbl, summary, "speed", "平均速度", ...
    "meanSpeed_mps", "m/s", "higher", "平均速度保持");
tbl = localAppendSummaryRow(tbl, summary, "speed", "最低速度", ...
    "minSpeed_mps", "m/s", "higher", "最低速度保持");
tbl = localAppendSummaryRow(tbl, summary, "path_tracking", "路径误差 RMSE", ...
    "lateralErrorRmse", "m", "lower", "路径跟随");
tbl = localAppendSummaryRow(tbl, summary, "path_tracking", "路径误差峰值", ...
    "lateralErrorPeak", "m", "lower", "峰值路径误差");
tbl = localAppendSummaryRow(tbl, summary, "path_tracking", "横摆误差 RMSE", ...
    "yawRateRmse", "rad/s", "lower", "横摆跟踪");
tbl = localAppendSummaryRow(tbl, summary, "path_tracking", "横摆误差峰值", ...
    "yawRatePeakError", "rad/s", "lower", "峰值横摆误差");
tbl = localAppendSummaryRow(tbl, summary, "tire_utilization", "轮胎峰值利用率", ...
    "tireUtilPeak", "", "lower", "轮胎峰值利用率");
tbl = localAppendSummaryRow(tbl, summary, "tire_utilization", "轮胎平均利用率", ...
    "tireUtilMean", "", "lower", "轮胎平均利用率");
tbl = localAppendSummaryRow(tbl, summary, "yaw_moment", "横摆力矩峰值", ...
    "mzPeakAbs_Nm", "Nm", "higher", "横摆力矩介入");
tbl = localAppendSummaryRow(tbl, summary, "yaw_moment", "横摆力矩绝对积分", ...
    "mzAbsIntegral_Nms", "Nms", "higher", "横摆力矩总介入");
tbl = localAppendSummaryRow(tbl, summary, "yaw_moment", "控制介入占比", ...
    "interventionRatio", "", "higher", "控制介入占比");
tbl = localAppendSummaryRow(tbl, summary, "torque_distribution", "驱动矩离散度峰值", ...
    "wheelTorqueSpreadPeak_Nm", "Nm", "higher", "驱动矩离散度");
tbl = localAppendSummaryRow(tbl, summary, "driver_demand", "油门均值", ...
    "throttleMean", "", "lower", "油门需求");
tbl = localAppendSummaryRow(tbl, summary, "acceleration", "纵向加速度 RMS", ...
    "axRms", "m/s^2", "higher", "纵向加速度保持");

tbl = localAppendAlignedRow(tbl, aligned, "speed", "时序平均速度差", ...
    "speed_mps_delta", "m/s", "higher", "时序平均速度");
tbl = localAppendAlignedRow(tbl, aligned, "path_tracking", "时序路径误差差", ...
    "lateralError_m_delta", "m", "lower_abs", "时序路径误差");
tbl = localAppendAlignedRow(tbl, aligned, "yaw_moment", "时序横摆力矩差", ...
    "mz_Nm_delta", "Nm", "higher_abs", "时序横摆力矩介入");
tbl = localAppendAlignedRow(tbl, aligned, "tire_utilization", "时序轮胎利用率差", ...
    "tireUtilMax_delta", "", "lower", "时序轮胎利用率");
end

function tbl = localAppendSummaryRow(tbl, summary, scope, metricName, fieldName, unitText, betterDirection, label)
off = localSummaryValue(summary, "dyc_off", fieldName);
on = localSummaryValue(summary, "dyc_on", fieldName);
tbl = [tbl; localWhereRow(scope, metricName, off, on, on - off, unitText, betterDirection, label)]; %#ok<AGROW>
end

function tbl = localAppendAlignedRow(tbl, aligned, scope, metricName, fieldName, unitText, betterDirection, label)
delta = localMeanColumn(aligned, fieldName);
tbl = [tbl; localWhereRow(scope, metricName, NaN, NaN, delta, unitText, betterDirection, label)]; %#ok<AGROW>
end

function row = localWhereRow(scope, metricName, off, on, delta, unitText, betterDirection, label)
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
elseif string(betterDirection) == "lower_abs"
    tf = delta < 0;
elseif string(betterDirection) == "higher_abs"
    tf = abs(delta) > 0;
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
    if string(betterDirection) == "lower" || string(betterDirection) == "lower_abs"
        text = string(label) + "降低 " + deltaText + "，支持 DYC 有效";
    else
        text = string(label) + "提升 " + deltaText + "，支持 DYC 有效";
    end
else
    if string(betterDirection) == "lower" || string(betterDirection) == "lower_abs"
        text = string(label) + "增加 " + deltaText + "，需要谨慎解读";
    else
        text = string(label) + "降低 " + deltaText + "，需要谨慎解读";
    end
end
end

function lines = localBuildAnalysisLines(summary, runResults, whereFasterTable)
lapDelta = localSummaryValue(summary, "dyc_on", "lapTime_s") - ...
    localSummaryValue(summary, "dyc_off", "lapTime_s");
lapPct = 100 * lapDelta / localSummaryValue(summary, "dyc_off", "lapTime_s");
meanSpeedDelta = localSummaryValue(summary, "dyc_on", "meanSpeed_mps") - ...
    localSummaryValue(summary, "dyc_off", "meanSpeed_mps");
minSpeedDelta = localSummaryValue(summary, "dyc_on", "minSpeed_mps") - ...
    localSummaryValue(summary, "dyc_off", "minSpeed_mps");
latRmseDelta = localSummaryValue(summary, "dyc_on", "lateralErrorRmse") - ...
    localSummaryValue(summary, "dyc_off", "lateralErrorRmse");
yawRmseDelta = localSummaryValue(summary, "dyc_on", "yawRateRmse") - ...
    localSummaryValue(summary, "dyc_off", "yawRateRmse");
tirePeakDelta = localSummaryValue(summary, "dyc_on", "tireUtilPeak") - ...
    localSummaryValue(summary, "dyc_off", "tireUtilPeak");
mzPeakDelta = localSummaryValue(summary, "dyc_on", "mzPeakAbs_Nm") - ...
    localSummaryValue(summary, "dyc_off", "mzPeakAbs_Nm");

lines = [
    "Autocross DYC 有效性分析"
    ""
    "总体判断：" + localTimeConclusion(lapDelta) + "，" + localStabilityConclusion(whereFasterTable, minSpeedDelta, tirePeakDelta)
    ""
    "完成时间：dyc_on - dyc_off = " + localSignedValue(lapDelta, " s") + "（" + localSignedValue(lapPct, " %") + "）。"
    ""
    "快在哪里：平均速度 " + localSignedValue(meanSpeedDelta, " m/s") + ...
        "；最低速度 " + localSignedValue(minSpeedDelta, " m/s") + ...
        "；路径误差 RMSE " + localSignedValue(latRmseDelta, " m") + ...
        "；横摆误差 RMSE " + localSignedValue(yawRmseDelta, " rad/s") + ...
        "；横摆力矩峰值 " + localSignedValue(mzPeakDelta, " Nm") + "。"
    ""
    "主要风险：轮胎峰值利用率变化 " + localSignedValue(tirePeakDelta, "") + ...
        "；有效运行数 " + string(sum(string(runResults.status) == "valid")) + "/" + string(height(runResults)) + ...
        "；停止原因 " + localStopReasons(runResults) + "。"
    ""
    "输出文件：report.html；comparison_metrics.csv；run_results.csv；signal_data/aligned_dyc_comparison.csv；autocross_where_faster.csv。"
    ""
    "验证边界：本报告仅代表当前 CarSim/Simulink 离线仿真结果，不代表 DIL、实时硬件或实车验证。"
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

function text = localStabilityConclusion(whereFasterTable, minSpeedDelta, tirePeakDelta)
pathRows = whereFasterTable(whereFasterTable.scope == "path_tracking", :);
pathOk = any(pathRows.supportsDyc);
hasMixed = (isfinite(minSpeedDelta) && minSpeedDelta < 0) || ...
    (isfinite(tirePeakDelta) && tirePeakDelta > 0);
if pathOk && hasMixed
    text = "稳定性证据混合/不足";
elseif pathOk
    text = "稳定性证据支持 DYC";
else
    text = "稳定性证据不足";
end
end

function text = localStopReasons(runResults)
if ~ismember('stopReason', runResults.Properties.VariableNames)
    text = "unavailable";
    return;
end
reasons = unique(string(runResults.stopReason));
reasons = reasons(strlength(reasons) > 0);
if isempty(reasons)
    text = "unavailable";
else
    text = strjoin(reasons, "；");
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

function value = localMeanColumn(tbl, fieldName)
value = NaN;
if ~ismember(fieldName, tbl.Properties.VariableNames)
    return;
end
values = double(tbl.(fieldName));
values = values(isfinite(values));
if ~isempty(values)
    value = mean(values);
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
