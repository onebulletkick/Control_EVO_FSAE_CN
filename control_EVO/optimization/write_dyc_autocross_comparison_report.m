function cfg = write_dyc_autocross_comparison_report(cfg, runResults, metrics)
%WRITE_DYC_AUTOCROSS_COMPARISON_REPORT 输出 Autocross DYC 开关对比报告产物。

cfg.reportPath = localPathUnderResultsDir(cfg, 'reportPath', 'report.html');
cfg.comparisonMetricsPath = localPathUnderResultsDir(cfg, 'comparisonMetricsPath', 'comparison_metrics.csv');
cfg.runResultsPath = localPathUnderResultsDir(cfg, 'runResultsPath', 'run_results.csv');
cfg.resultMatPath = localPathUnderResultsDir(cfg, 'resultMatPath', 'comparison_result.mat');
cfg.effectivenessAnalysisPath = localPathUnderResultsDir(cfg, 'effectivenessAnalysisPath', 'effectiveness_analysis.txt');
cfg.effectivenessAnalysisRelativePath = localRelativePathUnderFolder(cfg.effectivenessAnalysisPath, cfg.resultsDir);

if ~isfolder(cfg.resultsDir)
    mkdir(cfg.resultsDir);
end

writetable(metrics.summary, cfg.comparisonMetricsPath);
writetable(metrics.perRun, cfg.runResultsPath);
[cfg, analysisArtifacts] = export_dyc_autocross_analysis_data(cfg, runResults);
plotItems = localCreateAnalysisPlots(cfg, runResults);
plotManifest = localTryCreatePresentationPlots(cfg);
whereFasterAnalysis = localTryAnalyzeWhereFaster(cfg, metrics, analysisArtifacts);
analysisLines = whereFasterAnalysis.analysisLines;
localWriteTextFile(cfg.effectivenessAnalysisPath, analysisLines);
save(cfg.resultMatPath, 'cfg', 'runResults', 'metrics', 'analysisArtifacts', ...
    'analysisLines', 'plotItems', 'plotManifest', 'whereFasterAnalysis');
localWriteHtmlReport(cfg, runResults, metrics, analysisArtifacts, ...
    analysisLines, plotItems, plotManifest, whereFasterAnalysis);
end

function localWriteHtmlReport(cfg, runResults, metrics, analysisArtifacts, ...
    analysisLines, plotItems, plotManifest, whereFasterAnalysis)
comparison = metrics.comparison;
summary = metrics.summary;
perRun = metrics.perRun;

html = [
    "<!doctype html>"
    "<html lang=""zh-CN"">"
    "<head>"
    "<meta charset=""UTF-8"">"
    "<title>DYC Autocross 有无控制对比报告</title>"
    "<style>"
    "body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI','Microsoft YaHei',sans-serif;line-height:1.6;margin:0;background:#f6f7f9;color:#1f2933;}"
    "main{max-width:1120px;margin:0 auto;padding:32px 24px 48px;}"
    "h1{font-size:30px;margin:0 0 20px;}h2{font-size:20px;margin:28px 0 12px;}"
    ".card{background:#fff;border:1px solid #d8dee8;border-radius:8px;padding:18px 20px;margin:16px 0;box-shadow:0 1px 2px rgba(15,23,42,.05);}"
    ".grid{display:grid;grid-template-columns:180px 1fr;gap:8px 16px;}.label{font-weight:600;color:#52606d;}"
    "table{width:100%;border-collapse:collapse;background:#fff;font-size:14px;}th,td{border:1px solid #d8dee8;padding:8px 10px;text-align:left;vertical-align:top;}th{background:#e8edf3;}"
    "figure{margin:16px 0;padding:12px;background:#fff;border:1px solid #d8dee8;border-radius:8px;}figcaption{font-weight:600;margin-bottom:8px;}img{max-width:100%;height:auto;border:1px solid #e5e7eb;background:#fff;}"
    ".muted{color:#66788a;}.ok{color:#0f766e;font-weight:600;}.warn{color:#b45309;font-weight:600;}"
    "</style>"
    "</head>"
    "<body><main>"
    "<h1>DYC Autocross 有无控制对比报告</h1>"
    localExperimentSummaryHtml(cfg)
    localConclusionHtml(summary, comparison)
    localReportContentsHtml()
    localWhereFasterHtml(whereFasterAnalysis)
    "<h2>关键指标</h2>"
    localTableToHtml(localSelectedMetricsTable(summary), {'metric','dycOffValue','dycOnValue','delta','unit'})
    localPresentationPlotSectionHtml(plotManifest)
    localDataArtifactsHtml(analysisArtifacts, whereFasterAnalysis, plotManifest)
    "<h2>运行证据</h2>"
    localTableToHtml(perRun, {'caseId','repeatIndex','status','lapTime_s','stopReason','failureReason','lastRunLogPath','lastRunEndPath'})
    localBoundaryHtml()
    "</main></body></html>"
];

localWriteTextFile(cfg.reportPath, html);
end

function plotItems = localCreateAnalysisPlots(cfg, runResults)
plotDir = fullfile(cfg.resultsDir, 'plots');
if ~isfolder(plotDir)
    mkdir(plotDir);
end

specs = [
    struct('kind', 'speed', 'fileName', 'speed_comparison.png', ...
        'title', '速度对比', 'yLabel', 'speed (m/s)')
    struct('kind', 'yaw_rate', 'fileName', 'yaw_rate_comparison.png', ...
        'title', '横摆角速度对比', 'yLabel', 'yaw-rate (rad/s)')
    struct('kind', 'yaw_error', 'fileName', 'yaw_error_comparison.png', ...
        'title', '横摆误差对比', 'yLabel', 'yaw-rate error (rad/s)')
    struct('kind', 'lateral_error', 'fileName', 'lateral_error_comparison.png', ...
        'title', '路径误差对比', 'yLabel', 'lateral error (m)')
    struct('kind', 'yaw_moment', 'fileName', 'yaw_moment_comparison.png', ...
        'title', '横摆力矩对比', 'yLabel', 'Mz (Nm)')
    struct('kind', 'lateral_accel', 'fileName', 'lateral_accel_comparison.png', ...
        'title', '横向加速度对比', 'yLabel', 'Ay (m/s^2)')
    struct('kind', 'longitudinal_accel', 'fileName', 'longitudinal_accel_comparison.png', ...
        'title', '纵向加速度对比', 'yLabel', 'Ax (m/s^2)')
    struct('kind', 'throttle', 'fileName', 'throttle_comparison.png', ...
        'title', '油门开度对比', 'yLabel', 'normalized throttle')
    struct('kind', 'steering', 'fileName', 'steering_comparison.png', ...
        'title', '方向盘转角对比', 'yLabel', 'steering wheel angle (rad)')
    struct('kind', 'tire_utilization', 'fileName', 'tire_utilization_comparison.png', ...
        'title', '轮胎合力利用率对比', 'yLabel', 'max tire force / Fz')
    struct('kind', 'wheel_torque_spread', 'fileName', 'wheel_torque_spread_comparison.png', ...
        'title', '四轮驱动矩离散度对比', 'yLabel', 'max(Twheel)-min(Twheel) (Nm)')
];

plotItems = repmat(struct('title', "", 'relativePath', "", 'path', "", 'available', false, 'failureReason', ""), 0, 1);
for idx = 1:numel(specs)
    spec = specs(idx);
    plotPath = fullfile(plotDir, spec.fileName);
    [isAvailable, failureReason] = localWriteAnalysisPlot(plotPath, spec, runResults);
    if isAvailable
        item = struct();
        item.title = string(spec.title);
        item.relativePath = "plots/" + string(spec.fileName);
        item.path = string(plotPath);
        item.available = true;
        item.failureReason = "";
        plotItems(end+1, 1) = item; %#ok<AGROW>
    else
        item = struct();
        item.title = string(spec.title);
        item.relativePath = "";
        item.path = string(plotPath);
        item.available = false;
        item.failureReason = failureReason;
        plotItems(end+1, 1) = item; %#ok<AGROW>
    end
end
end

function plotManifest = localTryCreatePresentationPlots(cfg)
try
    plotManifest = plot_dyc_autocross_presentation_figures(cfg.resultsDir);
catch err
    plotManifest = localEmptyPresentationManifest();
    plotManifest = [plotManifest; table( ...
        "presentation_plot_manifest.csv", "展示图生成失败", "", "", false, ...
        "展示图未生成：" + string(err.message), ...
        'VariableNames', plotManifest.Properties.VariableNames)]; %#ok<AGROW>
end
end

function manifest = localEmptyPresentationManifest()
manifest = table('Size', [0 6], ...
    'VariableTypes', {'string','string','string','string','logical','string'}, ...
    'VariableNames', {'fileName','title','relativePath','fullPath','available','failureReason'});
end

function analysis = localTryAnalyzeWhereFaster(cfg, metrics, analysisArtifacts)
try
    analysis = analyze_dyc_autocross_report_results(cfg.resultsDir);
catch err
    analysis = struct();
    analysis.resultsDir = string(cfg.resultsDir);
    analysis.whereFasterPath = string(fullfile(cfg.resultsDir, 'autocross_where_faster.csv'));
    analysis.analysisTextPath = string(fullfile(cfg.resultsDir, 'autocross_where_faster_analysis.txt'));
    analysis.whereFasterTable = localEmptyWhereFasterTable();
    analysis.analysisLines = [
        localEffectivenessAnalysisLines(cfg, metrics, analysisArtifacts)
        ""
        "7. 快在哪里分析不可用"
        "自动后处理未生成 autocross_where_faster.csv，原因：" + string(err.message)
    ];
    analysis.conclusion = "快在哪里分析不可用";
end
end

function tbl = localEmptyWhereFasterTable()
tbl = table('Size', [0 9], ...
    'VariableTypes', {'string','string','double','double','double','string','string','logical','string'}, ...
    'VariableNames', {'scope','metric','dycOffValue','dycOnValue','delta','unit','betterDirection','supportsDyc','interpretation'});
end

function [isAvailable, failureReason] = localWriteAnalysisPlot(plotPath, spec, runResults)
isAvailable = false;
failureReason = "";

[offTime, offValue] = localSeriesForCase(runResults, "dyc_off", spec.kind);
[onTime, onValue] = localSeriesForCase(runResults, "dyc_on", spec.kind);
if isempty(offValue) && isempty(onValue)
    failureReason = "dyc_off 和 dyc_on 均缺少 " + string(spec.title) + " 所需信号";
    return;
end

fig = figure('Visible', 'off', 'Color', 'w');
cleanup = onCleanup(@() close(fig));
hold on;
if ~isempty(offValue)
    plot(offTime, offValue, 'LineWidth', 1.4, 'DisplayName', 'dyc_off');
end
if ~isempty(onValue)
    plot(onTime, onValue, 'LineWidth', 1.4, 'DisplayName', 'dyc_on');
end
grid on;
xlabel('time (s)');
ylabel(spec.yLabel);
title(spec.title);
legend('Location', 'best');

try
    exportgraphics(fig, plotPath, 'Resolution', 150);
catch
    saveas(fig, plotPath);
end
isAvailable = isfile(plotPath);
if ~isAvailable
    failureReason = "图像文件写出失败: " + string(plotPath);
end
end

function [time, value] = localSeriesForCase(runResults, caseId, kind)
time = [];
value = [];
run = localFirstSignalRun(runResults, caseId);
if isempty(run) || ~isfield(run, 'signals') || ~isstruct(run.signals)
    return;
end

signals = run.signals;
switch string(kind)
    case "speed"
        value = localVectorField(signals, 'speed_mps');
    case "yaw_rate"
        value = localVectorField(signals, 'yawRate_radps');
    case "yaw_error"
        value = localErrorVector(signals, 'yawRate_radps', 'yawRateTarget_radps');
    case "lateral_error"
        value = localErrorVector(signals, 'latVeh_m', 'latTarget_m');
    case "yaw_moment"
        value = localVectorField(signals, 'mz_Nm');
    case "lateral_accel"
        value = localVectorField(signals, 'ay_mps2');
    case "longitudinal_accel"
        value = localVectorField(signals, 'ax_mps2');
    case "throttle"
        value = localVectorField(signals, 'throttle');
    case "steering"
        value = localVectorField(signals, 'steerSW_rad');
    case "tire_utilization"
        value = localTireUtilizationMax(signals);
    case "wheel_torque_spread"
        value = localWheelTorqueSpread(signals);
end

if isempty(value)
    return;
end

time = localVectorField(signals, 'time_s');
if numel(time) < numel(value)
    time = (0:numel(value)-1)';
else
    time = time(1:numel(value));
end
end

function run = localFirstSignalRun(runResults, caseId)
run = [];
fallbackRun = [];
for idx = 1:numel(runResults)
    if string(localStructField(runResults(idx), 'caseId')) ~= string(caseId)
        continue;
    end
    if ~localRunHasSignalData(runResults(idx))
        continue;
    end
    if string(localStructField(runResults(idx), 'status')) == "valid"
        run = runResults(idx);
        return;
    end
    if isempty(fallbackRun)
        fallbackRun = runResults(idx);
    end
end
run = fallbackRun;
end

function tf = localRunHasSignalData(run)
tf = false;
if ~isstruct(run) || ~isfield(run, 'signals') || ~isstruct(run.signals)
    return;
end
fields = {'time_s','speed_mps','yawRate_radps','yawRateTarget_radps', ...
    'latVeh_m','latTarget_m','ay_mps2','ax_mps2','mz_Nm', ...
    'station_m','throttle','steerSW_rad','myDrL1_Nm','tireFxL1_N'};
for idx = 1:numel(fields)
    if isfield(run.signals, fields{idx}) && ~isempty(run.signals.(fields{idx}))
        tf = true;
        return;
    end
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
    if count < 1
        continue;
    end
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

function value = localErrorVector(signals, actualField, targetField)
actual = localVectorField(signals, actualField);
target = localVectorField(signals, targetField);
if isempty(actual) || isempty(target)
    value = [];
    return;
end
n = min(numel(actual), numel(target));
value = actual(1:n) - target(1:n);
end

function value = localVectorField(signals, fieldName)
value = [];
if ~isstruct(signals) || ~isfield(signals, fieldName)
    return;
end
value = double(signals.(fieldName));
value = value(:);
if isempty(value) || ~any(isfinite(value))
    value = [];
end
end

function value = localStructField(s, fieldName)
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
else
    value = "";
end
end

function html = localExperimentSummaryHtml(cfg)
html = [
    "<h2>报告概览</h2>"
    "<section class=""card grid"">"
    "<div class=""label"">工况</div><div>" + localEscape(localNestedField(cfg, {'scenario','displayName'})) + "</div>"
    "<div class=""label"">modelName</div><div>" + localEscape(localField(cfg, 'modelName')) + "</div>"
    "<div class=""label"">dyc_off</div><div>Kp/Ki/Kd = 0/0/0</div>"
    "<div class=""label"">dyc_on</div><div>Kp/Ki/Kd = 6000/200/0</div>"
    "<div class=""label"">simfilePath</div><div>" + localEscape(localField(cfg, 'simfilePath')) + "</div>"
    "<div class=""label"">stopTime</div><div>" + localEscape(localNestedField(cfg, {'simulation','stopTime'})) + "</div>"
    "<div class=""label"">repeatCount</div><div>" + localEscape(localField(cfg, 'repeatCount')) + "</div>"
    "<div class=""label"">resultsDir</div><div>" + localEscape(localField(cfg, 'resultsDir')) + "</div>"
    "</section>"
];
end

function html = localConclusionHtml(summary, comparison)
html = [
    "<h2>核心结论</h2>"
    "<section class=""card"">"
];

if string(comparison.status) == "valid"
    offLap = localSummaryValue(summary, "dyc_off", "lapTime_s");
    onLap = localSummaryValue(summary, "dyc_on", "lapTime_s");
    delta = comparison.lapTimeDelta_s;
    pct = comparison.lapTimeDelta_pct;
    html = [
        html
        "<p class=""ok"">完成时间有效。dyc_on - dyc_off = " + localFormatSignedValue(delta, " s") + "（" + localFormatSignedValue(pct, " %") + "）。</p>"
        "<p>DYC 关闭：" + localFormatValue(offLap, " s") + "；DYC 开启：" + localFormatValue(onLap, " s") + "。</p>"
    ];
else
    reason = localEscape(localStringValue(comparison.failureReason));
    html = [
        html
        "<p class=""warn"">当前数据不输出完成时间结论。原因：" + reason + "</p>"
    ];
end

html = [html; "</section>"];
end

function html = localReportContentsHtml()
items = [
    "圈速与完成时间差"
    "平均/最低速度保持"
    "路径误差与横摆误差"
    "横摆力矩介入"
    "轮胎利用率"
    "驱动矩分配"
    "展示图与导出文件索引"
];
html = [
    "<h2>报告内容</h2>"
    "<section class=""card""><ul>"
];
for idx = 1:numel(items)
    html = [html; "<li>" + localEscape(items(idx)) + "</li>"]; %#ok<AGROW>
end
html = [html; "</ul></section>"];
end

function tbl = localSelectedMetricsTable(summary)
specs = [
    struct('label', "完成时间", 'field', "lapTime_s", 'unit', "s")
    struct('label', "平均速度", 'field', "meanSpeed_mps", 'unit', "m/s")
    struct('label', "最低速度", 'field', "minSpeed_mps", 'unit', "m/s")
    struct('label', "路径误差 RMSE", 'field', "lateralErrorRmse", 'unit', "m")
    struct('label', "横摆误差 RMSE", 'field', "yawRateRmse", 'unit', "rad/s")
    struct('label', "轮胎峰值利用率", 'field', "tireUtilPeak", 'unit', "")
    struct('label', "横摆力矩峰值", 'field', "mzPeakAbs_Nm", 'unit', "Nm")
    struct('label', "驱动矩离散度峰值", 'field', "wheelTorqueSpreadPeak_Nm", 'unit', "Nm")
];
tbl = table('Size', [0 5], ...
    'VariableTypes', {'string','double','double','double','string'}, ...
    'VariableNames', {'metric','dycOffValue','dycOnValue','delta','unit'});
for idx = 1:numel(specs)
    off = localSummaryValue(summary, "dyc_off", specs(idx).field);
    on = localSummaryValue(summary, "dyc_on", specs(idx).field);
    tbl = [tbl; table(specs(idx).label, off, on, on - off, specs(idx).unit, ...
        'VariableNames', tbl.Properties.VariableNames)]; %#ok<AGROW>
end
end

function lines = localEffectivenessAnalysisLines(cfg, metrics, analysisArtifacts)
summary = metrics.summary;
comparison = metrics.comparison;
if string(comparison.status) == "valid"
    lapDelta = double(comparison.lapTimeDelta_s);
    lapPct = double(comparison.lapTimeDelta_pct);
    verdict = "完成时间可用";
else
    lapDelta = NaN;
    lapPct = NaN;
    verdict = "完成时间不可用：" + localStringValue(comparison.failureReason);
end

lines = [
    "Autocross DYC 有效性分析"
    ""
    "总体判断：" + verdict
    ""
    "完成时间：dyc_on - dyc_off = " + localFormatSignedValue(lapDelta, " s") + "（" + localFormatSignedValue(lapPct, " %") + "）。"
    ""
    "快在哪里：详见 autocross_where_faster.csv。"
    ""
    "主要风险：后处理对齐数据不可用时，HTML 仍保留运行证据和基础指标。"
    ""
    "输出文件：comparison_metrics.csv；run_results.csv；" + localArtifactField(analysisArtifacts, 'manifestRelativePath') + "；" + localArtifactField(analysisArtifacts, 'analysisDataMatRelativePath') + "。"
    ""
    "验证边界：本报告仅代表当前 CarSim/Simulink 离线仿真结果，不代表 DIL、实时硬件或实车验证。"
];
end

function html = localWhereFasterHtml(whereFasterAnalysis)
html = [
    "<h2>快在哪里</h2>"
    "<section class=""card"">"
];

if ~isstruct(whereFasterAnalysis) || ~isfield(whereFasterAnalysis, 'whereFasterTable') || ...
        height(whereFasterAnalysis.whereFasterTable) == 0
    html = [
        html
        "<p>当前结果缺少可用于“快在哪里”分析的对齐时序数据，因此本章节不输出分项表。</p>"
        "</section>"
    ];
    return;
end

html = [
    html
    "<p>delta 均为 dyc_on - dyc_off。</p>"
    localTableToHtml(whereFasterAnalysis.whereFasterTable, ...
        {'metric','dycOffValue','dycOnValue','delta','unit','interpretation'})
    "</section>"
];
end

function text = localArtifactField(artifacts, fieldName)
text = "unavailable";
if isstruct(artifacts) && isfield(artifacts, fieldName)
    candidate = localStringValue(artifacts.(fieldName));
    if strlength(candidate) > 0
        text = candidate;
    end
end
end

function html = localPresentationPlotSectionHtml(plotManifest)
html = [
    "<h2>展示图</h2>"
    "<section class=""card"">"
];

if isempty(plotManifest) || height(plotManifest) == 0 || ~ismember('available', plotManifest.Properties.VariableNames)
    html = [
        html
        "<p>当前未生成展示版图表。</p>"
        "</section>"
    ];
    return;
end

availableRows = plotManifest(plotManifest.available, :);
if height(availableRows) == 0
    html = [
        html
        "<p>展示版图表未生成。可检查 signal_data/aligned_dyc_comparison.csv 与 comparison_metrics.csv 是否完整。</p>"
        "</section>"
    ];
    return;
end

html = [
    html
    "<p>以下为展示版图表，原始分析图仍保存在 plots/ 目录。</p>"
];
for idx = 1:height(availableRows)
    html = [
        html
        "<figure>"
        "<figcaption>" + localEscape(availableRows.title(idx)) + "</figcaption>"
        "<img src=""" + localEscape(availableRows.relativePath(idx)) + """ alt=""" + localEscape(availableRows.title(idx)) + """>"
        "</figure>"
    ]; %#ok<AGROW>
end

html = [html; "</section>"];
end

function html = localDataArtifactsHtml(analysisArtifacts, whereFasterAnalysis, plotManifest)
html = [
    "<h2>输出文件</h2>"
    "<section class=""card"">"
    "<p>报告、表格、MAT 数据包和图表均保存在本次结果目录下。</p>"
];

artifactTable = table( ...
    ["HTML 报告"; "文字结论"; "指标汇总"; "单次运行"; "信号清单"; "对齐对比数据"; "MAT 数据包"; "快在哪里分析表"; "快在哪里文字分析"; "原始分析图"; "展示图清单"], ...
    ["report.html"; "effectiveness_analysis.txt"; "comparison_metrics.csv"; "run_results.csv"; ...
    analysisArtifacts.manifestRelativePath; analysisArtifacts.alignedComparisonRelativePath; ...
    analysisArtifacts.analysisDataMatRelativePath; localRelativeArtifactPath(whereFasterAnalysis, 'whereFasterPath'); ...
    localRelativeArtifactPath(whereFasterAnalysis, 'analysisTextPath'); "plots/"; localPresentationManifestRelativePath(plotManifest)], ...
    ["当前仪表盘页面"; "精简文字结论"; "dyc_off/dyc_on 核心指标"; "每次运行状态和停止原因"; ...
    "每个 case/repeat 的导出状态、来源和行数"; "按 dyc_off 时间轴插值对齐后的时序差值"; ...
    "manifest、alignedComparison 和 timeSeriesTables"; "完成时间、速度、路径误差、横摆力矩和轮胎利用率的可读汇总"; ...
    "精简文字结论"; "原始时序分析 PNG"; "plots_presentation 目录中的展示版 PNG 清单"], ...
    'VariableNames', {'artifact','relativePath','description'});

if isfield(analysisArtifacts, 'timeSeriesTables') && ~isempty(analysisArtifacts.timeSeriesTables)
    for idx = 1:numel(analysisArtifacts.timeSeriesTables)
        item = analysisArtifacts.timeSeriesTables(idx);
        artifactTable = [artifactTable; table( ...
            "时序数据 " + string(item.caseId) + " repeat" + string(item.repeatIndex), ...
            string(item.relativePath), ...
            "单次运行原始/派生时序信号", ...
            'VariableNames', artifactTable.Properties.VariableNames)]; %#ok<AGROW>
    end
end

html = [
    html
    localTableToHtml(artifactTable, artifactTable.Properties.VariableNames)
    "</section>"
];
end

function relativePath = localRelativeArtifactPath(analysis, fieldName)
relativePath = "unavailable";
if ~isstruct(analysis) || ~isfield(analysis, fieldName)
    return;
end
pathText = string(analysis.(fieldName));
if contains(pathText, "autocross_where_faster.csv")
    relativePath = "autocross_where_faster.csv";
elseif contains(pathText, "autocross_where_faster_analysis.txt")
    relativePath = "autocross_where_faster_analysis.txt";
elseif strlength(pathText) > 0
    relativePath = pathText;
end
end

function relativePath = localPresentationManifestRelativePath(plotManifest)
relativePath = "plots_presentation/presentation_plot_manifest.csv";
if isempty(plotManifest) || height(plotManifest) == 0
    return;
end
if ismember('relativePath', plotManifest.Properties.VariableNames) && any(strlength(plotManifest.relativePath) > 0)
    return;
end
if ismember('fileName', plotManifest.Properties.VariableNames) && any(plotManifest.fileName == "presentation_plot_manifest.csv")
    return;
end
end

function html = localInterventionHtml(metrics)
summary = metrics.summary;
mzFields = {'mzRms_Nm','mzPeakAbs_Nm','mzAbsIntegral_Nms','interventionRatio'};
dycOnSummary = localCaseSummaryRow(summary, "dyc_on");
hasMz = height(dycOnSummary) == 1 && localHasFiniteMetric(dycOnSummary, mzFields);

html = [
    "<h2>控制介入</h2>"
    "<section class=""card"">"
];

if ~hasMz
    html = [
        html
        "<p>控制介入信号不可用：mz_Nm 或相关介入指标缺失，因此本报告不根据横摆力矩幅值判断控制器实际介入强度。</p>"
        "</section>"
    ];
    return;
end

html = [
    html
    "<p>下表列出 dyc_on 的横摆力矩 RMS、峰值、绝对积分和介入占比。数值越高通常表示 DYC 对车辆横摆运动施加了更强或更频繁的修正，但仍需结合 yaw tracking、路径偏差和圈速变化共同判断。</p>"
    localTableToHtml(dycOnSummary, [{'caseId','displayName'}, mzFields])
];

if isfield(metrics, 'delta') && ~isempty(metrics.delta)
    html = [
        html
        "<p>相对 dyc_off 的控制介入差值如下，按 dyc_on - dyc_off 计算。</p>"
        localTableToHtml(metrics.delta, {'mzRmsDelta_Nm','mzPeakAbsDelta_Nm','mzAbsIntegralDelta_Nms','interventionRatioDelta'})
    ];
end

html = [html; "</section>"];
end

function html = localBoundaryHtml()
html = [
    "<h2>验证边界</h2>"
    "<section class=""card"">"
    "<p>本报告仅代表当前 CarSim/Simulink 离线仿真结果，不代表 DIL、实时硬件或实车验证。</p>"
    "</section>"
];
end

function html = localTableToHtml(tbl, columns)
columns = columns(ismember(columns, tbl.Properties.VariableNames));
html = "<table><thead><tr>";
for idx = 1:numel(columns)
    html = html + "<th>" + localEscape(columns{idx}) + "</th>";
end
html = html + "</tr></thead><tbody>";
for rowIdx = 1:height(tbl)
    html = html + "<tr>";
    for colIdx = 1:numel(columns)
        value = tbl.(columns{colIdx})(rowIdx);
        html = html + "<td>" + localEscape(localDisplayValue(value)) + "</td>";
    end
    html = html + "</tr>";
end
html = html + "</tbody></table>";
end

function value = localSummaryValue(summary, caseId, fieldName)
row = localCaseSummaryRow(summary, caseId);
if height(row) ~= 1 || ~ismember(fieldName, summary.Properties.VariableNames)
    value = NaN;
else
    value = row.(fieldName);
end
end

function row = localCaseSummaryRow(summary, caseId)
row = summary(summary.caseId == string(caseId), :);
end

function tf = localHasFiniteMetric(row, fields)
tf = false;
for idx = 1:numel(fields)
    if ismember(fields{idx}, row.Properties.VariableNames) && any(isfinite(row.(fields{idx})))
        tf = true;
        return;
    end
end
end

function text = localFormatValue(value, suffix)
value = double(value);
if isempty(value) || ~isfinite(value(1))
    text = "unavailable";
else
    text = sprintf('%.4f%s', value(1), suffix);
end
end

function text = localFormatSignedValue(value, suffix)
value = double(value);
if isempty(value) || ~isfinite(value(1))
    text = "unavailable";
else
    text = sprintf('%+.4f%s', value(1), suffix);
end
end

function text = localDisplayValue(value)
if isnumeric(value) || islogical(value)
    value = double(value);
    if isempty(value) || ~isfinite(value(1))
        text = "unavailable";
    else
        text = string(sprintf('%.6g', value(1)));
    end
else
    text = localStringValue(value);
    if strlength(text) == 0
        text = "";
    end
end
end

function text = localStringValue(value)
text = string(value);
if isempty(text)
    text = "";
else
    text = text(1);
end
end

function value = localField(cfg, fieldName)
if isfield(cfg, fieldName)
    value = cfg.(fieldName);
else
    value = "";
end
end

function value = localNestedField(cfg, names)
value = cfg;
for idx = 1:numel(names)
    name = names{idx};
    if isstruct(value) && isfield(value, name)
        value = value.(name);
    else
        value = "";
        return;
    end
end
end

function text = localEscape(value)
text = localStringValue(value);
text = replace(text, "&", "&amp;");
text = replace(text, "<", "&lt;");
text = replace(text, ">", "&gt;");
text = replace(text, """", "&quot;");
text = replace(text, "'", "&#39;");
end

function localWriteTextFile(pathValue, lines)
fid = fopen(pathValue, 'w', 'n', 'UTF-8');
if fid < 0
    error('dyc:autocrossComparison:ReportWriteFailed', 'Unable to write report: %s', char(string(pathValue)));
end
cleanup = onCleanup(@() fclose(fid));
for idx = 1:numel(lines)
    fprintf(fid, '%s\n', char(lines(idx)));
end
end

function pathOut = localPathUnderResultsDir(cfg, fieldName, fileName)
if isfield(cfg, fieldName)
    candidatePath = cfg.(fieldName);
else
    candidatePath = fullfile(cfg.resultsDir, fileName);
end

if localCanonicalParentIsUnderFolder(candidatePath, cfg.resultsDir)
    pathOut = candidatePath;
elseif localRawPathClaimsUnderFolder(candidatePath, cfg.resultsDir)
    error('dyc:autocrossComparison:ArtifactPathOutsideResultsDir', ...
        'Autocross comparison artifact path for %s escapes resultsDir after canonical normalization: %s', ...
        fieldName, char(string(candidatePath)));
else
    pathOut = fullfile(cfg.resultsDir, fileName);
end
end

function relativePath = localRelativePathUnderFolder(pathValue, folderValue)
pathText = string(localCanonicalPath(pathValue));
folderText = string(localCanonicalPath(folderValue));
if ~localPathIsUnderFolder(pathText, folderText)
    relativePath = localStringValue(pathValue);
    return;
end
if pathText == folderText
    relativePath = "";
    return;
end
relativePath = extractAfter(pathText, strlength(folderText) + 1);
relativePath = replace(relativePath, string(filesep), "/");
end

function tf = localCanonicalParentIsUnderFolder(pathValue, folderValue)
targetParent = fileparts(char(string(pathValue)));
if isempty(targetParent)
    targetParent = pwd;
end
tf = localPathIsUnderFolder(localCanonicalPath(targetParent), localCanonicalPath(folderValue));
end

function tf = localRawPathClaimsUnderFolder(pathValue, folderValue)
targetPath = char(java.io.File(char(string(pathValue))).getAbsolutePath());
folderPath = char(java.io.File(char(string(folderValue))).getAbsolutePath());
tf = localPathIsUnderFolder(string(targetPath), string(folderPath));
end

function canonicalPath = localCanonicalPath(pathValue)
canonicalPath = string(char(java.io.File(char(string(pathValue))).getCanonicalPath()));
end

function tf = localPathIsUnderFolder(pathValue, folderValue)
pathText = string(pathValue);
folderText = string(folderValue);
folderPrefix = folderText + string(filesep);
if ispc
    tf = strcmpi(char(pathText), char(folderText)) || startsWith(pathText, folderPrefix, 'IgnoreCase', true);
else
    tf = pathText == folderText || startsWith(pathText, folderPrefix);
end
end
