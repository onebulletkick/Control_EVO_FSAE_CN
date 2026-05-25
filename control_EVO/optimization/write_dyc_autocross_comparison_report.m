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
    localDetailedEffectivenessHtml(cfg, analysisLines)
    localWhereFasterHtml(whereFasterAnalysis)
    "<h2>关键指标</h2>"
    localTableToHtml(summary, summary.Properties.VariableNames)
    localMechanismHtml()
    localMechanismDeltaHtml(metrics)
    localPlotSectionHtml(plotItems)
    localPresentationPlotSectionHtml(plotManifest)
    localDataArtifactsHtml(analysisArtifacts, whereFasterAnalysis, plotManifest)
    localInterventionHtml(metrics)
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
    "<h2>实验摘要</h2>"
    "<section class=""card grid"">"
    "<div class=""label"">modelName</div><div>" + localEscape(localField(cfg, 'modelName')) + "</div>"
    "<div class=""label"">simfilePath</div><div>" + localEscape(localField(cfg, 'simfilePath')) + "</div>"
    "<div class=""label"">stopTime</div><div>" + localEscape(localNestedField(cfg, {'simulation','stopTime'})) + "</div>"
    "<div class=""label"">repeatCount</div><div>" + localEscape(localField(cfg, 'repeatCount')) + "</div>"
    "<div class=""label"">resultsDir</div><div>" + localEscape(localField(cfg, 'resultsDir')) + "</div>"
    "</section>"
];
end

function html = localConclusionHtml(summary, comparison)
html = [
    "<h2>结论</h2>"
    "<section class=""card"">"
];

if string(comparison.status) == "valid"
    offLap = localSummaryValue(summary, "dyc_off", "lapTime_s");
    onLap = localSummaryValue(summary, "dyc_on", "lapTime_s");
    delta = comparison.lapTimeDelta_s;
    pct = comparison.lapTimeDelta_pct;
    html = [
        html
        "<p class=""ok"">comparison.status = valid</p>"
        "<p>DYC 关闭圈速：" + localFormatValue(offLap, " s") + "；DYC 开启圈速：" + localFormatValue(onLap, " s") + "；圈速差值：" + localFormatSignedValue(delta, " s") + "（" + localFormatSignedValue(pct, " %") + "）。</p>"
    ];
else
    reason = localEscape(localStringValue(comparison.failureReason));
    html = [
        html
        "<p class=""warn"">comparison.status = invalid</p>"
        "<p>当前数据不输出 DYC 有效性结论。原因：" + reason + "</p>"
    ];
end

html = [html; "</section>"];
end

function lines = localEffectivenessAnalysisLines(cfg, metrics, analysisArtifacts)
summary = metrics.summary;
comparison = metrics.comparison;

offLap = localSummaryValue(summary, "dyc_off", "lapTime_s");
onLap = localSummaryValue(summary, "dyc_on", "lapTime_s");

lines = [
    "详细 DYC 有效性分析"
    ""
    "1. 有效性判断"
];

if string(comparison.status) == "valid"
    lapDelta = double(comparison.lapTimeDelta_s);
    lapPct = double(comparison.lapTimeDelta_pct);
    if isfinite(lapDelta) && lapDelta < 0
        verdict = "当前单次 Autocross 仿真支持 DYC 有效：DYC 开启后圈速缩短，同时需要结合横向路径误差、速度保持、横摆力矩和轮胎利用率判断收益来源。";
    elseif isfinite(lapDelta) && lapDelta > 0
        verdict = "当前单次 Autocross 仿真不支持 DYC 带来圈速收益：DYC 开启后圈速变慢，需要继续检查控制介入是否过强、轮胎利用率是否恶化或速度保持是否受损。";
    else
        verdict = "当前单次 Autocross 仿真圈速差值接近零，不能只凭圈速判断 DYC 是否有效，需要依赖机理指标继续分析。";
    end
    lines = [
        lines
        verdict
        "圈速：dyc_off = " + localFormatValue(offLap, " s") + "，dyc_on = " + localFormatValue(onLap, " s") + "，dyc_on - dyc_off = " + localFormatSignedValue(lapDelta, " s") + "（" + localFormatSignedValue(lapPct, " %") + "）。"
    ];
else
    lines = [
        lines
        "当前数据不输出 DYC 有效性结论，因为 comparison.status = invalid。"
        "失败原因：" + localStringValue(comparison.failureReason)
        "圈速：dyc_off = " + localFormatValue(offLap, " s") + "，dyc_on = " + localFormatValue(onLap, " s") + "。"
    ];
end

lines = [
    lines
    ""
    "2. 独立后处理数据"
    "本次数据不是只写进 HTML，而是单独落盘，后续可用 MATLAB、Python 或 Excel 继续处理。"
    "- 指标汇总表：" + localStringValue(localField(cfg, 'comparisonMetricsPath'))
    "- 单次运行表：" + localStringValue(localField(cfg, 'runResultsPath'))
    "- 时序清单：" + localArtifactField(analysisArtifacts, 'manifestRelativePath')
    "- 对齐后的 dyc_on - dyc_off 时序差值：" + localArtifactField(analysisArtifacts, 'alignedComparisonRelativePath')
    "- MAT 数据包：" + localArtifactField(analysisArtifacts, 'analysisDataMatRelativePath')
    ""
    "3. 机理证据链"
    localEffectivenessMetricLine("横向路径误差 RMSE", summary, "lateralErrorRmse", metrics, "lateralErrorRmseDelta", " m", "负值表示 DYC 开启后车辆横向位置更接近目标轨迹。")
    localEffectivenessMetricLine("速度保持均值", summary, "meanSpeed_mps", metrics, "meanSpeedDelta_mps", " m/s", "正值表示 DYC 没有通过明显牺牲平均速度换取稳定性。")
    localEffectivenessMetricLine("最低速度", summary, "minSpeed_mps", metrics, "minSpeedDelta_mps", " m/s", "正值通常意味着弯中或低速段速度保持更好。")
    localEffectivenessMetricLine("油门均值", summary, "throttleMean", metrics, "throttleMeanDelta", "", "若速度提升同时油门均值下降，说明收益更可能来自姿态/路径效率而非单纯加大油门。")
    localEffectivenessMetricLine("轮胎利用率峰值", summary, "tireUtilPeak", metrics, "tireUtilPeakDelta", "", "负值表示最大轮胎合力利用率下降，说明峰值饱和风险降低；该值来自 CarSim 导出 Fx/Fy/Fz 后处理。")
    localEffectivenessMetricLine("四轮驱动矩离散度峰值", summary, "wheelTorqueSpreadPeak_Nm", metrics, "wheelTorqueSpreadPeakDelta_Nm", " Nm", "正值表示控制器通过左右/前后轮驱动矩差异制造附加横摆力矩。")
    localEffectivenessMetricLine("横摆力矩峰值", summary, "mzPeakAbs_Nm", metrics, "mzPeakAbsDelta_Nm", " Nm", "正值表示 DYC 实际输出了更强的横摆修正能力。")
    localEffectivenessMetricLine("横摆力矩绝对积分", summary, "mzAbsIntegral_Nms", metrics, "mzAbsIntegralDelta_Nms", " Nms", "正值表示整圈控制介入总量增加。")
    localEffectivenessMetricLine("控制介入占比", summary, "interventionRatio", metrics, "interventionRatioDelta", "", "正值表示 DYC 在更多时间片超过横摆力矩有效阈值。")
    ""
    "4. 为什么 DYC 有效"
    "DYC 的直接作用不是增加整车总驱动力，而是通过四轮驱动矩差异形成附加横摆力矩，让车辆横摆响应更接近目标。"
    "如果圈速缩短、横向路径误差下降、速度保持没有恶化，同时横摆力矩和四轮驱动矩离散度明显增加，就能形成一条较完整的有效性证据链：控制器介入 -> 姿态/路径改善 -> 弯中或出弯效率提升 -> 圈速改善。"
    "轮胎利用率用于解释收益是否来自更合理的轮胎负荷分配。如果 DYC 开启后峰值利用率下降或没有恶化，说明控制器没有单纯把车辆推向更强饱和。"
    ""
    "5. 还能继续分析的数据"
    "已导出的 aligned_dyc_comparison.csv 还包含 station、speed、yaw-rate、lateral error、Ay/Ax、Mz、throttle、steer、tireUtilMax、wheelTorqueSpread 和 leftRightDriveTorqueDelta 等列，可继续做分段/弯道级分析。"
    "值得继续深挖的方向包括：按 station 分段统计入口/弯中/出弯速度，统计轮胎利用率超过阈值的持续时间，分析 leftRightDriveTorqueDelta 与 Mz 的对应关系，以及观察油门开度和纵向加速度是否说明 DYC 改善了出弯牵引。"
    ""
    "6. 证据不足与边界"
    "本结论来自单次 Autocross 仿真或当前 repeat 配置，不等同于统计显著结论；若 repeatCount 增加，应比较均值、标准差和最差圈。"
    "当前结果只代表离线 Simulink/CarSim 协同仿真，不代表 DIL、实时硬件或实车验证。"
    "如果 CarSim Export 未包含目标横摆角速度、方向盘转角或更多轮胎通道，相关 yaw tracking、driver demand 和 per-wheel tire saturation 结论只能标记为不可用，不能强行外推。"
];
end

function html = localDetailedEffectivenessHtml(cfg, analysisLines)
relativePath = localStringValue(localField(cfg, 'effectivenessAnalysisRelativePath'));
if strlength(relativePath) == 0
    relativePath = "effectiveness_analysis.txt";
end

html = [
    "<h2>详细有效性分析</h2>"
    "<section class=""card"">"
    "<p>下面把圈速、误差、速度保持、控制介入和轮胎利用率组织成单独的证据链。同一份文字分析已写入 <a href=""" + localEscape(relativePath) + """>" + localEscape(relativePath) + "</a>，便于脱离 HTML 继续引用。</p>"
];

for idx = 1:numel(analysisLines)
    line = strtrim(string(analysisLines(idx)));
    if strlength(line) == 0 || line == "详细 DYC 有效性分析"
        continue;
    end
    if ~isempty(regexp(char(line), '^\d+\.\s', 'once'))
        html = [html; "<h3>" + localEscape(line) + "</h3>"]; %#ok<AGROW>
    else
        html = [html; "<p>" + localEscape(line) + "</p>"]; %#ok<AGROW>
    end
end

html = [html; "</section>"];
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
    "<p>下表从已导出的 CSV/MAT 结果中汇总 DYC 开启后相对关闭状态的变化。delta 均为 dyc_on - dyc_off；supportsDyc 表示该分项是否支持 DYC 有效。</p>"
    localTableToHtml(whereFasterAnalysis.whereFasterTable, ...
        {'scope','metric','dycOffValue','dycOnValue','delta','unit','supportsDyc','interpretation'})
    "</section>"
];
end

function line = localEffectivenessMetricLine(label, summary, metricName, metrics, deltaName, suffix, interpretation)
offValue = localSummaryValue(summary, "dyc_off", metricName);
onValue = localSummaryValue(summary, "dyc_on", metricName);
deltaValue = localDeltaValue(metrics, deltaName);
line = "- " + string(label) + "：dyc_off = " + localFormatValue(offValue, suffix) + ...
    "，dyc_on = " + localFormatValue(onValue, suffix) + ...
    "，dyc_on - dyc_off = " + localFormatSignedValue(deltaValue, suffix) + ...
    "。" + string(interpretation);
end

function value = localDeltaValue(metrics, fieldName)
value = NaN;
if ~isfield(metrics, 'delta') || isempty(metrics.delta)
    return;
end
delta = metrics.delta;
if istable(delta) && ismember(fieldName, delta.Properties.VariableNames) && height(delta) >= 1
    value = delta.(fieldName)(1);
end
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

function html = localMechanismHtml()
html = [
    "<h2>机理解释</h2>"
    "<section class=""card"">"
    "<p>Autocross 工况的圈速不仅取决于纵向加速，还取决于车辆能否稳定跟踪目标横摆响应并减少路径偏差。DYC 通过附加横摆力矩修正 yaw tracking error，在转向和横向加速度快速变化时帮助车辆更接近期望姿态。</p>"
    "<p>本报告将横摆跟踪、横向路径偏差、横向响应、速度保持和控制介入强度放在一起观察：如果 DYC 开启后 yaw-rate error 与 lateral error 下降，同时平均速度或最小速度没有明显恶化，并且控制介入信号存在合理幅值，则可以解释为控制器在弯中姿态和出弯速度之间取得了更好的折中。</p>"
    "</section>"
];
end

function html = localMechanismDeltaHtml(metrics)
if ~isfield(metrics, 'delta') || isempty(metrics.delta)
    html = "";
    return;
end

columns = {'baselineCaseId','testCaseId','status','failureReason', ...
    'yawRateRmseDelta','yawRateMaeDelta','yawRatePeakErrorDelta', ...
    'lateralErrorRmseDelta','lateralErrorPeakDelta','ayPeakAbsDelta','ayRmsDelta', ...
    'ayStdDelta','axPeakAbsDelta','axRmsDelta','meanSpeedDelta_mps', ...
    'minSpeedDelta_mps','speedStdDelta_mps','stationEndDelta_m', ...
    'throttleMeanDelta','throttlePeakDelta','steerRmsDelta_rad','steerPeakAbsDelta_rad', ...
    'tireUtilPeakDelta','tireUtilMeanDelta','wheelTorqueSpreadRmsDelta_Nm', ...
    'wheelTorqueSpreadPeakDelta_Nm', ...
    'mzRmsDelta_Nm','mzPeakAbsDelta_Nm','mzAbsIntegralDelta_Nms','interventionRatioDelta'};
html = [
    "<h2>机理指标差值</h2>"
    "<section class=""card"">"
    "<p>下表按 dyc_on - dyc_off 计算横摆跟踪、路径偏差、速度保持和控制介入指标差值。负的误差差值通常表示 DYC 开启后误差降低，正的速度差值表示速度指标提高。</p>"
    localTableToHtml(metrics.delta, columns)
    "</section>"
];
end

function html = localPlotSectionHtml(plotItems)
html = [
    "<h2>数据分析图</h2>"
    "<section class=""card"">"
];

availableItems = plotItems([plotItems.available]);
if isempty(availableItems)
    html = [
        html
        "<p>当前运行结果中缺少可用于绘图的时序信号，因此未生成速度、横摆误差、路径误差或横摆力矩曲线。</p>"
        "</section>"
    ];
    return;
end

html = [
    html
    "<p>以下图表由 MATLAB 根据本次 runResults.signals 生成，用于辅助观察 DYC 开启前后的速度保持、横摆跟踪、路径误差和控制介入差异。</p>"
];
for idx = 1:numel(availableItems)
    html = [
        html
        "<figure>"
        "<figcaption>" + localEscape(availableItems(idx).title) + "</figcaption>"
        "<img src=""" + localEscape(availableItems(idx).relativePath) + """ alt=""" + localEscape(availableItems(idx).title) + """>"
        "</figure>"
    ];
end

html = [html; "</section>"];
end

function html = localPresentationPlotSectionHtml(plotManifest)
html = [
    "<h2>展示版图表</h2>"
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
    "<p>以下图表由 MATLAB 从报告结果目录重新绘制，适合直接用于汇报展示。</p>"
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
    "<h2>可复用数据文件</h2>"
    "<section class=""card"">"
    "<p>本次仿真提取出的时序数据已单独导出，可直接用 MATLAB、Python、Excel 或其他工具继续做后处理。原始 case 时序、对齐后的 dyc_on - dyc_off 对比数据、MAT 数据包和“快在哪里”分析都保存在结果目录。</p>"
];

artifactTable = table( ...
    ["信号清单"; "对齐对比数据"; "MAT 数据包"; "快在哪里分析表"; "快在哪里文字分析"; "展示版图表清单"], ...
    [analysisArtifacts.manifestRelativePath; analysisArtifacts.alignedComparisonRelativePath; ...
    analysisArtifacts.analysisDataMatRelativePath; localRelativeArtifactPath(whereFasterAnalysis, 'whereFasterPath'); ...
    localRelativeArtifactPath(whereFasterAnalysis, 'analysisTextPath'); localPresentationManifestRelativePath(plotManifest)], ...
    ["每个 case/repeat 的导出状态、来源和行数"; "按 dyc_off 时间轴插值对齐后的时序差值"; ...
    "manifest、alignedComparison 和 timeSeriesTables"; "完成时间、速度、路径误差、横摆力矩和轮胎利用率的可读汇总"; ...
    "与 HTML/effectiveness_analysis.txt 复用的自动结论"; "plots_presentation 目录中的展示版 PNG 清单"], ...
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
    "<p>本结果只代表当前离线 Simulink/CarSim 协同仿真上下文中的对比输出，不代表 DIL、实时硬件或真实车辆验证结论。若要扩展到 DIL 或实车，需要额外的实时性、接口、传感器、执行器和安全边界验证。</p>"
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
