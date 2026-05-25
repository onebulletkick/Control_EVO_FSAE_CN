function cfg = write_dyc_figure8_comparison_report(cfg, runResults, metrics)
%WRITE_DYC_FIGURE8_COMPARISON_REPORT 输出八字绕环 DYC 开关对比报告产物。

cfg.reportPath = localPathUnderResultsDir(cfg, 'reportPath', 'report.html');
cfg.comparisonMetricsPath = localPathUnderResultsDir(cfg, 'comparisonMetricsPath', 'comparison_metrics.csv');
cfg.runResultsPath = localPathUnderResultsDir(cfg, 'runResultsPath', 'run_results.csv');
cfg.resultMatPath = localPathUnderResultsDir(cfg, 'resultMatPath', 'comparison_result.mat');
cfg.effectivenessAnalysisPath = localPathUnderResultsDir(cfg, 'effectivenessAnalysisPath', 'effectiveness_analysis.txt');
cfg.figure8.segmentMetricsPath = localPathUnderResultsDir(cfg, {'figure8','segmentMetricsPath'}, 'figure8_segment_metrics.csv');
cfg.figure8.segmentDeltaPath = localPathUnderResultsDir(cfg, {'figure8','segmentDeltaPath'}, 'figure8_segment_delta.csv');

if ~isfolder(cfg.resultsDir)
    mkdir(cfg.resultsDir);
end

writetable(metrics.summary, cfg.comparisonMetricsPath);
writetable(metrics.perRun, cfg.runResultsPath);
writetable(metrics.figure8Segments, cfg.figure8.segmentMetricsPath);
writetable(metrics.figure8SegmentDelta, cfg.figure8.segmentDeltaPath);
[cfg, analysisArtifacts] = export_dyc_autocross_analysis_data(cfg, runResults);
plotManifest = plot_dyc_figure8_presentation_figures(cfg.resultsDir);
whereFasterAnalysis = analyze_dyc_figure8_report_results(cfg.resultsDir);
analysisLines = whereFasterAnalysis.analysisLines;
localWriteTextFile(cfg.effectivenessAnalysisPath, analysisLines);
localWriteHtmlReport(cfg, metrics, analysisArtifacts, plotManifest, analysisLines, whereFasterAnalysis);
save(cfg.resultMatPath, 'cfg', 'runResults', 'metrics', 'analysisArtifacts', 'plotManifest', 'analysisLines', 'whereFasterAnalysis');
end

function localWriteHtmlReport(cfg, metrics, analysisArtifacts, plotManifest, analysisLines, whereFasterAnalysis)
html = [
    "<!doctype html>"
    "<html lang=""zh-CN"">"
    "<head>"
    "<meta charset=""UTF-8"">"
    "<title>" + localEscape(cfg.reportTitle) + "</title>"
    "<style>"
    "body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI','Microsoft YaHei',sans-serif;line-height:1.6;margin:0;background:#f6f7f9;color:#1f2933;}"
    "main{max-width:1120px;margin:0 auto;padding:32px 24px 48px;}"
    "h1{font-size:30px;margin:0 0 20px;}h2{font-size:20px;margin:28px 0 12px;}h3{font-size:16px;margin:18px 0 8px;}"
    ".card{background:#fff;border:1px solid #d8dee8;border-radius:8px;padding:18px 20px;margin:16px 0;box-shadow:0 1px 2px rgba(15,23,42,.05);}"
    ".ok{color:#0f766e;font-weight:700;}.warn{color:#b45309;font-weight:700;}"
    "table{width:100%;border-collapse:collapse;background:#fff;font-size:14px;}th,td{border:1px solid #d8dee8;padding:8px 10px;text-align:left;vertical-align:top;}th{background:#e8edf3;}"
    "figure{margin:16px 0;padding:12px;background:#fff;border:1px solid #d8dee8;border-radius:8px;}figcaption{font-weight:600;margin-bottom:8px;}img{max-width:100%;height:auto;border:1px solid #e5e7eb;background:#fff;}"
    "</style>"
    "</head>"
    "<body><main>"
    "<h1>" + localEscape(cfg.reportTitle) + "</h1>"
    localOverviewHtml(cfg)
    localConclusionHtml(metrics)
    localReportContentsHtml()
    localWhereFasterHtml(whereFasterAnalysis)
    "<h2>关键指标</h2>"
    localTableToHtml(localSelectedMetricsTable(metrics.summary), {'metric','dycOffValue','dycOnValue','delta','unit'})
    "<h2>八字绕环左右转/换向分段指标</h2>"
    "<section class=""card"">"
    "<p>保留左转、右转和换向过渡区的分段差值，便于定位收益来源。</p>"
    localTableToHtml(metrics.figure8SegmentDelta, metrics.figure8SegmentDelta.Properties.VariableNames)
    "</section>"
    localPlotHtml(plotManifest)
    "<h2>输出文件</h2>"
    "<section class=""card"">"
    "<p>报告、表格、MAT 数据包和图表均保存在本次结果目录下。</p>"
    localArtifactHtml(analysisArtifacts, cfg)
    "</section>"
    "<h2>运行证据</h2>"
    localTableToHtml(metrics.perRun, {'caseId','repeatIndex','status','lapTime_s','stopReason','failureReason','lastRunLogPath','lastRunEndPath'})
    "<h2>验证边界</h2>"
    "<section class=""card""><p>本报告仅代表当前 CarSim/Simulink 离线仿真结果，不代表 DIL、实时硬件或实车验证。</p></section>"
    "</main></body></html>"
];
localWriteTextFile(cfg.reportPath, html);
end

function html = localWhereFasterHtml(whereFasterAnalysis)
html = [
    "<h2>快在哪里</h2>"
    "<section class=""card"">"
    "<p>delta 均为 dyc_on - dyc_off。</p>"
    localTableToHtml(whereFasterAnalysis.whereFasterTable, ...
        {'metric','dycOffValue','dycOnValue','delta','unit','interpretation'})
    "</section>"
];
end

function html = localOverviewHtml(cfg)
html = [
    "<h2>报告概览</h2>"
    "<section class=""card"">"
    "<div>工况：" + localEscape(localNestedField(cfg, {'scenario','displayName'})) + "</div>"
    "<div>dyc_off：Kp/Ki/Kd = 0/0/0</div>"
    "<div>dyc_on：Kp/Ki/Kd = 6000/200/0</div>"
    "<div>结果目录：" + localEscape(localField(cfg, 'resultsDir')) + "</div>"
    "</section>"
];
end

function html = localConclusionHtml(metrics)
comparison = metrics.comparison;
evidence = metrics.figure8Evidence;
html = [
    "<h2>核心结论</h2>"
    "<section class=""card"">"
];
if string(comparison.status) == "valid"
    html = [
        html
        "<p>完成时间差值 dyc_on - dyc_off = " + localFormatSigned(comparison.lapTimeDelta_s, " s") + "（" + localFormatSigned(comparison.lapTimeDelta_pct, " %") + "）。</p>"
        "<p class=""ok"">" + localEscape(evidence.stabilityConclusion) + "</p>"
    ];
else
    html = [
        html
        "<p class=""warn"">当前数据不输出完成时间有效性结论。原因：" + localEscape(comparison.failureReason) + "</p>"
        "<p class=""warn"">" + localEscape(evidence.failureReason) + "</p>"
    ];
end
html = [html; "</section>"];
end

function html = localReportContentsHtml()
items = [
    "完成时间与圈速差"
    "整体速度、路径误差和轮胎利用率"
    "左转、右转、换向过渡区分段差值"
    "横摆力矩介入与驱动矩分配"
    "展示图与导出文件索引"
];
html = [
    "<h2>报告提供的信息</h2>"
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
    struct('label', "轮胎峰值利用率", 'field', "tireUtilPeak", 'unit', "")
    struct('label', "横摆力矩峰值", 'field', "mzPeakAbs_Nm", 'unit', "Nm")
    struct('label', "控制介入占比", 'field', "interventionRatio", 'unit', "")
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

function lines = localAnalysisLines(cfg, metrics, analysisArtifacts)
comparison = metrics.comparison;
evidence = metrics.figure8Evidence;
lines = [
    "八字绕环 DYC 有效性分析"
    ""
    "1. 双主线结论"
    "完成时间：dyc_on - dyc_off = " + localFormatSigned(comparison.lapTimeDelta_s, " s") + "（" + localFormatSigned(comparison.lapTimeDelta_pct, " %") + "）。"
    "稳定性证据链：" + localStringValue(evidence.stabilityConclusion)
    ""
    "2. 左右转/换向分段"
];
if string(evidence.status) == "valid"
    lines = [
        lines
        "分段来源：" + localStringValue(evidence.segmentSource)
        "左转、右转、换向过渡区分别统计路径误差、速度保持、轮胎利用率、横摆力矩和驱动矩离散度。"
    ];
else
    lines = [
        lines
        "分段证据不可用：" + localStringValue(evidence.failureReason)
    ];
end

lines = [
    lines
    ""
    "3. 本地数据文件"
    "- 指标汇总表：" + localStringValue(localField(cfg, 'comparisonMetricsPath'))
    "- 单次运行表：" + localStringValue(localField(cfg, 'runResultsPath'))
    "- 分段指标表：" + localStringValue(cfg.figure8.segmentMetricsPath)
    "- 分段差值表：" + localStringValue(cfg.figure8.segmentDeltaPath)
    "- 对齐后的 dyc_on - dyc_off 时序差值：" + localArtifactField(analysisArtifacts, 'alignedComparisonRelativePath')
    "- MAT 数据包：" + localArtifactField(analysisArtifacts, 'analysisDataMatRelativePath')
    ""
    "4. 验证边界"
    "本结论来自当前八字绕环 CarSim/Simulink 联合仿真，不代表 DIL、实时硬件或实车验证。"
];
end

function html = localAnalysisHtml(analysisLines)
html = [
    "<h2>详细有效性分析</h2>"
    "<section class=""card"">"
    "<p>同一份文字分析已写入 effectiveness_analysis.txt。</p>"
];
for idx = 1:numel(analysisLines)
    line = strtrim(string(analysisLines(idx)));
    if strlength(line) == 0 || line == "八字绕环 DYC 有效性分析"
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

function html = localPlotHtml(plotManifest)
html = [
    "<h2>展示图</h2>"
    "<section class=""card"">"
];
for idx = 1:height(plotManifest)
    if ~plotManifest.available(idx)
        continue;
    end
    html = [
        html
        "<figure>"
        "<figcaption>" + localEscape(plotManifest.title(idx)) + "</figcaption>"
        "<img src=""" + localEscape(plotManifest.relativePath(idx)) + """ alt=""" + localEscape(plotManifest.title(idx)) + """>"
        "</figure>"
    ]; %#ok<AGROW>
end
html = [html; "</section>"];
end

function html = localArtifactHtml(analysisArtifacts, cfg)
tbl = table( ...
    ["HTML 报告"; "指标汇总"; "单次运行"; "信号清单"; "对齐对比数据"; "MAT 数据包"; "八字分段指标"; "八字分段差值"; "快在哪里分析表"; "快在哪里文字分析"; "展示图清单"], ...
    ["report.html"; "comparison_metrics.csv"; "run_results.csv"; ...
    localArtifactField(analysisArtifacts, 'manifestRelativePath'); ...
    localArtifactField(analysisArtifacts, 'alignedComparisonRelativePath'); ...
    localArtifactField(analysisArtifacts, 'analysisDataMatRelativePath'); ...
    "figure8_segment_metrics.csv"; "figure8_segment_delta.csv"; ...
    "figure8_where_faster.csv"; "figure8_where_faster_analysis.txt"; ...
    "plots_presentation/presentation_plot_manifest.csv"], ...
    ["当前仪表盘页面"; "dyc_off/dyc_on 核心指标"; "每次运行状态和停止原因"; ...
    "每个 case/repeat 的导出状态、来源和行数"; ...
    "按 dyc_off 时间轴插值对齐后的时序差值"; ...
    "manifest、alignedComparison、timeSeriesTables 和 artifacts"; ...
    "左转、右转、换向过渡区的分段统计"; ...
    "dyc_on - dyc_off 的分段差值"; ...
    "完成时间、速度、路径误差、轮胎利用率和横摆力矩的可读汇总"; ...
    "精简文字结论"; ...
    "plots_presentation 目录中的展示版 PNG 清单"], ...
    'VariableNames', {'artifact','relativePath','description'});
html = localTableToHtml(tbl, tbl.Properties.VariableNames);
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
        html = html + "<td>" + localEscape(localDisplayValue(tbl.(columns{colIdx})(rowIdx))) + "</td>";
    end
    html = html + "</tr>";
end
html = html + "</tbody></table>";
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
end
end

function text = localFormatSigned(value, suffix)
value = double(value);
if isempty(value) || ~isfinite(value(1))
    text = "unavailable";
else
    text = sprintf('%+.4f%s', value(1), suffix);
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

function text = localArtifactField(artifacts, fieldName)
text = "unavailable";
if isstruct(artifacts) && isfield(artifacts, fieldName)
    candidate = localStringValue(artifacts.(fieldName));
    if strlength(candidate) > 0
        text = candidate;
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
    error('dyc:figure8Comparison:ReportWriteFailed', 'Unable to write report: %s', char(string(pathValue)));
end
cleanup = onCleanup(@() fclose(fid));
for idx = 1:numel(lines)
    fprintf(fid, '%s\n', char(lines(idx)));
end
end

function pathOut = localPathUnderResultsDir(cfg, fieldName, fileName)
if iscell(fieldName)
    candidatePath = localNestedField(cfg, fieldName);
    fieldLabel = strjoin(fieldName, '.');
else
    candidatePath = localField(cfg, fieldName);
    fieldLabel = fieldName;
end
if strlength(string(candidatePath)) == 0
    candidatePath = fullfile(cfg.resultsDir, fileName);
end

if localCanonicalParentIsUnderFolder(candidatePath, cfg.resultsDir)
    pathOut = candidatePath;
elseif localRawPathClaimsUnderFolder(candidatePath, cfg.resultsDir)
    error('dyc:figure8Comparison:ArtifactPathOutsideResultsDir', ...
        'Figure-8 comparison artifact path for %s escapes resultsDir after canonical normalization: %s', ...
        char(string(fieldLabel)), char(string(candidatePath)));
else
    pathOut = fullfile(cfg.resultsDir, fileName);
end
end

function value = localNestedField(s, names)
value = s;
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
