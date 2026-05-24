function cfg = write_dyc_autocross_comparison_report(cfg, runResults, metrics)
%WRITE_DYC_AUTOCROSS_COMPARISON_REPORT 输出 Autocross DYC 开关对比报告产物。

cfg.reportPath = localPathUnderResultsDir(cfg, 'reportPath', 'report.html');
cfg.comparisonMetricsPath = localPathUnderResultsDir(cfg, 'comparisonMetricsPath', 'comparison_metrics.csv');
cfg.runResultsPath = localPathUnderResultsDir(cfg, 'runResultsPath', 'run_results.csv');
cfg.resultMatPath = localPathUnderResultsDir(cfg, 'resultMatPath', 'comparison_result.mat');

if ~isfolder(cfg.resultsDir)
    mkdir(cfg.resultsDir);
end

writetable(metrics.summary, cfg.comparisonMetricsPath);
writetable(metrics.perRun, cfg.runResultsPath);
save(cfg.resultMatPath, 'cfg', 'runResults', 'metrics');
localWriteHtmlReport(cfg, metrics);
end

function localWriteHtmlReport(cfg, metrics)
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
    ".muted{color:#66788a;}.ok{color:#0f766e;font-weight:600;}.warn{color:#b45309;font-weight:600;}"
    "</style>"
    "</head>"
    "<body><main>"
    "<h1>DYC Autocross 有无控制对比报告</h1>"
    localExperimentSummaryHtml(cfg)
    localConclusionHtml(summary, comparison)
    "<h2>关键指标</h2>"
    localTableToHtml(summary, summary.Properties.VariableNames)
    localMechanismHtml()
    localInterventionHtml(summary)
    "<h2>运行证据</h2>"
    localTableToHtml(perRun, {'caseId','repeatIndex','status','lapTime_s','stopReason','failureReason','lastRunLogPath','lastRunEndPath'})
    localBoundaryHtml()
    "</main></body></html>"
];

localWriteTextFile(cfg.reportPath, html);
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

function html = localMechanismHtml()
html = [
    "<h2>机理解释</h2>"
    "<section class=""card"">"
    "<p>Autocross 工况的圈速不仅取决于纵向加速，还取决于车辆能否稳定跟踪目标横摆响应并减少路径偏差。DYC 通过附加横摆力矩修正 yaw tracking error，在转向和横向加速度快速变化时帮助车辆更接近期望姿态。</p>"
    "<p>本报告将横摆跟踪、横向路径偏差、横向响应、速度保持和控制介入强度放在一起观察：如果 DYC 开启后 yaw-rate error 与 lateral error 下降，同时平均速度或最小速度没有明显恶化，并且控制介入信号存在合理幅值，则可以解释为控制器在弯中姿态和出弯速度之间取得了更好的折中。</p>"
    "</section>"
];
end

function html = localInterventionHtml(summary)
mzFields = {'mzRms_Nm','mzPeakAbs_Nm','mzAbsIntegral_Nms','interventionRatio'};
hasMz = false;
for idx = 1:numel(mzFields)
    if any(isfinite(summary.(mzFields{idx})))
        hasMz = true;
        break;
    end
end

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
    "<p>下表列出横摆力矩 RMS、峰值、绝对积分和介入占比。数值越高通常表示 DYC 对车辆横摆运动施加了更强或更频繁的修正，但仍需结合 yaw tracking、路径偏差和圈速变化共同判断。</p>"
    localTableToHtml(summary, [{'caseId','displayName'}, mzFields])
    "</section>"
];
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
row = summary(summary.caseId == string(caseId), :);
if height(row) ~= 1 || ~ismember(fieldName, summary.Properties.VariableNames)
    value = NaN;
else
    value = row.(fieldName);
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
