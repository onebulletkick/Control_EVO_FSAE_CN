function manifest = plot_dyc_figure8_presentation_figures(resultsDir)
%PLOT_DYC_FIGURE8_PRESENTATION_FIGURES 从八字绕环导出数据重绘展示版图表。

resultsDir = char(string(resultsDir));
alignedPath = fullfile(resultsDir, 'signal_data', 'aligned_dyc_comparison.csv');
metricsPath = fullfile(resultsDir, 'comparison_metrics.csv');
segmentDeltaPath = fullfile(resultsDir, 'figure8_segment_delta.csv');
if ~isfile(alignedPath)
    error('dyc:figure8PresentationPlots:MissingAlignedComparison', ...
        'Missing aligned comparison data: %s', alignedPath);
end
if ~isfile(metricsPath)
    error('dyc:figure8PresentationPlots:MissingMetrics', ...
        'Missing comparison metrics data: %s', metricsPath);
end

aligned = readtable(alignedPath, 'TextType', 'string');
summary = readtable(metricsPath, 'TextType', 'string');
if isfile(segmentDeltaPath)
    segmentDelta = readtable(segmentDeltaPath, 'TextType', 'string');
else
    segmentDelta = table();
end

plotDir = fullfile(resultsDir, 'plots_presentation');
if ~isfolder(plotDir)
    mkdir(plotDir);
end

palette = localPalette();
specs = [
    struct('fileName', "key_metrics_summary.png", 'title', "关键指标总览", ...
        'plotFcn', @() localPlotKeyMetrics(summary, palette))
    struct('fileName', "figure8_left_right_segment_map.png", 'title', "左/右/换向分段收益", ...
        'plotFcn', @() localPlotSegmentMap(segmentDelta, palette))
    struct('fileName', "speed_lateral_error_overview.png", 'title', "速度与路径误差", ...
        'plotFcn', @() localPlotSpeedAndError(aligned, palette))
    struct('fileName', "yaw_moment_transition_response.png", 'title', "横摆力矩与换向响应", ...
        'plotFcn', @() localPlotYawMomentTransition(aligned, palette))
    struct('fileName', "tire_utilization_left_right.png", 'title', "轮胎利用率左右转对比", ...
        'plotFcn', @() localPlotTireUtilization(aligned, segmentDelta, palette))
    struct('fileName', "effectiveness_evidence_chain.png", 'title', "DYC 有效性证据链", ...
        'plotFcn', @() localPlotEvidenceChain(summary, segmentDelta, palette))
];

manifest = table('Size', [numel(specs), 5], ...
    'VariableTypes', {'string','string','string','string','logical'}, ...
    'VariableNames', {'fileName','title','relativePath','fullPath','available'});

for idx = 1:numel(specs)
    spec = specs(idx);
    plotPath = fullfile(plotDir, char(spec.fileName));
    fig = spec.plotFcn();
    cleanup = onCleanup(@() close(fig));
    localSaveFigure(fig, plotPath);
    clear cleanup;
    manifest.fileName(idx) = spec.fileName;
    manifest.title(idx) = spec.title;
    manifest.relativePath(idx) = "plots_presentation/" + spec.fileName;
    manifest.fullPath(idx) = string(plotPath);
    manifest.available(idx) = isfile(plotPath);
end

writetable(manifest, fullfile(plotDir, 'presentation_plot_manifest.csv'));
end

function palette = localPalette()
palette = struct();
palette.off = [35 79 124] / 255;
palette.on = [190 59 49] / 255;
palette.delta = [27 129 114] / 255;
palette.gold = [217 150 58] / 255;
palette.grid = [220 226 235] / 255;
palette.text = [31 41 55] / 255;
palette.muted = [107 114 128] / 255;
end

function fig = localBaseFigure(titleText)
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1480 880]);
tl = tiledlayout(fig, 'flow', 'Padding', 'compact', 'TileSpacing', 'compact');
title(tl, titleText, 'FontName', 'Microsoft YaHei', 'FontSize', 20, 'FontWeight', 'bold');
end

function fig = localPlotKeyMetrics(summary, palette)
fig = localBaseFigure('八字绕环 DYC 关键指标总览');
metrics = {
    'lapTime_s', '完成时间', 's', true;
    'lateralErrorRmse', '路径误差 RMSE', 'm', true;
    'meanSpeed_mps', '平均速度', 'm/s', false;
    'tireUtilPeak', '轮胎利用率峰值', '', true;
    'mzPeakAbs_Nm', '横摆力矩峰值', 'Nm', false;
    'interventionRatio', '控制介入占比', '', false};
for idx = 1:size(metrics, 1)
    ax = nexttile;
    localBarPair(ax, summary, metrics{idx, 1}, metrics{idx, 2}, metrics{idx, 3}, palette);
end
end

function fig = localPlotSegmentMap(segmentDelta, palette)
fig = localBaseFigure('左转 / 右转 / 换向过渡分段收益');
if isempty(segmentDelta) || height(segmentDelta) == 0
    ax = nexttile([1 2]);
    axis(ax, 'off');
    text(ax, 0.05, 0.55, '分段证据不可用', 'FontName', 'Microsoft YaHei', ...
        'FontSize', 18, 'FontWeight', 'bold', 'Color', palette.text);
    return;
end

segmentLabels = localSegmentLabels(segmentDelta.segmentType);
fields = {'lateralErrorRmseDelta','tireUtilPeakDelta','meanSpeedDelta_mps'};
titles = {'路径误差 RMSE 差值', '轮胎利用率峰值差值', '平均速度差值'};
for idx = 1:numel(fields)
    ax = nexttile;
    values = localColumn(segmentDelta, fields{idx});
    bar(ax, values, 0.66, 'FaceColor', palette.delta, 'EdgeColor', 'none');
    yline(ax, 0, '-', 'Color', palette.muted);
    ax.XTick = 1:numel(segmentLabels);
    ax.XTickLabel = segmentLabels;
    title(ax, titles{idx});
    ylabel(ax, 'dyc on - dyc off');
    localStyleAxes(ax, palette);
end
end

function fig = localPlotSpeedAndError(aligned, palette)
fig = localBaseFigure('八字绕环速度保持与路径误差');
t = localColumn(aligned, 'time_s');
ax = nexttile([1 2]);
localLinePair(ax, t, localColumn(aligned, 'speed_mps_dyc_off'), ...
    localColumn(aligned, 'speed_mps_dyc_on'), '速度对比', 'speed (m/s)', palette);
ax = nexttile([1 2]);
localLinePair(ax, t, abs(localColumn(aligned, 'lateralError_m_dyc_off')), ...
    abs(localColumn(aligned, 'lateralError_m_dyc_on')), '路径误差绝对值', '|lateral error| (m)', palette);
ax = nexttile([1 2]);
localFilledDelta(ax, t, localColumn(aligned, 'speed_mps_delta'), ...
    '速度差值 dyc\_on - dyc\_off', 'delta speed (m/s)', palette);
end

function fig = localPlotYawMomentTransition(aligned, palette)
fig = localBaseFigure('横摆力矩与连续换向响应');
t = localColumn(aligned, 'time_s');
ax = nexttile([1 2]);
localLinePair(ax, t, localColumn(aligned, 'ay_mps2_dyc_off'), ...
    localColumn(aligned, 'ay_mps2_dyc_on'), '横向加速度换向', 'Ay (m/s^2)', palette);
ax = nexttile([1 2]);
localLinePair(ax, t, localColumn(aligned, 'mz_Nm_dyc_off'), ...
    localColumn(aligned, 'mz_Nm_dyc_on'), '横摆力矩输出', 'Mz (Nm)', palette);
ax = nexttile([1 2]);
localLinePair(ax, t, localColumn(aligned, 'wheelTorqueSpread_Nm_dyc_off'), ...
    localColumn(aligned, 'wheelTorqueSpread_Nm_dyc_on'), '四轮驱动矩离散度', 'max(T)-min(T) (Nm)', palette);
end

function fig = localPlotTireUtilization(aligned, segmentDelta, palette)
fig = localBaseFigure('八字绕环轮胎利用率');
t = localColumn(aligned, 'time_s');
ax = nexttile([1 2]);
localLinePair(ax, t, localColumn(aligned, 'tireUtilMax_dyc_off'), ...
    localColumn(aligned, 'tireUtilMax_dyc_on'), '最大轮胎合力利用率时序', 'max tire force / Fz', palette);
yline(ax, 1.0, '--', 'reference 1.0', 'LabelHorizontalAlignment', 'left');

ax = nexttile([1 2]);
if ~isempty(segmentDelta) && height(segmentDelta) > 0 && ismember('tireUtilPeakDelta', segmentDelta.Properties.VariableNames)
    bar(ax, localColumn(segmentDelta, 'tireUtilPeakDelta'), 0.66, 'FaceColor', palette.gold, 'EdgeColor', 'none');
    ax.XTick = 1:height(segmentDelta);
    ax.XTickLabel = localSegmentLabels(segmentDelta.segmentType);
    yline(ax, 0, '-', 'Color', palette.muted);
    title(ax, '分段轮胎峰值利用率差值');
    ylabel(ax, 'dyc on - dyc off');
else
    axis(ax, 'off');
    text(ax, 0.05, 0.55, '分段轮胎利用率证据不可用', 'FontName', 'Microsoft YaHei', 'FontSize', 16);
end
localStyleAxes(ax, palette);
end

function fig = localPlotEvidenceChain(summary, segmentDelta, palette)
fig = localBaseFigure('八字绕环 DYC 有效性证据链');
ax = nexttile([1 2]);
items = [
    "完成时间缩短"
    "路径误差降低"
    "平均速度提升"
    "轮胎峰值利用率降低"
    "横摆力矩输出增加"
    "控制介入占比增加"
];
values = [
    localCaseValue(summary, 'dyc_off', 'lapTime_s') - localCaseValue(summary, 'dyc_on', 'lapTime_s')
    localMeanSegmentBenefit(segmentDelta, 'lateralErrorRmseDelta', -1)
    localCaseValue(summary, 'dyc_on', 'meanSpeed_mps') - localCaseValue(summary, 'dyc_off', 'meanSpeed_mps')
    localMeanSegmentBenefit(segmentDelta, 'tireUtilPeakDelta', -1)
    localCaseValue(summary, 'dyc_on', 'mzPeakAbs_Nm') - localCaseValue(summary, 'dyc_off', 'mzPeakAbs_Nm')
    localCaseValue(summary, 'dyc_on', 'interventionRatio') - localCaseValue(summary, 'dyc_off', 'interventionRatio')
];
finiteValues = values(isfinite(values));
if isempty(finiteValues) || max(abs(finiteValues)) == 0
    normalized = zeros(size(values));
else
    normalized = values ./ max(abs(finiteValues));
end
barh(ax, 1:numel(items), normalized, 0.72, 'FaceColor', palette.delta, 'EdgeColor', 'none');
ax.YTick = 1:numel(items);
ax.YTickLabel = items;
ax.YDir = 'reverse';
xlabel(ax, 'normalized evidence direction');
title(ax, '正方向代表支持 DYC 有效的证据');
xlim(ax, [-1.15 1.15]);
localStyleAxes(ax, palette);
end

function localBarPair(ax, summary, metricName, titleText, unitText, palette)
off = localCaseValue(summary, 'dyc_off', metricName);
on = localCaseValue(summary, 'dyc_on', metricName);
b = bar(ax, 1:2, [off on], 0.62, 'FaceColor', 'flat', 'EdgeColor', 'none');
b.CData = [palette.off; palette.on];
ax.XTick = 1:2;
ax.XTickLabel = {'dyc off','dyc on'};
ylabel(ax, unitText);
title(ax, titleText);
localStyleAxes(ax, palette);
end

function localLinePair(ax, t, off, on, titleText, yLabelText, palette)
plot(ax, t, off, 'Color', palette.off, 'LineWidth', 1.7, 'DisplayName', 'dyc off');
hold(ax, 'on');
plot(ax, t, on, 'Color', palette.on, 'LineWidth', 1.7, 'DisplayName', 'dyc on');
title(ax, titleText);
xlabel(ax, 'time (s)');
ylabel(ax, yLabelText);
legend(ax, 'Location', 'best');
localStyleAxes(ax, palette);
end

function localFilledDelta(ax, t, delta, titleText, yLabelText, palette)
area(ax, t, delta, 'FaceColor', palette.delta, 'FaceAlpha', 0.18, ...
    'EdgeColor', palette.delta, 'LineWidth', 1.4);
yline(ax, 0, '-', 'Color', palette.muted);
title(ax, titleText);
xlabel(ax, 'time (s)');
ylabel(ax, yLabelText);
localStyleAxes(ax, palette);
end

function localStyleAxes(ax, palette)
ax.FontName = 'Microsoft YaHei';
ax.FontSize = 11;
ax.LineWidth = 0.9;
ax.Box = 'off';
ax.XColor = palette.text;
ax.YColor = palette.text;
ax.GridColor = palette.grid;
ax.GridAlpha = 0.55;
grid(ax, 'on');
end

function localSaveFigure(fig, plotPath)
try
    exportgraphics(fig, plotPath, 'Resolution', 300);
catch
    saveas(fig, plotPath);
end
end

function values = localColumn(tbl, name)
if ~isempty(tbl) && istable(tbl) && ismember(name, tbl.Properties.VariableNames)
    values = double(tbl.(name));
    values = values(:);
else
    values = NaN(height(tbl), 1);
end
end

function value = localCaseValue(summary, caseId, metricName)
value = NaN;
if ~ismember('caseId', summary.Properties.VariableNames) || ...
        ~ismember(metricName, summary.Properties.VariableNames)
    return;
end
row = summary(summary.caseId == string(caseId), :);
if height(row) == 1
    value = double(row.(metricName)(1));
end
end

function labels = localSegmentLabels(segmentTypes)
labels = strings(size(segmentTypes));
for idx = 1:numel(segmentTypes)
    switch string(segmentTypes(idx))
        case "left"
            labels(idx) = "左转";
        case "right"
            labels(idx) = "右转";
        case "transition"
            labels(idx) = "换向过渡";
        otherwise
            labels(idx) = string(segmentTypes(idx));
    end
end
end

function value = localMeanSegmentBenefit(segmentDelta, fieldName, direction)
if isempty(segmentDelta) || ~istable(segmentDelta) || ~ismember(fieldName, segmentDelta.Properties.VariableNames)
    value = NaN;
    return;
end
raw = double(segmentDelta.(fieldName));
raw = raw(isfinite(raw));
if isempty(raw)
    value = NaN;
else
    value = direction * mean(raw);
end
end
