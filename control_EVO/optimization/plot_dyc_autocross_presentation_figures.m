function manifest = plot_dyc_autocross_presentation_figures(resultsDir)
%PLOT_DYC_AUTOCROSS_PRESENTATION_FIGURES 从已导出的 Autocross 数据重绘展示版图表。

if isstruct(resultsDir)
    resultsDir = resultsDir.resultsDir;
end
resultsDir = char(string(resultsDir));

alignedPath = fullfile(resultsDir, 'signal_data', 'aligned_dyc_comparison.csv');
metricsPath = fullfile(resultsDir, 'comparison_metrics.csv');
if ~isfile(alignedPath)
    error('dyc:autocrossPresentationPlots:MissingAlignedComparison', ...
        'Missing aligned comparison data: %s', alignedPath);
end
if ~isfile(metricsPath)
    error('dyc:autocrossPresentationPlots:MissingMetrics', ...
        'Missing comparison metrics data: %s', metricsPath);
end

aligned = readtable(alignedPath, 'TextType', 'string');
summary = readtable(metricsPath, 'TextType', 'string');

plotDir = fullfile(resultsDir, 'plots_presentation');
if ~isfolder(plotDir)
    mkdir(plotDir);
end

palette = localPalette();
specs = [
    struct('fileName', "key_metrics_summary.png", ...
        'title', "关键指标总览", ...
        'plotFcn', @() localPlotKeyMetrics(summary, palette))
    struct('fileName', "speed_lateral_error_overview.png", ...
        'title', "速度与路径误差", ...
        'plotFcn', @() localPlotSpeedAndError(aligned, palette))
    struct('fileName', "yaw_moment_torque_distribution.png", ...
        'title', "横摆力矩与驱动矩分配", ...
        'plotFcn', @() localPlotYawMomentAndTorque(aligned, palette))
    struct('fileName', "tire_utilization_comparison.png", ...
        'title', "轮胎利用率", ...
        'plotFcn', @() localPlotTireUtilization(aligned, palette))
    struct('fileName', "throttle_accel_overview.png", ...
        'title', "油门与加速度", ...
        'plotFcn', @() localPlotThrottleAndAccel(aligned, palette))
    struct('fileName', "effectiveness_evidence_chain.png", ...
        'title', "DYC 有效性证据链", ...
        'plotFcn', @() localPlotEvidenceChain(summary, palette))
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
palette.bg = [1 1 1];
end

function fig = localBaseFigure(titleText)
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1480 880]);
tl = tiledlayout(fig, 'flow', 'Padding', 'compact', 'TileSpacing', 'compact');
title(tl, titleText, 'FontName', 'Microsoft YaHei', 'FontSize', 20, 'FontWeight', 'bold');
end

function fig = localPlotKeyMetrics(summary, palette)
fig = localBaseFigure('Autocross DYC 关键指标总览');

ax = nexttile;
localBarPair(ax, summary, 'lapTime_s', '圈速', 's', palette, true);

ax = nexttile;
localBarPair(ax, summary, 'lateralErrorRmse', '横向路径误差 RMSE', 'm', palette, true);

ax = nexttile;
localBarPair(ax, summary, 'meanSpeed_mps', '平均速度', 'm/s', palette, false);

ax = nexttile;
localBarPair(ax, summary, 'tireUtilPeak', '轮胎利用率峰值', '', palette, true);

ax = nexttile;
localBarPair(ax, summary, 'mzPeakAbs_Nm', '横摆力矩峰值', 'Nm', palette, false);

ax = nexttile;
localBarPair(ax, summary, 'interventionRatio', '控制介入占比', '', palette, false);
end

function fig = localPlotSpeedAndError(aligned, palette)
fig = localBaseFigure('速度保持与路径误差对比');
t = localColumn(aligned, 'time_s');

ax = nexttile([1 2]);
localLinePair(ax, t, localColumn(aligned, 'speed_mps_dyc_off'), ...
    localColumn(aligned, 'speed_mps_dyc_on'), '速度对比', 'speed (m/s)', palette);

ax = nexttile([1 2]);
localLinePair(ax, t, abs(localColumn(aligned, 'lateralError_m_dyc_off')), ...
    abs(localColumn(aligned, 'lateralError_m_dyc_on')), '横向路径误差绝对值', '|lateral error| (m)', palette);

ax = nexttile([1 2]);
localFilledDelta(ax, t, localColumn(aligned, 'speed_mps_delta'), ...
    '速度差值 dyc\_on - dyc\_off', 'delta speed (m/s)', palette);
end

function fig = localPlotYawMomentAndTorque(aligned, palette)
fig = localBaseFigure('横摆力矩与四轮驱动矩分配');
t = localColumn(aligned, 'time_s');

ax = nexttile([1 2]);
localLinePair(ax, t, localColumn(aligned, 'mz_Nm_dyc_off'), ...
    localColumn(aligned, 'mz_Nm_dyc_on'), '横摆力矩输出', 'Mz (Nm)', palette);

ax = nexttile([1 2]);
localLinePair(ax, t, localColumn(aligned, 'wheelTorqueSpread_Nm_dyc_off'), ...
    localColumn(aligned, 'wheelTorqueSpread_Nm_dyc_on'), '四轮驱动矩离散度', 'max(T)-min(T) (Nm)', palette);

ax = nexttile([1 2]);
localLinePair(ax, t, localColumn(aligned, 'leftRightDriveTorqueDelta_Nm_dyc_off'), ...
    localColumn(aligned, 'leftRightDriveTorqueDelta_Nm_dyc_on'), '左右驱动矩差', 'right-left torque (Nm)', palette);
end

function fig = localPlotTireUtilization(aligned, palette)
fig = localBaseFigure('轮胎利用率对比');
t = localColumn(aligned, 'time_s');
off = localColumn(aligned, 'tireUtilMax_dyc_off');
on = localColumn(aligned, 'tireUtilMax_dyc_on');

ax = nexttile([1 2]);
localLinePair(ax, t, off, on, '最大轮胎合力利用率时序', 'max tire force / Fz', palette);
localThresholdLine(ax, 1.0, 'reference 1.0');

ax = nexttile;
localHistogramPair(ax, off, on, '利用率分布', 'max tire force / Fz', palette);

ax = nexttile;
localFilledDelta(ax, t, localColumn(aligned, 'tireUtilMax_delta'), ...
    '利用率差值 dyc\_on - dyc\_off', 'delta utilization', palette);
end

function fig = localPlotThrottleAndAccel(aligned, palette)
fig = localBaseFigure('油门与加速度对比');
t = localColumn(aligned, 'time_s');

ax = nexttile([1 2]);
localLinePair(ax, t, localColumn(aligned, 'throttle_dyc_off'), ...
    localColumn(aligned, 'throttle_dyc_on'), '油门开度', 'normalized throttle', palette);

ax = nexttile([1 2]);
localLinePair(ax, t, localColumn(aligned, 'ax_mps2_dyc_off'), ...
    localColumn(aligned, 'ax_mps2_dyc_on'), '纵向加速度', 'Ax (m/s^2)', palette);

ax = nexttile([1 2]);
localLinePair(ax, t, localColumn(aligned, 'ay_mps2_dyc_off'), ...
    localColumn(aligned, 'ay_mps2_dyc_on'), '横向加速度', 'Ay (m/s^2)', palette);
end

function fig = localPlotEvidenceChain(summary, palette)
fig = localBaseFigure('DYC 有效性证据链');
ax = nexttile([1 2]);

items = [
    "圈速缩短"
    "路径误差降低"
    "平均速度提升"
    "轮胎峰值利用率降低"
    "横摆力矩输出增加"
    "控制介入占比增加"
];
rawValues = [
    localCaseValue(summary, 'dyc_off', 'lapTime_s') - localCaseValue(summary, 'dyc_on', 'lapTime_s')
    localCaseValue(summary, 'dyc_off', 'lateralErrorRmse') - localCaseValue(summary, 'dyc_on', 'lateralErrorRmse')
    localCaseValue(summary, 'dyc_on', 'meanSpeed_mps') - localCaseValue(summary, 'dyc_off', 'meanSpeed_mps')
    localCaseValue(summary, 'dyc_off', 'tireUtilPeak') - localCaseValue(summary, 'dyc_on', 'tireUtilPeak')
    localCaseValue(summary, 'dyc_on', 'mzPeakAbs_Nm') - localCaseValue(summary, 'dyc_off', 'mzPeakAbs_Nm')
    localCaseValue(summary, 'dyc_on', 'interventionRatio') - localCaseValue(summary, 'dyc_off', 'interventionRatio')
];
units = ["s", "m", "m/s", "", "Nm", ""];

score = rawValues;
finiteScore = score(isfinite(score));
if isempty(finiteScore) || max(abs(finiteScore)) == 0
    normalized = zeros(size(score));
else
    normalized = score ./ max(abs(finiteScore));
end

y = 1:numel(items);
barh(ax, y, normalized, 0.72, 'FaceColor', palette.delta, 'EdgeColor', 'none');
ax.YTick = y;
ax.YTickLabel = items;
ax.YDir = 'reverse';
xlabel(ax, 'normalized evidence direction');
title(ax, '正方向代表支持 DYC 有效的证据');
localStyleAxes(ax, palette);
grid(ax, 'on');
for idx = 1:numel(items)
    label = localFormatSigned(rawValues(idx), units(idx));
    text(ax, normalized(idx) + 0.04 * signOrOne(normalized(idx)), idx, label, ...
        'FontName', 'Microsoft YaHei', 'FontSize', 11, 'Color', palette.text, ...
        'VerticalAlignment', 'middle');
end
xlim(ax, [-1.15 1.15]);

ax = nexttile([1 2]);
axis(ax, 'off');
text(ax, 0.02, 0.78, '读图方式', 'FontName', 'Microsoft YaHei', ...
    'FontSize', 16, 'FontWeight', 'bold', 'Color', palette.text);
text(ax, 0.02, 0.58, '这张图把不同单位的指标统一成方向性证据：圈速越短、路径误差越小、平均速度越高、轮胎峰值利用率越低，越支持 DYC 在 Autocross 中改善车辆姿态和弯道效率。', ...
    'FontName', 'Microsoft YaHei', 'FontSize', 12, 'Color', palette.text, ...
    'Units', 'normalized');
text(ax, 0.02, 0.34, '横摆力矩和控制介入占比不是单独证明圈速收益，而是证明控制器确实发生了有效介入，需要与路径误差、速度和轮胎利用率一起解释。', ...
    'FontName', 'Microsoft YaHei', 'FontSize', 12, 'Color', palette.text, ...
    'Units', 'normalized');
end

function localBarPair(ax, summary, metricName, titleText, unitText, palette, lowerIsBetter)
off = localCaseValue(summary, 'dyc_off', metricName);
on = localCaseValue(summary, 'dyc_on', metricName);
values = [off, on];
bar(ax, 1:2, values, 0.62, 'FaceColor', 'flat', 'EdgeColor', 'none');
ax.Children.CData = [palette.off; palette.on];
ax.XTick = 1:2;
ax.XTickLabel = {'dyc off','dyc on'};
ylabel(ax, unitText);
title(ax, titleText);
localStyleAxes(ax, palette);

delta = on - off;
if lowerIsBetter
    note = "delta " + localFormatSigned(delta, unitText);
else
    note = "delta " + localFormatSigned(delta, unitText);
end
text(ax, 1.5, max(values, [], 'omitnan') * 1.05, note, ...
    'HorizontalAlignment', 'center', 'FontName', 'Microsoft YaHei', ...
    'FontWeight', 'bold', 'Color', palette.text);
ylim(ax, localYLim(values));
end

function localLinePair(ax, t, off, on, titleText, yLabelText, palette)
plot(ax, t, off, 'Color', palette.off, 'LineWidth', 1.8, 'DisplayName', 'dyc off');
hold(ax, 'on');
plot(ax, t, on, 'Color', palette.on, 'LineWidth', 1.8, 'DisplayName', 'dyc on');
title(ax, titleText);
xlabel(ax, 'time (s)');
ylabel(ax, yLabelText);
legend(ax, 'Location', 'best');
localStyleAxes(ax, palette);
end

function localFilledDelta(ax, t, delta, titleText, yLabelText, palette)
area(ax, t, delta, 'FaceColor', palette.delta, 'FaceAlpha', 0.18, ...
    'EdgeColor', palette.delta, 'LineWidth', 1.5);
yline(ax, 0, '-', 'Color', palette.muted, 'LineWidth', 1.0);
title(ax, titleText);
xlabel(ax, 'time (s)');
ylabel(ax, yLabelText);
localStyleAxes(ax, palette);
end

function localHistogramPair(ax, off, on, titleText, xLabelText, palette)
histogram(ax, off(isfinite(off)), 28, 'FaceColor', palette.off, ...
    'FaceAlpha', 0.45, 'EdgeColor', 'none', 'DisplayName', 'dyc off');
hold(ax, 'on');
histogram(ax, on(isfinite(on)), 28, 'FaceColor', palette.on, ...
    'FaceAlpha', 0.45, 'EdgeColor', 'none', 'DisplayName', 'dyc on');
title(ax, titleText);
xlabel(ax, xLabelText);
ylabel(ax, 'count');
legend(ax, 'Location', 'best');
localStyleAxes(ax, palette);
end

function localThresholdLine(ax, yValue, labelText)
yline(ax, yValue, '--', labelText, 'LineWidth', 1.1, ...
    'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'bottom');
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
if ismember(name, tbl.Properties.VariableNames)
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

function text = localFormatSigned(value, unitText)
if ~isfinite(value)
    text = "unavailable";
    return;
end
if strlength(string(unitText)) > 0
    text = string(sprintf('%+.4g %s', value, char(unitText)));
else
    text = string(sprintf('%+.4g', value));
end
end

function limits = localYLim(values)
values = values(isfinite(values));
if isempty(values)
    limits = [0 1];
    return;
end
low = min(0, min(values));
high = max(values);
if high == low
    high = low + 1;
end
padding = 0.18 * (high - low);
limits = [low, high + padding];
end

function value = signOrOne(x)
if x < 0
    value = -1;
else
    value = 1;
end
end
