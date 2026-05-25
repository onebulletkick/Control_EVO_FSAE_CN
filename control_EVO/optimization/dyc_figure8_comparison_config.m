function cfg = dyc_figure8_comparison_config(varargin)
%DYC_FIGURE8_COMPARISON_CONFIG 八字绕环 DYC 开关对比实验配置。

cfg = dyc_autocross_comparison_config();
cfg.scenario = struct('id', "figure8", 'displayName', "八字绕环");
cfg.reportTitle = "DYC 八字绕环有无控制对比报告";
cfg.resultsDir = fullfile(cfg.resultsRoot, [cfg.runTimestamp '_dyc_figure8_comparison']);
cfg.figure8 = struct();
cfg.figure8.transitionAyAbsThreshold_mps2 = 0.35;
cfg.figure8.transitionYawRateAbsThreshold_radps = 0.03;
cfg.figure8.minSegmentSampleCount = 2;
cfg.figure8.segmentMetricsPath = fullfile(cfg.resultsDir, 'figure8_segment_metrics.csv');
cfg.figure8.segmentDeltaPath = fullfile(cfg.resultsDir, 'figure8_segment_delta.csv');
cfg.figure8.presentationPlotManifestPath = fullfile(cfg.resultsDir, 'plots_presentation', 'presentation_plot_manifest.csv');
cfg = localRefreshArtifactPaths(cfg, localExplicitArtifactMap(struct()));

if nargin > 0
    overrides = varargin{1};
    hasResultsRootOverride = isfield(overrides, 'resultsRoot');
    hasRunTimestampOverride = isfield(overrides, 'runTimestamp');
    hasResultsDirOverride = isfield(overrides, 'resultsDir');
    explicitArtifacts = localExplicitArtifactMap(overrides);

    cfg = localMergeStruct(cfg, overrides);

    if (hasResultsRootOverride || hasRunTimestampOverride) && ~hasResultsDirOverride
        cfg.resultsDir = fullfile(cfg.resultsRoot, [cfg.runTimestamp '_dyc_figure8_comparison']);
    end
    if hasResultsRootOverride || hasRunTimestampOverride || hasResultsDirOverride
        cfg = localRefreshArtifactPaths(cfg, explicitArtifacts);
    end
end
end

function out = localMergeStruct(base, overrides)
out = base;
names = fieldnames(overrides);
for idx = 1:numel(names)
    name = names{idx};
    if isstruct(overrides.(name)) && isscalar(overrides.(name)) && ...
            isfield(out, name) && isstruct(out.(name)) && isscalar(out.(name))
        out.(name) = localMergeStruct(out.(name), overrides.(name));
    else
        out.(name) = overrides.(name);
    end
end
end

function explicitArtifacts = localExplicitArtifactMap(overrides)
artifactFields = {'reportPath', 'comparisonMetricsPath', 'runResultsPath', ...
    'resultMatPath', 'figure8SegmentMetricsPath', 'figure8SegmentDeltaPath'};
explicitArtifacts = struct();
for idx = 1:numel(artifactFields)
    fieldName = artifactFields{idx};
    explicitArtifacts.(fieldName) = isfield(overrides, fieldName);
end
end

function cfg = localRefreshArtifactPaths(cfg, explicitArtifacts)
if ~explicitArtifacts.reportPath
    cfg.reportPath = fullfile(cfg.resultsDir, 'report.html');
end
if ~explicitArtifacts.comparisonMetricsPath
    cfg.comparisonMetricsPath = fullfile(cfg.resultsDir, 'comparison_metrics.csv');
end
if ~explicitArtifacts.runResultsPath
    cfg.runResultsPath = fullfile(cfg.resultsDir, 'run_results.csv');
end
if ~explicitArtifacts.resultMatPath
    cfg.resultMatPath = fullfile(cfg.resultsDir, 'comparison_result.mat');
end
if ~isfield(cfg, 'figure8') || ~isstruct(cfg.figure8)
    cfg.figure8 = struct();
end
if ~explicitArtifacts.figure8SegmentMetricsPath
    cfg.figure8.segmentMetricsPath = fullfile(cfg.resultsDir, 'figure8_segment_metrics.csv');
end
if ~explicitArtifacts.figure8SegmentDeltaPath
    cfg.figure8.segmentDeltaPath = fullfile(cfg.resultsDir, 'figure8_segment_delta.csv');
end
cfg.figure8.presentationPlotManifestPath = fullfile(cfg.resultsDir, 'plots_presentation', 'presentation_plot_manifest.csv');
end
