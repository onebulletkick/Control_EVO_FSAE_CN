function result = run_dyc_figure8_report(varargin)
%RUN_DYC_FIGURE8_REPORT 一键生成八字绕环 DYC 对比报告。

optimizationFolder = fileparts(mfilename('fullpath'));
if ~contains(path, optimizationFolder)
    addpath(optimizationFolder);
end

if nargin == 0
    result = compare_dyc_figure8_effectiveness();
elseif nargin == 1
    result = compare_dyc_figure8_effectiveness(varargin{1});
else
    error('dyc:figure8Report:TooManyInputs', ...
        'run_dyc_figure8_report accepts at most one overrides struct.');
end

result.generatedPaths = localGeneratedPaths(result.cfg);
localPrintGeneratedPaths(result.generatedPaths);
end

function paths = localGeneratedPaths(cfg)
paths = struct();
paths.resultsDir = string(cfg.resultsDir);
paths.reportPath = string(cfg.reportPath);
paths.comparisonMetricsPath = string(cfg.comparisonMetricsPath);
paths.figure8SegmentDeltaPath = string(cfg.figure8.segmentDeltaPath);
paths.plotsPresentationDir = string(fullfile(cfg.resultsDir, 'plots_presentation'));
paths.whereFasterPath = string(fullfile(cfg.resultsDir, 'figure8_where_faster.csv'));
end

function localPrintGeneratedPaths(paths)
fprintf('\nDYC 八字绕环报告已生成：\n');
fprintf('  report.html: %s\n', paths.reportPath);
fprintf('  comparison_metrics.csv: %s\n', paths.comparisonMetricsPath);
fprintf('  figure8_segment_delta.csv: %s\n', paths.figure8SegmentDeltaPath);
fprintf('  plots_presentation: %s\n', paths.plotsPresentationDir);
fprintf('  figure8_where_faster.csv: %s\n\n', paths.whereFasterPath);
end
