function result = run_dyc_autocross_report(varargin)
%RUN_DYC_AUTOCROSS_REPORT 一键生成 Autocross DYC 对比报告。

optimizationFolder = fileparts(mfilename('fullpath'));
if ~contains(path, optimizationFolder)
    addpath(optimizationFolder);
end

if nargin == 0
    result = compare_dyc_autocross_effectiveness();
elseif nargin == 1
    result = compare_dyc_autocross_effectiveness(varargin{1});
else
    error('dyc:autocrossReport:TooManyInputs', ...
        'run_dyc_autocross_report accepts at most one overrides struct.');
end

result.generatedPaths = localGeneratedPaths(result.cfg);
localPrintGeneratedPaths(result.generatedPaths);
end

function paths = localGeneratedPaths(cfg)
paths = struct();
paths.resultsDir = string(cfg.resultsDir);
paths.reportPath = string(cfg.reportPath);
paths.comparisonMetricsPath = string(cfg.comparisonMetricsPath);
paths.runResultsPath = string(cfg.runResultsPath);
paths.plotsPresentationDir = string(fullfile(cfg.resultsDir, 'plots_presentation'));
paths.whereFasterPath = string(fullfile(cfg.resultsDir, 'autocross_where_faster.csv'));
end

function localPrintGeneratedPaths(paths)
fprintf('\nDYC Autocross 报告已生成：\n');
fprintf('  report.html: %s\n', paths.reportPath);
fprintf('  comparison_metrics.csv: %s\n', paths.comparisonMetricsPath);
fprintf('  run_results.csv: %s\n', paths.runResultsPath);
fprintf('  plots_presentation: %s\n', paths.plotsPresentationDir);
fprintf('  autocross_where_faster.csv: %s\n\n', paths.whereFasterPath);
end
