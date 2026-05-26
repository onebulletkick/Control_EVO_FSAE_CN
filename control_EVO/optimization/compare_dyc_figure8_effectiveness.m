function result = compare_dyc_figure8_effectiveness(varargin)
%COMPARE_DYC_FIGURE8_EFFECTIVENESS 串联八字绕环 DYC 开关对比流程。

if nargin == 0
    cfg = dyc_figure8_comparison_config();
elseif nargin == 1
    cfg = dyc_figure8_comparison_config(varargin{1});
else
    error('dyc:figure8Comparison:TooManyInputs', ...
        'compare_dyc_figure8_effectiveness accepts at most one overrides struct.');
end

runResults = struct([]);
for caseIdx = 1:numel(cfg.cases)
    caseResults = evaluate_dyc_autocross_case(cfg.cases(caseIdx), cfg);
    runResults = [runResults caseResults]; %#ok<AGROW>
end

metrics = compute_dyc_figure8_metrics(runResults, cfg);
cfg = write_dyc_figure8_comparison_report(cfg, runResults, metrics);

result = struct();
result.cfg = cfg;
result.runResults = runResults;
result.metrics = metrics;
end
