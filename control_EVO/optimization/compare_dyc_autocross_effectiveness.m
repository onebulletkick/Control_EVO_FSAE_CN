function result = compare_dyc_autocross_effectiveness(varargin)
%COMPARE_DYC_AUTOCROSS_EFFECTIVENESS 串联 Autocross DYC 开关对比流程。

if nargin == 0
    cfg = dyc_autocross_comparison_config();
elseif nargin == 1
    cfg = dyc_autocross_comparison_config(varargin{1});
else
    error('dyc:autocrossComparison:TooManyInputs', ...
        'compare_dyc_autocross_effectiveness accepts at most one overrides struct.');
end

runResults = struct([]);
for caseIdx = 1:numel(cfg.cases)
    caseResults = evaluate_dyc_autocross_case(cfg.cases(caseIdx), cfg);
    runResults = [runResults caseResults]; %#ok<AGROW>
end

metrics = compute_dyc_autocross_metrics(runResults, cfg);
cfg = write_dyc_autocross_comparison_report(cfg, runResults, metrics);

result = struct();
result.cfg = cfg;
result.runResults = runResults;
result.metrics = metrics;
end
