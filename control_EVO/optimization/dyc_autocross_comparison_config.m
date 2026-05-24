function cfg = dyc_autocross_comparison_config(varargin)
%DYC_AUTOCROSS_COMPARISON_CONFIG Autocross DYC 开关对比实验配置。

optimizationFolder = fileparts(mfilename('fullpath'));
controlFolder = fileparts(optimizationFolder);
repoRoot = fileparts(controlFolder);

cfg = struct();
cfg.repoRoot = repoRoot;
cfg.optimizationFolder = optimizationFolder;
cfg.modelFolder = controlFolder;
cfg.modelName = 'DYC_1_9_test';
cfg.modelPath = fullfile(cfg.modelFolder, [cfg.modelName '.slx']);
cfg.simfilePath = fullfile(cfg.modelFolder, 'simfile.sim');
cfg.pidBlock = [cfg.modelName '/EVO_Control_System/YawMomentControl/PID_YawMomentController'];
cfg.yawMomentModeSwitchBlock = [cfg.modelName '/EVO_Control_System/YawMomentControl/YawMomentControlMode'];
cfg.yawMomentPidSwitchValue = '1';
cfg.yawMomentPidGotoTag = 'Mz_pid';
cfg.repeatCount = 1;

cfg.cases = [ ...
    struct( ...
        'id', 'dyc_off', ...
        'displayName', 'DYC 关闭', ...
        'pid', struct('Kp', 0, 'Ki', 0, 'Kd', 0), ...
        'description', '强制使用 PID 分支，零 PID 增益表示无额外横摆力矩干预。'), ...
    struct( ...
        'id', 'dyc_on', ...
        'displayName', '当前 PID DYC', ...
        'pid', struct('Kp', 6000, 'Ki', 200, 'Kd', 0), ...
        'description', '使用当前基线 PID 参数进行 DYC 横摆力矩控制。')];

cfg.metric = struct();
cfg.metric.primaryMetric = 'carsim_autocross_stop_time';
cfg.metric.sStop_m = 245;
cfg.metric.finishStationTolerance_m = 1.0;
cfg.metric.requiredStopReason = 'VS Command STOP_RUN_NOW End event triggered';

cfg.penalty = struct();
cfg.penalty.hardFailureObjective = 9999;

cfg.simulation = struct();
cfg.simulation.stopTime = '120';
cfg.simulation.useFastRestart = 'off';
cfg.simulation.verifyLastRunTimestamp = true;
cfg.simulation.simulateFcn = [];

cfg.thresholds = struct();
cfg.thresholds.mzActive_Nm = 50;

cfg.resultsRoot = fullfile(optimizationFolder, 'results');
cfg.runTimestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
cfg.resultsDir = fullfile(cfg.resultsRoot, [cfg.runTimestamp '_dyc_autocross_comparison']);
cfg.reportPath = fullfile(cfg.resultsDir, 'report.html');
cfg.comparisonMetricsPath = fullfile(cfg.resultsDir, 'comparison_metrics.csv');
cfg.runResultsPath = fullfile(cfg.resultsDir, 'run_results.csv');
cfg.resultMatPath = fullfile(cfg.resultsDir, 'comparison_result.mat');

cfg.testMode = false;

if nargin > 0
    overrides = varargin{1};
    hasResultsRootOverride = isfield(overrides, 'resultsRoot');
    hasRunTimestampOverride = isfield(overrides, 'runTimestamp');
    hasResultsDirOverride = isfield(overrides, 'resultsDir');
    explicitArtifacts = localExplicitArtifactMap(overrides);

    cfg = localMergeStruct(cfg, overrides);

    if (hasResultsRootOverride || hasRunTimestampOverride) && ~hasResultsDirOverride
        cfg.resultsDir = fullfile(cfg.resultsRoot, [cfg.runTimestamp '_dyc_autocross_comparison']);
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
artifactFields = {'reportPath', 'comparisonMetricsPath', 'runResultsPath', 'resultMatPath'};
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
end
