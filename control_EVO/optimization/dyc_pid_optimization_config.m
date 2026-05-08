function cfg = dyc_pid_optimization_config(varargin)
%DYC_PID_OPTIMIZATION_CONFIG Configuration for DYC PID laptime optimization.

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

cfg.pidBounds = struct();
cfg.pidBounds.Kp = [2000 12000];
cfg.pidBounds.Ki = [0 800];
cfg.pidBounds.Kd = [0 800];

cfg.baseline = struct('Kp', 6000, 'Ki', 200, 'Kd', 0);

cfg.metric = struct();
cfg.metric.primaryMetric = 'carsim_station_stop_time';
cfg.metric.sStop_m = 245;
cfg.metric.finishStationTolerance_m = 1.0;
cfg.metric.requiredStopReason = 'Station limit reached';

cfg.optimizer = struct();
cfg.optimizer.maxObjectiveEvaluations = 60;
cfg.optimizer.initialRandomEvaluations = 10;
cfg.optimizer.acquisitionFunctionName = 'expected-improvement-plus';
cfg.optimizer.objectiveDeterministic = false;

cfg.penalty = struct();
cfg.penalty.hardFailureObjective = 9999;

cfg.preflight = struct();
cfg.preflight.bayesoptAvailableFcn = @() ~isempty(which('bayesopt'));
cfg.preflight.modelInfoFcn = [];

cfg.resultsRoot = fullfile(optimizationFolder, 'results');
cfg.runTimestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
cfg.resultsDir = fullfile(cfg.resultsRoot, [cfg.runTimestamp '_pid_laptime']);
cfg.runLogPath = fullfile(cfg.resultsDir, 'run_log.csv');
cfg.summaryPath = fullfile(cfg.resultsDir, 'summary.md');
cfg.resultMatPath = fullfile(cfg.resultsDir, 'optimization_result.mat');
cfg.bestPidScriptPath = fullfile(cfg.resultsDir, 'best_pid_set_param.m');

cfg.simulation = struct();
cfg.simulation.stopTime = '30';
cfg.simulation.useFastRestart = 'off';
cfg.simulation.verifyLastRunTimestamp = true;
cfg.simulation.simulateFcn = [];

cfg.testMode = false;

if nargin > 0
    overrides = varargin{1};
    cfg = localMergeStruct(cfg, overrides);
end
end

function out = localMergeStruct(base, overrides)
out = base;
names = fieldnames(overrides);
for idx = 1:numel(names)
    name = names{idx};
    if isstruct(overrides.(name)) && isfield(out, name) && isstruct(out.(name))
        out.(name) = localMergeStruct(out.(name), overrides.(name));
    else
        out.(name) = overrides.(name);
    end
end
end
