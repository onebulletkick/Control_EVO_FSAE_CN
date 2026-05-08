function result = optimize_dyc_pid_laptime(overrides)
%OPTIMIZE_DYC_PID_LAPTIME Tune DYC PID gains against CarSim laptime.

if nargin < 1 || isempty(overrides)
    overrides = struct();
end

cfg = dyc_pid_optimization_config(overrides);
cfg = localNormalizeArtifactPaths(cfg);
if ~isfolder(cfg.resultsDir)
    mkdir(cfg.resultsDir);
end

preflight = run_dyc_pid_preflight(cfg);
if ~preflight.ok
    failureText = strjoin(string(preflight.failures), "; ");
    error("DYC:PIDOptimization:PreflightFailed", ...
        "PID optimization preflight failed: %s", failureText);
end
localResetRunLog(cfg);

allResults = struct([]);
bayesoptResult = struct('Mode', "testMode");
iteration = 0;

if cfg.testMode
    candidates = localDryRunCandidates(cfg);
    for idx = 1:numel(candidates)
        iteration = iteration + 1;
        candidateResult = evaluate_dyc_pid_candidate(candidates(idx), cfg, iteration);
        if iteration == 1
            allResults = candidateResult;
        else
            allResults(iteration) = candidateResult; %#ok<AGROW>
        end
    end
else
    variables = [
        optimizableVariable('Kp', cfg.pidBounds.Kp)
        optimizableVariable('Ki', cfg.pidBounds.Ki)
        optimizableVariable('Kd', cfg.pidBounds.Kd)
    ];

    objectiveFcn = @localBayesObjective;
    bayesoptResult = bayesopt(objectiveFcn, variables, ...
        'MaxObjectiveEvaluations', cfg.optimizer.maxObjectiveEvaluations, ...
        'NumSeedPoints', cfg.optimizer.initialRandomEvaluations, ...
        'AcquisitionFunctionName', cfg.optimizer.acquisitionFunctionName, ...
        'IsObjectiveDeterministic', cfg.optimizer.objectiveDeterministic);
    allResults = localReadRunLog(cfg);
end

write_dyc_pid_optimization_report(cfg, allResults, bayesoptResult);

result = struct();
result.cfg = cfg;
result.preflight = preflight;
result.results = allResults;
result.bayesoptResult = bayesoptResult;
result.best = localBestResult(allResults);

    function objective = localBayesObjective(x)
        iteration = iteration + 1;
        candidate = localCandidateFromBayesoptRow(x);
        candidateResult = evaluate_dyc_pid_candidate(candidate, cfg, iteration);
        objective = candidateResult.objective;
        if ~isfinite(objective)
            objective = cfg.penalty.hardFailureObjective;
        end
    end
end

function cfg = localNormalizeArtifactPaths(cfg)
cfg.runLogPath = localPathUnderResultsDir(cfg, 'runLogPath', 'run_log.csv');
cfg.summaryPath = localPathUnderResultsDir(cfg, 'summaryPath', 'summary.md');
cfg.resultMatPath = localPathUnderResultsDir(cfg, 'resultMatPath', 'optimization_result.mat');
cfg.bestPidScriptPath = localPathUnderResultsDir(cfg, 'bestPidScriptPath', 'best_pid_set_param.m');
end

function candidates = localDryRunCandidates(cfg)
count = max(0, round(double(cfg.optimizer.maxObjectiveEvaluations)));
candidates = repmat(struct('Kp', 0, 'Ki', 0, 'Kd', 0), 1, count);
for idx = 1:count
    alpha = (idx - 1) / max(1, count - 1);
    candidates(idx).Kp = cfg.pidBounds.Kp(1) + alpha * diff(cfg.pidBounds.Kp);
    candidates(idx).Ki = cfg.pidBounds.Ki(1) + alpha * diff(cfg.pidBounds.Ki);
    candidates(idx).Kd = cfg.pidBounds.Kd(1) + alpha * diff(cfg.pidBounds.Kd);
end
end

function candidate = localCandidateFromBayesoptRow(x)
candidate = struct();
candidate.Kp = localScalarDouble(x.Kp);
candidate.Ki = localScalarDouble(x.Ki);
candidate.Kd = localScalarDouble(x.Kd);
end

function value = localScalarDouble(inputValue)
value = double(inputValue);
if isempty(value)
    value = NaN;
else
    value = value(1);
end
end

function results = localReadRunLog(cfg)
runLogPath = localPathUnderResultsDir(cfg, 'runLogPath', 'run_log.csv');
if ~isfile(runLogPath)
    results = struct([]);
    return;
end

tableOut = readtable(runLogPath, 'TextType', 'string');
if isempty(tableOut)
    results = struct([]);
else
    results = table2struct(tableOut)';
end
end

function best = localBestResult(results)
best = struct();
if isempty(results)
    return;
end

validMask = false(1, numel(results));
objectives = NaN(1, numel(results));
for idx = 1:numel(results)
    if isfield(results(idx), 'status') && isfield(results(idx), 'objective')
        objectives(idx) = localScalarDouble(results(idx).objective);
        validMask(idx) = string(results(idx).status) == "valid" && isfinite(objectives(idx));
    end
end

if ~any(validMask)
    return;
end

validIdx = find(validMask);
[~, bestValidIdx] = min(objectives(validIdx));
best = results(validIdx(bestValidIdx));
end

function localResetRunLog(cfg)
runLogPath = localPathUnderResultsDir(cfg, 'runLogPath', 'run_log.csv');
if isfile(runLogPath)
    delete(runLogPath);
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
    error('dyc:optimization:ArtifactPathOutsideResultsDir', ...
        'Optimization artifact path for %s escapes resultsDir after canonical normalization: %s', ...
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
