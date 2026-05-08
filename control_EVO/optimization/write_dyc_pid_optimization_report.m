function write_dyc_pid_optimization_report(cfg, results, bayesoptResult)
%WRITE_DYC_PID_OPTIMIZATION_REPORT Write summary artifacts for PID optimization.

cfg.summaryPath = localPathUnderResultsDir(cfg, 'summaryPath', 'summary.md');
cfg.resultMatPath = localPathUnderResultsDir(cfg, 'resultMatPath', 'optimization_result.mat');
cfg.bestPidScriptPath = localPathUnderResultsDir(cfg, 'bestPidScriptPath', 'best_pid_set_param.m');
if ~isfolder(cfg.resultsDir)
    mkdir(cfg.resultsDir);
end

resultTable = localNormalizeResults(results);
validRows = resultTable(resultTable.status == "valid", :);
hasBest = ~isempty(validRows);
if hasBest
    [~, bestIdx] = min(validRows.objective);
    best = validRows(bestIdx, :);
else
    best = resultTable([], :);
end

save(cfg.resultMatPath, 'cfg', 'results', 'bayesoptResult', 'resultTable', 'best');
localWriteApplyScript(cfg, best, hasBest);
localWriteSummary(cfg, resultTable, best, hasBest, bayesoptResult);
end

function resultTable = localNormalizeResults(results)
fields = localResultFields();
if isempty(results)
    resultTable = localEmptyResultTable();
    return;
end

results = results(:);
resultTable = localEmptyResultTable(numel(results));
for rowIdx = 1:numel(results)
    row = localNormalizeResult(results(rowIdx));
    for fieldIdx = 1:numel(fields)
        name = fields{fieldIdx};
        if any(strcmp(name, localTextFields()))
            resultTable.(name)(rowIdx) = localStringScalar(row.(name));
        else
            resultTable.(name)(rowIdx) = localNumericScalar(row.(name));
        end
    end
end
end

function row = localNormalizeResult(result)
fields = localResultFields();
textFields = localTextFields();
for idx = 1:numel(fields)
    name = fields{idx};
    if ~isfield(result, name)
        if any(strcmp(name, textFields))
            result.(name) = "";
        else
            result.(name) = NaN;
        end
    end
end
row = struct();
for idx = 1:numel(fields)
    name = fields{idx};
    if any(strcmp(name, textFields))
        row.(name) = localStringScalar(result.(name));
    else
        row.(name) = result.(name);
    end
end
end

function fields = localResultFields()
fields = {'iteration','Kp','Ki','Kd','lapTime_s','finishStation_m','objective','penalty','stopReason','status','failureReason','lastRunLogPath','elapsedWallTime_s'};
end

function fields = localTextFields()
fields = {'stopReason','status','failureReason','lastRunLogPath'};
end

function resultTable = localEmptyResultTable(rowCount)
if nargin < 1
    rowCount = 0;
end
fields = localResultFields();
variableTypes = ["double","double","double","double","double","double","double","double","string","string","string","string","double"];
resultTable = table('Size', [rowCount numel(fields)], 'VariableTypes', variableTypes, 'VariableNames', fields);
end

function value = localStringScalar(inputValue)
value = string(inputValue);
if isempty(value)
    value = "";
else
    value = value(1);
end
end

function value = localNumericScalar(inputValue)
value = double(inputValue);
if isempty(value)
    value = NaN;
else
    value = value(1);
end
end

function localWriteApplyScript(cfg, best, hasBest)
if ~hasBest
    lines = [
        "% No valid PID candidate was found."
        "% No PID apply command was written."
    ];
    writelines(lines, cfg.bestPidScriptPath);
    return;
end

lines = [
    "mdl = '" + cfg.modelName + "';"
    "pidBlock = [mdl '/EVO_Control_System/YawMomentControl/PID_YawMomentController'];"
    "bestKp = " + string(best.Kp) + ";"
    "bestKi = " + string(best.Ki) + ";"
    "bestKd = " + string(best.Kd) + ";"
    "set_param(pidBlock, 'P', num2str(bestKp), 'I', num2str(bestKi), 'D', num2str(bestKd));"
];
writelines(lines, cfg.bestPidScriptPath);
end

function localWriteSummary(cfg, resultTable, best, hasBest, bayesoptResult)
validCount = sum(resultTable.status == "valid");
invalidCount = height(resultTable) - validCount;
baseline = cfg.baseline;
baselineLine = sprintf('Baseline PID: Kp = %.6g, Ki = %.6g, Kd = %.6g', baseline.Kp, baseline.Ki, baseline.Kd);
if hasBest
    bestLine = sprintf('Best PID: Kp = %.6g, Ki = %.6g, Kd = %.6g, laptime = %.4f s, objective = %.4f', best.Kp, best.Ki, best.Kd, best.lapTime_s, best.objective);
    applyLine = "See `best_pid_set_param.m`.";
else
    bestLine = "No valid PID candidate found. No apply command was generated.";
    applyLine = "No valid PID candidate was found, so no apply command is available.";
end

lines = [
    "# DYC PID Laptime Optimization Summary"
    ""
    "## Metric"
    ""
    "Primary metric: `" + string(cfg.metric.primaryMetric) + "`"
    localMetricValidationLine(cfg)
    ""
    "## Baseline"
    ""
    baselineLine
    ""
    "## Best PID"
    ""
    bestLine
    ""
    "## Run Counts"
    ""
    "Evaluated candidates: " + string(height(resultTable))
    "Valid candidates: " + string(validCount)
    "Invalid candidates: " + string(invalidCount)
    ""
    "## Best Apply Command"
    ""
    applyLine
    ""
    "## Optimizer"
    ""
    "Bayesopt result class: " + string(class(bayesoptResult))
];

writelines(lines, cfg.summaryPath);
end

function line = localMetricValidationLine(cfg)
metricName = string(cfg.metric.primaryMetric);
if metricName == "carsim_autocross_stop_time" || metricName == "carsim_autocross_external_stop_time"
    line = "Valid finish requires an Autocross end event or `External control (manual or external model)`; the stop time is used as laptime.";
else
    line = "Valid finish requires `Station limit reached` at `SSTOP = " + string(cfg.metric.sStop_m) + " m`.";
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
