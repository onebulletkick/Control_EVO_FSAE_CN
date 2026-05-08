function append_dyc_optimization_checkpoint(cfg, result)
%APPEND_DYC_OPTIMIZATION_CHECKPOINT Append one optimization row to CSV.

cfg.runLogPath = localPathUnderResultsDir(cfg, 'runLogPath', 'run_log.csv');
if ~isfolder(cfg.resultsDir)
    mkdir(cfg.resultsDir);
end

row = localNormalizeTextColumns(struct2table(localNormalizeResult(result), 'AsArray', true));
if isfile(cfg.runLogPath)
    existing = readtable(cfg.runLogPath, 'TextType', 'string');
    output = [existing; row];
else
    output = row;
end
writetable(output, cfg.runLogPath);
end

function resultTable = localNormalizeTextColumns(resultTable)
textFields = {'stopReason','status','failureReason','lastRunLogPath'};
for idx = 1:numel(textFields)
    name = textFields{idx};
    resultTable.(name) = localColumnToString(resultTable.(name));
end
end

function values = localColumnToString(column)
values = strings(size(column));
for idx = 1:numel(column)
    if iscell(column)
        values(idx) = localStringScalar(column{idx});
    else
        values(idx) = localStringScalar(column(idx));
    end
end
end

function row = localNormalizeResult(result)
fields = {'iteration','Kp','Ki','Kd','lapTime_s','finishStation_m','objective','penalty','stopReason','status','failureReason','lastRunLogPath','elapsedWallTime_s'};
textFields = {'stopReason','status','failureReason','lastRunLogPath'};
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

function value = localStringScalar(inputValue)
value = string(inputValue);
if isempty(value)
    value = "";
else
    value = value(1);
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
