function info = parse_dyc_simfile(simfilePath)
%PARSE_DYC_SIMFILE Parse a CarSim simfile and resolve result paths.

info = struct();
info.simfilePath = simfilePath;
info.isValid = false;
info.failureReason = '';
info.rootFileName = '';
info.outputPath = '';
info.workDir = '';
info.outputFilePrefix = '';
info.logFile = '';
info.endFile = '';
info.progDir = '';
info.matlabSolverFolder = '';
info.productVersion = '';
info.extModelStep_s = NaN;

if ~isfile(simfilePath)
    info.failureReason = 'simfile not found';
    return;
end

lines = readlines(simfilePath);
macros = containers.Map('KeyType', 'char', 'ValueType', 'char');

for idx = 1:numel(lines)
    line = strtrim(lines(idx));
    if strlength(line) == 0
        continue;
    end

    macroMatch = regexp(char(line), '^SET_MACRO\s+\$\(([^)]+)\)\$\s+(.+)$', 'tokens', 'once');
    if ~isempty(macroMatch)
        macros(macroMatch{1}) = strtrim(macroMatch{2});
        continue;
    end

    productMatch = regexp(char(line), '^PRODUCT_VER\s+(.+)$', 'tokens', 'once');
    if ~isempty(productMatch)
        info.productVersion = strtrim(productMatch{1});
        continue;
    end

    progDirMatch = regexp(char(line), '^PROGDIR\s+(.+)$', 'tokens', 'once');
    if ~isempty(progDirMatch)
        info.progDir = strtrim(progDirMatch{1});
        info.matlabSolverFolder = fullfile(info.progDir, 'Programs', 'solvers', 'Matlab');
        continue;
    end

    stepMatch = regexp(char(line), '^EXT_MODEL_STEP\s+([0-9.Ee+-]+)', 'tokens', 'once');
    if ~isempty(stepMatch)
        info.extModelStep_s = str2double(stepMatch{1});
    end
end

if ~isKey(macros, 'TIMESTAMP')
    macros('TIMESTAMP') = 'LastRun';
end

required = {'ROOT_FILE_NAME', 'OUTPUT_PATH', 'WORK_DIR', 'OUTPUT_FILE_PREFIX'};
for idx = 1:numel(required)
    if ~isKey(macros, required{idx})
        info.failureReason = ['missing macro ' required{idx}];
        return;
    end
end

[info.outputPath, resolved, reason] = localResolveMacros(macros('OUTPUT_PATH'), macros, 'OUTPUT_PATH');
if ~resolved
    info.failureReason = reason;
    return;
end

[info.workDir, resolved, reason] = localResolveMacros(macros('WORK_DIR'), macros, 'WORK_DIR');
if ~resolved
    info.failureReason = reason;
    return;
end

[info.outputFilePrefix, resolved, reason] = localResolveMacros(macros('OUTPUT_FILE_PREFIX'), macros, 'OUTPUT_FILE_PREFIX');
if ~resolved
    info.failureReason = reason;
    return;
end

[info.rootFileName, resolved, reason] = localResolveMacros(macros('ROOT_FILE_NAME'), macros, 'ROOT_FILE_NAME');
if ~resolved
    info.failureReason = reason;
    return;
end

info.logFile = [info.outputFilePrefix '_log.txt'];
info.endFile = [info.outputFilePrefix '_end.par'];
info.isValid = true;
end

function [value, isResolved, failureReason] = localResolveMacros(value, macros, fieldName)
keys = macros.keys;
isResolved = false;
failureReason = '';
seenValues = containers.Map('KeyType', 'char', 'ValueType', 'logical');
maxIterations = max(100, numel(keys) * 10);

for iteration = 1:maxIterations
    if isKey(seenValues, value)
        failureReason = ['cyclic macro expansion in ' fieldName];
        return;
    end
    seenValues(value) = true;

    changed = false;
    for idx = 1:numel(keys)
        token = ['$(' keys{idx} ')$'];
        if contains(value, token)
            nextValue = strrep(value, token, macros(keys{idx}));
            if strcmp(nextValue, value)
                failureReason = ['cyclic macro expansion in ' fieldName];
                return;
            end
            value = nextValue;
            changed = true;
        end
    end

    if ~changed
        if ~isempty(regexp(value, '\$\([^)]+\)\$', 'once'))
            failureReason = ['unresolved macro token in ' fieldName];
            return;
        end
        isResolved = true;
        return;
    end
end

failureReason = ['macro expansion limit exceeded in ' fieldName];
end
