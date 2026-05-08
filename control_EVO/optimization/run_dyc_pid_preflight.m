function report = run_dyc_pid_preflight(cfg)
%RUN_DYC_PID_PREFLIGHT Validate prerequisites before PID optimization.

report = struct();
report.ok = false;
report.failures = strings(0, 1);
report.warnings = strings(0, 1);
report.model = struct();
report.simfile = struct();
report.metric = struct();

if ~cfg.testMode || localHasCustomModelInfoFcn(cfg)
    add_dyc_carsim_solver_path(cfg);
    report.model = localCollectModelInfo(cfg);
    [modelFailures, modelWarnings] = localValidateModelInfo(cfg, report.model);
    report.failures = [report.failures; modelFailures];
    report.warnings = [report.warnings; modelWarnings];
end

if ~cfg.testMode
    if ~cfg.preflight.bayesoptAvailableFcn()
        report.failures(end+1, 1) = "bayesopt is unavailable";
    end
end

if ~isfile(cfg.simfilePath)
    report.failures(end+1, 1) = "simfile not found: " + string(cfg.simfilePath);
    return;
end

simfileInfo = parse_dyc_simfile(cfg.simfilePath);
report.simfile = simfileInfo;
if ~simfileInfo.isValid
    report.failures(end+1, 1) = "simfile parse failed: " + simfileInfo.failureReason;
    return;
end

metric = extract_dyc_laptime_metric(simfileInfo.logFile, simfileInfo.endFile, cfg);
report.metric = metric;
if metric.status ~= "valid"
    report.failures(end+1, 1) = "baseline laptime invalid: " + metric.failureReason;
end

if isempty(report.failures)
    report.ok = true;
end
end

function tf = localHasCustomModelInfoFcn(cfg)
tf = isfield(cfg.preflight, 'modelInfoFcn') && ~isempty(cfg.preflight.modelInfoFcn);
end

function info = localCollectModelInfo(cfg)
if localHasCustomModelInfoFcn(cfg)
    info = cfg.preflight.modelInfoFcn(cfg);
else
    info = localInspectSimulinkModel(cfg);
end
end

function [failures, warnings] = localValidateModelInfo(cfg, info)
failures = strings(0, 1);
warnings = strings(0, 1);

if ~localLogicalField(info, 'modelFileExists', false)
    failures(end+1, 1) = "model file not found: " + string(cfg.modelPath);
end

loadError = localStringField(info, 'loadError', "");
if loadError ~= ""
    failures(end+1, 1) = "model load failed: " + loadError;
end

pidInfo = localStructField(info, 'pidBlock');
if ~localLogicalField(pidInfo, 'exists', false)
    failures(end+1, 1) = "PID block not found: " + string(cfg.pidBlock);
else
    missingParams = strings(0, 1);
    paramNames = ["P"; "I"; "D"];
    for idx = 1:numel(paramNames)
        fieldName = "has" + paramNames(idx);
        if ~localLogicalField(pidInfo, fieldName, false)
            missingParams(end+1, 1) = paramNames(idx); %#ok<AGROW>
        end
    end
    if ~isempty(missingParams)
        failures(end+1, 1) = "PID block is missing parameter(s): " + strjoin(missingParams, ", ");
    end
end

switchInfo = localStructField(info, 'yawMomentSwitch');
if ~localLogicalField(switchInfo, 'exists', false)
    failures(end+1, 1) = "YawMomentControlMode switch not found: " + string(cfg.yawMomentModeSwitchBlock);
    return;
end

blockType = localStringField(switchInfo, 'blockType', "");
if blockType ~= "ManualSwitch"
    failures(end+1, 1) = "YawMomentControlMode is not a ManualSwitch: " + blockType;
end

selectedPort = localManualSwitchSelectedPort(cfg.yawMomentPidSwitchValue);
if isnan(selectedPort)
    failures(end+1, 1) = "unsupported PID ManualSwitch value: " + string(cfg.yawMomentPidSwitchValue);
else
    selectedGotoTag = localStringField(switchInfo, 'selectedGotoTag', "");
    if selectedGotoTag ~= string(cfg.yawMomentPidGotoTag)
        failures(end+1, 1) = "PID ManualSwitch value " + string(cfg.yawMomentPidSwitchValue) + ...
            " selects GotoTag '" + selectedGotoTag + "', expected '" + string(cfg.yawMomentPidGotoTag) + "'";
    end
end

savedValue = localStringField(switchInfo, 'savedValue', "");
if savedValue ~= "" && savedValue ~= string(cfg.yawMomentPidSwitchValue)
    warnings(end+1, 1) = "saved YawMomentControlMode switch value is " + savedValue + ...
        "; candidate simulation will force PID value " + string(cfg.yawMomentPidSwitchValue);
end
end

function info = localInspectSimulinkModel(cfg)
info = localEmptyModelInfo(cfg);
info.modelFileExists = isfile(cfg.modelPath);
if ~info.modelFileExists
    return;
end

wasLoaded = bdIsLoaded(cfg.modelName);
try
    load_system(cfg.modelPath);
catch err
    info.loadError = string(err.message);
    return;
end
cleanup = onCleanup(@() localCloseIfLoadedHere(cfg.modelName, wasLoaded));

info.pidBlock.exists = localBlockExists(cfg.pidBlock);
if info.pidBlock.exists
    params = get_param(cfg.pidBlock, 'ObjectParameters');
    info.pidBlock.hasP = isfield(params, 'P');
    info.pidBlock.hasI = isfield(params, 'I');
    info.pidBlock.hasD = isfield(params, 'D');
end

info.yawMomentSwitch.exists = localBlockExists(cfg.yawMomentModeSwitchBlock);
if info.yawMomentSwitch.exists
    info.yawMomentSwitch.blockType = string(get_param(cfg.yawMomentModeSwitchBlock, 'BlockType'));
    info.yawMomentSwitch.savedValue = string(get_param(cfg.yawMomentModeSwitchBlock, 'sw'));
    info.yawMomentSwitch.configuredSelectedPort = localManualSwitchSelectedPort(cfg.yawMomentPidSwitchValue);
    info.yawMomentSwitch.inputGotoTags = localInputGotoTags(cfg.yawMomentModeSwitchBlock);
    if ~isnan(info.yawMomentSwitch.configuredSelectedPort) && ...
            info.yawMomentSwitch.configuredSelectedPort <= numel(info.yawMomentSwitch.inputGotoTags)
        info.yawMomentSwitch.selectedGotoTag = ...
            info.yawMomentSwitch.inputGotoTags(info.yawMomentSwitch.configuredSelectedPort);
    end
end
end

function info = localEmptyModelInfo(cfg)
info = struct();
info.modelPath = string(cfg.modelPath);
info.modelFileExists = false;
info.loadError = "";
info.pidBlock = struct('path', string(cfg.pidBlock), 'exists', false, ...
    'hasP', false, 'hasI', false, 'hasD', false);
info.yawMomentSwitch = struct('path', string(cfg.yawMomentModeSwitchBlock), ...
    'exists', false, 'blockType', "", 'savedValue', "", ...
    'configuredSelectedPort', NaN, 'selectedGotoTag', "", ...
    'inputGotoTags', strings(0, 1));
end

function tf = localBlockExists(blockPath)
tf = getSimulinkBlockHandle(blockPath) ~= -1;
end

function tags = localInputGotoTags(blockPath)
portHandles = get_param(blockPath, 'PortHandles');
inports = portHandles.Inport;
tags = strings(numel(inports), 1);
for idx = 1:numel(inports)
    lineHandle = get_param(inports(idx), 'Line');
    if isequal(lineHandle, -1)
        continue;
    end
    sourceBlock = get_param(lineHandle, 'SrcBlockHandle');
    if isequal(sourceBlock, -1)
        continue;
    end
    try
        tags(idx) = string(get_param(sourceBlock, 'GotoTag'));
    catch
        tags(idx) = "";
    end
end
end

function selectedPort = localManualSwitchSelectedPort(switchValue)
switchValue = strtrim(char(string(switchValue)));
if strcmp(switchValue, '1')
    selectedPort = 1;
elseif strcmp(switchValue, '0')
    selectedPort = 2;
else
    selectedPort = NaN;
end
end

function localCloseIfLoadedHere(modelName, wasLoaded)
if ~wasLoaded && bdIsLoaded(modelName)
    close_system(modelName, 0);
end
end

function value = localLogicalField(inputStruct, fieldName, defaultValue)
if isstruct(inputStruct) && isfield(inputStruct, fieldName)
    value = logical(inputStruct.(fieldName));
else
    value = defaultValue;
end
end

function value = localStringField(inputStruct, fieldName, defaultValue)
if isstruct(inputStruct) && isfield(inputStruct, fieldName)
    value = string(inputStruct.(fieldName));
    if isempty(value)
        value = defaultValue;
    else
        value = value(1);
    end
else
    value = defaultValue;
end
end

function value = localStructField(inputStruct, fieldName)
if isstruct(inputStruct) && isfield(inputStruct, fieldName) && isstruct(inputStruct.(fieldName))
    value = inputStruct.(fieldName);
else
    value = struct();
end
end
