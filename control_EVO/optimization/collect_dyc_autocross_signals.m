function signals = collect_dyc_autocross_signals(simOut, simfileInfo, cfg)
%COLLECT_DYC_AUTOCROSS_SIGNALS 收集 Autocross 对比评估所需信号。

signals = localEmptySignals(simfileInfo);

if isstruct(simOut) && isfield(simOut, 'signals') && isstruct(simOut.signals)
    signals = localMergeSignalStruct(signals, simOut.signals);
    if ~isfield(simOut.signals, 'source') || strlength(string(simOut.signals.source)) == 0
        signals.source = "struct";
    end
    signals.available = localHasStandardSignalData(signals);
    if signals.available
        signals.failureReason = "";
    else
        signals.failureReason = "struct signals did not contain any standard Autocross signal data";
    end
    return;
end

if localIsSimulationOutput(simOut)
    [signals, foundCount] = localCollectFromSimulationOutput(signals, simOut);
    if foundCount > 0
        signals.available = true;
        signals.source = "SimulationOutput";
        signals.failureReason = "";
    else
        [signals, foundCount, failureReason] = localCollectFromCarSimFiles(signals, simfileInfo, cfg);
        if foundCount > 0
            signals.available = true;
            signals.source = "CarSimVSB";
            signals.failureReason = "";
        else
            signals.failureReason = "Simulink.SimulationOutput does not contain supported logsout or yout signals; " + failureReason;
        end
    end
    return;
end

signals.failureReason = "unsupported simulation output type for Autocross signal collection";
end

function [signals, foundCount, failureReason] = localCollectFromCarSimFiles(signals, simfileInfo, cfg)
foundCount = 0;
failureReason = "CarSim LastRun.vs/LastRun.vsb fallback unavailable";

[vsPath, vsbPath] = localCarSimOutputPaths(simfileInfo);
if strlength(vsPath) == 0 || ~isfile(vsPath) || ~isfile(vsbPath)
    failureReason = "CarSim LastRun.vs or LastRun.vsb not found";
    return;
end

try
    metadata = jsondecode(fileread(vsPath));
    group = metadata.VsChannelGroup;
    channels = group.Channels;
    nChannels = numel(channels);
    data = localReadCarSimVsb(vsbPath, nChannels);
catch err
    failureReason = "failed to read CarSim VSB output: " + string(err.message);
    return;
end

if isempty(data)
    failureReason = "CarSim VSB output has no readable samples";
    return;
end

signals.time_s = localCarSimTime(group, size(data, 1));
foundCount = foundCount + 1;

targetFields = localTargetFields();
for fieldIdx = 1:size(targetFields, 1)
    fieldName = targetFields{fieldIdx, 1};
    if strcmp(fieldName, 'time_s')
        continue;
    end

    aliases = targetFields{fieldIdx, 2};
    [value, units] = localReadCarSimChannel(data, channels, aliases);
    if ~isempty(value)
        signals.(fieldName) = localConvertCarSimUnits(value, units, fieldName);
        foundCount = foundCount + 1;
    end
end

if isempty(signals.mz_Nm)
    signals.mz_Nm = localDerivedYawMomentFromWheelTorque(data, channels, cfg);
    if ~isempty(signals.mz_Nm)
        foundCount = foundCount + 1;
    end
end

if foundCount <= 1
    foundCount = 0;
    failureReason = "CarSim VSB output did not contain supported Autocross channels";
end
end

function [vsPath, vsbPath] = localCarSimOutputPaths(simfileInfo)
vsPath = "";
vsbPath = "";
if ~isstruct(simfileInfo)
    return;
end

basePath = "";
if isfield(simfileInfo, 'logFile')
    basePath = regexprep(string(simfileInfo.logFile), '_log\.txt$', '');
elseif isfield(simfileInfo, 'endFile')
    basePath = regexprep(string(simfileInfo.endFile), '_end\.par$', '');
end

if strlength(basePath) == 0
    return;
end

vsPath = basePath + ".vs";
vsbPath = basePath + ".vsb";
end

function data = localReadCarSimVsb(vsbPath, nChannels)
data = [];
fileInfo = dir(vsbPath);
if isempty(fileInfo) || nChannels <= 0
    return;
end

for headerBytes = [24 0]
    dataBytes = fileInfo.bytes - headerBytes;
    if dataBytes <= 0 || mod(dataBytes, 4 * nChannels) ~= 0
        continue;
    end

    fid = fopen(vsbPath, 'rb');
    if fid < 0
        return;
    end
    cleanup = onCleanup(@() fclose(fid));
    fseek(fid, headerBytes, 'bof');
    raw = fread(fid, [nChannels, inf], 'single=>double');
    if ~isempty(raw)
        data = raw.';
        return;
    end
end
end

function time_s = localCarSimTime(group, nSamples)
xStart = localOptionalNumericField(group, 'XStart', 0);
xStep = localOptionalNumericField(group, 'XStep', 1);
time_s = xStart + (0:nSamples-1) * xStep;
end

function value = localOptionalNumericField(s, fieldName, defaultValue)
value = defaultValue;
if isstruct(s) && isfield(s, fieldName) && isnumeric(s.(fieldName)) && isscalar(s.(fieldName))
    value = double(s.(fieldName));
end
end

function [value, units] = localReadCarSimChannel(data, channels, aliases)
value = [];
units = "";
for aliasIdx = 1:numel(aliases)
    alias = string(aliases{aliasIdx});
    for channelIdx = 1:numel(channels)
        channelAliases = localCarSimChannelAliases(channels(channelIdx));
        if any(strcmpi(channelAliases, alias))
            value = data(:, channelIdx).';
            units = localCarSimChannelUnits(channels(channelIdx));
            return;
        end
    end
end
end

function aliases = localCarSimChannelAliases(channel)
aliases = strings(0, 1);
if isstruct(channel) && isfield(channel, 'NameAliases')
    aliases = string(channel.NameAliases);
    aliases = aliases(:);
end
end

function units = localCarSimChannelUnits(channel)
units = "";
if isstruct(channel) && isfield(channel, 'Units')
    units = string(channel.Units);
end
end

function value = localConvertCarSimUnits(value, units, fieldName)
unitText = lower(strtrim(string(units)));
switch string(fieldName)
    case "speed_mps"
        if any(unitText == ["km/h", "kph"])
            value = value / 3.6;
        end
    case {"yawRate_radps", "yawRateTarget_radps"}
        if any(unitText == ["deg/s", "deg/sec"])
            value = deg2rad(value);
        end
    case "ay_mps2"
        if unitText == "g"
            value = value * 9.80665;
        end
end
end

function mz_Nm = localDerivedYawMomentFromWheelTorque(data, channels, cfg)
mz_Nm = [];
[myL1, ~] = localReadCarSimChannel(data, channels, {'My_Dr_L1'});
[myL2, ~] = localReadCarSimChannel(data, channels, {'My_Dr_L2'});
[myR1, ~] = localReadCarSimChannel(data, channels, {'My_Dr_R1'});
[myR2, ~] = localReadCarSimChannel(data, channels, {'My_Dr_R2'});
if isempty(myL1) || isempty(myL2) || isempty(myR1) || isempty(myR2)
    return;
end

veh = localVehicleParams(cfg);
if isempty(veh)
    return;
end

mz_Nm = ((myR1 - myL1) ./ veh.r) * (veh.tf / 2) + ...
    ((myR2 - myL2) ./ veh.r) * (veh.tr / 2);
end

function veh = localVehicleParams(cfg)
veh = [];
if ~isstruct(cfg) || ~isfield(cfg, 'modelFolder') || ~isfolder(cfg.modelFolder)
    return;
end

oldPath = path;
cleanup = onCleanup(@() path(oldPath));
addpath(char(string(cfg.modelFolder)));

try
    candidate = DYC_vehicle_params();
catch
    return;
end

requiredFields = {'r', 'tf', 'tr'};
for idx = 1:numel(requiredFields)
    if ~isfield(candidate, requiredFields{idx})
        return;
    end
end
veh = candidate;
end

function signals = localEmptySignals(simfileInfo)
signals = struct();
signals.available = false;
signals.source = "";
signals.failureReason = "";
signals.time_s = [];
signals.speed_mps = [];
signals.yawRate_radps = [];
signals.yawRateTarget_radps = [];
signals.latVeh_m = [];
signals.latTarget_m = [];
signals.ay_mps2 = [];
signals.mz_Nm = [];
signals.lastRunLogPath = "";
signals.lastRunEndPath = "";

if nargin > 0 && isstruct(simfileInfo)
    if isfield(simfileInfo, 'logFile')
        signals.lastRunLogPath = string(simfileInfo.logFile);
    end
    if isfield(simfileInfo, 'endFile')
        signals.lastRunEndPath = string(simfileInfo.endFile);
    end
end
end

function signals = localMergeSignalStruct(signals, supplied)
names = fieldnames(supplied);
for idx = 1:numel(names)
    signals.(names{idx}) = supplied.(names{idx});
end
end

function tf = localIsSimulationOutput(value)
tf = isa(value, 'Simulink.SimulationOutput');
end

function [signals, foundCount] = localCollectFromSimulationOutput(signals, simOut)
foundCount = 0;
datasets = {};

[ok, logsout] = localGetSimulationOutputValue(simOut, 'logsout');
if ok
    datasets{end+1} = logsout;
end

[ok, yout] = localGetSimulationOutputValue(simOut, 'yout');
if ok
    datasets{end+1} = yout;
end

targetFields = localTargetFields();
for fieldIdx = 1:size(targetFields, 1)
    fieldName = targetFields{fieldIdx, 1};
    aliases = targetFields{fieldIdx, 2};
    for datasetIdx = 1:numel(datasets)
        [value, timeValue] = localReadDatasetAliases(datasets{datasetIdx}, aliases);
        if ~isempty(value)
            signals.(fieldName) = value;
            if ~strcmp(fieldName, 'time_s') && isempty(signals.time_s) && ~isempty(timeValue)
                signals.time_s = timeValue;
            end
            foundCount = foundCount + 1;
            break;
        end
    end
end
end

function [ok, value] = localGetSimulationOutputValue(simOut, name)
ok = false;
value = [];
try
    value = simOut.get(name);
    ok = ~isempty(value);
catch
end
end

function [value, timeValue] = localReadDatasetAliases(dataset, aliases)
value = [];
timeValue = [];
if isempty(dataset)
    return;
end

for aliasIdx = 1:numel(aliases)
    alias = aliases{aliasIdx};
    element = localFindDatasetElement(dataset, alias);
    if ~isempty(element)
        value = localElementValues(element);
        timeValue = localElementTime(element);
        if ~isempty(value)
            return;
        end
    end
end
end

function element = localFindDatasetElement(dataset, alias)
element = [];

try
    count = dataset.numElements;
    for idx = 1:count
        candidate = dataset.get(idx);
        if isprop(candidate, 'Name') && strcmp(string(candidate.Name), string(alias))
            element = candidate;
            return;
        end
    end
catch
end
end

function value = localElementValues(element)
value = [];
try
    if isa(element, 'timeseries')
        value = double(element.Data);
        value = value(:).';
        return;
    end
catch
end

try
    if isprop(element, 'Values')
        value = localElementValues(element.Values);
        return;
    end
catch
end

try
    if isstruct(element) && isfield(element, 'Values')
        value = localElementValues(element.Values);
        return;
    end
catch
end

try
    if isnumeric(element)
        value = double(element);
        value = value(:).';
    end
catch
end
end

function value = localElementTime(element)
value = [];
try
    if isa(element, 'timeseries')
        value = double(element.Time);
        value = value(:).';
        return;
    end
catch
end

try
    if isprop(element, 'Values')
        value = localElementTime(element.Values);
        return;
    end
catch
end

try
    if isstruct(element) && isfield(element, 'Values')
        value = localElementTime(element.Values);
    end
catch
end
end

function tf = localHasStandardSignalData(signals)
fields = localStandardSignalFields();
tf = false;
for idx = 1:numel(fields)
    name = fields{idx};
    if isfield(signals, name) && ~isempty(signals.(name))
        tf = true;
        return;
    end
end
end

function fields = localStandardSignalFields()
fields = {'time_s','speed_mps','yawRate_radps','yawRateTarget_radps', ...
    'latVeh_m','latTarget_m','ay_mps2','mz_Nm'};
end

function fields = localTargetFields()
fields = {
    'time_s', {'time_s', 'Time'};
    'speed_mps', {'speed_mps', 'Vx', 'Vxz_Fwd'};
    'yawRate_radps', {'yawRate_radps', 'AVz', 'YawRate'};
    'yawRateTarget_radps', {'yawRateTarget_radps', 'YawRateTarget', 'AVzTarget'};
    'latVeh_m', {'latVeh_m', 'Lat_Veh', 'Yo'};
    'latTarget_m', {'latTarget_m', 'Lat_Targ', 'Y_Target'};
    'ay_mps2', {'ay_mps2', 'Ay'};
    'mz_Nm', {'mz_Nm', 'Mz_selected', 'Mz'};
    };
end
