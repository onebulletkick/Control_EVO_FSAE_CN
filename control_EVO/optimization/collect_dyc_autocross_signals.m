function signals = collect_dyc_autocross_signals(simOut, simfileInfo, ~)
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
        signals.failureReason = "Simulink.SimulationOutput does not contain supported logsout or yout signals";
    end
    return;
end

signals.failureReason = "unsupported simulation output type for Autocross signal collection";
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
    'latVeh_m', {'latVeh_m', 'Lat_Veh'};
    'latTarget_m', {'latTarget_m', 'Lat_Targ'};
    'ay_mps2', {'ay_mps2', 'Ay'};
    'mz_Nm', {'mz_Nm', 'Mz_selected', 'Mz'};
    };
end
