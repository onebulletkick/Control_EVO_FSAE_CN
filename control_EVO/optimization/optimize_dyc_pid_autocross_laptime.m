function result = optimize_dyc_pid_autocross_laptime(overrides)
%OPTIMIZE_DYC_PID_AUTOCROSS_LAPTIME Tune PID gains for Autocross stop-time runs.

autocrossOverrides = struct();
autocrossOverrides.metric = struct();
autocrossOverrides.metric.primaryMetric = 'carsim_autocross_stop_time';
autocrossOverrides.metric.requiredStopReason = 'Autocross event or external control';
autocrossOverrides.simulation = struct();
autocrossOverrides.simulation.stopTime = '120';

if nargin < 1 || isempty(overrides)
    overrides = struct();
end

result = optimize_dyc_pid_laptime(localMergeStruct(autocrossOverrides, overrides));
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
