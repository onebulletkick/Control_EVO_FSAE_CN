function metric = extract_dyc_laptime_metric(logFile, endFile, cfg)
%EXTRACT_DYC_LAPTIME_METRIC Extract configured laptime metric from CarSim logs.

metric = struct();
metric.status = "invalid";
metric.lapTime_s = NaN;
metric.finishStation_m = NaN;
metric.objective = cfg.penalty.hardFailureObjective;
metric.penalty = cfg.penalty.hardFailureObjective;
metric.stopReason = "";
metric.failureReason = "";
metric.logFile = string(logFile);
metric.endFile = string(endFile);
metric.sStop_m = cfg.metric.sStop_m;
metric.svStation_m = NaN;

if ~isfile(logFile)
    metric.failureReason = "LastRun_log.txt not found";
    return;
end

logText = fileread(logFile);
if ~contains(logText, "Run started") || ~contains(logText, "Run stopped")
    metric.failureReason = "CarSim log does not contain Run started and Run stopped";
    return;
end

metricName = string(cfg.metric.primaryMetric);
if metricName == "carsim_autocross_stop_time" || metricName == "carsim_autocross_external_stop_time"
    metric = localExtractAutocrossStopMetric(metric, logText, endFile);
    return;
end

pattern = 'Run stopped at t\s*=\s*([0-9.Ee+-]+)\.\s*Station limit reached:\s*driver station\s*=\s*([0-9.Ee+-]+)';
tokens = regexp(logText, pattern, 'tokens', 'once');
if isempty(tokens)
    metric.failureReason = "CarSim stop reason was not station limit";
    return;
end

metric.lapTime_s = str2double(tokens{1});
metric.finishStation_m = str2double(tokens{2});
metric.stopReason = "Station limit reached";

if isfile(endFile)
    metric.svStation_m = localReadParValue(endFile, 'SV_STATION');
end

if ~isfinite(metric.lapTime_s) || metric.lapTime_s <= 0
    metric.failureReason = "laptime was missing or non-positive";
    metric.lapTime_s = NaN;
    metric.finishStation_m = NaN;
    return;
end

stationError = abs(metric.finishStation_m - cfg.metric.sStop_m);
if stationError > cfg.metric.finishStationTolerance_m
    metric.failureReason = "finish station outside tolerance";
    metric.lapTime_s = NaN;
    return;
end

metric.status = "valid";
metric.penalty = 0;
metric.objective = metric.lapTime_s;
metric.failureReason = "";
end

function metric = localExtractAutocrossStopMetric(metric, logText, endFile)
pattern = ['Run stopped at t\s*=\s*([0-9.Ee+-]+)\.\s*' ...
    '(External control \(manual or external model\)|VS Command STOP_RUN_NOW End event triggered)'];
tokens = regexp(logText, pattern, 'tokens', 'once');
if isempty(tokens)
    metric.failureReason = "CarSim stop reason was not Autocross event or external control";
    return;
end

metric.lapTime_s = str2double(tokens{1});
metric.stopReason = string(tokens{2});

if isfile(endFile)
    metric.svStation_m = localReadParValue(endFile, 'SV_STATION');
end

if ~isfinite(metric.lapTime_s) || metric.lapTime_s <= 0
    metric.failureReason = "laptime was missing or non-positive";
    metric.lapTime_s = NaN;
    return;
end

metric.status = "valid";
metric.penalty = 0;
metric.objective = metric.lapTime_s;
metric.failureReason = "";
end

function value = localReadParValue(parFile, key)
value = NaN;
lines = readlines(parFile);
pattern = ['^' regexptranslate('escape', key) '\s+([0-9.Ee+-]+)'];
for idx = 1:numel(lines)
    tokens = regexp(char(strtrim(lines(idx))), pattern, 'tokens', 'once');
    if ~isempty(tokens)
        value = str2double(tokens{1});
        return;
    end
end
end
