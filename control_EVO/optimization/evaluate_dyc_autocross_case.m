function results = evaluate_dyc_autocross_case(caseDef, cfg)
%EVALUATE_DYC_AUTOCROSS_CASE 执行单个 Autocross 对比 case 的重复仿真。

repeatCount = cfg.repeatCount;
results = repmat(localEmptyResult(caseDef, 1), 1, repeatCount);

for repeatIndex = 1:repeatCount
    ticStart = tic;
    result = localEmptyResult(caseDef, repeatIndex);

    try
        simfileInfo = parse_dyc_simfile(cfg.simfilePath);
        result.lastRunLogPath = string(simfileInfo.logFile);
        result.lastRunEndPath = string(simfileInfo.endFile);
        result.signals = localEmptySignals(simfileInfo);

        if ~simfileInfo.isValid
            result = localHardFailure(result, cfg, "simfile parse failed: " + string(simfileInfo.failureReason));
            result.elapsedWallTime_s = toc(ticStart);
            results(repeatIndex) = result;
            continue;
        end

        beforeLogEvidence = localFileEvidence(simfileInfo.logFile);
        beforeEndEvidence = localFileEvidence(simfileInfo.endFile);

        if isempty(cfg.simulation.simulateFcn)
            simOut = localRunSimulation(caseDef, cfg);
        else
            simOut = cfg.simulation.simulateFcn(cfg, caseDef, repeatIndex);
        end

        afterLogEvidence = localFileEvidence(simfileInfo.logFile);
        afterEndEvidence = localFileEvidence(simfileInfo.endFile);
        if localShouldRejectStaleFile(cfg, beforeLogEvidence, afterLogEvidence)
            result = localHardFailure(result, cfg, ...
                "LastRun_log.txt did not refresh after simulation; refusing to score stale CarSim log");
            result.elapsedWallTime_s = toc(ticStart);
            results(repeatIndex) = result;
            continue;
        end
        if localShouldRejectStaleFile(cfg, beforeEndEvidence, afterEndEvidence)
            result = localHardFailure(result, cfg, ...
                "LastRun_end.par did not refresh after simulation; refusing to mix fresh log with stale CarSim end evidence");
            result.elapsedWallTime_s = toc(ticStart);
            results(repeatIndex) = result;
            continue;
        end

        metric = extract_dyc_laptime_metric(simfileInfo.logFile, simfileInfo.endFile, cfg);
        result.lapTime_s = metric.lapTime_s;
        result.finishStation_m = metric.finishStation_m;
        result.svStation_m = metric.svStation_m;
        result.objective = metric.objective;
        result.penalty = metric.penalty;
        result.stopReason = metric.stopReason;
        result.status = metric.status;
        result.failureReason = metric.failureReason;
        result.lastRunLogPath = string(simfileInfo.logFile);
        result.lastRunEndPath = string(simfileInfo.endFile);
        result.signals = collect_dyc_autocross_signals(simOut, simfileInfo, cfg);
    catch err
        result = localHardFailure(result, cfg, string(err.message));
    end

    result.elapsedWallTime_s = toc(ticStart);
    results(repeatIndex) = result;
end
end

function result = localEmptyResult(caseDef, repeatIndex)
result = struct();
result.caseId = string(caseDef.id);
result.displayName = string(caseDef.displayName);
result.repeatIndex = repeatIndex;
result.Kp = caseDef.pid.Kp;
result.Ki = caseDef.pid.Ki;
result.Kd = caseDef.pid.Kd;
result.lapTime_s = NaN;
result.finishStation_m = NaN;
result.svStation_m = NaN;
result.objective = NaN;
result.penalty = NaN;
result.stopReason = "";
result.status = "invalid";
result.failureReason = "";
result.lastRunLogPath = "";
result.lastRunEndPath = "";
result.elapsedWallTime_s = NaN;
result.signals = localEmptySignals(struct());
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
if isstruct(simfileInfo)
    if isfield(simfileInfo, 'logFile')
        signals.lastRunLogPath = string(simfileInfo.logFile);
    end
    if isfield(simfileInfo, 'endFile')
        signals.lastRunEndPath = string(simfileInfo.endFile);
    end
end
end

function result = localHardFailure(result, cfg, failureReason)
result.status = "invalid";
result.objective = cfg.penalty.hardFailureObjective;
result.penalty = cfg.penalty.hardFailureObjective;
result.failureReason = failureReason;
end

function simOut = localRunSimulation(caseDef, cfg)
oldFolder = pwd;
cleanup = onCleanup(@() cd(oldFolder));
cd(cfg.modelFolder);

add_dyc_carsim_solver_path(cfg);
load_system(cfg.modelName);
in = Simulink.SimulationInput(cfg.modelName);
in = in.setModelParameter('StopTime', cfg.simulation.stopTime);
in = in.setModelParameter('FastRestart', cfg.simulation.useFastRestart);
in = in.setBlockParameter(cfg.pidBlock, 'P', num2str(caseDef.pid.Kp));
in = in.setBlockParameter(cfg.pidBlock, 'I', num2str(caseDef.pid.Ki));
in = in.setBlockParameter(cfg.pidBlock, 'D', num2str(caseDef.pid.Kd));
in = in.setBlockParameter(cfg.yawMomentModeSwitchBlock, 'sw', ...
    char(string(cfg.yawMomentPidSwitchValue)));
simOut = sim(in);
end

function tf = localShouldRejectStaleFile(cfg, beforeEvidence, afterEvidence)
tf = false;
if ~isfield(cfg.simulation, 'verifyLastRunTimestamp') || ~cfg.simulation.verifyLastRunTimestamp
    return;
end

if ~beforeEvidence.exists || ~afterEvidence.exists
    return;
end

tf = beforeEvidence.lastModifiedMs == afterEvidence.lastModifiedMs && ...
    beforeEvidence.sizeBytes == afterEvidence.sizeBytes && ...
    beforeEvidence.contentSignature == afterEvidence.contentSignature;
end

function evidence = localFileEvidence(filePath)
evidence = struct();
evidence.exists = isfile(filePath);
evidence.lastModifiedMs = NaN;
evidence.sizeBytes = NaN;
evidence.contentSignature = "";

if ~evidence.exists
    return;
end

fileObj = java.io.File(char(string(filePath)));
evidence.lastModifiedMs = double(fileObj.lastModified());
evidence.sizeBytes = double(fileObj.length());
evidence.contentSignature = localContentSignature(filePath);
end

function signature = localContentSignature(filePath)
fid = fopen(filePath, 'r');
if fid < 0
    signature = "";
    return;
end
cleanup = onCleanup(@() fclose(fid));
bytes = fread(fid, Inf, '*uint8');
sampleSize = 4096;
if numel(bytes) > 2 * sampleSize
    bytes = [bytes(1:sampleSize); bytes(end-sampleSize+1:end)];
end
signature = string(sprintf('%02X', bytes));
end
