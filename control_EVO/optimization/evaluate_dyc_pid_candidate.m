function result = evaluate_dyc_pid_candidate(candidate, cfg, iteration)
%EVALUATE_DYC_PID_CANDIDATE Run and score one PID candidate.

ticStart = tic;
result = localEmptyResult(candidate, iteration);

try
    simfileInfo = parse_dyc_simfile(cfg.simfilePath);
    if ~simfileInfo.isValid
        result.objective = cfg.penalty.hardFailureObjective;
        result.penalty = cfg.penalty.hardFailureObjective;
        result.failureReason = "simfile parse failed: " + simfileInfo.failureReason;
        result.elapsedWallTime_s = toc(ticStart);
        append_dyc_optimization_checkpoint(cfg, result);
        return;
    end

    beforeLogEvidence = localLastRunLogEvidence(simfileInfo.logFile);

    if isempty(cfg.simulation.simulateFcn)
        localRunSimulation(candidate, cfg);
    else
        cfg.simulation.simulateFcn(cfg, candidate);
    end

    afterLogEvidence = localLastRunLogEvidence(simfileInfo.logFile);
    if localShouldRejectStaleLastRun(cfg, beforeLogEvidence, afterLogEvidence)
        result.objective = cfg.penalty.hardFailureObjective;
        result.penalty = cfg.penalty.hardFailureObjective;
        result.failureReason = "LastRun_log.txt did not refresh after simulation; refusing to score stale CarSim log";
        result.lastRunLogPath = string(simfileInfo.logFile);
        result.elapsedWallTime_s = toc(ticStart);
        append_dyc_optimization_checkpoint(cfg, result);
        return;
    end

    metric = extract_dyc_laptime_metric(simfileInfo.logFile, simfileInfo.endFile, cfg);
    result.lapTime_s = metric.lapTime_s;
    result.finishStation_m = metric.finishStation_m;
    result.objective = metric.objective;
    result.penalty = metric.penalty;
    result.stopReason = metric.stopReason;
    result.status = metric.status;
    result.failureReason = metric.failureReason;
    result.lastRunLogPath = string(simfileInfo.logFile);
catch err
    result.status = "invalid";
    result.objective = cfg.penalty.hardFailureObjective;
    result.penalty = cfg.penalty.hardFailureObjective;
    result.failureReason = string(err.message);
end

result.elapsedWallTime_s = toc(ticStart);
append_dyc_optimization_checkpoint(cfg, result);
end

function result = localEmptyResult(candidate, iteration)
result = struct();
result.iteration = iteration;
result.Kp = candidate.Kp;
result.Ki = candidate.Ki;
result.Kd = candidate.Kd;
result.lapTime_s = NaN;
result.finishStation_m = NaN;
result.objective = NaN;
result.penalty = NaN;
result.stopReason = "";
result.status = "invalid";
result.failureReason = "";
result.lastRunLogPath = "";
result.elapsedWallTime_s = NaN;
end

function localRunSimulation(candidate, cfg)
oldFolder = pwd;
cleanup = onCleanup(@() cd(oldFolder));
cd(cfg.modelFolder);

add_dyc_carsim_solver_path(cfg);
load_system(cfg.modelName);
in = Simulink.SimulationInput(cfg.modelName);
in = in.setModelParameter('StopTime', cfg.simulation.stopTime);
in = in.setModelParameter('FastRestart', cfg.simulation.useFastRestart);
in = in.setBlockParameter(cfg.pidBlock, 'P', num2str(candidate.Kp));
in = in.setBlockParameter(cfg.pidBlock, 'I', num2str(candidate.Ki));
in = in.setBlockParameter(cfg.pidBlock, 'D', num2str(candidate.Kd));
in = in.setBlockParameter(cfg.yawMomentModeSwitchBlock, 'sw', ...
    char(string(cfg.yawMomentPidSwitchValue)));
sim(in);
end

function tf = localShouldRejectStaleLastRun(cfg, beforeLogEvidence, afterLogEvidence)
tf = false;
if ~isfield(cfg.simulation, 'verifyLastRunTimestamp') || ~cfg.simulation.verifyLastRunTimestamp
    return;
end

if ~beforeLogEvidence.exists || ~afterLogEvidence.exists
    return;
end

tf = beforeLogEvidence.lastModifiedMs == afterLogEvidence.lastModifiedMs && ...
    beforeLogEvidence.sizeBytes == afterLogEvidence.sizeBytes && ...
    beforeLogEvidence.contentSignature == afterLogEvidence.contentSignature;
end

function evidence = localLastRunLogEvidence(logFile)
evidence = struct();
evidence.exists = isfile(logFile);
evidence.lastModifiedMs = NaN;
evidence.sizeBytes = NaN;
evidence.contentSignature = "";

if ~evidence.exists
    return;
end

fileObj = java.io.File(char(string(logFile)));
evidence.lastModifiedMs = double(fileObj.lastModified());
evidence.sizeBytes = double(fileObj.length());
evidence.contentSignature = localContentSignature(logFile);
end

function signature = localContentSignature(logFile)
fid = fopen(logFile, 'r');
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
