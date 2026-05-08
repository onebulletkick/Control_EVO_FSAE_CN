classdef evaluateDycPidCandidateTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addOptimizationPath(testCase)
            testFolder = fileparts(mfilename('fullpath'));
            optimizationFolder = fileparts(testFolder);
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(optimizationFolder));
        end
    end

    methods (Test)
        function testEvaluateCandidateUsesMetricAndCheckpoint(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            simfilePath = fullfile(fixture.Folder, 'simfile.sim');
            resultsDir = fullfile(fixture.Folder, 'Results', 'Run_abc');
            mkdir(resultsDir);
            writelines([
                "SIMFILE"
                "SET_MACRO $(ROOT_FILE_NAME)$ Run_abc"
                "SET_MACRO $(OUTPUT_PATH)$ " + fullfile(fixture.Folder, 'Results')
                "SET_MACRO $(WORK_DIR)$ " + fixture.Folder + filesep
                "SET_MACRO $(OUTPUT_FILE_PREFIX)$ $(WORK_DIR)$Results\$(ROOT_FILE_NAME)$\LastRun"
                "END"
            ], simfilePath);

            overrides = struct();
            overrides.simfilePath = simfilePath;
            overrides.resultsDir = fixture.Folder;
            overrides.runLogPath = fullfile(fixture.Folder, 'run_log.csv');
            overrides.testMode = true;
            overrides.simulation = struct('simulateFcn', @(cfg, candidate) localFakeSimulation(cfg, candidate));
            cfg = dyc_pid_optimization_config(overrides);
            candidate = struct('Kp', 7000, 'Ki', 250, 'Kd', 20);

            result = evaluate_dyc_pid_candidate(candidate, cfg, 3);

            testCase.verifyEqual(result.status, "valid");
            testCase.verifyEqual(result.iteration, 3);
            testCase.verifyEqual(result.Kp, 7000);
            testCase.verifyEqual(result.lapTime_s, 20.5, 'AbsTol', 1e-9);
            testCase.verifyTrue(isfile(cfg.runLogPath));
        end

        function testMissingSimfileUsesHardFailureObjective(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            overrides = struct();
            overrides.simfilePath = fullfile(fixture.Folder, 'missing_simfile.sim');
            overrides.resultsDir = fixture.Folder;
            overrides.runLogPath = fullfile(fixture.Folder, 'run_log.csv');
            overrides.testMode = true;
            cfg = dyc_pid_optimization_config(overrides);
            candidate = struct('Kp', 7000, 'Ki', 250, 'Kd', 20);

            result = evaluate_dyc_pid_candidate(candidate, cfg, 4);
            runLog = readtable(cfg.runLogPath);

            testCase.verifyEqual(result.status, "invalid");
            testCase.verifyEqual(result.objective, cfg.penalty.hardFailureObjective);
            testCase.verifyEqual(result.penalty, cfg.penalty.hardFailureObjective);
            testCase.verifyTrue(contains(result.failureReason, "simfile parse failed") || ...
                contains(result.failureReason, "simfile"));
            testCase.verifyTrue(isfinite(runLog.objective(1)));
            testCase.verifyEqual(runLog.objective(1), cfg.penalty.hardFailureObjective);
        end

        function testPreExistingLastRunLogMustRefreshBeforeScoring(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            simfilePath = fullfile(fixture.Folder, 'simfile.sim');
            resultsDir = fullfile(fixture.Folder, 'Results', 'Run_abc');
            mkdir(resultsDir);
            writelines([
                "SIMFILE"
                "SET_MACRO $(ROOT_FILE_NAME)$ Run_abc"
                "SET_MACRO $(OUTPUT_PATH)$ " + fullfile(fixture.Folder, 'Results')
                "SET_MACRO $(WORK_DIR)$ " + fixture.Folder + filesep
                "SET_MACRO $(OUTPUT_FILE_PREFIX)$ $(WORK_DIR)$Results\$(ROOT_FILE_NAME)$\LastRun"
                "END"
            ], simfilePath);
            writelines([
                "Run started: VS output file = stale.vsb"
                "Run stopped at t = 19.5. Station limit reached: driver station = 245.002"
            ], fullfile(resultsDir, 'LastRun_log.txt'));
            writelines("SV_STATION 245.0016584 ; m ! Station", fullfile(resultsDir, 'LastRun_end.par'));

            overrides = struct();
            overrides.simfilePath = simfilePath;
            overrides.resultsDir = fixture.Folder;
            overrides.runLogPath = fullfile(fixture.Folder, 'run_log.csv');
            overrides.testMode = true;
            overrides.simulation = struct('simulateFcn', @(~, ~) []);
            cfg = dyc_pid_optimization_config(overrides);
            candidate = struct('Kp', 7000, 'Ki', 250, 'Kd', 20);

            result = evaluate_dyc_pid_candidate(candidate, cfg, 5);

            testCase.verifyEqual(result.status, "invalid");
            testCase.verifyEqual(result.objective, cfg.penalty.hardFailureObjective);
            testCase.verifyTrue(contains(result.failureReason, "did not refresh"));
        end

        function testSimulationInputAppliesPidParametersOneAtATime(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            mdl = localCreateMinimalPidModel(testCase);
            [simfilePath, resultsDir] = localWriteFixtureSimfile(fixture.Folder);
            localWriteStationStopLog(fullfile(resultsDir, 'LastRun_log.txt'), ...
                fullfile(resultsDir, 'LastRun_end.par'), 20.25);

            overrides = struct();
            overrides.modelFolder = pwd;
            overrides.modelName = mdl;
            overrides.modelPath = fullfile(pwd, [mdl '.slx']);
            overrides.pidBlock = [mdl '/PID'];
            overrides.yawMomentModeSwitchBlock = [mdl '/YawMomentControlMode'];
            overrides.simfilePath = simfilePath;
            overrides.resultsDir = fixture.Folder;
            overrides.runLogPath = fullfile(fixture.Folder, 'run_log.csv');
            overrides.testMode = true;
            overrides.simulation = struct('simulateFcn', [], ...
                'verifyLastRunTimestamp', false, 'stopTime', '0.1', ...
                'useFastRestart', 'off');
            cfg = dyc_pid_optimization_config(overrides);
            candidate = struct('Kp', 7, 'Ki', 2, 'Kd', 0.5);

            result = evaluate_dyc_pid_candidate(candidate, cfg, 6);

            testCase.verifyEqual(result.status, "valid");
            testCase.verifyEqual(result.lapTime_s, 20.25, 'AbsTol', 1e-9);
            testCase.verifyNotEqual(string(get_param(cfg.pidBlock, 'P')), "7");
        end

        function testSimulationAddsCarSimSolverPathFromSimfile(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            mdl = localCreateMinimalPidModel(testCase);
            solverFolder = fullfile(fixture.Folder, 'Programs', 'solvers', 'Matlab');
            mkdir(solverFolder);
            [simfilePath, resultsDir] = localWriteFixtureSimfile(fixture.Folder, fixture.Folder);
            localWriteStationStopLog(fullfile(resultsDir, 'LastRun_log.txt'), ...
                fullfile(resultsDir, 'LastRun_end.par'), 20.25);
            testCase.addTeardown(@() localRemovePath(solverFolder));
            localRemovePath(solverFolder);

            overrides = struct();
            overrides.modelFolder = pwd;
            overrides.modelName = mdl;
            overrides.modelPath = fullfile(pwd, [mdl '.slx']);
            overrides.pidBlock = [mdl '/PID'];
            overrides.yawMomentModeSwitchBlock = [mdl '/YawMomentControlMode'];
            overrides.simfilePath = simfilePath;
            overrides.resultsDir = fixture.Folder;
            overrides.runLogPath = fullfile(fixture.Folder, 'run_log.csv');
            overrides.testMode = true;
            overrides.simulation = struct('simulateFcn', [], ...
                'verifyLastRunTimestamp', false, 'stopTime', '0.1', ...
                'useFastRestart', 'off');
            cfg = dyc_pid_optimization_config(overrides);
            candidate = struct('Kp', 7, 'Ki', 2, 'Kd', 0.5);

            result = evaluate_dyc_pid_candidate(candidate, cfg, 7);

            testCase.verifyEqual(result.status, "valid");
            testCase.verifyTrue(contains(path, solverFolder));
        end
    end
end

function [simfilePath, resultsDir] = localWriteFixtureSimfile(folder, progDir)
if nargin < 2
    progDir = "";
end
simfilePath = fullfile(folder, 'simfile.sim');
resultsDir = fullfile(folder, 'Results', 'Run_abc');
mkdir(resultsDir);
lines = [
    "SIMFILE"
    "SET_MACRO $(ROOT_FILE_NAME)$ Run_abc"
    "SET_MACRO $(OUTPUT_PATH)$ " + fullfile(folder, 'Results')
    "SET_MACRO $(WORK_DIR)$ " + folder + filesep
    "SET_MACRO $(OUTPUT_FILE_PREFIX)$ $(WORK_DIR)$Results\$(ROOT_FILE_NAME)$\LastRun"
];
if strlength(string(progDir)) > 0
    lines(end+1, 1) = "PROGDIR " + string(progDir) + filesep;
end
lines(end+1, 1) = "END";
writelines(lines, simfilePath);
end

function localFakeSimulation(cfg, candidate)
info = parse_dyc_simfile(cfg.simfilePath);
logFolder = fileparts(info.logFile);
if ~isfolder(logFolder)
    mkdir(logFolder);
end
lapTime = 20.5 + 0.0001 * abs(candidate.Kp - 7000);
writelines([
    "Run started: VS output file = temp.vsb"
    "Run stopped at t = " + string(lapTime) + ". Station limit reached: driver station = 245.002"
], info.logFile);
writelines("SV_STATION 245.0016584 ; m ! Station", info.endFile);
end

function mdl = localCreateMinimalPidModel(testCase)
mdl = "tmpDycPidCandidate" + string(char(java.util.UUID.randomUUID));
mdl = char(replace(mdl, "-", "_"));
new_system(mdl);
testCase.addTeardown(@() localCloseModel(mdl));

add_block('simulink/Sources/Constant', [mdl '/Input'], 'Value', '1');
add_block('simulink/Continuous/PID Controller', [mdl '/PID']);
add_block('simulink/Sinks/Terminator', [mdl '/PidTerminator']);
add_line(mdl, 'Input/1', 'PID/1');
add_line(mdl, 'PID/1', 'PidTerminator/1');

add_block('simulink/Sources/Constant', [mdl '/PidMode'], 'Value', '1');
add_block('simulink/Sources/Constant', [mdl '/MpcMode'], 'Value', '2');
add_block('simulink/Signal Routing/Manual Switch', [mdl '/YawMomentControlMode']);
add_block('simulink/Sinks/Terminator', [mdl '/ModeTerminator']);
add_line(mdl, 'PidMode/1', 'YawMomentControlMode/1');
add_line(mdl, 'MpcMode/1', 'YawMomentControlMode/2');
add_line(mdl, 'YawMomentControlMode/1', 'ModeTerminator/1');
set_param(mdl, 'StopTime', '0.1');
end

function localCloseModel(mdl)
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
end

function localRemovePath(folder)
if contains(path, folder)
    rmpath(folder);
end
end

function localWriteStationStopLog(logFile, endFile, lapTime)
logFolder = fileparts(logFile);
if ~isfolder(logFolder)
    mkdir(logFolder);
end
writelines([
    "Run started: VS output file = temp.vsb"
    "Run stopped at t = " + string(lapTime) + ". Station limit reached: driver station = 245.002"
], logFile);
writelines("SV_STATION 245.0016584 ; m ! Station", endFile);
end
