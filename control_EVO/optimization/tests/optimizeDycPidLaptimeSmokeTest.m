classdef optimizeDycPidLaptimeSmokeTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addOptimizationPath(testCase)
            testFolder = fileparts(mfilename('fullpath'));
            optimizationFolder = fileparts(testFolder);
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(optimizationFolder));
        end
    end

    methods (Test)
        function testDryRunModeCreatesReportArtifacts(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            overrides = localFixtureOverrides(fixture.Folder, 4, @localFakeValidSimulation);

            result = optimize_dyc_pid_laptime(overrides);
            runLog = readtable(result.cfg.runLogPath, 'TextType', 'string');

            testCase.verifyEqual(height(runLog), 4);
            testCase.verifyEqual(numel(result.results), 4);
            testCase.verifyTrue(isstruct(result.best));
            testCase.verifyEqual(result.best.status, "valid");
            testCase.verifyTrue(isfinite(result.best.objective));
            testCase.verifyTrue(isfile(result.cfg.summaryPath));
            testCase.verifyTrue(isfile(result.cfg.runLogPath));
            testCase.verifyTrue(isfile(result.cfg.resultMatPath));
            testCase.verifyTrue(isfile(result.cfg.bestPidScriptPath));
            testCase.verifyTrue(contains(fileread(result.cfg.bestPidScriptPath), 'set_param'));
        end

        function testResultsDirOnlyOverrideReturnsCoherentArtifactPaths(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            overrides = localFixtureOverrides(fixture.Folder, 2, @localFakeValidSimulation);
            artifactsDir = fullfile(fixture.Folder, 'optimizer-output');
            overrides.resultsDir = artifactsDir;
            overrides = rmfield(overrides, {'runLogPath', 'summaryPath', ...
                'resultMatPath', 'bestPidScriptPath'});

            result = optimize_dyc_pid_laptime(overrides);

            testCase.verifyEqual(result.cfg.runLogPath, fullfile(artifactsDir, 'run_log.csv'));
            testCase.verifyEqual(result.cfg.summaryPath, fullfile(artifactsDir, 'summary.md'));
            testCase.verifyEqual(result.cfg.resultMatPath, fullfile(artifactsDir, 'optimization_result.mat'));
            testCase.verifyEqual(result.cfg.bestPidScriptPath, fullfile(artifactsDir, 'best_pid_set_param.m'));
            testCase.verifyTrue(isfile(result.cfg.runLogPath));
            testCase.verifyTrue(isfile(result.cfg.summaryPath));
            testCase.verifyTrue(isfile(result.cfg.resultMatPath));
            testCase.verifyTrue(isfile(result.cfg.bestPidScriptPath));
        end

        function testDryRunAllInvalidReturnsEmptyBestAndReport(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            overrides = localFixtureOverrides(fixture.Folder, 2, @localFakeInvalidSimulation);

            result = optimize_dyc_pid_laptime(overrides);

            testCase.verifyEqual(numel(result.results), 2);
            testCase.verifyEqual([result.results.status], ["invalid" "invalid"]);
            testCase.verifyTrue(isstruct(result.best));
            testCase.verifyFalse(isfield(result.best, 'Kp'));
            testCase.verifyTrue(isfile(result.cfg.summaryPath));
            testCase.verifyTrue(isfile(result.cfg.resultMatPath));
            testCase.verifyTrue(contains(fileread(result.cfg.summaryPath), 'No valid PID candidate found'));
            testCase.verifyFalse(contains(fileread(result.cfg.bestPidScriptPath), 'set_param'));
        end

        function testPreflightFailureErrorsBeforeEvaluatingCandidates(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            overrides = localFixtureOverrides(fixture.Folder, 3, @localFakeValidSimulation);
            delete(overrides.simfilePath);

            testCase.verifyError(@() optimize_dyc_pid_laptime(overrides), ...
                'DYC:PIDOptimization:PreflightFailed');
            testCase.verifyFalse(isfile(overrides.runLogPath));
        end

        function testAutocrossEntryPointAcceptsExternalControlMetric(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            overrides = localFixtureOverrides(fixture.Folder, 2, @localFakeAutocrossSimulation);
            info = parse_dyc_simfile(overrides.simfilePath);
            localWriteExternalControlLog(info.logFile, info.endFile, 20.374);

            result = optimize_dyc_pid_autocross_laptime(overrides);

            testCase.verifyEqual(result.cfg.metric.primaryMetric, 'carsim_autocross_stop_time');
            testCase.verifyEqual(result.cfg.simulation.stopTime, '120');
            testCase.verifyEqual(numel(result.results), 2);
            testCase.verifyEqual([result.results.status], ["valid" "valid"]);
            testCase.verifyEqual(result.best.status, "valid");
            testCase.verifyTrue(contains(fileread(result.cfg.summaryPath), 'carsim_autocross_stop_time'));
        end
    end
end

function overrides = localFixtureOverrides(folder, maxEvaluations, simulateFcn)
simfilePath = fullfile(folder, 'simfile.sim');
resultsDir = fullfile(folder, 'Results', 'Run_abc');
mkdir(resultsDir);
writelines([
    "SIMFILE"
    "SET_MACRO $(ROOT_FILE_NAME)$ Run_abc"
    "SET_MACRO $(OUTPUT_PATH)$ " + fullfile(folder, 'Results')
    "SET_MACRO $(WORK_DIR)$ " + folder + filesep
    "SET_MACRO $(OUTPUT_FILE_PREFIX)$ $(WORK_DIR)$Results\$(ROOT_FILE_NAME)$\LastRun"
    "END"
], simfilePath);
localWriteStationStopLog(fullfile(resultsDir, 'LastRun_log.txt'), fullfile(resultsDir, 'LastRun_end.par'), 21.6025);

overrides = struct();
overrides.simfilePath = simfilePath;
overrides.resultsDir = folder;
overrides.runLogPath = fullfile(folder, 'run_log.csv');
overrides.summaryPath = fullfile(folder, 'summary.md');
overrides.resultMatPath = fullfile(folder, 'optimization_result.mat');
overrides.bestPidScriptPath = fullfile(folder, 'best_pid_set_param.m');
overrides.testMode = true;
overrides.optimizer = struct('maxObjectiveEvaluations', maxEvaluations, ...
    'initialRandomEvaluations', 1);
overrides.simulation = struct('simulateFcn', simulateFcn);
end

function localFakeValidSimulation(cfg, candidate)
info = parse_dyc_simfile(cfg.simfilePath);
lapTime = 21.0 - 0.00001 * candidate.Kp + 0.0001 * candidate.Ki + 0.0002 * candidate.Kd;
localWriteStationStopLog(info.logFile, info.endFile, lapTime);
end

function localFakeInvalidSimulation(cfg, ~)
info = parse_dyc_simfile(cfg.simfilePath);
writelines([
    "Run started: VS output file = temp.vsb"
    "Run stopped at t = 30.0000. Time limit reached."
], info.logFile);
writelines("SV_STATION 200.0 ; m ! Station", info.endFile);
end

function localFakeAutocrossSimulation(cfg, candidate)
info = parse_dyc_simfile(cfg.simfilePath);
lapTime = 20.5 - 0.00001 * candidate.Kp + 0.0001 * candidate.Ki + 0.0002 * candidate.Kd;
localWriteAutocrossEventLog(info.logFile, info.endFile, lapTime);
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

function localWriteExternalControlLog(logFile, endFile, lapTime)
logFolder = fileparts(logFile);
if ~isfolder(logFolder)
    mkdir(logFolder);
end
writelines([
    "Run started: VS output file = temp.vsb"
    "Run stopped at t = " + string(lapTime) + ". External control (manual or external model)"
], logFile);
writelines("TSTART " + string(lapTime) + " ; s ! Starting time for the simulation clock", endFile);
end

function localWriteAutocrossEventLog(logFile, endFile, lapTime)
logFolder = fileparts(logFile);
if ~isfolder(logFolder)
    mkdir(logFolder);
end
writelines([
    "Run started: VS output file = temp.vsb"
    "Event #1 occurred at t = " + string(lapTime) + ": SV_N_START_CROSS >= 2"
    "Run stopped at t = " + string(lapTime) + ". VS Command STOP_RUN_NOW End event triggered"
    "Used Dataset: Events; End Events"
], logFile);
writelines("SV_STATION 1234.5 ; m ! Station", endFile);
end
