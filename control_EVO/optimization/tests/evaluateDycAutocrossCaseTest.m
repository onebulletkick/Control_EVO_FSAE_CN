classdef evaluateDycAutocrossCaseTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addOptimizationPath(testCase)
            testFolder = fileparts(mfilename('fullpath'));
            optimizationFolder = fileparts(testFolder);
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(optimizationFolder));
        end
    end

    methods (Test)
        function testEvaluateCaseUsesAutocrossMetricAndSignals(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            [simfilePath, resultsDir] = localWriteFixtureSimfile(fixture.Folder);
            overrides = localBaseOverrides(fixture.Folder, simfilePath, ...
                @(cfg, caseDef, repeatIndex) localFakeAutocrossSimulation(cfg, caseDef, repeatIndex, 83.25), true);
            cfg = dyc_autocross_comparison_config(overrides);
            caseDef = cfg.cases(2);

            results = evaluate_dyc_autocross_case(caseDef, cfg);

            testCase.verifyEqual(numel(results), 1);
            testCase.verifyEqual(results.status, "valid");
            testCase.verifyEqual(results.caseId, "dyc_on");
            testCase.verifyEqual(results.displayName, "当前 PID DYC");
            testCase.verifyEqual(results.repeatIndex, 1);
            testCase.verifyEqual(results.Kp, 6000);
            testCase.verifyEqual(results.Ki, 200);
            testCase.verifyEqual(results.Kd, 0);
            testCase.verifyEqual(results.lapTime_s, 83.25, 'AbsTol', 1e-9);
            testCase.verifyEqual(results.stopReason, "VS Command STOP_RUN_NOW End event triggered");
            testCase.verifyEqual(results.lastRunLogPath, string(fullfile(resultsDir, 'LastRun_log.txt')));
            testCase.verifyEqual(results.lastRunEndPath, string(fullfile(resultsDir, 'LastRun_end.par')));
            testCase.verifyEqual(results.signals.source, "fake");
            testCase.verifyEqual(results.signals.yawRate_radps, [0 0.12 0.18]);
        end

        function testPreExistingLastRunLogMustRefresh(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            [simfilePath, resultsDir] = localWriteFixtureSimfile(fixture.Folder);
            localWriteAutocrossEventLog(fullfile(resultsDir, 'LastRun_log.txt'), ...
                fullfile(resultsDir, 'LastRun_end.par'), 80.0);
            overrides = localBaseOverrides(fixture.Folder, simfilePath, @(~, ~, ~) [], true);
            cfg = dyc_autocross_comparison_config(overrides);

            results = evaluate_dyc_autocross_case(cfg.cases(2), cfg);

            testCase.verifyEqual(results.status, "invalid");
            testCase.verifyEqual(results.objective, cfg.penalty.hardFailureObjective);
            testCase.verifyEqual(results.penalty, cfg.penalty.hardFailureObjective);
            testCase.verifyTrue(contains(results.failureReason, "did not refresh"));
        end

        function testRepeatCountRunsCaseMultipleTimes(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            [simfilePath, ~] = localWriteFixtureSimfile(fixture.Folder);
            overrides = localBaseOverrides(fixture.Folder, simfilePath, @localFakeRepeatSimulation, false);
            overrides.repeatCount = 2;
            cfg = dyc_autocross_comparison_config(overrides);

            results = evaluate_dyc_autocross_case(cfg.cases(2), cfg);

            testCase.verifyEqual(numel(results), 2);
            testCase.verifyEqual([results.repeatIndex], [1 2]);
            testCase.verifyEqual([results.lapTime_s], [81.5 82.5], 'AbsTol', 1e-9);
            testCase.verifyEqual([results.status], ["valid" "valid"]);
        end

        function testCollectSignalsUnsupportedOutputReturnsUnavailable(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            [simfilePath, ~] = localWriteFixtureSimfile(fixture.Folder);
            simfileInfo = parse_dyc_simfile(simfilePath);
            cfg = dyc_autocross_comparison_config(struct('simfilePath', simfilePath, ...
                'resultsDir', fixture.Folder, 'testMode', true));

            signals = collect_dyc_autocross_signals(42, simfileInfo, cfg);

            testCase.verifyFalse(signals.available);
            testCase.verifyNotEmpty(signals.failureReason);
            testCase.verifyEqual(signals.lastRunLogPath, string(simfileInfo.logFile));
            testCase.verifyEqual(signals.lastRunEndPath, string(simfileInfo.endFile));
        end

        function testCollectSignalsUsesTimeseriesTimeFromDataset(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            [simfilePath, ~] = localWriteFixtureSimfile(fixture.Folder);
            simfileInfo = parse_dyc_simfile(simfilePath);
            cfg = dyc_autocross_comparison_config(struct('simfilePath', simfilePath, ...
                'resultsDir', fixture.Folder, 'testMode', true));
            simOut = localDatasetSimulationOutput();

            signals = collect_dyc_autocross_signals(simOut, simfileInfo, cfg);

            testCase.verifyTrue(signals.available);
            testCase.verifyEqual(signals.source, "SimulationOutput");
            testCase.verifyEqual(signals.time_s, [0 0.5 1.0], 'AbsTol', 1e-12);
            testCase.verifyEqual(signals.mz_Nm, [0 80 120], 'AbsTol', 1e-12);
            testCase.verifyEqual(signals.speed_mps, [12 14 15], 'AbsTol', 1e-12);
            testCase.verifyEqual(signals.yawRate_radps, [0.1 0.2 0.3], 'AbsTol', 1e-12);
        end

        function testRefusesMixedFreshLogAndStaleEndFile(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            [simfilePath, resultsDir] = localWriteFixtureSimfile(fixture.Folder);
            logFile = fullfile(resultsDir, 'LastRun_log.txt');
            endFile = fullfile(resultsDir, 'LastRun_end.par');
            localWriteAutocrossEventLog(logFile, endFile, 80.0);
            overrides = localBaseOverrides(fixture.Folder, simfilePath, ...
                @(cfg, ~, ~) localRefreshLogOnly(cfg, 83.25), true);
            cfg = dyc_autocross_comparison_config(overrides);

            results = evaluate_dyc_autocross_case(cfg.cases(2), cfg);

            testCase.verifyEqual(results.status, "invalid");
            testCase.verifyEqual(results.objective, cfg.penalty.hardFailureObjective);
            testCase.verifyTrue(contains(results.failureReason, "LastRun_end.par"));
            testCase.verifyTrue(contains(results.failureReason, "did not refresh"));
        end

        function testMissingLastRunEndAfterSimulationIsHardFailure(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            [simfilePath, ~] = localWriteFixtureSimfile(fixture.Folder);
            overrides = localBaseOverrides(fixture.Folder, simfilePath, ...
                @(cfg, ~, ~) localWriteFreshLogWithoutEnd(cfg, 83.25), true);
            cfg = dyc_autocross_comparison_config(overrides);

            results = evaluate_dyc_autocross_case(cfg.cases(2), cfg);

            testCase.verifyEqual(results.status, "invalid");
            testCase.verifyEqual(results.objective, cfg.penalty.hardFailureObjective);
            testCase.verifyEqual(results.penalty, cfg.penalty.hardFailureObjective);
            testCase.verifyTrue(contains(results.failureReason, "LastRun_end.par"));
            testCase.verifyTrue(contains(results.failureReason, "not found"));
        end

        function testRequiredStopReasonMismatchIsHardFailure(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            [simfilePath, ~] = localWriteFixtureSimfile(fixture.Folder);
            overrides = localBaseOverrides(fixture.Folder, simfilePath, ...
                @(cfg, ~, ~) localFakeExternalControlStopSimulation(cfg, 83.25), true);
            cfg = dyc_autocross_comparison_config(overrides);

            results = evaluate_dyc_autocross_case(cfg.cases(2), cfg);

            testCase.verifyEqual(results.status, "invalid");
            testCase.verifyTrue(isnan(results.lapTime_s));
            testCase.verifyEqual(results.objective, cfg.penalty.hardFailureObjective);
            testCase.verifyEqual(results.penalty, cfg.penalty.hardFailureObjective);
            testCase.verifyTrue(contains(results.failureReason, cfg.metric.requiredStopReason));
            testCase.verifyTrue(contains(results.failureReason, "External control"));
        end

        function testStructSignalsWithoutStandardDataAreUnavailable(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            [simfilePath, ~] = localWriteFixtureSimfile(fixture.Folder);
            simfileInfo = parse_dyc_simfile(simfilePath);
            cfg = dyc_autocross_comparison_config(struct('simfilePath', simfilePath, ...
                'resultsDir', fixture.Folder, 'testMode', true));

            signals = collect_dyc_autocross_signals(struct('signals', struct('source', "empty")), ...
                simfileInfo, cfg);

            testCase.verifyFalse(signals.available);
            testCase.verifyEqual(signals.source, "empty");
            testCase.verifyNotEmpty(signals.failureReason);
        end
    end
end

function overrides = localBaseOverrides(folder, simfilePath, simulateFcn, verifyLastRunTimestamp)
overrides = struct();
overrides.simfilePath = simfilePath;
overrides.resultsDir = folder;
overrides.testMode = true;
overrides.simulation = struct('simulateFcn', simulateFcn, ...
    'verifyLastRunTimestamp', verifyLastRunTimestamp);
end

function [simfilePath, resultsDir] = localWriteFixtureSimfile(folder)
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
end

function simOut = localFakeAutocrossSimulation(cfg, ~, ~, lapTime)
info = parse_dyc_simfile(cfg.simfilePath);
localWriteAutocrossEventLog(info.logFile, info.endFile, lapTime);

signals = struct();
signals.source = "fake";
signals.time_s = [0 1 2];
signals.speed_mps = [12 14 15];
signals.yawRate_radps = [0 0.12 0.18];
signals.yawRateTarget_radps = [0 0.10 0.15];
signals.latVeh_m = [0 0.2 0.4];
signals.latTarget_m = [0 0.1 0.3];
signals.ay_mps2 = [0 1.2 1.8];
signals.mz_Nm = [0 80 120];
simOut = struct('signals', signals);
end

function simOut = localFakeRepeatSimulation(cfg, ~, repeatIndex)
lapTime = 80.5 + repeatIndex;
simOut = localFakeAutocrossSimulation(cfg, [], repeatIndex, lapTime);
end

function simOut = localDatasetSimulationOutput()
time = [0 0.5 1.0]';
logsout = Simulink.SimulationData.Dataset;
logsout = logsout.addElement(timeseries([0 80 120]', time), 'Mz_selected');
logsout = logsout.addElement(timeseries([12 14 15]', time), 'Vx');
logsout = logsout.addElement(timeseries([0.1 0.2 0.3]', time), 'AVz');
simOut = Simulink.SimulationOutput;
simOut.logsout = logsout;
end

function simOut = localRefreshLogOnly(cfg, lapTime)
info = parse_dyc_simfile(cfg.simfilePath);
writelines([
    "Run started: VS output file = fresh.vsb"
    "Run stopped at t = " + string(lapTime) + ". VS Command STOP_RUN_NOW End event triggered"
], info.logFile);
signals = struct();
signals.speed_mps = [1 2 3];
simOut = struct('signals', signals);
end

function simOut = localWriteFreshLogWithoutEnd(cfg, lapTime)
info = parse_dyc_simfile(cfg.simfilePath);
writelines([
    "Run started: VS output file = missing_end.vsb"
    "Run stopped at t = " + string(lapTime) + ". VS Command STOP_RUN_NOW End event triggered"
], info.logFile);
signals = struct();
signals.speed_mps = [1 2 3];
simOut = struct('signals', signals);
end

function simOut = localFakeExternalControlStopSimulation(cfg, lapTime)
info = parse_dyc_simfile(cfg.simfilePath);
writelines([
    "Run started: VS output file = external_stop.vsb"
    "Run stopped at t = " + string(lapTime) + ". External control (manual or external model)"
], info.logFile);
writelines("SV_STATION 1234.5 ; m ! Station", info.endFile);
signals = struct();
signals.speed_mps = [1 2 3];
simOut = struct('signals', signals);
end

function localWriteAutocrossEventLog(logFile, endFile, lapTime)
logFolder = fileparts(logFile);
if ~isfolder(logFolder)
    mkdir(logFolder);
end
writelines([
    "Run started: VS output file = temp.vsb"
    "Run stopped at t = " + string(lapTime) + ". VS Command STOP_RUN_NOW End event triggered"
], logFile);
writelines("SV_STATION 1234.5 ; m ! Station", endFile);
end
