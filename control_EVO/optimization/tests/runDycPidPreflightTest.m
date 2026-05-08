classdef runDycPidPreflightTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addOptimizationPath(testCase)
            testFolder = fileparts(mfilename('fullpath'));
            optimizationFolder = fileparts(testFolder);
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(optimizationFolder));
        end
    end

    methods (Test)
        function testPreflightRejectsMissingSimfile(testCase)
            cfg = dyc_pid_optimization_config(struct('simfilePath', 'Z:\missing\simfile.sim', ...
                'testMode', true));

            report = run_dyc_pid_preflight(cfg);

            testCase.verifyFalse(report.ok);
            testCase.verifyTrue(any(contains(report.failures, "simfile")));
        end

        function testPreflightAcceptsFixtureBaseline(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            simfilePath = localWriteValidBaselineFixture(fixture.Folder);

            cfg = dyc_pid_optimization_config(struct('simfilePath', simfilePath, ...
                'testMode', true));
            report = run_dyc_pid_preflight(cfg);

            testCase.verifyTrue(report.ok);
            testCase.verifyEqual(report.metric.lapTime_s, 21.6025, 'AbsTol', 1e-9);
        end

        function testPreflightReportsMissingModelInNonTestMode(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            simfilePath = localWriteValidBaselineFixture(fixture.Folder);
            cfg = dyc_pid_optimization_config(struct('simfilePath', simfilePath, ...
                'modelPath', fullfile(fixture.Folder, 'missing_model.slx'), ...
                'preflight', struct('bayesoptAvailableFcn', @() true), ...
                'testMode', false));

            report = run_dyc_pid_preflight(cfg);

            testCase.verifyFalse(report.ok);
            testCase.verifyTrue(any(contains(report.failures, "model file not found")));
            testCase.verifyEqual(report.metric.status, "valid");
        end

        function testPreflightReportsMissingBayesoptInNonTestMode(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            simfilePath = localWriteValidBaselineFixture(fixture.Folder);
            cfg = dyc_pid_optimization_config(struct('simfilePath', simfilePath, ...
                'modelPath', simfilePath, ...
                'preflight', struct('bayesoptAvailableFcn', @() false), ...
                'testMode', false));

            report = run_dyc_pid_preflight(cfg);

            testCase.verifyFalse(report.ok);
            testCase.verifyTrue(any(contains(report.failures, "bayesopt")));
            testCase.verifyEqual(report.metric.status, "valid");
        end

        function testPreflightReportsInvalidSimfileParse(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            simfilePath = fullfile(fixture.Folder, 'simfile.sim');
            writelines(["SIMFILE"; "END"], simfilePath);
            cfg = dyc_pid_optimization_config(struct('simfilePath', simfilePath, ...
                'testMode', true));

            report = run_dyc_pid_preflight(cfg);

            testCase.verifyFalse(report.ok);
            testCase.verifyTrue(any(contains(report.failures, "simfile parse failed")));
            testCase.verifyNotEmpty(report.simfile.failureReason);
        end

        function testPreflightReportsInvalidLaptime(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            simfilePath = localWriteValidBaselineFixture(fixture.Folder, ...
                ["Run started: VS output file = temp.vsb"; "Run stopped at t = 30.0000. Time limit reached."]);
            cfg = dyc_pid_optimization_config(struct('simfilePath', simfilePath, ...
                'testMode', true));

            report = run_dyc_pid_preflight(cfg);

            testCase.verifyFalse(report.ok);
            testCase.verifyTrue(any(contains(report.failures, "baseline laptime invalid")));
            testCase.verifyEqual(report.metric.status, "invalid");
            testCase.verifyNotEmpty(report.metric.failureReason);
        end

        function testPreflightRejectsMissingPidBlockFromModelCheck(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            simfilePath = localWriteValidBaselineFixture(fixture.Folder);
            cfg = dyc_pid_optimization_config(struct('simfilePath', simfilePath, ...
                'preflight', struct('modelInfoFcn', @localMissingPidBlockInfo), ...
                'testMode', true));

            report = run_dyc_pid_preflight(cfg);

            testCase.verifyFalse(report.ok);
            testCase.verifyTrue(any(contains(report.failures, "PID block")));
            testCase.verifyEqual(report.metric.status, "valid");
        end

        function testPreflightRejectsMissingYawMomentSwitchFromModelCheck(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            simfilePath = localWriteValidBaselineFixture(fixture.Folder);
            cfg = dyc_pid_optimization_config(struct('simfilePath', simfilePath, ...
                'preflight', struct('modelInfoFcn', @localMissingYawMomentSwitchInfo), ...
                'testMode', true));

            report = run_dyc_pid_preflight(cfg);

            testCase.verifyFalse(report.ok);
            testCase.verifyTrue(any(contains(report.failures, "YawMomentControlMode")));
            testCase.verifyEqual(report.metric.status, "valid");
        end

        function testPreflightWarnsWhenSavedSwitchIsNotPid(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            simfilePath = localWriteValidBaselineFixture(fixture.Folder);
            cfg = dyc_pid_optimization_config(struct('simfilePath', simfilePath, ...
                'preflight', struct('modelInfoFcn', @localSavedSwitchNotPidInfo), ...
                'testMode', true));

            report = run_dyc_pid_preflight(cfg);

            testCase.verifyTrue(report.ok);
            testCase.verifyTrue(any(contains(report.warnings, "will force PID")));
            testCase.verifyEqual(report.metric.status, "valid");
        end
    end
end

function simfilePath = localWriteValidBaselineFixture(folder, logLines)
if nargin < 2
    logLines = [
        "Run started: VS output file = temp.vsb"
        "Run stopped at t = 21.6025. Station limit reached: driver station = 245.002"
    ];
end

simfilePath = fullfile(folder, 'simfile.sim');
resultsDir = fullfile(folder, 'Results', 'Run_abc');
mkdir(resultsDir);
logFile = fullfile(resultsDir, 'LastRun_log.txt');
endFile = fullfile(resultsDir, 'LastRun_end.par');
writelines([
    "SIMFILE"
    "SET_MACRO $(ROOT_FILE_NAME)$ Run_abc"
    "SET_MACRO $(OUTPUT_PATH)$ " + fullfile(folder, 'Results')
    "SET_MACRO $(WORK_DIR)$ " + folder + filesep
    "SET_MACRO $(OUTPUT_FILE_PREFIX)$ $(WORK_DIR)$Results\$(ROOT_FILE_NAME)$\LastRun"
    "PRODUCT_VER 2024.1"
    "EXT_MODEL_STEP 0.00050000"
    "END"
], simfilePath);
writelines(logLines, logFile);
writelines("SV_STATION 245.0016584 ; m ! Station", endFile);
end

function info = localMissingPidBlockInfo(cfg)
info = localValidModelInfo(cfg);
info.pidBlock.exists = false;
end

function info = localMissingYawMomentSwitchInfo(cfg)
info = localValidModelInfo(cfg);
info.yawMomentSwitch.exists = false;
end

function info = localSavedSwitchNotPidInfo(cfg)
info = localValidModelInfo(cfg);
info.yawMomentSwitch.savedValue = "0";
end

function info = localValidModelInfo(cfg)
info = struct();
info.modelPath = string(cfg.modelPath);
info.modelFileExists = true;
info.pidBlock = struct('path', string(cfg.pidBlock), 'exists', true, ...
    'hasP', true, 'hasI', true, 'hasD', true);
info.yawMomentSwitch = struct('path', string(cfg.yawMomentModeSwitchBlock), ...
    'exists', true, 'blockType', "ManualSwitch", ...
    'savedValue', string(cfg.yawMomentPidSwitchValue), ...
    'configuredSelectedPort', 1, ...
    'selectedGotoTag', string(cfg.yawMomentPidGotoTag));
end
