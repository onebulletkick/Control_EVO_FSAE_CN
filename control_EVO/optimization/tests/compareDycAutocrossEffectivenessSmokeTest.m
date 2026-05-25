classdef compareDycAutocrossEffectivenessSmokeTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addOptimizationPath(testCase)
            testFolder = fileparts(mfilename('fullpath'));
            optimizationFolder = fileparts(testFolder);
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(optimizationFolder));
        end
    end

    methods (Test)
        function testMainEntryRunsBothCasesWithFakeSimulation(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            [simfilePath, ~] = localWriteFixtureSimfile(fixture.Folder);
            resultsDir = fullfile(fixture.Folder, 'artifacts');
            overrides = localBaseOverrides(resultsDir, simfilePath, @localFakeAutocrossSimulation);

            result = compare_dyc_autocross_effectiveness(overrides);

            testCase.verifyEqual(numel(result.runResults), 2);
            testCase.verifyEqual([result.runResults.caseId], ["dyc_off" "dyc_on"]);
            testCase.verifyEqual(result.metrics.comparison.status, "valid");
            testCase.verifyLessThan(result.metrics.comparison.lapTimeDelta_s, 0);
            testCase.verifyTrue(isfile(result.cfg.reportPath));
            testCase.verifyTrue(isfile(result.cfg.comparisonMetricsPath));
            testCase.verifyTrue(isfile(result.cfg.runResultsPath));
            testCase.verifyTrue(isfile(result.cfg.resultMatPath));
            testCase.verifyTrue(isfile(fullfile(resultsDir, 'autocross_where_faster.csv')));
            testCase.verifyTrue(isfile(fullfile(resultsDir, 'autocross_where_faster_analysis.txt')));
            testCase.verifyTrue(isfile(fullfile(resultsDir, 'plots_presentation', 'presentation_plot_manifest.csv')));

            html = fileread(result.cfg.reportPath);
            testCase.verifyTrue(contains(html, '快在哪里'));

            runResultsTable = readtable(result.cfg.runResultsPath, 'TextType', 'string');
            testCase.verifyEqual(sort(runResultsTable.caseId), ["dyc_off"; "dyc_on"]);
        end

        function testMainEntryPropagatesPathSafetyError(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            [simfilePath, ~] = localWriteFixtureSimfile(fixture.Folder);
            resultsDir = fullfile(fixture.Folder, 'artifacts');
            overrides = localBaseOverrides(resultsDir, simfilePath, @localFakeAutocrossSimulation);
            overrides.reportPath = fullfile(resultsDir, '..', 'escape', 'report.html');

            testCase.verifyError(@() compare_dyc_autocross_effectiveness(overrides), ...
                'dyc:autocrossComparison:ArtifactPathOutsideResultsDir');
        end

        function testMainEntryReturnsEffectiveFallbackArtifactPath(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            [simfilePath, ~] = localWriteFixtureSimfile(fixture.Folder);
            resultsDir = fullfile(fixture.Folder, 'artifacts');
            unrelatedPath = fullfile(fixture.Folder, 'unrelated_report.html');
            overrides = localBaseOverrides(resultsDir, simfilePath, @localFakeAutocrossSimulation);
            overrides.reportPath = unrelatedPath;

            result = compare_dyc_autocross_effectiveness(overrides);

            defaultReportPath = fullfile(resultsDir, 'report.html');
            testCase.verifyTrue(isfile(defaultReportPath));
            testCase.verifyFalse(isfile(unrelatedPath));
            testCase.verifyEqual(result.cfg.reportPath, defaultReportPath);
        end

        function testRepeatCountProducesFourRuns(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            [simfilePath, ~] = localWriteFixtureSimfile(fixture.Folder);
            resultsDir = fullfile(fixture.Folder, 'artifacts');
            overrides = localBaseOverrides(resultsDir, simfilePath, @localFakeRepeatSimulation);
            overrides.repeatCount = 2;

            result = compare_dyc_autocross_effectiveness(overrides);

            testCase.verifyEqual(numel(result.runResults), 4);
            testCase.verifyEqual([result.runResults.caseId], ["dyc_off" "dyc_off" "dyc_on" "dyc_on"]);
            testCase.verifyEqual([result.runResults.repeatIndex], [1 2 1 2]);
            testCase.verifyEqual(result.metrics.comparison.status, "valid");
        end
    end
end

function overrides = localBaseOverrides(resultsDir, simfilePath, simulateFcn)
overrides = struct();
overrides.simfilePath = simfilePath;
overrides.resultsDir = resultsDir;
overrides.testMode = true;
overrides.simulation = struct( ...
    'simulateFcn', simulateFcn, ...
    'verifyLastRunTimestamp', true);
end

function [simfilePath, outputDir] = localWriteFixtureSimfile(folder)
simfilePath = fullfile(folder, 'simfile.sim');
outputDir = fullfile(folder, 'Results', 'Run_autocross');
mkdir(outputDir);
writelines([
    "SIMFILE"
    "SET_MACRO $(ROOT_FILE_NAME)$ Run_autocross"
    "SET_MACRO $(OUTPUT_PATH)$ " + fullfile(folder, 'Results')
    "SET_MACRO $(WORK_DIR)$ " + folder + filesep
    "SET_MACRO $(OUTPUT_FILE_PREFIX)$ $(WORK_DIR)$Results\$(ROOT_FILE_NAME)$\LastRun"
    "END"
], simfilePath);
end

function simOut = localFakeAutocrossSimulation(cfg, caseDef, repeatIndex)
if string(caseDef.id) == "dyc_on"
    lapTime = 58;
else
    lapTime = 60;
end
simOut = localWriteFakeRun(cfg, caseDef, repeatIndex, lapTime);
end

function simOut = localFakeRepeatSimulation(cfg, caseDef, repeatIndex)
if string(caseDef.id) == "dyc_on"
    lapTime = 58 + repeatIndex / 10;
else
    lapTime = 60 + repeatIndex / 10;
end
simOut = localWriteFakeRun(cfg, caseDef, repeatIndex, lapTime);
end

function simOut = localWriteFakeRun(cfg, caseDef, repeatIndex, lapTime)
info = parse_dyc_simfile(cfg.simfilePath);
localWriteAutocrossEventLog(info.logFile, info.endFile, lapTime, caseDef.id, repeatIndex);

signals = struct();
signals.source = "fake";
signals.time_s = [0 1 2 3];
signals.yawRateTarget_radps = [0 0.10 0.20 0.10];
signals.latTarget_m = [0 0.05 0.10 0.05];
signals.ay_mps2 = [0 1.2 1.8 0.9];
if string(caseDef.id) == "dyc_on"
    signals.speed_mps = [16 17 18 17];
    signals.yawRate_radps = [0 0.11 0.19 0.11];
    signals.latVeh_m = [0 0.06 0.09 0.06];
    signals.mz_Nm = [0 90 130 80];
else
    signals.speed_mps = [14 15 15.5 15];
    signals.yawRate_radps = [0 0.16 0.11 0.17];
    signals.latVeh_m = [0 0.20 0.28 0.18];
    signals.mz_Nm = [0 0 0 0];
end
simOut = struct('signals', signals);
end

function localWriteAutocrossEventLog(logFile, endFile, lapTime, caseId, repeatIndex)
logFolder = fileparts(logFile);
if ~isfolder(logFolder)
    mkdir(logFolder);
end
writelines([
    "Run started: VS output file = " + string(caseId) + "_" + string(repeatIndex) + ".vsb"
    "Run stopped at t = " + string(lapTime) + ". VS Command STOP_RUN_NOW End event triggered"
], logFile);
writelines("SV_STATION 1234.5 ; m ! Station", endFile);
end
