classdef runDycAutocrossReportSmokeTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addOptimizationPath(testCase)
            testFolder = fileparts(mfilename('fullpath'));
            optimizationFolder = fileparts(testFolder);
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(optimizationFolder));
        end
    end

    methods (Test)
        function testOneClickEntryRunsComparisonAndPrintsArtifactPaths(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            simfilePath = localWriteFixtureSimfile(fixture.Folder);
            resultsDir = fullfile(fixture.Folder, 'one_click_autocross_report');
            overrides = struct();
            overrides.simfilePath = simfilePath;
            overrides.resultsDir = resultsDir;
            overrides.testMode = true;
            overrides.simulation = struct( ...
                'simulateFcn', @localFakeAutocrossSimulation, ...
                'verifyLastRunTimestamp', true);

            commandText = evalc('result = run_dyc_autocross_report(overrides);');

            testCase.verifyEqual(numel(result.runResults), 2);
            testCase.verifyEqual([result.runResults.caseId], ["dyc_off" "dyc_on"]);
            testCase.verifyTrue(isfield(result, 'generatedPaths'));
            testCase.verifyEqual(result.generatedPaths.reportPath, string(fullfile(resultsDir, 'report.html')));
            testCase.verifyEqual(result.generatedPaths.comparisonMetricsPath, string(fullfile(resultsDir, 'comparison_metrics.csv')));
            testCase.verifyEqual(result.generatedPaths.whereFasterPath, string(fullfile(resultsDir, 'autocross_where_faster.csv')));
            testCase.verifyEqual(result.generatedPaths.plotsPresentationDir, string(fullfile(resultsDir, 'plots_presentation')));
            testCase.verifyTrue(contains(commandText, 'report.html'));
            testCase.verifyTrue(contains(commandText, 'comparison_metrics.csv'));
            testCase.verifyTrue(contains(commandText, 'autocross_where_faster.csv'));
            testCase.verifyTrue(contains(commandText, 'plots_presentation'));
            testCase.verifyTrue(isfile(fullfile(resultsDir, 'report.html')));
            testCase.verifyTrue(isfile(fullfile(resultsDir, 'comparison_result.mat')));
            testCase.verifyTrue(isfile(fullfile(resultsDir, 'signal_data', 'aligned_dyc_comparison.csv')));
            testCase.verifyTrue(isfile(fullfile(resultsDir, 'plots_presentation', 'presentation_plot_manifest.csv')));
            testCase.verifyTrue(isfile(fullfile(resultsDir, 'autocross_where_faster.csv')));
            testCase.verifyTrue(isfile(fullfile(resultsDir, 'autocross_where_faster_analysis.txt')));
        end
    end
end

function simfilePath = localWriteFixtureSimfile(folder)
simfilePath = fullfile(folder, 'simfile.sim');
resultsDir = fullfile(folder, 'Results', 'Run_autocross');
mkdir(resultsDir);
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
    signals = localSignals(true);
else
    lapTime = 60;
    signals = localSignals(false);
end
info = parse_dyc_simfile(cfg.simfilePath);
logFolder = fileparts(info.logFile);
if ~isfolder(logFolder)
    mkdir(logFolder);
end
writelines([
    "Run started: VS output file = " + string(caseDef.id) + "_" + string(repeatIndex) + ".vsb"
    "Run stopped at t = " + string(lapTime) + ". VS Command STOP_RUN_NOW End event triggered"
], info.logFile);
writelines("SV_STATION 1234.5 ; m ! Station", info.endFile);
simOut = struct('signals', signals);
end

function signals = localSignals(isDycOn)
t = (0:0.25:12)';
turn = sin(t);
signals = struct();
signals.source = "fake_autocross";
signals.time_s = t;
signals.station_m = 25 * t;
signals.yawRateTarget_radps = 0.18 * turn;
signals.latTarget_m = 0.05 * turn;
signals.ay_mps2 = 3.5 * turn;
signals.ax_mps2 = 0.6 + 0.1 * cos(t);
signals.throttle = 0.28 + 0.02 * cos(t);
signals.steerSW_rad = 0.4 * turn;
if isDycOn
    signals.speed_mps = 15 + 1.2 * cos(t);
    signals.yawRate_radps = 0.19 * turn;
    signals.latVeh_m = 0.07 * turn;
    signals.mz_Nm = 120 * turn;
    tireScale = 0.78;
    torque = 95 + 18 * abs(turn);
else
    signals.speed_mps = 14.5 + 1.0 * cos(t);
    signals.yawRate_radps = 0.25 * turn;
    signals.latVeh_m = 0.18 * turn;
    signals.mz_Nm = 5 * turn;
    tireScale = 0.95;
    torque = 8 + 2 * abs(turn);
end
signals.myDrL1_Nm = zeros(size(t));
signals.myDrL2_Nm = zeros(size(t));
signals.myDrR1_Nm = torque .* max(turn, 0);
signals.myDrR2_Nm = torque .* max(-turn, 0);
signals.tireFxL1_N = 80 * ones(size(t));
signals.tireFxL2_N = 80 * ones(size(t));
signals.tireFxR1_N = 80 * ones(size(t));
signals.tireFxR2_N = 80 * ones(size(t));
signals.tireFyL1_N = 420 * tireScale * abs(turn);
signals.tireFyL2_N = 390 * tireScale * abs(turn);
signals.tireFyR1_N = 410 * tireScale * abs(turn);
signals.tireFyR2_N = 385 * tireScale * abs(turn);
signals.tireFzL1_N = 500 * ones(size(t));
signals.tireFzL2_N = 500 * ones(size(t));
signals.tireFzR1_N = 500 * ones(size(t));
signals.tireFzR2_N = 500 * ones(size(t));
end
