classdef compareDycFigure8EffectivenessSmokeTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addOptimizationPath(testCase)
            testFolder = fileparts(mfilename('fullpath'));
            optimizationFolder = fileparts(testFolder);
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(optimizationFolder));
        end
    end

    methods (Test)
        function testMainEntryRunsBothCasesAndWritesFullFigure8Artifacts(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            [simfilePath, ~] = localWriteFixtureSimfile(fixture.Folder);
            resultsDir = fullfile(fixture.Folder, 'figure8_artifacts');
            overrides = localBaseOverrides(resultsDir, simfilePath, @localFakeFigure8Simulation);

            result = compare_dyc_figure8_effectiveness(overrides);

            testCase.verifyEqual(numel(result.runResults), 2);
            testCase.verifyEqual([result.runResults.caseId], ["dyc_off" "dyc_on"]);
            testCase.verifyEqual(result.metrics.comparison.status, "valid");
            testCase.verifyTrue(isfield(result.metrics, 'figure8Segments'));
            testCase.verifyTrue(isfile(result.cfg.reportPath));
            testCase.verifyTrue(isfile(result.cfg.comparisonMetricsPath));
            testCase.verifyTrue(isfile(result.cfg.runResultsPath));
            testCase.verifyTrue(isfile(result.cfg.resultMatPath));
            testCase.verifyTrue(isfile(fullfile(resultsDir, 'figure8_segment_metrics.csv')));
            testCase.verifyTrue(isfile(fullfile(resultsDir, 'figure8_segment_delta.csv')));
            testCase.verifyTrue(isfile(fullfile(resultsDir, 'effectiveness_analysis.txt')));
            testCase.verifyTrue(isfile(fullfile(resultsDir, 'plots_presentation', 'presentation_plot_manifest.csv')));

            html = fileread(result.cfg.reportPath);
            testCase.verifyTrue(contains(html, 'DYC 八字绕环有无控制对比报告'));
            testCase.verifyTrue(contains(html, '双主线'));
            testCase.verifyTrue(contains(html, '左转'));
            testCase.verifyTrue(contains(html, '右转'));
            testCase.verifyTrue(contains(html, '换向过渡'));
            testCase.verifyTrue(contains(html, '稳定性有效，圈速收益有限'));
            testCase.verifyTrue(contains(html, 'plots_presentation/figure8_left_right_segment_map.png'));

            analysisText = fileread(fullfile(resultsDir, 'effectiveness_analysis.txt'));
            testCase.verifyTrue(contains(analysisText, '八字绕环 DYC 有效性分析'));
            testCase.verifyTrue(contains(analysisText, '完成时间'));
            testCase.verifyTrue(contains(analysisText, '稳定性证据链'));
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
outputDir = fullfile(folder, 'Results', 'Run_figure8');
mkdir(outputDir);
writelines([
    "SIMFILE"
    "SET_MACRO $(ROOT_FILE_NAME)$ Run_figure8"
    "SET_MACRO $(OUTPUT_PATH)$ " + fullfile(folder, 'Results')
    "SET_MACRO $(WORK_DIR)$ " + folder + filesep
    "SET_MACRO $(OUTPUT_FILE_PREFIX)$ $(WORK_DIR)$Results\$(ROOT_FILE_NAME)$\LastRun"
    "END"
], simfilePath);
end

function simOut = localFakeFigure8Simulation(cfg, caseDef, repeatIndex)
if string(caseDef.id) == "dyc_on"
    lapTime = 59.9;
    signals = localFigure8Signals(true);
else
    lapTime = 60.0;
    signals = localFigure8Signals(false);
end
info = parse_dyc_simfile(cfg.simfilePath);
localWriteFigure8EventLog(info.logFile, info.endFile, lapTime, caseDef.id, repeatIndex);
simOut = struct('signals', signals);
end

function localWriteFigure8EventLog(logFile, endFile, lapTime, caseId, repeatIndex)
logFolder = fileparts(logFile);
if ~isfolder(logFolder)
    mkdir(logFolder);
end
writelines([
    "Run started: VS output file = " + string(caseId) + "_" + string(repeatIndex) + ".vsb"
    "Run stopped at t = " + string(lapTime) + ". VS Command STOP_RUN_NOW End event triggered"
], logFile);
writelines("SV_STATION 888.8 ; m ! Station", endFile);
end

function signals = localFigure8Signals(isDycOn)
t = (0:0.25:12)';
turn = sin(2 * pi * t / 6);
signals = struct();
signals.source = "fake_figure8";
signals.time_s = t;
signals.station_m = 10 * t;
signals.ay_mps2 = 4.5 * turn;
signals.yawRate_radps = 0.25 * turn;
signals.yawRateTarget_radps = 0.24 * turn;
signals.latTarget_m = zeros(size(t));
signals.ax_mps2 = 0.5 + 0.08 * cos(t);
signals.throttle = 0.28 + 0.02 * cos(t);
signals.steerSW_rad = 0.5 * turn;

if isDycOn
    signals.speed_mps = 12.4 + 0.8 * cos(t);
    signals.latVeh_m = 0.08 * sign(turn) + 0.03 * turn;
    signals.mz_Nm = 140 * turn;
    torque = 95 + 20 * abs(turn);
    tireScale = 0.76;
else
    signals.speed_mps = 12.2 + 0.7 * cos(t);
    signals.latVeh_m = 0.18 * sign(turn) + 0.06 * turn;
    signals.mz_Nm = 5 * turn;
    torque = 9 + 2 * abs(turn);
    tireScale = 0.95;
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
