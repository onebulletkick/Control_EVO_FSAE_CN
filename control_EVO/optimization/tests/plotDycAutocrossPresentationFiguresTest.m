classdef plotDycAutocrossPresentationFiguresTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addOptimizationPath(testCase)
            testFolder = fileparts(mfilename('fullpath'));
            optimizationFolder = fileparts(testFolder);
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(optimizationFolder));
        end
    end

    methods (Test)
        function testPlotsPresentationFiguresFromExportedCsv(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            resultsDir = fixture.Folder;
            localWriteFixtureData(resultsDir);

            manifest = plot_dyc_autocross_presentation_figures(resultsDir);

            expectedFiles = [
                "key_metrics_summary.png"
                "speed_lateral_error_overview.png"
                "yaw_moment_torque_distribution.png"
                "tire_utilization_comparison.png"
                "throttle_accel_overview.png"
                "effectiveness_evidence_chain.png"
            ];
            testCase.verifyEqual(string(manifest.fileName), expectedFiles);
            testCase.verifyTrue(all(manifest.available));
            testCase.verifyTrue(isfile(fullfile(resultsDir, 'plots_presentation', 'presentation_plot_manifest.csv')));
            testCase.verifyFalse(isfolder(fullfile(resultsDir, 'plots')), ...
                "展示版重绘不应依赖或覆盖原始 plots 目录。");

            for idx = 1:numel(expectedFiles)
                plotPath = fullfile(resultsDir, 'plots_presentation', expectedFiles(idx));
                testCase.verifyTrue(isfile(plotPath), "Missing plot: " + string(plotPath));
                fileInfo = dir(plotPath);
                testCase.verifyGreaterThan(fileInfo.bytes, 1000, "Plot is unexpectedly small: " + string(plotPath));
            end
        end

        function testMissingAlignedComparisonRaisesClearError(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);

            testCase.verifyError(@() plot_dyc_autocross_presentation_figures(fixture.Folder), ...
                'dyc:autocrossPresentationPlots:MissingAlignedComparison');
        end
    end
end

function localWriteFixtureData(resultsDir)
signalDir = fullfile(resultsDir, 'signal_data');
mkdir(signalDir);

time = linspace(0, 20, 101)';
speedOff = 12 + 2.0 * sin(time / 3);
speedOn = speedOff + 0.4 + 0.2 * sin(time / 5);
latOff = 0.25 * sin(time / 2);
latOn = 0.16 * sin(time / 2 + 0.2);
mzOff = 10 * sin(time);
mzOn = 120 * sin(time) + 60 * cos(time / 3);
tireOff = 0.88 + 0.16 * sin(time / 2);
tireOn = 0.76 + 0.10 * sin(time / 2 + 0.3);
torqueOff = 8 + 2 * sin(time);
torqueOn = 90 + 25 * sin(time / 1.8);
throttleOff = 0.30 + 0.05 * sin(time / 2);
throttleOn = 0.26 + 0.04 * sin(time / 2 + 0.4);
axOff = 1.0 + 0.2 * cos(time / 2);
axOn = 1.1 + 0.18 * cos(time / 2 + 0.3);

aligned = table();
aligned.time_s = time;
aligned.station_m_dyc_off = 20 * time;
aligned.station_m_dyc_on = 20 * time + 1.5;
aligned.station_m_delta = aligned.station_m_dyc_on - aligned.station_m_dyc_off;
aligned.speed_mps_dyc_off = speedOff;
aligned.speed_mps_dyc_on = speedOn;
aligned.speed_mps_delta = speedOn - speedOff;
aligned.yawRate_radps_dyc_off = 0.2 * sin(time / 2);
aligned.yawRate_radps_dyc_on = 0.18 * sin(time / 2);
aligned.yawRate_radps_delta = aligned.yawRate_radps_dyc_on - aligned.yawRate_radps_dyc_off;
aligned.yawRateError_radps_dyc_off = 0.05 * sin(time);
aligned.yawRateError_radps_dyc_on = 0.03 * sin(time);
aligned.yawRateError_radps_delta = aligned.yawRateError_radps_dyc_on - aligned.yawRateError_radps_dyc_off;
aligned.lateralError_m_dyc_off = latOff;
aligned.lateralError_m_dyc_on = latOn;
aligned.lateralError_m_delta = latOn - latOff;
aligned.ay_mps2_dyc_off = 5 * sin(time / 2);
aligned.ay_mps2_dyc_on = 5.2 * sin(time / 2);
aligned.ay_mps2_delta = aligned.ay_mps2_dyc_on - aligned.ay_mps2_dyc_off;
aligned.ax_mps2_dyc_off = axOff;
aligned.ax_mps2_dyc_on = axOn;
aligned.ax_mps2_delta = axOn - axOff;
aligned.mz_Nm_dyc_off = mzOff;
aligned.mz_Nm_dyc_on = mzOn;
aligned.mz_Nm_delta = mzOn - mzOff;
aligned.throttle_dyc_off = throttleOff;
aligned.throttle_dyc_on = throttleOn;
aligned.throttle_delta = throttleOn - throttleOff;
aligned.steerSW_rad_dyc_off = NaN(size(time));
aligned.steerSW_rad_dyc_on = NaN(size(time));
aligned.steerSW_rad_delta = NaN(size(time));
aligned.tireUtilMax_dyc_off = tireOff;
aligned.tireUtilMax_dyc_on = tireOn;
aligned.tireUtilMax_delta = tireOn - tireOff;
aligned.wheelTorqueSpread_Nm_dyc_off = torqueOff;
aligned.wheelTorqueSpread_Nm_dyc_on = torqueOn;
aligned.wheelTorqueSpread_Nm_delta = torqueOn - torqueOff;
aligned.leftRightDriveTorqueDelta_Nm_dyc_off = 2 * sin(time);
aligned.leftRightDriveTorqueDelta_Nm_dyc_on = 80 * sin(time / 2);
aligned.leftRightDriveTorqueDelta_Nm_delta = aligned.leftRightDriveTorqueDelta_Nm_dyc_on - aligned.leftRightDriveTorqueDelta_Nm_dyc_off;
writetable(aligned, fullfile(signalDir, 'aligned_dyc_comparison.csv'));

summary = table();
summary.caseId = ["dyc_off"; "dyc_on"];
summary.displayName = ["DYC 关闭"; "当前 PID DYC"];
summary.validRunCount = [1; 1];
summary.lapTime_s = [62.0; 60.0];
summary.lapTimeStd_s = [NaN; NaN];
summary.yawRateRmse = [0.05; 0.03];
summary.yawRateMae = [0.04; 0.02];
summary.yawRatePeakError = [0.08; 0.05];
summary.lateralErrorRmse = [0.25; 0.16];
summary.lateralErrorPeak = [0.35; 0.22];
summary.ayPeakAbs = [5.0; 5.2];
summary.ayRms = [3.0; 3.1];
summary.ayStd = [2.0; 2.1];
summary.axPeakAbs = [1.2; 1.3];
summary.axRms = [0.8; 0.9];
summary.meanSpeed_mps = [13.0; 13.5];
summary.minSpeed_mps = [8.5; 9.0];
summary.speedStd_mps = [1.2; 1.1];
summary.stationEnd_m = [400; 402];
summary.throttleMean = [0.30; 0.26];
summary.throttlePeak = [0.38; 0.34];
summary.steerRms_rad = [NaN; NaN];
summary.steerPeakAbs_rad = [NaN; NaN];
summary.tireUtilPeak = [1.04; 0.86];
summary.tireUtilMean = [0.88; 0.76];
summary.wheelTorqueSpreadRms_Nm = [8; 90];
summary.wheelTorqueSpreadPeak_Nm = [10; 115];
summary.mzRms_Nm = [10; 120];
summary.mzPeakAbs_Nm = [12; 170];
summary.mzAbsIntegral_Nms = [100; 900];
summary.interventionRatio = [0; 0.65];
writetable(summary, fullfile(resultsDir, 'comparison_metrics.csv'));
end
