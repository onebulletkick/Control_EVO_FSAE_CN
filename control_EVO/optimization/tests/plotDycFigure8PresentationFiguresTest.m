classdef plotDycFigure8PresentationFiguresTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addOptimizationPath(testCase)
            testFolder = fileparts(mfilename('fullpath'));
            optimizationFolder = fileparts(testFolder);
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(optimizationFolder));
        end
    end

    methods (Test)
        function testPlotsFigure8PresentationFigures(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            resultsDir = fixture.Folder;
            localWriteFixtureData(resultsDir);

            manifest = plot_dyc_figure8_presentation_figures(resultsDir);

            expectedFiles = [
                "key_metrics_summary.png"
                "figure8_left_right_segment_map.png"
                "speed_lateral_error_overview.png"
                "yaw_moment_transition_response.png"
                "tire_utilization_left_right.png"
                "effectiveness_evidence_chain.png"
            ];
            testCase.verifyEqual(string(manifest.fileName), expectedFiles);
            testCase.verifyTrue(all(manifest.available));
            testCase.verifyTrue(isfile(fullfile(resultsDir, 'plots_presentation', 'presentation_plot_manifest.csv')));
            for idx = 1:numel(expectedFiles)
                plotPath = fullfile(resultsDir, 'plots_presentation', expectedFiles(idx));
                testCase.verifyTrue(isfile(plotPath), "Missing plot: " + string(plotPath));
                fileInfo = dir(plotPath);
                testCase.verifyGreaterThan(fileInfo.bytes, 1000, "Plot is unexpectedly small: " + string(plotPath));
            end
        end
    end
end

function localWriteFixtureData(resultsDir)
signalDir = fullfile(resultsDir, 'signal_data');
mkdir(signalDir);
t = linspace(0, 12, 121)';
turn = sin(2 * pi * t / 6);
aligned = table();
aligned.time_s = t;
aligned.station_m_dyc_off = 10 * t;
aligned.station_m_dyc_on = 10 * t + 1;
aligned.station_m_delta = aligned.station_m_dyc_on - aligned.station_m_dyc_off;
aligned.speed_mps_dyc_off = 12 + 0.5 * cos(t);
aligned.speed_mps_dyc_on = 12.2 + 0.6 * cos(t);
aligned.speed_mps_delta = aligned.speed_mps_dyc_on - aligned.speed_mps_dyc_off;
aligned.yawRate_radps_dyc_off = 0.25 * turn;
aligned.yawRate_radps_dyc_on = 0.22 * turn;
aligned.yawRate_radps_delta = aligned.yawRate_radps_dyc_on - aligned.yawRate_radps_dyc_off;
aligned.yawRateError_radps_dyc_off = 0.05 * turn;
aligned.yawRateError_radps_dyc_on = 0.03 * turn;
aligned.yawRateError_radps_delta = aligned.yawRateError_radps_dyc_on - aligned.yawRateError_radps_dyc_off;
aligned.lateralError_m_dyc_off = 0.20 * sign(turn);
aligned.lateralError_m_dyc_on = 0.10 * sign(turn);
aligned.lateralError_m_delta = aligned.lateralError_m_dyc_on - aligned.lateralError_m_dyc_off;
aligned.ay_mps2_dyc_off = 4.5 * turn;
aligned.ay_mps2_dyc_on = 4.5 * turn;
aligned.ay_mps2_delta = zeros(size(t));
aligned.ax_mps2_dyc_off = 0.5 + 0.1 * cos(t);
aligned.ax_mps2_dyc_on = 0.55 + 0.1 * cos(t);
aligned.ax_mps2_delta = aligned.ax_mps2_dyc_on - aligned.ax_mps2_dyc_off;
aligned.mz_Nm_dyc_off = 5 * turn;
aligned.mz_Nm_dyc_on = 140 * turn;
aligned.mz_Nm_delta = aligned.mz_Nm_dyc_on - aligned.mz_Nm_dyc_off;
aligned.throttle_dyc_off = 0.28 + 0.02 * cos(t);
aligned.throttle_dyc_on = 0.25 + 0.02 * cos(t);
aligned.throttle_delta = aligned.throttle_dyc_on - aligned.throttle_dyc_off;
aligned.steerSW_rad_dyc_off = 0.5 * turn;
aligned.steerSW_rad_dyc_on = 0.5 * turn;
aligned.steerSW_rad_delta = zeros(size(t));
aligned.tireUtilMax_dyc_off = 0.95 * abs(turn);
aligned.tireUtilMax_dyc_on = 0.76 * abs(turn);
aligned.tireUtilMax_delta = aligned.tireUtilMax_dyc_on - aligned.tireUtilMax_dyc_off;
aligned.wheelTorqueSpread_Nm_dyc_off = 8 + 2 * abs(turn);
aligned.wheelTorqueSpread_Nm_dyc_on = 90 + 20 * abs(turn);
aligned.wheelTorqueSpread_Nm_delta = aligned.wheelTorqueSpread_Nm_dyc_on - aligned.wheelTorqueSpread_Nm_dyc_off;
aligned.leftRightDriveTorqueDelta_Nm_dyc_off = 2 * turn;
aligned.leftRightDriveTorqueDelta_Nm_dyc_on = 90 * turn;
aligned.leftRightDriveTorqueDelta_Nm_delta = aligned.leftRightDriveTorqueDelta_Nm_dyc_on - aligned.leftRightDriveTorqueDelta_Nm_dyc_off;
writetable(aligned, fullfile(signalDir, 'aligned_dyc_comparison.csv'));

summary = table();
summary.caseId = ["dyc_off"; "dyc_on"];
summary.displayName = ["DYC 关闭"; "当前 PID DYC"];
summary.validRunCount = [1; 1];
summary.lapTime_s = [60.0; 59.9];
summary.lapTimeStd_s = [NaN; NaN];
summary.yawRateRmse = [0.05; 0.03];
summary.yawRateMae = [0.04; 0.02];
summary.yawRatePeakError = [0.08; 0.05];
summary.lateralErrorRmse = [0.2; 0.1];
summary.lateralErrorPeak = [0.25; 0.13];
summary.ayPeakAbs = [4.5; 4.5];
summary.ayRms = [3.0; 3.0];
summary.ayStd = [2.0; 2.0];
summary.axPeakAbs = [0.6; 0.65];
summary.axRms = [0.5; 0.55];
summary.meanSpeed_mps = [12.0; 12.2];
summary.minSpeed_mps = [11.5; 11.7];
summary.speedStd_mps = [0.5; 0.5];
summary.stationEnd_m = [120; 121];
summary.throttleMean = [0.28; 0.25];
summary.throttlePeak = [0.30; 0.27];
summary.steerRms_rad = [0.35; 0.35];
summary.steerPeakAbs_rad = [0.5; 0.5];
summary.tireUtilPeak = [0.95; 0.76];
summary.tireUtilMean = [0.60; 0.48];
summary.wheelTorqueSpreadRms_Nm = [8; 90];
summary.wheelTorqueSpreadPeak_Nm = [10; 110];
summary.mzRms_Nm = [5; 140];
summary.mzPeakAbs_Nm = [5; 140];
summary.mzAbsIntegral_Nms = [30; 600];
summary.interventionRatio = [0; 0.7];
writetable(summary, fullfile(resultsDir, 'comparison_metrics.csv'));

segmentDelta = table();
segmentDelta.segmentType = ["left"; "right"; "transition"];
segmentDelta.lateralErrorRmseDelta = [-0.1; -0.09; -0.03];
segmentDelta.tireUtilPeakDelta = [-0.2; -0.18; -0.05];
segmentDelta.meanSpeedDelta_mps = [0.2; 0.15; 0.05];
writetable(segmentDelta, fullfile(resultsDir, 'figure8_segment_delta.csv'));
end
