classdef analyzeDycAutocrossReportResultsTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addOptimizationPath(testCase)
            testFolder = fileparts(mfilename('fullpath'));
            optimizationFolder = fileparts(testFolder);
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(optimizationFolder));
        end
    end

    methods (Test)
        function testBuildsWhereFasterAnalysisFromExportedCsv(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            localWriteAutocrossCsvFixture(fixture.Folder);

            analysis = analyze_dyc_autocross_report_results(fixture.Folder);

            testCase.verifyTrue(isfield(analysis, 'whereFasterTable'));
            testCase.verifyTrue(isfile(fullfile(fixture.Folder, 'autocross_where_faster.csv')));
            testCase.verifyTrue(isfile(fullfile(fixture.Folder, 'autocross_where_faster_analysis.txt')));
            testCase.verifyTrue(any(analysis.whereFasterTable.scope == "overall"));
            testCase.verifyTrue(any(analysis.whereFasterTable.scope == "speed"));
            testCase.verifyTrue(any(analysis.whereFasterTable.scope == "path_tracking"));
            testCase.verifyTrue(any(analysis.whereFasterTable.scope == "yaw_moment"));
            testCase.verifyTrue(any(analysis.whereFasterTable.scope == "tire_utilization"));
            testCase.verifyTrue(any(contains(analysis.whereFasterTable.interpretation, "平均速度")));
            testCase.verifyTrue(any(contains(analysis.whereFasterTable.interpretation, "路径")));
            testCase.verifyTrue(any(contains(analysis.whereFasterTable.interpretation, "横摆力矩")));
            testCase.verifyTrue(any(contains(analysis.analysisLines, "快在哪里")));
            testCase.verifyTrue(any(contains(analysis.analysisLines, "稳定性证据混合")));
        end

        function testMissingRequiredCsvRaisesClearError(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);

            testCase.verifyError(@() analyze_dyc_autocross_report_results(fixture.Folder), ...
                'dyc:autocrossReportAnalysis:MissingFile');
        end
    end
end

function localWriteAutocrossCsvFixture(resultsDir)
signalDir = fullfile(resultsDir, 'signal_data');
mkdir(signalDir);

runResults = table();
runResults.caseId = ["dyc_off"; "dyc_on"];
runResults.displayName = ["DYC 关闭"; "当前 PID DYC"];
runResults.repeatIndex = [1; 1];
runResults.Kp = [0; 6000];
runResults.Ki = [0; 200];
runResults.Kd = [0; 0];
runResults.status = ["valid"; "valid"];
runResults.lapTime_s = [62.0; 60.0];
runResults.finishStation_m = [NaN; NaN];
runResults.svStation_m = [1234.5; 1234.5];
runResults.objective = [62.0; 60.0];
runResults.penalty = [0; 0];
runResults.stopReason = ["VS Command STOP_RUN_NOW End event triggered"; "VS Command STOP_RUN_NOW End event triggered"];
runResults.failureReason = [""; ""];
runResults.lastRunLogPath = ["fake_off_log"; "fake_on_log"];
runResults.lastRunEndPath = ["fake_off_end"; "fake_on_end"];
writetable(runResults, fullfile(resultsDir, 'run_results.csv'));

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
summary.minSpeed_mps = [8.5; 8.3];
summary.speedStd_mps = [1.2; 1.1];
summary.stationEnd_m = [400; 402];
summary.throttleMean = [0.30; 0.26];
summary.throttlePeak = [0.38; 0.34];
summary.steerRms_rad = [NaN; NaN];
summary.steerPeakAbs_rad = [NaN; NaN];
summary.tireUtilPeak = [1.04; 1.18];
summary.tireUtilMean = [0.88; 0.76];
summary.wheelTorqueSpreadRms_Nm = [8; 90];
summary.wheelTorqueSpreadPeak_Nm = [10; 115];
summary.mzRms_Nm = [10; 120];
summary.mzPeakAbs_Nm = [12; 170];
summary.mzAbsIntegral_Nms = [100; 900];
summary.interventionRatio = [0; 0.65];
writetable(summary, fullfile(resultsDir, 'comparison_metrics.csv'));

time = linspace(0, 20, 101)';
aligned = table();
aligned.time_s = time;
aligned.station_m_dyc_off = 20 * time;
aligned.station_m_dyc_on = 20 * time + 1.5;
aligned.station_m_delta = aligned.station_m_dyc_on - aligned.station_m_dyc_off;
aligned.speed_mps_dyc_off = 12 + 2.0 * sin(time / 3);
aligned.speed_mps_dyc_on = aligned.speed_mps_dyc_off + 0.4;
aligned.speed_mps_delta = aligned.speed_mps_dyc_on - aligned.speed_mps_dyc_off;
aligned.yawRate_radps_dyc_off = 0.2 * sin(time / 2);
aligned.yawRate_radps_dyc_on = 0.18 * sin(time / 2);
aligned.yawRate_radps_delta = aligned.yawRate_radps_dyc_on - aligned.yawRate_radps_dyc_off;
aligned.yawRateError_radps_dyc_off = 0.05 * sin(time);
aligned.yawRateError_radps_dyc_on = 0.03 * sin(time);
aligned.yawRateError_radps_delta = aligned.yawRateError_radps_dyc_on - aligned.yawRateError_radps_dyc_off;
aligned.lateralError_m_dyc_off = 0.25 * sin(time / 2);
aligned.lateralError_m_dyc_on = 0.16 * sin(time / 2 + 0.2);
aligned.lateralError_m_delta = aligned.lateralError_m_dyc_on - aligned.lateralError_m_dyc_off;
aligned.ay_mps2_dyc_off = 5 * sin(time / 2);
aligned.ay_mps2_dyc_on = 5.2 * sin(time / 2);
aligned.ay_mps2_delta = aligned.ay_mps2_dyc_on - aligned.ay_mps2_dyc_off;
aligned.ax_mps2_dyc_off = 1.0 + 0.2 * cos(time / 2);
aligned.ax_mps2_dyc_on = 1.1 + 0.18 * cos(time / 2 + 0.3);
aligned.ax_mps2_delta = aligned.ax_mps2_dyc_on - aligned.ax_mps2_dyc_off;
aligned.mz_Nm_dyc_off = 10 * sin(time);
aligned.mz_Nm_dyc_on = 120 * sin(time) + 60 * cos(time / 3);
aligned.mz_Nm_delta = aligned.mz_Nm_dyc_on - aligned.mz_Nm_dyc_off;
aligned.throttle_dyc_off = 0.30 + 0.05 * sin(time / 2);
aligned.throttle_dyc_on = 0.26 + 0.04 * sin(time / 2 + 0.4);
aligned.throttle_delta = aligned.throttle_dyc_on - aligned.throttle_dyc_off;
aligned.steerSW_rad_dyc_off = NaN(size(time));
aligned.steerSW_rad_dyc_on = NaN(size(time));
aligned.steerSW_rad_delta = NaN(size(time));
aligned.tireUtilMax_dyc_off = 0.88 + 0.16 * sin(time / 2);
aligned.tireUtilMax_dyc_on = 0.76 + 0.10 * sin(time / 2 + 0.3);
aligned.tireUtilMax_delta = aligned.tireUtilMax_dyc_on - aligned.tireUtilMax_dyc_off;
aligned.wheelTorqueSpread_Nm_dyc_off = 8 + 2 * sin(time);
aligned.wheelTorqueSpread_Nm_dyc_on = 90 + 25 * sin(time / 1.8);
aligned.wheelTorqueSpread_Nm_delta = aligned.wheelTorqueSpread_Nm_dyc_on - aligned.wheelTorqueSpread_Nm_dyc_off;
aligned.leftRightDriveTorqueDelta_Nm_dyc_off = 2 * sin(time);
aligned.leftRightDriveTorqueDelta_Nm_dyc_on = 80 * sin(time / 2);
aligned.leftRightDriveTorqueDelta_Nm_delta = aligned.leftRightDriveTorqueDelta_Nm_dyc_on - aligned.leftRightDriveTorqueDelta_Nm_dyc_off;
writetable(aligned, fullfile(signalDir, 'aligned_dyc_comparison.csv'));
end
