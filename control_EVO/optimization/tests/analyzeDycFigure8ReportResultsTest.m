classdef analyzeDycFigure8ReportResultsTest < matlab.unittest.TestCase
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
            localWriteFigure8CsvFixture(fixture.Folder);

            analysis = analyze_dyc_figure8_report_results(fixture.Folder);

            testCase.verifyTrue(isfield(analysis, 'whereFasterTable'));
            testCase.verifyTrue(isfile(fullfile(fixture.Folder, 'figure8_where_faster.csv')));
            testCase.verifyTrue(isfile(fullfile(fixture.Folder, 'figure8_where_faster_analysis.txt')));
            testCase.verifyTrue(any(analysis.whereFasterTable.scope == "overall"));
            testCase.verifyTrue(any(analysis.whereFasterTable.scope == "left"));
            testCase.verifyTrue(any(analysis.whereFasterTable.scope == "right"));
            testCase.verifyTrue(any(analysis.whereFasterTable.scope == "transition"));
            testCase.verifyTrue(any(contains(analysis.whereFasterTable.interpretation, "右转")));
            testCase.verifyTrue(any(contains(analysis.whereFasterTable.interpretation, "换向")));
            testCase.verifyTrue(any(contains(analysis.analysisLines, "右转")));
            testCase.verifyTrue(any(contains(analysis.analysisLines, "稳定性证据混合")));
        end

        function testMissingRequiredCsvRaisesClearError(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);

            testCase.verifyError(@() analyze_dyc_figure8_report_results(fixture.Folder), ...
                'dyc:figure8ReportAnalysis:MissingFile');
        end
    end
end

function localWriteFigure8CsvFixture(resultsDir)
summary = table();
summary.caseId = ["dyc_off"; "dyc_on"];
summary.displayName = ["DYC 关闭"; "当前 PID DYC"];
summary.validRunCount = [1; 1];
summary.lapTime_s = [21.801; 21.561];
summary.lapTimeStd_s = [NaN; NaN];
summary.yawRateRmse = [NaN; NaN];
summary.yawRateMae = [NaN; NaN];
summary.yawRatePeakError = [NaN; NaN];
summary.lateralErrorRmse = [0.236; 0.226];
summary.lateralErrorPeak = [1.277; 1.233];
summary.ayPeakAbs = [15.91; 15.16];
summary.ayRms = [12.92; 13.13];
summary.ayStd = [12.92; 13.14];
summary.axPeakAbs = [2.0; 2.33];
summary.axRms = [0.65; 0.62];
summary.meanSpeed_mps = [11.150; 11.212];
summary.minSpeed_mps = [11.038; 10.891];
summary.speedStd_mps = [0.12; 0.12];
summary.stationEnd_m = [244.99; 244.88];
summary.throttleMean = [0.066; 0.047];
summary.throttlePeak = [0.157; 0.185];
summary.steerRms_rad = [1.88; 1.82];
summary.steerPeakAbs_rad = [2.38; 2.41];
summary.tireUtilPeak = [2.29; 2.77];
summary.tireUtilMean = [1.79; 1.75];
summary.wheelTorqueSpreadRms_Nm = [0; 43.98];
summary.wheelTorqueSpreadPeak_Nm = [0; 175.36];
summary.mzRms_Nm = [0; 198.58];
summary.mzPeakAbs_Nm = [0; 774.59];
summary.mzAbsIntegral_Nms = [0; 4047.49];
summary.interventionRatio = [0; 0.936];
writetable(summary, fullfile(resultsDir, 'comparison_metrics.csv'));

segments = table();
segments.caseId = ["dyc_off"; "dyc_on"; "dyc_off"; "dyc_on"; "dyc_off"; "dyc_on"];
segments.segmentType = ["left"; "left"; "right"; "right"; "transition"; "transition"];
segments.segmentSource = repmat("ay_mps2", 6, 1);
segments.sampleCount = [436; 432; 426; 418; 11; 13];
segments.lateralErrorRmse = [0.305; 0.296; 0.119; 0.101; 0.440; 0.385];
segments.lateralErrorPeak = [1.277; 1.233; 1.024; 0.925; 1.090; 0.997];
segments.meanSpeed_mps = [11.144; 11.200; 11.146; 11.213; 11.538; 11.567];
segments.minSpeed_mps = [11.038; 10.891; 11.097; 11.164; 11.449; 11.514];
segments.tireUtilPeak = [2.30; 2.56; 2.20; 2.75; 0.60; 0.58];
segments.tireUtilMean = [1.80; 1.77; 1.70; 1.65; 0.30; 0.29];
segments.mzPeakAbs_Nm = [0; 338; 0; 775; 0; 31];
segments.mzRms_Nm = [0; 181; 0; 218; 0; 24];
segments.wheelTorqueSpreadPeak_Nm = [0; 75; 0; 175; 0; 7];
segments.wheelTorqueSpreadRms_Nm = [0; 40; 0; 48; 0; 5];
writetable(segments, fullfile(resultsDir, 'figure8_segment_metrics.csv'));

segmentDelta = table();
segmentDelta.segmentType = ["left"; "right"; "transition"];
segmentDelta.sampleCountDelta = [-4; -8; 2];
segmentDelta.lateralErrorRmseDelta = [-0.009; -0.018; -0.054];
segmentDelta.lateralErrorPeakDelta = [-0.045; -0.099; -0.092];
segmentDelta.meanSpeedDelta_mps = [0.056; 0.067; 0.029];
segmentDelta.minSpeedDelta_mps = [-0.147; 0.067; 0.066];
segmentDelta.tireUtilPeakDelta = [0.256; 0.551; -0.016];
segmentDelta.tireUtilMeanDelta = [-0.033; -0.049; -0.009];
segmentDelta.mzPeakAbsDelta_Nm = [338; 775; 31];
segmentDelta.mzRmsDelta_Nm = [181; 218; 24];
segmentDelta.wheelTorqueSpreadPeakDelta_Nm = [75; 175; 7];
segmentDelta.wheelTorqueSpreadRmsDelta_Nm = [40; 48; 5];
writetable(segmentDelta, fullfile(resultsDir, 'figure8_segment_delta.csv'));
end
