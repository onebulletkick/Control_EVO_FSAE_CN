classdef dycFigure8ComparisonConfigTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addOptimizationPath(testCase)
            testFolder = fileparts(mfilename('fullpath'));
            optimizationFolder = fileparts(testFolder);
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(optimizationFolder));
        end
    end

    methods (Test)
        function testDefaultConfigDefinesFigure8Comparison(testCase)
            cfg = dyc_figure8_comparison_config();

            testCase.verifyEqual(cfg.scenario.id, "figure8");
            testCase.verifyEqual(cfg.scenario.displayName, "八字绕环");
            testCase.verifyEqual(cfg.reportTitle, "DYC 八字绕环有无控制对比报告");
            testCase.verifyTrue(endsWith(cfg.resultsDir, '_dyc_figure8_comparison'));
            testCase.verifyEqual(cfg.metric.primaryMetric, 'carsim_station_stop_time');
            testCase.verifyEqual(cfg.metric.requiredStopReason, 'Station limit reached');
            testCase.verifyEqual(cfg.cases(1).id, 'dyc_off');
            testCase.verifyEqual(cfg.cases(1).pid.Kp, 0);
            testCase.verifyEqual(cfg.cases(2).id, 'dyc_on');
            testCase.verifyEqual(cfg.cases(2).pid.Kp, 6000);
            testCase.verifyTrue(isfield(cfg.figure8, 'transitionAyAbsThreshold_mps2'));
            testCase.verifyGreaterThan(cfg.figure8.transitionAyAbsThreshold_mps2, 0);
        end

        function testOverridesRefreshFigure8OutputPaths(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            cfg = dyc_figure8_comparison_config(struct( ...
                'resultsRoot', fixture.Folder, ...
                'runTimestamp', 'fixed', ...
                'repeatCount', 3, ...
                'simulation', struct('stopTime', '90')));

            testCase.verifyEqual(cfg.repeatCount, 3);
            testCase.verifyEqual(cfg.simulation.stopTime, '90');
            testCase.verifyEqual(cfg.resultsDir, fullfile(fixture.Folder, 'fixed_dyc_figure8_comparison'));
            testCase.verifyEqual(cfg.reportPath, fullfile(cfg.resultsDir, 'report.html'));
            testCase.verifyEqual(cfg.comparisonMetricsPath, fullfile(cfg.resultsDir, 'comparison_metrics.csv'));
            testCase.verifyEqual(cfg.runResultsPath, fullfile(cfg.resultsDir, 'run_results.csv'));
            testCase.verifyEqual(cfg.resultMatPath, fullfile(cfg.resultsDir, 'comparison_result.mat'));
        end
    end
end
