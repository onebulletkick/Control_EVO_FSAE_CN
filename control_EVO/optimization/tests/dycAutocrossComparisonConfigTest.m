classdef dycAutocrossComparisonConfigTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addOptimizationPath(testCase)
            testFolder = fileparts(mfilename('fullpath'));
            optimizationFolder = fileparts(testFolder);
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(optimizationFolder));
        end
    end

    methods (Test)
        function testDefaultConfigDefinesAutocrossComparison(testCase)
            cfg = dyc_autocross_comparison_config();

            testCase.verifyEqual(cfg.modelName, 'DYC_1_9_test');
            testCase.verifyEqual(cfg.modelFolder, fullfile(cfg.repoRoot, 'control_EVO'));
            testCase.verifyEqual(cfg.modelPath, fullfile(cfg.modelFolder, 'DYC_1_9_test.slx'));
            testCase.verifyEqual(cfg.simfilePath, fullfile(cfg.modelFolder, 'simfile.sim'));
            testCase.verifyEqual(cfg.pidBlock, 'DYC_1_9_test/EVO_Control_System/YawMomentControl/PID_YawMomentController');
            testCase.verifyEqual(cfg.yawMomentModeSwitchBlock, 'DYC_1_9_test/EVO_Control_System/YawMomentControl/YawMomentControlMode');
            testCase.verifyEqual(cfg.yawMomentPidSwitchValue, '1');
            testCase.verifyEqual(cfg.yawMomentPidGotoTag, 'Mz_pid');
            testCase.verifyEqual(cfg.repeatCount, 1);

            testCase.verifyEqual(numel(cfg.cases), 2);
            testCase.verifyEqual(cfg.cases(1).id, 'dyc_off');
            testCase.verifyEqual(cfg.cases(1).displayName, 'DYC 关闭');
            testCase.verifyEqual(cfg.cases(1).pid.Kp, 0);
            testCase.verifyEqual(cfg.cases(1).pid.Ki, 0);
            testCase.verifyEqual(cfg.cases(1).pid.Kd, 0);
            testCase.verifyTrue(contains(cfg.cases(1).description, 'PID 分支'));
            testCase.verifyTrue(contains(cfg.cases(1).description, '无额外横摆力矩干预'));
            testCase.verifyEqual(cfg.cases(2).id, 'dyc_on');
            testCase.verifyEqual(cfg.cases(2).displayName, '当前 PID DYC');
            testCase.verifyEqual(cfg.cases(2).pid.Kp, 6000);
            testCase.verifyEqual(cfg.cases(2).pid.Ki, 200);
            testCase.verifyEqual(cfg.cases(2).pid.Kd, 0);
            testCase.verifyTrue(contains(cfg.cases(2).description, '当前基线 PID'));

            testCase.verifyEqual(cfg.metric.primaryMetric, 'carsim_autocross_stop_time');
            testCase.verifyEqual(cfg.metric.sStop_m, 245);
            testCase.verifyEqual(cfg.metric.finishStationTolerance_m, 1.0);
            testCase.verifyEqual(cfg.metric.requiredStopReason, 'VS Command STOP_RUN_NOW End event triggered');
            testCase.verifyEqual(cfg.penalty.hardFailureObjective, 9999);
            testCase.verifyEqual(cfg.simulation.stopTime, '120');
            testCase.verifyEqual(cfg.simulation.useFastRestart, 'off');
            testCase.verifyTrue(cfg.simulation.verifyLastRunTimestamp);
            testCase.verifyEmpty(cfg.simulation.simulateFcn);
            testCase.verifyEqual(cfg.resultsRoot, fullfile(cfg.optimizationFolder, 'results'));
            testCase.verifyEqual(cfg.reportPath, fullfile(cfg.resultsDir, 'report.html'));
            testCase.verifyEqual(cfg.comparisonMetricsPath, fullfile(cfg.resultsDir, 'comparison_metrics.csv'));
            testCase.verifyEqual(cfg.runResultsPath, fullfile(cfg.resultsDir, 'run_results.csv'));
            testCase.verifyEqual(cfg.resultMatPath, fullfile(cfg.resultsDir, 'comparison_result.mat'));
            testCase.verifyTrue(endsWith(cfg.resultsDir, '_dyc_autocross_comparison'));
            testCase.verifyFalse(cfg.testMode);
        end

        function testOverrideMergesNestedSimulationAndOutputPaths(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            overrides = struct( ...
                'resultsRoot', fixture.Folder, ...
                'repeatCount', 3, ...
                'simulation', struct('stopTime', '90', 'verifyLastRunTimestamp', false), ...
                'thresholds', struct('mzActive_Nm', 75));

            cfg = dyc_autocross_comparison_config(overrides);

            testCase.verifyEqual(cfg.resultsRoot, fixture.Folder);
            testCase.verifyEqual(cfg.repeatCount, 3);
            testCase.verifyEqual(cfg.simulation.stopTime, '90');
            testCase.verifyEqual(cfg.simulation.useFastRestart, 'off');
            testCase.verifyFalse(cfg.simulation.verifyLastRunTimestamp);
            testCase.verifyEqual(cfg.thresholds.mzActive_Nm, 75);
            testCase.verifyTrue(startsWith(cfg.resultsDir, fixture.Folder));
            testCase.verifyTrue(endsWith(cfg.resultsDir, '_dyc_autocross_comparison'));
            testCase.verifyEqual(cfg.reportPath, fullfile(cfg.resultsDir, 'report.html'));
            testCase.verifyEqual(cfg.comparisonMetricsPath, fullfile(cfg.resultsDir, 'comparison_metrics.csv'));
            testCase.verifyEqual(cfg.runResultsPath, fullfile(cfg.resultsDir, 'run_results.csv'));
            testCase.verifyEqual(cfg.resultMatPath, fullfile(cfg.resultsDir, 'comparison_result.mat'));
        end

        function testRunTimestampOverrideRefreshesDefaultOutputPaths(testCase)
            cfg = dyc_autocross_comparison_config(struct('runTimestamp', 'fixed'));

            testCase.verifyEqual(cfg.runTimestamp, 'fixed');
            testCase.verifyEqual(cfg.resultsDir, fullfile(cfg.resultsRoot, 'fixed_dyc_autocross_comparison'));
            testCase.verifyEqual(cfg.reportPath, fullfile(cfg.resultsDir, 'report.html'));
            testCase.verifyEqual(cfg.comparisonMetricsPath, fullfile(cfg.resultsDir, 'comparison_metrics.csv'));
            testCase.verifyEqual(cfg.runResultsPath, fullfile(cfg.resultsDir, 'run_results.csv'));
            testCase.verifyEqual(cfg.resultMatPath, fullfile(cfg.resultsDir, 'comparison_result.mat'));
        end

        function testResultsDirOverrideMovesArtifactPaths(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            resultsDir = fullfile(fixture.Folder, 'manual_result_dir');

            cfg = dyc_autocross_comparison_config(struct('resultsDir', resultsDir));

            testCase.verifyEqual(cfg.resultsDir, resultsDir);
            testCase.verifyEqual(cfg.reportPath, fullfile(resultsDir, 'report.html'));
            testCase.verifyEqual(cfg.comparisonMetricsPath, fullfile(resultsDir, 'comparison_metrics.csv'));
            testCase.verifyEqual(cfg.runResultsPath, fullfile(resultsDir, 'run_results.csv'));
            testCase.verifyEqual(cfg.resultMatPath, fullfile(resultsDir, 'comparison_result.mat'));
        end

        function testExplicitArtifactPathsArePreserved(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            resultsDir = fullfile(fixture.Folder, 'manual_result_dir');
            customReport = fullfile(fixture.Folder, 'custom_report.html');
            customRunResults = fullfile(fixture.Folder, 'custom_run_results.csv');

            cfg = dyc_autocross_comparison_config(struct( ...
                'resultsDir', resultsDir, ...
                'reportPath', customReport, ...
                'runResultsPath', customRunResults));

            testCase.verifyEqual(cfg.reportPath, customReport);
            testCase.verifyEqual(cfg.runResultsPath, customRunResults);
            testCase.verifyEqual(cfg.comparisonMetricsPath, fullfile(resultsDir, 'comparison_metrics.csv'));
            testCase.verifyEqual(cfg.resultMatPath, fullfile(resultsDir, 'comparison_result.mat'));
        end

        function testThresholdsAreAvailableForInterventionMetrics(testCase)
            cfg = dyc_autocross_comparison_config();

            testCase.verifyTrue(isfield(cfg.thresholds, 'mzActive_Nm'));
            testCase.verifyGreaterThan(cfg.thresholds.mzActive_Nm, 0);
        end
    end
end
