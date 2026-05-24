classdef writeDycAutocrossComparisonReportTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addOptimizationPath(testCase)
            testFolder = fileparts(mfilename('fullpath'));
            optimizationFolder = fileparts(testFolder);
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(optimizationFolder));
        end
    end

    methods (Test)
        function testReportWritesHtmlCsvAndMat(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            cfg = localConfig(fixture.Folder);
            runResults = [ ...
                localRun("dyc_off", "DYC 关闭", 1, 62.0, localSignals([0 1 2], [0.30 -0.20 0.10], [0 0 0], [0.40 -0.20 0.30], [0 0 0], [2.0 -3.0 1.0], [14 15 14], [0 0 0])), ...
                localRun("dyc_on", "当前 PID DYC", 1, 60.0, localSignals([0 1 2], [0.10 -0.05 0.05], [0 0 0], [0.10 -0.05 0.05], [0 0 0], [1.5 -2.0 1.0], [16 17 16], [0 80 -120]))];
            metrics = compute_dyc_autocross_metrics(runResults, cfg);

            write_dyc_autocross_comparison_report(cfg, runResults, metrics);

            reportPath = fullfile(cfg.resultsDir, 'report.html');
            comparisonMetricsPath = fullfile(cfg.resultsDir, 'comparison_metrics.csv');
            runResultsPath = fullfile(cfg.resultsDir, 'run_results.csv');
            resultMatPath = fullfile(cfg.resultsDir, 'comparison_result.mat');
            testCase.verifyTrue(isfile(reportPath));
            testCase.verifyTrue(isfile(comparisonMetricsPath));
            testCase.verifyTrue(isfile(runResultsPath));
            testCase.verifyTrue(isfile(resultMatPath));

            html = fileread(reportPath);
            testCase.verifyTrue(contains(html, 'DYC Autocross 有无控制对比报告'));
            testCase.verifyTrue(contains(html, '圈速差值'));
            testCase.verifyTrue(contains(html, '机理解释'));
            testCase.verifyTrue(contains(html, '控制介入'));
            testCase.verifyTrue(contains(html, '运行证据'));
            testCase.verifyTrue(contains(html, '验证边界'));

            comparisonMetrics = readtable(comparisonMetricsPath, 'TextType', 'string');
            perRun = readtable(runResultsPath, 'TextType', 'string');
            testCase.verifyEqual(height(comparisonMetrics), 2);
            testCase.verifyEqual(height(perRun), 2);
        end

        function testReportRejectsEscapedArtifactPath(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            cfg = localConfig(fixture.Folder);
            cfg.reportPath = fullfile(fixture.Folder, '..', 'escape', 'report.html');
            runResults = localValidRunPair();
            metrics = compute_dyc_autocross_metrics(runResults, cfg);

            testCase.verifyError(@() write_dyc_autocross_comparison_report(cfg, runResults, metrics), ...
                'dyc:autocrossComparison:ArtifactPathOutsideResultsDir');
            testCase.verifyFalse(isfile(fullfile(fixture.Folder, '..', 'escape', 'report.html')));
        end

        function testUnrelatedOutsideArtifactPathFallsBackToDefault(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            cfg = localConfig(fixture.Folder);
            unrelatedPath = fullfile(tempdir, 'unrelated_report.html');
            if isfile(unrelatedPath)
                delete(unrelatedPath);
            end
            cfg.reportPath = unrelatedPath;
            runResults = localValidRunPair();
            metrics = compute_dyc_autocross_metrics(runResults, cfg);

            write_dyc_autocross_comparison_report(cfg, runResults, metrics);

            testCase.verifyTrue(isfile(fullfile(cfg.resultsDir, 'report.html')));
            testCase.verifyFalse(isfile(unrelatedPath));
        end

        function testInvalidComparisonReportStatesNoConclusion(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            cfg = localConfig(fixture.Folder);
            offRun = localRun("dyc_off", "DYC 关闭", 1, 62.0, localSignals([0 1], [0.1 0.1], [0 0], [0.1 0.1], [0 0], [1 1], [10 10], [0 0]));
            onRun = localRun("dyc_on", "当前 PID DYC", 1, NaN, struct());
            onRun.status = "invalid";
            onRun.failureReason = "测试失败运行";
            runResults = [offRun onRun];
            metrics = compute_dyc_autocross_metrics(runResults, cfg);

            write_dyc_autocross_comparison_report(cfg, runResults, metrics);

            html = fileread(fullfile(cfg.resultsDir, 'report.html'));
            testCase.verifyTrue(contains(html, '不输出 DYC 有效性结论'));
        end
    end
end

function cfg = localConfig(resultsDir)
cfg = dyc_autocross_comparison_config(struct( ...
    'resultsDir', resultsDir, ...
    'modelName', 'DYC_1_9_test', ...
    'simfilePath', fullfile(resultsDir, 'simfile.sim'), ...
    'repeatCount', 1, ...
    'testMode', true));
end

function runResults = localValidRunPair()
runResults = [ ...
    localRun("dyc_off", "DYC 关闭", 1, 62.0, localSignals([0 1], [0.2 0.1], [0 0], [0.2 0.1], [0 0], [1 2], [10 11], [0 0])), ...
    localRun("dyc_on", "当前 PID DYC", 1, 60.0, localSignals([0 1], [0.1 0.05], [0 0], [0.1 0.05], [0 0], [1 1], [12 13], [0 100]))];
end

function run = localRun(caseId, displayName, repeatIndex, lapTime_s, signals)
run = struct();
run.caseId = caseId;
run.displayName = displayName;
run.repeatIndex = repeatIndex;
run.Kp = 6000;
run.Ki = 200;
run.Kd = 0;
run.status = "valid";
run.lapTime_s = lapTime_s;
run.finishStation_m = 245;
run.svStation_m = 1234;
run.objective = lapTime_s;
run.penalty = 0;
run.stopReason = "VS Command STOP_RUN_NOW End event triggered";
run.failureReason = "";
run.lastRunLogPath = "E:\example\LastRun_log.txt";
run.lastRunEndPath = "E:\example\LastRun_end.par";
run.elapsedWallTime_s = 1.2;
run.signals = signals;
end

function signals = localSignals(time_s, yawRate, yawTarget, latVeh, latTarget, ay, speed, mz)
signals = struct();
signals.time_s = time_s;
signals.yawRate_radps = yawRate;
signals.yawRateTarget_radps = yawTarget;
signals.latVeh_m = latVeh;
signals.latTarget_m = latTarget;
signals.ay_mps2 = ay;
signals.speed_mps = speed;
signals.mz_Nm = mz;
end
