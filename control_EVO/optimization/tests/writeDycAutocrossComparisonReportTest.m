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
            signalManifestPath = fullfile(cfg.resultsDir, 'signal_data', 'signal_manifest.csv');
            alignedComparisonPath = fullfile(cfg.resultsDir, 'signal_data', 'aligned_dyc_comparison.csv');
            offSignalsPath = fullfile(cfg.resultsDir, 'signal_data', 'dyc_off_repeat1_timeseries.csv');
            onSignalsPath = fullfile(cfg.resultsDir, 'signal_data', 'dyc_on_repeat1_timeseries.csv');
            testCase.verifyTrue(isfile(reportPath));
            testCase.verifyTrue(isfile(comparisonMetricsPath));
            testCase.verifyTrue(isfile(runResultsPath));
            testCase.verifyTrue(isfile(resultMatPath));
            testCase.verifyTrue(isfile(signalManifestPath));
            testCase.verifyTrue(isfile(alignedComparisonPath));
            testCase.verifyTrue(isfile(offSignalsPath));
            testCase.verifyTrue(isfile(onSignalsPath));

            html = fileread(reportPath);
            testCase.verifyTrue(contains(html, 'DYC Autocross 有无控制对比报告'));
            testCase.verifyTrue(contains(html, '圈速差值'));
            testCase.verifyTrue(contains(html, '机理解释'));
            testCase.verifyTrue(contains(html, '机理指标差值'));
            testCase.verifyTrue(contains(html, 'yawRateRmseDelta'));
            testCase.verifyTrue(contains(html, 'interventionRatioDelta'));
            testCase.verifyTrue(contains(html, '数据分析图'));
            testCase.verifyTrue(contains(html, '速度对比'));
            testCase.verifyTrue(contains(html, '横摆角速度对比'));
            testCase.verifyTrue(contains(html, '横摆误差对比'));
            testCase.verifyTrue(contains(html, '路径误差对比'));
            testCase.verifyTrue(contains(html, '横摆力矩对比'));
            testCase.verifyTrue(contains(html, 'plots/speed_comparison.png'));
            testCase.verifyTrue(contains(html, '控制介入'));
            testCase.verifyTrue(contains(html, '可复用数据文件'));
            testCase.verifyTrue(contains(html, 'signal_data/aligned_dyc_comparison.csv'));
            testCase.verifyTrue(contains(html, '运行证据'));
            testCase.verifyTrue(contains(html, '验证边界'));

            comparisonMetrics = readtable(comparisonMetricsPath, 'TextType', 'string');
            perRun = readtable(runResultsPath, 'TextType', 'string');
            manifest = readtable(signalManifestPath, 'TextType', 'string');
            offSignals = readtable(offSignalsPath, 'TextType', 'string');
            aligned = readtable(alignedComparisonPath, 'TextType', 'string');
            testCase.verifyEqual(height(comparisonMetrics), 2);
            testCase.verifyEqual(height(perRun), 2);
            testCase.verifyEqual(height(manifest), 2);
            testCase.verifyTrue(all(ismember({'time_s','speed_mps','lateralError_m','tireUtilMax','wheelTorqueSpread_Nm'}, offSignals.Properties.VariableNames)));
            testCase.verifyTrue(all(ismember({'time_s','speed_mps_dyc_off','speed_mps_dyc_on','speed_mps_delta','lateralError_m_delta','tireUtilMax_delta','wheelTorqueSpread_Nm_delta'}, aligned.Properties.VariableNames)));
            testCase.verifyPlotFile(cfg.resultsDir, 'speed_comparison.png');
            testCase.verifyPlotFile(cfg.resultsDir, 'yaw_rate_comparison.png');
            testCase.verifyPlotFile(cfg.resultsDir, 'yaw_error_comparison.png');
            testCase.verifyPlotFile(cfg.resultsDir, 'lateral_error_comparison.png');
            testCase.verifyPlotFile(cfg.resultsDir, 'yaw_moment_comparison.png');
            testCase.verifyPlotFile(cfg.resultsDir, 'lateral_accel_comparison.png');
            testCase.verifyPlotFile(cfg.resultsDir, 'longitudinal_accel_comparison.png');
            testCase.verifyPlotFile(cfg.resultsDir, 'throttle_comparison.png');
            testCase.verifyPlotFile(cfg.resultsDir, 'steering_comparison.png');
            testCase.verifyPlotFile(cfg.resultsDir, 'tire_utilization_comparison.png');
            testCase.verifyPlotFile(cfg.resultsDir, 'wheel_torque_spread_comparison.png');
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

        function testReportPlotsSignalDataFromInvalidRuns(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            cfg = localConfig(fixture.Folder);
            offRun = localRun("dyc_off", "DYC 关闭", 1, NaN, ...
                localSignals([0 1], [0.2 0.1], [0 0], [0.2 0.1], [0 0], [1 2], [10 11], [0 0]));
            onRun = localRun("dyc_on", "当前 PID DYC", 1, NaN, ...
                localSignals([0 1], [0.1 0.05], [0 0], [0.1 0.05], [0 0], [1 1], [12 13], [0 100]));
            offRun.status = "invalid";
            onRun.status = "invalid";
            offRun.failureReason = "测试无效圈";
            onRun.failureReason = "测试无效圈";
            runResults = [offRun onRun];
            metrics = compute_dyc_autocross_metrics(runResults, cfg);

            write_dyc_autocross_comparison_report(cfg, runResults, metrics);

            testCase.verifyPlotFile(cfg.resultsDir, 'speed_comparison.png');
            testCase.verifyPlotFile(cfg.resultsDir, 'yaw_rate_comparison.png');
            testCase.verifyPlotFile(cfg.resultsDir, 'yaw_error_comparison.png');
            html = fileread(cfg.reportPath);
            testCase.verifyTrue(contains(html, 'plots/speed_comparison.png'));
        end

        function testControlInterventionAvailabilityUsesDycOnOnly(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            cfg = localConfig(fixture.Folder);
            offRun = localRun("dyc_off", "DYC 关闭", 1, 62.0, ...
                localSignals([0 1], [0.2 0.1], [0 0], [0.2 0.1], [0 0], [1 2], [10 11], [80 100]));
            onRun = localRun("dyc_on", "当前 PID DYC", 1, 60.0, ...
                localSignals([0 1], [0.1 0.05], [0 0], [0.1 0.05], [0 0], [1 1], [12 13], [NaN NaN]));
            runResults = [offRun onRun];
            metrics = compute_dyc_autocross_metrics(runResults, cfg);

            write_dyc_autocross_comparison_report(cfg, runResults, metrics);

            html = fileread(fullfile(cfg.resultsDir, 'report.html'));
            testCase.verifyTrue(contains(html, '控制介入信号不可用'));
            testCase.verifyFalse(contains(html, '下表列出 dyc_on 的横摆力矩'));
        end
    end

    methods
        function verifyPlotFile(testCase, resultsDir, fileName)
            plotPath = fullfile(resultsDir, 'plots', fileName);
            hasFile = isfile(plotPath);
            testCase.verifyTrue(hasFile, "Missing plot file: " + string(plotPath));
            if hasFile
                fileInfo = dir(plotPath);
                testCase.verifyGreaterThan(fileInfo.bytes, 0, "Empty plot file: " + string(plotPath));
            end
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
n = numel(time_s);
signals.station_m = reshape(10 * (0:n-1), size(time_s));
signals.ax_mps2 = abs(ay) + 0.5;
signals.throttle = 0.2 + 0.01 * speed;
signals.steerSW_rad = 0.5 * yawRate;
signals.myDrL1_Nm = zeros(size(time_s));
signals.myDrL2_Nm = zeros(size(time_s));
signals.myDrR1_Nm = mz / 4;
signals.myDrR2_Nm = mz / 2;
signals.tireFxL1_N = 100 + mz;
signals.tireFxL2_N = 100 + mz;
signals.tireFxR1_N = 120 + mz;
signals.tireFxR2_N = 120 + mz;
signals.tireFyL1_N = 80 + 10 * ay;
signals.tireFyL2_N = 80 + 10 * ay;
signals.tireFyR1_N = 90 + 10 * ay;
signals.tireFyR2_N = 90 + 10 * ay;
signals.tireFzL1_N = 500 * ones(size(time_s));
signals.tireFzL2_N = 500 * ones(size(time_s));
signals.tireFzR1_N = 500 * ones(size(time_s));
signals.tireFzR2_N = 500 * ones(size(time_s));
end
