classdef computeDycAutocrossMetricsTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addOptimizationPath(testCase)
            testFolder = fileparts(mfilename('fullpath'));
            optimizationFolder = fileparts(testFolder);
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(optimizationFolder));
        end
    end

    methods (Test)
        function testComputesLapTimeDeltaAndStabilityMetrics(testCase)
            cfg = dyc_autocross_comparison_config();
            offRun = localRun("dyc_off", "DYC 关闭", 1, 62.0, localSignals( ...
                [0 1 2], [0.30 -0.20 0.10], [0 0 0], [0.40 -0.20 0.30], [0 0 0], ...
                [2.0 -3.0 1.0], [14 15 14], [0 0 0]));
            onRun = localRun("dyc_on", "当前 PID DYC", 1, 60.0, localSignals( ...
                [0 1 2], [0.10 -0.05 0.05], [0 0 0], [0.10 -0.05 0.05], [0 0 0], ...
                [1.5 -2.0 1.0], [16 17 16], [0 80 -120]));

            result = compute_dyc_autocross_metrics([offRun onRun], cfg);

            testCase.verifyEqual(height(result.perRun), 2);
            testCase.verifyEqual(result.comparison.status, "valid");
            testCase.verifyEqual(result.comparison.lapTimeDelta_s, -2.0, 'AbsTol', 1e-12);
            testCase.verifyEqual(result.comparison.lapTimeDelta_pct, -100 * 2 / 62, 'AbsTol', 1e-10);

            expectedDeltaColumns = {'baselineCaseId','testCaseId','status','failureReason', ...
                'lapTimeDelta_s','lapTimeDelta_pct','yawRateRmseDelta','yawRateMaeDelta', ...
                'yawRatePeakErrorDelta','lateralErrorRmseDelta','lateralErrorPeakDelta', ...
                'ayPeakAbsDelta','ayRmsDelta','ayStdDelta','axPeakAbsDelta','axRmsDelta', ...
                'meanSpeedDelta_mps', 'minSpeedDelta_mps','speedStdDelta_mps', ...
                'stationEndDelta_m','throttleMeanDelta','throttlePeakDelta', ...
                'steerRmsDelta_rad','steerPeakAbsDelta_rad','tireUtilPeakDelta', ...
                'tireUtilMeanDelta','wheelTorqueSpreadRmsDelta_Nm', ...
                'wheelTorqueSpreadPeakDelta_Nm', ...
                'mzRmsDelta_Nm','mzPeakAbsDelta_Nm', ...
                'mzAbsIntegralDelta_Nms','interventionRatioDelta'};
            testCase.verifyEqual(height(result.delta), 1);
            testCase.verifyTrue(all(ismember(expectedDeltaColumns, result.delta.Properties.VariableNames)));
            delta = result.delta(1, :);
            testCase.verifyEqual(delta.baselineCaseId, "dyc_off");
            testCase.verifyEqual(delta.testCaseId, "dyc_on");
            testCase.verifyEqual(delta.status, "valid");
            testCase.verifyEqual(delta.failureReason, "");
            testCase.verifyLessThan(delta.yawRateRmseDelta, 0);
            testCase.verifyLessThan(delta.yawRateMaeDelta, 0);
            testCase.verifyLessThan(delta.lateralErrorRmseDelta, 0);
            testCase.verifyLessThan(delta.lateralErrorPeakDelta, 0);
            testCase.verifyGreaterThan(delta.meanSpeedDelta_mps, 0);
            testCase.verifyGreaterThan(delta.minSpeedDelta_mps, 0);
            testCase.verifyLessThan(delta.axPeakAbsDelta, 0);
            testCase.verifyGreaterThan(delta.throttleMeanDelta, 0);
            testCase.verifyLessThan(delta.steerRmsDelta_rad, 0);
            testCase.verifyGreaterThan(delta.tireUtilPeakDelta, 0);
            testCase.verifyGreaterThan(delta.wheelTorqueSpreadPeakDelta_Nm, 0);
            testCase.verifyGreaterThan(delta.mzRmsDelta_Nm, 0);
            testCase.verifyGreaterThan(delta.mzPeakAbsDelta_Nm, 0);
            testCase.verifyGreaterThan(delta.mzAbsIntegralDelta_Nms, 0);
            testCase.verifyGreaterThan(delta.interventionRatioDelta, 0);
            testCase.verifyEqual(result.comparison.lapTimeDelta_s, delta.lapTimeDelta_s, 'AbsTol', 1e-12);
            testCase.verifyEqual(result.comparison.lapTimeDelta_pct, delta.lapTimeDelta_pct, 'AbsTol', 1e-10);

            offSummary = result.summary(result.summary.caseId == "dyc_off", :);
            onSummary = result.summary(result.summary.caseId == "dyc_on", :);
            testCase.verifyLessThan(onSummary.yawRateRmse, offSummary.yawRateRmse);
            testCase.verifyLessThan(onSummary.lateralErrorRmse, offSummary.lateralErrorRmse);
            testCase.verifyGreaterThan(onSummary.meanSpeed_mps, offSummary.meanSpeed_mps);
            testCase.verifyLessThan(onSummary.axPeakAbs, offSummary.axPeakAbs);
            testCase.verifyGreaterThan(onSummary.tireUtilPeak, offSummary.tireUtilPeak);
            testCase.verifyGreaterThan(onSummary.wheelTorqueSpreadPeak_Nm, offSummary.wheelTorqueSpreadPeak_Nm);
            testCase.verifyGreaterThan(onSummary.mzRms_Nm, 0);
            testCase.verifyGreaterThan(onSummary.interventionRatio, 0);
        end

        function testInvalidRunsAreExcludedFromSummary(testCase)
            cfg = dyc_autocross_comparison_config();
            validRun = localRun("dyc_off", "DYC 关闭", 1, 62.0, localSignals( ...
                [0 1], [0.1 0.2], [0 0], [0.1 0.2], [0 0], [1 2], [10 11], [0 0]));
            invalidRun = localRun("dyc_off", "DYC 关闭", 2, 40.0, localSignals( ...
                [0 1], [3 3], [0 0], [2 2], [0 0], [8 8], [20 20], [100 100]));
            invalidRun.status = "invalid";
            invalidRun.failureReason = "测试失败运行";

            result = compute_dyc_autocross_metrics([validRun invalidRun], cfg);

            offSummary = result.summary(result.summary.caseId == "dyc_off", :);
            testCase.verifyEqual(offSummary.validRunCount, 1);
            testCase.verifyEqual(offSummary.lapTime_s, 62.0, 'AbsTol', 1e-12);
        end

        function testMissingSignalsProduceNanButKeepLaptime(testCase)
            cfg = dyc_autocross_comparison_config();
            run = localRun("dyc_on", "当前 PID DYC", 1, 60.0, struct());

            result = compute_dyc_autocross_metrics(run, cfg);

            onRun = result.perRun(result.perRun.caseId == "dyc_on", :);
            testCase.verifyEqual(onRun.lapTime_s, 60.0, 'AbsTol', 1e-12);
            testCase.verifyTrue(isnan(onRun.yawRateRmse));
            testCase.verifyTrue(isnan(onRun.lateralErrorRmse));
        end

        function testAggregatesRepeatRunsWithStd(testCase)
            cfg = dyc_autocross_comparison_config();
            first = localRun("dyc_on", "当前 PID DYC", 1, 60.0, localSignals( ...
                [0 1], [0.1 0.1], [0 0], [0.1 0.1], [0 0], [1 1], [10 10], [0 100]));
            second = localRun("dyc_on", "当前 PID DYC", 2, 62.0, localSignals( ...
                [0 1], [0.2 0.2], [0 0], [0.2 0.2], [0 0], [2 2], [12 12], [0 100]));

            result = compute_dyc_autocross_metrics([first second], cfg);

            onSummary = result.summary(result.summary.caseId == "dyc_on", :);
            testCase.verifyEqual(onSummary.validRunCount, 2);
            testCase.verifyEqual(onSummary.lapTime_s, 61.0, 'AbsTol', 1e-12);
            testCase.verifyEqual(onSummary.lapTimeStd_s, std([60; 62]), 'AbsTol', 1e-12);
            testCase.verifyEqual(onSummary.yawRateRmse, 0.15, 'AbsTol', 1e-12);
        end

        function testNanOnlyMzProducesNanIntegral(testCase)
            cfg = dyc_autocross_comparison_config();
            run = localRun("dyc_on", "当前 PID DYC", 1, 60.0, localSignals( ...
                [0 1 2], [0.1 0.1 0.1], [0 0 0], [0.1 0.1 0.1], [0 0 0], ...
                [1 1 1], [10 10 10], [NaN NaN NaN]));

            result = compute_dyc_autocross_metrics(run, cfg);

            onRun = result.perRun(result.perRun.caseId == "dyc_on", :);
            testCase.verifyTrue(isnan(onRun.mzAbsIntegral_Nms));
        end

        function testComparisonInvalidWhenAggregateLapTimesAreNan(testCase)
            cfg = dyc_autocross_comparison_config();
            offRun = localRun("dyc_off", "DYC 关闭", 1, NaN, localSignals( ...
                [0 1], [0.1 0.1], [0 0], [0.1 0.1], [0 0], [1 1], [10 10], [0 0]));
            onRun = localRun("dyc_on", "当前 PID DYC", 1, NaN, localSignals( ...
                [0 1], [0.1 0.1], [0 0], [0.1 0.1], [0 0], [1 1], [10 10], [0 100]));

            result = compute_dyc_autocross_metrics([offRun onRun], cfg);

            testCase.verifyEqual(result.comparison.status, "invalid");
            testCase.verifyTrue(contains(result.comparison.failureReason, "lap time"));
            testCase.verifyTrue(isnan(result.comparison.lapTimeDelta_s));
            testCase.verifyTrue(isnan(result.comparison.lapTimeDelta_pct));
            testCase.verifyEqual(result.delta.status, "invalid");
            testCase.verifyTrue(isnan(result.delta.yawRateRmseDelta));
            testCase.verifyTrue(isnan(result.delta.mzAbsIntegralDelta_Nms));
        end

        function testComparisonInvalidWhenBaselineLapTimeIsZero(testCase)
            cfg = dyc_autocross_comparison_config();
            offRun = localRun("dyc_off", "DYC 关闭", 1, 0, localSignals( ...
                [0 1], [0.1 0.1], [0 0], [0.1 0.1], [0 0], [1 1], [10 10], [0 0]));
            onRun = localRun("dyc_on", "当前 PID DYC", 1, 60, localSignals( ...
                [0 1], [0.1 0.1], [0 0], [0.1 0.1], [0 0], [1 1], [10 10], [0 100]));

            result = compute_dyc_autocross_metrics([offRun onRun], cfg);

            testCase.verifyEqual(result.comparison.status, "invalid");
            testCase.verifyTrue(contains(result.comparison.failureReason, "baseline"));
            testCase.verifyTrue(isnan(result.comparison.lapTimeDelta_s));
            testCase.verifyTrue(isnan(result.comparison.lapTimeDelta_pct));
            testCase.verifyEqual(result.delta.status, "invalid");
            testCase.verifyTrue(isnan(result.delta.lapTimeDelta_s));
            testCase.verifyTrue(isnan(result.delta.interventionRatioDelta));
        end
    end
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
