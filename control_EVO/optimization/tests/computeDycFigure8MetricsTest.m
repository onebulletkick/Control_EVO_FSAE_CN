classdef computeDycFigure8MetricsTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addOptimizationPath(testCase)
            testFolder = fileparts(mfilename('fullpath'));
            optimizationFolder = fileparts(testFolder);
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(optimizationFolder));
        end
    end

    methods (Test)
        function testSegmentsLeftRightAndTransitionEvidence(testCase)
            cfg = dyc_figure8_comparison_config();
            offRun = localRun("dyc_off", 62.0, localFigure8Signals(false, true));
            onRun = localRun("dyc_on", 61.9, localFigure8Signals(true, true));

            result = compute_dyc_figure8_metrics([offRun onRun], cfg);

            testCase.verifyEqual(result.comparison.status, "valid");
            testCase.verifyTrue(isfield(result, 'figure8Segments'));
            testCase.verifyTrue(isfield(result, 'figure8SegmentDelta'));
            testCase.verifyTrue(isfield(result, 'figure8Evidence'));
            testCase.verifyEqual(sort(unique(result.figure8Segments.segmentType)), ...
                ["left"; "right"; "transition"]);
            testCase.verifyEqual(result.figure8Evidence.segmentSource, "ay_mps2");
            testCase.verifyEqual(result.figure8Evidence.status, "valid");

            leftDelta = result.figure8SegmentDelta(result.figure8SegmentDelta.segmentType == "left", :);
            rightDelta = result.figure8SegmentDelta(result.figure8SegmentDelta.segmentType == "right", :);
            transitionDelta = result.figure8SegmentDelta(result.figure8SegmentDelta.segmentType == "transition", :);
            testCase.verifyLessThan(leftDelta.lateralErrorRmseDelta, 0);
            testCase.verifyLessThan(rightDelta.lateralErrorRmseDelta, 0);
            testCase.verifyLessThan(leftDelta.tireUtilPeakDelta, 0);
            testCase.verifyLessThan(rightDelta.tireUtilPeakDelta, 0);
            testCase.verifyGreaterThanOrEqual(transitionDelta.sampleCountDelta, 0);
            testCase.verifyGreaterThan(leftDelta.mzPeakAbsDelta_Nm, 0);
            testCase.verifyGreaterThan(rightDelta.wheelTorqueSpreadPeakDelta_Nm, 0);
        end

        function testFallsBackToYawRateWhenAyMissing(testCase)
            cfg = dyc_figure8_comparison_config();
            offSignals = localFigure8Signals(false, false);
            onSignals = localFigure8Signals(true, false);
            offSignals.ay_mps2 = [];
            onSignals.ay_mps2 = [];

            result = compute_dyc_figure8_metrics([ ...
                localRun("dyc_off", 62.0, offSignals), ...
                localRun("dyc_on", 61.9, onSignals)], cfg);

            testCase.verifyEqual(result.figure8Evidence.status, "valid");
            testCase.verifyEqual(result.figure8Evidence.segmentSource, "yawRate_radps");
            testCase.verifyTrue(any(result.figure8Segments.segmentType == "left"));
            testCase.verifyTrue(any(result.figure8Segments.segmentType == "right"));
        end

        function testMissingTurnSignalsMarksSegmentationUnavailable(testCase)
            cfg = dyc_figure8_comparison_config();
            offSignals = localFigure8Signals(false, false);
            onSignals = localFigure8Signals(true, false);
            offSignals.ay_mps2 = [];
            offSignals.yawRate_radps = [];
            onSignals.ay_mps2 = [];
            onSignals.yawRate_radps = [];

            result = compute_dyc_figure8_metrics([ ...
                localRun("dyc_off", 62.0, offSignals), ...
                localRun("dyc_on", 61.9, onSignals)], cfg);

            testCase.verifyEqual(result.figure8Evidence.status, "unavailable");
            testCase.verifyTrue(contains(result.figure8Evidence.failureReason, "分段证据不可用"));
            testCase.verifyEqual(height(result.figure8Segments), 0);
        end
    end
end

function run = localRun(caseId, lapTime_s, signals)
run = struct();
run.caseId = caseId;
run.displayName = caseId;
run.repeatIndex = 1;
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

function signals = localFigure8Signals(isDycOn, includeAy)
t = (0:0.2:10)';
turn = sin(2 * pi * t / 5);
if includeAy
    ay = 5.0 * turn;
else
    ay = [];
end

signals = struct();
signals.time_s = t;
signals.station_m = 12 * t;
signals.ay_mps2 = ay;
signals.yawRate_radps = 0.3 * turn;
signals.yawRateTarget_radps = 0.28 * turn;
signals.latTarget_m = zeros(size(t));
signals.ax_mps2 = 0.5 + 0.1 * cos(t);
signals.throttle = 0.25 + 0.02 * cos(t);
signals.steerSW_rad = 0.6 * turn;

if isDycOn
    signals.speed_mps = 12.2 + 0.7 * cos(t);
    signals.latVeh_m = 0.08 * sign(turn) + 0.03 * turn;
    signals.mz_Nm = 120 * turn;
    torque = 90 + 20 * abs(turn);
    tireScale = 0.78;
else
    signals.speed_mps = 12.0 + 0.6 * cos(t);
    signals.latVeh_m = 0.18 * sign(turn) + 0.06 * turn;
    signals.mz_Nm = 8 * turn;
    torque = 8 + 2 * abs(turn);
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
signals.tireFyL1_N = 400 * tireScale * abs(turn);
signals.tireFyL2_N = 380 * tireScale * abs(turn);
signals.tireFyR1_N = 390 * tireScale * abs(turn);
signals.tireFyR2_N = 370 * tireScale * abs(turn);
signals.tireFzL1_N = 500 * ones(size(t));
signals.tireFzL2_N = 500 * ones(size(t));
signals.tireFzR1_N = 500 * ones(size(t));
signals.tireFzR2_N = 500 * ones(size(t));
end
