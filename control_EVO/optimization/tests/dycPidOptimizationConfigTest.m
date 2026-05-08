classdef dycPidOptimizationConfigTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addOptimizationPath(testCase)
            testFolder = fileparts(mfilename('fullpath'));
            optimizationFolder = fileparts(testFolder);
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(optimizationFolder));
        end
    end

    methods (Test)
        function testDefaultConfigUsesRepoPaths(testCase)
            cfg = dyc_pid_optimization_config();

            testCase.verifyEqual(cfg.modelName, 'DYC_1_9_test');
            testCase.verifyEqual(cfg.modelFolder, fullfile(cfg.repoRoot, 'control_EVO'));
            testCase.verifyEqual(cfg.modelPath, fullfile(cfg.modelFolder, 'DYC_1_9_test.slx'));
            testCase.verifyEqual(cfg.simfilePath, fullfile(cfg.modelFolder, 'simfile.sim'));
            testCase.verifyEqual(cfg.pidBlock, 'DYC_1_9_test/EVO_Control_System/YawMomentControl/PID_YawMomentController');
            testCase.verifyEqual(cfg.yawMomentModeSwitchBlock, 'DYC_1_9_test/EVO_Control_System/YawMomentControl/YawMomentControlMode');
            testCase.verifyEqual(cfg.yawMomentPidSwitchValue, '1');
            testCase.verifyEqual(cfg.yawMomentPidGotoTag, 'Mz_pid');
            testCase.verifyEqual(cfg.metric.primaryMetric, 'carsim_station_stop_time');
            testCase.verifyEqual(cfg.metric.sStop_m, 245);
            testCase.verifyEqual(cfg.metric.finishStationTolerance_m, 1.0);
        end

        function testPidBoundsMatchDesign(testCase)
            cfg = dyc_pid_optimization_config();

            testCase.verifyEqual(cfg.pidBounds.Kp, [2000 12000]);
            testCase.verifyEqual(cfg.pidBounds.Ki, [0 800]);
            testCase.verifyEqual(cfg.pidBounds.Kd, [0 800]);
            testCase.verifyEqual(cfg.optimizer.maxObjectiveEvaluations, 60);
            testCase.verifyEqual(cfg.optimizer.initialRandomEvaluations, 10);
            testCase.verifyEqual(cfg.penalty.hardFailureObjective, 9999);
        end

        function testPreflightBayesoptCheckIsConfigurable(testCase)
            cfg = dyc_pid_optimization_config(struct('preflight', struct('bayesoptAvailableFcn', @() false)));

            testCase.verifyTrue(isfield(cfg.preflight, 'bayesoptAvailableFcn'));
            testCase.verifyTrue(isfield(cfg.preflight, 'modelInfoFcn'));
            testCase.verifyFalse(cfg.preflight.bayesoptAvailableFcn());
        end
    end
end
