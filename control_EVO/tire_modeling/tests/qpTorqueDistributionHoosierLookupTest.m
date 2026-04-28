classdef qpTorqueDistributionHoosierLookupTest < matlab.unittest.TestCase
    %qpTorqueDistributionHoosierLookupTest 验证QP横摆差动扭矩分配接入Hoosier查表。

    properties
        ControlFolder
        ModelingFolder
        Model
    end

    methods (TestClassSetup)
        function addControlPaths(testCase)
            % QP测试同时需要控制根目录和轮胎建模目录。
            testFolder = fileparts(mfilename('fullpath'));
            testCase.ModelingFolder = fileparts(testFolder);
            testCase.ControlFolder = fileparts(testCase.ModelingFolder);
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(testCase.ControlFolder));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(testCase.ModelingFolder));
        end

        function buildModelOnce(testCase)
            % Hoosier模型由原始TTC数据生成一次，后续用例复用以减少测试时间。
            projectRoot = fileparts(testCase.ControlFolder);
            testCase.Model = build_hoosier43075_model(projectRoot);
        end
    end

    methods (TestMethodSetup)
        function forceHoosierMode(~)
            % 每条用例开始前强制使用Hoosier查表，避免默认Pacejka配置影响旧链路验证。
            setenv('TIRE_CONTROL_LOOKUP_MODE', 'hoosier');
            setenv('TIRE_CONTROL_MODEL_FILE', '');
            setenv('PACEJKA_CONTROL_MODEL_FILE', '');
            clear QP_TorqueDistribution DYC_torque_request_manager
        end
    end

    methods (TestMethodTeardown)
        function resetSFunctionState(~)
            % 清除S-Function持久状态和环境变量，保证测试互不污染。
            clear QP_TorqueDistribution DYC_torque_request_manager
            setenv('HOOSIER43075_MODEL_FILE', '');
            setenv('TIRE_CONTROL_LOOKUP_MODE', '');
            setenv('TIRE_CONTROL_MODEL_FILE', '');
            setenv('PACEJKA_CONTROL_MODEL_FILE', '');
        end
    end

    methods (Test)
        function testDeltaOutputFiniteAndCandidateInsideLookupLimit(testCase)
            % 正常横摆需求下，候选最终扭矩应落在轮胎查表和电机限扭边界内。
            baseTorque = 25 * ones(4, 1);
            u = testCase.sampleInput(0, 40, 1.0, [80; 100; 90; 110], ...
                baseTorque);
            output = testCase.runTorqueDistributionOutput(u);
            deltaTorque = output(1:4);
            candidateTorque = testCase.candidateTorque(output, u);
            expectedLimit = testCase.expectedCombinedLimit(u);

            testCase.verifySize(deltaTorque, [4 1]);
            testCase.verifyTrue(all(isfinite(deltaTorque)));
            testCase.verifyLessThanOrEqual(abs(candidateTorque), ...
                expectedLimit + 1e-6);
            testCase.verifyLessThanOrEqual(abs(candidateTorque), ...
                400 * ones(4, 1) + 1e-6);
        end

        function testDeltaOutputRespectsSlewLimit(testCase)
            % 连续两拍差动扭矩变化量不能超过QP内部限斜率。
            u = testCase.sampleInput(0, 600, 1.0, zeros(4, 1));
            firstDelta = testCase.runTorqueDistribution(u);
            secondOutput = QP_TorqueDistribution(0.0005, firstDelta, u, 3);
            secondDelta = secondOutput(1:4);
            maxDeltaStep = testCase.maxTorqueStep();

            testCase.verifyLessThanOrEqual(abs(firstDelta), ...
                maxDeltaStep * ones(4, 1) + 1e-6);
            testCase.verifyGreaterThan(max(abs(firstDelta)), 0.1);
            testCase.verifyLessThanOrEqual(abs(secondDelta(:) - firstDelta(:)), ...
                maxDeltaStep * ones(4, 1) + 1e-6);
        end

        function testLimitShrinkUsesPhysicalLimitPriority(testCase)
            % 轮胎能力突然降低时，应优先收缩到新的物理上限内。
            u = testCase.sampleInput(0, 600, 1.0, zeros(4, 1));
            beforeShrink = testCase.runForSteps(u, 160);
            testCase.verifyGreaterThan(max(abs(beforeShrink)), 10);

            shrink = u;
            shrink(1:4) = [70; 70; 70; 70];
            shrink(5:8) = [80; 80; 80; 80];
            afterOutput = QP_TorqueDistribution(160 * 0.0005, beforeShrink, ...
                shrink, 3);
            afterDelta = afterOutput(1:4);
            candidateTorque = testCase.candidateTorque(afterOutput, shrink);
            roadLimit = testCase.roadMuLimit(shrink);

            testCase.verifyTrue(all(isfinite(afterDelta)));
            testCase.verifyLessThanOrEqual(abs(candidateTorque), roadLimit + 1e-6);
            testCase.verifyGreaterThan(afterOutput(8), 0);
        end

        function testInvalidYawDemandFallsBackToLastDelta(testCase)
            u = testCase.sampleInput(0, 600, 1.0, zeros(4, 1));
            lastDelta = testCase.runForSteps(u, 20);

            invalidDemand = u;
            invalidDemand(11) = NaN;
            output = QP_TorqueDistribution(20 * 0.0005, lastDelta, ...
                invalidDemand, 3);
            deltaTorque = output(1:4);

            testCase.verifySize(output, [10 1]);
            testCase.verifyTrue(all(isfinite(output)));
            testCase.verifyEqual(deltaTorque(:), lastDelta(:), 'AbsTol', 1e-12);
        end

        function testZeroYawDemandOutputsZeroDelta(testCase)
            baseTorque = [5; 5; 5; 5];
            u = testCase.sampleInput(0, 0, 1.0, zeros(4, 1), baseTorque);
            deltaTorque = testCase.runForSteps(u, 20);

            testCase.verifyEqual(deltaTorque(:), zeros(4, 1), 'AbsTol', 1e-9);
        end

        function testYawDemandAppliesBalancedDelta(testCase)
            baseTorque = [30; 30; 30; 30];
            mzReq = 80;
            u = testCase.sampleInput(0, mzReq, 1.0, zeros(4, 1), baseTorque);
            output = testCase.runForStepsOutput(u, 220);
            deltaTorque = output(1:4);
            veh = DYC_vehicle_params;
            yawArm = [-veh.tf/(2*veh.r), veh.tf/(2*veh.r), ...
                -veh.tr/(2*veh.r), veh.tr/(2*veh.r)];

            testCase.verifyEqual(sum(deltaTorque), 0, 'AbsTol', 1e-2);
            testCase.verifyEqual(yawArm * deltaTorque(:), mzReq, 'AbsTol', 1);
            testCase.verifyEqual(output(6), mzReq, 'AbsTol', 1);
            testCase.verifyEqual(output(9), mzReq, 'AbsTol', 1e-9);
        end

        function testYawDemandCanSacrificeLimitedDriveTorqueWhenPositiveHeadroomIsExhausted(testCase)
            probe = testCase.sampleInput(0, 0, 1.0, zeros(4, 1), ...
                zeros(4, 1));
            physicalLimit = testCase.expectedCombinedLimit(probe);
            baseTorque = [0.25 * physicalLimit(1); ...
                          0.98 * physicalLimit(2); ...
                          0.25 * physicalLimit(3); ...
                          0.98 * physicalLimit(4)];
            mzReq = 140;
            u = testCase.sampleInput(0, mzReq, 1.0, zeros(4, 1), ...
                baseTorque);

            output = testCase.runForStepsOutput(u, 260);
            deltaTorque = output(1:4);
            totalDelta = sum(deltaTorque);
            achievedYawMoment = output(6);
            candidateTorque = testCase.candidateTorque(output, u);
            expectedLimit = testCase.expectedCombinedLimit(u);

            testCase.verifyLessThan(totalDelta, -10);
            testCase.verifyGreaterThan(achievedYawMoment, 70);
            testCase.verifyGreaterThan(achievedYawMoment, 0.55 * mzReq);
            testCase.verifyGreaterThanOrEqual(totalDelta, ...
                -testCase.expectedTotalCutLimit(baseTorque) - 1e-6);
            testCase.verifyLessThanOrEqual(totalDelta, ...
                testCase.expectedTotalAddLimit(baseTorque) + 1e-6);
            testCase.verifyLessThanOrEqual(abs(candidateTorque), ...
                expectedLimit + 1e-6);
        end

        function testTotalDeltaStaysInsideStrategyBoundsUnderLargeYawDemand(testCase)
            probe = testCase.sampleInput(0, 0, 1.0, zeros(4, 1), ...
                zeros(4, 1));
            physicalLimit = testCase.expectedCombinedLimit(probe);
            baseTorque = [0.30 * physicalLimit(1); ...
                          0.97 * physicalLimit(2); ...
                          0.30 * physicalLimit(3); ...
                          0.97 * physicalLimit(4)];
            mzReq = 900;
            u = testCase.sampleInput(0, mzReq, 1.0, zeros(4, 1), ...
                baseTorque);

            output = testCase.runForStepsOutput(u, 320);
            totalDelta = sum(output(1:4));

            testCase.verifyGreaterThanOrEqual(totalDelta, ...
                -testCase.expectedTotalCutLimit(baseTorque) - 1e-6);
            testCase.verifyLessThanOrEqual(totalDelta, ...
                testCase.expectedTotalAddLimit(baseTorque) + 1e-6);
            testCase.verifyGreaterThan(abs(output(5)), 100);
        end

        function testCandidateTorqueInsideRemainingBounds(testCase)
            baseTorque = 45 * ones(4, 1);
            u = testCase.sampleInput(0, 120, 0.70, [20; 20; 20; 20], ...
                baseTorque);
            output = testCase.runForStepsOutput(u, 220);
            candidateTorque = testCase.candidateTorque(output, u);
            expectedLimit = testCase.expectedCombinedLimit(u);

            testCase.verifyLessThanOrEqual(abs(candidateTorque), ...
                expectedLimit + 1e-6);
        end

        function testRoadMuStillCapsLookupLimit(testCase)
            baseTorque = 10 * ones(4, 1);
            u = testCase.sampleInput(0, 40, 0.35, [20; 20; 20; 20], ...
                baseTorque);
            output = testCase.runTorqueDistributionOutput(u);
            candidateTorque = testCase.candidateTorque(output, u);
            roadLimit = testCase.roadMuLimit(u);

            testCase.verifyLessThanOrEqual(abs(candidateTorque), roadLimit + 1e-6);
        end

        function testHigherLateralForceReducesAvailableTorque(testCase)
            lowFy = testCase.expectedCombinedLimit(testCase.sampleInput(0, 0, ...
                1.0, zeros(4, 1)));
            highFy = testCase.expectedCombinedLimit(testCase.sampleInput(0, 0, ...
                1.0, [300; 300; 300; 300]));

            testCase.verifyLessThan(highFy, lowFy);
        end

        function testMissingLookupFallsBackToRoadMu(testCase)
            missingModel = fullfile(tempdir, 'missing_hoosier43075_model.mat');
            setenv('HOOSIER43075_MODEL_FILE', missingModel);
            clear QP_TorqueDistribution

            u = testCase.sampleInput(0, 25, 0.75, [50; 50; 50; 50]);
            output = testCase.runTorqueDistributionOutput(u);
            candidateTorque = testCase.candidateTorque(output, u);
            roadLimit = testCase.roadMuLimit(u);

            testCase.verifyTrue(all(isfinite(output)));
            testCase.verifyLessThanOrEqual(abs(candidateTorque), roadLimit + 1e-6);
        end

        function testBaseDemandIsNotFoldedIntoQpTarget(testCase)
            % QP目标只分配DYC差动扭矩，基础扭矩不应重复进入横摆力矩目标。
            baseTorque = 300 * ones(4, 1);
            wheelRpm = 1800 * ones(4, 1);
            u = testCase.sampleInput(0, 0, 1.0, zeros(4, 1), baseTorque, ...
                1, wheelRpm);

            output = testCase.runTorqueDistributionOutput(u);

            testCase.verifyEqual(output(1:4), zeros(4, 1), 'AbsTol', 1e-9);
            testCase.verifyEqual(output(5), 0, 'AbsTol', 1e-9);
            testCase.verifyEqual(output(9), 0, 'AbsTol', 1e-9);
            testCase.verifyLessThan(output(7), 0);
        end

        function testHighWheelSpeedMotorLimitBoundsCandidateTorque(testCase)
            wheelRpm = 1000 * ones(4, 1);
            u = testCase.sampleInput(0, 80, 1.0, zeros(4, 1), ...
                zeros(4, 1), 1, wheelRpm);
            output = testCase.runForStepsOutput(u, 220);
            candidateTorque = testCase.candidateTorque(output, u);
            motorLimit = DYC_motor_wheel_torque_limit(wheelRpm);

            testCase.verifyLessThanOrEqual(abs(candidateTorque), motorLimit + 1e-6);
        end

        function testPowerMarginDiagnosticIsBeforeTcs(testCase)
            throttle = 0.2;
            wheelRpm = 500 * ones(4, 1);
            u = testCase.sampleInput(0, 0, 1.0, zeros(4, 1), ...
                300 * ones(4, 1), throttle, wheelRpm);

            output = testCase.runTorqueDistributionOutput(u);

            testCase.verifyEqual(output(1:4), zeros(4, 1), 'AbsTol', 1e-9);
            testCase.verifyLessThan(output(7), 0);
        end

        function testBrakeEnableIsAppliedBeforeQpCandidate(testCase)
            [driveOutput, brakeOutput, rawBase, brakeBase, driveBase] = ...
                testCase.runQpThroughBrakeRequest();
            driveCandidate = driveBase + driveOutput(1:4);
            brakeCandidate = brakeBase + brakeOutput(1:4);
            limitedAfterQp = DYC_apply_motor_limits(brakeCandidate, ...
                1, zeros(4, 1));

            testCase.verifyGreaterThan(sum(rawBase), 100);
            testCase.verifyLessThan(sum(brakeBase), sum(rawBase));
            testCase.verifyLessThan(sum(brakeCandidate), sum(driveCandidate));
            testCase.verifyEqual(limitedAfterQp, brakeCandidate, 'AbsTol', 1e-6);
        end

        function testInfeasibleYawDemandExposesYawSlack(testCase)
            u = testCase.sampleInput(0, 5000, 1.0, zeros(4, 1));

            output = testCase.runTorqueDistributionOutput(u);

            testCase.verifyGreaterThan(abs(output(5)), 1000);
        end

        function testSFunctionInterfaceRemainsCompatible(testCase)
            [sys, x0, str, ts] = QP_TorqueDistribution([], [], [], 0);

            testCase.verifyEqual(sys(3), 10); % 输出数量
            testCase.verifyEqual(sys(4), 21); % 输入数量
            testCase.verifyEqual(x0, zeros(4, 1));
            testCase.verifyEmpty(str);
            testCase.verifyEqual(ts, [0.0005 0]);
        end
    end

    methods
        function u = sampleInput(~, fxReq, mzReq, roadMu, fyVector, ...
                baseTorque, throttle, wheelRpm)
            % 构造21路QP输入向量，通道顺序与S-Function接口保持一致。
            if nargin < 6
                baseTorque = zeros(4, 1);
            end
            if nargin < 7
                throttle = 1;
            end
            if nargin < 8
                wheelRpm = zeros(4, 1);
            end

            fz = [420; 390; 430; 400]; % [L1; R1; L2; R2]
            u = zeros(21, 1);
            u(1) = fyVector(1); % FyL1
            u(2) = fyVector(3); % FyL2
            u(3) = fyVector(2); % FyR1
            u(4) = fyVector(4); % FyR2
            u(5) = fz(1);       % FzL1
            u(6) = fz(3);       % FzL2
            u(7) = fz(2);       % FzR1
            u(8) = fz(4);       % FzR2
            u(9) = 0;
            u(10) = fxReq;
            u(11) = mzReq;
            u(12) = roadMu;
            u(13:16) = baseTorque(:);
            u(17) = throttle;
            u(18:21) = wheelRpm(:);
        end

        function deltaTorque = runTorqueDistribution(~, u)
            % 初始化S-Function后取前4路差动扭矩输出。
            QP_TorqueDistribution([], [], [], 0);
            output = QP_TorqueDistribution(0, zeros(4, 1), u, 3);
            deltaTorque = output(1:4);
        end

        function output = runTorqueDistributionOutput(~, u)
            % 返回完整10路输出，便于检查诊断量。
            QP_TorqueDistribution([], [], [], 0);
            output = QP_TorqueDistribution(0, zeros(4, 1), u, 3);
            output = output(:);
        end

        function deltaTorque = runForSteps(~, u, stepCount)
            % 连续运行多拍，用于观察限斜率和物理边界收敛过程。
            QP_TorqueDistribution([], [], [], 0);
            deltaTorque = zeros(4, 1);
            for i = 1:stepCount
                output = QP_TorqueDistribution((i - 1) * 0.0005, ...
                    deltaTorque, u, 3);
                deltaTorque = output(1:4);
            end
        end

        function output = runForStepsOutput(~, u, stepCount)
            QP_TorqueDistribution([], [], [], 0);
            deltaTorque = zeros(4, 1);
            output = zeros(10, 1);
            for i = 1:stepCount
                output = QP_TorqueDistribution((i - 1) * 0.0005, ...
                    deltaTorque, u, 3);
                output = output(:);
                deltaTorque = output(1:4);
            end
        end

        function candidateTorque = candidateTorque(~, output, u)
            % 将差动扭矩叠加基础扭矩，得到QP候选最终轮端扭矩。
            candidateTorque = u(13:16) + output(1:4);
        end

        function limit = expectedCombinedLimit(testCase, u)
            [fz, fy] = testCase.wheelLoadsAndLateralForces(u);
            lookup = lookup_hoosier43075_limits(fz, fy, [], [], testCase.Model);
            limit = min([lookup.T_limit(:), testCase.roadMuLimit(u), ...
                DYC_motor_wheel_torque_limit(u(18:21)), ...
                400 * ones(4, 1)], [], 2);
        end

        function limit = roadMuLimit(testCase, u)
            [fz, fy] = testCase.wheelLoadsAndLateralForces(u);
            roadMu = max(u(12), 0.05);
            veh = DYC_vehicle_params;
            limit = veh.r * sqrt(max((roadMu .* fz).^2 - fy.^2, 0));
        end

        function [fz, fy] = wheelLoadsAndLateralForces(~, u)
            fz = max([u(5); u(7); u(6); u(8)], 1);
            fy = [u(1); u(3); u(2); u(4)];
        end

        function step = maxTorqueStep(~)
            step = 1500 * 0.0005;
        end

        function limit = expectedTotalCutLimit(~, baseTorque)
            baseDriveTorque = sum(max(baseTorque(:), 0));
            limit = min(160, 0.25 * baseDriveTorque + 20);
        end

        function limit = expectedTotalAddLimit(~, baseTorque)
            baseDriveTorque = sum(max(baseTorque(:), 0));
            limit = min(80, 0.10 * baseDriveTorque);
        end

        function [driveOutput, brakeOutput, rawBase, brakeBase, driveBase] = ...
                runQpThroughBrakeRequest(testCase)
            % 复现油门-刹车序列，验证基础扭矩管理器在QP前完成平滑使能。
            clear QP_TorqueDistribution DYC_torque_request_manager
            DYC_torque_request_manager(0, 0, true);
            deltaTorque = zeros(4, 1);
            output = zeros(10, 1);
            rawBase = zeros(4, 1);
            baseAlloc = zeros(4, 1);
            for i = 1:260
                [baseAlloc, rawBase] = DYC_torque_request_manager(1, 0);
                u = testCase.sampleInput(0, 0, 1.0, zeros(4, 1), ...
                    baseAlloc, 1, zeros(4, 1));
                output = QP_TorqueDistribution((i - 1) * 0.0005, ...
                    deltaTorque, u, 3);
                deltaTorque = output(1:4);
            end
            driveOutput = output(:);
            driveBase = baseAlloc;

            for i = 1:80
                [baseAlloc, rawBase] = DYC_torque_request_manager(1, 1);
                u = testCase.sampleInput(0, 0, 1.0, zeros(4, 1), ...
                    baseAlloc, 1, zeros(4, 1));
                output = QP_TorqueDistribution((260 + i - 1) * 0.0005, ...
                    deltaTorque, u, 3);
                deltaTorque = output(1:4);
            end
            brakeOutput = output(:);
            brakeBase = baseAlloc;
        end
    end
end
