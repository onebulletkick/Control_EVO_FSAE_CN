classdef baseMotorTorqueTest < matlab.unittest.TestCase
    %baseMotorTorqueTest 验证踏板基础扭矩和最终电机限扭。

    methods (TestClassSetup)
        function addControlPath(testCase)
            % 测试只依赖control_EVO根目录中的公共函数。
            testFolder = fileparts(mfilename('fullpath'));
            controlFolder = fileparts(fileparts(testFolder));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(controlFolder));
        end
    end

    methods (Test)
        function testZeroThrottleOutputsZeroBaseTorque(testCase)
            % 油门为0时，基础驱动扭矩必须完全为0。
            baseTorque = DYC_base_motor_torque(0);

            testCase.verifyEqual(baseTorque, zeros(4, 1), 'AbsTol', 1e-12);
        end

        function testFullThrottleOutputsDriverIntent(testCase)
            % 满油门输出电机峰值扭矩经减速比和传动效率折算后的轮端扭矩。
            baseTorque = DYC_base_motor_torque(1);
            expected = 21 * 11 * 0.99;

            testCase.verifyEqual(baseTorque, expected * ones(4, 1), ...
                'RelTol', 1e-12);
        end

        function testHalfThrottleOutputsHalfBaseTorque(testCase)
            baseTorque = DYC_base_motor_torque(0.5);
            expected = 0.5 * 21 * 11 * 0.99;

            testCase.verifyEqual(baseTorque, expected * ones(4, 1), ...
                'RelTol', 1e-12);
        end

        function testWheelSpeedDoesNotChangeBaseTorque(testCase)
            lowSpeedBase = DYC_base_motor_torque(0.7, zeros(4, 1));
            highSpeedBase = DYC_base_motor_torque(0.7, 1200 * ones(4, 1));

            testCase.verifyEqual(highSpeedBase, lowSpeedBase, ...
                'AbsTol', 1e-12);
        end

        function testFinalLimiterDoesNotClipLowSpeedCommand(testCase)
            command = 100 * ones(4, 1);
            limited = DYC_apply_motor_limits(command, 1, zeros(4, 1));

            testCase.verifyEqual(limited, command, 'AbsTol', 1e-12);
        end

        function testFinalLimiterReducesHighSpeedCommand(testCase)
            command = 200 * ones(4, 1);
            limited = DYC_apply_motor_limits(command, 1, 1000 * ones(4, 1));

            testCase.verifyLessThan(limited, command);
            testCase.verifyGreaterThan(limited, zeros(4, 1));
        end

        function testFinalLimiterZerosOverspeedDriveTorque(testCase)
            command = 200 * ones(4, 1);
            limited = DYC_apply_motor_limits(command, 1, 1800 * ones(4, 1));

            testCase.verifyEqual(limited, zeros(4, 1), 'AbsTol', 1e-12);
        end

        function testFinalLimiterCapsTotalPositiveDrivePower(testCase)
            % 最终限扭器需要保证四轮正驱动功率总和不超过油门对应功率。
            throttle = 0.5;
            wheelRpm = 500 * ones(4, 1);
            command = 300 * ones(4, 1);
            limited = DYC_apply_motor_limits(command, throttle, wheelRpm);
            wheelOmega = abs(wheelRpm) * (2*pi/60);
            drivePower = sum(max(limited, 0) .* wheelOmega);

            testCase.verifyLessThanOrEqual(drivePower, throttle * 78e3 + 1e-6);
        end

        function testFinalLimiterKeepsNegativeTorqueSymmetric(testCase)
            command = -100 * ones(4, 1);
            limited = DYC_apply_motor_limits(command, 0, zeros(4, 1));

            testCase.verifyEqual(limited, command, 'AbsTol', 1e-12);
        end
    end
end
