classdef simpleLoadTransferDistributionTest < matlab.unittest.TestCase
    %simpleLoadTransferDistributionTest 验证简单载荷转移分配输出语义。

    methods (TestClassSetup)
        function addControlPath(testCase)
            % 被测函数位于control_EVO根目录，测试启动时显式加入路径。
            testFolder = fileparts(mfilename('fullpath'));
            controlFolder = fileparts(fileparts(testFolder));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(controlFolder));
        end
    end

    methods (Test)
        function testZeroYawDemandOutputsBaseTorque(testCase)
            baseTorque = [10; 20; 30; 40];
            u = testCase.sampleInput(0, baseTorque);

            torque = DYC_simple_load_transfer_distribution(u);

            testCase.verifyEqual(torque, baseTorque, 'AbsTol', 1e-12);
        end

        function testYawDemandAddsDifferentialTorque(testCase)
            % 验证差动扭矩合成后能回到目标附加横摆力矩。
            baseTorque = [30; 30; 30; 30];
            u = testCase.sampleInput(100, baseTorque);

            torque = DYC_simple_load_transfer_distribution(u);
            veh = DYC_vehicle_params;
            yawArm = [-veh.tf/(2*veh.r), veh.tf/(2*veh.r), ...
                -veh.tr/(2*veh.r), veh.tr/(2*veh.r)];
            deltaTorque = torque - baseTorque;

            testCase.verifyNotEqual(torque, baseTorque);
            testCase.verifyEqual(yawArm * deltaTorque, 100, 'AbsTol', 1e-10);
        end

        function testZeroLoadWheelKeepsBaseTorque(testCase)
            baseTorque = [10; 20; 30; 40];
            u = testCase.sampleInput(100, baseTorque);
            u(5) = 0; % FzL1

            torque = DYC_simple_load_transfer_distribution(u);

            testCase.verifyEqual(torque(1), baseTorque(1), 'AbsTol', 1e-12);
            testCase.verifyNotEqual(torque(2:4), baseTorque(2:4));
        end
    end

    methods
        function u = sampleInput(~, mzReq, baseTorque)
            % 构造与QP输入兼容的最小16路输入向量。
            u = zeros(16, 1);
            u(5) = 420; % FzL1
            u(6) = 430; % FzL2
            u(7) = 390; % FzR1
            u(8) = 400; % FzR2
            u(11) = mzReq;
            u(13:16) = baseTorque(:);
        end
    end
end
