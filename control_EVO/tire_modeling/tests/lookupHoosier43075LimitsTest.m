classdef lookupHoosier43075LimitsTest < matlab.unittest.TestCase
    %lookupHoosier43075LimitsTest 验证 Hoosier 43075 控制查表接口。

    properties
        ProjectRoot
        Model
    end

    methods (TestClassSetup)
        function addModelingFolder(testCase)
            % 只把轮胎建模目录加入路径，保持测试依赖边界清晰。
            testFolder = fileparts(mfilename('fullpath'));
            modelingFolder = fileparts(testFolder);
            controlFolder = fileparts(modelingFolder);
            testCase.ProjectRoot = fileparts(controlFolder);
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(modelingFolder));
        end

        function buildModelOnce(testCase)
            % 从原始TTC数据生成一次Hoosier控制查表，后续用例复用该结构体。
            testCase.Model = build_hoosier43075_model(testCase.ProjectRoot);
        end
    end

    methods (Test)
        function testDataSelection(testCase)
            testCase.verifyEqual(testCase.Model.dataSelection.lateralPrimaryRuns, [2 4 5 6]);
            testCase.verifyEqual(testCase.Model.dataSelection.longitudinalProxyRuns, [71 72 73]);
            testCase.verifyGreaterThan(testCase.Model.dataSelection.lateralPrimaryPoints, 500);
            testCase.verifyGreaterThan(testCase.Model.dataSelection.longitudinalProxyPoints, 500);
        end

        function testLookupFiniteLimits(testCase)
            out = lookup_hoosier43075_limits([0; 100; 750; 1500], [0; 10; 100; 500], ...
                [], [], testCase.Model);

            testCase.verifySize(out.Fx_limit, [4 1]);
            testCase.verifyTrue(all(isfinite(out.Fx_limit)));
            testCase.verifyTrue(all(isfinite(out.T_limit)));
            testCase.verifyGreaterThanOrEqual(out.Fx_limit, zeros(4, 1));
            testCase.verifyGreaterThanOrEqual(out.T_limit, zeros(4, 1));
        end

        function testTorqueLimitUsesWheelRadius(testCase)
            out = lookup_hoosier43075_limits(800, 0, [], [], testCase.Model);

            testCase.verifyEqual(out.T_limit, ...
                min(out.Fx_limit * testCase.Model.config.wheelRadiusM, ...
                testCase.Model.config.motorTorqueLimitNm), AbsTol=1e-10);
        end

        function testTorqueLimitNotMoreAggressiveThanMuOne(testCase)
            % 控制表默认不应比固定Mu=1的旧限幅方案更激进。
            fz = [0; 200; 500; 800; 1100; 1500];
            out = lookup_hoosier43075_limits(fz, zeros(size(fz)), [], [], testCase.Model);
            fixedMuOneLimit = min(fz .* testCase.Model.config.wheelRadiusM, ...
                testCase.Model.config.motorTorqueLimitNm);

            testCase.verifyLessThanOrEqual(out.T_limit, fixedMuOneLimit + 1e-9);
        end

        function testLateralHoldoutValidation(testCase)
            testCase.verifyEqual(testCase.Model.validation.holdoutRun, 6);
            testCase.verifyGreaterThan(testCase.Model.validation.pointCount, 100);
            testCase.verifyTrue(isfinite(testCase.Model.validation.rmseN));
            testCase.verifyLessThan(testCase.Model.validation.normalizedRmse, 1.5);
        end

        function testOptionalForceEstimates(testCase)
            out = lookup_hoosier43075_limits([500; 1000], [0; 50], [0.01; -0.02], ...
                [0.03; -0.04], testCase.Model);

            testCase.verifySize(out.Fy_est, [2 1]);
            testCase.verifySize(out.Fx_est, [2 1]);
            testCase.verifyTrue(all(isfinite(out.Fy_est)));
            testCase.verifyTrue(all(isfinite(out.Fx_est)));
        end
    end
end
