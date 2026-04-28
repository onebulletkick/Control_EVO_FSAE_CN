classdef buildPacejkaControlLookupTest < matlab.unittest.TestCase
    %buildPacejkaControlLookupTest 验证 Pacejka 生成控制查表链路。

    properties
        ProjectRoot
        ControlFolder
        ModelingFolder
        Model
    end

    methods (TestClassSetup)
        function addControlPaths(testCase)
            % 同时加入控制代码、轮胎建模代码和Pacejka包入口。
            testFolder = fileparts(mfilename('fullpath'));
            testCase.ModelingFolder = fileparts(testFolder);
            testCase.ControlFolder = fileparts(testCase.ModelingFolder);
            testCase.ProjectRoot = fileparts(testCase.ControlFolder);
            pacejkaMatlabFolder = fullfile(testCase.ControlFolder, ...
                'round9_pacejka_engineering_package', 'matlab');

            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(testCase.ControlFolder));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(testCase.ModelingFolder));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(pacejkaMatlabFolder));
        end

        function buildModelOnce(testCase)
            % 建模过程较重，每个测试类只生成一次Pacejka控制查表。
            testCase.Model = build_pacejka_control_lookup(testCase.ProjectRoot);
        end
    end

    methods (TestMethodTeardown)
        function resetLookupMode(~)
            % 每个用例结束后清理环境变量，避免查表模式串到下一条测试。
            clear QP_TorqueDistribution
            setenv('TIRE_CONTROL_LOOKUP_MODE', '');
            setenv('TIRE_CONTROL_MODEL_FILE', '');
            setenv('PACEJKA_CONTROL_MODEL_FILE', '');
        end
    end

    methods (Test)
        function testGeneratedModelSchema(testCase)
            testCase.verifyEqual(testCase.Model.source, 'round9_pacejka_generated_lookup');
            testCase.verifyEqual(testCase.Model.dataSelection.lateralModelKey, ...
                "Hoosier16_0X7_5_10R20_C2000_rim7in");
            testCase.verifyEqual(testCase.Model.dataSelection.longitudinalModelKey, ...
                "Hoosier18_0X6_0_10R20_rim7in");
            testCase.verifyGreaterThan(numel(testCase.Model.lateral.fzBreakpointsN), 3);
            testCase.verifyGreaterThan(numel(testCase.Model.longitudinalProxy.fzBreakpointsN), 3);
        end

        function testLookupFiniteAndCapped(testCase)
            fz = [200; 800; 1200];
            out = lookup_tire_control_limits(fz, [0; 100; 300], [], [], testCase.Model);
            fixedMuOneLimit = fz .* testCase.Model.config.wheelRadiusM;

            testCase.verifySize(out.Fx_limit, [3 1]);
            testCase.verifyTrue(all(isfinite(out.Fx_limit)));
            testCase.verifyTrue(all(isfinite(out.T_limit)));
            testCase.verifyLessThanOrEqual(out.T_limit, fixedMuOneLimit + 1e-9);
        end

        function testLateralUsageReducesLongitudinalLimit(testCase)
            fz = [600; 900; 1200];
            free = lookup_tire_control_limits(fz, zeros(size(fz)), [], [], testCase.Model);
            used = lookup_tire_control_limits(fz, 0.5 .* free.Fy_limit, [], [], ...
                testCase.Model);

            testCase.verifyLessThan(used.Fx_limit, free.Fx_limit);
        end

        function testPacejkaModeRunsTorqueDistribution(testCase)
            % 验证Pacejka生成表能被QP S-Function按环境变量临时接入。
            setenv('TIRE_CONTROL_LOOKUP_MODE', 'pacejka');
            setenv('PACEJKA_CONTROL_MODEL_FILE', testCase.Model.config.modelPath);
            clear QP_TorqueDistribution

            torque = testCase.runTorqueDistribution(testCase.sampleInput());

            testCase.verifySize(torque, [4 1]);
            testCase.verifyTrue(all(isfinite(torque)));
            testCase.verifyLessThanOrEqual(abs(torque), 400 * ones(4, 1) + 1e-6);
        end

        function testCodeConfigDefaultsToPacejka(testCase)
            setenv('TIRE_CONTROL_LOOKUP_MODE', '');
            setenv('TIRE_CONTROL_MODEL_FILE', '');
            setenv('PACEJKA_CONTROL_MODEL_FILE', '');
            clear QP_TorqueDistribution

            cfg = DYC_tire_lookup_config();
            out = lookup_tire_control_limits(800, 0);

            testCase.verifyEqual(cfg.mode, 'pacejka');
            testCase.verifyEqual(out.modelSource, "round9_pacejka_generated_lookup");
        end
    end

    methods
        function u = sampleInput(~)
            u = zeros(21, 1);
            u(1:4) = [60; 80; 70; 90];
            u(5:8) = [420; 430; 390; 400];
            u(9) = 0.05;
            u(10) = 500;
            u(11) = 80;
            u(12) = 1.0;
            u(13:16) = 20;
            u(17) = 1;
            u(18:21) = 0;
        end

        function torque = runTorqueDistribution(~, u)
            QP_TorqueDistribution([], [], [], 0);
            output = QP_TorqueDistribution(0, zeros(4, 1), u, 3);
            torque = output(1:4);
        end
    end
end
