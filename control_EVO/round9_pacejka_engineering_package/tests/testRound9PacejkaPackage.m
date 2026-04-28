classdef testRound9PacejkaPackage < matlab.unittest.TestCase
    %testRound9PacejkaPackage 验证Round9 Pacejka包产物和预测接口。
    properties
        PackageRoot
        ModelFile
        SummaryFile
    end

    methods (TestMethodSetup)
        function configurePaths(testCase)
            % 兼容工程包目录和开发目录两种布局，自动解析模型产物路径。
            testCase.PackageRoot = fileparts(fileparts(mfilename("fullpath")));
            if isfolder(fullfile(testCase.PackageRoot, "matlab"))
                addpath(fullfile(testCase.PackageRoot, "matlab"));
                testCase.ModelFile = fullfile(testCase.PackageRoot, "model", "pacejka_round9.mat");
                testCase.SummaryFile = fullfile(testCase.PackageRoot, "model", "engineering_usability_summary.csv");
            else
                addpath(testCase.PackageRoot);
                testCase.ModelFile = fullfile(testCase.PackageRoot, "output", "pacejka_round9", "pacejka_round9.mat");
                testCase.SummaryFile = fullfile(testCase.PackageRoot, "output", "pacejka_round9", "engineering_usability_summary.csv");
            end
        end
    end

    methods (Test)
        function packageContainsUsableModel(testCase)
            % 模型文件和工程摘要必须一一对应，并至少包含一个可仿真模型。
            testCase.verifyTrue(isfile(testCase.ModelFile));
            testCase.verifyTrue(isfile(testCase.SummaryFile));

            loaded = load(testCase.ModelFile, "result");
            summary = readtable(testCase.SummaryFile, "TextType", "string");

            testCase.verifyEqual(numel(loaded.result.models), height(summary));
            testCase.verifyGreaterThanOrEqual(sum(summary.simulationReady), 1);
            testCase.verifyGreaterThanOrEqual(sum(summary.trendReady), sum(summary.simulationReady));
        end

        function predictionInterfaceReturnsFiniteForSimulationReadyModel(testCase)
            % 对第一个simulationReady模型做典型输入预测，输出必须为有限值。
            loaded = load(testCase.ModelFile, "result");
            summary = readtable(testCase.SummaryFile, "TextType", "string");
            idx = find(summary.simulationReady, 1);

            input = struct( ...
                "ModelKey", summary.modelKey(idx), ...
                "SA_deg", [-3; 0; 3], ...
                "SR", [-0.03; 0; 0.03], ...
                "FZ_N", [650; 800; 950], ...
                "IA_deg", [0; 1; 2], ...
                "P_kPa", [82; 84; 86], ...
                "V_kph", [40; 40; 40]);

            y = predictTireForces(loaded.result, input);

            testCase.verifyTrue(all(isfinite(y.FX_N)));
            testCase.verifyTrue(all(isfinite(y.FY_N)));
            testCase.verifyTrue(all(isfinite(y.MZ_Nm)));
        end
    end
end
