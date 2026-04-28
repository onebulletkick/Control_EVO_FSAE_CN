% Round9 Pacejka模型快速示例。
% 可从包根目录或examples目录直接运行，用于确认模型加载和预测接口可用。

scriptDir = fileparts(mfilename("fullpath"));
packageRoot = fileparts(scriptDir);

% 兼容工程包布局和单文件开发布局，自动解析模型与摘要文件位置。
if isfolder(fullfile(packageRoot, "matlab"))
    addpath(fullfile(packageRoot, "matlab"));
    modelFile = fullfile(packageRoot, "model", "pacejka_round9.mat");
    summaryFile = fullfile(packageRoot, "model", "engineering_usability_summary.csv");
elseif isfile(fullfile(packageRoot, "fitRound9PacejkaModels.m"))
    addpath(packageRoot);
    modelFile = fullfile(packageRoot, "output", "pacejka_round9", "pacejka_round9.mat");
    summaryFile = fullfile(packageRoot, "output", "pacejka_round9", "engineering_usability_summary.csv");
else
    error("Round9Pacejka:PackageLayout", "Could not find the package MATLAB files.");
end

load(modelFile, "result");
summary = readtable(summaryFile, "TextType", "string");

% 优先选择可仿真的模型；若没有，则退到仅可看趋势的模型。
idx = find(summary.simulationReady, 1);
if isempty(idx)
    idx = find(summary.trendReady, 1);
end
if isempty(idx)
    error("Round9Pacejka:NoUsableModel", "No trend-ready model was found.");
end

modelKey = summary.modelKey(idx);
% 构造典型侧偏角/滑移率扫描点，检查横向、纵向和回正力矩输出。
input = struct( ...
    "ModelKey", modelKey, ...
    "SA_deg", [-4; -2; 0; 2; 4], ...
    "SR", [-0.05; -0.02; 0; 0.02; 0.05], ...
    "FZ_N", 800, ...
    "IA_deg", 1.0, ...
    "P_kPa", 84, ...
    "V_kph", 40);

y = predictTireForces(result, input);

disp("Selected model:");
disp(modelKey);
disp(summary(idx, ["overallGrade", "lateralGrade", "longitudinalGrade", "aligningGrade", "recommendation"]));

predictionTable = table(input.SA_deg, input.SR, y.FX_N, y.FY_N, y.MZ_Nm, ...
    'VariableNames', {'SA_deg', 'SR', 'FX_N', 'FY_N', 'MZ_Nm'});
disp(predictionTable);
