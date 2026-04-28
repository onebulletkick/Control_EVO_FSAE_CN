function model = build_pacejka_control_lookup(projectRoot, opts)
%BUILD_PACEJKA_CONTROL_LOOKUP 从 Round9 Pacejka 模型生成控制用查表。
% 生成结果与 lookup_tire_control_limits 兼容，用于在控制器中低风险切换。
% 该生成器离线运行，不把Pacejka求值函数直接放进实时QP循环。

if nargin < 1 || isempty(projectRoot)
    projectRoot = localProjectRoot();
end
if nargin < 2 || isempty(opts)
    opts = struct;
end

projectRoot = char(projectRoot);
cfg = localApplyOptions(localDefaultConfig(projectRoot), opts);
localAssertPackageFiles(cfg);

% 读取离线Pacejka拟合包，并选择横向/纵向各自最适合控制查表的来源模型。
addpath(cfg.pacejkaMatlabFolder);
loaded = load(cfg.pacejkaModelFile, 'result');
summary = readtable(cfg.summaryFile, 'TextType', 'string');

lateralSource = localSelectModel(loaded.result, cfg.lateralModelKey);
longitudinalSource = localSelectModel(loaded.result, cfg.longitudinalModelKey);
lateralSummary = localSummaryRow(summary, cfg.lateralModelKey);
longitudinalSummary = localSummaryRow(summary, cfg.longitudinalModelKey);

lateralFit = localBuildAxisFit(loaded.result, lateralSource, 'lateral', cfg);
longitudinalFit = localBuildAxisFit(loaded.result, longitudinalSource, ...
    'longitudinal', cfg);

% 输出结构体字段与Hoosier控制查表保持一致，便于QP无缝切换查表来源。
model = struct;
model.name = 'Round9_Pacejka_generated_control_lookup';
model.source = 'round9_pacejka_generated_lookup';
model.generatedAt = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
model.config = cfg;
model.dataSelection = struct( ...
    'lateralModelKey', string(cfg.lateralModelKey), ...
    'longitudinalModelKey', string(cfg.longitudinalModelKey), ...
    'lateralSummary', lateralSummary, ...
    'longitudinalSummary', longitudinalSummary);
model.lateral = lateralFit;
model.longitudinalProxy = longitudinalFit;
model.notes = localModelNotes();

localEnsureOutputFolder(cfg);
save(cfg.modelPath, 'model');
localWriteReport(model, cfg);

fprintf('Saved Pacejka control lookup: %s\n', cfg.modelPath);
fprintf('Saved Pacejka control report: %s\n', cfg.reportPath);
end

function cfg = localDefaultConfig(projectRoot)
% 默认配置固定项目内路径、轮胎模型选择和控制侧保守系数。
cfg = struct;
cfg.projectRoot = projectRoot;
cfg.packageRoot = fullfile(projectRoot, 'control_EVO', ...
    'round9_pacejka_engineering_package');
cfg.pacejkaMatlabFolder = fullfile(cfg.packageRoot, 'matlab');
cfg.pacejkaModelFile = fullfile(cfg.packageRoot, 'model', 'pacejka_round9.mat');
cfg.summaryFile = fullfile(cfg.packageRoot, 'model', ...
    'engineering_usability_summary.csv');
cfg.outputFolder = fullfile(projectRoot, 'control_EVO', 'tire_modeling', ...
    'outputs');
cfg.modelPath = fullfile(cfg.outputFolder, 'pacejka_control_lookup_model.mat');
cfg.reportPath = fullfile(cfg.outputFolder, 'pacejka_control_lookup_report.md');

cfg.lateralModelKey = 'Hoosier16_0X7_5_10R20_C2000_rim7in';
cfg.longitudinalModelKey = 'Hoosier18_0X6_0_10R20_rim7in';
cfg.fzBreakpointsN = [200 400 600 800 1000 1200];
cfg.alphaGridDeg = linspace(-12, 12, 241);
cfg.kappaGrid = linspace(-0.25, 0.25, 251);
cfg.smallAlphaLimitDeg = 3.0;
cfg.smallAlphaFloorDeg = 0.2;
cfg.smallKappaLimit = 0.08;
cfg.smallKappaFloor = 0.004;
cfg.inclinationDeg = 0.0;
cfg.speedKph = 40.0;
cfg.muScaleTc = 0.70;
cfg.controlMuCeiling = 1.00;
cfg.wheelRadiusM = 0.260;
cfg.motorTorqueLimitNm = 400.0;
end

function cfg = localApplyOptions(cfg, opts)
% 允许测试或离线实验覆盖默认配置，但主流程仍使用清晰的字段名。
fields = fieldnames(opts);
for i = 1:numel(fields)
    cfg.(fields{i}) = opts.(fields{i});
end
cfg.fzBreakpointsN = double(cfg.fzBreakpointsN(:));
cfg.alphaGridDeg = double(cfg.alphaGridDeg(:));
cfg.kappaGrid = double(cfg.kappaGrid(:));
end

function projectRoot = localProjectRoot()
thisFile = mfilename('fullpath');
modelingFolder = fileparts(thisFile);
controlFolder = fileparts(modelingFolder);
projectRoot = fileparts(controlFolder);
end

function localAssertPackageFiles(cfg)
% 生成查表前先确认Pacejka包、模型和工程摘要都存在。
requiredPaths = {cfg.pacejkaMatlabFolder, cfg.pacejkaModelFile, cfg.summaryFile};
for i = 1:numel(requiredPaths)
    pathValue = requiredPaths{i};
    if ~(isfolder(pathValue) || isfile(pathValue))
        error('build_pacejka_control_lookup:missingPackageFile', ...
            '缺少 Pacejka 包文件: %s', pathValue);
    end
end
end

function selected = localSelectModel(result, modelKey)
% 按modelKey从离线拟合结果中取出指定轮胎/轮辋模型。
keys = string({result.models.modelKey});
idx = find(keys == string(modelKey), 1);
if isempty(idx)
    error('build_pacejka_control_lookup:unknownModelKey', ...
        '找不到 Pacejka 模型: %s', char(modelKey));
end
selected = result.models(idx);
end

function row = localSummaryRow(summary, modelKey)
idx = find(summary.modelKey == string(modelKey), 1);
if isempty(idx)
    row = table;
else
    row = summary(idx, :);
end
end

function fit = localBuildAxisFit(result, sourceModel, axisName, cfg)
% 扫描指定载荷断点和滑移网格，把连续Pacejka预测压缩成控制实时查表。
if strcmp(axisName, 'lateral')
    slip = deg2rad(cfg.alphaGridDeg);
    slipName = 'alpha';
    forceName = 'fy';
    smallLimit = deg2rad(cfg.smallAlphaLimitDeg);
    smallFloor = deg2rad(cfg.smallAlphaFloorDeg);
else
    slip = cfg.kappaGrid;
    slipName = 'kappa';
    forceName = 'fx';
    smallLimit = cfg.smallKappaLimit;
    smallFloor = cfg.smallKappaFloor;
end

rows = zeros(numel(cfg.fzBreakpointsN), 7);
for i = 1:numel(cfg.fzBreakpointsN)
    fz = cfg.fzBreakpointsN(i);
    force = localPredictCenteredForce(result, sourceModel, axisName, slip, fz, cfg);
    slope = localSmallSlipSlope(slip, force, smallLimit, smallFloor);
    [muPeak, peakSlip] = localPeakMuAndSlip(force, slip, fz);
    rows(i, :) = [fz, muPeak, abs(slope), abs(peakSlip), 0, numel(slip), ...
        localSlopeSign(slope)];
end

tableData = array2table(rows, 'VariableNames', ...
    {'Fz_N','mu','stiffness_N_per_slip','peakSlip','rmse_N','pointCount','slopeSign'});

fit = struct;
fit.label = char(axisName);
fit.sourceModelKey = string(sourceModel.modelKey);
fit.sourceTireLabel = string(sourceModel.tireLabel);
fit.slipName = slipName;
fit.forceName = forceName;
fit.table = tableData;
fit.fzBreakpointsN = tableData.Fz_N(:);
fit.mu = tableData.mu(:);
fit.stiffness = tableData.stiffness_N_per_slip(:);
fit.peakSlip = tableData.peakSlip(:);
fit.rmseN = tableData.rmse_N(:);
fit.pointCount = tableData.pointCount(:);
fit.slopeSign = localSlopeSign(median(tableData.slopeSign, 'omitnan'));
fit.meanMu = mean(tableData.mu, 'omitnan');
fit.meanRmseN = mean(tableData.rmse_N, 'omitnan');
end

function force = localPredictCenteredForce(result, sourceModel, axisName, slip, fz, cfg)
% 扣除零滑移偏置，避免把离线拟合的静态偏置带入实时限幅表。
input = localPredictionInput(sourceModel, axisName, numel(slip), fz, cfg);
zeroInput = localPredictionInput(sourceModel, axisName, 1, fz, cfg);

if strcmp(axisName, 'lateral')
    input.SA_deg = rad2deg(slip);
    zeroInput.SA_deg = 0;
    y = predictTireForces(result, input);
    y0 = predictTireForces(result, zeroInput);
    force = y.FY_N(:) - y0.FY_N;
else
    input.SR = slip;
    zeroInput.SR = 0;
    y = predictTireForces(result, input);
    y0 = predictTireForces(result, zeroInput);
    force = y.FX_N(:) - y0.FX_N;
end
end

function input = localPredictionInput(sourceModel, axisName, n, fz, cfg)
center = localBestCenter(sourceModel, axisName);
input = struct( ...
    'ModelKey', string(sourceModel.modelKey), ...
    'SA_deg', zeros(n, 1), ...
    'SR', zeros(n, 1), ...
    'FZ_N', repmat(fz, n, 1), ...
    'IA_deg', repmat(cfg.inclinationDeg, n, 1), ...
    'P_kPa', repmat(center.P_kPa, n, 1), ...
    'V_kph', repmat(cfg.speedKph, n, 1), ...
    'SuppressRangeWarning', true);
end

function center = localBestCenter(sourceModel, axisName)
% 使用源模型拟合中心的胎压/速度；缺失时采用Round9常用工况默认值。
if strcmp(axisName, 'longitudinal') && ...
        isfield(sourceModel, 'longitudinal') && ...
        isfield(sourceModel.longitudinal, 'center')
    center = sourceModel.longitudinal.center;
elseif isfield(sourceModel, 'lateral') && isfield(sourceModel.lateral, 'center')
    center = sourceModel.lateral.center;
else
    center = struct('P_kPa', 82.7, 'V_kph', 40.0);
end
if ~isfield(center, 'P_kPa') || ~isfinite(center.P_kPa)
    center.P_kPa = 82.7;
end
if ~isfield(center, 'V_kph') || ~isfinite(center.V_kph)
    center.V_kph = 40.0;
end
end

function slope = localSmallSlipSlope(slip, force, smallLimit, smallFloor)
% 用小滑移区斜率代表控制器线性区刚度，点数不足时适度放宽窗口。
mask = abs(slip) <= smallLimit & abs(slip) >= smallFloor;
if nnz(mask) < 4
    mask = abs(slip) <= smallLimit;
end
slope = localLinearSlope(slip(mask), force(mask));
end

function slope = localLinearSlope(x, y)
x = x(:);
y = y(:);
valid = isfinite(x) & isfinite(y);
x = x(valid);
y = y(valid);
if numel(x) < 2
    slope = NaN;
    return;
end
xCentered = x - mean(x, 'omitnan');
yCentered = y - mean(y, 'omitnan');
denominator = sum(xCentered.^2, 'omitnan');
if denominator <= eps
    slope = NaN;
else
    slope = sum(xCentered .* yCentered, 'omitnan') / denominator;
end
end

function [muPeak, peakSlip] = localPeakMuAndSlip(force, slip, fz)
[peakForce, idx] = max(abs(force));
muPeak = peakForce / max(fz, eps);
peakSlip = slip(idx);
end

function signValue = localSlopeSign(slope)
signValue = sign(slope);
if signValue == 0 || ~isfinite(signValue)
    signValue = 1;
end
end

function localEnsureOutputFolder(cfg)
if ~isfolder(cfg.outputFolder)
    mkdir(cfg.outputFolder);
end
end

function localWriteReport(model, cfg)
% 写出人可读报告，记录来源模型、保守系数和上车前验证提醒。
fid = fopen(cfg.reportPath, 'w');
if fid < 0
    error('build_pacejka_control_lookup:reportOpenFailed', ...
        '无法写入报告: %s', cfg.reportPath);
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, '# Pacejka 生成控制查表报告\n\n');
fprintf(fid, '- 生成时间：%s\n', model.generatedAt);
fprintf(fid, '- 横向来源：%s\n', char(model.dataSelection.lateralModelKey));
fprintf(fid, '- 纵向来源：%s\n', char(model.dataSelection.longitudinalModelKey));
fprintf(fid, '- 输出模型：`%s`\n', cfg.modelPath);
fprintf(fid, '- 控制封顶：controlMuCeiling = %.2f\n', cfg.controlMuCeiling);
fprintf(fid, '- 纵向保守缩放：muScaleTc = %.2f\n\n', cfg.muScaleTc);

fprintf(fid, '## 使用方式\n\n');
fprintf(fid, '保持原 Hoosier 查表：`TIRE_CONTROL_LOOKUP_MODE=hoosier` 或不设置。\n\n');
fprintf(fid, '切换到 Pacejka 生成表：`TIRE_CONTROL_LOOKUP_MODE=pacejka`。\n\n');
fprintf(fid, '也可以用 `TIRE_CONTROL_MODEL_FILE` 指向任意兼容查表 `.mat`。\n\n');

fprintf(fid, '## 横向查表\n\n');
localWriteFitTable(fid, model.lateral, 'mu_y', 'C_alpha');

fprintf(fid, '\n## 纵向查表\n\n');
localWriteFitTable(fid, model.longitudinalProxy, 'mu_x_model_raw', ...
    'C_kappa_model_raw');

fprintf(fid, '\n## 注意\n\n');
fprintf(fid, '- 该表已经做零滑移偏置修正，避免把 Pacejka 零点偏置直接带入控制限幅。\n');
fprintf(fid, '- QP 里仍会叠加路面 Mu 和电机峰值限制，不会单独依赖该表。\n');
fprintf(fid, '- 上车前仍需要 CarSim 和实车数据验证。\n');
end

function localWriteFitTable(fid, fit, muName, stiffnessName)
fprintf(fid, '| Fz_N | %s | %s | peakSlip | points |\n', muName, stiffnessName);
fprintf(fid, '| ---: | ---: | ---: | ---: | ---: |\n');
for i = 1:height(fit.table)
    fprintf(fid, '| %.1f | %.4f | %.2f | %.5f | %d |\n', ...
        fit.table.Fz_N(i), fit.table.mu(i), fit.table.stiffness_N_per_slip(i), ...
        fit.table.peakSlip(i), fit.table.pointCount(i));
end
end

function notes = localModelNotes()
notes = [
    "该模型由 Round9 Pacejka 稳态模型扫描生成，不直接把 Pacejka 函数放入实时 QP。"
    "横向默认使用目标 Hoosier 16x7.5-10 R20 C2000 7 inch 模型。"
    "纵向默认使用完整的 Hoosier 18x6.0-10 R20 7 inch 模型作为保守来源。"
    "所有查表力峰值默认再由 controlMuCeiling 和路面 Mu 双重限制。"
    ];
end
