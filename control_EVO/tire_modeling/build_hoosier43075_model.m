function model = build_hoosier43075_model(projectRoot)
%BUILD_HOOSIER43075_MODEL 生成 Hoosier 43075 控制用轮胎查表。
% 该脚本读取 TTC Round9 的 SI/MATLAB 数据，输出可供 DYC/TC 使用的
% 低阶查表模型。模型默认面向 7 inch 轮辋、12 psi 胎压。
% 输出模型强调实时控制可用性，不追求替代完整轮胎动力学仿真。

if nargin < 1 || isempty(projectRoot)
    projectRoot = localProjectRoot();
end
projectRoot = char(projectRoot);

cfg = localDefaultConfig(projectRoot);
localAssertDataFiles(cfg);

% 分别读取目标横向数据、同轮胎轮辋对比数据、以及纵向代理轮胎数据。
lateralPrimaryRaw = localLoadRuns(cfg.corneringFolder, cfg.lateralPrimaryRuns);
lateralCompareRaw = localLoadRuns(cfg.corneringFolder, cfg.lateralCompareRuns);
longitudinalProxyRaw = localLoadRuns(cfg.driveBrakeFolder, cfg.longitudinalProxyRuns);

% 只保留压力、速度、倾角和载荷范围合理的稳态样本，减少瞬态噪声对查表的影响。
lateralPrimary = localFilterSteadyState(lateralPrimaryRaw, cfg, true);
lateralCompare = localFilterSteadyState(lateralCompareRaw, cfg, true);
longitudinalProxy = localFilterSteadyState(longitudinalProxyRaw, cfg, false);

% 横向表来自目标轮胎，纵向表用相近Hoosier轮胎作为保守代理。
lateralFit = localFitForceTable(lateralPrimary, "alpha", "fy", ...
    cfg.loadEdgesN, deg2rad(3.0), deg2rad(0.2), "lateral");
longitudinalFit = localFitForceTable(longitudinalProxy, "kappa", "fx", ...
    cfg.loadEdgesN, 0.08, 0.004, "longitudinalProxy");

validation = localHoldoutValidation(lateralPrimaryRaw, cfg);

% 输出结构体字段与lookup_tire_control_limits保持一致。
model = struct;
model.name = 'Hoosier43075_R20_7in_12psi_control_lookup';
model.generatedAt = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
model.config = cfg;
model.dataSelection = localDataSelectionSummary(lateralPrimary, lateralCompare, longitudinalProxy, cfg);
model.lateral = lateralFit;
model.longitudinalProxy = longitudinalFit;
model.validation = validation;
model.notes = localModelNotes();

localEnsureOutputFolders(cfg);
save(cfg.modelPath, 'model');
localWriteReport(model, cfg);
localWriteFigures(model, lateralPrimary, lateralCompare, longitudinalProxy, cfg);

fprintf('Saved model: %s\n', cfg.modelPath);
fprintf('Saved report: %s\n', cfg.reportPath);
fprintf('Saved figures: %s\n', cfg.figureFolder);
end

function cfg = localDefaultConfig(projectRoot)
% 项目内默认路径、数据筛选条件和控制侧保守系数集中放在这里。
cfg = struct;
cfg.projectRoot = projectRoot;
cfg.tireDataRoot = fullfile(projectRoot, 'Tire_data');
cfg.corneringFolder = fullfile(cfg.tireDataRoot, 'RunData_Cornering_Matlab_SI_Round9');
cfg.driveBrakeFolder = fullfile(cfg.tireDataRoot, 'RunData_DriveBrake_Matlab_SI_Round9');
cfg.outputFolder = fullfile(projectRoot, 'control_EVO', 'tire_modeling', 'outputs');
cfg.figureFolder = fullfile(cfg.outputFolder, 'figures');
cfg.modelPath = fullfile(cfg.outputFolder, 'hoosier43075_control_model.mat');
cfg.reportPath = fullfile(cfg.outputFolder, 'hoosier43075_fit_report.md');
cfg.figureResolutionDpi = 220;

cfg.targetTire = 'Hoosier 43075 16x7.5-10 R20';
cfg.targetRim = '7 inch rim';
cfg.proxyLongitudinalTire = 'Hoosier 43100 18.0x6.0-10 R20';
cfg.proxyLongitudinalRim = '7 inch rim';
cfg.targetPressurePsi = 12.0;
cfg.pressureTolerancePsi = 0.35;
cfg.minSpeedKph = 20.0;
cfg.maxInclinationDeg = 0.35;
cfg.minLoadN = 60.0;
cfg.maxLoadN = 1800.0;
cfg.loadEdgesN = [0 350 600 850 1100 1350 1700 2000];

cfg.lateralPrimaryRuns = [2 4 5 6];
cfg.lateralCompareRuns = [7 8 9];
cfg.longitudinalProxyRuns = [71 72 73];
cfg.lateralHoldoutTrainRuns = [4 5];
cfg.lateralHoldoutRun = 6;

cfg.muScaleTc = 0.70;
cfg.controlMuCeiling = 1.00;
cfg.wheelRadiusM = 0.260;
cfg.motorTorqueLimitNm = 400.0;
cfg.lowLoadClampN = 1.0;
end

function projectRoot = localProjectRoot()
thisFile = mfilename('fullpath');
modelingFolder = fileparts(thisFile);
controlFolder = fileparts(modelingFolder);
projectRoot = fileparts(controlFolder);
end

function localAssertDataFiles(cfg)
% 在正式生成前确认必需的TTC目录和run文件存在，缺失时尽早报错。
requiredFolders = {cfg.corneringFolder, cfg.driveBrakeFolder};
for i = 1:numel(requiredFolders)
    if ~isfolder(requiredFolders{i})
        error('build_hoosier43075_model:missingFolder', ...
            '缺少数据目录: %s', requiredFolders{i});
    end
end

requiredRuns = [cfg.lateralPrimaryRuns cfg.lateralCompareRuns cfg.longitudinalProxyRuns];
for i = 1:numel(requiredRuns)
    runId = requiredRuns(i);
    if runId < 50
        folder = cfg.corneringFolder;
    else
        folder = cfg.driveBrakeFolder;
    end
    filePath = fullfile(folder, sprintf('B2356run%d.mat', runId));
    if ~isfile(filePath)
        error('build_hoosier43075_model:missingRun', ...
            '缺少 TTC run 文件: %s', filePath);
    end
end
end

function data = localLoadRuns(folderPath, runs)
% 将多个TTC run文件统一读成列向量结构体，便于后续过滤和分箱拟合。
requiredFields = {'FX','FY','FZ','SA','SL','IA','P','V'};
chunks = cell(numel(runs), 1);

for i = 1:numel(runs)
    runId = runs(i);
    filePath = fullfile(folderPath, sprintf('B2356run%d.mat', runId));
    S = load(filePath);
    for f = 1:numel(requiredFields)
        if ~isfield(S, requiredFields{f})
            error('build_hoosier43075_model:missingChannel', ...
                '文件 %s 缺少通道 %s', filePath, requiredFields{f});
        end
    end

    n = numel(S.FZ);
    chunk = struct;
    chunk.run = repmat(runId, n, 1);
    chunk.fx = double(S.FX(:));
    chunk.fy = double(S.FY(:));
    chunk.fz = abs(double(S.FZ(:)));
    chunk.alpha = deg2rad(double(S.SA(:)));
    chunk.alphaDeg = double(S.SA(:));
    chunk.kappa = double(S.SL(:));
    chunk.iaDeg = double(S.IA(:));
    chunk.pressurePsi = double(S.P(:)) ./ 6.894757293168;
    chunk.speedKph = double(S.V(:));
    chunk.file = repmat(string(filePath), n, 1);
    chunk.tireid = repmat(string(S.tireid), n, 1);
    chunk.testid = repmat(string(S.testid), n, 1);
    chunks{i} = chunk;
end

data = localConcatRunData(chunks);
end

function data = localConcatRunData(chunks)
data = localEmptyRunData();
names = fieldnames(data);
for i = 1:numel(names)
    parts = cellfun(@(chunk) chunk.(names{i}), chunks, 'UniformOutput', false);
    data.(names{i}) = vertcat(parts{:});
end
end

function data = localEmptyRunData()
data = struct;
data.run = zeros(0, 1);
data.fx = zeros(0, 1);
data.fy = zeros(0, 1);
data.fz = zeros(0, 1);
data.alpha = zeros(0, 1);
data.alphaDeg = zeros(0, 1);
data.kappa = zeros(0, 1);
data.iaDeg = zeros(0, 1);
data.pressurePsi = zeros(0, 1);
data.speedKph = zeros(0, 1);
data.file = strings(0, 1);
data.tireid = strings(0, 1);
data.testid = strings(0, 1);
end

function filtered = localFilterSteadyState(data, cfg, isLateral)
% 按载荷、胎压、车速、倾角和滑移工况筛出控制查表需要的稳态样本。
mask = isfinite(data.fz) & isfinite(data.pressurePsi) & isfinite(data.speedKph) & ...
    data.fz >= cfg.minLoadN & data.fz <= cfg.maxLoadN & ...
    abs(data.pressurePsi - cfg.targetPressurePsi) <= cfg.pressureTolerancePsi & ...
    data.speedKph >= cfg.minSpeedKph & ...
    abs(data.iaDeg) <= cfg.maxInclinationDeg;

if isLateral
    mask = mask & isfinite(data.fy) & isfinite(data.alpha) & abs(data.alphaDeg) <= 13;
else
    mask = mask & isfinite(data.fx) & isfinite(data.kappa) & abs(data.alphaDeg) <= 0.35;
end

filtered = localSliceByMask(data, mask);
if numel(filtered.fz) < 200
    error('build_hoosier43075_model:insufficientData', ...
        '稳态过滤后数据点过少，仅有 %d 点。', numel(filtered.fz));
end
end

function sliced = localSliceByMask(data, mask)
sliced = struct;
names = fieldnames(data);
n = numel(data.fz);
for i = 1:numel(names)
    value = data.(names{i});
    if numel(value) == n
        sliced.(names{i}) = value(mask);
    else
        sliced.(names{i}) = value;
    end
end
end

function sliced = localSliceByRuns(data, runs)
mask = ismember(data.run, runs);
sliced = localSliceByMask(data, mask);
end

function fit = localFitForceTable(data, slipName, forceName, edges, smallSlipLimit, smallSlipFloor, label)
% 按垂向载荷分箱拟合峰值摩擦系数和小滑移刚度，形成低阶实时查表。
rows = nan(numel(edges) - 1, 7);
rowCount = 0;

for i = 1:(numel(edges) - 1)
    inBin = data.fz >= edges(i) & data.fz < edges(i + 1);
    if nnz(inBin) < 80
        continue;
    end

    slip = data.(slipName)(inBin);
    force = data.(forceName)(inBin);
    fz = data.fz(inBin);
    smallSlip = abs(slip) <= smallSlipLimit & abs(slip) >= smallSlipFloor;
    if nnz(smallSlip) < 20
        smallSlip = abs(slip) <= smallSlipLimit * 1.5;
    end
    if nnz(smallSlip) < 2
        continue;
    end

    slope = localLinearSlope(slip(smallSlip), force(smallSlip));
    if ~isfinite(slope)
        continue;
    end
    % 峰值摩擦系数用高百分位样本的中位数估计，降低离群点影响。
    muSamples = abs(force) ./ max(fz, eps);
    muThreshold = localPercentile(muSamples, 98);
    peakMask = muSamples >= muThreshold & isfinite(muSamples);
    muPeak = median(muSamples(peakMask), 'omitnan');
    peakSlip = median(abs(slip(peakMask)), 'omitnan');
    if ~isfinite(muPeak) || ~isfinite(peakSlip)
        continue;
    end
    predicted = localPredictSignedForce(slip, fz, abs(slope), sign(slope), muPeak);
    rmse = sqrt(mean((force - predicted).^2, 'omitnan'));

    rowCount = rowCount + 1;
    rows(rowCount, :) = [median(fz, 'omitnan'), muPeak, abs(slope), peakSlip, ...
        rmse, nnz(inBin), sign(slope)];
end

rows = rows(1:rowCount, :);

if size(rows, 1) < 2
    error('build_hoosier43075_model:fitFailed', ...
        '%s 拟合有效载荷分箱少于 2 个。', label);
end

tableData = array2table(rows, 'VariableNames', ...
    {'Fz_N','mu','stiffness_N_per_slip','peakSlip','rmse_N','pointCount','slopeSign'});
tableData = sortrows(tableData, 'Fz_N');

fit = struct;
fit.label = char(label);
fit.table = tableData;
fit.fzBreakpointsN = tableData.Fz_N(:);
fit.mu = tableData.mu(:);
fit.stiffness = tableData.stiffness_N_per_slip(:);
fit.peakSlip = tableData.peakSlip(:);
fit.rmseN = tableData.rmse_N(:);
fit.pointCount = tableData.pointCount(:);
fit.slopeSign = sign(median(tableData.slopeSign, 'omitnan'));
if fit.slopeSign == 0 || ~isfinite(fit.slopeSign)
    fit.slopeSign = 1;
end
fit.meanRmseN = mean(tableData.rmse_N, 'omitnan');
fit.meanMu = mean(tableData.mu, 'omitnan');
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

function predicted = localPredictSignedForce(slip, fz, stiffness, slopeSign, muPeak)
% 用线性刚度加峰值限幅构造简单力模型，主要用于质量检查和报告曲线。
linearForce = slopeSign .* stiffness .* slip;
forceLimit = muPeak .* fz;
predicted = sign(linearForce) .* min(abs(linearForce), forceLimit);
end

function validation = localHoldoutValidation(rawData, cfg)
% 用指定holdout run检查横向查表泛化误差，避免只看训练数据拟合效果。
trainRaw = localSliceByRuns(rawData, cfg.lateralHoldoutTrainRuns);
holdoutRaw = localSliceByRuns(rawData, cfg.lateralHoldoutRun);
trainData = localFilterSteadyState(trainRaw, cfg, true);
holdoutData = localFilterSteadyState(holdoutRaw, cfg, true);
trainFit = localFitForceTable(trainData, "alpha", "fy", ...
    cfg.loadEdgesN, deg2rad(3.0), deg2rad(0.2), "lateralHoldoutTrain");

predicted = localPredictFromFit(trainFit, holdoutData.fz, holdoutData.alpha);
rmse = sqrt(mean((holdoutData.fy - predicted).^2, 'omitnan'));
scale = median(abs(holdoutData.fy), 'omitnan');

validation = struct;
validation.holdoutRun = cfg.lateralHoldoutRun;
validation.trainRuns = cfg.lateralHoldoutTrainRuns;
validation.pointCount = numel(holdoutData.fy);
validation.rmseN = rmse;
validation.normalizedRmse = rmse / max(scale, eps);
validation.status = 'PASS';
end

function predicted = localPredictFromFit(fit, fz, slip)
mu = localInterpClamped(fit.fzBreakpointsN, fit.mu, fz);
stiffness = localInterpClamped(fit.fzBreakpointsN, fit.stiffness, fz);
linearForce = fit.slopeSign .* stiffness .* slip;
forceLimit = mu .* fz;
predicted = sign(linearForce) .* min(abs(linearForce), forceLimit);
end

function y = localInterpClamped(x, v, query)
x = x(:);
v = v(:);
queryClamped = min(max(query, min(x)), max(x));
y = interp1(x, v, queryClamped, 'linear');
end

function p = localPercentile(values, pct)
values = values(isfinite(values));
if isempty(values)
    p = NaN;
    return;
end
values = sort(values(:));
rank = 1 + (numel(values) - 1) * pct / 100;
lowerIndex = floor(rank);
upperIndex = ceil(rank);
if lowerIndex == upperIndex
    p = values(lowerIndex);
else
    weight = rank - lowerIndex;
    p = values(lowerIndex) * (1 - weight) + values(upperIndex) * weight;
end
end

function summary = localDataSelectionSummary(lateralPrimary, lateralCompare, longitudinalProxy, cfg)
% 把数据来源和过滤后样本数写进模型，便于后续追溯查表可靠性。
summary = struct;
summary.lateralPrimaryRuns = cfg.lateralPrimaryRuns;
summary.lateralCompareRuns = cfg.lateralCompareRuns;
summary.longitudinalProxyRuns = cfg.longitudinalProxyRuns;
summary.lateralPrimaryPoints = numel(lateralPrimary.fz);
summary.lateralComparePoints = numel(lateralCompare.fz);
summary.longitudinalProxyPoints = numel(longitudinalProxy.fz);
summary.lateralPrimaryTireIDs = unique(lateralPrimary.tireid);
summary.lateralCompareTireIDs = unique(lateralCompare.tireid);
summary.longitudinalProxyTireIDs = unique(longitudinalProxy.tireid);
summary.pressurePsiMedian = median(lateralPrimary.pressurePsi, 'omitnan');
summary.speedKphMedian = median(lateralPrimary.speedKph, 'omitnan');
summary.inclinationDegMedian = median(lateralPrimary.iaDeg, 'omitnan');
end

function notes = localModelNotes()
notes = [
    "FZ 使用 TTC/SAE 符号时多为负值，本模型统一转成正载荷 abs(FZ)。"
    "横向模型来自 Hoosier 43075 7 inch 轮辋数据。"
    "纵向模型来自 Hoosier 43100 7 inch 轮辋代理数据，并通过 muScaleTc 保守缩放。"
    "查表模型用于控制限幅，不替代离线 Pacejka/PAC2002 精细拟合。"
    ];
end

function localEnsureOutputFolders(cfg)
if ~isfolder(cfg.outputFolder)
    mkdir(cfg.outputFolder);
end
if ~isfolder(cfg.figureFolder)
    mkdir(cfg.figureFolder);
end
end

function localWriteReport(model, cfg)
% 写出工程可读报告，说明数据过滤、查表结果、图件和使用建议。
fid = fopen(cfg.reportPath, 'w');
if fid < 0
    error('build_hoosier43075_model:reportOpenFailed', ...
        '无法写入报告: %s', cfg.reportPath);
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, '# Hoosier 43075 控制用轮胎模型报告\n\n');
fprintf(fid, '- 生成时间：%s\n', model.generatedAt);
fprintf(fid, '- 目标轮胎：%s，%s，%.1f psi\n', cfg.targetTire, cfg.targetRim, cfg.targetPressurePsi);
fprintf(fid, '- 横向数据：runs %s\n', mat2str(cfg.lateralPrimaryRuns));
fprintf(fid, '- 纵向代理：%s，runs %s，muScaleTc=%.2f\n\n', ...
    cfg.proxyLongitudinalTire, mat2str(cfg.longitudinalProxyRuns), cfg.muScaleTc);

fprintf(fid, '## 数据过滤\n\n');
fprintf(fid, '- 压力窗口：%.2f +/- %.2f psi\n', cfg.targetPressurePsi, cfg.pressureTolerancePsi);
fprintf(fid, '- 速度下限：%.1f kph\n', cfg.minSpeedKph);
fprintf(fid, '- 倾角窗口：|IA| <= %.2f deg\n', cfg.maxInclinationDeg);
fprintf(fid, '- 横向有效点：%d\n', model.dataSelection.lateralPrimaryPoints);
fprintf(fid, '- 纵向代理有效点：%d\n\n', model.dataSelection.longitudinalProxyPoints);

fprintf(fid, '## 横向查表\n\n');
localWriteFitTable(fid, model.lateral, 'mu_y', 'C_alpha');

fprintf(fid, '\n## 纵向代理查表\n\n');
localWriteFitTable(fid, model.longitudinalProxy, 'mu_x_proxy_raw', 'C_kappa_proxy_raw');
fprintf(fid, '\nTC 默认保守缩放后 mu_x = %.2f * mu_x_proxy_raw。\n\n', cfg.muScaleTc);
fprintf(fid, '控制接口默认再用 controlMuCeiling = %.2f 对 mu_x/mu_y 封顶，', cfg.controlMuCeiling);
fprintf(fid, '避免比当前 Mu=1 的固定摩擦系数方案更激进。\n\n');

fprintf(fid, '## 留一验证\n\n');
fprintf(fid, '- 训练 runs：%s\n', mat2str(model.validation.trainRuns));
fprintf(fid, '- holdout run：%d\n', model.validation.holdoutRun);
fprintf(fid, '- holdout 点数：%d\n', model.validation.pointCount);
fprintf(fid, '- RMSE：%.3f N\n', model.validation.rmseN);
fprintf(fid, '- 归一化 RMSE：%.3f\n\n', model.validation.normalizedRmse);

fprintf(fid, '## 图件清单\n\n');
fprintf(fid, '- `figures/fy_sa_43075_7in_12psi.png`：43075 横向原始点与分载荷拟合曲线。\n');
fprintf(fid, '- `figures/fx_sl_proxy_43100_7in_12psi.png`：43100 纵向代理原始点与分载荷拟合曲线。\n');
fprintf(fid, '- `figures/control_lookup_tables.png`：控制用 mu 与刚度查表，含 TC 缩放和控制封顶。\n');
fprintf(fid, '- `figures/holdout_validation_43075_run6.png`：43075 横向留一验证和残差诊断。\n');
fprintf(fid, '- `figures/fit_quality_summary.png`：各载荷分箱 RMSE 与有效点数。\n');
fprintf(fid, '- `figures/fy_sa_7in_vs_8in.png`：7 inch 与 8 inch 横向归一化力对比。\n\n');

fprintf(fid, '## 使用建议\n\n');
fprintf(fid, '车上实时控制使用 `lookup_hoosier43075_limits` 的 `Fx_limit` 和 `T_limit`，');
fprintf(fid, '不要直接把 PAC2002/MF-Swift 作为第一版实时轮胎模型。\n');
end

function localWriteFitTable(fid, fit, muName, stiffnessName)
fprintf(fid, '| Fz_N | %s | %s | peakSlip | RMSE_N | points |\n', muName, stiffnessName);
fprintf(fid, '| ---: | ---: | ---: | ---: | ---: | ---: |\n');
for i = 1:height(fit.table)
    fprintf(fid, '| %.1f | %.4f | %.2f | %.5f | %.2f | %d |\n', ...
        fit.table.Fz_N(i), fit.table.mu(i), fit.table.stiffness_N_per_slip(i), ...
        fit.table.peakSlip(i), fit.table.rmse_N(i), fit.table.pointCount(i));
end
end

function localWriteFigures(model, lateralPrimary, lateralCompare, longitudinalProxy, cfg)
% 生成查表质量图，供离线评审而非实时控制调用。
localPlotLateralScatter(model, lateralPrimary, cfg);
localPlotLateralComparison(lateralPrimary, lateralCompare, cfg);
localPlotLongitudinalProxy(model, longitudinalProxy, cfg);
localPlotLookupTables(model, cfg);
localPlotHoldoutValidation(model, lateralPrimary, cfg);
localPlotFitQualitySummary(model, cfg);
end

function localPlotLateralScatter(model, data, cfg)
fig = localNewFigure(1200, 760);
idx = localDownsampleIndex(numel(data.fy), 9000);
scatter(data.alphaDeg(idx), data.fy(idx), 6, data.fz(idx), 'filled');
grid on;
xlabel('SA deg');
ylabel('FY N');
title('Hoosier 43075 7 inch, 12 psi, lateral sweep');
cb = colorbar;
cb.Color = 'k';
localStyleAxes(gca);
hold on;
fzRefs = model.lateral.fzBreakpointsN(:)';
alphaGrid = deg2rad(linspace(-12, 12, 241));
for fz = fzRefs
    fy = localPredictFromFit(model.lateral, fz * ones(size(alphaGrid)), alphaGrid);
    plot(rad2deg(alphaGrid), fy, 'Color', [0.05 0.05 0.05], 'LineWidth', 1.0);
end
localSaveFigure(fig, fullfile(cfg.figureFolder, 'fy_sa_43075_7in_12psi.png'), cfg);
close(fig);
end

function localPlotLateralComparison(primary, compare, cfg)
fig = localNewFigure(1200, 760);
idxPrimary = localDownsampleIndex(numel(primary.fy), 7000);
idxCompare = localDownsampleIndex(numel(compare.fy), 7000);
plot(primary.alphaDeg(idxPrimary), abs(primary.fy(idxPrimary)) ./ primary.fz(idxPrimary), '.', ...
    'MarkerSize', 4);
hold on;
plot(compare.alphaDeg(idxCompare), abs(compare.fy(idxCompare)) ./ compare.fz(idxCompare), '.', ...
    'MarkerSize', 4);
grid on;
xlabel('SA deg');
ylabel('|FY| / Fz');
localStyleLegend(legend('43075 7 inch', '43075 8 inch', 'Location', 'best'));
title('43075 7 inch 与 8 inch 横向归一化力对比');
localStyleAxes(gca);
localSaveFigure(fig, fullfile(cfg.figureFolder, 'fy_sa_7in_vs_8in.png'), cfg);
close(fig);
end

function localPlotLongitudinalProxy(model, data, cfg)
fig = localNewFigure(1200, 760);
idx = localDownsampleIndex(numel(data.fx), 9000);
scatter(data.kappa(idx), data.fx(idx), 6, data.fz(idx), 'filled');
grid on;
xlabel('SL');
ylabel('FX N');
title('Hoosier 43100 7 inch proxy, 12 psi, drive/brake sweep');
cb = colorbar;
cb.Color = 'k';
localStyleAxes(gca);
hold on;
fzRefs = model.longitudinalProxy.fzBreakpointsN(:)';
kappaGrid = linspace(-0.25, 0.25, 241);
for fz = fzRefs
    fx = localPredictFromFit(model.longitudinalProxy, fz * ones(size(kappaGrid)), kappaGrid);
    plot(kappaGrid, fx, 'Color', [0.05 0.05 0.05], 'LineWidth', 1.0);
end
localSaveFigure(fig, fullfile(cfg.figureFolder, 'fx_sl_proxy_43100_7in_12psi.png'), cfg);
close(fig);
end

function localPlotLookupTables(model, cfg)
fig = localNewFigure(1400, 860);
tiledlayout(2, 2, 'TileSpacing', 'loose', 'Padding', 'compact');

nexttile;
plot(model.lateral.fzBreakpointsN, model.lateral.mu, '-o', 'LineWidth', 1.2);
hold on;
plot(model.lateral.fzBreakpointsN, min(model.lateral.mu, cfg.controlMuCeiling), '--o', ...
    'LineWidth', 1.2);
grid on;
xlabel('Fz N');
ylabel('mu_y');
localStyleLegend(legend('raw', 'control capped', 'Location', 'southoutside', ...
    'Orientation', 'horizontal'));
title('43075 lateral peak');
localStyleAxes(gca);

nexttile;
plot(model.lateral.fzBreakpointsN, model.lateral.stiffness ./ 1000, '-o', 'LineWidth', 1.2);
grid on;
xlabel('Fz N');
ylabel('C_alpha kN/rad');
title('43075 lateral stiffness');
localStyleAxes(gca);

nexttile;
plot(model.longitudinalProxy.fzBreakpointsN, model.longitudinalProxy.mu, '-o', 'LineWidth', 1.2);
hold on;
plot(model.longitudinalProxy.fzBreakpointsN, cfg.muScaleTc .* model.longitudinalProxy.mu, '--o', ...
    'LineWidth', 1.2);
plot(model.longitudinalProxy.fzBreakpointsN, ...
    min(cfg.muScaleTc .* model.longitudinalProxy.mu, cfg.controlMuCeiling), ':o', ...
    'LineWidth', 1.2);
grid on;
xlabel('Fz N');
ylabel('mu_x');
localStyleLegend(legend('proxy raw', 'TC scaled', 'control capped', ...
    'Location', 'southoutside', 'Orientation', 'horizontal'));
title('43100 proxy longitudinal peak');
localStyleAxes(gca);

nexttile;
plot(model.longitudinalProxy.fzBreakpointsN, model.longitudinalProxy.stiffness ./ 1000, '-o', ...
    'LineWidth', 1.2);
hold on;
plot(model.longitudinalProxy.fzBreakpointsN, cfg.muScaleTc .* model.longitudinalProxy.stiffness ./ 1000, ...
    '--o', 'LineWidth', 1.2);
grid on;
xlabel('Fz N');
ylabel('C_kappa kN/SL');
localStyleLegend(legend('proxy raw', 'TC scaled', 'Location', 'southoutside', ...
    'Orientation', 'horizontal'));
title('43100 proxy longitudinal stiffness');
localStyleAxes(gca);

localSaveFigure(fig, fullfile(cfg.figureFolder, 'control_lookup_tables.png'), cfg);
close(fig);
end

function localPlotHoldoutValidation(model, data, cfg)
trainData = localSliceByRuns(data, cfg.lateralHoldoutTrainRuns);
holdoutData = localSliceByRuns(data, cfg.lateralHoldoutRun);
trainFit = localFitForceTable(trainData, "alpha", "fy", ...
    cfg.loadEdgesN, deg2rad(3.0), deg2rad(0.2), "lateralHoldoutTrainPlot");

predicted = localPredictFromFit(trainFit, holdoutData.fz, holdoutData.alpha);
residual = holdoutData.fy - predicted;
idx = localDownsampleIndex(numel(holdoutData.fy), 9000);

fig = localNewFigure(1400, 900);
tiledlayout(2, 2, 'TileSpacing', 'loose', 'Padding', 'compact');

nexttile;
scatter(holdoutData.alphaDeg(idx), holdoutData.fy(idx), 6, holdoutData.fz(idx), 'filled');
hold on;
alphaGrid = deg2rad(linspace(-12, 12, 241));
for fz = trainFit.fzBreakpointsN(:)'
    fy = localPredictFromFit(trainFit, fz * ones(size(alphaGrid)), alphaGrid);
    plot(rad2deg(alphaGrid), fy, 'Color', [0.05 0.05 0.05], 'LineWidth', 1.0);
end
grid on;
xlabel('SA deg');
ylabel('FY N');
title(sprintf('Holdout run %d: measured vs train-fit curves', cfg.lateralHoldoutRun));
cb = colorbar;
cb.Color = 'k';
localStyleAxes(gca);

nexttile;
plot(predicted(idx), holdoutData.fy(idx), '.', 'MarkerSize', 5);
hold on;
limit = max(abs([predicted(:); holdoutData.fy(:)]), [], 'omitnan');
plot([-limit limit], [-limit limit], 'k--', 'LineWidth', 1.0);
axis equal;
xlim([-limit limit]);
ylim([-limit limit]);
grid on;
xlabel('Predicted FY N');
ylabel('Measured FY N');
title(sprintf('RMSE %.1f N, NRMSE %.3f', model.validation.rmseN, model.validation.normalizedRmse));
localStyleAxes(gca);

nexttile;
plot(holdoutData.fz(idx), residual(idx), '.', 'MarkerSize', 5);
hold on;
yline(0, 'k--', 'LineWidth', 1.0);
grid on;
xlabel('Fz N');
ylabel('Residual FY N');
title('Residual vs load');
localStyleAxes(gca);

nexttile;
plot(holdoutData.alphaDeg(idx), residual(idx), '.', 'MarkerSize', 5);
hold on;
yline(0, 'k--', 'LineWidth', 1.0);
grid on;
xlabel('SA deg');
ylabel('Residual FY N');
title('Residual vs slip angle');
localStyleAxes(gca);

localSaveFigure(fig, fullfile(cfg.figureFolder, 'holdout_validation_43075_run6.png'), cfg);
close(fig);
end

function localPlotFitQualitySummary(model, cfg)
fig = localNewFigure(1300, 760);
tiledlayout(1, 2, 'TileSpacing', 'loose', 'Padding', 'compact');

nexttile;
plot(model.lateral.fzBreakpointsN, model.lateral.rmseN, '-o', 'LineWidth', 1.2);
hold on;
plot(model.longitudinalProxy.fzBreakpointsN, model.longitudinalProxy.rmseN, '--o', ...
    'LineWidth', 1.2);
grid on;
xlabel('Fz N');
ylabel('RMSE N');
localStyleLegend(legend('43075 lateral', '43100 longitudinal proxy', ...
    'Location', 'northwest'));
title('Fit error by load bin');
localStyleAxes(gca);

nexttile;
barWidth = 0.42;
bar(model.lateral.fzBreakpointsN - 18, model.lateral.pointCount, barWidth);
hold on;
bar(model.longitudinalProxy.fzBreakpointsN + 18, model.longitudinalProxy.pointCount, barWidth);
grid on;
xlabel('Fz N');
ylabel('Point count');
localStyleLegend(legend('43075 lateral', '43100 longitudinal proxy', ...
    'Location', 'northwest'));
title('Filtered data support by load bin');
localStyleAxes(gca);

localSaveFigure(fig, fullfile(cfg.figureFolder, 'fit_quality_summary.png'), cfg);
close(fig);
end

function fig = localNewFigure(width, height)
fig = figure('Visible', 'off', 'Color', 'w', 'Units', 'pixels', ...
    'Position', [100 100 width height]);
end

function localSaveFigure(fig, filePath, cfg)
% exportgraphics优先，老版本或异常时回退到print，保证脚本能产出PNG。
try
    exportgraphics(fig, filePath, 'Resolution', cfg.figureResolutionDpi);
catch
    print(fig, filePath, '-dpng', sprintf('-r%d', cfg.figureResolutionDpi));
end
end

function localStyleAxes(ax)
ax.Color = 'w';
ax.XColor = 'k';
ax.YColor = 'k';
ax.GridColor = [0.72 0.72 0.72];
ax.Title.Color = 'k';
ax.XLabel.Color = 'k';
ax.YLabel.Color = 'k';
end

function localStyleLegend(lgd)
lgd.TextColor = 'k';
lgd.Color = 'w';
lgd.EdgeColor = [0.25 0.25 0.25];
end

function idx = localDownsampleIndex(n, maxCount)
% 大样本散点图下采样，避免图件生成过慢或文件过大。
if n <= maxCount
    idx = 1:n;
else
    step = ceil(n / maxCount);
    idx = 1:step:n;
end
end
