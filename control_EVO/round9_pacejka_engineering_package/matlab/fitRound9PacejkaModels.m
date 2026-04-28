function result = fitRound9PacejkaModels(opts)
%FITROUND9PACEJKAMODELS 拟合TTC Round9稳态Pacejka风格轮胎模型。
%
% result = fitRound9PacejkaModels()
% result = fitRound9PacejkaModels(opts)
%
% 保存产物为 <OutputDir>/pacejka_round9.mat，内部包含返回的result结构体。
% FZ_N统一存为正载荷；力和回正力矩沿用源数据的Calspan/SAE符号。

if nargin < 1 || isempty(opts)
    opts = struct();
end
opts = localOptions(opts);

if ~isfolder(opts.DataRoot)
    error("Round9Pacejka:MissingDataRoot", "DataRoot does not exist: %s", opts.DataRoot);
end
if ~isfolder(opts.OutputDir)
    mkdir(opts.OutputDir);
end

rng(opts.RandomSeed);

% 先建立run文件和摘要表的对应关系，再按轮胎/轮辋组合分组拟合。
summary = localReadSummaryTables(opts.DataRoot);
cornerFiles = localListRunFiles(fullfile(opts.DataRoot, "RunData_Cornering_Matlab_SI_Round9"));
driveBrakeFiles = localListRunFiles(fullfile(opts.DataRoot, "RunData_DriveBrake_Matlab_SI_Round9"));
dataIntegrity = localInspectDataFiles(cornerFiles, driveBrakeFiles);

groups = localBuildGroups(summary, cornerFiles, driveBrakeFiles);
if isfinite(opts.MaxGroups)
    groups = groups(1:min(numel(groups), opts.MaxGroups));
end

models = repmat(localEmptyModel(), 0, 1);
for k = 1:numel(groups)
    fprintf("Fitting %d/%d: %s\n", k, numel(groups), groups(k).modelKey);
    % 每个模型组分别拟合横向、纵向、回正力矩和组合滑移缩放。
    cornerClean = localLoadRunData(groups(k).cornerFiles, opts);
    driveBrakeClean = localLoadRunData(groups(k).driveBrakeFiles, opts);
    [cornerData, cornerPreprocessing] = localApplySteadyStateFilter(cornerClean, "cornering", opts);
    [driveBrakeData, driveBrakePreprocessing] = localApplySteadyStateFilter(driveBrakeClean, "driveBrake", opts);

    [lateral, lateralQuality] = localFitSubmodel(cornerData, "SA_deg", "FY_N", "lateral", opts);
    [aligning, aligningQuality] = localFitSubmodel(cornerData, "SA_deg", "MZ_Nm", "aligning", opts);

    pureLongitudinalData = localPureLongitudinalSubset(driveBrakeData, opts);
    pureLongitudinalPreprocessing = localPreprocessingStats(pureLongitudinalData, height(pureLongitudinalData), ...
        height(pureLongitudinalData), opts.UseSteadyStateFilter, "pure longitudinal SA window");
    [longitudinal, longitudinalQuality] = localFitSubmodel( ...
        pureLongitudinalData, "SR", "FX_N", "longitudinal", opts);

    combined = localFitCombinedSlip(driveBrakeData, lateral, longitudinal, opts);

    model = localEmptyModel();
    model.modelKey = groups(k).modelKey;
    model.tireLabel = groups(k).tireLabel;
    model.tireCode = groups(k).tireCode;
    model.rimWidth_in = groups(k).rimWidth_in;
    model.corneringRuns = localRunNumbers(groups(k).cornerFiles);
    model.driveBrakeRuns = localRunNumbers(groups(k).driveBrakeFiles);
    model.lateral = lateral;
    model.longitudinal = longitudinal;
    model.aligning = aligning;
    model.combined = combined;
    model.validRange = localValidRange(cornerData, driveBrakeData);
    model.fitDomain = localFitDomain(cornerData, driveBrakeData, pureLongitudinalData);
    model.fitQuality = struct( ...
        "lateral", lateralQuality, ...
        "longitudinal", longitudinalQuality, ...
        "aligning", aligningQuality, ...
        "combined", combined.fitQuality);
    model.preprocessing = struct( ...
        "cornering", cornerPreprocessing, ...
        "driveBrake", driveBrakePreprocessing, ...
        "pureLongitudinal", pureLongitudinalPreprocessing);
    model.sampleCounts = struct( ...
        "cornering", height(cornerData), ...
        "driveBrake", height(driveBrakeData), ...
        "pureLongitudinal", height(pureLongitudinalData));
    model.engineeringUse = localEngineeringUse(model);

    models(end + 1, 1) = model; %#ok<AGROW>

    if opts.MakePlots
        % 图件只服务离线检查，不参与控制器实时运行。
        localWriteFitPlot(model, cornerData, pureLongitudinalData, opts.OutputDir);
        localWriteSensitivityPlot(model, opts.OutputDir);
    end
end

result = struct();
result.createdAt = string(datetime("now", "TimeZone", "local"));
result.options = opts;
result.dataIntegrity = dataIntegrity;
result.summaryTables = summary.sourceFile;
result.models = models;
result.engineeringSummary = localEngineeringSummaryTable(models);

if opts.SaveModelFile
    save(fullfile(opts.OutputDir, "pacejka_round9.mat"), "result");
    writetable(result.engineeringSummary, fullfile(opts.OutputDir, "engineering_usability_summary.csv"));
end
end

function opts = localOptions(userOpts)
% 默认选项集中定义数据路径、稳态过滤阈值、优化边界和报告输出位置。
projectRoot = fileparts(mfilename("fullpath"));
defaults = struct( ...
    "DataRoot", fullfile(projectRoot, "Tire_data"), ...
    "OutputDir", fullfile(projectRoot, "output", "pacejka_round9"), ...
    "MaxGroups", Inf, ...
    "MaxSamplesPerRun", 2500, ...
    "MaxFitSamples", 8000, ...
    "MinSamplesPerFit", 80, ...
    "MinFZ_N", 100, ...
    "MinSpeed_kph", 8, ...
    "MaxAbsSA_deg", 15, ...
    "MaxAbsSR", 1.5, ...
    "MaxAbsForce_N", 8000, ...
    "MaxAbsMoment_Nm", 800, ...
    "UseSteadyStateFilter", true, ...
    "SteadyStateSmoothingPoints", 5, ...
    "MaxAbsSARate_degps", 5.0, ...
    "MaxAbsSRRate_persec", 0.25, ...
    "MaxAbsFZRate_Nps", 3500, ...
    "MaxAbsIARate_degps", 3.0, ...
    "MaxAbsPressureRate_kPaps", 12.0, ...
    "MaxAbsSpeedRate_kphps", 8.0, ...
    "PureLongitudinalSAWindow_deg", 0.75, ...
    "PureLongitudinalFallbackSAWindow_deg", 1.5, ...
    "MinCombinedReferenceForce_N", 75, ...
    "HoldoutStrategy", "stratified", ...
    "HoldoutStride", 5, ...
    "MaxFitIterations", 140, ...
    "MaxFitFunctionEvaluations", 2400, ...
    "MakePlots", true, ...
    "SaveModelFile", true, ...
    "RandomSeed", 11);

opts = defaults;
fields = fieldnames(userOpts);
for k = 1:numel(fields)
    opts.(fields{k}) = userOpts.(fields{k});
end
opts.DataRoot = char(opts.DataRoot);
opts.OutputDir = char(opts.OutputDir);
end

function files = localListRunFiles(dirPath)
% 收集指定目录中的Round9 run数据文件，并按run编号排序。
listing = dir(fullfile(dirPath, "B2356run*.mat"));
if isempty(listing)
    files = strings(0, 1);
    return
end

runNos = zeros(numel(listing), 1);
for k = 1:numel(listing)
    runNos(k) = localRunNumber(listing(k).name);
end
[~, order] = sort(runNos);
listing = listing(order);
files = strings(numel(listing), 1);
for k = 1:numel(listing)
    files(k) = string(fullfile(listing(k).folder, listing(k).name));
end
end

function dataIntegrity = localInspectDataFiles(cornerFiles, driveBrakeFiles)
% 记录数据目录中文件数量和缺失通道，方便报告中判断拟合结果是否可信。
required = ["ET", "V", "SA", "IA", "P", "FX", "FY", "FZ", "MZ", "SR", "TSTI", "TSTC", "TSTO"];
allFiles = [cornerFiles(:); driveBrakeFiles(:)];
missing = strings(0, 1);
rowCounts = zeros(numel(allFiles), 1);

for k = 1:numel(allFiles)
    info = whos("-file", allFiles(k));
    names = string({info.name});
    missingHere = setdiff(required, names);
    if ~isempty(missingHere)
        missing = [missing; allFiles(k) + ":" + missingHere(:)]; %#ok<AGROW>
    end
    etInfo = info(names == "ET");
    if ~isempty(etInfo)
        rowCounts(k) = etInfo.size(1);
    end
end

dataIntegrity = struct();
dataIntegrity.corneringFiles = numel(cornerFiles);
dataIntegrity.driveBrakeFiles = numel(driveBrakeFiles);
dataIntegrity.totalFiles = numel(allFiles);
dataIntegrity.totalRows = sum(rowCounts);
dataIntegrity.missingRequiredChannels = missing;
end

function summary = localReadSummaryTables(dataRoot)
% 读取Round9的测试计划、轮胎信息和轮辋信息，形成后续分组索引。
sourceFile = fullfile(dataRoot, "fromCalspan_Round9", "2356 Summary Tables.xlsx");
if ~isfile(sourceFile)
    error("Round9Pacejka:MissingSummaryTables", "Missing summary workbook: %s", sourceFile);
end

scheduleCells = readcell(sourceFile, "Sheet", "Test Schedule");
runNo = zeros(0, 1);
tireCode = strings(0, 1);
command = strings(0, 1);
tags = strings(0, 1);
for r = 5:size(scheduleCells, 1)
    run = localParseRunNo(localCellString(scheduleCells{r, 1}));
    if isnan(run)
        continue
    end
    runNo(end + 1, 1) = run; %#ok<AGROW>
    tireCode(end + 1, 1) = localCellString(scheduleCells{r, 2}); %#ok<AGROW>
    command(end + 1, 1) = localCellString(scheduleCells{r, 11}); %#ok<AGROW>
    tags(end + 1, 1) = localCellString(scheduleCells{r, 12}); %#ok<AGROW>
end
schedule = table(runNo, tireCode, command, tags);

tireCells = readcell(sourceFile, "Sheet", "Tire ID Schedule");
tireCode = strings(0, 1);
manufacturer = strings(0, 1);
tireSize = strings(0, 1);
description = strings(0, 1);
compoundCode = strings(0, 1);
for r = 5:size(tireCells, 1)
    code = localCellString(tireCells{r, 1});
    if code == ""
        continue
    end
    tireCode(end + 1, 1) = code; %#ok<AGROW>
    manufacturer(end + 1, 1) = localCellString(tireCells{r, 2}); %#ok<AGROW>
    tireSize(end + 1, 1) = localCellString(tireCells{r, 3}); %#ok<AGROW>
    description(end + 1, 1) = localCellString(tireCells{r, 4}); %#ok<AGROW>
    compoundCode(end + 1, 1) = localCellString(tireCells{r, 5}); %#ok<AGROW>
end
tireInfo = table(tireCode, manufacturer, tireSize, description, compoundCode);

tireWheel = localReadTireWheelTable(sourceFile);

summary = struct();
summary.sourceFile = sourceFile;
summary.schedule = schedule;
summary.tireInfo = tireInfo;
summary.tireWheel = tireWheel;
end

function tireWheel = localReadTireWheelTable(sourceFile)
% 从Tire Weights工作表抽取轮胎编号、轮辋编号和轮辋宽度。
cells = readcell(sourceFile, "Sheet", "Tire Weights");
wheelIds = strings(0, 1);
wheelSizes = strings(0, 1);
for r = 7:size(cells, 1)
    wheelSize = localCellString(cells{r, 1});
    wheelId = localCellString(cells{r, 2});
    if wheelSize ~= "" && wheelId ~= ""
        wheelIds(end + 1, 1) = wheelId; %#ok<AGROW>
        wheelSizes(end + 1, 1) = wheelSize; %#ok<AGROW>
    end
end

tireCode = strings(0, 1);
wheelId = strings(0, 1);
wheelSize = strings(0, 1);
rimWidth_in = zeros(0, 1);
for r = 7:size(cells, 1)
    tireNo = localCellDouble(cells{r, 5});
    usedWheel = localCellString(cells{r, 9});
    if isnan(tireNo) || usedWheel == ""
        continue
    end
    idx = find(wheelIds == usedWheel, 1);
    if isempty(idx)
        sizeText = "";
    else
        sizeText = wheelSizes(idx);
    end
    tireCode(end + 1, 1) = "2356tire" + string(round(tireNo)); %#ok<AGROW>
    wheelId(end + 1, 1) = usedWheel; %#ok<AGROW>
    wheelSize(end + 1, 1) = sizeText; %#ok<AGROW>
    rimWidth_in(end + 1, 1) = localParseRimWidth(sizeText); %#ok<AGROW>
end
tireWheel = table(tireCode, wheelId, wheelSize, rimWidth_in);
end

function groups = localBuildGroups(summary, cornerFiles, driveBrakeFiles)
% 将cornering和drive/brake run按同一modelKey聚合，确保同一轮胎配置一起拟合。
groups = repmat(localEmptyGroup(), 0, 1);
groupIndex = containers.Map("KeyType", "char", "ValueType", "double");

    function addFiles(files, fieldName)
        for i = 1:numel(files)
            runNo = localRunNumber(files(i));
            meta = localRunMeta(summary, runNo);
            key = char(meta.modelKey);
            if ~isKey(groupIndex, key)
                groups(end + 1, 1) = localEmptyGroup(); %#ok<AGROW>
                idx = numel(groups);
                groupIndex(key) = idx;
                groups(idx).modelKey = meta.modelKey;
                groups(idx).tireLabel = meta.tireLabel;
                groups(idx).tireCode = meta.tireCode;
                groups(idx).rimWidth_in = meta.rimWidth_in;
            else
                idx = groupIndex(key);
            end
            groups(idx).(fieldName)(end + 1, 1) = files(i);
        end
    end

addFiles(cornerFiles, "cornerFiles");
addFiles(driveBrakeFiles, "driveBrakeFiles");

[~, order] = sort(string({groups.modelKey}));
groups = groups(order);
end

function meta = localRunMeta(summary, runNo)
schedule = summary.schedule;
idx = find(schedule.runNo == runNo, 1);
if isempty(idx)
    tireCode = "unknownRun" + string(runNo);
else
    tireCode = schedule.tireCode(idx);
end

tireInfo = summary.tireInfo;
tireIdx = find(tireInfo.tireCode == tireCode, 1);
if isempty(tireIdx)
    tireLabel = tireCode;
else
    tireLabel = strtrim(strjoin( ...
        [tireInfo.manufacturer(tireIdx), tireInfo.tireSize(tireIdx), tireInfo.description(tireIdx)], " "));
end

tireWheel = summary.tireWheel;
wheelIdx = find(tireWheel.tireCode == tireCode, 1);
if isempty(wheelIdx)
    rimWidth = NaN;
else
    rimWidth = tireWheel.rimWidth_in(wheelIdx);
end
if isnan(rimWidth)
    rimText = "rimUnknown";
else
    rimText = "rim" + string(rimWidth) + "in";
end

modelKey = matlab.lang.makeValidName(tireLabel + "_" + rimText);
meta = struct( ...
    "tireCode", tireCode, ...
    "tireLabel", tireLabel, ...
    "rimWidth_in", rimWidth, ...
    "modelKey", string(modelKey));
end

function data = localLoadRunData(files, opts)
% 读取MATLAB格式run数据，统一字段名、单位和符号，供拟合阶段直接使用。
if isempty(files)
    data = localEmptyDataTable();
    return
end

tables = cell(numel(files), 1);
for k = 1:numel(files)
    filePath = files(k);
    S = load(char(filePath), "ET", "V", "SA", "IA", "P", "FX", "FY", "FZ", "MZ", "SR", "TSTI", "TSTC", "TSTO");
    n = numel(S.ET);
    T = table( ...
        repmat(localRunNumber(filePath), n, 1), ...
        S.ET(:), ...
        S.SA(:), ...
        S.SR(:), ...
        -S.FZ(:), ...
        S.IA(:), ...
        S.P(:), ...
        S.V(:), ...
        S.FX(:), ...
        S.FY(:), ...
        S.MZ(:), ...
        S.TSTI(:), ...
        S.TSTC(:), ...
        S.TSTO(:), ...
        repmat(string(filePath), n, 1), ...
        'VariableNames', {'runNo', 'ET_s', 'SA_deg', 'SR', 'FZ_N', 'IA_deg', ...
        'P_kPa', 'V_kph', 'FX_N', 'FY_N', 'MZ_Nm', 'TSTI_C', 'TSTC_C', 'TSTO_C', 'sourceFile'});

    valid = isfinite(T.SA_deg) & isfinite(T.SR) & isfinite(T.FZ_N) & ...
        isfinite(T.IA_deg) & isfinite(T.P_kPa) & isfinite(T.V_kph) & ...
        isfinite(T.FX_N) & isfinite(T.FY_N) & isfinite(T.MZ_Nm) & ...
        T.FZ_N >= opts.MinFZ_N & T.V_kph >= opts.MinSpeed_kph & ...
        abs(T.SA_deg) <= opts.MaxAbsSA_deg & abs(T.SR) <= opts.MaxAbsSR & ...
        abs(T.FX_N) <= opts.MaxAbsForce_N & abs(T.FY_N) <= opts.MaxAbsForce_N & ...
        abs(T.MZ_Nm) <= opts.MaxAbsMoment_Nm;
    T = T(valid, :);
    tables{k} = localDownsampleTable(T, opts.MaxSamplesPerRun);
end

data = vertcat(tables{:});
end

function [fitData, stats] = localApplySteadyStateFilter(cleanData, mode, opts)
% 根据是否启用稳态过滤决定拟合数据，并返回过滤前后样本数量。
cleanedRows = height(cleanData);
if cleanedRows == 0
    stats = localPreprocessingStats(cleanData, 0, 0, opts.UseSteadyStateFilter, "empty");
    fitData = cleanData;
    return
end

if ~opts.UseSteadyStateFilter
    stats = localPreprocessingStats(cleanData, cleanedRows, cleanedRows, false, "cleaned data");
    fitData = cleanData;
    return
end

steadyMask = localSteadyStateMask(cleanData, mode, opts);
steadyData = cleanData(steadyMask, :);
steadyRows = height(steadyData);

if steadyRows >= opts.MinSamplesPerFit
    fitData = steadyData;
    selection = "steady-state selected";
else
    fitData = cleanData;
    selection = "cleaned fallback; not enough steady-state rows";
end

stats = localPreprocessingStats(fitData, cleanedRows, steadyRows, true, selection);
end

function mask = localSteadyStateMask(data, mode, opts)
% 用载荷、速度、胎压、倾角和通道变化率过滤掉明显非稳态样本。
mask = false(height(data), 1);
runIds = unique(data.runNo, "stable");
for i = 1:numel(runIds)
    idx = find(data.runNo == runIds(i));
    if numel(idx) < max(6, opts.SteadyStateSmoothingPoints)
        mask(idx) = true;
        continue
    end

    runData = data(idx, :);
    saRate = localRate(runData.ET_s, runData.SA_deg, opts.SteadyStateSmoothingPoints);
    srRate = localRate(runData.ET_s, runData.SR, opts.SteadyStateSmoothingPoints);
    fzRate = localRate(runData.ET_s, runData.FZ_N, opts.SteadyStateSmoothingPoints);
    iaRate = localRate(runData.ET_s, runData.IA_deg, opts.SteadyStateSmoothingPoints);
    pressureRate = localRate(runData.ET_s, runData.P_kPa, opts.SteadyStateSmoothingPoints);
    speedRate = localRate(runData.ET_s, runData.V_kph, opts.SteadyStateSmoothingPoints);

    common = fzRate <= opts.MaxAbsFZRate_Nps & ...
        iaRate <= opts.MaxAbsIARate_degps & ...
        pressureRate <= opts.MaxAbsPressureRate_kPaps & ...
        speedRate <= opts.MaxAbsSpeedRate_kphps;

    if mode == "driveBrake"
        runMask = common & saRate <= opts.MaxAbsSARate_degps & srRate <= opts.MaxAbsSRRate_persec;
    else
        runMask = common & saRate <= opts.MaxAbsSARate_degps;
    end
    mask(idx) = runMask;
end
end

function rate = localRate(time, value, smoothingPoints)
time = double(time(:));
value = double(value(:));
dt = diff(time);
validDt = dt(isfinite(dt) & dt > 0);
if isempty(validDt)
    nominalDt = 0.01;
else
    nominalDt = median(validDt);
end
dt(~isfinite(dt) | dt <= 0) = nominalDt;
dt = max(dt, eps);
rate = abs([0; diff(value) ./ dt]);
rate(~isfinite(rate)) = Inf;
if smoothingPoints > 1 && numel(rate) >= smoothingPoints
    rate = movmedian(rate, smoothingPoints, "omitnan");
end
end

function stats = localPreprocessingStats(fitData, cleanedRows, steadyRows, filterEnabled, selection)
stats = struct();
stats.cleanedRows = cleanedRows;
stats.steadyRows = steadyRows;
stats.fittingRows = height(fitData);
stats.steadyFraction = steadyRows / max(cleanedRows, 1);
stats.filterEnabled = logical(filterEnabled);
stats.selection = string(selection);
if isempty(fitData)
    stats.runCount = 0;
else
    stats.runCount = numel(unique(fitData.runNo));
end
end

function data = localPureLongitudinalSubset(data, opts)
if isempty(data)
    return
end
pure = data(abs(data.SA_deg) <= opts.PureLongitudinalSAWindow_deg, :);
if height(pure) < opts.MinSamplesPerFit
    pure = data(abs(data.SA_deg) <= opts.PureLongitudinalFallbackSAWindow_deg, :);
end
data = pure;
end

function [submodel, quality] = localFitSubmodel(data, slipName, targetName, kind, opts)
% 拟合单一力/力矩轴的Magic Formula参数，并同步生成质量评价。
submodel = localDisabledSubmodel(kind, slipName, "insufficient data");
quality = localEmptyQuality("insufficient data");
if height(data) < opts.MinSamplesPerFit
    return
end

[trainData, holdoutData] = localTrainHoldoutSplit(data, opts);
trainData = localDownsampleTable(trainData, opts.MaxFitSamples);
center = localCenter(trainData);
theta0 = localInitialGuess(trainData, slipName, targetName);
[lb, ub] = localBounds(slipName, targetName);
targetScale = max(1, localPercentile(abs(trainData.(targetName)), 90));

objective = @(theta) (localPredictTheta(theta, center, slipName, trainData) - trainData.(targetName)) ./ targetScale;

try
    fitOptions = optimoptions("lsqnonlin", ...
        "Display", "off", ...
        "MaxIterations", opts.MaxFitIterations, ...
        "MaxFunctionEvaluations", opts.MaxFitFunctionEvaluations);
    [theta, resnorm, residual, exitflag] = lsqnonlin(objective, theta0, lb, ub, fitOptions);
catch
    searchOptions = optimset("Display", "off", "MaxIter", opts.MaxFitIterations);
    penaltyObjective = @(theta) sum(objective(localClamp(theta, lb, ub)).^2);
    theta = localClamp(fminsearch(penaltyObjective, theta0, searchOptions), lb, ub);
    residual = objective(theta);
    resnorm = sum(residual.^2);
    exitflag = NaN;
end

submodel = struct( ...
    "kind", string(kind), ...
    "slipVariable", string(slipName), ...
    "targetVariable", string(targetName), ...
    "params", theta(:).', ...
    "center", center, ...
    "fitRows", height(trainData), ...
    "holdoutRows", height(holdoutData), ...
    "holdoutStrategy", string(opts.HoldoutStrategy), ...
    "resnorm", resnorm, ...
    "exitflag", exitflag);
quality = localFitQuality(submodel, holdoutData, slipName, targetName);
quality.trainResidualRMS = sqrt(mean(residual.^2)) * targetScale;
quality.status = "fit";
end

function prediction = localPredictTheta(theta, center, slipName, data)
submodel = struct("params", theta(:).', "center", center);
prediction = round9PacejkaEval(submodel, data.(slipName), localInputFromTable(data));
end

function [trainData, holdoutData] = localTrainHoldoutSplit(data, opts)
% 按run分层抽取holdout样本，避免训练集和验证集完全重合。
runs = unique(data.runNo, "stable");
if string(opts.HoldoutStrategy) == "lastRun" && numel(runs) > 1
    holdoutRun = runs(end);
    holdoutMask = data.runNo == holdoutRun;
else
    holdoutMask = false(height(data), 1);
    for i = 1:numel(runs)
        idx = find(data.runNo == runs(i));
        if numel(idx) >= opts.HoldoutStride * 2
            holdoutMask(idx(opts.HoldoutStride:opts.HoldoutStride:end)) = true;
        end
    end
    if ~any(holdoutMask)
        holdoutMask(opts.HoldoutStride:opts.HoldoutStride:end) = true;
    end
end
holdoutData = data(holdoutMask, :);
trainData = data(~holdoutMask, :);
if isempty(trainData) || isempty(holdoutData)
    trainData = data;
    holdoutData = data;
end
end

function center = localCenter(data)
center = struct( ...
    "FZ_N", max(localMedian(data.FZ_N), eps), ...
    "IA_deg", localMedian(data.IA_deg), ...
    "P_kPa", max(localMedian(data.P_kPa), eps), ...
    "V_kph", max(localMedian(data.V_kph), eps));
end

function theta0 = localInitialGuess(data, slipName, targetName)
slip = data.(slipName);
target = data.(targetName);
fz = data.FZ_N;

trend = sum((slip - localMedian(slip)) .* (target - localMedian(target)));
signD = sign(trend);
if signD == 0
    signD = 1;
end

if slipName == "SR"
    logB = log(20);
else
    logB = log(0.25);
end

if targetName == "MZ_Nm"
    d0Limit = 0.8;
else
    d0Limit = 4.0;
end
d0 = signD * min(d0Limit, max(0.02, localMedian(abs(target) ./ max(fz, eps))));
sv0 = min(1.0, max(-1.0, localMedian(target) ./ max(localMedian(fz), eps)));

theta0 = [logB, 1.35, d0, 0.0, 0.0, sv0, 0.0, 0.0, 0.0, 0.0];
end

function [lb, ub] = localBounds(slipName, targetName)
% 为优化参数设置宽边界，避免无约束拟合走向非物理值。
if slipName == "SR"
    logBLimits = [log(0.5), log(250)];
    shiftLimits = [-0.08, 0.08];
else
    logBLimits = [log(0.01), log(8)];
    shiftLimits = [-5, 5];
end

if targetName == "MZ_Nm"
    dLimits = [-1.5, 1.5];
    svLimits = [-0.6, 0.6];
else
    dLimits = [-5.0, 5.0];
    svLimits = [-1.5, 1.5];
end

lb = [logBLimits(1), 0.25, dLimits(1), -3.0, shiftLimits(1), svLimits(1), -2.0, -2.0, -2.0, -2.0];
ub = [logBLimits(2), 3.0, dLimits(2), 3.0, shiftLimits(2), svLimits(2), 2.0, 2.0, 2.0, 2.0];
end

function quality = localFitQuality(submodel, data, slipName, targetName)
% 用RMSE、NRMSE、相关系数和样本数描述拟合质量，供工程分级使用。
quality = localEmptyQuality("not evaluated");
if isempty(data)
    return
end

pred = round9PacejkaEval(submodel, data.(slipName), localInputFromTable(data));
target = data.(targetName);
valid = isfinite(pred) & isfinite(target);
if ~any(valid)
    return
end

pred = pred(valid);
target = target(valid);
slip = data.(slipName);
slip = slip(valid);
fz = data.FZ_N;
fz = fz(valid);

quality.status = "evaluated";
quality.rmse = sqrt(mean((pred - target).^2));
quality.mae = mean(abs(pred - target));
quality.n = numel(target);

[obsPeak, obsIdx] = max(abs(target));
[predPeak, predIdx] = max(abs(pred));
quality.peakAbsTarget = obsPeak;
quality.peakAbsPrediction = predPeak;
quality.peakAbsError = predPeak - obsPeak;
quality.peakSlipError = abs(slip(predIdx) - slip(obsIdx));

if targetName == "FY_N" || targetName == "FX_N"
    obsMu = max(abs(target) ./ max(fz, eps));
    predMu = max(abs(pred) ./ max(fz, eps));
    quality.peakMuObserved = obsMu;
    quality.peakMuPredicted = predMu;
    quality.peakMuError = predMu - obsMu;
else
    quality.peakMuObserved = NaN;
    quality.peakMuPredicted = NaN;
    quality.peakMuError = NaN;
end
end

function combined = localFitCombinedSlip(driveData, lateral, longitudinal, opts)
% 拟合组合滑移缩放系数，用简单衰减函数描述SA对FX、SR对FY的影响。
combined = struct();
combined.aSA_on_FX = 0;
combined.aSR_on_FY = 0;
combined.fitQuality = struct( ...
    "fxSamples", 0, ...
    "fySamples", 0, ...
    "fxScaleRmse", NaN, ...
    "fyScaleRmse", NaN);

if isempty(driveData)
    return
end

input = localInputFromTable(driveData);
fxPure = round9PacejkaEval(longitudinal, driveData.SR, input);
fyPure = round9PacejkaEval(lateral, driveData.SA_deg, input);

[combined.aSA_on_FX, combined.fitQuality.fxSamples, combined.fitQuality.fxScaleRmse] = ...
    localFitScaleParameter(abs(driveData.SA_deg), driveData.FX_N, fxPure, opts);
[combined.aSR_on_FY, combined.fitQuality.fySamples, combined.fitQuality.fyScaleRmse] = ...
    localFitScaleParameter(abs(driveData.SR), driveData.FY_N, fyPure, opts);
end

function [a, n, rmse] = localFitScaleParameter(slipMagnitude, observed, purePrediction, opts)
a = 0;
rmse = NaN;
valid = isfinite(slipMagnitude) & isfinite(observed) & isfinite(purePrediction) & ...
    abs(purePrediction) >= opts.MinCombinedReferenceForce_N;
slipMagnitude = slipMagnitude(valid);
observed = observed(valid);
purePrediction = purePrediction(valid);
n = numel(observed);
if n < opts.MinSamplesPerFit
    return
end

ratio = abs(observed) ./ max(abs(purePrediction), eps);
ratio = min(1.0, max(0.05, ratio));
objective = @(candidate) mean((ratio - 1 ./ sqrt(1 + (candidate .* slipMagnitude).^2)).^2);
a = fminbnd(objective, 0, 50);
rmse = sqrt(objective(a));
end

function range = localValidRange(cornerData, driveBrakeData)
data = [cornerData; driveBrakeData];
names = ["SA_deg", "SR", "FZ_N", "IA_deg", "P_kPa", "V_kph"];
range = struct();
for k = 1:numel(names)
    name = names(k);
    if isempty(data)
        range.(name) = [NaN NaN];
    else
        range.(name) = [min(data.(name)), max(data.(name))];
    end
end
end

function domain = localFitDomain(cornerData, driveBrakeData, pureLongitudinalData)
domain = struct();
domain.cornering = localDataDomain(cornerData);
domain.driveBrake = localDataDomain(driveBrakeData);
domain.pureLongitudinal = localDataDomain(pureLongitudinalData);
end

function domain = localDataDomain(data)
names = ["SA_deg", "SR", "FZ_N", "IA_deg", "P_kPa", "V_kph", "TSTI_C", "TSTC_C", "TSTO_C"];
domain = struct();
domain.rows = height(data);
if isempty(data)
    domain.runs = zeros(0, 1);
else
    domain.runs = unique(data.runNo);
end
for k = 1:numel(names)
    name = names(k);
    if isempty(data)
        domain.(name) = [NaN NaN];
    else
        domain.(name) = [min(data.(name)), max(data.(name))];
    end
end
end

function engineeringUse = localEngineeringUse(model)
% 将拟合质量翻译成工程可用性结论，避免只凭图像主观判断。
engineeringUse = struct();
engineeringUse.lateral = localAxisEngineeringUse(model.fitQuality.lateral, "force");
engineeringUse.longitudinal = localAxisEngineeringUse(model.fitQuality.longitudinal, "force");
engineeringUse.aligning = localAxisEngineeringUse(model.fitQuality.aligning, "moment");
engineeringUse.combined = localCombinedEngineeringUse(model.combined);
engineeringUse.overallGrade = localOverallEngineeringGrade(engineeringUse, model);
engineeringUse.simulationReady = localGradeAtLeast(engineeringUse.lateral.grade, "B") && ...
    localGradeAtLeast(engineeringUse.longitudinal.grade, "B") && ...
    localGradeAtLeast(engineeringUse.aligning.grade, "C");
engineeringUse.trendReady = localGradeAtLeast(engineeringUse.lateral.grade, "C") && ...
    localGradeAtLeast(engineeringUse.aligning.grade, "C");
engineeringUse.recommendation = localEngineeringRecommendation(engineeringUse, model);
end

function axisUse = localAxisEngineeringUse(quality, axisKind)
axisUse = struct();
axisUse.rmsePeakPct = NaN;
axisUse.grade = "Unavailable";
axisUse.usableForSimulation = false;
axisUse.usableForTrend = false;
axisUse.reason = "not fitted";

if ~isfield(quality, "rmse") || ~isfinite(quality.rmse) || ...
        ~isfield(quality, "peakAbsTarget") || ~isfinite(quality.peakAbsTarget) || quality.peakAbsTarget <= 0
    return
end

axisUse.rmsePeakPct = 100 * quality.rmse / max(quality.peakAbsTarget, eps);
if axisKind == "moment"
    thresholds = [10, 15, 22];
else
    thresholds = [10, 18, 28];
end

if axisUse.rmsePeakPct <= thresholds(1)
    grade = "A";
elseif axisUse.rmsePeakPct <= thresholds(2)
    grade = "B";
elseif axisUse.rmsePeakPct <= thresholds(3)
    grade = "C";
else
    grade = "D";
end

axisUse.grade = grade;
axisUse.usableForSimulation = localGradeAtLeast(grade, "B");
axisUse.usableForTrend = localGradeAtLeast(grade, "C");
axisUse.reason = "holdout RMSE is " + string(round(axisUse.rmsePeakPct, 1)) + "% of observed peak";
if isfield(quality, "peakMuError")
    axisUse.peakMuAbsError = abs(quality.peakMuError);
else
    axisUse.peakMuAbsError = NaN;
end
axisUse.holdoutRows = quality.n;
end

function combinedUse = localCombinedEngineeringUse(combined)
combinedUse = struct("grade", "Unavailable", "usableForTrend", false, "reason", "no combined-slip fit");
if ~isfield(combined, "fitQuality") || ~isfield(combined.fitQuality, "fxSamples")
    return
end
sampleCount = min(combined.fitQuality.fxSamples, combined.fitQuality.fySamples);
if sampleCount >= 2000
    grade = "C";
elseif sampleCount >= 500
    grade = "D";
else
    grade = "Unavailable";
end
combinedUse.grade = grade;
combinedUse.usableForTrend = localGradeAtLeast(grade, "C");
combinedUse.reason = "simplified combined-slip attenuation fit from " + string(sampleCount) + " paired samples";
combinedUse.fxScaleRmse = combined.fitQuality.fxScaleRmse;
combinedUse.fyScaleRmse = combined.fitQuality.fyScaleRmse;
end

function grade = localOverallEngineeringGrade(engineeringUse, model)
grades = [engineeringUse.lateral.grade, engineeringUse.aligning.grade];
if ~isempty(model.driveBrakeRuns)
    grades(end + 1) = engineeringUse.longitudinal.grade;
end
if any(grades == "Unavailable")
    grade = "Unavailable";
    return
end
rank = arrayfun(@localGradeRank, grades);
[~, idx] = max(rank);
grade = grades(idx);
end

function recommendation = localEngineeringRecommendation(engineeringUse, model)
if engineeringUse.simulationReady
    recommendation = "usable as a lap-sim seed; verify against vehicle data before final decisions";
elseif isempty(model.driveBrakeRuns) && engineeringUse.lateral.usableForTrend
    recommendation = "partial model: lateral and MZ only; no longitudinal data for this tire/rim";
elseif engineeringUse.trendReady
    recommendation = "trend-level engineering use; improve segmentation before final lap-sim decisions";
else
    recommendation = "not engineering-ready; inspect source runs and segmentation";
end
end

function tf = localGradeAtLeast(grade, target)
tf = localGradeRank(grade) <= localGradeRank(target);
end

function rank = localGradeRank(grade)
switch string(grade)
    case "A"
        rank = 1;
    case "B"
        rank = 2;
    case "C"
        rank = 3;
    case "D"
        rank = 4;
    otherwise
        rank = 5;
end
end

function summary = localEngineeringSummaryTable(models)
% 把所有模型的工程可用性汇总成表格，便于选择控制查表来源。
n = numel(models);
modelKey = strings(n, 1);
tireLabel = strings(n, 1);
rimWidth_in = NaN(n, 1);
lateralGrade = strings(n, 1);
longitudinalGrade = strings(n, 1);
aligningGrade = strings(n, 1);
combinedGrade = strings(n, 1);
overallGrade = strings(n, 1);
trendReady = false(n, 1);
simulationReady = false(n, 1);
recommendation = strings(n, 1);
lateralRmsePeakPct = NaN(n, 1);
longitudinalRmsePeakPct = NaN(n, 1);
aligningRmsePeakPct = NaN(n, 1);
corneringRows = zeros(n, 1);
driveBrakeRows = zeros(n, 1);
pureLongitudinalRows = zeros(n, 1);
corneringSteadyFraction = NaN(n, 1);
driveBrakeSteadyFraction = NaN(n, 1);

for i = 1:n
    model = models(i);
    modelKey(i) = string(model.modelKey);
    tireLabel(i) = string(model.tireLabel);
    rimWidth_in(i) = model.rimWidth_in;
    lateralGrade(i) = model.engineeringUse.lateral.grade;
    longitudinalGrade(i) = model.engineeringUse.longitudinal.grade;
    aligningGrade(i) = model.engineeringUse.aligning.grade;
    combinedGrade(i) = model.engineeringUse.combined.grade;
    overallGrade(i) = model.engineeringUse.overallGrade;
    trendReady(i) = model.engineeringUse.trendReady;
    simulationReady(i) = model.engineeringUse.simulationReady;
    recommendation(i) = model.engineeringUse.recommendation;
    lateralRmsePeakPct(i) = model.engineeringUse.lateral.rmsePeakPct;
    longitudinalRmsePeakPct(i) = model.engineeringUse.longitudinal.rmsePeakPct;
    aligningRmsePeakPct(i) = model.engineeringUse.aligning.rmsePeakPct;
    corneringRows(i) = model.sampleCounts.cornering;
    driveBrakeRows(i) = model.sampleCounts.driveBrake;
    pureLongitudinalRows(i) = model.sampleCounts.pureLongitudinal;
    corneringSteadyFraction(i) = model.preprocessing.cornering.steadyFraction;
    driveBrakeSteadyFraction(i) = model.preprocessing.driveBrake.steadyFraction;
end

summary = table(modelKey, tireLabel, rimWidth_in, lateralGrade, longitudinalGrade, ...
    aligningGrade, combinedGrade, overallGrade, trendReady, simulationReady, ...
    lateralRmsePeakPct, longitudinalRmsePeakPct, aligningRmsePeakPct, ...
    corneringRows, driveBrakeRows, pureLongitudinalRows, ...
    corneringSteadyFraction, driveBrakeSteadyFraction, recommendation);
end

function localWriteFitPlot(model, cornerData, longData, outputDir)
% 生成拟合散点图和曲线图，用于人工检查拟合趋势。
plotDir = fullfile(outputDir, "plots");
if ~isfolder(plotDir)
    mkdir(plotDir);
end

fig = figure("Visible", "off");
layout = tiledlayout(fig, 1, 3, "TileSpacing", "compact");
title(layout, strrep(model.modelKey, "_", " "));

nexttile;
localScatterFit(cornerData, model.lateral, "SA_deg", "FY_N", "FY vs SA");
nexttile;
localScatterFit(longData, model.longitudinal, "SR", "FX_N", "FX vs SR");
nexttile;
localScatterFit(cornerData, model.aligning, "SA_deg", "MZ_Nm", "MZ vs SA");

fileName = fullfile(plotDir, model.modelKey + ".png");
exportgraphics(fig, fileName, "Resolution", 160);
close(fig);
end

function localWriteSensitivityPlot(model, outputDir)
% 生成载荷、倾角、胎压、速度敏感性图，帮助判断模型外推风险。
plotDir = fullfile(outputDir, "plots");
if ~isfolder(plotDir)
    mkdir(plotDir);
end

fig = figure("Visible", "off", "Position", [100 100 1200 720]);
layout = tiledlayout(fig, 2, 3, "TileSpacing", "compact");
title(layout, "Sensitivities " + strrep(model.modelKey, "_", " "));

nexttile;
localPlotSensitivityFamily(model, "SA_deg", "FZ_N", "FY_N", "FY load sensitivity");
nexttile;
localPlotSensitivityFamily(model, "SR", "FZ_N", "FX_N", "FX load sensitivity");
nexttile;
localPlotSensitivityFamily(model, "SA_deg", "P_kPa", "FY_N", "FY pressure sensitivity");
nexttile;
localPlotSensitivityFamily(model, "SA_deg", "IA_deg", "FY_N", "FY camber sensitivity");
nexttile;
localPlotSensitivityFamily(model, "SA_deg", "IA_deg", "MZ_Nm", "MZ camber sensitivity");
nexttile;
localPlotSensitivityFamily(model, "SR", "P_kPa", "FX_N", "FX pressure sensitivity");

fileName = fullfile(plotDir, model.modelKey + "_sensitivities.png");
exportgraphics(fig, fileName, "Resolution", 160);
close(fig);
end

function localScatterFit(data, submodel, slipName, targetName, plotTitle)
if isempty(data)
    title(plotTitle);
    grid on;
    return
end
data = localDownsampleTable(data, 1200);
pred = round9PacejkaEval(submodel, data.(slipName), localInputFromTable(data));
scatter(data.(slipName), data.(targetName), 5, "filled", "MarkerFaceAlpha", 0.15);
hold on;
[slipSorted, order] = sort(data.(slipName));
plot(slipSorted, pred(order), ".", "MarkerSize", 3);
title(plotTitle);
grid on;
end

function localPlotSensitivityFamily(model, slipName, variedName, targetName, plotTitle)
[slip, input] = localBaselinePredictionInput(model, slipName);
input.SuppressRangeWarning = true;
values = localSensitivityValues(model, variedName);
hold on;
hasCurve = false;
for k = 1:numel(values)
    variedInput = input;
    variedInput.(variedName) = repmat(values(k), numel(slip), 1);
    y = predictTireForces(model, variedInput);
    target = y.(targetName);
    if all(~isfinite(target))
        continue
    end
    plot(slip, target, "DisplayName", variedName + "=" + string(round(values(k), 3)));
    hasCurve = true;
end
title(plotTitle);
xlabel(slipName);
ylabel(targetName);
grid on;
if hasCurve && numel(values) > 1
    legend("Location", "best");
elseif ~hasCurve
    text(0.5, 0.5, "not available", "Units", "normalized", "HorizontalAlignment", "center");
end
end

function [slip, input] = localBaselinePredictionInput(model, slipName)
center = model.lateral.center;
if slipName == "SR" && isfield(model, "longitudinal")
    center = model.longitudinal.center;
end

if isfield(model.validRange, slipName) && all(isfinite(model.validRange.(slipName)))
    slipRange = model.validRange.(slipName);
else
    if slipName == "SR"
        slipRange = [-0.25 0.25];
    else
        slipRange = [-12 12];
    end
end

if slipName == "SR"
    slipRange = [max(-0.30, slipRange(1)), min(0.30, slipRange(2))];
else
    slipRange = [max(-12, slipRange(1)), min(12, slipRange(2))];
end
if slipRange(1) >= slipRange(2)
    slipRange = slipRange + [-1 1];
end

slip = linspace(slipRange(1), slipRange(2), 90).';
n = numel(slip);
input = struct( ...
    "SA_deg", zeros(n, 1), ...
    "SR", zeros(n, 1), ...
    "FZ_N", repmat(center.FZ_N, n, 1), ...
    "IA_deg", repmat(center.IA_deg, n, 1), ...
    "P_kPa", repmat(center.P_kPa, n, 1), ...
    "V_kph", repmat(center.V_kph, n, 1));
input.(slipName) = slip;
end

function values = localSensitivityValues(model, variedName)
center = model.lateral.center;
if isfield(model, "longitudinal") && variedName == "FZ_N" && isfinite(model.longitudinal.center.FZ_N)
    center = model.longitudinal.center;
end

if isfield(model.validRange, variedName) && all(isfinite(model.validRange.(variedName)))
    range = model.validRange.(variedName);
    values = [range(1), center.(variedName), range(2)];
else
    values = center.(variedName);
end
values = unique(values(isfinite(values)), "stable");
if isempty(values)
    values = 0;
end
end

function input = localInputFromTable(data)
input = struct( ...
    "SA_deg", data.SA_deg, ...
    "SR", data.SR, ...
    "FZ_N", data.FZ_N, ...
    "IA_deg", data.IA_deg, ...
    "P_kPa", data.P_kPa, ...
    "V_kph", data.V_kph);
end

function T = localDownsampleTable(T, maxRows)
if height(T) <= maxRows || ~isfinite(maxRows)
    return
end
idx = unique(round(linspace(1, height(T), maxRows)));
T = T(idx(:), :);
end

function model = localEmptyModel()
% 统一初始化模型结构，保证即使某些轴拟合失败也有完整字段。
disabled = localDisabledSubmodel("none", "SA_deg", "not fit");
emptyQuality = localEmptyQuality("not fit");
model = struct( ...
    "modelKey", "", ...
    "tireLabel", "", ...
    "tireCode", "", ...
    "rimWidth_in", NaN, ...
    "corneringRuns", zeros(0, 1), ...
    "driveBrakeRuns", zeros(0, 1), ...
    "lateral", disabled, ...
    "longitudinal", disabled, ...
    "aligning", disabled, ...
    "combined", struct("aSA_on_FX", 0, "aSR_on_FY", 0, "fitQuality", struct()), ...
    "validRange", struct(), ...
    "fitDomain", struct(), ...
    "fitQuality", struct("lateral", emptyQuality, "longitudinal", emptyQuality, "aligning", emptyQuality, "combined", struct()), ...
    "preprocessing", struct("cornering", struct(), "driveBrake", struct(), "pureLongitudinal", struct()), ...
    "sampleCounts", struct("cornering", 0, "driveBrake", 0, "pureLongitudinal", 0), ...
    "engineeringUse", struct());
end

function group = localEmptyGroup()
group = struct( ...
    "modelKey", "", ...
    "tireLabel", "", ...
    "tireCode", "", ...
    "rimWidth_in", NaN, ...
    "cornerFiles", strings(0, 1), ...
    "driveBrakeFiles", strings(0, 1));
end

function submodel = localDisabledSubmodel(kind, slipName, reason)
submodel = struct( ...
    "kind", string(kind), ...
    "slipVariable", string(slipName), ...
    "targetVariable", "", ...
    "params", NaN(1, 10), ...
    "center", struct("FZ_N", 1, "IA_deg", 0, "P_kPa", 1, "V_kph", 1), ...
    "fitRows", 0, ...
    "holdoutRows", 0, ...
    "resnorm", NaN, ...
    "exitflag", NaN, ...
    "reason", string(reason));
end

function quality = localEmptyQuality(status)
quality = struct( ...
    "status", string(status), ...
    "rmse", NaN, ...
    "mae", NaN, ...
    "n", 0, ...
    "trainResidualRMS", NaN, ...
    "peakAbsTarget", NaN, ...
    "peakAbsPrediction", NaN, ...
    "peakAbsError", NaN, ...
    "peakSlipError", NaN, ...
    "peakMuObserved", NaN, ...
    "peakMuPredicted", NaN, ...
    "peakMuError", NaN);
end

function data = localEmptyDataTable()
% 空表字段与正常数据表一致，简化上游空数据分支处理。
data = table( ...
    zeros(0, 1), zeros(0, 1), zeros(0, 1), zeros(0, 1), zeros(0, 1), ...
    zeros(0, 1), zeros(0, 1), zeros(0, 1), zeros(0, 1), zeros(0, 1), ...
    zeros(0, 1), zeros(0, 1), zeros(0, 1), zeros(0, 1), strings(0, 1), ...
    'VariableNames', {'runNo', 'ET_s', 'SA_deg', 'SR', 'FZ_N', 'IA_deg', ...
    'P_kPa', 'V_kph', 'FX_N', 'FY_N', 'MZ_Nm', 'TSTI_C', 'TSTC_C', 'TSTO_C', 'sourceFile'});
end

function runNos = localRunNumbers(files)
runNos = zeros(numel(files), 1);
for k = 1:numel(files)
    runNos(k) = localRunNumber(files(k));
end
runNos = unique(runNos);
end

function runNo = localRunNumber(fileName)
token = regexp(char(fileName), "run(\d+)", "tokens", "once");
if isempty(token)
    runNo = NaN;
else
    runNo = str2double(token{1});
end
end

function runNo = localParseRunNo(value)
token = regexp(char(value), "(\d+)$", "tokens", "once");
if isempty(token)
    runNo = NaN;
else
    runNo = str2double(token{1});
end
end

function rimWidth = localParseRimWidth(wheelSize)
token = regexp(char(wheelSize), "x\s*([0-9.]+)", "tokens", "once");
if isempty(token)
    rimWidth = NaN;
else
    rimWidth = str2double(token{1});
end
end

function text = localCellString(value)
if localIsMissingCell(value)
    text = "";
elseif isnumeric(value)
    if isnan(value)
        text = "";
    else
        text = string(value);
    end
else
    text = strtrim(string(value));
end
end

function value = localCellDouble(cellValue)
if localIsMissingCell(cellValue)
    value = NaN;
elseif isnumeric(cellValue)
    value = double(cellValue);
else
    value = str2double(string(cellValue));
end
end

function tf = localIsMissingCell(value)
if isempty(value)
    tf = true;
    return
end

try
    missingMask = ismissing(value);
    tf = all(missingMask(:));
catch
    tf = false;
end
end

function x = localMedian(x)
x = x(isfinite(x));
if isempty(x)
    x = NaN;
else
    x = median(x);
end
end

function value = localPercentile(x, pct)
x = sort(x(isfinite(x)));
if isempty(x)
    value = NaN;
    return
end
idx = max(1, min(numel(x), round((pct / 100) * numel(x))));
value = x(idx);
end

function theta = localClamp(theta, lb, ub)
theta = min(ub, max(lb, theta));
end
