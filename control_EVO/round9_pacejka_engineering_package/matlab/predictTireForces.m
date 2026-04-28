function output = predictTireForces(model, input)
%PREDICTTIREFORCES 使用Round9稳态轮胎模型预测轮胎力。
%
% output = predictTireForces(model, input)
%
% 必需输入字段：
%   SA_deg, SR, FZ_N, IA_deg, P_kPa, V_kph
%
% FZ_N为正载荷；输出力和力矩沿用源数据的Calspan/SAE符号约定。

model = localSelectModel(model, input);
input = localNormalizeInput(input);
if ~isfield(input, "SuppressRangeWarning") || ~input.SuppressRangeWarning
    localWarnIfOutOfRange(model, input);
end

fxPure = round9PacejkaEval(model.longitudinal, input.SR, input);
fyPure = round9PacejkaEval(model.lateral, input.SA_deg, input);
mz = round9PacejkaEval(model.aligning, input.SA_deg, input);

% 组合滑移项用简单缩放描述：侧偏角削弱纵向力，滑移率削弱横向力。
combined = localCombinedDefaults(model);
gFx = 1 ./ sqrt(1 + (combined.aSA_on_FX .* input.SA_deg).^2);
gFy = 1 ./ sqrt(1 + (combined.aSR_on_FY .* input.SR).^2);

output = struct();
output.FX_N = fxPure .* gFx;
output.FY_N = fyPure .* gFy;
output.MZ_Nm = mz;
output.modelKey = string(model.modelKey);
output.combinedScale = struct("FX", gFx, "FY", gFy);
end

function selected = localSelectModel(model, input)
% 允许传入完整result或单个model；完整result时必须能唯一确定模型。
if isfield(model, "models")
    models = model.models;
    if isfield(input, "ModelIndex")
        idx = input.ModelIndex;
    elseif isfield(input, "ModelKey")
        keys = string({models.modelKey});
        idx = find(keys == string(input.ModelKey), 1);
        if isempty(idx)
            error("Round9Pacejka:UnknownModelKey", ...
                "ModelKey '%s' was not found.", string(input.ModelKey));
        end
    elseif isscalar(models)
        idx = 1;
    else
        error("Round9Pacejka:AmbiguousModel", ...
            "Pass one model entry or provide input.ModelKey/Input.ModelIndex.");
    end
    selected = models(idx);
else
    selected = model;
end
end

function input = localNormalizeInput(input)
% 将标量输入广播成统一长度的列向量，并检查必需字段。
required = ["SA_deg", "SR", "FZ_N", "IA_deg", "P_kPa", "V_kph"];
for k = 1:numel(required)
    if ~isfield(input, required(k))
        error("Round9Pacejka:MissingInput", "Missing input.%s.", required(k));
    end
end

n = 1;
for k = 1:numel(required)
    n = max(n, numel(input.(required(k))));
end

for k = 1:numel(required)
    name = required(k);
    value = double(input.(name));
    if isscalar(value)
        value = repmat(value, n, 1);
    else
        value = value(:);
    end
    if numel(value) ~= n
        error("Round9Pacejka:InputSizeMismatch", ...
            "Input field %s has %d elements; expected 1 or %d.", ...
            name, numel(value), n);
    end
    input.(name) = value;
end
end

function localWarnIfOutOfRange(model, input)
% 预测点超出拟合数据范围时只警告不阻断，便于离线扫描整张曲线。
if ~isfield(model, "validRange")
    return
end

names = ["SA_deg", "SR", "FZ_N", "IA_deg", "P_kPa", "V_kph"];
outside = strings(0, 1);
for k = 1:numel(names)
    name = names(k);
    if ~isfield(model.validRange, name)
        continue
    end
    range = model.validRange.(name);
    value = input.(name);
    if any(value < range(1) | value > range(2))
        outside(end + 1, 1) = name; %#ok<AGROW>
    end
end

if ~isempty(outside)
    warning("Round9Pacejka:OutOfRange", ...
        "Prediction input is outside the fitted data range for: %s.", ...
        strjoin(outside, ", "));
end
end

function combined = localCombinedDefaults(model)
% 老模型可能没有组合滑移字段，缺省时退化为纯滑移预测。
combined = struct("aSA_on_FX", 0, "aSR_on_FY", 0);
if isfield(model, "combined")
    if isfield(model.combined, "aSA_on_FX") && isfinite(model.combined.aSA_on_FX)
        combined.aSA_on_FX = model.combined.aSA_on_FX;
    end
    if isfield(model.combined, "aSR_on_FY") && isfinite(model.combined.aSR_on_FY)
        combined.aSR_on_FY = model.combined.aSR_on_FY;
    end
end
end
