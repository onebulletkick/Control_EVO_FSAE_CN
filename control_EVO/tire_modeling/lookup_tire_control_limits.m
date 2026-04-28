function out = lookup_tire_control_limits(Fz, FyUsed, alphaRad, kappa, modelInput)
%LOOKUP_TIRE_CONTROL_LIMITS 查询控制用轮胎查表边界。
% Fz 使用正载荷，单位 N。FyUsed 为当前已占用横向力，单位 N。
% modelInput 可以是模型结构体，也可以是包含变量 model 的 .mat 文件。
% 输出字段中的 Fx_limit/T_limit 是控制分配可直接使用的纵向力/轮端扭矩上限。

if nargin < 2 || isempty(FyUsed)
    FyUsed = zeros(size(Fz));
end
if nargin < 3
    alphaRad = [];
end
if nargin < 4
    kappa = [];
end
if nargin < 5 || isempty(modelInput)
    modelInput = localDefaultModelFile();
end

model = localLoadModel(modelInput);
cfg = localConfig(model);
longitudinalFit = localLongitudinalFit(model);

% 标量输入会扩展到与Fz同长度；向量输入必须与Fz一一对应。
Fz = max(double(Fz(:)), 0);
FyUsed = localColumnInput(FyUsed, numel(Fz), 'FyUsed');

% 先按载荷插值得到横向、纵向峰值和小滑移刚度，再应用控制侧保守封顶。
muYRaw = localInterpClamped(model.lateral.fzBreakpointsN, model.lateral.mu, Fz);
muY = min(muYRaw, cfg.controlMuCeiling);
muXProxyRaw = localInterpClamped(longitudinalFit.fzBreakpointsN, ...
    longitudinalFit.mu, Fz);
muXScaledUncapped = cfg.muScaleTc .* muXProxyRaw;
muX = min(muXScaledUncapped, cfg.controlMuCeiling);
CAlpha = localInterpClamped(model.lateral.fzBreakpointsN, ...
    model.lateral.stiffness, Fz);
CKappaProxyRaw = localInterpClamped(longitudinalFit.fzBreakpointsN, ...
    longitudinalFit.stiffness, Fz);
CKappa = cfg.muScaleTc .* CKappaProxyRaw;

% 摩擦椭圆近似：已占用横向力越大，剩余可用纵向力越小。
fyLimit = muY .* Fz;
fxPeak = muX .* Fz;
fyUsage = abs(FyUsed) ./ max(fyLimit, eps);
fxLimit = fxPeak .* sqrt(max(1 - fyUsage.^2, 0));
tLimit = min(fxLimit .* cfg.wheelRadiusM, cfg.motorTorqueLimitNm);

out = struct;
out.Fz = Fz;
out.mu_x = muX;
out.mu_x_proxy_raw = muXProxyRaw;
out.mu_x_scaled_uncapped = muXScaledUncapped;
out.mu_y = muY;
out.mu_y_raw = muYRaw;
out.control_mu_ceiling = cfg.controlMuCeiling;
out.C_alpha = CAlpha;
out.C_kappa = CKappa;
out.C_kappa_proxy_raw = CKappaProxyRaw;
out.Fx_limit = fxLimit;
out.Fy_limit = fyLimit;
out.T_limit = tLimit;
out.modelName = string(model.name);

if isfield(model, 'source')
    out.modelSource = string(model.source);
else
    out.modelSource = "unknown";
end

if ~isempty(alphaRad)
    % 可选估算横向力，便于离线检查查表刚度和峰值限幅是否合理。
    alphaRad = localColumnInput(alphaRad, numel(Fz), 'alphaRad');
    fyLinear = model.lateral.slopeSign .* CAlpha .* alphaRad;
    out.Fy_est = sign(fyLinear) .* min(abs(fyLinear), fyLimit);
end

if ~isempty(kappa)
    % 可选估算纵向力，供TC/DYC调试时查看滑移率对应的线性近似输出。
    kappa = localColumnInput(kappa, numel(Fz), 'kappa');
    fxLinear = longitudinalFit.slopeSign .* CKappa .* kappa;
    out.Fx_est = sign(fxLinear) .* min(abs(fxLinear), fxPeak);
end
end

function model = localLoadModel(modelInput)
% 支持直接传结构体，也支持传.mat文件路径，方便测试和S-Function复用同一入口。
if isstruct(modelInput)
    model = modelInput;
    return;
end

loaded = load(char(modelInput), 'model');
if ~isfield(loaded, 'model')
    error('lookup_tire_control_limits:missingModel', ...
        '模型文件中缺少变量 model: %s', char(modelInput));
end
model = loaded.model;
end

function cfg = localConfig(model)
% 给旧模型补齐默认配置；新模型中的config字段优先级更高。
cfg = struct;
cfg.muScaleTc = 1.0;
cfg.controlMuCeiling = 1.0;
cfg.wheelRadiusM = 0.260;
cfg.motorTorqueLimitNm = 400.0;

if ~isfield(model, 'config')
    return;
end

names = fieldnames(cfg);
for i = 1:numel(names)
    name = names{i};
    if isfield(model.config, name) && isfinite(model.config.(name))
        cfg.(name) = model.config.(name);
    end
end
end

function longitudinalFit = localLongitudinalFit(model)
% Hoosier控制表使用longitudinalProxy，Pacejka生成表兼容longitudinal字段。
if isfield(model, 'longitudinalProxy')
    longitudinalFit = model.longitudinalProxy;
elseif isfield(model, 'longitudinal')
    longitudinalFit = model.longitudinal;
else
    error('lookup_tire_control_limits:missingLongitudinalFit', ...
        '模型缺少 longitudinalProxy 或 longitudinal 查表。');
end
end

function value = localColumnInput(value, n, name)
% 将标量广播为列向量，并对错误维度给出明确异常。
value = double(value);
if isscalar(value)
    value = repmat(value, n, 1);
else
    value = value(:);
end
if numel(value) ~= n
    error('lookup_tire_control_limits:inputSizeMismatch', ...
        '%s 的元素数量必须为 1 或 %d。', name, n);
end
end

function modelFile = localDefaultModelFile()
% 默认模型文件由代码配置决定；只有allowEnvironmentOverride=true时才读取环境变量。
baseFolder = fileparts(mfilename('fullpath'));
cfg = localTireLookupConfig();

if cfg.allowEnvironmentOverride
    overrideFile = getenv('TIRE_CONTROL_MODEL_FILE');
    if ~isempty(overrideFile)
        modelFile = char(overrideFile);
        return;
    end

    envMode = getenv('TIRE_CONTROL_LOOKUP_MODE');
    if ~isempty(envMode)
        cfg.mode = char(envMode);
    end
end

if ~isempty(cfg.modelFile)
    modelFile = char(cfg.modelFile);
    return;
end

mode = lower(strtrim(char(cfg.mode)));
switch mode
    case 'pacejka'
        pacejkaFile = '';
        if cfg.allowEnvironmentOverride
            pacejkaFile = getenv('PACEJKA_CONTROL_MODEL_FILE');
        end
        if ~isempty(pacejkaFile)
            modelFile = char(pacejkaFile);
        else
            modelFile = fullfile(baseFolder, 'outputs', ...
                'pacejka_control_lookup_model.mat');
        end
    otherwise
        hoosierFile = '';
        if cfg.allowEnvironmentOverride
            hoosierFile = getenv('HOOSIER43075_MODEL_FILE');
        end
        if ~isempty(hoosierFile)
            modelFile = char(hoosierFile);
        else
            modelFile = fullfile(baseFolder, 'outputs', ...
                'hoosier43075_control_model.mat');
        end
end
end

function cfg = localTireLookupConfig()
% 与QP_TorqueDistribution保持同一套配置读取规则。
cfg = struct('mode', 'hoosier', 'modelFile', '', ...
    'allowEnvironmentOverride', true);
if exist('DYC_tire_lookup_config', 'file') ~= 2
    return;
end

try
    userCfg = DYC_tire_lookup_config();
catch
    return;
end

if isstruct(userCfg)
    if isfield(userCfg, 'mode')
        cfg.mode = userCfg.mode;
    end
    if isfield(userCfg, 'modelFile')
        cfg.modelFile = userCfg.modelFile;
    end
    if isfield(userCfg, 'allowEnvironmentOverride')
        cfg.allowEnvironmentOverride = logical(userCfg.allowEnvironmentOverride);
    end
end
end

function y = localInterpClamped(x, v, query)
% 查询点超出载荷断点时钳制到边界，避免外推得到过激轮胎能力。
x = x(:);
v = v(:);
queryClamped = min(max(query, min(x)), max(x));
y = interp1(x, v, queryClamped, 'linear');
end
