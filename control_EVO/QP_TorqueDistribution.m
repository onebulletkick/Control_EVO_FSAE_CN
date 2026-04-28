function [sys,x0,str,ts] = QP_TorqueDistribution(t,x,u,flag)
%QP_TORQUEDISTRIBUTION 基于二次规划分配DYC附加横摆力矩到四轮差动扭矩。
% t、x、u分别为S-Function时间、离散状态和输入向量，flag为Simulink调用阶段。
% 输出前4路为差动扭矩[L1;R1;L2;R2]，后6路为QP诊断量。
% 轮序约定：L1/R1为前轴左/右，L2/R2为后轴左/右。

% 缓存上一拍差动扭矩和轮胎查表模型，保证S-Function多次调用时状态连续。
persistent lastDeltaTorque tireModel tireModelReady tireModelLoadFailed
if isempty(lastDeltaTorque) || flag == 0
    lastDeltaTorque = zeros(4,1);
end
if isempty(tireModelReady) || flag == 0
    tireModel = [];
    tireModelReady = false;
    tireModelLoadFailed = false;
end

switch flag
    case 0                                                                  % 初始化尺寸、状态和采样周期。
        [sys,x0,str,ts] = mdlInitializeSizes;
    case 2                                                                  % 将上一拍差动扭矩写入离散状态。
        sys = mdlUpdates(lastDeltaTorque);
    case 3                                                                  % 根据当前输入求解四轮差动扭矩。
        [sys,lastDeltaTorque,tireModel,tireModelReady,tireModelLoadFailed] = ...
            mdlOutputs(t,x,u,lastDeltaTorque,tireModel,tireModelReady, ...
            tireModelLoadFailed);
    case {1,4,9}                                                            % 连续状态、下一采样时刻、终止阶段无额外动作。
        sys = [];
    otherwise                                                               % 未支持的flag视为模型配置错误。
        error(['non-handled flag = ', num2str(flag)]);
end

% flag = 0，定义Level-1 MATLAB S-Function接口尺寸。
function [sys,x0,str,ts] = mdlInitializeSizes
sizes = simsizes;                                                           %读入初始化参数模板
sizes.NumContStates  = 0;                                                   %连续状态数量
sizes.NumDiscStates  = 4;                                                   %离散状态数量
sizes.NumOutputs     = 10;                                                  %输出数量
sizes.NumInputs      = 21;                                                  %输入数量
sizes.DirFeedthrough = 1;                                                   %输入传到输出口
sizes.NumSampleTimes = 1;                                                   %采用一种采样周期
sys = simsizes(sizes);                                                      %初始化

x0 = [0;0;0;0];                                                             %初始状态设置
str = [];                                                                   %s-function必需，设置为空字符串

sampleTime = 0.0005;                                                        %周期步长
ts  = [sampleTime 0];                                                       %采样周期 [周期步长，偏移量]

%**********************************************************************************************************%

% flag = 2，离散状态保存上一拍差动扭矩，供外部调试和下一拍限斜率使用。
function sys = mdlUpdates(lastDeltaTorque)
sys = lastDeltaTorque;

%**********************************************************************************************************%

% flag = 3，求解当前采样点的QP输出。
function [sys,lastDeltaTorque,tireModel,tireModelReady,tireModelLoadFailed] = ...
    mdlOutputs(~,~,u,lastDeltaTorque,tireModel,tireModelReady, ...
    tireModelLoadFailed)
veh = DYC_vehicle_params;                                                    %统一车辆参数
r = veh.r;
tf = veh.tf;
tr = veh.tr;
yawArm = [-tf/2/r, tf/2/r, -tr/2/r, tr/2/r];

% 输入向量前12路沿用原控制模型约定，13:16为基础轮端扭矩，17:21为功率/轮速扩展量。
FyL1 = u(1);                                                                % 各轮横向力，单位N。
FyL2 = u(2);
FyR1 = u(3);
FyR2 = u(4);
FzL1 = u(5);                                                                % 各轮垂向载荷，单位N，按正载荷处理。
FzL2 = u(6);
FzR1 = u(7);
FzR2 = u(8);
Mz = u(11);                                                                 %所需附加横摆力矩
Mu = u(12);                                                                 %摩擦系数
baseTorque = [u(13); u(14); u(15); u(16)];                                  %踏板基础轮端扭矩[L1;R1;L2;R2]
throttle = 1;                                                               %油门开度，兼容旧调用默认满功率
wheelRpm = zeros(4,1);                                                      %四轮轮速rpm[L1;R1;L2;R2]
if numel(u) >= 21
    throttle = u(17);
    wheelRpm = [u(18); u(19); u(20); u(21)];
end

Tdmin = -400;                                                               %电机峰值扭矩
Tdmax = 400;
sampleTime = 0.0005;                                                        %S-Function采样周期，单位s
deltaRateLimit = 1500;                                                      %单轮差动扭矩变化率上限，单位Nm/s
deltaContinuityWeight = 2e-3;                                               %抑制相邻采样差动扭矩跳变
totalDeltaWeight = 1e-2;                                                    %优先保持DYC不改变总驱动扭矩
maxTotalDrivePowerW = 78e3;                                                  %整车正驱动功率上限

minNormalLoad = 1;                                                          %防止启动瞬间Fz为0导致矩阵奇异
minFriction = 0.05;                                                         %防止Mu异常导致除零
fz = max([FzL1; FzR1; FzL2; FzR2], minNormalLoad);
fy = [FyL1; FyR1; FyL2; FyR2];
roadMu = max(Mu, minFriction);

if ~isfinite(Mz)
    % 上游横摆力矩目标无效时保持上一拍差动扭矩，并输出可解释的诊断量。
    candidateTorque = baseTorque(:) + lastDeltaTorque(:);
    achievedYawMoment = yawArm * lastDeltaTorque(:);
    sys = localBuildOutput(lastDeltaTorque, 0, achievedYawMoment, ...
        localDrivePowerMargin(candidateTorque, wheelRpm, throttle, ...
        maxTotalDrivePowerW), 0, 0, 0);
    return;
end

% 所有进入QP的量都做有限值和维度保护，避免单个异常通道使优化器崩溃。
fz(~isfinite(fz)) = minNormalLoad;
fy(~isfinite(fy)) = 0;
baseTorque(~isfinite(baseTorque)) = 0;
baseTorque = baseTorque(:);
if ~isfinite(throttle)
    throttle = 0;
end
throttle = min(max(throttle, 0), 1);
if numel(wheelRpm) ~= 4 || any(~isfinite(wheelRpm))
    wheelRpm = zeros(4,1);
else
    wheelRpm = wheelRpm(:);
end
if ~isfinite(roadMu)
    roadMu = minFriction;
end
if numel(lastDeltaTorque) ~= 4 || any(~isfinite(lastDeltaTorque))
    lastDeltaTorque = zeros(4,1);
else
    lastDeltaTorque = lastDeltaTorque(:);
end

% 优先使用控制查表给出轮胎上限；模型不可用时回退到原Mu摩擦圆。
roadTorqueLimit = r*sqrt(max((roadMu*fz).^2 - fy.^2, 0));
[lookupTorqueLimit,tireModel,tireModelReady,tireModelLoadFailed] = ...
    localTireControlTorqueLimit(fz, fy, roadTorqueLimit, tireModel, ...
    tireModelReady, tireModelLoadFailed, r, Tdmax);
tireTorqueLimit = min(lookupTorqueLimit, roadTorqueLimit);
motorTorqueLimit = DYC_motor_wheel_torque_limit(wheelRpm);
torqueLimit = min([tireTorqueLimit(:), motorTorqueLimit(:), ...
    Tdmax*ones(4,1)], [], 2);
physicalLbTorque = max(-torqueLimit, Tdmin*ones(4,1));
physicalUbTorque = min(torqueLimit, Tdmax*ones(4,1));

% QP只分配DYC差动扭矩，边界由最终扭矩T_base + T_delta的剩余余量决定。
deltaPhysicalLb = physicalLbTorque - baseTorque;
deltaPhysicalUb = physicalUbTorque - baseTorque;
deltaPhysicalLb = min(deltaPhysicalLb, zeros(4,1));                         %基础扭矩已越界时，允许零差动量交给安全层处理
deltaPhysicalUb = max(deltaPhysicalUb, zeros(4,1));

maxDeltaStep = deltaRateLimit * sampleTime;
slewLb = lastDeltaTorque - maxDeltaStep;
slewUb = lastDeltaTorque + maxDeltaStep;
deltaLb = max(deltaPhysicalLb, slewLb);
deltaUb = min(deltaPhysicalUb, slewUb);
slewConflict = deltaLb > deltaUb;
if any(slewConflict)
    deltaLb(slewConflict) = deltaPhysicalLb(slewConflict);
    deltaUb(slewConflict) = deltaPhysicalUb(slewConflict);
end
slewConflictCount = sum(double(slewConflict));

deltaCapacity = max(abs([deltaPhysicalLb, deltaPhysicalUb]), [], 2);
[totalCutLimitNm,totalAddLimitNm] = localTotalDeltaLimits(baseTorque);
tireWeight = 1./max(deltaCapacity.^2, eps);
yawSlackWeight = 1e3;

% 优化目标由三部分组成：轮胎余量利用均衡、横摆力矩松弛惩罚、差动扭矩连续性。
H = diag([tireWeight; yawSlackWeight]);
f = zeros(5,1);
H = localApplyTotalDeltaPreference(H, totalDeltaWeight);
[H, f] = localApplyDeltaContinuityCost(H, f, lastDeltaTorque, ...
    deltaContinuityWeight);

A = [ 1,  1,  1,  1, 0;
     -1, -1, -1, -1, 0];
b = [totalAddLimitNm; totalCutLimitNm];
Aeq = [yawArm, 1];
beq = Mz;
lb = [deltaLb; -Inf];
ub = [deltaUb;  Inf];
x0 = [zeros(4,1); Mz];

options = optimset('Algorithm','active-set','Display','off');                %求解QP
try
    [X,~,exitflag] = quadprog(H,f,A,b,Aeq,beq,lb,ub,x0,options);
catch
    fallbackDelta = zeros(4,1);
    candidateTorque = baseTorque + fallbackDelta;
    achievedYawMoment = yawArm * fallbackDelta;
    sys = localBuildOutput(fallbackDelta, Mz - achievedYawMoment, ...
        achievedYawMoment, localDrivePowerMargin(candidateTorque, wheelRpm, ...
        throttle, maxTotalDrivePowerW), slewConflictCount, Mz, ...
        localMaxDeltaUtilization(fallbackDelta, deltaCapacity));
    lastDeltaTorque = fallbackDelta;
    return;
end

if exitflag <= 0 || numel(X) < 5 || any(~isfinite(X(1:4)))
    % 求解失败时退回零差动扭矩，保持基础扭矩路径可用，并用诊断量暴露未满足的横摆需求。
    fallbackDelta = zeros(4,1);
    candidateTorque = baseTorque + fallbackDelta;
    achievedYawMoment = yawArm * fallbackDelta;
    sys = localBuildOutput(fallbackDelta, Mz - achievedYawMoment, ...
        achievedYawMoment, localDrivePowerMargin(candidateTorque, wheelRpm, ...
        throttle, maxTotalDrivePowerW), slewConflictCount, Mz, ...
        localMaxDeltaUtilization(fallbackDelta, deltaCapacity));
    lastDeltaTorque = fallbackDelta;
    return;
end

deltaTorque = X(1:4);                                                        %输出Mz对应的四轮差动扭矩
achievedYawMoment = yawArm * deltaTorque;
candidateTorque = baseTorque + deltaTorque;
drivePowerMarginW = localDrivePowerMargin(candidateTorque, wheelRpm, ...
    throttle, maxTotalDrivePowerW);
maxDeltaUtilization = localMaxDeltaUtilization(deltaTorque, deltaCapacity);
sys = localBuildOutput(deltaTorque, X(5), achievedYawMoment, ...
    drivePowerMarginW, slewConflictCount, Mz, maxDeltaUtilization);
lastDeltaTorque = deltaTorque;

function [torqueLimit,tireModel,tireModelReady,tireModelLoadFailed] = ...
    localTireControlTorqueLimit(fz, fy, fallbackLimit, tireModel, ...
    tireModelReady, tireModelLoadFailed, wheelRadiusM, motorTorqueLimitNm)
% 从控制用轮胎查表读取四轮轮端扭矩上限；查表不可用时返回摩擦圆备用上限。
torqueLimit = fallbackLimit;
if tireModelLoadFailed
    return;
end
if ~tireModelReady
    [tireModel,tireModelReady,tireModelLoadFailed] = localLoadTireControlModel();
end
if ~tireModelReady
    return;
end

try
    tireModel.config.wheelRadiusM = wheelRadiusM;
    tireModel.config.motorTorqueLimitNm = motorTorqueLimitNm;
    limit = lookup_tire_control_limits(fz, fy, [], [], tireModel);
    candidate = limit.T_limit(:);
    if numel(candidate) == 4 && all(isfinite(candidate)) && all(candidate >= 0)
        torqueLimit = candidate;
    end
catch
    torqueLimit = fallbackLimit;
end

function [totalCutLimitNm,totalAddLimitNm] = localTotalDeltaLimits(baseTorque)
% 允许DYC在极限工况下小幅改变总驱动扭矩，减扭能力大于加扭能力。
baseTorque = baseTorque(:);
if numel(baseTorque) ~= 4 || any(~isfinite(baseTorque))
    baseTorque = zeros(4,1);
end
baseDriveTorque = sum(max(baseTorque, 0));
totalCutLimitNm = min(160, 0.25 * baseDriveTorque + 20);
totalAddLimitNm = min(80, 0.10 * baseDriveTorque);

function H = localApplyTotalDeltaPreference(H, weight)
% 在H矩阵中加入总差动扭矩惩罚，鼓励DYC主要做左右转矩搬移而不是改变总驱动。
if ~isfinite(weight) || weight <= 0
    return;
end

H(1:4,1:4) = H(1:4,1:4) + 2 * weight * ones(4);

function [H, f] = localApplyDeltaContinuityCost(H, f, lastDeltaTorque, weight)
% 加入与上一拍差动扭矩的距离惩罚，降低QP输出跳变。
if numel(lastDeltaTorque) ~= 4 || any(~isfinite(lastDeltaTorque)) || ...
        ~isfinite(weight) || weight <= 0
    return;
end

lastDeltaTorque = lastDeltaTorque(:);
H(1:4,1:4) = H(1:4,1:4) + 2 * weight * eye(4);
f(1:4) = f(1:4) - 2 * weight * lastDeltaTorque;

function sys = localBuildOutput(deltaTorque, yawSlack, achievedYawMoment, ...
    drivePowerMargin, slewConflictCount, targetYawMoment, maxDeltaUtilization)
% 统一打包S-Function输出：4路差动扭矩 + 6路诊断量。
deltaTorque = deltaTorque(:);
if numel(deltaTorque) ~= 4 || any(~isfinite(deltaTorque))
    deltaTorque = zeros(4,1);
end
diagnostics = [yawSlack; achievedYawMoment; drivePowerMargin; ...
    slewConflictCount; targetYawMoment; maxDeltaUtilization];
diagnostics(~isfinite(diagnostics)) = 0;
sys = [deltaTorque; diagnostics];

function drivePowerMarginW = localDrivePowerMargin(torque, wheelRpm, ...
    throttle, maxTotalDrivePowerW)
% 计算当前候选轮端扭矩相对总正驱动功率上限的余量，负值代表超功率请求。
torque = torque(:);
if numel(torque) ~= 4 || any(~isfinite(torque))
    torque = zeros(4,1);
end
wheelRpm = wheelRpm(:);
if numel(wheelRpm) ~= 4 || any(~isfinite(wheelRpm))
    wheelRpm = zeros(4,1);
end
if ~isfinite(throttle)
    throttle = 0;
end
throttle = min(max(throttle, 0), 1);
wheelOmegaRadPerSec = abs(wheelRpm) * (2*pi/60);
availableDrivePowerW = throttle * maxTotalDrivePowerW;
drivePowerMarginW = availableDrivePowerW - ...
    sum(max(torque, 0) .* wheelOmegaRadPerSec);

function maxDeltaUtilization = localMaxDeltaUtilization(deltaTorque, deltaCapacity)
% 诊断差动扭矩对物理可用余量的最大占用比例，用于判断QP是否接近边界。
deltaTorque = deltaTorque(:);
deltaCapacity = deltaCapacity(:);
if numel(deltaTorque) ~= 4 || any(~isfinite(deltaTorque)) || ...
        numel(deltaCapacity) ~= 4 || any(~isfinite(deltaCapacity))
    maxDeltaUtilization = 0;
    return;
end
maxDeltaUtilization = max(abs(deltaTorque) ./ max(deltaCapacity, eps));

function [tireModel,tireModelReady,tireModelLoadFailed] = localLoadTireControlModel()
% 惰性加载轮胎控制查表模型，避免每个采样点重复读.mat文件。
tireModel = [];
tireModelReady = false;
tireModelLoadFailed = false;

modelFile = localTireControlModelFile();
tireFolder = fileparts(fileparts(modelFile));
lookupFile = fullfile(tireFolder, 'lookup_tire_control_limits.m');
if exist(tireFolder, 'dir') == 7 && exist(lookupFile, 'file') == 2
    addpath(tireFolder);
end

if exist(modelFile, 'file') ~= 2
    tireModelLoadFailed = true;
    return;
end

try
    loaded = load(modelFile, 'model');
    if isfield(loaded, 'model')
        tireModel = loaded.model;
        tireModelReady = true;
    else
        tireModelLoadFailed = true;
    end
catch
    tireModelLoadFailed = true;
end

function modelFile = localTireControlModelFile()
% 按代码配置和可选环境变量解析当前轮胎查表模型文件。
baseFolder = fileparts(which('QP_TorqueDistribution'));
if isempty(baseFolder)
    baseFolder = fileparts(mfilename('fullpath'));
end

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
            modelFile = fullfile(baseFolder, 'tire_modeling', 'outputs', ...
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
            modelFile = fullfile(baseFolder, 'tire_modeling', 'outputs', ...
                'hoosier43075_control_model.mat');
        end
end

function cfg = localTireLookupConfig()
% 若项目提供DYC_tire_lookup_config，则以项目配置覆盖兼容默认值。
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
