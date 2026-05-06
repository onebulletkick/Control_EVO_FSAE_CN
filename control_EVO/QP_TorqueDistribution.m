function QP_TorqueDistribution(block)
%QP_TORQUEDISTRIBUTION 基于二次规划分配DYC附加横摆力矩到四轮差动扭矩（Level-2 S-Function）。
%   前4路输出为差动扭矩[L1;R1;L2;R2]，后6路为QP诊断量。
%   轮序约定：L1/R1为前轴左/右，L2/R2为后轴左/右。
%   核心算法位于qp_torque_distribution_core.m。

setup(block);

% =========================================================================
function setup(block)
% 初始化Level-2 S-Function接口尺寸和采样周期。

block.NumInputPorts  = 1;
block.NumOutputPorts = 1;

block.InputPort(1).Dimensions       = 21;
block.InputPort(1).DirectFeedthrough = true;

block.OutputPort(1).Dimensions = 10;

% NumContStates and NumDiscStates not needed when using DWork only.

block.SampleTimes = [0.0005 0];

block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
block.RegBlockMethod('InitializeConditions', @InitConditions);
block.RegBlockMethod('Outputs',              @Output);
block.RegBlockMethod('Update',               @Update);

% =========================================================================
function DoPostPropSetup(block)
% 在PostPropagationSetup中配置DWork（R2026a要求）。
%   Dwork(1): lastDeltaTorque (4x1) — 上一拍差动扭矩
block.NumDworks = 1;
block.Dwork(1).Dimensions = 4;
block.Dwork(1).DatatypeID = 0; % double
block.Dwork(1).Complexity = 'Real';
block.Dwork(1).Name = 'lastDeltaTorque';

% =========================================================================
function InitConditions(block)
block.Dwork(1).Data = zeros(4, 1);

% =========================================================================
function Output(block)
% 求解当前采样点的QP输出。

% tireModel为结构体，无法放入DWork，使用persistent变量缓存。
persistent cachedTireModel cachedReady cachedFailed
if isempty(cachedReady)
    cachedTireModel = [];
    cachedReady = false;
    cachedFailed = false;
end

lastDeltaTorque = block.Dwork(1).Data;

[deltaTorque, diagnostics, lastDeltaTorque, ...
    cachedTireModel, cachedReady, cachedFailed] = ...
    qp_torque_distribution_core(block.InputPort(1).Data, ...
    lastDeltaTorque, cachedTireModel, cachedReady, cachedFailed);

block.OutputPort(1).Data = [deltaTorque; diagnostics];
block.Dwork(1).Data = lastDeltaTorque;

% =========================================================================
function Update(~)
% 离散状态更新：DWork(1)已在Output中更新，此处无需额外操作。
