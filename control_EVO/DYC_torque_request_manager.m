function [baseTorqueAlloc, rawBaseTorque, driveEnable] = ...
    DYC_torque_request_manager(throttle, brakeStatus, resetState)
%DYC_TORQUE_REQUEST_MANAGER 生成分配层使用的基础扭矩请求。
%   制动使能只作用在进入控制分配的基础扭矩上，原始踏板需求单独保留用于诊断。
%#codegen

if nargin < 3
    resetState = false;
end

% driveEnableState用于在刹车/松刹车边沿平滑切换驱动扭矩请求。
persistent driveEnableState
if isempty(driveEnableState) || resetState
    driveEnableState = 0;
end

if ~isfinite(throttle)
    throttle = 0;
end
if ~isfinite(brakeStatus)
    brakeStatus = 0;
end

sampleTime = 0.0005;
enableRisingRate = 5;
enableFallingRate = 20;

% brakeStatus=1表示制动介入，此时分配层基础驱动扭矩目标降为0。
brakeCmd = min(max(double(brakeStatus), 0), 1);
targetEnable = 1 - brakeCmd;
enableError = targetEnable - driveEnableState;
maxRiseStep = enableRisingRate * sampleTime;
maxFallStep = enableFallingRate * sampleTime;

% 上升和下降使用不同速率：松刹车恢复较缓，踩刹车切断更快。
if enableError > maxRiseStep
    driveEnableState = driveEnableState + maxRiseStep;
elseif enableError < -maxFallStep
    driveEnableState = driveEnableState - maxFallStep;
else
    driveEnableState = targetEnable;
end
driveEnableState = min(max(driveEnableState, 0), 1);

rawBaseTorque = DYC_base_motor_torque(throttle);
baseTorqueAlloc = rawBaseTorque .* driveEnableState;
driveEnable = driveEnableState;
end
