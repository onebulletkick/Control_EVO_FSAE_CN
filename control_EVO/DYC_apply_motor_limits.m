function limitedWheelTorque = DYC_apply_motor_limits(commandWheelTorque, throttle, wheelRpm)
%DYC_APPLY_MOTOR_LIMITS 对最终四轮轮端扭矩执行电机限扭和总功率限扭。
%   commandWheelTorque  四轮期望轮端扭矩，顺序为[L1; R1; L2; R2]。
%   throttle            油门开度，用于总正驱动功率上限。
%   wheelRpm            四轮轮速，单位rpm，顺序为[L1; R1; L2; R2]。
%#codegen

% 整车电池/电机侧允许的总正驱动功率，用于防止四轮同时请求过大正扭矩。
maxTotalDrivePowerW = 78e3;

% 统一把输入整理为4x1列向量；异常输入按0处理，避免模型仿真中断。
cmd = commandWheelTorque(:);
if numel(cmd) ~= 4
    cmd = zeros(4, 1);
end
cmd(~isfinite(cmd)) = 0;

wheelRpm = wheelRpm(:);
if numel(wheelRpm) ~= 4
    wheelRpm = zeros(4, 1);
end
wheelRpm(~isfinite(wheelRpm)) = 0;
absWheelRpm = abs(wheelRpm);

% 先按单个电机外特性限扭，再按总正驱动功率统一缩放正扭矩。
throttleCmd = min(max(throttle, 0), 1);
wheelTorqueLimit = DYC_motor_wheel_torque_limit(absWheelRpm);
limitedWheelTorque = min(max(cmd, -wheelTorqueLimit), wheelTorqueLimit);

positiveTorque = max(limitedWheelTorque, 0);
wheelOmegaRadPerSec = absWheelRpm * (2*pi/60);
requestedDrivePowerW = sum(positiveTorque .* wheelOmegaRadPerSec);
availableDrivePowerW = throttleCmd * maxTotalDrivePowerW;

% 只缩放正驱动扭矩；负扭矩/制动力矩保持对称限扭后的结果。
if requestedDrivePowerW > availableDrivePowerW
    if availableDrivePowerW <= 0
        powerScale = 0;
    else
        powerScale = availableDrivePowerW / requestedDrivePowerW;
    end
    for idx = 1:4
        if limitedWheelTorque(idx) > 0
            limitedWheelTorque(idx) = limitedWheelTorque(idx) * powerScale;
        end
    end
end
end
