function wheelTorqueLimit = DYC_motor_wheel_torque_limit(wheelRpm)
%DYC_MOTOR_WHEEL_TORQUE_LIMIT 根据轮速计算四轮轮端电机扭矩上限。
%   wheelRpm          四轮轮速，单位rpm，顺序为[L1; R1; L2; R2]。
%   wheelTorqueLimit  四轮对称轮端扭矩上限，单位Nm。
%#codegen

motorToWheelGearRatio = 11;
drivelineEfficiency = 0.99;

% 模型内部统一使用4x1轮速列向量，非法或缺失轮速按0 rpm处理。
wheelRpm = wheelRpm(:);
if numel(wheelRpm) ~= 4
    wheelRpm = zeros(4, 1);
end
wheelRpm(~isfinite(wheelRpm)) = 0;

% 电机转速由轮速乘减速比得到，再把电机端限扭折算回轮端限扭。
motorRpm = abs(wheelRpm) * motorToWheelGearRatio;
motorTorqueLimit = localMotorTorqueLimit(motorRpm);
wheelTorqueLimit = motorTorqueLimit * motorToWheelGearRatio * ...
    drivelineEfficiency;
end

function motorTorqueLimit = localMotorTorqueLimit(motorRpm)
% 原始表格的转速轴按0.1 krpm记录，这里换算为电机rpm。
rpmTable = 10 * [0; 128.04; 256.08; 384.12; 512.16; 585.71; 640.2; ...
    768.25; 896.29; 1024.3; 1152.4; 1256.64; 1536.5; 1949.56];
torqueTable = [21; 21; 21; 21; 21; 21; 19.2127; 16.0104; ...
    13.7232; 12.0082; 10.6734; 9.788; 5.8298; 0];

% 超出最高转速时按0 Nm外推，避免高速区继续输出驱动扭矩。
motorTorqueLimit = interp1(rpmTable, torqueTable, motorRpm, 'linear', 0);
motorTorqueLimit = max(motorTorqueLimit, 0);
end
