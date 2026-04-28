function baseWheelTorque = DYC_base_motor_torque(throttle, varargin)
%DYC_BASE_MOTOR_TORQUE 根据油门开度生成四轮基础轮端驱动扭矩。
%   throttle          油门开度，期望范围为0~1，函数内部会做限幅保护。
%   baseWheelTorque   踏板对应的基础轮端扭矩，顺序为[L1; R1; L2; R2]。
%   该函数只表达驾驶员扭矩意图，不按轮速、电机外特性或总功率限扭。
%#codegen

% 当前基础模型只表达驾驶员油门意图：电机峰值扭矩经减速比和传动效率折算到轮端。
maxMotorTorqueNm = 21;
motorToWheelGearRatio = 11;
drivelineEfficiency = 0.99;

% 油门限幅后四轮平均给同一基础扭矩，后续DYC/QP再叠加差动分配。
throttleCmd = min(max(throttle, 0), 1);
baseTorquePerWheel = throttleCmd * maxMotorTorqueNm * ...
    motorToWheelGearRatio * drivelineEfficiency;
baseWheelTorque = baseTorquePerWheel * ones(4, 1);

% 保留可选输入形参，兼容旧调用点传入wheelRpm的形式。
if nargin > 1
    unusedInput = varargin{1}; %#ok<NASGU>
end
end
