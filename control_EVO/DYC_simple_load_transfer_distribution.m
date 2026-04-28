function torqueCommand = DYC_simple_load_transfer_distribution(u)
%DYC_SIMPLE_LOAD_TRANSFER_DISTRIBUTION 载荷比例法输出基础扭矩叠加DYC差动扭矩。
%   输入u的前12路与QP一致，13:16为T_base [L1; R1; L2; R2]。
%#codegen

% 车辆几何参数统一从公共入口读取，避免与QP分配或MPC参数漂移。
veh = DYC_vehicle_params;
r = veh.r;
tf = veh.tf;
tr = veh.tr;

% 输入u沿用QP S-Function的通道顺序；本函数只用垂向载荷、目标横摆力矩和基础扭矩。
FzL1 = u(5);
FzL2 = u(6);
FzR1 = u(7);
FzR2 = u(8);
Mz = u(11);
baseTorque = [u(13); u(14); u(15); u(16)];
baseTorque(~isfinite(baseTorque)) = 0;

deltaTorque = zeros(4, 1);
Fztotal = FzL1 + FzR1 + FzL2 + FzR2;
if isfinite(Fztotal) && Fztotal > 1e-6 && isfinite(Mz)
    % 按四轮垂向载荷比例分摊目标横摆力矩，再换算成左右轮差动扭矩。
    MZL1 = FzL1/Fztotal*Mz;
    MZR1 = FzR1/Fztotal*Mz;
    MZL2 = FzL2/Fztotal*Mz;
    MZR2 = FzR2/Fztotal*Mz;

    deltaTorque(1) = -2*MZL1*r/tf;
    deltaTorque(2) =  2*MZR1*r/tf;
    deltaTorque(3) = -2*MZL2*r/tr;
    deltaTorque(4) =  2*MZR2*r/tr;
end

% 若某个车轮垂向载荷无效或为0，该轮不承接DYC差动扭矩，保留基础扭矩。
if FzL1 <= 0 || ~isfinite(FzL1)
    deltaTorque(1) = 0;
end
if FzR1 <= 0 || ~isfinite(FzR1)
    deltaTorque(2) = 0;
end
if FzL2 <= 0 || ~isfinite(FzL2)
    deltaTorque(3) = 0;
end
if FzR2 <= 0 || ~isfinite(FzR2)
    deltaTorque(4) = 0;
end

torqueCommand = baseTorque + deltaTorque;
end
