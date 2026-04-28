% 将模型中的轮胎载荷估算MATLAB Function更新为带物理保护的版本。
% 脚本通过Stateflow API写入chart.Script，避免手工编辑Simulink块。
repoRoot = fileparts(fileparts(mfilename('fullpath')));
modelPath = fullfile(repoRoot, 'control_EVO', 'DYC_1_9_test.slx');
modelName = 'DYC_1_9_test';
blockPath = [modelName '/Subsystem/MATLAB Function2'];

scriptLines = {
'function [Fx_fl, Fy_fl, Fz_fl, Fx_fr, Fy_fr, Fz_fr, Fx_rl, Fy_rl, Fz_rl, Fx_rr, Fy_rr, Fz_rr] = calculateTireForces(ax, ay)'
'% 根据质心处纵向/横向加速度估算四轮垂向载荷。'
'% ax 和 ay 的单位应为 m/s^2，上游 Gain 已将 CarSim 的 g 单位转换为 m/s^2。'
''
'm = 300;      % 整车质量，kg'
'g = 9.81;     % 重力加速度，m/s^2'
'Lf = 0.9;     % 质心到前轴距离，m'
'Lr = 0.8;     % 质心到后轴距离，m'
'L = Lf + Lr;  % 轴距，m'
'h = 0.3;      % 质心高度，m'
'tf = 1.2;     % 前轮距，m'
'tr = 1.2;     % 后轮距，m'
''
'% 纵向载荷转移：ax 为正时，载荷由前轴向后轴转移。'
'Fzf = m*g*Lr/L - m*ax*h/L;'
'Fzr = m*g*Lf/L + m*ax*h/L;'
''
'% 横向载荷转移：沿用当前模型约定，ay 为正时右侧车轮增载、左侧车轮减载。'
'% 先按动态前/后轴载荷分别计算该轴内部的左右载荷转移量。'
'dFzf_lat = (Fzf/g) * ay * h / tf;'
'dFzr_lat = (Fzr/g) * ay * h / tr;'
''
'% 四轮垂向载荷。'
'Fz_fl = Fzf/2 - dFzf_lat;'
'Fz_fr = Fzf/2 + dFzf_lat;'
'Fz_rl = Fzr/2 - dFzr_lat;'
'Fz_rr = Fzr/2 + dFzr_lat;'
''
'% 物理保护：垂向载荷不允许为负，并将总载荷归一回整车重量。'
'Fz = max([Fz_fl; Fz_fr; Fz_rl; Fz_rr], 0);'
'sumFz = sum(Fz);'
'targetTotal = m*g;'
'if sumFz > 1e-6'
'    Fz = Fz * targetTotal / sumFz;'
'end'
''
'Fz_fl = Fz(1);'
'Fz_fr = Fz(2);'
'Fz_rl = Fz(3);'
'Fz_rr = Fz(4);'
''
'% 当前块只估算垂向载荷，纵向/横向轮胎力暂不在这里估算。'
'Fx_fl = 0;'
'Fy_fl = 0;'
'Fx_fr = 0;'
'Fy_fr = 0;'
'Fx_rl = 0;'
'Fy_rl = 0;'
'Fx_rr = 0;'
'Fy_rr = 0;'
'end'
};
newScript = strjoin(scriptLines, newline);

open_system(modelPath);
rt = sfroot;
chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', blockPath);
if isempty(chart)
    error('Cannot find MATLAB Function chart: %s', blockPath);
end
chart.Script = char(newScript);
save_system(modelName);
close_system(modelName, 0);
fprintf('Updated %s\n', blockPath);
