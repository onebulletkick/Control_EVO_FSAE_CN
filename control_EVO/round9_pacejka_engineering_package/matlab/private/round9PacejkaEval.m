function y = round9PacejkaEval(submodel, slip, input)
%ROUND9PACEJKAEVAL 计算简化稳态Magic Formula轮胎力/力矩。

slip = slip(:);
if ~isfield(submodel, "params") || isempty(submodel.params) || any(~isfinite(submodel.params))
    y = NaN(size(slip));
    return
end

theta = double(submodel.params(:).');
if numel(theta) < 10
    theta(10) = 0;
end

% 以拟合中心点为基准，把载荷、倾角、胎压和速度转成无量纲偏差。
center = submodel.center;
fz = input.FZ_N(:);
ia = input.IA_deg(:);
pressure = input.P_kPa(:);
speed = input.V_kph(:);

logB = theta(1);
C = theta(2);
D0 = theta(3);
E = theta(4);
Sh = theta(5);
Sv0 = theta(6);
dFz = theta(7);
dIA = theta(8);
dP = theta(9);
dV = theta(10);

zFz = (fz - center.FZ_N) ./ max(abs(center.FZ_N), eps);
zIA = (ia - center.IA_deg) ./ 5;
zP = (pressure - center.P_kPa) ./ max(abs(center.P_kPa), eps);
zV = (speed - center.V_kph) ./ max(abs(center.V_kph), eps);

% D项随工况变化，B/C/E/Sh/Sv描述基础Magic Formula形状和偏置。
B = exp(logB);
shapeScale = max(0.05, 1 + dFz .* zFz + dIA .* zIA + dP .* zP + dV .* zV);
D = fz .* D0 .* shapeScale;
Sv = fz .* Sv0;
slipEff = slip + Sh;
phi = B .* slipEff;

y = Sv + D .* sin(C .* atan(phi - E .* (phi - atan(phi))));
end
