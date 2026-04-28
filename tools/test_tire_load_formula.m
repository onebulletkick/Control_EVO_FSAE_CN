% 验证轮胎垂向载荷估算公式：总载荷守恒，横向加速度符号对应左右增载关系。
cases = [
    0,     0;
    2.0,   0;
   -4.0,   0;
    0,     4.0;
    0,    -4.0;
    4.0,   6.0
];

m = 300;
g = 9.81;
tol = 1e-8;

for i = 1:size(cases, 1)
    % 每组工况只检查垂向载荷，不验证纵向/横向轮胎力。
    ax = cases(i, 1);
    ay = cases(i, 2);
    [~, ~, Fz_fl, ~, ~, Fz_fr, ~, ~, Fz_rl, ~, ~, Fz_rr] = localCalculateTireForces(ax, ay);
    Fz = [Fz_fl, Fz_fr, Fz_rl, Fz_rr];

    assert(all(Fz >= -tol), 'negative normal load at case %d', i);
    assert(abs(sum(Fz) - m*g) < 1e-6, 'total load mismatch at case %d', i);

    if ay > 0
        assert(Fz_fr > Fz_fl, 'positive ay should load front right in current sign convention');
        assert(Fz_rr > Fz_rl, 'positive ay should load rear right in current sign convention');
    elseif ay < 0
        assert(Fz_fl > Fz_fr, 'negative ay should load front left in current sign convention');
        assert(Fz_rl > Fz_rr, 'negative ay should load rear left in current sign convention');
    end

    fprintf('case %d ax=%g ay=%g Fz=[%.3f %.3f %.3f %.3f] sum=%.3f\n', ...
        i, ax, ay, Fz_fl, Fz_fr, Fz_rl, Fz_rr, sum(Fz));
end

fprintf('All tire load formula checks passed.\n');

function [Fx_fl, Fy_fl, Fz_fl, Fx_fr, Fy_fr, Fz_fr, Fx_rl, Fy_rl, Fz_rl, Fx_rr, Fy_rr, Fz_rr] = localCalculateTireForces(ax, ay)
% 与模型MATLAB Function保持一致的本地副本，用于不打开Simulink时快速验证公式。
m = 300;
g = 9.81;
Lf = 0.9;
Lr = 0.8;
L = Lf + Lr;
h = 0.3;
tf = 1.2;
tr = 1.2;

Fzf = m*g*Lr/L - m*ax*h/L;
Fzr = m*g*Lf/L + m*ax*h/L;

dFzf_lat = (Fzf/g) * ay * h / tf;
dFzr_lat = (Fzr/g) * ay * h / tr;

Fz_fl = Fzf/2 - dFzf_lat;
Fz_fr = Fzf/2 + dFzf_lat;
Fz_rl = Fzr/2 - dFzr_lat;
Fz_rr = Fzr/2 + dFzr_lat;

Fz = max([Fz_fl; Fz_fr; Fz_rl; Fz_rr], 0);
sumFz = sum(Fz);
targetTotal = m*g;
if sumFz > 1e-6
    Fz = Fz * targetTotal / sumFz;
end

Fz_fl = Fz(1);
Fz_fr = Fz(2);
Fz_rl = Fz(3);
Fz_rr = Fz(4);

Fx_fl = 0;
Fy_fl = 0;
Fx_fr = 0;
Fy_fr = 0;
Fx_rl = 0;
Fy_rl = 0;
Fx_rr = 0;
Fy_rr = 0;
end
