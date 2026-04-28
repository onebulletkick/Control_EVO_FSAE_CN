% 验证刹车平滑使能逻辑：踩刹车快速切断，松刹车按限速恢复。
dt = 0.001;
t = 0:dt:0.5;
brake = zeros(size(t));
brake(t >= 0.10 & t < 0.25) = 1;

enable = zeros(size(t));
enable(1) = 1;
risingLimit = 5;
fallingLimit = -20;

for k = 2:numel(t)
    % Rate Limiter等效离散实现，用于和Simulink块参数保持一致。
    target = 1 - brake(k);
    delta = target - enable(k-1);
    delta = min(delta, risingLimit * dt);
    delta = max(delta, fallingLimit * dt);
    enable(k) = enable(k-1) + delta;
end

assert(abs(enable(1) - 1) < 1e-12, '初始未刹车时使能应为 1');
assert(enable(find(t >= 0.15, 1)) < 1e-9, '刹车 0.05 s 后使能应衰减到 0');
assert(enable(find(t >= 0.45, 1)) > 0.99, '松开刹车 0.20 s 后使能应恢复到 1');

rawTorque = [10; -20; 30; -40];
% 原始扭矩乘以平滑使能，检查刹车窗口内驱动输出被切断。
assert(all(rawTorque * enable(find(t >= 0.15, 1)) == 0), '刹车持续时输出扭矩应为 0');
assert(norm(rawTorque * enable(1) - rawTorque) < 1e-12, '未刹车时输出应保持原始扭矩');

fprintf('Brake smooth switch checks passed. min_enable=%.3f max_enable=%.3f\n', min(enable), max(enable));
