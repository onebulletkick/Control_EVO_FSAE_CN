function cfg = DYC_tire_lookup_config()
%DYC_TIRE_LOOKUP_CONFIG 轮胎控制查表选择配置。
% mode 可选 'hoosier' 或 'pacejka'。modelFile 留空时使用对应默认模型文件。
% 该文件是控制器默认查表模式的代码侧来源，仿真/测试若要临时覆盖需显式打开环境变量入口。

cfg = struct;
cfg.mode = 'pacejka';                 % 默认使用Round9 Pacejka生成的控制查表。
cfg.modelFile = '';                   % 留空时由调用方按mode拼接默认模型文件路径。
cfg.allowEnvironmentOverride = false; % 默认禁止环境变量改写，避免仿真结果被外部状态悄悄影响。
end
