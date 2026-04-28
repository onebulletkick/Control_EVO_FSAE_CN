function out = lookup_hoosier43075_limits(Fz, FyUsed, alphaRad, kappa, modelInput)
%LOOKUP_HOOSIER43075_LIMITS 查询 Hoosier 43075 控制用轮胎边界。
% 该入口保留向后兼容；通用实现见 lookup_tire_control_limits。
% 新控制链可直接调用通用入口，本函数用于旧测试和历史脚本不必改名。

if nargin < 2
    FyUsed = [];
end
if nargin < 3
    alphaRad = [];
end
if nargin < 4
    kappa = [];
end
if nargin < 5 || isempty(modelInput)
    modelInput = localHoosierModelFile();
end

out = lookup_tire_control_limits(Fz, FyUsed, alphaRad, kappa, modelInput);
end

function modelFile = localHoosierModelFile()
% 测试可用环境变量临时指定模型文件，正常项目路径使用outputs下的默认产物。
overrideFile = getenv('HOOSIER43075_MODEL_FILE');
if ~isempty(overrideFile)
    modelFile = char(overrideFile);
    return;
end

baseFolder = fileparts(mfilename('fullpath'));
modelFile = fullfile(baseFolder, 'outputs', 'hoosier43075_control_model.mat');
end
