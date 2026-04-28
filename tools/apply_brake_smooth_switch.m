% 在旧版DYC模型中插入制动平滑使能链路，使刹车状态能平滑切断基础驱动扭矩。
% 脚本会重复删除自己创建的块，因此可多次运行；不负责重建其他人工连线。
repoRoot = fileparts(fileparts(mfilename('fullpath')));
modelPath = fullfile(repoRoot, 'control_EVO', 'DYC_1_9_test.slx');
modelName = 'DYC_1_9_test';
subsys = [modelName '/Subsystem'];

open_system(modelPath);

% 将 CarSim 的 Bk_Stat 正式发布为 brake_status 信号。
brakeGoto = Simulink.ID.getFullName([modelName ':3244']);
set_param(brakeGoto, 'GotoTag', 'brake_status');

% 清理本脚本创建过的块，便于重复执行。
ownedBlocks = {
    [subsys '/brake_status_from']
    [subsys '/brake_status_double']
    [subsys '/enable_one']
    [subsys '/enable_raw']
    [subsys '/enable_smooth']
    [subsys '/torque_enable_product']
};
for i = 1:numel(ownedBlocks)
    if blockExists(ownedBlocks{i})
        delete_block(ownedBlocks{i});
    end
end

% 断开 Saturation 输入端当前连线，后续改接平滑后的扭矩。
disconnectInputIfConnected([subsys '/Saturation'], 1);

% 构造 brake_status -> 1 - brake_status -> Rate Limiter -> torque scale。
add_block('simulink/Signal Routing/From', [subsys '/brake_status_from'], ...
    'GotoTag', 'brake_status', ...
    'Position', [1510 945 1575 975]);

add_block('simulink/Signal Attributes/Data Type Conversion', [subsys '/brake_status_double'], ...
    'OutDataTypeStr', 'double', ...
    'Position', [1620 940 1690 980]);

add_block('simulink/Sources/Constant', [subsys '/enable_one'], ...
    'Value', '1', ...
    'Position', [1620 875 1655 905]);

add_block('simulink/Math Operations/Sum', [subsys '/enable_raw'], ...
    'Inputs', '+-', ...
    'IconShape', 'rectangular', ...
    'Position', [1740 900 1775 950]);

add_block('simulink/Discontinuities/Rate Limiter', [subsys '/enable_smooth'], ...
    'RisingSlewLimit', '5', ...
    'FallingSlewLimit', '-20', ...
    'InitialCondition', '1', ...
    'Position', [1825 905 1905 945]);

add_block('simulink/Math Operations/Product', [subsys '/torque_enable_product'], ...
    'Inputs', '**', ...
    'Position', [1845 660 1895 730]);

safeAddLine(subsys, 'brake_status_from/1', 'brake_status_double/1');
safeAddLine(subsys, 'brake_status_double/1', 'enable_raw/2');
safeAddLine(subsys, 'enable_one/1', 'enable_raw/1');
safeAddLine(subsys, 'enable_raw/1', 'enable_smooth/1');
safeAddLine(subsys, 'enable_smooth/1', 'torque_enable_product/2');

% 保留原 Scope5 对原始扭矩的观察，只把进入 Saturation 的路径改为平滑后的扭矩。
safeAddLine(subsys, 'Mux/1', 'Scope5/1');
safeAddLine(subsys, 'Mux/1', 'torque_enable_product/1');
safeAddLine(subsys, 'torque_enable_product/1', 'Saturation/1');

save_system(modelName);
close_system(modelName, 0);
fprintf('Brake smooth switch inserted into %s\n', subsys);

function tf = blockExists(blockPath)
% 判断块是否存在，供脚本重复运行时安全清理。
try
    get_param(blockPath, 'Handle');
    tf = true;
catch
    tf = false;
end
end

function safeAddLine(systemPath, srcPort, dstPort)
% 添加信号线；已连接或对象已被清理时不视为致命错误。
try
    add_line(systemPath, srcPort, dstPort, 'autorouting', 'on');
catch ME
    alreadyConnected = contains(ME.message, 'already') || contains(ME.message, '已有信号线连接');
    invalidObject = contains(ME.message, 'Invalid Simulink object name');
    if ~alreadyConnected && ~invalidObject
        rethrow(ME);
    end
end
end

function disconnectInputIfConnected(blockPath, portNumber)
% 删除指定输入端口已有连线，便于后续改接新路径。
ports = get_param(blockPath, 'PortHandles');
lineHandle = get_param(ports.Inport(portNumber), 'Line');
if lineHandle ~= -1
    delete_line(lineHandle);
end
end
