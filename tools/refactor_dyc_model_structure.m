function summary = refactor_dyc_model_structure(modelFile)
%REFACTOR_DYC_MODEL_STRUCTURE 将DYC_1_9_test控制层整理成可读组件。
%   该脚本只做Simulink结构整理：保留原算法块，把相关块包进命名清晰的子系统。

if nargin < 1 || strlength(string(modelFile)) == 0
    repoRoot = fileparts(fileparts(mfilename('fullpath')));
    modelFile = fullfile(repoRoot, 'control_EVO', 'DYC_1_9_test.slx');
end

modelFile = char(modelFile);
[modelFolder, modelName] = fileparts(modelFile);
if bdIsLoaded(modelName) && strcmp(get_param(modelName, 'Dirty'), 'on')
    error('refactor_dyc_model_structure:dirtyLoadedModel', ...
        'Model %s is already loaded with unsaved changes. Save or close it before refactoring.', modelName);
end

backupFile = createTimestampedBackup(modelFile, modelName, modelFolder);
if ~isempty(modelFolder)
    addpath(modelFolder);
end

open_system(modelFile);
model = modelName;

controlRoot = findControlRoot(model);
if endsWith(controlRoot, '/Subsystem')
    set_param(controlRoot, 'Name', 'EVO_Control_System');
end
controlRoot = [model '/EVO_Control_System'];

makeGotoTagsGlobal(controlRoot);
renameBlocks(model);

groups = componentGroups();
for idx = 1:numel(groups)
    wrapComponent(model, controlRoot, groups(idx).name, groups(idx).sids);
end

nameComponentInterfaces(controlRoot);
terminateUnusedSensorChannels([controlRoot '/VehicleStateInput']);
terminateReservedFxTags([controlRoot '/DiagnosticsLogging']);
positionComponents(model, controlRoot);
addArchitectureAnnotation(controlRoot);

save_system(model);

summary = struct;
summary.model = model;
summary.modelFile = get_param(model, 'FileName');
summary.backupFile = backupFile;
summary.controlRoot = controlRoot;
summary.components = {groups.name};
summary.gotoTagsGlobal = countGlobalGotoTags(controlRoot);
end

function backupFile = createTimestampedBackup(modelFile, modelName, modelFolder)
% 修改模型前创建时间戳备份，便于结构调整出现问题时手工回退。
if ~isfile(modelFile)
    error('refactor_dyc_model_structure:missingModelFile', ...
        'Model file not found: %s.', modelFile);
end

timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
backupFile = fullfile(modelFolder, sprintf('%s.before_structure_%s.slx', modelName, timestamp));
suffix = 1;
while isfile(backupFile)
    backupFile = fullfile(modelFolder, sprintf('%s.before_structure_%s_%02d.slx', ...
        modelName, timestamp, suffix));
    suffix = suffix + 1;
end
copyfile(modelFile, backupFile);
end

function controlRoot = findControlRoot(model)
% 兼容重构前的Subsystem和重构后的EVO_Control_System两种根节点名称。
candidate = [model '/EVO_Control_System'];
if getSimulinkBlockHandle(candidate) ~= -1
    controlRoot = candidate;
    return;
end

candidate = [model '/Subsystem'];
if getSimulinkBlockHandle(candidate) ~= -1
    controlRoot = candidate;
    return;
end

error('refactor_dyc_model_structure:rootNotFound', ...
    'Cannot find Subsystem or EVO_Control_System in model %s.', model);
end

function makeGotoTagsGlobal(controlRoot)
% 顶层Goto/From跨组件通信时使用global可见性，避免封装后信号断开。
gotos = find_system(controlRoot, 'SearchDepth', 1, 'BlockType', 'Goto');
for idx = 1:numel(gotos)
    set_param(gotos{idx}, 'TagVisibility', 'global');
end
end

function n = countGlobalGotoTags(controlRoot)
gotos = find_system(controlRoot, 'LookUnderMasks', 'all', ...
    'BlockType', 'Goto');
n = 0;
for idx = 1:numel(gotos)
    if strcmp(get_param(gotos{idx}, 'TagVisibility'), 'global')
        n = n + 1;
    end
end
end

function renameBlocks(model)
% 将原始SID对应的关键块改成工程语义名称，便于后续按名称连线和阅读。
renames = {
    '3',    'CarSimSensorDemux'
    '1253', 'Vx_kph_to_mps'
    '13',   'SteerAngleSum'
    '14',   'SteerAverageDeg'
    '5',    'Goto_Vx_mps'
    '12',   'Goto_Steer_deg'
    '3211', 'Ax_g_to_mps2'
    '3212', 'Ay_g_to_mps2'
    '3183', 'TireForceEstimator'
    '3189', 'Mu_Assumed'
    '3190', 'Goto_Mu'
    '3277', 'Fx_RequestZero'
    '3278', 'Goto_Fx_request'
    '4',    'ReferenceTargetGenerator'
    '3223', 'PID_YawMomentController'
    '3268', 'YawMomentControlMode'
    '3284', 'TorqueRequestManager'
    '3285', 'BaseTorqueDemux'
    '3296', 'RawBaseTorqueDemux'
    '148',  'AllocationInputMux'
    '2753', 'SimpleLoadTransferAllocation'
    '2',    'SimpleTorqueMux'
    '3267', 'TorqueAllocationMode'
    '3287', 'FinalMotorLimiter'
    '177',  'WheelTorqueSaturation'
    '3216', 'DrivelineGain'
    };

for idx = 1:size(renames, 1)
    path = sidPath(model, renames{idx, 1});
    if getSimulinkBlockHandle(path) ~= -1
        safeSetName(path, renames{idx, 2});
    end
end
end

function safeSetName(blockPath, newName)
if strcmp(get_param(blockPath, 'Name'), newName)
    return;
end
set_param(blockPath, 'Name', newName);
end

function groups = componentGroups()
% 按数据流阶段定义组件分组；每组中的SID来自当前模型基线。
groups = struct('name', {}, 'sids', {});

groups(end+1) = struct( ...
    'name', 'VehicleStateInput', ...
    'sids', ["3","1253","13","14","5","12","3211","3212", ...
             "3273","8","9","3183","3188","3191","3192", ...
             "3193","3194","3195","3196","3197","3198", ...
             "3200","3201","3202","3189","3190","3277", ...
             "3278","3283","3279"]);

groups(end+1) = struct( ...
    'name', 'ControlTargetGeneration', ...
    'sids', ["15","17","1267","4","22","23"]);

groups(end+1) = struct( ...
    'name', 'YawMomentControl', ...
    'sids', ["385","386","387","388","549","550","410", ...
             "411","412","413","499","3230","3223","3224", ...
             "1269","780","3226","1243","3268","788","3274"]);

groups(end+1) = struct( ...
    'name', 'TorqueRequest', ...
    'sids', ["3284","3285","3296"]);

groups(end+1) = struct( ...
    'name', 'TorqueAllocation', ...
    'sids', ["149","150","151","152","153","154","155","156", ...
             "157","161","171","182","148","2753","2","3264", ...
             "3292","3293","3301","3267"]);

groups(end+1) = struct( ...
    'name', 'ActuatorOutputLimiter', ...
    'sids', ["3287","177","3216"]);

groups(end+1) = struct( ...
    'name', 'DiagnosticsLogging', ...
    'sids', ["19","20","21","24","25","26","1299","3263", ...
             "3275","3276","3282","3288","3289","3290", ...
             "3291","3294","3295","3297","3298","3299"]);
end

function wrapComponent(model, controlRoot, componentName, sids)
% 将同一功能阶段的块包进子系统，同时保留原有块和信号行为。
if getSimulinkBlockHandle([controlRoot '/' componentName]) ~= -1
    return;
end

handles = [];
for idx = 1:numel(sids)
    path = sidPath(model, sids(idx));
    handle = getSimulinkBlockHandle(path);
    if handle == -1
        error('refactor_dyc_model_structure:blockMissing', ...
            'Missing block SID %s while creating %s.', sids(idx), componentName);
    end
    parent = get_param(path, 'Parent');
    if ~strcmp(parent, controlRoot)
        error('refactor_dyc_model_structure:wrongParent', ...
            'Block %s is not at the expected control-root level.', path);
    end
    handles(end+1) = handle; %#ok<AGROW>
end

Simulink.BlockDiagram.createSubsystem(handles, ...
    'Name', componentName, 'MakeNameUnique', 'off');

componentPath = [controlRoot '/' componentName];
if getSimulinkBlockHandle(componentPath) == -1
    error('refactor_dyc_model_structure:createFailed', ...
        'Component %s was not created.', componentName);
end

set_param(componentPath, 'Description', componentDescription(componentName));
end

function text = componentDescription(componentName)
switch componentName
    case 'VehicleStateInput'
        text = '车辆状态输入整理：CarSim sensor 拆分、单位转换、轮胎力估算和基础状态 tag 输出。';
    case 'ControlTargetGeneration'
        text = '控制目标生成：根据车辆状态和转角生成理想侧偏角与理想横摆角速度。';
    case 'YawMomentControl'
        text = '横摆力矩控制：保留 PID 与 MPC 两条路径，并通过 YawMomentControlMode 选择。';
    case 'TorqueRequest'
        text = '基础扭矩请求：根据油门和制动状态生成控制分配层使用的基础扭矩。';
    case 'TorqueAllocation'
        text = '转矩分配：保留简化载荷转移分配和 QP 分配，并通过 TorqueAllocationMode 选择。';
    case 'ActuatorOutputLimiter'
        text = '执行器输出限制：统一执行电机转速限扭、最终饱和和传动输出。';
    case 'DiagnosticsLogging'
        text = '诊断记录：集中显示基础扭矩、DYC 增量、QP 诊断和最终限扭差值。';
    otherwise
        text = '';
end
end

function positionComponents(model, controlRoot)
% 按CarSim输入到执行器输出的主数据流顺序排列顶层组件。
positions = {
    'VehicleStateInput',       [340 210 690 430]
    'ControlTargetGeneration', [900 230 1250 390]
    'YawMomentControl',        [1460 150 1810 430]
    'TorqueRequest',           [900 610 1250 790]
    'TorqueAllocation',        [1460 560 1810 860]
    'ActuatorOutputLimiter',   [2020 600 2370 810]
    'DiagnosticsLogging',      [2020 170 2370 460]
    };

for idx = 1:size(positions, 1)
    path = [controlRoot '/' positions{idx, 1}];
    if getSimulinkBlockHandle(path) ~= -1
        set_param(path, 'Position', positions{idx, 2});
    end
end

safePosition([model '/EVO_Control_System'], [390 160 690 280]);
end

function nameComponentInterfaces(controlRoot)
% 给各组件Inport/Outport命名，让顶层连线表达真实物理含义。
renamePorts([controlRoot '/VehicleStateInput'], 'Inport', {
    '1', 'sensor_bus'
    });
nameVehicleStateOutputs([controlRoot '/VehicleStateInput']);

renamePorts([controlRoot '/TorqueAllocation'], 'Inport', {
    '1',  'throttle'
    '2',  'AVy_L1'
    '3',  'AVy_R1'
    '4',  'AVy_L2'
    '5',  'AVy_R2'
    '6',  'T_base_alloc_4w_N_m'
    '7',  'T_base_alloc_L2_N_m'
    '8',  'T_base_alloc_R2_N_m'
    '9',  'T_base_alloc_L1_N_m'
    '10', 'T_base_alloc_R1_N_m'
    });
renamePorts([controlRoot '/TorqueAllocation'], 'Outport', {
    '1', 'T_selected_4w_N_m'
    '2', 'QP_candidate_diag'
    '3', 'FzL1'
    '4', 'FzL2'
    '5', 'FzR1'
    '6', 'FzR2'
    });

renamePorts([controlRoot '/ActuatorOutputLimiter'], 'Outport', {
    '1', 'T_cmd_out_4w_N_m'
    '2', 'T_to_CarSim_4w_N_m'
    });

renamePorts([controlRoot '/DiagnosticsLogging'], 'Inport', {
    '1',  'T_cmd_out_4w_N_m'
    '2',  'QP_candidate_diag'
    '3',  'T_selected_4w_N_m'
    '4',  'FzL1'
    '5',  'FzL2'
    '6',  'FzR1'
    '7',  'FzR2'
    '8',  'T_base_alloc_4w_N_m'
    '9',  'T_base_alloc_L2_N_m'
    '10', 'T_base_alloc_R2_N_m'
    '11', 'T_base_raw_L1_N_m'
    '12', 'T_base_raw_R1_N_m'
    '13', 'T_base_raw_L2_N_m'
    '14', 'T_base_raw_R2_N_m'
    '15', 'T_base_alloc_L1_N_m'
    '16', 'T_base_alloc_R1_N_m'
    });

safeRename([controlRoot '/YawMomentControl/Scope1'], 'Scope_Mz_pid_vs_mpc');
safeRename([controlRoot '/VehicleStateInput/Scope2'], 'Scope_brake_status');
safeRename([controlRoot '/DiagnosticsLogging/Scope'], 'Scope_T_cmd_out');
safeRename([controlRoot '/DiagnosticsLogging/Scope5'], 'Scope_T_selected_alloc');
safeRename([controlRoot '/DiagnosticsLogging/Scope3'], 'Scope_Fz_distribution');
safeRename([controlRoot '/DiagnosticsLogging/beta'], 'Scope_beta_ref_actual');
safeRename([controlRoot '/DiagnosticsLogging/wr'], 'Scope_wr_ref_actual');
end

function nameVehicleStateOutputs(subsystemPath)
outports = find_system(subsystemPath, 'SearchDepth', 1, 'BlockType', 'Outport');
if numel(outports) >= 10
    renamePorts(subsystemPath, 'Outport', {
        '1',  'throttle'
        '2',  'sensor_ch10_unused'
        '3',  'sensor_ch11_unused'
        '4',  'sensor_ch12_unused'
        '5',  'sensor_ch13_unused'
        '6',  'AVy_L1'
        '7',  'AVy_R1'
        '8',  'AVy_L2'
        '9',  'AVy_R2'
        '10', 'brake_status'
        });
    deleteUnusedOutports(subsystemPath, { ...
        'sensor_ch10_unused', 'sensor_ch11_unused', ...
        'sensor_ch12_unused', 'sensor_ch13_unused'});
else
    renamePorts(subsystemPath, 'Outport', {
        '1', 'throttle'
        '2', 'AVy_L1'
        '3', 'AVy_R1'
        '4', 'AVy_L2'
        '5', 'AVy_R2'
        '6', 'brake_status'
        });
end
end

function deleteUnusedOutports(subsystemPath, names)
for idx = 1:numel(names)
    path = [subsystemPath '/' names{idx}];
    if getSimulinkBlockHandle(path) ~= -1
        delete_block(path);
    end
end
end

function terminateUnusedSensorChannels(vehicleStatePath)
if getSimulinkBlockHandle(vehicleStatePath) == -1
    return;
end

demuxPath = [vehicleStatePath '/CarSimSensorDemux'];
if getSimulinkBlockHandle(demuxPath) == -1
    return;
end

ports = [10, 11, 12, 13];
names = {'Term_sensor_ch10_unused', 'Term_sensor_ch11_unused', ...
    'Term_sensor_ch12_unused', 'Term_sensor_ch13_unused'};

for idx = 1:numel(ports)
    termPath = [vehicleStatePath '/' names{idx}];
    if getSimulinkBlockHandle(termPath) == -1
        add_block('simulink/Sinks/Terminator', termPath);
    end

    lineHandles = get_param(demuxPath, 'LineHandles');
    lineHandle = lineHandles.Outport(ports(idx));
    if lineHandle ~= -1
        dst = get_param(lineHandle, 'DstBlockHandle');
        target = getSimulinkBlockHandle(termPath);
        if isempty(dst) || ~any(dst == target)
            delete_line(lineHandle);
            lineHandle = -1;
        end
    end

    if lineHandle == -1
        add_line(vehicleStatePath, ...
            sprintf('CarSimSensorDemux/%d', ports(idx)), ...
            sprintf('%s/1', names{idx}), 'autorouting', 'on');
    end
end
end

function terminateReservedFxTags(diagnosticsPath)
if getSimulinkBlockHandle(diagnosticsPath) == -1
    return;
end

tags = {'FxL1', 'FxR1', 'FxL2', 'FxR2'};
for idx = 1:numel(tags)
    fromName = ['From_' tags{idx} '_unused'];
    termName = ['Term_' tags{idx} '_unused'];
    fromPath = [diagnosticsPath '/' fromName];
    termPath = [diagnosticsPath '/' termName];

    if getSimulinkBlockHandle(fromPath) == -1
        add_block('simulink/Signal Routing/From', fromPath, ...
            'GotoTag', tags{idx});
    else
        set_param(fromPath, 'GotoTag', tags{idx});
    end

    if getSimulinkBlockHandle(termPath) == -1
        add_block('simulink/Sinks/Terminator', termPath);
    end

    ensureLine(diagnosticsPath, [fromName '/1'], [termName '/1']);
end
end

function ensureLine(parentPath, source, destination)
sourceParts = split(string(source), '/');
destinationParts = split(string(destination), '/');
sourcePath = [parentPath '/' char(sourceParts(1))];
destinationPath = [parentPath '/' char(destinationParts(1))];
sourcePort = str2double(sourceParts(2));
destinationPort = str2double(destinationParts(2));

if getSimulinkBlockHandle(sourcePath) ~= -1
    sourceLines = get_param(sourcePath, 'LineHandles');
    if numel(sourceLines.Outport) >= sourcePort && ...
            sourceLines.Outport(sourcePort) ~= -1
        return;
    end
end

if getSimulinkBlockHandle(destinationPath) ~= -1
    destinationLines = get_param(destinationPath, 'LineHandles');
    if numel(destinationLines.Inport) >= destinationPort && ...
            destinationLines.Inport(destinationPort) ~= -1
        return;
    end
end

try
    add_line(parentPath, source, destination, 'autorouting', 'on');
catch ME
    if ~contains(ME.identifier, 'LineExists') && ...
            ~contains(ME.message, 'already has a line') && ...
            ~contains(ME.message, '已有信号线连接')
        rethrow(ME);
    end
end
end

function renamePorts(subsystemPath, blockType, portMap)
if getSimulinkBlockHandle(subsystemPath) == -1
    return;
end

for idx = 1:size(portMap, 1)
    port = portMap{idx, 1};
    newName = portMap{idx, 2};
    blocks = find_system(subsystemPath, 'SearchDepth', 1, ...
        'BlockType', blockType, 'Port', port);
    if ~isempty(blocks)
        safeRename(blocks{1}, newName);
    end
end
end

function safeRename(blockPath, newName)
if getSimulinkBlockHandle(blockPath) == -1
    return;
end
if strcmp(get_param(blockPath, 'Name'), newName)
    return;
end
set_param(blockPath, 'Name', newName);
end

function safePosition(blockPath, position)
if getSimulinkBlockHandle(blockPath) ~= -1
    set_param(blockPath, 'Position', position);
end
end

function addArchitectureAnnotation(controlRoot)
% 在模型中保留架构注释，帮助后来者理解顶层数据流。
existing = find_system(controlRoot, 'FindAll', 'on', 'Type', 'annotation');
for idx = 1:numel(existing)
    try
        text = get_param(existing(idx), 'Text');
        if contains(string(text), '组件化主数据流')
            delete(existing(idx));
        end
    catch
    end
end

note = Simulink.Annotation(controlRoot, ...
    sprintf(['组件化主数据流：VehicleStateInput -> ControlTargetGeneration -> ', ...
             'YawMomentControl -> TorqueRequest -> TorqueAllocation -> ', ...
             'ActuatorOutputLimiter；DiagnosticsLogging 只负责观测记录。']));
note.Position = [320 40 2470 95];
end

function path = sidPath(model, sid)
path = Simulink.ID.getFullName([model ':' char(sid)]);
end
