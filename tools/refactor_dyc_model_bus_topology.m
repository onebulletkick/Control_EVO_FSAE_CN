function summary = refactor_dyc_model_bus_topology(modelFile)
%REFACTOR_DYC_MODEL_BUS_TOPOLOGY 用虚拟Bus减少顶层标量连线。
%   该脚本假设模型已经过refactor_dyc_model_structure组件化；只调整接口和连线，不改算法块。

if nargin < 1 || strlength(string(modelFile)) == 0
    repoRoot = fileparts(fileparts(mfilename('fullpath')));
    modelFile = fullfile(repoRoot, 'control_EVO', 'DYC_1_9_test.slx');
end

modelFile = char(modelFile);
[modelFolder, modelName] = fileparts(modelFile);
backupFile = createTimestampedBackup(modelFile, modelName, modelFolder);
if ~isempty(modelFolder)
    addpath(modelFolder);
end

if bdIsLoaded(modelName) && strcmp(get_param(modelName, 'Dirty'), 'on')
    error('refactor_dyc_model_bus_topology:dirtyLoadedModel', ...
        'Model %s is already loaded with unsaved changes. Save or close it before refactoring.', modelName);
end

open_system(modelFile);
model = modelName;
root = [model '/EVO_Control_System'];
if getSimulinkBlockHandle(root) == -1
    error('refactor_dyc_model_bus_topology:missingRoot', ...
        'Expected EVO_Control_System in %s.', model);
end

configureVehicleStateInput(root);
configureControlTargetGeneration(root);
configureYawMomentControl(root);
configureTorqueRequest(root);
configureTorqueAllocation(root);
configureActuatorOutputLimiter(root);
configureDiagnosticsLogging(root);
rebuildTopLevelBusWiring(root);
positionTopLevelComponents(root);
addBusArchitectureAnnotation(root);
deleteDanglingLines(root);

save_system(model);

summary = struct;
summary.model = model;
summary.modelFile = get_param(model, 'FileName');
summary.backupFile = backupFile;
summary.root = root;
summary.topLevelLineCount = countTopLevelLines(root);
summary.logicalTopLevelConnectionCount = countLogicalTopLevelConnections(root);
summary.diagnosticsInputCount = numel(find_system([root '/DiagnosticsLogging'], ...
    'SearchDepth', 1, 'BlockType', 'Inport'));
end

function backupFile = createTimestampedBackup(modelFile, modelName, modelFolder)
% 修改模型前创建时间戳备份，保证Bus化调整可回退。
if ~isfile(modelFile)
    error('refactor_dyc_model_bus_topology:missingModelFile', ...
        'Model file not found: %s.', modelFile);
end

timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
backupFile = fullfile(modelFolder, sprintf('%s.before_bus_%s.slx', modelName, timestamp));
suffix = 1;
while isfile(backupFile)
    backupFile = fullfile(modelFolder, sprintf('%s.before_bus_%s_%02d.slx', ...
        modelName, timestamp, suffix));
    suffix = suffix + 1;
end
copyfile(modelFile, backupFile);
end

function configureVehicleStateInput(root)
% 将CarSim传感器、估算载荷和常量打包为VehicleStateBus。
ss = [root '/VehicleStateInput'];
ensureBusCreator(ss, 'VehicleStateBusCreator', 22, {
    'CarSimSensorDemux', 9,  'throttle'
    'CarSimSensorDemux', 8,  'brake_status'
    'Vx_kph_to_mps',     1,  'Vx_mps'
    'SteerAverageDeg',   1,  'steer_deg'
    'beta_from_vx_vy',   1,  'beta_deg'
    'CarSimSensorDemux', 4,  'omega_deg_s'
    'Ax_g_to_mps2',      1,  'ax_mps2'
    'Ay_g_to_mps2',      1,  'ay_mps2'
    'CarSimSensorDemux', 14, 'AVy_L1'
    'CarSimSensorDemux', 15, 'AVy_R1'
    'CarSimSensorDemux', 16, 'AVy_L2'
    'CarSimSensorDemux', 17, 'AVy_R2'
    'Mu_Assumed',        1,  'mu_assumed'
    'Fx_RequestZero',    1,  'Fx_request_N'
    'TireForceEstimator', 2, 'FyL1'
    'TireForceEstimator', 8, 'FyL2'
    'TireForceEstimator', 5, 'FyR1'
    'TireForceEstimator', 11, 'FyR2'
    'TireForceEstimator', 3, 'FzL1'
    'TireForceEstimator', 9, 'FzL2'
    'TireForceEstimator', 6, 'FzR1'
    'TireForceEstimator', 12, 'FzR2'
    });
ensureOutport(ss, 'VehicleStateBus', 1);
connectByPorts(ss, [ss '/VehicleStateBusCreator'], 1, ...
    [ss '/VehicleStateBus'], 1, 'VehicleStateBus');
connectToTerminator(ss, 'TireForceEstimator', 1, 'Term_FxL1_unused', 'FxL1_unused');
connectToTerminator(ss, 'TireForceEstimator', 4, 'Term_FxR1_unused', 'FxR1_unused');
connectToTerminator(ss, 'TireForceEstimator', 7, 'Term_FxL2_unused', 'FxL2_unused');
connectToTerminator(ss, 'TireForceEstimator', 10, 'Term_FxR2_unused', 'FxR2_unused');
deleteBlocksIfPresent(ss, {'throttle','AVy_L1','AVy_R1','AVy_L2','AVy_R2','brake_status', ...
    'Goto3','Goto4','Goto5','Goto13','Goto14','Goto15','Goto16','Goto17', ...
    'Goto18','Goto19','Goto23','Goto24','Goto25','Goto26', ...
    'Goto_Fx_request','Goto_Mu','Goto_Steer_deg','Goto_Vx_mps'});
setBlockPosition([ss '/VehicleStateBusCreator'], [930 350 990 670]);
setBlockPosition([ss '/VehicleStateBus'], [1080 500 1110 520]);
end

function configureControlTargetGeneration(root)
% 从VehicleStateBus取车速、转角和附着系数，生成目标响应Bus。
ss = [root '/ControlTargetGeneration'];
ensureInport(ss, 'VehicleStateBus', 1);
ensureBusSelector(ss, 'VehicleStateForTargetSelector', ...
    {'Vx_mps','steer_deg','mu_assumed'});
connectByPorts(ss, [ss '/VehicleStateBus'], 1, ...
    [ss '/VehicleStateForTargetSelector'], 1, '');
connectByPorts(ss, [ss '/VehicleStateForTargetSelector'], 1, ...
    [ss '/ReferenceTargetGenerator'], 2, 'Vx_mps');
connectByPorts(ss, [ss '/VehicleStateForTargetSelector'], 2, ...
    [ss '/ReferenceTargetGenerator'], 3, 'steer_deg');
connectByPorts(ss, [ss '/VehicleStateForTargetSelector'], 3, ...
    [ss '/ReferenceTargetGenerator'], 1, 'mu_assumed');

ensureBusCreator(ss, 'TargetBusCreator', 2, {
    'ReferenceTargetGenerator', 1, 'beta_ref_deg'
    'ReferenceTargetGenerator', 2, 'omega_ref_deg_s'
    });
ensureOutport(ss, 'TargetBus', 1);
connectByPorts(ss, [ss '/TargetBusCreator'], 1, [ss '/TargetBus'], 1, 'TargetBus');
deleteBlocksIfPresent(ss, {'From','From1','From10','Goto2','Goto6'});
setBlockPosition([ss '/VehicleStateBus'], [80 120 110 140]);
setBlockPosition([ss '/VehicleStateForTargetSelector'], [190 100 290 170]);
setBlockPosition([ss '/ReferenceTargetGenerator'], [380 95 570 185]);
setBlockPosition([ss '/TargetBusCreator'], [660 115 720 165]);
setBlockPosition([ss '/TargetBus'], [800 135 830 155]);
end

function configureYawMomentControl(root)
% 从车辆状态和目标响应Bus中选取MPC/PID所需量，并输出横摆力矩Bus。
ss = [root '/YawMomentControl'];
ensureInport(ss, 'VehicleStateBus', 1);
ensureInport(ss, 'TargetBus', 2);
ensureBusSelector(ss, 'VehicleStateForYawSelector', ...
    {'beta_deg','omega_deg_s','steer_deg','Vx_mps'});
ensureBusSelector(ss, 'TargetForYawSelector', ...
    {'beta_ref_deg','omega_ref_deg_s'});
connectByPorts(ss, [ss '/VehicleStateBus'], 1, [ss '/VehicleStateForYawSelector'], 1, '');
connectByPorts(ss, [ss '/TargetBus'], 1, [ss '/TargetForYawSelector'], 1, '');
connectByPorts(ss, [ss '/VehicleStateForYawSelector'], 1, [ss '/Gain23'], 1, 'beta_deg');
connectByPorts(ss, [ss '/VehicleStateForYawSelector'], 2, [ss '/Gain24'], 1, 'omega_deg_s');
connectByPorts(ss, [ss '/VehicleStateForYawSelector'], 3, [ss '/Gain41'], 1, 'steer_deg');
connectByPorts(ss, [ss '/VehicleStateForYawSelector'], 4, [ss '/MPC_Controller1'], 6, 'Vx_mps');
connectByPorts(ss, [ss '/TargetForYawSelector'], 1, [ss '/Gain29'], 1, 'beta_ref_deg');
connectByPorts(ss, [ss '/TargetForYawSelector'], 2, [ss '/Gain30'], 1, 'omega_ref_deg_s');

ensureBusCreator(ss, 'YawMomentBusCreator', 3, {
    'PID_YawMomentController', 1, 'Mz_pid_N_m'
    'MPC_Controller1',        1, 'Mz_mpc_N_m'
    'YawMomentControlMode',   1, 'Mz_selected_N_m'
    });
ensureOutport(ss, 'YawMomentBus', 1);
connectByPorts(ss, [ss '/YawMomentBusCreator'], 1, [ss '/YawMomentBus'], 1, 'YawMomentBus');
deleteBlocksIfPresent(ss, {'From57','From58','From59','From60','From73','From74','Goto39'});
setBlockPosition([ss '/VehicleStateBus'], [40 130 70 150]);
setBlockPosition([ss '/TargetBus'], [40 260 70 280]);
setBlockPosition([ss '/VehicleStateForYawSelector'], [160 105 270 205]);
setBlockPosition([ss '/TargetForYawSelector'], [160 245 270 315]);
setBlockPosition([ss '/YawMomentBusCreator'], [760 210 825 285]);
setBlockPosition([ss '/YawMomentBus'], [910 240 940 260]);
end

function configureTorqueRequest(root)
% 将油门和制动状态转换为基础扭矩请求Bus。
ss = [root '/TorqueRequest'];
ensureInport(ss, 'VehicleStateBus', 1);
ensureBusSelector(ss, 'VehicleStateForTorqueRequestSelector', ...
    {'brake_status','throttle'});
connectByPorts(ss, [ss '/VehicleStateBus'], 1, ...
    [ss '/VehicleStateForTorqueRequestSelector'], 1, '');
connectByPorts(ss, [ss '/VehicleStateForTorqueRequestSelector'], 1, ...
    [ss '/TorqueRequestManager'], 2, 'brake_status');
connectByPorts(ss, [ss '/VehicleStateForTorqueRequestSelector'], 2, ...
    [ss '/TorqueRequestManager'], 1, 'throttle');

ensureBusCreator(ss, 'TorqueRequestBusCreator', 9, {
    'TorqueRequestManager', 1, 'T_base_alloc_4w_N_m'
    'RawBaseTorqueDemux',  1, 'T_base_raw_L1_N_m'
    'RawBaseTorqueDemux',  2, 'T_base_raw_R1_N_m'
    'RawBaseTorqueDemux',  3, 'T_base_raw_L2_N_m'
    'RawBaseTorqueDemux',  4, 'T_base_raw_R2_N_m'
    'BaseTorqueDemux',     1, 'T_base_alloc_L1_N_m'
    'BaseTorqueDemux',     2, 'T_base_alloc_R1_N_m'
    'BaseTorqueDemux',     3, 'T_base_alloc_L2_N_m'
    'BaseTorqueDemux',     4, 'T_base_alloc_R2_N_m'
    });
ensureOutport(ss, 'TorqueRequestBus', 1);
connectByPorts(ss, [ss '/TorqueRequestBusCreator'], 1, ...
    [ss '/TorqueRequestBus'], 1, 'TorqueRequestBus');
deleteBlocksIfPresent(ss, {'brake_status','throttle', ...
    'T_base_alloc_4w_N_m','T_base_raw_L1_N_m','T_base_raw_R1_N_m', ...
    'T_base_raw_L2_N_m','T_base_raw_R2_N_m','T_base_alloc_L1_N_m', ...
    'T_base_alloc_R1_N_m','T_base_alloc_L2_N_m','T_base_alloc_R2_N_m'});
setBlockPosition([ss '/VehicleStateBus'], [60 170 90 190]);
setBlockPosition([ss '/VehicleStateForTorqueRequestSelector'], [180 140 290 215]);
setBlockPosition([ss '/TorqueRequestBusCreator'], [720 120 790 305]);
setBlockPosition([ss '/TorqueRequestBus'], [880 205 910 225]);
end

function configureTorqueAllocation(root)
% 将车辆状态、横摆力矩和基础扭矩请求整理成QP/简单分配输入，并输出分配Bus。
ss = [root '/TorqueAllocation'];
ensureInport(ss, 'VehicleStateBus', 1);
ensureInport(ss, 'YawMomentBus', 2);
ensureInport(ss, 'TorqueRequestBus', 3);
ensureBusSelector(ss, 'VehicleStateForAllocationSelector', ...
    {'FyL1','FyL2','FyR1','FyR2','FzL1','FzL2','FzR1','FzR2', ...
     'throttle','AVy_L1','AVy_R1','AVy_L2','AVy_R2','mu_assumed','Fx_request_N','steer_deg'});
ensureBusSelector(ss, 'YawMomentForAllocationSelector', {'Mz_selected_N_m'});
ensureBusSelector(ss, 'TorqueRequestForAllocationSelector', ...
    {'T_base_alloc_4w_N_m','T_base_alloc_L1_N_m','T_base_alloc_R1_N_m', ...
     'T_base_alloc_L2_N_m','T_base_alloc_R2_N_m'});
connectByPorts(ss, [ss '/VehicleStateBus'], 1, [ss '/VehicleStateForAllocationSelector'], 1, '');
connectByPorts(ss, [ss '/YawMomentBus'], 1, [ss '/YawMomentForAllocationSelector'], 1, '');
connectByPorts(ss, [ss '/TorqueRequestBus'], 1, [ss '/TorqueRequestForAllocationSelector'], 1, '');

connectByPorts(ss, [ss '/VehicleStateForAllocationSelector'], 1, [ss '/AllocationInputMux'], 1, 'FyL1');
connectByPorts(ss, [ss '/VehicleStateForAllocationSelector'], 2, [ss '/AllocationInputMux'], 2, 'FyL2');
connectByPorts(ss, [ss '/VehicleStateForAllocationSelector'], 3, [ss '/AllocationInputMux'], 3, 'FyR1');
connectByPorts(ss, [ss '/VehicleStateForAllocationSelector'], 4, [ss '/AllocationInputMux'], 4, 'FyR2');
connectByPorts(ss, [ss '/VehicleStateForAllocationSelector'], 5, [ss '/AllocationInputMux'], 5, 'FzL1');
connectByPorts(ss, [ss '/VehicleStateForAllocationSelector'], 6, [ss '/AllocationInputMux'], 6, 'FzL2');
connectByPorts(ss, [ss '/VehicleStateForAllocationSelector'], 7, [ss '/AllocationInputMux'], 7, 'FzR1');
connectByPorts(ss, [ss '/VehicleStateForAllocationSelector'], 8, [ss '/AllocationInputMux'], 8, 'FzR2');
connectByPorts(ss, [ss '/VehicleStateForAllocationSelector'], 9, [ss '/AllocationInputMux'], 17, 'throttle');
connectByPorts(ss, [ss '/VehicleStateForAllocationSelector'], 10, [ss '/AllocationInputMux'], 18, 'AVy_L1');
connectByPorts(ss, [ss '/VehicleStateForAllocationSelector'], 11, [ss '/AllocationInputMux'], 19, 'AVy_R1');
connectByPorts(ss, [ss '/VehicleStateForAllocationSelector'], 12, [ss '/AllocationInputMux'], 20, 'AVy_L2');
connectByPorts(ss, [ss '/VehicleStateForAllocationSelector'], 13, [ss '/AllocationInputMux'], 21, 'AVy_R2');
connectByPorts(ss, [ss '/VehicleStateForAllocationSelector'], 14, [ss '/AllocationInputMux'], 12, 'mu_assumed');
connectByPorts(ss, [ss '/VehicleStateForAllocationSelector'], 15, [ss '/AllocationInputMux'], 10, 'Fx_request_N');
connectByPorts(ss, [ss '/VehicleStateForAllocationSelector'], 16, [ss '/AllocationInputMux'], 9, 'steer_deg');
connectByPorts(ss, [ss '/YawMomentForAllocationSelector'], 1, [ss '/AllocationInputMux'], 11, 'Mz_selected_N_m');
connectByPorts(ss, [ss '/TorqueRequestForAllocationSelector'], 1, ...
    [ss '/T_qp_candidate_base_plus_delta'], 2, 'T_base_alloc_4w_N_m');
connectByPorts(ss, [ss '/TorqueRequestForAllocationSelector'], 2, [ss '/AllocationInputMux'], 13, 'T_base_alloc_L1_N_m');
connectByPorts(ss, [ss '/TorqueRequestForAllocationSelector'], 3, [ss '/AllocationInputMux'], 14, 'T_base_alloc_R1_N_m');
connectByPorts(ss, [ss '/TorqueRequestForAllocationSelector'], 4, [ss '/AllocationInputMux'], 15, 'T_base_alloc_L2_N_m');
connectByPorts(ss, [ss '/TorqueRequestForAllocationSelector'], 5, [ss '/AllocationInputMux'], 16, 'T_base_alloc_R2_N_m');

ensureBusCreator(ss, 'AllocationBusCreator', 6, {
    'TorqueAllocationMode',   1, 'T_selected_4w_N_m'
    'QP_DiagnosticSelector',  1, 'QP_candidate_diag'
    'VehicleStateForAllocationSelector', 5, 'FzL1'
    'VehicleStateForAllocationSelector', 6, 'FzL2'
    'VehicleStateForAllocationSelector', 7, 'FzR1'
    'VehicleStateForAllocationSelector', 8, 'FzR2'
    });
ensureOutport(ss, 'AllocationBus', 1);
connectByPorts(ss, [ss '/AllocationBusCreator'], 1, [ss '/AllocationBus'], 1, 'AllocationBus');
deleteBlocksIfPresent(ss, {'throttle','AVy_L1','AVy_R1','AVy_L2','AVy_R2', ...
    'T_base_alloc_4w_N_m','T_base_alloc_L2_N_m','T_base_alloc_R2_N_m', ...
    'T_base_alloc_L1_N_m','T_base_alloc_R1_N_m','T_selected_4w_N_m', ...
    'QP_candidate_diag','FzL1','FzL2','FzR1','FzR2', ...
    'From22','From23','From24','From25','From26','From27','From28','From29', ...
    'From30','From32','From33','From36'});
setBlockPosition([ss '/VehicleStateBus'], [30 220 60 240]);
setBlockPosition([ss '/YawMomentBus'], [30 390 60 410]);
setBlockPosition([ss '/TorqueRequestBus'], [30 560 60 580]);
setBlockPosition([ss '/VehicleStateForAllocationSelector'], [150 170 270 360]);
setBlockPosition([ss '/YawMomentForAllocationSelector'], [150 385 270 430]);
setBlockPosition([ss '/TorqueRequestForAllocationSelector'], [150 515 285 650]);
setBlockPosition([ss '/AllocationBusCreator'], [820 320 890 460]);
setBlockPosition([ss '/AllocationBus'], [980 385 1010 405]);
end

function configureActuatorOutputLimiter(root)
% 对分配结果执行最终电机/功率限扭，并输出执行器Bus和CarSim扭矩向量。
ss = [root '/ActuatorOutputLimiter'];
ensureInport(ss, 'VehicleStateBus', 1);
ensureInport(ss, 'AllocationBus', 2);
ensureBusSelector(ss, 'VehicleStateForActuatorSelector', ...
    {'throttle','AVy_L1','AVy_R1','AVy_L2','AVy_R2'});
ensureBusSelector(ss, 'AllocationForActuatorSelector', {'T_selected_4w_N_m'});
connectByPorts(ss, [ss '/VehicleStateBus'], 1, [ss '/VehicleStateForActuatorSelector'], 1, '');
connectByPorts(ss, [ss '/AllocationBus'], 1, [ss '/AllocationForActuatorSelector'], 1, '');
connectByPorts(ss, [ss '/AllocationForActuatorSelector'], 1, [ss '/FinalMotorLimiter'], 1, 'T_selected_4w_N_m');
connectByPorts(ss, [ss '/VehicleStateForActuatorSelector'], 1, [ss '/FinalMotorLimiter'], 2, 'throttle');
connectByPorts(ss, [ss '/VehicleStateForActuatorSelector'], 2, [ss '/FinalMotorLimiter'], 3, 'AVy_L1');
connectByPorts(ss, [ss '/VehicleStateForActuatorSelector'], 3, [ss '/FinalMotorLimiter'], 4, 'AVy_R1');
connectByPorts(ss, [ss '/VehicleStateForActuatorSelector'], 4, [ss '/FinalMotorLimiter'], 5, 'AVy_L2');
connectByPorts(ss, [ss '/VehicleStateForActuatorSelector'], 5, [ss '/FinalMotorLimiter'], 6, 'AVy_R2');

ensureBusCreator(ss, 'ActuatorBusCreator', 2, {
    'WheelTorqueSaturation', 1, 'T_cmd_out_4w_N_m'
    'DrivelineGain',        1, 'T_to_CarSim_4w_N_m'
    });
ensureOutport(ss, 'ActuatorBus', 1);
ensureOutport(ss, 'T_to_CarSim_4w_N_m', 2);
connectByPorts(ss, [ss '/ActuatorBusCreator'], 1, [ss '/ActuatorBus'], 1, 'ActuatorBus');
connectByPorts(ss, [ss '/DrivelineGain'], 1, [ss '/T_to_CarSim_4w_N_m'], 1, 'T_to_CarSim_4w_N_m');
deleteBlocksIfPresent(ss, {'throttle','AVy_L1','AVy_R1','AVy_L2','AVy_R2', ...
    'T_selected','T_cmd_out_4w_N_m'});
setBlockPosition([ss '/VehicleStateBus'], [30 170 60 190]);
setBlockPosition([ss '/AllocationBus'], [30 330 60 350]);
setBlockPosition([ss '/VehicleStateForActuatorSelector'], [140 130 260 245]);
setBlockPosition([ss '/AllocationForActuatorSelector'], [140 315 260 360]);
setBlockPosition([ss '/ActuatorBusCreator'], [760 240 830 295]);
setBlockPosition([ss '/ActuatorBus'], [930 245 960 265]);
setBlockPosition([ss '/T_to_CarSim_4w_N_m'], [930 345 960 365]);
end

function configureDiagnosticsLogging(root)
% 诊断子系统只旁路读取Bus，不参与主控制闭环。
ss = [root '/DiagnosticsLogging'];
ensureInport(ss, 'VehicleStateBus', 1);
ensureInport(ss, 'TargetBus', 2);
ensureInport(ss, 'YawMomentBus', 3);
ensureInport(ss, 'TorqueRequestBus', 4);
ensureInport(ss, 'AllocationBus', 5);
ensureInport(ss, 'ActuatorBus', 6);
ensureBusSelector(ss, 'VehicleStateForDiagnosticsSelector', {'beta_deg','omega_deg_s'});
ensureBusSelector(ss, 'TargetForDiagnosticsSelector', {'beta_ref_deg','omega_ref_deg_s'});
ensureBusSelector(ss, 'YawMomentForDiagnosticsSelector', {'Mz_selected_N_m'});
ensureBusSelector(ss, 'TorqueRequestForDiagnosticsSelector', ...
    {'T_base_alloc_4w_N_m','T_base_alloc_L1_N_m','T_base_alloc_R1_N_m', ...
     'T_base_alloc_L2_N_m','T_base_alloc_R2_N_m','T_base_raw_L1_N_m', ...
     'T_base_raw_R1_N_m','T_base_raw_L2_N_m','T_base_raw_R2_N_m'});
ensureBusSelector(ss, 'AllocationForDiagnosticsSelector', ...
    {'QP_candidate_diag','T_selected_4w_N_m','FzL1','FzL2','FzR1','FzR2'});
ensureBusSelector(ss, 'ActuatorForDiagnosticsSelector', {'T_cmd_out_4w_N_m'});

connectByPorts(ss, [ss '/VehicleStateBus'], 1, [ss '/VehicleStateForDiagnosticsSelector'], 1, '');
connectByPorts(ss, [ss '/TargetBus'], 1, [ss '/TargetForDiagnosticsSelector'], 1, '');
connectByPorts(ss, [ss '/YawMomentBus'], 1, [ss '/YawMomentForDiagnosticsSelector'], 1, '');
connectByPorts(ss, [ss '/TorqueRequestBus'], 1, [ss '/TorqueRequestForDiagnosticsSelector'], 1, '');
connectByPorts(ss, [ss '/AllocationBus'], 1, [ss '/AllocationForDiagnosticsSelector'], 1, '');
connectByPorts(ss, [ss '/ActuatorBus'], 1, [ss '/ActuatorForDiagnosticsSelector'], 1, '');

connectByPorts(ss, [ss '/TargetForDiagnosticsSelector'], 1, [ss '/Scope_beta_ref_actual'], 1, 'beta_ref_deg');
connectByPorts(ss, [ss '/VehicleStateForDiagnosticsSelector'], 1, [ss '/Scope_beta_ref_actual'], 2, 'beta_deg');
connectByPorts(ss, [ss '/TargetForDiagnosticsSelector'], 2, [ss '/Scope_wr_ref_actual'], 1, 'omega_ref_deg_s');
connectByPorts(ss, [ss '/VehicleStateForDiagnosticsSelector'], 2, [ss '/Scope_wr_ref_actual'], 2, 'omega_deg_s');
connectToTerminator(ss, 'YawMomentForDiagnosticsSelector', 1, 'Term_Mz_selected_diag', 'Mz_selected_N_m');

connectByPorts(ss, [ss '/ActuatorForDiagnosticsSelector'], 1, [ss '/T_cmd_scope_demux'], 1, 'T_cmd_out_4w_N_m');
connectByPorts(ss, [ss '/ActuatorForDiagnosticsSelector'], 1, [ss '/SafetyDelta_T_final_minus_selected'], 1, 'T_cmd_out_4w_N_m');
connectByPorts(ss, [ss '/ActuatorForDiagnosticsSelector'], 1, [ss '/Scope_T_final_mismatch'], 2, 'T_cmd_out_4w_N_m');
connectByPorts(ss, [ss '/AllocationForDiagnosticsSelector'], 1, [ss '/QP_DiagnosticDemux'], 1, 'QP_candidate_diag');
connectByPorts(ss, [ss '/AllocationForDiagnosticsSelector'], 2, [ss '/T_alloc_scope_demux'], 1, 'T_selected_4w_N_m');
connectByPorts(ss, [ss '/AllocationForDiagnosticsSelector'], 2, [ss '/SafetyDelta_T_final_minus_selected'], 2, 'T_selected_4w_N_m');
connectByPorts(ss, [ss '/AllocationForDiagnosticsSelector'], 2, [ss '/Scope_T_final_mismatch'], 1, 'T_selected_4w_N_m');
connectByPorts(ss, [ss '/AllocationForDiagnosticsSelector'], 2, [ss '/DYC_delta_T_minus_base'], 1, 'T_selected_4w_N_m');
connectByPorts(ss, [ss '/AllocationForDiagnosticsSelector'], 3, [ss '/Scope_Fz_distribution'], 1, 'FzL1');
connectByPorts(ss, [ss '/AllocationForDiagnosticsSelector'], 4, [ss '/Scope_Fz_distribution'], 2, 'FzL2');
connectByPorts(ss, [ss '/AllocationForDiagnosticsSelector'], 5, [ss '/Scope_Fz_distribution'], 3, 'FzR1');
connectByPorts(ss, [ss '/AllocationForDiagnosticsSelector'], 6, [ss '/Scope_Fz_distribution'], 4, 'FzR2');
connectByPorts(ss, [ss '/TorqueRequestForDiagnosticsSelector'], 1, [ss '/DYC_delta_T_minus_base'], 2, 'T_base_alloc_4w_N_m');
connectByPorts(ss, [ss '/TorqueRequestForDiagnosticsSelector'], 2, [ss '/Scope_T_base_alloc'], 1, 'T_base_alloc_L1_N_m');
connectByPorts(ss, [ss '/TorqueRequestForDiagnosticsSelector'], 3, [ss '/Scope_T_base_alloc'], 2, 'T_base_alloc_R1_N_m');
connectByPorts(ss, [ss '/TorqueRequestForDiagnosticsSelector'], 4, [ss '/Scope_T_base_alloc'], 3, 'T_base_alloc_L2_N_m');
connectByPorts(ss, [ss '/TorqueRequestForDiagnosticsSelector'], 5, [ss '/Scope_T_base_alloc'], 4, 'T_base_alloc_R2_N_m');
connectByPorts(ss, [ss '/TorqueRequestForDiagnosticsSelector'], 6, [ss '/Scope_T_base_driver_raw'], 1, 'T_base_raw_L1_N_m');
connectByPorts(ss, [ss '/TorqueRequestForDiagnosticsSelector'], 7, [ss '/Scope_T_base_driver_raw'], 2, 'T_base_raw_R1_N_m');
connectByPorts(ss, [ss '/TorqueRequestForDiagnosticsSelector'], 8, [ss '/Scope_T_base_driver_raw'], 3, 'T_base_raw_L2_N_m');
connectByPorts(ss, [ss '/TorqueRequestForDiagnosticsSelector'], 9, [ss '/Scope_T_base_driver_raw'], 4, 'T_base_raw_R2_N_m');

deleteBlocksIfPresent(ss, {'T_cmd_out_4w_N_m','QP_candidate_diag', ...
    'T_selected_4w_N_m','FzL1','FzL2','FzR1','FzR2', ...
    'T_base_alloc_4w_N_m','T_base_alloc_L2_N_m','T_base_alloc_R2_N_m', ...
    'T_base_raw_L1_N_m','T_base_raw_R1_N_m','T_base_raw_L2_N_m', ...
    'T_base_raw_R2_N_m','T_base_alloc_L1_N_m','T_base_alloc_R1_N_m', ...
    'From3','From4','From5','From6', ...
    'From_FxL1_unused','From_FxL2_unused','From_FxR1_unused','From_FxR2_unused', ...
    'Term_FxL1_unused','Term_FxL2_unused','Term_FxR1_unused','Term_FxR2_unused'});
setBlockPosition([ss '/VehicleStateBus'], [40 80 70 100]);
setBlockPosition([ss '/TargetBus'], [40 170 70 190]);
setBlockPosition([ss '/YawMomentBus'], [40 260 70 280]);
setBlockPosition([ss '/TorqueRequestBus'], [40 350 70 370]);
setBlockPosition([ss '/AllocationBus'], [40 500 70 520]);
setBlockPosition([ss '/ActuatorBus'], [40 660 70 680]);
end

function rebuildTopLevelBusWiring(root)
% 删除顶层旧标量线，按Bus主干重新连接各组件。
deleteAllLines(root);
connectByPorts(root, [root '/sensor'], 1, [root '/VehicleStateInput'], 1, 'sensor_bus');
connectSubsystemPort(root, 'VehicleStateInput', 'VehicleStateBus', 'ControlTargetGeneration', 'VehicleStateBus', 'VehicleStateBus');
connectSubsystemPort(root, 'VehicleStateInput', 'VehicleStateBus', 'YawMomentControl', 'VehicleStateBus', 'VehicleStateBus');
connectSubsystemPort(root, 'VehicleStateInput', 'VehicleStateBus', 'TorqueRequest', 'VehicleStateBus', 'VehicleStateBus');
connectSubsystemPort(root, 'VehicleStateInput', 'VehicleStateBus', 'TorqueAllocation', 'VehicleStateBus', 'VehicleStateBus');
connectSubsystemPort(root, 'VehicleStateInput', 'VehicleStateBus', 'ActuatorOutputLimiter', 'VehicleStateBus', 'VehicleStateBus');
connectSubsystemPort(root, 'VehicleStateInput', 'VehicleStateBus', 'DiagnosticsLogging', 'VehicleStateBus', 'VehicleStateBus');
connectSubsystemPort(root, 'ControlTargetGeneration', 'TargetBus', 'YawMomentControl', 'TargetBus', 'TargetBus');
connectSubsystemPort(root, 'ControlTargetGeneration', 'TargetBus', 'DiagnosticsLogging', 'TargetBus', 'TargetBus');
connectSubsystemPort(root, 'YawMomentControl', 'YawMomentBus', 'TorqueAllocation', 'YawMomentBus', 'YawMomentBus');
connectSubsystemPort(root, 'YawMomentControl', 'YawMomentBus', 'DiagnosticsLogging', 'YawMomentBus', 'YawMomentBus');
connectSubsystemPort(root, 'TorqueRequest', 'TorqueRequestBus', 'TorqueAllocation', 'TorqueRequestBus', 'TorqueRequestBus');
connectSubsystemPort(root, 'TorqueRequest', 'TorqueRequestBus', 'DiagnosticsLogging', 'TorqueRequestBus', 'TorqueRequestBus');
connectSubsystemPort(root, 'TorqueAllocation', 'AllocationBus', 'ActuatorOutputLimiter', 'AllocationBus', 'AllocationBus');
connectSubsystemPort(root, 'TorqueAllocation', 'AllocationBus', 'DiagnosticsLogging', 'AllocationBus', 'AllocationBus');
connectSubsystemPort(root, 'ActuatorOutputLimiter', 'ActuatorBus', 'DiagnosticsLogging', 'ActuatorBus', 'ActuatorBus');
connectSubsystemToBlock(root, 'ActuatorOutputLimiter', 'T_to_CarSim_4w_N_m', 'T', 1, 'T_to_CarSim_4w_N_m');
end

function positionTopLevelComponents(root)
setBlockPosition([root '/sensor'], [80 360 110 380]);
setBlockPosition([root '/VehicleStateInput'], [210 300 500 440]);
setBlockPosition([root '/ControlTargetGeneration'], [690 155 980 255]);
setBlockPosition([root '/YawMomentControl'], [1160 140 1450 290]);
setBlockPosition([root '/TorqueRequest'], [690 520 980 640]);
setBlockPosition([root '/TorqueAllocation'], [1160 420 1450 650]);
setBlockPosition([root '/ActuatorOutputLimiter'], [1640 420 1930 610]);
setBlockPosition([root '/DiagnosticsLogging'], [1640 100 1930 320]);
setBlockPosition([root '/T'], [2080 505 2110 525]);
end

function addBusArchitectureAnnotation(root)
existing = find_system(root, 'FindAll', 'on', 'Type', 'annotation');
for idx = 1:numel(existing)
    try
        text = get_param(existing(idx), 'Text');
        if contains(string(text), 'Bus化主数据流') || contains(string(text), '组件化主数据流')
            delete(existing(idx));
        end
    catch
    end
end

note = Simulink.Annotation(root, ...
    ['Bus化主数据流：VehicleStateBus -> TargetBus -> YawMomentBus -> ', ...
     'TorqueRequestBus / AllocationBus -> ActuatorBus；DiagnosticsLogging 只接旁路Bus。']);
note.Position = [250 35 2040 90];
end

function ensureBusCreator(parent, name, inputCount, entries)
% 创建或复用Bus Creator，并按entries表把源块输出接入指定端口。
path = ensureBlock(parent, name, 'simulink/Signal Routing/Bus Creator');
set_param(path, 'Inputs', num2str(inputCount), 'DisplayOption', 'signals');
for idx = 1:size(entries, 1)
    connectByPorts(parent, [parent '/' entries{idx, 1}], entries{idx, 2}, ...
        path, idx, entries{idx, 3});
end
end

function ensureBusSelector(parent, name, signals)
path = ensureBlock(parent, name, 'simulink/Signal Routing/Bus Selector');
set_param(path, 'OutputSignals', strjoin(signals, ','));
end

function ensureInport(parent, name, portNumber)
path = ensureBlock(parent, name, 'simulink/Sources/In1');
set_param(path, 'Port', num2str(portNumber));
end

function ensureOutport(parent, name, portNumber)
path = ensureBlock(parent, name, 'simulink/Sinks/Out1');
set_param(path, 'Port', num2str(portNumber));
end

function path = ensureBlock(parent, name, libraryPath)
path = [parent '/' name];
if getSimulinkBlockHandle(path) == -1
    add_block(libraryPath, path, 'MakeNameUnique', 'off');
end
end

function deleteBlocksIfPresent(parent, names)
for idx = 1:numel(names)
    path = [parent '/' names{idx}];
    if getSimulinkBlockHandle(path) ~= -1
        delete_block(path);
    end
end
end

function connectToTerminator(parent, sourceBlock, sourcePort, termName, signalName)
termPath = ensureBlock(parent, termName, 'simulink/Sinks/Terminator');
connectByPorts(parent, [parent '/' sourceBlock], sourcePort, termPath, 1, signalName);
end

function connectSubsystemPort(parent, srcSubsystem, srcPortName, dstSubsystem, dstPortName, lineName)
srcIndex = subsystemPortIndex([parent '/' srcSubsystem], 'Outport', srcPortName);
dstIndex = subsystemPortIndex([parent '/' dstSubsystem], 'Inport', dstPortName);
connectByPorts(parent, [parent '/' srcSubsystem], srcIndex, ...
    [parent '/' dstSubsystem], dstIndex, lineName);
end

function connectSubsystemToBlock(parent, srcSubsystem, srcPortName, dstBlock, dstPort, lineName)
srcIndex = subsystemPortIndex([parent '/' srcSubsystem], 'Outport', srcPortName);
connectByPorts(parent, [parent '/' srcSubsystem], srcIndex, ...
    [parent '/' dstBlock], dstPort, lineName);
end

function idx = subsystemPortIndex(subsystemPath, blockType, portName)
blocks = find_system(subsystemPath, 'SearchDepth', 1, ...
    'BlockType', blockType, 'Name', portName);
if isempty(blocks)
    error('refactor_dyc_model_bus_topology:missingPort', ...
        'Missing %s %s in %s.', blockType, portName, subsystemPath);
end
idx = str2double(get_param(blocks{1}, 'Port'));
end

function connectByPorts(parent, srcBlock, srcPort, dstBlock, dstPort, lineName)
% 基于端口句柄连线，先清理目标输入旧线，避免重复连线错误。
if getSimulinkBlockHandle(srcBlock) == -1
    error('refactor_dyc_model_bus_topology:missingSource', 'Missing source %s.', srcBlock);
end
if getSimulinkBlockHandle(dstBlock) == -1
    error('refactor_dyc_model_bus_topology:missingDestination', 'Missing destination %s.', dstBlock);
end

srcHandles = get_param(srcBlock, 'PortHandles');
dstHandles = get_param(dstBlock, 'PortHandles');
if numel(srcHandles.Outport) < srcPort
    error('refactor_dyc_model_bus_topology:sourcePort', 'Missing output %d on %s.', srcPort, srcBlock);
end
if numel(dstHandles.Inport) < dstPort
    error('refactor_dyc_model_bus_topology:destinationPort', 'Missing input %d on %s.', dstPort, dstBlock);
end

deleteInputLine(dstBlock, dstPort);
lineHandle = add_line(parent, srcHandles.Outport(srcPort), ...
    dstHandles.Inport(dstPort), 'autorouting', 'on');
if strlength(string(lineName)) > 0
    sourceType = string(get_param(srcBlock, 'BlockType'));
    if sourceType ~= "BusSelector"
        set_param(lineHandle, 'Name', char(lineName));
    end
end
end

function deleteInputLine(blockPath, portNumber)
lineHandles = get_param(blockPath, 'LineHandles');
if numel(lineHandles.Inport) >= portNumber && lineHandles.Inport(portNumber) ~= -1
    delete_line(lineHandles.Inport(portNumber));
end
end

function deleteAllLines(parent)
lines = find_system(parent, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'line');
for idx = 1:numel(lines)
    try
        delete_line(lines(idx));
    catch
    end
end
end

function deleteDanglingLines(root)
lines = find_system(root, 'FindAll', 'on', 'Type', 'line');
for idx = 1:numel(lines)
    try
        src = get_param(lines(idx), 'SrcBlockHandle');
        dst = get_param(lines(idx), 'DstBlockHandle');
        if src == -1 || isempty(dst) || all(dst == -1)
            delete_line(lines(idx));
        end
    catch
    end
end
end

function setBlockPosition(path, position)
if getSimulinkBlockHandle(path) ~= -1
    set_param(path, 'Position', position);
end
end

function n = countTopLevelLines(root)
lines = find_system(root, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'line');
n = numel(lines);
end

function n = countLogicalTopLevelConnections(root)
lines = find_system(root, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'line');
edges = strings(0, 1);
for idx = 1:numel(lines)
    src = get_param(lines(idx), 'SrcBlockHandle');
    dst = get_param(lines(idx), 'DstBlockHandle');
    if src == -1 || isempty(dst)
        continue;
    end
    srcName = string(get_param(src, 'Name'));
    for dstIdx = 1:numel(dst)
        dstName = string(get_param(dst(dstIdx), 'Name'));
        edges(end + 1, 1) = srcName + "->" + dstName; %#ok<AGROW>
    end
end
n = numel(unique(edges));
end
