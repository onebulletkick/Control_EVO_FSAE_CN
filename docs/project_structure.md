# 项目详细结构说明

本文按 `git ls-files` 的当前跟踪文件整理，用于说明仓库中每个目录、子项目和主要文件的职责。模型文件、脚本、测试和 README 逐项说明；批量图片、图表和模型输出按同质文件组归类说明。

## 根目录

根目录保存仓库级说明、许可证和通用配置。

| 路径 | 用途 |
| --- | --- |
| `.gitattributes` | Git 属性配置，约束文本和二进制文件处理方式。 |
| `.gitignore` | 忽略 MATLAB/Simulink 缓存、CarSim 本地运行文件、临时文件和本地实验产物。 |
| `LICENSE` | 项目开源许可证。 |
| `README.md` | GitHub 仓库主页，提供项目概览、快速入口、验证边界和进一步阅读链接。 |
| `DYC_设计流程说明.md` | DYC 设计流程、控制路线、联合仿真到 DIL 的阶段说明。 |

## CarSim 示例数据库

`carsim_models/` 存放可导入 CarSim 的示例数据库压缩包，不包含 CarSim 程序本体或许可证。

| 路径 | 用途 |
| --- | --- |
| `carsim_models/E_example.zip` | 主 DYC 联合仿真示例数据库包。 |
| `carsim_models/DIL_example.zip` | DIL 示例数据库包。 |

## control_EVO 主工程

`control_EVO/` 是 MATLAB/Simulink 控制工程主体，包含主模型、DIL 兼容模型、控制算法函数、优化工具、轮胎建模和 Pacejka 工程包。

### Simulink 与 CarSim 入口

| 路径 | 用途 |
| --- | --- |
| `control_EVO/DYC_1_9_test.slx` | 当前主 Simulink + CarSim 联合仿真模型。 |
| `control_EVO/DIL_26_5_1.slx` | 当前 DIL 示例 Simulink + CarSim 联合仿真模型。 |
| `control_EVO/DIL_26_5_1_2025b.slx` | 面向 MATLAB/Simulink R2025b 的 DIL 兼容模型。 |
| `control_EVO/DIL_26_5_1_2023b.slx` | 面向 MATLAB/Simulink R2023b 的 DIL 兼容模型。 |
| `control_EVO/DIL_26_5_1_2020a.slx` | 面向 MATLAB/Simulink R2020a 的 DIL 兼容模型。 |
| `control_EVO/simfile.template.sim` | CarSim `.sim` 文件模板；实际 `simfile.sim` 由本机 CarSim Run 生成。 |

### 控制算法函数

| 路径 | 用途 |
| --- | --- |
| `control_EVO/DYC_vehicle_params.m` | 车辆参数统一入口，避免控制模块重复硬编码参数。 |
| `control_EVO/DYC_base_motor_torque.m` | 将踏板或基础请求转换为轮端基础扭矩。 |
| `control_EVO/DYC_torque_request_manager.m` | 扭矩请求管理、制动平滑和驱动使能逻辑。 |
| `control_EVO/DYC_simple_load_transfer_distribution.m` | 基于载荷转移比例的四轮扭矩分配基线。 |
| `control_EVO/QP_TorqueDistribution.m` | QP 四轮扭矩分配 Level-2 S-Function 包装器。 |
| `control_EVO/qp_torque_distribution_core.m` | QP 扭矩分配核心算法，供 S-Function 和测试复用。 |
| `control_EVO/DYC_tire_lookup_config.m` | 控制侧轮胎查表模式配置，默认使用 Pacejka 查表。 |
| `control_EVO/DYC_motor_wheel_torque_limit.m` | 电机转速相关的轮端扭矩限制。 |
| `control_EVO/DYC_apply_motor_limits.m` | 电机限扭、总功率限制和最终输出限幅。 |

### PID 圈速优化工具

`control_EVO/optimization/` 是外部 MATLAB 工具目录，用于在固定 CarSim Run 上搜索 DYC PID 参数，不为候选参数保存主模型。

| 路径 | 用途 |
| --- | --- |
| `control_EVO/optimization/README.md` | 优化器使用说明、入口、输出和边界声明。 |
| `control_EVO/optimization/dyc_pid_optimization_config.m` | 优化器默认配置和可覆盖参数入口。 |
| `control_EVO/optimization/optimize_dyc_pid_laptime.m` | 默认 station-stop 圈速优化入口。 |
| `control_EVO/optimization/optimize_dyc_pid_autocross_laptime.m` | Autocross stop-time 圈速优化入口。 |
| `control_EVO/optimization/evaluate_dyc_pid_candidate.m` | 对单组 PID 候选参数运行仿真并提取评价结果。 |
| `control_EVO/optimization/extract_dyc_laptime_metric.m` | 从 CarSim 停止原因、停止时间和站点信息中提取评价指标。 |
| `control_EVO/optimization/parse_dyc_simfile.m` | 读取并解析 CarSim `.sim` 文件中的运行信息。 |
| `control_EVO/optimization/run_dyc_pid_preflight.m` | 优化前检查模型、PID 模块、配置和依赖是否可用。 |
| `control_EVO/optimization/add_dyc_carsim_solver_path.m` | 补充 CarSim solver/S-Function MATLAB 路径。 |
| `control_EVO/optimization/append_dyc_optimization_checkpoint.m` | 将每次候选评价追加写入运行日志。 |
| `control_EVO/optimization/write_dyc_pid_optimization_report.m` | 写出优化结果、summary、MAT 文件和最佳 PID 应用脚本。 |

`control_EVO/optimization/tests/` 覆盖优化器配置、候选评价、指标提取、`.sim` 解析、preflight、输出文件和 smoke 流程：

- `dycOptimizationOutputTest.m`
- `dycPidOptimizationConfigTest.m`
- `evaluateDycPidCandidateTest.m`
- `extractDycLaptimeMetricTest.m`
- `optimizeDycPidLaptimeSmokeTest.m`
- `parseDycSimfileTest.m`
- `runDycPidPreflightTest.m`

### 轮胎建模与控制查表

`control_EVO/tire_modeling/` 存放 Hoosier 轮胎建模、Pacejka 控制查表、输出结果和测试。

| 路径 | 用途 |
| --- | --- |
| `control_EVO/tire_modeling/README.md` | 轮胎建模、控制查表和测试说明。 |
| `control_EVO/tire_modeling/Hoosier43075_Tire_Model_Analysis.md` | Hoosier 43075 轮胎模型分析说明。 |
| `control_EVO/tire_modeling/build_hoosier43075_model.m` | 构建 Hoosier 43075 控制侧轮胎模型。 |
| `control_EVO/tire_modeling/build_pacejka_control_lookup.m` | 构建 Pacejka 控制查表模型。 |
| `control_EVO/tire_modeling/lookup_hoosier43075_limits.m` | 查询 Hoosier 43075 轮胎力限制。 |
| `control_EVO/tire_modeling/lookup_tire_control_limits.m` | 统一轮胎控制查表入口。 |
| `control_EVO/tire_modeling/outputs/hoosier43075_control_model.mat` | Hoosier 43075 控制模型输出。 |
| `control_EVO/tire_modeling/outputs/hoosier43075_fit_report.md` | Hoosier 43075 拟合报告。 |
| `control_EVO/tire_modeling/outputs/pacejka_control_lookup_model.mat` | Pacejka 控制查表模型输出。 |
| `control_EVO/tire_modeling/outputs/pacejka_control_lookup_report.md` | Pacejka 控制查表报告。 |

`control_EVO/tire_modeling/outputs/figures/` 保存轮胎查表和拟合质量图，覆盖 `control_lookup_tables.png`、`fit_quality_summary.png`、滑移率代理图、侧偏角力图、轮辋对比图和 holdout 验证图。

`control_EVO/tire_modeling/tests/` 覆盖基础电机扭矩、Pacejka 查表构建、Hoosier 查表、QP 分配和载荷比例分配：

- `baseMotorTorqueTest.m`
- `buildPacejkaControlLookupTest.m`
- `lookupHoosier43075LimitsTest.m`
- `qpTorqueDistributionHoosierLookupTest.m`
- `simpleLoadTransferDistributionTest.m`

### Round9 Pacejka 工程包

`control_EVO/round9_pacejka_engineering_package/` 存放 TTC Round9 Pacejka 工程模型、示例、文档、图表和测试。

| 路径 | 用途 |
| --- | --- |
| `control_EVO/round9_pacejka_engineering_package/README.md` | 工程包入口说明。 |
| `control_EVO/round9_pacejka_engineering_package/docs/pacejka_round9_usage.md` | Round9 Pacejka 使用说明。 |
| `control_EVO/round9_pacejka_engineering_package/examples/round9_pacejka_quickstart.m` | 快速使用示例。 |
| `control_EVO/round9_pacejka_engineering_package/matlab/fitRound9PacejkaModels.m` | 拟合 Round9 Pacejka 模型。 |
| `control_EVO/round9_pacejka_engineering_package/matlab/predictTireForces.m` | 使用工程模型预测轮胎力。 |
| `control_EVO/round9_pacejka_engineering_package/matlab/private/round9PacejkaEval.m` | Pacejka 公式内部求值函数。 |
| `control_EVO/round9_pacejka_engineering_package/model/pacejka_round9.mat` | Round9 Pacejka 模型数据。 |
| `control_EVO/round9_pacejka_engineering_package/model/engineering_usability_summary.csv` | 工程可用性汇总表。 |
| `control_EVO/round9_pacejka_engineering_package/tests/testRound9PacejkaPackage.m` | 工程包回归测试。 |

`control_EVO/round9_pacejka_engineering_package/plots/` 保存不同轮胎、轮辋和灵敏度组合的 PNG 图，覆盖 Goodyear、Hoosier 和 MRF 轮胎组合：

- Goodyear 18.0X6.5-10 Eagle Racing Special：rim6in、rim7in 及对应 sensitivities。
- Goodyear 20.0X7.0-13 Eagle Sports Car Special：rim7in、rim8in 及对应 sensitivities。
- Hoosier 16.0X6.0-10 R20 C2000：rim6in、rim7in 及对应 sensitivities。
- Hoosier 16.0X7.5-10 R20 C2000：rim7in、rim8in 及对应 sensitivities。
- Hoosier 18.0X6.0-10 R20：rim6in、rim7in 及对应 sensitivities。
- Hoosier 20.5X7.0-13 R20：rim7in、rim8in 及对应 sensitivities。
- MRF 18.0X6.0-10 ZTD1：rim6in、rim7in 及对应 sensitivities。

## docs 公开文档与图片资产

`docs/` 存放公开技术文档和 README 引用的图片资产。

| 路径 | 用途 |
| --- | --- |
| `docs/control_evo_technical_design.md` | 完整技术设计文档，包含公式、架构、控制链路、验证边界和阅读顺序。 |
| `docs/project_structure.md` | 当前项目详细结构说明。 |
| `docs/assets/community/qq-group-qrcode.jpg` | 社群二维码图片。 |
| `docs/assets/quickstart/carsim-run-control-select-4wd-stability.png` | CarSim Run Control 选择示例截图。 |
| `docs/assets/quickstart/carsim-simulink-model-path.png` | CarSim Models Simulink 路径配置截图。 |
| `docs/assets/quickstart/evo-control-system-block-quickstart.png` | EVO_Control_System 模型块快速入口截图。 |
| `docs/assets/quickstart/evo-control-system-main-flow.html` | 主控制链路图源文件。 |
| `docs/assets/quickstart/evo-control-system-main-flow.png` | 主控制链路图图片。 |
| `docs/assets/quickstart/evo-control-system-diagnostics.html` | 诊断与阅读顺序图源文件。 |
| `docs/assets/quickstart/evo-control-system-diagnostics.png` | 诊断与阅读顺序图图片。 |

## tools 工程辅助脚本

`tools/` 存放模型整理、图生成和局部验证脚本。运行前应先理解脚本目标，并在必要时使用分支或备份模型。

| 路径 | 用途 |
| --- | --- |
| `tools/apply_brake_smooth_switch.m` | 应用制动平滑开关相关模型调整。 |
| `tools/refactor_dyc_model_structure.m` | 主模型结构整理脚本。 |
| `tools/refactor_dyc_model_bus_topology.m` | Bus 拓扑整理脚本。 |
| `tools/render_evo_quickstart_diagram.js` | 生成 README 快速入口图。 |
| `tools/test_brake_smooth_switch.m` | 制动平滑开关局部验证脚本。 |
| `tools/test_tire_load_formula.m` | 轮胎载荷公式局部验证脚本。 |
| `tools/update_tire_load_function.m` | 更新轮胎载荷相关函数的脚本。 |

## 覆盖说明

- 本文覆盖当前 `git ls-files` 中的根目录文件、`carsim_models/`、`control_EVO/`、`docs/` 和 `tools/`。
- 对 `.slx`、`.m`、`.md`、`.zip`、`.mat`、`.csv`、`.html`、`.jpg` 等关键文件逐项说明。
- 对数量较多且语义相同的 PNG 图表按目录和轮胎组合分组说明，避免重复描述每张图的相同用途。
