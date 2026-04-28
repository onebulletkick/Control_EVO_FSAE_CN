# 轮胎控制查表

这个目录存放 DYC/TC 控制侧使用的轮胎边界查表。它不是把完整轮胎模型直接塞进实时控制器，而是把 TTC Round 9 数据和 Pacejka 工程模型整理成稳定、可测试、适合扭矩分配限幅的低阶表。

## 默认配置

- 横向轮胎：Hoosier 43075 16x7.5-10 R20, 7 inch rim, 12 psi。
- 横向数据：`RunData_Cornering_Matlab_SI_Round9` 的 runs `2,4,5,6`。
- 纵向代理：Hoosier 43100 18.0x6.0-10 R20, 7 inch rim, 12 psi。
- 纵向数据：`RunData_DriveBrake_Matlab_SI_Round9` 的 runs `71,72,73`。
- TC 保守系数：`muScaleTc = 0.70`。
- 控制封顶：`controlMuCeiling = 1.00`，避免查表边界比固定 `Mu=1` 方案更激进。

## 生成 Hoosier 查表

```matlab
repo = '<本地 Control_EVO 仓库路径>';
model = build_hoosier43075_model(repo);
```

查询控制边界：

```matlab
modelFile = fullfile(repo, 'control_EVO', 'tire_modeling', 'outputs', 'hoosier43075_control_model.mat');
out = lookup_hoosier43075_limits([400; 800; 1200], [0; 100; 200], [], [], modelFile);
```

## 输出文件

- `outputs/hoosier43075_control_model.mat`：控制用查表模型。
- `outputs/pacejka_control_lookup_model.mat`：由 Round9 Pacejka 模型扫描生成的控制用查表模型。
- `outputs/hoosier43075_fit_report.md`：数据选择、拟合表和验证摘要。
- `outputs/pacejka_control_lookup_report.md`：Pacejka 生成查表的来源模型和使用说明。
- `Hoosier43075_Tire_Model_Analysis.md`：面向汇报和交接的分析报告。
- `outputs/figures/`：横向、纵向代理和查表图。

## 两种控制查表选项

主控制器读取 `DYC_tire_lookup_config.m`：

```matlab
cfg.mode = 'pacejka'; % 可改为 'hoosier'
```

代码默认使用 Pacejka 生成表。切换回直接 Hoosier 控制查表时，把 `DYC_tire_lookup_config.m` 里的 `cfg.mode` 改为 `'hoosier'`。

如需重新生成 Pacejka 控制查表：

```matlab
model = build_pacejka_control_lookup(repo);
```

`cfg.modelFile` 可指向任意兼容查表 `.mat`。`cfg.allowEnvironmentOverride = false` 时，控制器只读取代码配置，不读取环境变量覆盖。

这些模型用于控制限幅和扭矩分配边界，不替代离线 Pacejka/PAC2002 精细模型，也不等同于 CarSim 求解器内部轮胎模型。
