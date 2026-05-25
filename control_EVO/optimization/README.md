# DYC 优化与自动报告工具

本目录存放外部 MATLAB 工具，用于在固定 CarSim Run 上做 DYC PID 圈速优化，以及生成 `dyc_off` / `dyc_on` 对比报告。工具只通过 MATLAB 脚本临时设置仿真参数，不为了报告或优化自动保存 `DYC_1_9_test.slx`。

## 1. 最短使用路径

### Autocross 一键报告

先在 CarSim 选择 Autocross Run，确认该 Run 已配置两圈结束事件或 external stop，然后执行 `Send to Simulink`。MATLAB 中运行：

```matlab
repo = 'D:\Control_EVO';
addpath(fullfile(repo, 'control_EVO', 'optimization'));
result = run_dyc_autocross_report();
```

脚本会自动运行：

```text
dyc_off: Kp/Ki/Kd = 0/0/0
dyc_on : Kp/Ki/Kd = 6000/200/0
```

完成后命令行会打印 `report.html`、`comparison_metrics.csv`、`autocross_where_faster.csv` 和 `plots_presentation` 的路径。

### 八字绕环一键报告

先在 CarSim 选择八字绕环 Run，并执行 `Send to Simulink`。MATLAB 中运行：

```matlab
repo = 'D:\Control_EVO';
addpath(fullfile(repo, 'control_EVO', 'optimization'));
result = run_dyc_figure8_report();
```

八字绕环报告会额外输出左转、右转、换向过渡区的分段统计。

### 已有结果后处理

如果已经有一次报告结果目录，只想重新生成“快在哪里”分析，不重新仿真：

```matlab
resultsDir = 'D:\Control_EVO\control_EVO\optimization\results\<某次结果目录>';
analysis = analyze_dyc_autocross_report_results(resultsDir);
analysis = analyze_dyc_figure8_report_results(resultsDir);
```

Autocross 后处理需要已有 `comparison_metrics.csv`、`run_results.csv` 和 `signal_data/aligned_dyc_comparison.csv`。Figure-8 后处理需要已有 `comparison_metrics.csv`、`figure8_segment_metrics.csv` 和 `figure8_segment_delta.csv`。

## 2. 自动报告输出

报告结果默认写入：

```text
control_EVO/optimization/results/YYYYMMDD_HHMMSS_<scenario>/
```

常见输出：

- `report.html`：完整 HTML 报告。
- `effectiveness_analysis.txt`：与 HTML 复用的文字结论。
- `comparison_metrics.csv`：`dyc_off` / `dyc_on` 总体指标对比。
- `run_results.csv`：每次仿真的停止原因、圈速和运行状态。
- `autocross_where_faster.csv` 或 `figure8_where_faster.csv`：自动“快在哪里”分析表。
- `autocross_where_faster_analysis.txt` 或 `figure8_where_faster_analysis.txt`：与 HTML 复用的“快在哪里”文字结论。
- `signal_data/`：原始时序 CSV、对齐后的 `dyc_on - dyc_off` 时序差值和 `analysis_data.mat`。
- `plots/`：报告内基础时序图。
- `plots_presentation/`：展示版 PNG 图和 `presentation_plot_manifest.csv`。
- `comparison_result.mat`：配置、运行结果、指标和后处理结果数据包。

`control_EVO/optimization/results/` 是本地生成物目录，默认不提交到 Git。

## 3. 圈速停止事件要求

Autocross 和 Figure-8 的默认圈速提取方式不同。

Autocross 使用 `carsim_autocross_stop_time`，接受两类停止原因：

```text
Run stopped at t = <time>. VS Command STOP_RUN_NOW End event triggered
Run stopped at t = <time>. External control (manual or external model)
```

Figure-8 使用 `carsim_station_stop_time`，要求 station-stop 日志：

```text
Run stopped at t = <time>. Station limit reached: driver station = <station>
```

如果日志停止原因不匹配，报告会把该 run 标记为 invalid，不会强行输出圈速有效结论。

## 4. 报告怎么判断 DYC 有效

报告不只比较完成时间。自动结论采用双主线：

- 圈速主线：`dyc_on - dyc_off` 的完成时间差。
- 机理主线：速度保持、最低速度、路径误差 RMSE/峰值、横摆误差、横摆力矩介入、轮胎利用率、驱动矩离散度、油门和纵向加速度。

如果圈速变快，但轮胎峰值利用率、最低速度或其他稳定性指标有局部恶化，报告会写成“圈速有效，稳定性证据混合/不足”，不会直接宣称全面稳定性提升。

## 5. CarSim 前置条件

正常交互式联合仿真仍从 CarSim 开始：

1. 在 CarSim 中选择目标数据库和 Run。
2. 检查 Run Control -> Models 中的 Simulink 模型路径。
3. 检查 Import/Export channels 与 Simulink 端口一致。
4. 点击 `Send to Simulink`。
5. 回到 MATLAB 运行本目录脚本。

如果本机尚未加入 CarSim solver 路径，可先运行：

```matlab
add_dyc_carsim_solver_path();
```

脚本不手工编辑 `simfile.sim`。如果需要切换工况，应在 CarSim GUI 中切换 Run 后重新 `Send to Simulink`。

## 6. PID 圈速优化入口

PID 优化仍保留为独立流程，默认优化变量是：

```text
Kp, Ki, Kd
```

预检查：

```matlab
repo = 'D:\Control_EVO';
addpath(fullfile(repo, 'control_EVO', 'optimization'));
cfg = dyc_pid_optimization_config();
report = run_dyc_pid_preflight(cfg)
```

Station-stop 优化：

```matlab
repo = 'D:\Control_EVO';
addpath(fullfile(repo, 'control_EVO', 'optimization'));
result = optimize_dyc_pid_laptime();
```

Autocross 优化：

```matlab
repo = 'D:\Control_EVO';
addpath(fullfile(repo, 'control_EVO', 'optimization'));
result = optimize_dyc_pid_autocross_laptime();
```

候选仿真通过 `Simulink.SimulationInput` 临时设置 `PID_YawMomentController` 参数，并临时把 `YawMomentControlMode` 切到 PID 分支。优化结果会生成 `best_pid_set_param.m`，是否把最佳参数写入模型需要使用者人工确认后再执行。

## 7. 测试

只运行优化工具测试：

```matlab
repo = 'D:\Control_EVO';
addpath(fullfile(repo, 'control_EVO', 'optimization'));
results = runtests(fullfile(repo, 'control_EVO', 'optimization', 'tests'));
table(results)
```

这些测试覆盖配置、`simfile.sim` 解析、日志指标提取、候选结果记录、报告输出、已有结果后处理和 fake simulation smoke。它们不等于真实 CarSim 长工况验证。

## 8. 验证边界

本目录脚本只能说明固定 CarSim procedure、当前 `simfile.sim` 和本机 CarSim solver/许可证环境下的离线 Simulink/CarSim 仿真表现。它不证明 DIL、实时硬件闭环、Live Video、力反馈或实车性能。
