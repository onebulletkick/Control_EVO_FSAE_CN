# DYC PID 圈速优化工具

本目录存放外部 MATLAB 工具，用于在固定 CarSim Run 上调节 DYC 横摆力矩 PID 的 `Kp`、`Ki`、`Kd`。优化器只负责候选参数搜索，不改变控制器架构，也不为了调参保存 `DYC_1_9_test.slx`。

默认优化对象是：

```text
Kp, Ki, Kd
```

## 1. 预检查

先把优化目录加入 MATLAB 路径，并运行 preflight：

```matlab
repo = 'D:\Control_EVO';
addpath(fullfile(repo, 'control_EVO', 'optimization'));
cfg = dyc_pid_optimization_config();
report = run_dyc_pid_preflight(cfg)
```

preflight 会检查模型路径、PID 模块、`simfile.sim`、CarSim 结果路径、`bayesopt` 可用性和基线指标。若本机 CarSim solver 路径未加入，可先调用：

```matlab
add_dyc_carsim_solver_path();
```

## 2. Station-stop 优化入口

默认入口使用 CarSim station-stop 时间作为圈速指标：

```matlab
repo = 'D:\Control_EVO';
addpath(fullfile(repo, 'control_EVO', 'optimization'));
result = optimize_dyc_pid_laptime();
```

有效运行需要 `LastRun_log.txt` 中出现类似日志：

```text
Run stopped at t = <time>. Station limit reached: driver station = <station>
```

当前 `E_example` 工况的终点配置为 `SSTOP = 245 m`。优化器会把停止时间作为目标值，时间越短目标越优。

## 3. Autocross 优化入口

对于依靠 CarSim 两圈结束事件停止的 Autocross 工况，使用：

```matlab
repo = 'D:\Control_EVO';
addpath(fullfile(repo, 'control_EVO', 'optimization'));
result = optimize_dyc_pid_autocross_laptime();
```

Autocross 入口接受两类停止原因。两圈结束事件：

```text
Run stopped at t = <time>. VS Command STOP_RUN_NOW End event triggered
```

部分短 Autocross 或 preview 工况中的外部控制停止：

```text
Run stopped at t = <time>. External control (manual or external model)
```

这两个停止时间都会作为圈速目标。Autocross 入口默认把候选仿真的 Simulink `StopTime` 提高到 `120`，避免两圈事件被较短的 skidpad 默认停止时间截断。

## 4. 八字绕环 DYC 自动报告入口

八字绕环报告入口用于对比 `dyc_off` 和 `dyc_on`，并自动生成 HTML 报告、CSV、MAT 数据包、展示图和“快在哪里”分析表。使用前先在 CarSim 选择八字绕环 Run，并执行 `Send to Simulink`，确保当前 `simfile.sim` 指向该工况。

```matlab
repo = 'D:\Control_EVO';
addpath(fullfile(repo, 'control_EVO', 'optimization'));
result = run_dyc_figure8_report();
```

该入口会调用 `compare_dyc_figure8_effectiveness()`，按当前 PID DYC 参数和零 PID 增益分别运行：

```text
dyc_off: Kp/Ki/Kd = 0/0/0
dyc_on : Kp/Ki/Kd = 6000/200/0
```

八字绕环默认使用 station-stop 完成时间作为圈速指标，要求 `LastRun_log.txt` 中出现类似日志：

```text
Run stopped at t = <time>. Station limit reached: driver station = <station>
```

常见输出目录为：

```text
control_EVO/optimization/results/YYYYMMDD_HHMMSS_dyc_figure8_comparison/
```

主要输出包括：

- `report.html`：完整 HTML 报告。
- `effectiveness_analysis.txt`：与 HTML 复用的文字结论。
- `comparison_metrics.csv`：总体指标对比。
- `figure8_segment_metrics.csv`：左转、右转、换向过渡区分段指标。
- `figure8_segment_delta.csv`：分段 `dyc_on - dyc_off` 差值。
- `figure8_where_faster.csv`：完成时间、速度、路径误差、轮胎利用率和横摆力矩的“快在哪里”分析表。
- `signal_data/analysis_data.mat`：后处理数据包。
- `plots_presentation/`：展示版 PNG 图和 manifest。

如果只想基于已有结果目录重新生成“快在哪里”分析，不重新跑仿真：

```matlab
resultsDir = 'D:\Control_EVO\control_EVO\optimization\results\<某次_dyc_figure8_comparison>';
analysis = analyze_dyc_figure8_report_results(resultsDir);
```

报告会区分“圈速有效”和“稳定性证据”。如果完成时间缩短但轮胎峰值利用率、最低速度等指标有局部变差，结论会写成稳定性证据混合或不足，而不是直接宣称全面稳定性提升。

`control_EVO/optimization/results/` 默认被 Git 忽略。报告、CSV、PNG 和 MAT 产物用于本地分析，不作为源码提交对象。

## 5. 优化行为

候选仿真通过 `Simulink.SimulationInput` 临时设置 `PID_YawMomentController` 参数，并临时把 `YawMomentControlMode` 切到 PID 分支。这样可以比较候选参数，同时避免为每个候选保存主模型。

preflight 可能提示当前 `.slx` 手动开关不是 PID。这个提示不代表优化器无法运行；候选仿真仍会在内存中强制使用 PID 分支。

优化结果写入：

```text
control_EVO/optimization/results/YYYYMMDD_HHMMSS_pid_laptime/
```

该目录默认被 Git 忽略。常见输出包括：

- `run_log.csv`：每个候选参数和目标值。
- `summary.md`：优化摘要。
- `optimization_result.mat`：MATLAB 结果数据。
- `best_pid_set_param.m`：手动应用最佳 PID 参数的脚本。

如需把最佳参数写入模型，应先检查 `best_pid_set_param.m`，再由使用者明确执行并保存模型。

## 6. 测试

只运行优化工具单元测试：

```matlab
repo = 'D:\Control_EVO';
addpath(fullfile(repo, 'control_EVO', 'optimization'));
results = runtests(fullfile(repo, 'control_EVO', 'optimization', 'tests'));
table(results)
```

这些测试覆盖配置、`simfile.sim` 解析、日志指标提取、候选结果记录、报告输出和轻量 smoke。它们不等于完整 CarSim 长工况验证。

## 7. 边界

这个优化流程只能说明固定 CarSim procedure、当前 `simfile.sim` 和本机 CarSim solver/许可证环境下的候选参数表现。它不证明 DIL、实时硬件闭环、Live Video、力反馈或实车性能。
