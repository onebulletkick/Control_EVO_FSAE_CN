# Round 9 Pacejka 工程包

这个目录放的是基于 TTC Round 9 数据整理出的 Pacejka 工程模型。完整用法见 `docs/pacejka_round9_usage.md`。

快速验证：

```matlab
run(fullfile("examples", "round9_pacejka_quickstart.m"))
```

目录内容：

- `model/pacejka_round9.mat`：拟合后的模型。
- `model/engineering_usability_summary.csv`：工程可用性评级表。
- `matlab/`：预测和重新拟合代码。
- `examples/`：快速使用示例。
- `plots/`：拟合图与敏感性图。
- `tests/`：MATLAB 单元测试。

此包不包含 TTC 原始数据，只包含拟合后的工程模型和分析产物。
