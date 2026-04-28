# Round 9 Pacejka 工程模型使用说明

## 这份包里有什么

这份包是基于 FSAE TTC / Calspan Round 9 数据拟合出的 MATLAB 稳态轮胎模型交付物。它的定位是：

- 可作为 lap sim / 车辆动力学仿真的初始 tire model
- 可用于 tire/rim/胎压/外倾角趋势比较
- 可用于工程报告和敏感性分析
- 不包含 TTC 原始 `.mat` / `.dat` 数据，只包含拟合后的模型、代码、图和评级表

核心产物：

- `model/pacejka_round9.mat`：拟合后的 14 个 tire/rim 模型
- `model/engineering_usability_summary.csv`：每个模型的工程可用性评级
- `plots/*.png`：拟合图和敏感性图
- `matlab/*.m`：拟合与预测接口源码
- `examples/round9_pacejka_quickstart.m`：快速使用示例
- `tests/testRound9PacejkaPackage.m`：包级 smoke test，不需要原始 TTC 数据

## 快速开始

1. 在 MATLAB 中打开解压后的包目录。
2. 运行：

```matlab
run(fullfile("examples", "round9_pacejka_quickstart.m"))
```

脚本会自动：

- 加载 `model/pacejka_round9.mat`
- 读取工程评级表
- 选择第一组 `simulationReady = true` 的模型
- 调用 `predictTireForces`
- 输出 `FX_N / FY_N / MZ_Nm`

## 验证安装

在包目录中运行：

```matlab
runtests(fullfile("tests", "testRound9PacejkaPackage.m"))
```

这个测试只检查交付包是否可用：

- 模型文件能加载
- 评级表能读取
- 至少有一个 `simulationReady` 模型
- `predictTireForces` 能输出有限的 `FX/FY/MZ`

## 预测接口

推荐使用 `predictTireForces`：

```matlab
addpath("matlab")
load(fullfile("model", "pacejka_round9.mat"), "result")

input = struct( ...
    "ModelKey", "Goodyear20_0X7_0_13EagleSportsCarSpecial_rim7in", ...
    "SA_deg", [-3; 0; 3], ...
    "SR", [-0.04; 0; 0.04], ...
    "FZ_N", [600; 800; 1000], ...
    "IA_deg", [0; 1; 2], ...
    "P_kPa", [82; 84; 86], ...
    "V_kph", [40; 40; 40]);

y = predictTireForces(result, input);
```

输入字段：

| 字段 | 单位 | 含义 |
| --- | --- | --- |
| `ModelKey` | - | `engineering_usability_summary.csv` 里的模型 key |
| `SA_deg` | deg | slip angle |
| `SR` | - | slip ratio |
| `FZ_N` | N | 正的法向载荷 |
| `IA_deg` | deg | inclination / camber angle |
| `P_kPa` | kPa | 胎压 |
| `V_kph` | kph | 速度 |

输出字段：

| 字段 | 单位 | 含义 |
| --- | --- | --- |
| `FX_N` | N | 纵向力 |
| `FY_N` | N | 侧向力 |
| `MZ_Nm` | Nm | 回正力矩 |

如果输入超出该模型拟合数据范围，函数会发出 `Round9Pacejka:OutOfRange` warning。这个 warning 不代表函数不能算，只是提示结果已经进入外推区间。

## 工程可用性评级

评级文件：`model/engineering_usability_summary.csv`

重点字段：

| 字段 | 含义 |
| --- | --- |
| `overallGrade` | 综合评级，A 最好，Unavailable 表示不可用 |
| `lateralGrade` | `FY-SA` 横向模型评级 |
| `longitudinalGrade` | `FX-SR` 纵向模型评级 |
| `aligningGrade` | `MZ-SA` 回正力矩模型评级 |
| `combinedGrade` | combined-slip 简化修正评级 |
| `trendReady` | 是否适合趋势分析 |
| `simulationReady` | 是否适合作为仿真初始模型 |
| `recommendation` | 使用建议 |

当前结果摘要：

- 14 个 tire/rim 模型已生成
- 10 个完整 `FX/FY/MZ` 模型 `simulationReady = true`
- 4 个 Hoosier 16-inch 组合没有 drive/brake 原始数据，因此是横向 partial model
- Goodyear 20.0x7.0-13 两个轮辋宽度整体评级为 A
- 大多数完整模型整体为 B，可作为 lap-sim seed

## 图像怎么读

每个模型有两张图：

- `<modelKey>.png`：`FY-SA`、`FX-SR`、`MZ-SA` 拟合图
- `<modelKey>_sensitivities.png`：载荷、胎压、外倾角敏感性图

看图顺序：

- 先看 `engineering_usability_summary.csv`，筛出 `simulationReady = true`
- 再看对应拟合图，确认峰值区域和低滑移刚度是否合理
- 最后看 sensitivity 图，判断胎压、载荷和外倾角趋势是否符合车辆设定

## 重新拟合

这个交付包默认不包含原始 TTC 数据。若本地保留了 Round 9 原始数据，可以重新拟合：

```matlab
addpath("matlab")

opts = struct( ...
    "DataRoot", "/Users/emberfallen/Desktop/Tire_data/Tire_data", ...
    "OutputDir", "model_refit", ...
    "UseSteadyStateFilter", true, ...
    "MakePlots", true, ...
    "SaveModelFile", true);

result = fitRound9PacejkaModels(opts);
```

默认拟合流程包括：

- 数据完整性检查
- 稳态点筛选
- 按 tire/rim 分组拟合
- `FY/FX/MZ` holdout 验证
- combined-slip 简化修正
- 工程可用性评级
- 模型文件、CSV、图像输出

## 使用边界

这版模型是工程可用的初始模型，不是最终比赛定版模型。

可以直接用于：

- 第一轮 lap sim
- 轮胎/轮辋趋势比较
- 胎压/外倾角敏感性研究
- 控制策略和参数扫描的初始 tire block

最终用于比赛决策前建议：

- 用实车 skidpad、acceleration、braking 数据校准峰值 grip
- 检查车上轮胎温度窗口是否和 Calspan 测试温度接近
- 不要对明显超出拟合域的载荷、速度、胎压做强结论

## 版本说明

当前包对应 v2 工程版：

- 相比初版，新增稳态筛选
- 新增分层 holdout
- 新增 per-axis 工程评级
- 新增 `engineering_usability_summary.csv`
- 新增敏感性图
