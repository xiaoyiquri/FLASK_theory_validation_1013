# FLASK-RRTConnect：六个 DynoBench 场景

独立 C++17 / Eigen / yaml-cpp 工程。目标仅为截图第一列：DF + 双向 RRTConnect，运行 Unicycle Bugtrap/Wall、Quad2D Bugtrap/Hole、Quad3D Block/Window。目录名称沿用最初理论验证任务；不继续扩展 SST、DP-RRT、机械臂或对比算法。

2026-10-12验证：CTest **19/19**，六场景seed41/42/43共 **18/18** 规划成功。实际轨迹和统计在 `results/rrtconnect_six_final_20261012`；性能与原生DynoBench认证边界见下文。

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 4
ctest --test-dir build --output-on-failure
python tools/benchmark.py --executable build/flask_plan --output results/six_run --seeds 41
```

Windows 可执行文件通常为 `build/flask_plan.exe`；多配置生成器可能位于 `build/Release`。输出目录必须不存在。脚本默认运行全部六场景，每场内部预算10秒、进程上限15秒，真实记录失败，不换规划器。直接运行单场景：

重复计时支持 `tools/benchmark.py --repeats 20`：对指定的每个seed/场景启动20个独立进程，文件名附加重复序号，全部原始记录保留。执行完后用 `python tools/summarize_runs.py <结果目录>` 生成首解时间、含shortcut总时间、cost和长度的均值/样本标准差，以及成功率和单独的失败耗时。默认仍为每seed一次。固定seed重复仅测量同一随机序列的耗时波动，不是不同seed的采样成功率实验。选择seed20/R=I时须显式传入对应的六份实验配置，不能用默认Unicycle R=10I配置替代。

```sh
build/flask_plan examples/dynobench_quad3d_window.yaml window.yaml 41
```

本机已配置构建目录为 `D:/codex_talk/project_1/tmp/build_flask_runtime`。从本项目目录可执行：

```powershell
& 'D:/NEWFILE/Python313/python.exe' tools/benchmark.py --executable ../../tmp/build_flask_runtime/flask_plan.exe --output results/my_six --seeds 41
```

离线依赖可指定 `FLASK_EIGEN_SOURCE` 和 `FLASK_YAML_SOURCE`，默认不下载。冻结内核在 `vendor/rrtpff`，数据来源/哈希在 `examples/dynobench_data/SOURCE_MANIFEST.json`。不修改相邻 RRT*/PFF 工程与原始数据。

搜索使用 extent 加权 nearest-neighbor、BVP 时间分数插值、两棵树正时间边、connect循环和BVP shortcut。metric/interpolation 对照作者公开 [OMPL port](https://github.com/ompl/ompl/pull/1472)，不是 Eq.33 的 cost neighborhood；range=.25是本地配置，无rewiring，无渐近最优性声明。

目标树截短是本地kinodynamic适配：在 `target→parent` 的incoming BVP上取末段起点；start树取outgoing BVP前段。公开几何port两棵树均用 `parent→target` 插值，不能把本修正误写成作者私有实验默认策略。Unicycle r2额外检查四次速度模平方的所有极值，拒绝网格之间的零速和速度超限；omega和其他物理约束仍按有限采样检查。

六场景均采用 r=2、rho=1。Unicycle固定非零正向速度分支，端点lift显式给出。Quad节点只固定位置/速度（Quad3D另有yaw/yaw rate）；每条cubic从高阶导数恢复物理状态和控制并做采样限幅。配置明确关闭两个高阶检查：`enforce_physical_endpoints: false`、`enforce_physical_continuity: false`，实际端点和拼接误差写入结果。r2不能保证完整姿态/omega连续。通用full-state r4仍可显式选择，默认严格检查，但不再用作六场景配置。

Unicycle可配置 `planner.unicycle_sample_speed`：大于0时均匀采样原状态的heading，用该速度lift出dy；0时使用完整flat box的均匀采样。六场景选择0.5m/s的pose lift，端点速度另由lifting指定。它是本地显式采样策略，不是作者默认值，也不是完整flat自由域的iid覆盖采样，不能套用Eq.34–35。

Unicycle使用Eq.20的SPD权重 `R=10I`，quad使用 `R=I`，rho仍为1。单位权重在本地Bugtrap经常产生转速超限的局部最优边，因此显式调整代价权重；这不是放宽速度/转速约束，也没有宣称恢复作者未公开权重。

几何采用单球机器人和box保守球覆盖，半径/cell显式配置，不冒充作者默认值。默认额外开启Bernstein连续球几何检查；状态/控制仍为有限采样。Algorithm 3可能漏检薄障碍；连续几何检查不等于完整控制证书。

输出：

- `runs.csv` / `summary.md`：成功率、轨迹长度(m)、规划时间(ms)、搜索时间、cost、失败原因。长度积分平移速度模长，不含yaw，不是BVP cost。规划时间包含shortcut，排除进程启动、长度求积和I/O。
- `manifest.json`：本轮二进制、配置、核心源码、runner的SHA-256与运行平台，便于核对结果来源。
- `*_41.yaml`：精确分段多项式、物理样本、输入快照、物理端点/拼接误差。
- `*_41.dynobench.yaml`：`result: [{states, actions}]` 可视化布局，按模型dt采样，末步可能不足dt。不是零阶保持积分输出，明确标记 `dynobench_checker_verified: false`。

本项目读取 DynoBench 数据运行独立规划器，未链接官方DynoBench/FCL runtime。原生动力学checker通过目前尚未达成，不能用“文件格式可读”替代。

审查见 [paper_implementation_status.md](docs/paper_implementation_status.md)，本轮实测见 [rrtconnect_six_review.md](docs/rrtconnect_six_review.md)。之前要求的三个理论测试和 [Theory-to-Code Matrix](docs/theoretical_guarantees.md) 保留为审计材料。历史实验输出保留，不代表当前版本结果。
