# 五指与掌心虚拟触觉布局试验

基线为七轴仿生臂 C4（bee4df5）。研究范围为检测和布置，报警不改变原抓放控制。首轮 32 次抓放，冻结候选布局与阈值后，在另外 8 次组合工况上复核。完整 HTML 包含统一规则、灵敏度、误报、监测范围和模型假设。

## 复跑

环境沿用 C4 的 Python 3.11、MuJoCo 3.3.7、g++ 和 C4 requirements.txt。在装好 C4 依赖的环境内，从仓库根目录依次运行：

```sh
python studies/seven-axis-tactile-c5/simulate.py --workers 4
python studies/seven-axis-tactile-c5/evaluate.py
python studies/seven-axis-tactile-c5/report.py
```

study 的工作目录在 `outputs/tactile-c5/`，含 40 条轨迹的抓放验收 JSON、原始运动与合成传感器信号、逐事件匹配、逐次误报、阈值对照表、PNG 和可离线打开的中文 HTML。调低 `--workers` 可降低并行占用。

- `zones.json` 定义固定在各末节和掌部的局部椭球，无接触、无质量。每种模型按原有手套几何与该区域的交集采样。
- `read_zones.cpp` 只读 MuJoCo 接触和局部相对速度。传感器接收区内所有接触，评分端另存桌面法向力、全手接触和物体掌坐标。局部速度属于可感知表面运动的理想化假设。
- `simulate.py` 固定控制配方。首轮受扰试验与独立验证施加的力是加在物体质心上的平滑外力，高位保持开始 350 ms 后起效、总长 180 ms。
- `evaluate.py` 在同一条记录上比较三种布局。主规则共享一次报警状态，另以逐区重新布防交叉检查。每条轨迹依次保存的列是时间、六区的法向力 N / 切向力幅值和 N / 载荷加权的局部滑动速度 mm/s、物体与全手套的法向力 N / 局部滑动速度 mm/s、物体与桌面的法向力 N、物体质心在掌坐标中的三轴位置 mm。切向力栏为各接触点切向力模长之和。
- `report.py` 逐条核对传感器轨迹和原控制日志的物体相对位置，并检查 C4 名义结果与已知低摩擦失败可复现。

[布局报告](evidence/report.html) · [按工况、阈值和布局比较](evidence/summary.csv) · [逐事件报警时间](evidence/events.csv) · [六区接触覆盖率](evidence/by_zone.csv)

[MuJoCo 接触力接口](https://mujoco.readthedocs.io/en/3.3.7/APIreference/APIfunctions.html#mj-contactforce) · [GelSlim 表面运动感知实物参考](https://web.mit.edu/mcube/research/gelslim.html)
