# 轻型机器人肘关节模组：选型计算与刚度优化

[English](README.md) · [硬件授权说明](docs/sources-and-rights.md) · [计算、边界条件和网格收敛](docs/mechanics.md)

单自由度肘关节带动已有五指手和六驱动前臂，从下垂抬到水平。中指端另加 150 g 时，水平保持需要 **2.390 N·m**。在归档静力工况下，加宽固定侧支架后，该支架引起的中指下沉由 **0.2669 降到 0.0576 mm**。

<a href="assets/cad-lift.gif"><img src="assets/cad-lift.gif" width="30%" alt="R2 CAD assembly lifting the forearm from vertical to horizontal"></a> <a href="assets/bracket-comparison.png"><img src="assets/bracket-comparison.png" width="67%" alt="Same-scale R1 and R2 yokes, with added ring and spine material highlighted"></a>

*左图来自实际 SolidWorks 装配运动。右图采用相同视角和比例，橙色标出新增环体和背脊连接。环外径从 94 增至 106 mm，新增的 26.93 g 留在上臂固定侧。点击图片可查看大图。*

## 快速复算

在仓库根目录用 Python 3.11–3.14 和 pip 运行。计算和绘图无需 SolidWorks，可在无图形界面的环境中完成。[核验环境和命令](docs/verification.md)

```sh
python -m venv .venv
# Windows PowerShell
.\.venv\Scripts\Activate.ps1
# macOS / Linux: source .venv/bin/activate
python -m pip install -r requirements.txt
python calculate.py --payload-g 150
python calculate.py --payload-g 300
```

如 PowerShell 限制环境激活，创建环境后可直接使用 `.\.venv\Scripts\python.exe` 执行安装与计算。终端打印保持扭矩、抬升峰值、峰值时刻和最高转速。每个工况写入独立的 `outputs/payload-<质量>g-<参数指纹>/`，包含扭矩曲线 PNG、时序 CSV、输入快照和结果摘要。相同输入会更新同一组输出。

```sh
python calculate.py --payload-g 300 --payload-position-mm 350 0 -11 --lift-time-s 1.5 --design-factor 1.8
python -m unittest discover -s tests -v
python examples.py
```

## 三个工况

空手运动组估算 **1.60106 kg**，水平重力臂 **115.18 mm**。U 沿抬平时的前臂指向手端，V 向上，W 沿肘轴；默认中指测点为 (395.214, −0.413, −10.956) mm。负载同时进入重力矩和绕肘轴惯量。加速、匀速、减速分别占抬升总时间的 25%、50%、25%。设计系数用于选型余量。

| 手端另加载荷 | 水平保持 N·m | 抬升过程峰值 N·m | 1.5 倍设计保持 N·m | 最高转速 rpm |
|---:|---:|---:|---:|---:|
| 0 g | 1.80847 | 1.74634 | 2.71271 | 16.67 |
| 150 g | 2.38983 | 2.30773 | 3.58475 | 16.67 |
| 300 g | 2.97119 | 2.86912 | 4.45679 | 16.67 |

默认 1.20 s 抬升，过程峰值出现在 0.90 s、75° 的匀速末端。停稳后重力力臂最大，因此水平保持值高于运动阶段峰值。150→300 g 的保持增量独立手算为 **0.150 × 9.80665 × 0.3952143649 = 0.58135934 N·m**，自动测试按未舍入值和 **10⁻⁹ N·m** 容差核对。[工况表 CSV](examples/cases.csv)

![三种负载下的关节扭矩曲线](examples/torque-overlay.png)

## 结构决策与结果边界

电机固定在上臂，两只外置轴承承受前臂重量，滑块传递驱动力矩。150 g 设计保持需求为 3.585 N·m，AK60-6 的额定输出 3 N·m，AK70-10 的额定输出 8.3 N·m。零速长期保持能力还需相应散热条件下的温升数据。[侧视受力图](assets/free-body.png)

同一静力工况下，支架估算质量由 **293.80 增至 320.73 g**，支架引起的中指下沉降低 **78.4%**。R2 支架和转接件两项贡献合计约 **0.179 mm**。齿轮箱、轴承、前臂等柔度仍需单独获得。CLI 修改负载只更新动力学；仓库中的有限元数据来自原先完成的九项研究。[网格收敛与反力 CSV](data/mesh-convergence.csv)

申请人确定目标、载荷工况和验收要求，推进方案评审；建模、求解、代码与展示资料使用工具辅助。项目为数字样机，未做实测。计算代码使用 MIT 许可。硬件设计文件按需评审，使用项目设计进行商业生产请先联系作者取得书面许可。[来源与授权范围](docs/sources-and-rights.md)
