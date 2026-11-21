"""Render the measured comparison and check its inputs against the archived C4 runs."""
import base64
import csv
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from evaluate import OUT, CASES, LAYOUTS, write_csv
from simulate import VALIDATION, HERE, C4

def read(name):
    with (OUT/name).open(encoding="utf-8-sig") as f:
        return list(csv.DictReader(f))

def main():
    summary=read("summary.csv")
    per=read("by_run.csv")
    events=read("events.csv")
    nominal=json.loads((OUT/"cylinder_nominal.json").read_text())
    ref=json.loads((C4/"evidence/cylinder_reference.json").read_text())
    assert abs(nominal["placement_error_mm"]-ref["C4"]["placement_mm"])<.001
    assert nominal["passed"] and json.loads((OUT/"ball_nominal.json").read_text())["passed"]
    assert abs(json.loads((OUT/"cylinder_mu055.json").read_text())["hold_span_mm"]-23.89)<.05
    for obj in ("cylinder","ball"):
        for case in CASES:
            with np.load(OUT/f"{obj}_{case}_sensors.npz") as z, np.load(OUT/f"{obj}_{case}.npz") as base:
                a=z["trace"]
                assert a.shape==(len(base["log"]),25) and np.isfinite(a).all()
                np.testing.assert_allclose(a[:,22:25],1000*base["log"][:,18:21],atol=1e-9)
    labels={"two":"拇指＋食指","five":"五个指腹","six":"五个指腹＋掌心"}
    def metric_table(subset):
        buf=""
        for r in summary:
            if r["scope"]==subset and r["threshold_mm_s"]=="24":
                delta=float(r["median_delay_ms"])
                when=("提前 " if delta<0 else "晚 ")+f"{abs(delta):g} ms"
                buf+=f"<tr><td>{labels[r['layout']]}</td><td>{r['hits']} / {r['events']}</td><td>{r['false']}</td><td>{when}</td></tr>"
        return "<table><tr><th>布局</th><th>检出 / 参考事件</th><th>误报次数</th><th>命中事件的报警中位时间</th></tr>"+buf+"</table>"

    tfile=OUT/"cylinder_mu055_comparison.npz"
    with np.load(tfile) as z:
        t=z["t"]; score=z["measured"]; drift=z["drift"]
    fig,axs=plt.subplots(2,1,sharex=True,figsize=(8.8,4.7),layout="constrained")
    palette=["#007669","#cd7b10","#6c4e87"]
    for (name,ids),color,style in zip(LAYOUTS.items(),palette,["-","--",":"]):
        axs[0].plot(t,score[:,ids].max(axis=1),style,color=color,label=name,lw=1.5)
        for r in events:
            if r["case"]=="mu055" and r["object"]=="cylinder" and r["layout"]==name and r["threshold_mm_s"]=="24" and r["alarm_s"]:
                axs[0].axvline(float(r["alarm_s"]),color=color,ls=style,alpha=.75)
    axs[0].axhline(24,color="#666",ls=":",lw=1)
    axs[0].set(ylim=(0,100),ylabel="Local speed (mm/s)",title="Cylinder | friction 0.55 | high hold")
    axs[0].legend(loc="upper left",ncol=3)
    axs[1].plot(t,drift,color="#254757")
    axs[1].axhline(2,color="#666",ls=":")
    e=next(r for r in events if r["case"]=="mu055" and r["object"]=="cylinder")
    for ax in axs:
        ax.axvspan(float(e["onset_s"]),float(e["end_s"]),color="#db9970",alpha=.17)
        ax.set_xlim(3.85,4.33)
        ax.spines[["top","right"]].set_visible(False)
    axs[1].set(xlabel="Simulated time (s)",ylabel="100 ms drift (mm)")
    img=OUT/"cylinder_mu055.png"
    fig.savefig(img,dpi=145);plt.close(fig)
    image=base64.b64encode(img.read_bytes()).decode()
    html=f"""<!DOCTYPE html><html lang="zh-CN"><meta charset="utf-8"><title>指腹触觉布局对照</title>
<style>body{{max-width:850px;margin:40px auto;padding:0 22px;font:16px/1.7 system-ui,'Microsoft YaHei',sans-serif;color:#203638}}h2{{margin-top:2em}}th,td{{border-bottom:1px solid #cbd7d6;text-align:left;padding:7px 14px}}table{{border-collapse:collapse}}img{{width:100%}}b{{color:#00695c}}</style>
<h1>七轴机械臂 · 指腹触觉布局对照</h1><p><b>在已有圆柱和小球任务上，优先保留拇指和食指两个虚拟感应区。</b>掌心暂作诊断记录。小球误报仍较多，本轮未用报警信号改变抓力。</p>
<h2>同一条抓放轨迹，三种布局</h2><p>每种物体 20 次，共 40 次完整 MuJoCo 自由物体抓放仿真。每条轨迹只运行一遍，三个检测器读取同一组带噪声的六区信号。首轮 32 次包含原 C4 配方、起点偏差、张力传感器滞后、摩擦变化和高位保持时施加的外力脉冲。在首轮比较 6、12、24 mm/s 后选取 24 mm/s；另用 8 次组合偏移、摩擦和外力工况验证。</p>
<h2>首轮 32 次</h2>{metric_table("all")}
<h2>独立验证 8 次</h2>{metric_table("validation")}
<p>提前或滞后以物体在掌坐标系中的显著移动为参照。各布局命中的事件集合有差异。验证组两指的 3 次误报全部来自小球。对两指和五指都命中的 7 个验证事件逐一比较，4 次同时，3 次五指更早，两指没有更早的事件。首轮共同命中的 10 个事件中，五指中位提前 9 ms。两指的优势主要是误报少和更容易重新布防。</p>
<h2>原先失败的低摩擦圆柱</h2><p>摩擦系数为 0.55 时，原有保持工况的世界坐标移动幅度 23.89 mm 得到复现。独立判据在 4.020 s 标记显著滑移，两指在 4.072 s 报警，五指和加掌心都在 4.060 s 报警。五指在这一例快 12 ms。阴影是参考事件，竖线为报警，水平虚线为阈值。</p><img src="data:image/png;base64,{image}" alt="低摩擦圆柱两指、五指和六区报警与物体相对位移">
<h2>布置与判断依据</h2><p>六个感应区是固定在末节指腹和掌心内侧的无质量 MuJoCo 椭球 site，尺寸和中心见 zones.json；只纳入相应手套碰撞面的局部接触点。拇指、食指参考原可换橡胶垫，其余位置为布局假设。圆柱正常抓放中食指区受载约占监测时间的 99.7%，拇指候选区未达到 0.06 N；抓球时两区均超过 99%。这个结果涉及划定的区域，不能据此断言整根拇指没有碰圆柱。受载计入所有接触对象。</p>
<p>每区记录法向载荷、切向力及载荷加权的接触面切向相对速度。以局部运动作为理想化触觉速度读数，加入载荷噪声 0.008 N、速度噪声 0.6 mm/s、6 ms 滤波时间常数和 8 ms 传输延迟。检测端只读本布局的局部信号和运动阶段。它代表能读出表面运动的假设传感器，尚未模拟橡胶内部应变、光学成像或实际电子硬件。</p>
<p>从指令开始抬升到最后接近桌面前启用报警，完整抓放都有采样。每区载荷需至少 0.06 N，三种布局统一取各区速度的最大值，连续 20 ms 超阈后报警；低于阈值的 70% 持续 30 ms 后重新布防。计划收指、最后放置和主动松手排除在报警评分外。参考事件要求离桌、物体中心在掌坐标内 100 ms 的位移至少 2 mm，并有全手接触面滑动或失去支撑的证据。此评分针对有明显平移的抓取失稳，不能覆盖原地旋转和所有微滑。</p>
<p>报警起点落在参考事件前 100 ms 到结束之间，才与该事件一对一匹配。额外报警计为误报，持续未复位的旧报警不能算对后来一次滑移的新预警。额外手指在正常受力调整时也有局部擦动。多区取最大值时，较早的报警有时遮住后来滑动。另测每区独立重新布防，首轮 12 mm/s 下五指可检出 22/22 次，误报 125 次；两指为 21/22 次、误报 60 次。结论和融合方式有关。</p>
<p>强扰动后的自由落体也保留在失败记录中。姿态数据和全手接触只用于离线评分；统计为有限确定性工况，每次采用一次传感器噪声实现。两指布局是后续闭环仿真的首选，小球报警仍需复核，当前不建议据此定型硬件。</p></html>"""
    (OUT/"触觉布局对照报告.html").write_text(html,encoding="utf-8")
    print("Validated all",len(CASES)*2,"trajectories and rendered report.")

if __name__=="__main__":
    main()
