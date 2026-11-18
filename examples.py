"""Rebuild the documented default load cases and comparison graph."""
import csv
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
from matplotlib import pyplot as plt
from calculate import LoadCase, PARAMETERS, solve

def main():
    out=Path(__file__).resolve().parent/"examples"
    out.mkdir(exist_ok=True)
    fig,ax=plt.subplots(figsize=(8.2,4.6),layout="constrained")
    with (out/"cases.csv").open("w",newline="",encoding="utf-8") as stream:
        writer=csv.writer(stream,lineterminator="\n")
        writer.writerow(["payload_g","moving_mass_kg","inertia_kg_m2","hold_nm",
                         "lift_peak_abs_nm","design_hold_nm","max_output_rpm"])
        for payload,color in [(0,"#126e71"),(150,"#28588c"),(300,"#b85629")]:
            rows,s=solve(LoadCase(payload,tuple(PARAMETERS["payload_position_mm"]),1.2,1.5))
            writer.writerow([payload]+[format(s[k],".10f") for k in
                ("total_moving_kg","axis_inertia_kg_m2","hold_nm","lift_peak_abs_nm",
                 "design_hold_nm","output_max_rpm")])
            ax.plot([r["time_s"] for r in rows],[r["output_nm"] for r in rows],
                    color=color,label=f"{payload} g",linewidth=2)
            ax.scatter(1.2,s["hold_nm"],s=28,color=color)
    ax.set(title="Load at the middle fingertip  |  90-degree lift in 1.20 s",
           xlabel="Time (s)",ylabel="Joint output torque (N m)")
    ax.grid(alpha=.18);ax.legend(title="Added mass",frameon=False)
    fig.savefig(out/"torque-overlay.png",dpi=150);plt.close(fig)

if __name__=="__main__":
    main()
