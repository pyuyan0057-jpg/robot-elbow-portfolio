import csv,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from study import HERE,OUT,c6,NAMES,REF,TEST,FINAL_TEST,TUNES,PRE,SELECTED
from final_check import FINAL_POLICY
from detector import Detector,replay,DT

E=HERE/"evidence"

def save(name,rows):
    with (E/name).open("w",encoding="utf-8-sig",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)

def res(case,mode,suffix=""):
    path=OUT/f"{case}_{mode}{suffix}.json"
    if not path.is_file() and mode in ("both","off") and not suffix:
        previous="firm" if mode=="both" else "off"
        path=c6.OUT/f"ball_{case}_{previous}.json"
    return json.loads(path.read_text())

def ablate():
    base=np.load(OUT/f"{REF}_off_data.npz")
    a0=base["trace"]
    t=a0[:,0];pulse=np.searchsorted(t,3.55);end=np.searchsorted(t,4.35)-1
    direction=base["directions"][pulse,:3]
    tape=np.load(OUT/f"{REF}_both_data.npz")["feedback"][:,2]
    rows=[];fig,axes=plt.subplots(2,1,figsize=(8.1,5.4),sharex=True,layout="constrained")
    for m,color in zip(NAMES,["#59686c","#26718d","#b25732","#68479b","#999","#aaa"]):
        z=np.load(OUT/f"{REF}_{m}_data.npz");a=z["trace"];r=z["feedback"];x=a[:,22:25]
        result=res(REF,m)
        np.testing.assert_allclose(a[t<=3.748],a0[t<=3.748],rtol=0,atol=1e-10)
        if m.startswith("fixed_"):np.testing.assert_array_equal(r[:,2],tape)
        if "index" in m: assert np.max(r[:,4])==0
        if "thumb" in m: assert np.max(r[:,3])==0
        delta=x[end]-a0[end,22:25]
        rows.append(dict(policy=m,pass_full=result["passed"],hold_span_mm=result["hold_span_mm"],
            end_minus_off_X_mm=delta[0],end_minus_off_Y_mm=delta[1],end_minus_off_Z_mm=delta[2],
            thumbward_mm=-delta@direction,stroke_index_mm=max(r[:,3]),stroke_thumb_mm=max(r[:,4]),
            alarms_s=np.round(t[r[:,1]>0],3).tolist(),executed_times_s=np.round(t[(r[:,2]>0)&(m!="off")],3).tolist()))
        if not m.startswith("fixed_"):
            axes[0].plot(t,-(x-x[pulse])@direction,label=m,color=color,lw=1.8)
            axes[1].plot(t,np.max(r[:,3:5],axis=1),color=color,label=m,lw=1.5)
    for ax in axes:
        ax.axvspan(3.55,3.73,color="#f0cc8c",alpha=.4)
        ax.axvline(3.748,color="#566",ls=":",lw=1)
        ax.set_xlim(3.48,4.35);ax.spines[["top","right"]].set_visible(False)
    axes[0].set_title("Same 3 N downward pulse | palm frame")
    axes[0].set_ylabel("Toward thumb (mm)");axes[0].legend(ncol=4,frameon=False)
    axes[1].set_ylabel("Max extra stroke (mm)");axes[1].set_xlabel("Simulation time (s)")
    fig.savefig(E/"finger_ablation.png",dpi=145);plt.close(fig)
    save("finger_ablation.csv",rows)
    for mode,old in (("off","off"),("both","firm")):
        a=np.load(OUT/f"{REF}_{mode}.npz")["log"]
        hist=np.load(c6.OUT/f"ball_{REF}_{old}.npz")["log"]
        np.testing.assert_allclose(a,hist,rtol=0,atol=1e-10)

def final_run(case,suffix=""):
    tag=f"{case}_{FINAL_POLICY}{suffix}"
    z=np.load(OUT/(tag+"_data.npz")); f=z["feedback"]; a=z["trace"];ph=z["times"]
    assert a.shape[0]==len(f) and np.isfinite(a).all() and np.isfinite(f).all()
    assert np.all(f[:,3:5]>=-1e-10) and np.max(f[:,3])<=1.95+1e-10 and np.max(f[:,4])<=1.4+1e-10
    assert np.max(np.abs(f[a[:,0]>=ph[4],3:5]))<1e-10
    assert np.max(np.abs(np.diff(f[:,3:5],axis=0)))<=22*DT+1e-10
    from study import recipe
    cfg,force,offset,duration,seed=recipe(case)
    expected=replay(a[:,1:7].reshape(-1,2,3),seed)
    np.testing.assert_allclose(expected.reshape(-1,6),f[:,5:],atol=1e-10,rtol=1e-11)
    det=Detector("ball");np.testing.assert_array_equal(f[:,1],[det.step(t,ph,y) for t,y in zip(f[:,0],expected)])
    h=(a[:,0]>ph[1]+.08)&(a[:,0]<ph[2])
    rr=res(case,FINAL_POLICY,suffix)
    sampled=np.ptp(a[h,22:25],axis=0).max()
    assert sampled<=rr["hold_span_mm"]+.1 and max(rr["peak_tension"])<=25.00001
    assert not rr["passed"] or (rr["hold_span_mm"]<2 and all(rr[k] for k in ["rest","hold","retained","reseated","released","no_retouch"]))
    off=res(case,"off",suffix);old=res(case,"both",suffix)
    minimum=(np.minimum(f[:,3],1.45)+np.minimum(f[:,4],.7))*.5
    return dict(case=case,off_pass=off["passed"],old_pass=old["passed"],new_pass=rr["passed"],
                off_span_mm=off["hold_span_mm"],old_span_mm=old["hold_span_mm"],new_span_mm=rr["hold_span_mm"],
                preloaded=bool(minimum.max()>0),additional_feedback=bool(((f[:,3]>1.45001)|(f[:,4]>.70001)).any()),
                new_retained=rr["retained"],new_released=rr["released"],new_place_error_mm=rr["placement_error_mm"],
                off_place_error_mm=off["placement_error_mm"],max_index_travel_mm=f[:,3].max(),max_thumb_travel_mm=f[:,4].max(),
                max_index_tension_N=rr["peak_tension"][0],max_thumb_tension_N=rr["peak_tension"][1],
                pulse_xyz_N=force.tolist(),pulse_duration_s=duration)

def main():
    E.mkdir(exist_ok=True);ablate()
    groups={"historical_44":[*c6.baseline.CASES,*c6.NEW,*c6.B,*c6.C,*c6.FINAL],
            "first_test_12":list(TEST),"final_blind_14":list(FINAL_TEST)}
    rows=[]
    for g,cases in groups.items():
        subset=[]
        for case in cases:
            row=final_run(case)
            row["group"]=g
            pre=OUT/f"{case}_preload_only.json"
            row["pre_only_pass"]=res(case,"preload_only")["passed"] if pre.exists() else ""
            row["pre_only_span_mm"]=res(case,"preload_only")["hold_span_mm"] if pre.exists() else ""
            subset.append(row)
        rows.extend(subset)
        print(g,len(subset),[(k,sum(r[k] for r in subset)) for k in ("off_pass","old_pass","new_pass")],
              "rescue vs off",[r["case"] for r in subset if not r["off_pass"] and r["new_pass"]],
              "worse vs off",[r["case"] for r in subset if r["off_pass"] and not r["new_pass"]])
    save("matched_trials.csv",rows)
    fine=[final_run(c,"_dt125") for c in ("push_y3","val_push_down","v_heavy_z","v_return_side")]
    fine.append(final_run("v_heavy_z2","_dt125"))
    save("step_125us.csv",fine)
    refined=[]
    for step,suffix in ((250,""),(125,"_dt125"),(100,"_dt100")):
        row=final_run("v_heavy_z2",suffix);row["step_us"]=step
        pre=res("v_heavy_z2","preload_only",suffix)
        row.update(pre_only_pass=pre["passed"],pre_only_span_mm=pre["hold_span_mm"])
        refined.append(row)
    save("rescue_timestep.csv",refined)
    sweep=[]
    for m in [*TUNES,*PRE]:
        for c in ["push_y3","val_push_down","push_nx3","val_push_diag","push_y6","nominal",*TEST]:
            if (OUT/f"{c}_{m}.json").is_file():
                r=res(c,m);sweep.append(dict(case=c,mode=m,pass_full=r["passed"],hold_mm=r["hold_span_mm"]))
    save("tuning.csv",sweep)
    print("PASS: matched C6 native logs, fixed commands, sensor replay, rate, stroke, release and native task criteria.")
if __name__=="__main__":
    main()
