import csv,json
import numpy as np
from experiment import *
from compare_all import policy
from detector import replay,Detector
from score import score
from evaluate import write_csv

E=HERE/"evidence"

def main():
    E.mkdir(exist_ok=True)
    model=mj.MjModel.from_xml_path(str(baseline.ENGINE/"arm_hand.xml"))
    d=mj.MjData(model)
    jr=[]
    for trial_seed in range(5):
        d.qvel[:]=np.random.default_rng(trial_seed).normal(size=model.nv)
        mj.mj_forward(model,d)
        for name in ("E3_F4_1-1","E3_F0_2-1","ball","palm"):
            b=model.body(name).id
            point=d.xpos[b]+np.array([.01,.008,.003])
            jac=np.zeros((3,model.nv));tw=np.zeros(6)
            mj.mj_jac(model,d,jac,None,point,b)
            mj.mj_objectVelocity(model,d,mj.mjtObj.mjOBJ_BODY,b,tw,0)
            old=tw[3:]+np.cross(tw[:3],point-d.xpos[b])
            fixed=tw[3:]+np.cross(tw[:3],point-d.xipos[b])
            v=jac@d.qvel
            jr.append(dict(pose=trial_seed,body=name,legacy_error_mm_s=np.linalg.norm(old-v)*1000,
                           corrected_error_mm_s=np.linalg.norm(fixed-v)*1000))
    assert max(row["corrected_error_mm_s"] for row in jr)<1e-8
    write_csv(E/"velocity_check.csv",jr)
    diag=[]
    for case,at in (("val_mu062_offset",4.782),("val_mu040_offset",4.782),("val_push_down",4.764)):
        old=np.load(baseline.OUT/f"ball_{case}_sensors.npz")["trace"]
        new=np.load(OUT/f"ball_{case}_off_sensors.npz")["trace"]
        np.testing.assert_allclose(old[:,22:25],new[:,22:25],atol=1e-10,rtol=0)
        i=round(at/.002);cols=[]
        for a in (old,new):
            v=measure(a[:,1:19].reshape(-1,6,3),seed("ball",case))
            cols.extend(v[i,:2])
        drift=np.linalg.norm(new[i,22:25]-new[i-50,22:25])
        scored=score("ball",case)
        assert not any(abs(t-at)<.12 for t in scored["alarms"])
        diag.append(dict(case=case,time_s=at,old_thumb=cols[0],old_index=cols[1],corrected_thumb=cols[2],
                         corrected_index=cols[3],centroid_drift_100ms_mm=drift,table_N=new[i,21]))
    write_csv(E/"three_false_alarms.csv",diag)
    paired=[];scores=[];examples=[]
    for group,cases in [("historical",baseline.CASES),("A",NEW),("B",B),("C",C),("D",FINAL)]:
        for case in cases:
            for obj in ("cylinder","ball"):
                mode=policy(obj)
                tag=OUT/f"{obj}_{case}"
                off=json.loads(Path(str(tag)+"_off.json").read_text())
                on=json.loads(Path(str(tag)+f"_{mode}.json").read_text())
                z=np.load(str(tag)+f"_{mode}_sensors.npz")
                a=z["trace"];phase=z["times"]
                r=np.load(str(tag)+f"_{mode}_reflex.npz")["trace"]
                y=replay(a[:,1:7].reshape(-1,2,3),seed(obj,case))
                np.testing.assert_allclose(r[:,4:],y.reshape(-1,6),rtol=1e-11,atol=1e-11)
                det=Detector(obj)
                flags=[det.step(t,phase,sample) for t,sample in zip(a[:,0],y)]
                assert np.array_equal(r[:,1],flags)
                log=np.load(str(tag)+f"_{mode}.npz")["log"]
                assert log[:,10:15].max()<=25.00001 and np.max(abs(r[a[:,0]>=phase[4],2:4]))<1e-10
                sc=score(obj,case);scores.append(dict(group=group,**sc))
                paired.append(dict(group=group,object=obj,case=case,policy=mode,off_pass=off["passed"],
                    on_pass=on["passed"],off_hold_mm=off["hold_span_mm"],on_hold_mm=on["hold_span_mm"],
                    alarms=int(r[:,1].sum()),max_stroke_mm=r[:,2:4].max(),peak_tendon_N=log[:,10:15].max(),
                    off_retained=off["retained"],on_retained=on["retained"],off_released=off["released"],
                    on_released=on["released"],on_place_error_mm=on["placement_error_mm"]))
    write_csv(E/"paired_trials.csv",paired)
    write_csv(E/"detection.csv",scores)
    for obj in ("ball","cylinder"):
        rows=[r for r in paired if r["object"]==obj]
        print(obj,"off/on",sum(r["off_pass"] for r in rows),sum(r["on_pass"] for r in rows),
              "recovered",[r["case"] for r in rows if r["on_pass"] and not r["off_pass"]],
              "regressed",[r["case"] for r in rows if not r["on_pass"] and r["off_pass"]])
    print("PASS",len(paired),"matched pairs, offline/online sensor and alarm parity, limits, release, numerical Jacobian.")
if __name__=="__main__":
    main()
