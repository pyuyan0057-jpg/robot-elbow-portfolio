import argparse,csv,json
from experiment import *
from detector import Detector,replay

def score(obj,case,mode="off"):
    tag=OUT/f"{obj}_{case}_{mode}"
    with np.load(str(tag)+"_sensors.npz") as z:
        a=z["trace"];ph=z["times"]
    values=replay(a[:,1:7].reshape(-1,2,3),seed(obj,case))
    d=Detector(obj)
    alarms=[i for i,t in enumerate(a[:,0]) if d.step(t,ph,values[i])]
    enabled,drift,events=audit(a,ph)
    hits,false=match(alarms,events,a[:,0])
    # Keep the legacy reference separately, including ballistic drift after final separation.
    # With no hand contact in the lead, event or remainder and >200 mm palm distance,
    # the object has already left the grasp. This is an offline classification only.
    detached=[j for j,(i,end) in enumerate(events) if
              np.max(a[max(0,i-50):,19])<.015 and np.linalg.norm(a[i,22:25])>200]
    return dict(obj=obj,case=case,events=len(events),hits=len(hits),fp=len(false),
                detached=detached,in_hand=len(events)-len(detached),
                detected_in_hand=sum(j not in detached for j in hits),
                misses=[round(a[i,0],3) for j,(i,end) in enumerate(events) if j not in hits],
                false_times=[round(a[i,0],3) for i in false],
                alarms=[round(a[i,0],3) for i in alarms],
                times=[round(a[i,0],3) for i,j in events])
if __name__=="__main__":
    group=sys.argv[1]
    cases={"historical":baseline.CASES,"A":NEW,"B":B,"C":C,"D":FINAL}[group]
    rows=[score(o,c) for c in cases for o in ("ball","cylinder")]
    for r in rows:
        if r["misses"]:print("miss",r)
    for o in ("ball","cylinder"):
        rr=[r for r in rows if r["obj"]==o];print(o,*[sum(r[k] for r in rr) for k in ("events","hits","fp")])
    (OUT/f"detection_{group}.json").write_text(json.dumps(rows,indent=2))
