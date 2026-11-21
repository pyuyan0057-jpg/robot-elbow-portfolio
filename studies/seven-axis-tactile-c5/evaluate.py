"""Causal local readouts vs independent object-in-palm motion; no control intervention."""
import csv
import json
from pathlib import Path
import numpy as np
from scipy.signal import lfilter
from simulate import OUT, ZONES, CASES, VALIDATION

DT = .002
LAYOUTS = {"two":[0,1], "five":[0,1,2,3,4], "six":list(range(6))}
THRESHOLDS = (6,12,24)
PRIMARY = 12


def intervals(mask, gap=0, minimum=1):
    transitions = np.diff(np.r_[False, mask, False].astype(int))
    pairs = zip(np.where(transitions==1)[0],np.where(transitions==-1)[0]-1)
    merged = []
    for a,b in pairs:
        if merged and a-merged[-1][1]-1 <= gap:
            merged[-1][1]=b
        else:
            merged.append([a,b])
    return [(a,b) for a,b in merged if b-a+1>=minimum]


def measure(raw, seed):
    # Simulated optical/shear-motion proxy. Noise/lag are design assumptions.
    n = len(raw)
    noise = np.random.default_rng(seed)
    load = np.maximum(0,raw[:,:,0]+noise.normal(0,.008,(n,6)))
    speed = np.maximum(0,raw[:,:,2]+noise.normal(0,.6,(n,6)))
    gain = 1-np.exp(-DT/.006)
    filt = lambda x: lfilter([gain],[1,gain-1],x,axis=0)
    # Filter load-weighted velocity to avoid large unloaded readings at contact edges.
    filtered_load = filt(load)
    observed = filt(load*speed)/np.maximum(filtered_load,.001)
    observed[filtered_load<.06]=0
    return np.vstack([np.zeros((4,6)), observed[:-4]])


def alarms(values, enabled, threshold):
    active=False
    above=below=0
    starts=[]
    for i,(value,on) in enumerate(zip(values,enabled)):
        if not on:
            above=below=0
            active=False
            continue
        above=above+1 if value>=threshold else 0
        below=below+1 if value<.7*threshold else 0
        if not active and above>=10:
            starts.append(i)
            active=True
        if active and below>=15:
            active=False
    return starts


def audit(trace,times):
    t=trace[:,0]
    enabled=(t>=times[0]) & (t<times[3]-.55)
    # Independent reference in palm coordinates, audited against whole-hand contact.
    # Table force and object pose are never given to the detector.
    airborne=trace[:,21]<.03
    p=trace[:,22:25]
    drift=np.zeros(len(t))
    drift[50:]=np.linalg.norm(p[50:]-p[:-50],axis=1)
    mask=enabled & airborne & (drift>=2) & ((trace[:,20]>=2) | (trace[:,19]<.015))
    # Require the full displacement window to be off the table.
    mask &= np.convolve(airborne.astype(int),np.ones(51,int),'full')[:len(t)]==51
    return enabled,drift,intervals(mask,gap=15,minimum=6)


def match(starts,events,t):
    # Match each rising edge once, in time order. Extra alarms count as false.
    hits={}
    false=[]
    for i in starts:
        targets=[j for j,(a,b) in enumerate(events) if j not in hits and a-50<=i<=b]
        if targets:
            hits[targets[0]]=i
        else:
            false.append(i)
    return hits,false


def write_csv(path,rows):
    with path.open("w",encoding="utf-8-sig",newline="") as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    events_out=[]; runs=[]; zones=[]; details=[]; alternate=[]
    for ci,case in enumerate(CASES):
        for oi,obj in enumerate(("cylinder","ball")):
            tag=OUT/f"{obj}_{case}"
            with np.load(str(tag)+"_sensors.npz") as z:
                trace=z["trace"]; times=z["times"]
            t=trace[:,0]
            assert np.allclose(np.diff(t),DT,rtol=0,atol=1e-9)
            raw=trace[:,1:19].reshape(-1,6,3)
            observed=measure(raw,481+ci*2+oi)
            enabled,drift,events=audit(trace,times)
            plant=json.loads(tag.with_suffix(".json").read_text())
            for j,key in enumerate(ZONES):
                zones.append(dict(object=obj,case=case,zone=key,
                    loaded_pct=round(100*np.mean(raw[enabled,j,0]>=.06),3),
                    peak_normal_N=round(raw[enabled,j,0].max(),4)))
            for th in THRESHOLDS:
                for name,channels in LAYOUTS.items():
                    score=observed[:,channels].max(axis=1)
                    starts=alarms(score,enabled,th)
                    if th==PRIMARY:
                        # Independent channel re-arming removes masking by an already active finger.
                        edges=sorted({edge for z in channels for edge in alarms(observed[:,z],enabled,th)})
                        distinct=[]
                        for edge in edges:
                            if not distinct or edge-distinct[-1]>=15:
                                distinct.append(edge)
                        alt_hits, alt_fp=match(distinct,events,t)
                        alternate.append(dict(object=obj,case=case,layout=name,events=len(events),
                                              hits=len(alt_hits),false=len(alt_fp),monitor_s=sum(enabled)*DT,
                                              delays_ms=[round(1000*(t[i]-t[events[j][0]])) for j,i in alt_hits.items()]))
                    hits,false=match(starts,events,t)
                    runs.append(dict(object=obj,case=case,layout=name,threshold=th,
                        task_pass=plant["passed"],events=len(events),hits=len(hits),miss=len(events)-len(hits),
                        false=len(false),monitor_s=sum(enabled)*DT,
                        delays_ms=[round(1000*(t[i]-t[events[j][0]])) for j,i in hits.items()]))
                    for j,(a,b) in enumerate(events):
                        i=hits.get(j)
                        events_out.append(dict(object=obj,case=case,layout=name,threshold_mm_s=th,event=j+1,
                            onset_s=t[a],end_s=t[b],alarm_s=None if i is None else t[i],
                            delay_ms=None if i is None else round(1000*(t[i]-t[a]))))
                    for i in false:
                        details.append(dict(object=obj,case=case,layout=name,threshold_mm_s=th,false_alarm_s=t[i]))
            if case in ("nominal","mu055"):
                np.savez_compressed(str(tag)+"_comparison.npz",t=t,enabled=enabled,drift=drift,
                                    measured=observed,events=np.array(events,dtype=int).reshape(-1,2))
    write_csv(OUT/"independent_channel_edges.csv",alternate)
    write_csv(OUT/"by_run.csv",runs); write_csv(OUT/"by_zone.csv",zones)
    if events_out: write_csv(OUT/"events.csv",events_out)
    if details: write_csv(OUT/"false_alarms.csv",details)
    summary=[]
    for scope in ("all","cylinder","ball","validation","validation_cylinder","validation_ball"):
        for th in THRESHOLDS:
            for name in LAYOUTS:
                rows=[r for r in runs if r["layout"]==name and r["threshold"]==th and
                      ((r["case"] in VALIDATION) == scope.startswith("validation")) and
                      (scope in ("all","validation") or r["object"]==scope.split("_")[-1])]
                hits=sum(r["hits"] for r in rows); false=sum(r["false"] for r in rows)
                delay=[v for r in rows for v in r["delays_ms"]]
                summary.append(dict(scope=scope,threshold_mm_s=th,layout=name,runs=len(rows),
                    events=sum(r["events"] for r in rows),hits=hits,miss=sum(r["miss"] for r in rows),
                    false=false,false_per_min=round(60*false/sum(r["monitor_s"] for r in rows),2),
                    median_delay_ms=float(np.median(delay)) if delay else None,
                    early=sum(v<0 for v in delay)))
    write_csv(OUT/"summary.csv",summary)
    print(*[r for r in summary if r["threshold_mm_s"]==PRIMARY],sep="\n")


if __name__=="__main__":
    main()
