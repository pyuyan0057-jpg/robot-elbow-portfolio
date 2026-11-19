"""Screen inverse-designed spring/stroke combinations with the archived collision hulls."""
import argparse
import csv
import gzip
import itertools
import json
from concurrent.futures import ProcessPoolExecutor
from functools import lru_cache
from pathlib import Path

import numpy as np
from inverse_release import DATA, smooth
from preload_design import DIGITS, inverse
from free_ball import run


@lru_cache(None)
def read_setup(location):
    return json.loads(DATA.read_text()),json.loads(Path(location).read_text())


@lru_cache(None)
def design(normal, reserve):
    return inverse(normal,reserve)


def evaluate(job):
    hull_path, spec, dt, stage, mass, out, tag = job
    normal,reserve,*offset = spec
    point = design(normal,reserve)
    setup,hulls = read_setup(hull_path)
    import copy
    hand=copy.deepcopy(setup["hand"])
    hand["digits"][0]["spring_Nm_rad"]=point["spring_Nm_rad"]
    strokes=np.array(point["stroke_mm"])+offset
    for d,s in zip(hand["digits"],strokes):
        d["locked_stroke_m"]=s*.001
    options=setup["options"].copy()
    options.update(ball_mass=mass*.001,dt=dt,private_hulls=hulls,elbow_reference=smooth,
                   hanging_only=stage!="lift",hanging_s=.14 if stage=="screen" else 1.)
    metrics={}
    result,rows=run(hand,trial_metrics=metrics,**options)
    times=np.array([r["t_s"] for r in rows])
    forces=np.array([[r[n+"_N"] for n in DIGITS] for r in rows])
    clean=np.array([r["hard_N"]==0 and r["plane_N"]==0 and r["ring_N"]<=.001 for r in rows]) & (forces.min(axis=1)>.01)
    early=next((t for t,c in zip(times,clean) if not c),None)
    angle=90 if stage=="lift" else 0
    tail=[r for r in rows if r["t_s"]>=times[-1]-.25]
    stable=all(abs(r["elbow_deg"]-angle)<1 and r["ball_rel_speed_m_s"]<.002 and r["spin_rel_rad_s"]<.2 for r in tail)
    record=dict(tag=tag,stage=stage,dt=dt,mass_g=mass,spec=list(spec),stroke_mm=strokes.tolist(),
                release_N=forces[0].tolist(),first_invalid_s=early,clean_samples=int(sum(clean)),
                samples=len(rows),clean_prefix_s=times[-1] if early is None else early,
                clean_total_s=float(np.sum(np.diff(times)*clean[:-1])), floor_s=result["floor_s"],
                peak_tension_N=metrics["all_step_peak_pull_N"],release_fixture_force_N=metrics["fixture_force_N"],
                release_gap_mm=[],final_travel_mm=rows[-1]["travel_rel_mm"],final_elbow_deg=rows[-1]["elbow_deg"],
                terminal_max_speed_m_s=max(r["ball_rel_speed_m_s"] for r in tail),
                terminal_max_spin_rad_s=max(r["spin_rel_rad_s"] for r in tail),
                passed=bool(stage!="screen" and clean.all() and stable and metrics["all_step_peak_pull_N"]<=25))
    ball=np.array([rows[0][k] for k in ("world_x_m","world_down_m","world_z_m")])
    for n in DIGITS:
        pad=np.array([rows[0][f"{n}_pad_{k}_m"] for k in "xyz"])
        record["release_gap_mm"].append(float(1000*np.linalg.norm(ball-pad)-19.2))
    if stage!="screen":
        target=Path(out)/f"{tag}-{stage}-{mass}g-{dt}.csv.gz"
        with gzip.open(target,"wt",newline="",encoding="utf-8") as file:
            writer=csv.DictWriter(file,fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    return record


def batch(pool, jobs, file):
    run_records=[]
    with file.open("w",encoding="utf-8") as stream:
        for row in pool.map(evaluate,jobs,chunksize=1):
            run_records.append(row)
            print(json.dumps(row),file=stream,flush=True)
    return run_records


def run_study(hulls,out,workers):
    out.mkdir(exist_ok=True,parents=True)
    specs=[(n,r,0.,0.,0.) for n,r in itertools.product((.2,.5,1.),(.5,2.,3.))]
    def jobs(cases,stage,dt=.00025,mass=33):
        return [(str(hulls),s,dt,stage,mass,str(out),tag) for tag,s in cases]
    with ProcessPoolExecutor(workers) as pool:
        first=batch(pool,jobs([(f"base{i}",s) for i,s in enumerate(specs)],"screen"),out/"base.jsonl")
        def rank(rec):
            return (rec["clean_prefix_s"],rec["clean_total_s"],min(rec["release_N"]),-max(0,max(rec["release_gap_mm"])))
        chosen=max(first,key=rank)
        n,r=chosen["spec"][:2]
        # Change thumb and middle preloading independently around the least-residual strokes.
        specs2=[(n,r,a,b,c) for a,b,c in itertools.product((0.,1.),(.5,2.,4.),(.5,2.,4.))]
        second=batch(pool,jobs([(f"offset{i}",s) for i,s in enumerate(specs2)],"screen"),out/"offset.jsonl")
        from scipy.stats import qmc
        points=qmc.Sobol(3,scramble=True,seed=19).random_base2(6)
        ranges=np.array([[-1.8,1.8],[-2.,3.],[-1.8,.5]])
        fine=[(f"fine{i}",(n,r,*delta)) for i,delta in enumerate(ranges[:,0]+points*np.ptp(ranges,axis=1))]
        third=batch(pool,jobs(fine,"screen"),out/"fine.jsonl")
        allrows=first+second+third
        winners=sorted(allrows,key=rank,reverse=True)[:3]
        cases=[(w["tag"],w["spec"]) for w in winners]
        hang=batch(pool,jobs(cases,"hang")+jobs(cases,"hang",.000125),out/"hang.jsonl")
        qualified=[case for case in cases if all(x["passed"] for x in hang if x["tag"]==case[0])]
        lift=batch(pool,jobs(qualified,"lift")+jobs(qualified,"lift",.000125),out/"lift.jsonl")
        heavier=[case for case in qualified if all(x["passed"] for x in lift if x["tag"]==case[0])]
        if heavier:
            test=batch(pool,jobs(heavier,"hang",mass=150)+jobs(heavier,"hang",.000125,150),out/"hang150.jsonl")
            ready=[case for case in heavier if all(x["passed"] for x in test if x["tag"]==case[0])]
            batch(pool,jobs(ready,"lift",mass=150)+jobs(ready,"lift",.000125,150),out/"lift150.jsonl")
    designs=[design(*s[:2]) for s in specs]
    (out/"designs.json").write_text(json.dumps(designs,indent=2)+"\n")
    print([(x["tag"],x["clean_prefix_s"],x["floor_s"],x["passed"]) for x in hang])


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("hulls",type=Path)
    parser.add_argument("out",type=Path)
    parser.add_argument("--workers",type=int,default=4)
    args=parser.parse_args()
    run_study(args.hulls,args.out,args.workers)
