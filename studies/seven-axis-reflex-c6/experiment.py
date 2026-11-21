import argparse, json, os, sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import subprocess
os.environ["OPENBLAS_NUM_THREADS"]="1"
import numpy as np
import mujoco as mj

HERE=Path(__file__).resolve().parent
C5=HERE.parent/"seven-axis-tactile-c5"
sys.path.insert(0,str(C5))
import simulate as baseline
from evaluate import audit, match, measure

OUT=HERE.parents[1]/"outputs"/"reflex-c6"
LIB=OUT/baseline.LIB.name
# (world force in N, pulse offset from start of hold, duration). Include return-stage disturbances.
NEW={
 "a_quiet":({"unknown_mm":[.6,0,-1.3]}, [0,0,0],.52,.15),
 "a_friction":({"mu_override":.585,"unknown_mm":[-.75,0,.55]},[0,0,0],.42,.19),
 "a_horizontal":({"unknown_mm":[-.5,0,.8]},[-3.25,.45,0],.52,.21),
 "a_gravity":({"unknown_mm":[.7,0,.3]},[.25,4.7,0],.48,.16),
 "a_return":({},[-3.2,.8,0],1.48,.19),
 "a_return_load":({"lag_ms":10},[-.3,5.1,0],1.62,.18),
}
B={
 "b_quiet":({"unknown_mm":[-1.2,0,-.4]},[0,0,0],.26,.14),
 "b_friction":({"mu_override":.515,"unknown_mm":[.3,0,1.1]},[0,0,0],.29,.22),
 "b_diag":({"unknown_mm":[.4,0,.65]},[-3.05,.9,.2],.27,.23),
 "b_down":({"unknown_mm":[-.8,0,.45]},[-.7,4.9,.3],.61,.21),
 "b_return":({"unknown_mm":[.2,0,.55]},[-3.7,.65,0],1.55,.17),
 "b_return_down":({"lag_ms":11},[.3,4.25,.4],1.36,.23),
}

# Final untouched set, defined after A/B development, before the first C simulation.
C={
 "c_soft":({"unknown_mm":[-.45,0,-.8]},[.2,.15,-.15],.33,.13),
 "c_friction":({"mu_override":.57,"unknown_mm":[.15,0,-.45]},[0,0,0],.4,.2),
 "c_diag":({"unknown_mm":[-.6,0,.25]},[-3.18,1.25,-.2],.53,.20),
 "c_down":({"unknown_mm":[.35,0,-.3]},[-.15,4.65,.2],.39,.195),
 "c_return_side":({"unknown_mm":[-.15,0,.25]},[-3.35,.85,.1],1.52,.185),
 "c_return_load":({"lag_ms":9},[.2,4.85,-.45],1.43,.20),
}

FINAL={
 "d_quiet":({"unknown_mm":[-.22,0,.62]},[.1,.2,0],.3,.25),
 "d_lowmu":({"mu_override":.545,"unknown_mm":[.4,0,-.6]},[0,0,0],.3,.2),
 "d_diag":({"unknown_mm":[.15,0,-.35]},[-3.10,.75,.4],.435,.205),
 "d_gravity":({"unknown_mm":[-.65,0,.32]},[-.3,4.55,.15],.465,.17),
 "d_return_side":({"unknown_mm":[.18,0,-.52]},[-3.45,.6,-.26],1.46,.20),
 "d_return_load":({"lag_ms":10.5},[-.25,4.45,.35],1.51,.205),
}

def params(obj,case):
    cfg=json.loads((baseline.ENGINE/f"recipe_{obj}.json").read_text())
    cfg.update(json.loads((baseline.C4/f"{obj}_changes.json").read_text()))
    if case in baseline.CASES:
        cfg.update(baseline.CASES[case])
        return cfg,np.array(baseline.PULSES.get(case,[0,0,0])),.35,.18
    extra,force,offset,duration=(NEW|B|C|FINAL)[case]
    cfg.update(extra)
    return cfg,np.array(force),offset,duration

def build():
    OUT.mkdir(exist_ok=True,parents=True)
    from build_runtime import build as be
    be()
    sdk=Path(mj.__file__).parent
    flags=["-static"] if os.name=="nt" else ["-fPIC","-Wl,-rpath,"+str(sdk)]
    lib=sdk/"mujoco.dll" if os.name=="nt" else next(sdk.glob("libmujoco.so*"))
    subprocess.run([os.environ.get("CXX","g++"),"-shared","-std=c++17","-O2",*flags,
                    "-I"+str(sdk/"include"),str(C5/"read_zones.cpp"),str(lib),"-o",str(LIB)],check=True)

def seed(obj,case):
    return 481+2*list(dict.fromkeys([*baseline.CASES,*NEW,*B,*C,*FINAL])).index(case)+int(obj=="ball")

def run(pair):
    from trial import execute
    obj,case,mode=pair
    cfg,force,offset,duration=params(obj,case)
    tag=OUT/f"{obj}_{case}_{mode}"
    rec=baseline.Recorder()
    rec.library=LIB
    def pulse(d,r,t,times):
        elapsed=t-times[1]-offset
        d.xfrc_applied[r.body,:3]=force*(np.sin(np.pi*elapsed/duration)**2 if 0<=elapsed<=duration else 0.)
    controller=None
    if mode!="off":
        from reflex import TendonReflex
        controller=TendonReflex(rec,seed(obj,case),mode,obj)
    result=execute(obj,cfg,tag=str(tag),observer=rec,disturbance=pulse,reflex=controller)
    np.savez_compressed(str(tag)+"_sensors.npz",trace=rec.data[:rec.count],times=rec.times)
    if controller is not None:
        np.savez_compressed(str(tag)+"_reflex.npz",trace=np.asarray(controller.log))
    print(obj,case,mode,result["passed"],round(result["hold_span_mm"],2),flush=True)

if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("group",choices=["historical","A","B","C","D","focus","all"])
    p.add_argument("--mode",default="off")
    p.add_argument("--build",action="store_true")
    p.add_argument("--workers",type=int,default=5)
    args=p.parse_args()
    if args.build: build()
    groups={"historical":baseline.CASES,"A":NEW,"B":B,"C":C,"D":FINAL,"all":list(baseline.CASES)+list(NEW)+list(B)+list(C)+list(FINAL),
            "focus":["nominal","mu055","mu075","mu045","val_mu062_offset","push_nx3","val_push_diag","val_push_down","push_y6"]}
    with ProcessPoolExecutor(args.workers) as pool:
        list(pool.map(run,[(obj,case,args.mode) for case in groups[args.group] for obj in ("ball","cylinder")]))
