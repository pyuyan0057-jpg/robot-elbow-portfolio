"""Run the final frozen controller, independent controls and time-step checks."""
import sys
from study import batch,TEST,FINAL_TEST,c6,OUT
FINAL_POLICY="window6"

if __name__=="__main__":
    group=sys.argv[1]
    if group=="blind":
        jobs=[(case,m) for case in FINAL_TEST for m in ("off","both","preload_only",FINAL_POLICY)]
    elif group=="archive":
        cases=[*c6.baseline.CASES,*c6.NEW,*c6.B,*c6.C,*c6.FINAL,*TEST]
        jobs=[(case,FINAL_POLICY) for case in cases if not (OUT/f"{case}_{FINAL_POLICY}_data.npz").is_file()]
    elif group=="timestep":
        jobs=[(case,m,.000125) for case in ("push_y3","val_push_down","v_heavy_z","v_return_side")
              for m in ("off","both",FINAL_POLICY)]
    elif group=="rescue_step":
        jobs=[("v_heavy_z2",m,dt) for dt in (.000125,.0001) for m in ("off","both","preload_only",FINAL_POLICY)]
    else:
        raise ValueError(group)
    batch(jobs,6)
