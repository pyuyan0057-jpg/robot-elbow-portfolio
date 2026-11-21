"""Compare the frozen actuator policy to the same 88 open-loop simulations."""
from experiment import baseline,NEW,B,C,FINAL,OUT,ProcessPoolExecutor,run

def policy(obj):
    return "rapid" if obj=="cylinder" else "firm"

if __name__=="__main__":
    jobs=[]
    for c in [*baseline.CASES,*NEW,*B,*C,*FINAL]:
        for obj in ("cylinder","ball"):
            mode=policy(obj)
            if not (OUT/f"{obj}_{c}_{mode}_reflex.npz").is_file():
                jobs.append((obj,c,mode))
    with ProcessPoolExecutor(8) as pool:
        list(pool.map(run,jobs))
