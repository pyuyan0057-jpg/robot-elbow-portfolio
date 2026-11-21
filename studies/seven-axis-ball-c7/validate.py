from study import batch,TEST,SELECTED,c6,OUT
import sys
if __name__=="__main__":
    if sys.argv[1]=="new":
        jobs=[(case,m) for case in TEST for m in ("off","both","preload_only",SELECTED)]
    else:
        cases=[*c6.baseline.CASES,*c6.NEW,*c6.B,*c6.C,*c6.FINAL]
        jobs=[(case,SELECTED) for case in cases if not (OUT/f"{case}_{SELECTED}_data.npz").exists()]
    batch(jobs,8)
