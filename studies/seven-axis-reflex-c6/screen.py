"""Development-only threshold sweep on the forty archived scenarios."""
import numpy as np
from experiment import OUT,baseline,seed
from evaluate import audit,measure,match

def edges(v,enabled,threshold,dwell):
    raw=(v>=threshold)&enabled
    on=np.flatnonzero(np.diff(np.r_[0,raw,0].astype(int))==1)
    off=np.flatnonzero(np.diff(np.r_[0,raw,0].astype(int))==-1)
    rises=on[off-on>=dwell]+dwell-1
    low=(v<.7*threshold)|~enabled
    l0=np.flatnonzero(np.diff(np.r_[0,low,0].astype(int))==1)
    l1=np.flatnonzero(np.diff(np.r_[0,low,0].astype(int))==-1)
    resets=l0[l1-l0>=15]+14
    groups=np.searchsorted(resets,rises)
    return rises[np.r_[True,np.diff(groups)>0]] if len(rises) else rises

if __name__=="__main__":
    rows=[]
    for c in baseline.CASES:
        for o in ("cylinder","ball"):
            z=np.load(OUT/f"{o}_{c}_off_sensors.npz");a=z["trace"];times=z["times"]
            en,dr,ev=audit(a,times);v=measure(a[:,1:19].reshape(-1,6,3),seed(o,c))[:,:2].max(axis=1)
            rows.append((o,c,a[:,0],times,en,ev,v))
    for o in ("cylinder","ball"):
        results=[]
        for th in (2,3,4,5,6,8,10,12,16,20,24):
            for dw in (2,4,7,10):
                hits=fp=n=oldfp=0; miss=[]
                for obj,c,t,ph,en,ev,v in rows:
                    if obj!=o:continue
                    h,f=match(edges(v,en,th,dw),ev,t)
                    n+=len(ev);hits+=len(h);fp+=len(f)
                    if obj=="ball" and c in ("val_mu062_offset","val_mu040_offset","val_push_down"):
                        oldfp+=sum(abs(t[i]-4.78)<.14 for i in f)
                    miss.extend((c,round(t[a],3),round(v[max(a-50,0):b+1].max(),1)) for j,(a,b) in enumerate(ev) if j not in h)
                results.append((n-hits,fp,oldfp,th,dw,n,miss))
        print(o,sorted(results)[:3])
