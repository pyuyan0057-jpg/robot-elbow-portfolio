"""Rate-limited two-tendon take-up; the upstream safety supervisor keeps priority."""
import numpy as np
from detector import Detector,Readout,DT

class TendonReflex:
    def __init__(self,recorder,seed,mode,obj):
        self.recorder=recorder
        self.detector=Detector(obj)
        self.seed=seed
        self.mode=mode
        self.stroke=np.zeros(2)
        self.target=np.zeros(2)
        self.log=[]
        self.readout=None
        self.obj=obj
    def __call__(self,k,t,times,ref,blocked):
        if k==0:
            n=int(np.ceil(times[-1]/DT))+1
            self.readout=Readout(n,self.seed)
        y=self.readout.step(k,self.recorder.data[k,1:7].reshape(2,3))
        hit=self.detector.step(t,times,y)
        if self.mode=="shadow":
            inc=np.zeros(2)
        else:
            # Index + thumb spool commands (mm). Each alarm adds one bounded take-up.
            # Values apply to the virtual actuators only.
            amount={"gentle":(.25,.25),"firm":(.65,.65),"rapid":(1.,1.),
                    "thumb_bias":(.38,1.15),"index_bias":(1.15,.38),"small":(.40,.40)}[self.mode]
            inc=np.array(amount)
        if hit and np.max(y[:,0])>=.06 and not blocked:
            self.target=np.minimum(self.target+inc,2*inc)
        if t>=times[3]-.55:
            self.target[:]=0.
        if blocked:
            self.target[:]=0.
            self.stroke[:]=0.
        rate=10. if self.mode in ("firm","gentle") else 22.
        self.stroke+=np.clip(self.target-self.stroke,-rate*DT,rate*DT)
        ref[14:16]+=self.stroke*.001
        self.log.append([t,float(hit),*self.stroke,*y.ravel()])
