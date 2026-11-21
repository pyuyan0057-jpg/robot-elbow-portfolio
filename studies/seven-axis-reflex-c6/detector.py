"""Streaming two-pad detector. Its inputs are two local pads, plan phases and the selected task."""
from collections import deque
import numpy as np
from scipy.signal import lfilter
DT=.002
ALPHA=1-np.exp(-DT/.006)

def noise_arrays(n,seed):
    rng=np.random.default_rng(seed)
    # Keep the original C5 channel noise realization when comparing the two transducers.
    load=rng.normal(0,.008,(n,6))[:,:2]
    speed=rng.normal(0,.6,(n,6))[:,:2]
    shear=rng.normal(0,.008,(n,2))
    return load,speed,shear

def inputs(raw,noise):
    load=np.maximum(0,raw[...,0]+noise[0])
    speed=np.maximum(0,raw[...,2]+noise[1])
    shear=np.maximum(0,raw[...,1]+noise[2])
    return np.stack([load,load*speed,shear],axis=-1)

def readout(filtered):
    load=filtered[...,0]
    speed=np.where(load>=.06,filtered[...,1]/np.maximum(.001,load),0.)
    return np.stack([load,speed,filtered[...,2]],axis=-1)

def replay(raw,seed):
    x=inputs(raw,noise_arrays(len(raw),seed))
    y=lfilter([ALPHA],[1,ALPHA-1],x,axis=0)
    return np.concatenate([np.zeros_like(y[:4]),readout(y[:-4])])

class Readout:
    def __init__(self,n,seed):
        self.noise=noise_arrays(n,seed)
        self.state=np.zeros((2,3))
        self.delay=deque(np.zeros((4,2,3)))
    def step(self,k,raw):
        x=inputs(raw,[a[k] for a in self.noise])
        self.state+=ALPHA*(x-self.state)
        self.delay.append(readout(self.state))
        return self.delay.popleft()

class Detector:
    def __init__(self,obj):
        self.obj=obj
        self.high=[0]*5
        self.low=[0]*5
        self.active=[False]*5
        self.peak_load=np.zeros(2)
        self.last=-10.
        self.last_fast=-10.
    def step(self,t,phase,y):
        if not phase[0]<=t<phase[3]-.55:
            return False
        load,speed,shear=y.T
        fast=max(speed)/24
        if self.obj=="ball":
            # Persistent lateral loading and creep on one pad can precede a large speed spike.
            slow=0.
        else:
            slow=max(speed)/12 if phase[0]<t<phase[1] and fast<1 else 0.
        self.peak_load=np.maximum(load,self.peak_load*np.exp(-DT/.30))
        # During force transfer, weak support or a receding contact can hide gross motion.
        # Keep separate edges for renewed motion, unloading and high shear.
        weak=unload=strong=0.
        if self.obj=="ball" and t>=phase[2]:
            if t-self.last_fast<.65 and load[0]<.06 and .06<load[1]<.4:
                weak=speed[1]/2.4
            if .065<t-self.last_fast<.5 and any((load>.10) & (load<.58*self.peak_load)):
                unload=max(speed)/8.
            strong=max(min(v/1.7,s/.33,n/.25) for n,v,s in y)
        flag=False
        for j,(ratio,dwell) in enumerate(((fast,10),(slow,17),(weak,3),(unload,5),(strong,5))):
            self.high[j]=self.high[j]+1 if ratio>=1 else 0
            self.low[j]=self.low[j]+1 if ratio<.7 else 0
            if not self.active[j] and self.high[j]>=dwell:
                self.active[j]=True
                if j==0: self.last_fast=t
                flag=True
            if self.active[j] and self.low[j]>=15:
                self.active[j]=False
        if flag and t-self.last>=.012:
            self.last=t
            return True
        return False
