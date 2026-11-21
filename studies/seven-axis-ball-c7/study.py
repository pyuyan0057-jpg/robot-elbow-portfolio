"""Matched MuJoCo experiments on the C6 ball; time in s and extra tendon travel in mm."""
import argparse
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import sys
import json
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/"seven-axis-reflex-c6"))
import experiment as c6
import numpy as np
from detector import Readout, Detector, DT

OUT=HERE.parents[1]/"outputs"/"ball-c7"
REF="push_y3"
NAMES=("off","index","thumb","both","fixed_index","fixed_thumb")
# (index, thumb, earliest pressure gain above a trailing per-pad baseline in N,
# speed mm/s, maximum time at elevated travel in s).
# (index stroke, thumb stroke, normal-load transient, spool rate, pulse duration)
TUNES={
 "short_i":(.35,0,.40,12.,.10), "short_t":(0,.35,.40,12.,.10),
 "short_pair":(.30,.15,.40,12.,.10), "balanced":(.55,.35,.40,18.,.14),
 "early_index":(.85,.15,.23,22.,.18), "early_thumb":(.15,.85,.23,22.,.18),
 "early_pair":(.80,.80,.23,22.,.18), "early_hold":(.80,.80,.23,22.,.38),
 "early_firm":(1.30,1.30,.23,30.,.16), "late":(.35,.20,1.0,10.,.12)
}

# First pair is preload in mm; second pair is transient added travel (index, thumb).
PRE={
 "pre_i":((.65,0),(.20,0)), "pre_t":((0,.65),(0,.20)),
 "pre_i1":((1.15,.18),(.20,.10)), "pre_t1":((.18,1.15),(.10,.20)),
 "pre_half":((.45,.45),(.18,.12)), "pre_pair":((.85,.85),(.18,.12)),
 "pre_pair1":((1.25,1.25),(.20,.15)), "pre_t2":((.70,1.45),(.20,.10)),
 "pre_i2":((1.45,.70),(.20,.10)), "pre_soft":((.2,.2),(.16,.10)),
 "pre_full":((1.6,1.6),(.20,.10)), "pre_asym":((1.70,1.10),(.20,.10)),
 "pre_lead":((1.85,1.05),(.15,.15)), "pre_lead_t":((1.85,1.45),(.15,.15)),
 "pre_high":((1.95,1.25),(.05,.20)), "pre_high_t":((1.95,1.75),(.05,.15)),
 "pre_equal":((1.85,1.85),(.15,.15)), "pre_high_i":((1.95,.8),(.05,.25)),
}

# A longer corrective pulse retains stroke through the peak disturbance.
LONG={
 "reserve_i":((1.35,.55),(.65,.80)), "reserve_pair":((1.2,1.),(.8,.8)),
 "reserve_bias":((1.6,.85),(.4,1.05)), "reserve_t":((1.65,.65),(.35,1.35)),
 "reserve_soft":((1.4,.8),(.40,.55)), "reserve_small":((1.0,.7),(.55,.65)),
 "reserve_even":((.9,.9),(.95,.95)), "reserve_fast":((1.25,.55),(.75,1.30))
}
LONG.update({
 "mid":((1.38,.68),(.52,.68)), "mid_i":((1.45,.70),(.50,.70)),
 "mid_t":((1.35,.80),(.55,.7)), "mid_stiff":((1.60,1.0),(.30,.6)),
 "mid_late":((1.50,.90),(.45,.70)), "mid_light":((1.45,.72),(.48,.62))
})
PRE.update(LONG)
SELECTED="mid_i"
PRE["preload_only"]=(PRE[SELECTED][0],(0.,0.))

class Recorder(c6.baseline.Recorder):
    def sample(self,m,d,r,k,t,times):
        super().sample(m,d,r,k,t,times)
        if k==0:
            self.directions=np.zeros((len(self.data),6))
        palm=d.xmat[r.palm].reshape(3,3)
        across=d.site_xpos[self.sites[1]]-d.site_xpos[self.sites[0]]
        self.directions[k]=np.r_[palm.T@(across/np.linalg.norm(across)),palm.T@np.array([0.,1.,0.])]

class Feedback:
    def __init__(self,rec,mode,seed,tape=()):
        self.rec=rec
        self.mode=mode
        self.seed=seed
        self.fixed=set(tape)
        self.target=np.zeros(2)
        self.stroke=np.zeros(2)
        self.det=Detector("ball")
        self.log=[]
        self.preloaded=False
        self.support_ticks=0
        self.baseline=None
        self.last=-10.
        self.until=-10.
        self.armed=False

    def step_policy(self,t,ph,y,alarm):
        if self.mode in PRE:
            return self.preload(t,ph,y)
        if self.mode not in TUNES:
            pair={"off":(0,0),"index":(.65,0),"thumb":(0,.65),"both":(.65,.65),
                  "fixed_index":(.65,0),"fixed_thumb":(0,.65)}[self.mode]
            trigger=round(t/DT) in self.fixed if self.mode.startswith("fixed_") else alarm
            if trigger and max(y[:,0])>=.06:
                self.target=np.minimum(2*np.array(pair),self.target+pair)
            return 10.,trigger
        return self.tuned(t,ph,y,alarm)

    def __call__(self,k,t,ph,ref,blocked):
        if k==0:
            self.read=Readout(int(np.ceil(ph[-1]/DT))+1,self.seed)
        y=self.read.step(k,self.rec.data[k,1:7].reshape(2,3))
        alarm=self.det.step(t,ph,y)
        rate,trigger=self.step_policy(t,ph,y,alarm)
        if t>=ph[3]-.55: self.target[:]=0
        if blocked:
            self.stroke[:]=0
            self.target[:]=0
        self.stroke+=np.clip(self.target-self.stroke,-rate*DT,rate*DT)
        ref[14:16]+=self.stroke*.001
        self.log.append([t,alarm,trigger,*self.stroke,*y.ravel()])

    def preload(self,t,ph,y):
        start,extra=PRE[self.mode]
        load=y[:,0]
        if self.baseline is None: self.baseline=load.copy()
        growth=load-self.baseline
        self.baseline+=(1-np.exp(-DT/.14))*growth
        if ph[0]<=t<ph[1] and min(load)>=.10:
            self.support_ticks+=1
        else:
            self.support_ticks=0
        if self.support_ticks>=15: self.preloaded=True
        trigger=False
        self.target[:]=start if self.preloaded else (0.,0.)
        if not self.preloaded:return 5.,False
        pulse=growth[0]>.4 or (max(y[:,1])>18 and max(growth)>.08 and max(y[:,2])>.12)
        if not pulse and max(growth)<.16:self.armed=True
        if ph[1]+.05<=t<ph[3]-.55 and pulse and self.armed and t-self.last>.34:
            self.until=t+(.18 if self.mode in LONG else .10)
            self.last=t
            self.armed=False
            trigger=True
        if t<self.until:self.target+=extra
        return ((22. if self.mode in LONG else 12.) if t>=ph[1] else 5.),trigger

    def tuned(self,t,ph,y,alarm):
        index,thumb,threshold,rate,width=TUNES[self.mode]
        load=y[:,0]
        if self.baseline is None: self.baseline=load.copy()
        growth=load-self.baseline
        self.baseline+=(1-np.exp(-DT/.14))*growth
        # A previously supported contact develops a rising compressive load, or local
        # slip becomes pronounced. A quiet unloading/rebound cannot start a fresh pulse.
        # Trigger uses measured pads only; the disturbance schedule is unavailable here.
        pulse=(growth[0]>threshold and self.baseline[0]>.18) or (
            max(y[:,1])>18 and max(growth)>.08 and max(y[:,2])>.12)
        allowed=ph[1]+.05<=t<ph[3]-.55
        if not pulse and max(growth)<threshold*.4:
            self.armed=True
        trigger=bool(allowed and pulse and self.armed and t-self.last>.34)
        if trigger:
            self.target[:]=(index,thumb)
            self.until=t+width
            self.last=t
            self.armed=False
        if t>=self.until: self.target[:]=0
        return rate,trigger

# Independent test suite. Off, previous two-finger control and the frozen candidate
# share one noise realization per case. Forces are world XYZ; +Y is downward.
TEST={
 "quiet_shift":([-.4,0,.8],[0,0,0],.29,.20),
 "quiet_mu":([.8,0,-.6],[0,0,0],.45,.16),
 "down_24":([-.3,0,.55],[.08,2.4,-.12],.44,.154),
 "down_30":([.27,0,.42],[-.08,3.0,.11],.57,.193),
 "down_34":([-.42,0,-.35],[.12,3.4,.06],.31,.198),
 "down_38":([.55,0,-.25],[-.09,3.8,.03],.48,.171),
 "down_42":([.15,0,-.65],[.18,4.2,-.10],.52,.184),
 "down_46":([-.52,0,.7],[-.22,4.6,.18],.395,.176),
 "side_22":([.15,0,.3],[-2.2,.25,.1],.47,.168),
 "side_31":([-.35,0,.28],[-3.1,.3,-.15],.40,.191),
 "return_down":([.48,0,-.3],[-.12,3.7,.25],1.49,.16),
 "return_side":([-.25,0,.35],[-2.9,.4,.2],1.57,.197)
}

def recipe(case):
    if case in TEST:
        cfg,*_=c6.params("ball","nominal")
        shift,force,offset,duration=TEST[case]
        cfg["unknown_mm"]=shift
        if case=="quiet_mu":cfg["mu_override"]=.61
        return cfg,np.array(force),offset,duration,8107+list(TEST).index(case)
    return (*c6.params("ball",case),c6.seed("ball",case))

def solve(job):
    from trial import execute
    case,mode=job
    cfg,force,offset,duration,seed=recipe(case)
    rec=Recorder();rec.library=c6.LIB
    fixed=[]
    if mode.startswith("fixed_"):
        z=np.load(OUT/(case+"_both_data.npz"))
        fixed=np.flatnonzero(z["feedback"][:,1])
    fb=Feedback(rec,mode,seed,fixed)
    def pulse(d,r,t,ph):
        elapsed=t-ph[1]-offset
        d.xfrc_applied[r.body,:3]=force*(np.sin(np.pi*elapsed/duration)**2 if 0<=elapsed<=duration else 0.)
    tag=OUT/(case+"_"+mode)
    result=execute("ball",cfg,tag=str(tag),observer=rec,disturbance=pulse,reflex=fb)
    np.savez_compressed(str(tag)+"_data.npz",feedback=fb.log,trace=rec.data[:rec.count],
                        directions=rec.directions[:rec.count],times=rec.times)
    print(case,mode,result["passed"],round(result["hold_span_mm"],3),flush=True)

def batch(jobs,workers=6):
    OUT.mkdir(parents=True,exist_ok=True)
    with ProcessPoolExecutor(workers) as pool:
        list(pool.map(solve,jobs))

if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("--cases",nargs="+",default=[REF])
    p.add_argument("--modes",nargs="+",default=list(NAMES[:4]))
    p.add_argument("--build",action="store_true")
    p.add_argument("--workers",type=int,default=6)
    a=p.parse_args()
    if a.build:c6.build()
    batch([(c,m) for c in a.cases for m in a.modes],a.workers)
