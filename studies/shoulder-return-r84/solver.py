"""R8.4 shoulder harness transient model. SI units except angles entered in degrees."""
from pathlib import Path
from math import cos, sin, tanh, exp, pi, radians
import csv
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from scipy.integrate import solve_ivp

DEST = Path(__file__).resolve().parents[2] / 'outputs' / 'shoulder-return-r84'
MOTIONS = {'full':(-90,90,90,0), 'forward':(0,0,90,90), 'back':(0,0,-90,90),
           'cross':(-90,0,90,90), 'reverse':(90,0,-90,90)}
COVER = 128.7 / 180
J = .00036065091147803336
FIELDS = ['t_s','S1_deg','S2_deg','distal_N','min_all_N','slack_mm','slider_mm',
          'stop_margin_mm','feed_abs_drag_N','feed_speed_mps','feed_low_N','feed_high_N']

def simulate(job, travel_s=4.0):
    name, motion, mu, extra, static, cycles, refined = job
    if not np.isfinite(travel_s) or travel_s <= 0:
        raise ValueError("travel_s must be a positive finite duration")
    period = 2*travel_s + 4
    far_end = 1 + travel_s
    return_start = far_end + 2
    return_end = return_start + travel_s
    a0,b0,a1,b1 = MOTIONS[motion]
    stiffness = 60000.
    force = 1.97*4.4482216153*.9 + .372*9.80665
    delta = -.005
    effective = COVER*mu + (1-COVER)*.25
    def evaluate(t, y, diagnostic=False):
        x,p,q,v,dx,dp,dq,dv = y
        tau = t % period
        if 1 < tau < far_end:
            h,rate = (1-cos(pi*(tau-1)/travel_s))/2,pi/(2*travel_s)*sin(pi*(tau-1)/travel_s)
        elif far_end <= tau < return_start:
            h,rate = 1,0
        elif return_start <= tau < return_end:
            h,rate = (1+cos(pi*(tau-return_start)/travel_s))/2,-pi/(2*travel_s)*sin(pi*(tau-return_start)/travel_s)
        else:
            h,rate = 0,0
        a,b = a0+(a1-a0)*h,b0+(b1-b0)*h
        l1,l2 = .08*radians(a+90),.08*radians(90-b)
        vl1,vl2 = .08*radians(a1-a0)*rate,-.08*radians(b1-b0)*rate
        stretch = (p-2*x-delta,l1/2+q-p,(l1+l2)/2+v-q,l2/2-v)
        speeds = (dp-2*dx,vl1/2+dq-dp,(vl1+vl2)/2+dv-dq,vl2/2-dv)
        T = [max(0.,stiffness*e+28*s) if e>0 else 0. for e,s in zip(stretch,speeds)]
        # Optional low-speed increase at the lined feed for reversal sensitivity
        rise = exp(-(dp/.005)**2) if static else 0.
        exponent = pi*(effective + COVER*mu*.5*rise)
        feed = (extra*(1+.3*rise)+tanh(exponent/2)*(T[0]+T[1]))*tanh(dp/.0002)
        if diagnostic:
            return [t,a,b,T[-1],min(T),1000*sum(max(0.,-e) for e in stretch),x*1000,
                    1000*min(x+.00445,.188495559+.00445-x),abs(feed),abs(dp),min(T[:2]),max(T[:2])]
        shaft_friction = tanh(.25*radians(135+a)/2)*(T[1]+T[2])
        fixed = name.startswith('baseline')
        rolling = .65+(shaft_friction if fixed else min(.06/.08,shaft_friction))
        second = .65+tanh(.25*radians(110-b)/2)*(T[2]+T[3])
        return (dx,dp,dq,dv,(2*T[0]-force-tanh(dx/.0002))/.667,
                (T[1]-T[0]-feed)/.05,
                (T[2]-T[1]-rolling*tanh(dq/.0002))/(.05+(0 if fixed else J/.08**2)),
                (T[3]-T[2]-second*tanh(dv/.0002))/.05)
    e = force/(2*stiffness)
    l1,l2 = .08*radians(a0+90),.08*radians(90-b0)
    v=l2/2-e
    q=(l1+l2)/2+v-e
    p=l1/2+q-e
    x=(p-delta-e)/2
    duration = period * cycles
    t=np.linspace(0, duration, int(np.ceil(duration*(800 if refined else 400)))+1)
    result=solve_ivp(evaluate,(0,t[-1]),[x,p,q,v,0,0,0,0],method='Radau' if refined else 'LSODA',
                     t_eval=t,rtol=2e-7 if refined else 1e-6,atol=1e-9 if refined else 3e-9,
                     max_step=.0125 if refined else .025)
    if not result.success: raise ValueError(result.message)
    z=np.array([evaluate(ti,yi,True) for ti,yi in zip(t,result.y.T)])
    if name.startswith(('trace','refine')):
        np.savetxt(DEST/(name+'.csv'),z,delimiter=',',header=','.join(FIELDS),comments='',fmt='%.9g')
    returning=(t%period >= return_start)&(t%period<=return_end)
    sliding=z[:,9]>.01
    return [name,motion,mu,effective,extra,int(static),cycles,z[:,3].min(),z[:,4].min(),z[:,5].max(),
            z[returning,5].max(),z[-1,5],z[:,7].min(),z[-1,6]-z[0,6],z[sliding,8].max() if any(sliding) else 0,
            t[z[:,5].argmax()],z[:,9].max()]
def main():
    import sys
    DEST.mkdir(parents=True, exist_ok=True)
    jobs=[]
    if '--boundary' in sys.argv:
        jobs=[('edge_'+motion+'_'+str(mu),motion,mu,.35,False,2,True) for motion in ('full','reverse') for mu in (.185,.19,.195,.2,.205)]
        jobs.extend([('baseline_R81','full',.25,.65,False,2,True),('rolling_only','full',.25,.65,False,2,True)])
        output='boundary.csv'
    elif '--refine' in sys.argv:
        jobs=[('refine_'+str(mu),'full',mu,.35,False,2,True) for mu in map(float,sys.argv[2:])]
        output='convergence.csv'
    else:
        for extra in (.10,.35,.65):
            for mu in (.035,.065,.09,.12,.15,.18,.21,.25):
                jobs.append(('sweep_'+str(extra)+'_'+str(mu),'full',mu,extra,False,2,False))
        for motion in MOTIONS:
            for mu in (.065,.15):
                jobs.append(('trace_'+motion+'_'+str(mu),motion,mu,.35,False,2,False))
            jobs.append(('breakaway_'+motion,motion,.065,.35,True,2,False))
        output='transients.csv'
    with (DEST/output).open('w',newline='') as f, ProcessPoolExecutor(max_workers=4) as pool:
        w=csv.writer(f)
        w.writerow(['case','motion','mu_liner','mu_eff','add_N','breakaway','cycles','min_distal_N','min_all_N',
                    'slack_max_mm','slack_return_mm','slack_end_mm','stop_margin_mm','residual_slider_mm',
                    'peak_feed_drag_above_10mmps_N','peak_slack_at_s','max_feed_speed_mps'])
        for row in pool.map(simulate,jobs):
            w.writerow(row)
            f.flush()
if __name__=='__main__':
    main()
