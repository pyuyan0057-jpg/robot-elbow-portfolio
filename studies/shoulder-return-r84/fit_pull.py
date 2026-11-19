"""Fit complete two-bend feed pull data. Input CSV columns:
direction,state,repeat,speed_mm_s,Tlow_N,Thigh_N
One group = identical direction, state, speed and independent repeat.
Supply startup maxima paired with simultaneous low-side tension for state 'start',
or synchronized steady sliding means for state 'sliding'. Use at least three loads.
"""
import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path
import numpy as np
COVER=128.7/180

def fit(lo, hi, bare=.25):
    lo,hi=np.array(lo,dtype=float),np.array(hi,dtype=float)
    if len(np.unique(lo))<3 or min(lo)<=0 or not np.isfinite(lo).all() or not np.isfinite(hi).all():
        raise ValueError("Three or more distinct positive low tensions and finite readings required")
    slope, intercept=np.linalg.lstsq(np.column_stack((lo,np.ones(len(lo)))),hi,rcond=None)[0]
    if slope<1 or intercept<0:
        raise ValueError("Unphysical fit. Check calibration, grouping and measurement uncertainty")
    mu=math.log(slope)/math.pi
    return (slope,intercept,mu,(mu-(1-COVER)*bare)/COVER,2*intercept/(1+slope),
            float(np.sqrt(np.mean((hi-slope*lo-intercept)**2))),min(lo),max(lo))

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('input',type=Path);ap.add_argument('output',type=Path)
    args=ap.parse_args()
    if args.input.resolve()==args.output.resolve():raise ValueError("Choose separate output")
    with args.input.open(encoding='utf-8-sig') as f:
        groups=defaultdict(list)
        for row in csv.DictReader(f):
            groups[tuple(row[k] for k in ('direction','state','repeat','speed_mm_s'))].append(row)
    if not groups:raise ValueError("No measurements")
    rows=[]
    for key,measurements in groups.items():
        rows.append([*key,*fit([r['Tlow_N'] for r in measurements],[r['Thigh_N'] for r in measurements])])
    with args.output.open('w',newline='') as f:
        w=csv.writer(f)
        w.writerow(['direction','state','repeat','speed_mm_s','slope','intercept_N','mu_effective',
                    'mu_liner_conditional_on_bare_025','add_N','RMSE_N','lowest_test_N','highest_test_N'])
        w.writerows(rows)
if __name__=='__main__':main()
