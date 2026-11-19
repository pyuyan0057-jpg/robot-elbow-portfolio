"""R5 shoulder cable reach screen. Distances are in mm. No wire is installed in the model."""
import csv
import math
from pathlib import Path

# Station coordinates in the down pose, in the R4 assembly frame.
# A is a proposed body-side feed-through within the flange opening.
# B uses the outboard (positive-X) face centre of the S2 plug keep-out.
# C uses the centre of the +X,+Z upper-arm M6 nut bounding box as a clamp reference.
ORIGIN = (0, 360, 0)
A = (0, 382, -179)
B = (140.9, 338, 0)
C = (55, 148.6, 55)

def offset(point):
    return tuple(a-b for a,b in zip(point,ORIGIN))

def rotate_z(point,deg):
    x,y,z=point
    c,s=math.cos(math.radians(deg)),math.sin(math.radians(deg))
    return (c*x-s*y,s*x+c*y,z)

def rotate_x(point,deg):
    x,y,z=point
    c,s=math.cos(math.radians(deg)),math.sin(math.radians(deg))
    return (x,c*y-s*z,s*y+c*z)

def spans(s1,s2):
    a,b,c=map(offset,(A,B,C))
    moving_b=rotate_z(b,s1)
    moving_c=rotate_z(rotate_x(c,-s2),s1)
    return math.dist(a,moving_b),math.dist(moving_b,moving_c)

def main():
    rows=[]
    for s1 in range(-90,91):
        for s2 in range(91):
            rows.append((s1,s2,*spans(s1,s2)))
    # Each adjacent body pair has only one relative rotation. Its squared chord
    # distance is k + p*cos(t) + q*sin(t); check boundaries and stationary angles.
    def extrema(fn,lo,hi):
        def squared(t): return fn(t)**2
        k=(squared(0)+squared(180))/2
        p=squared(0)-k
        q=squared(90)-k
        theta=math.degrees(math.atan2(q,p))
        candidates=[lo,hi]+[theta+i*180 for i in range(-3,4) if lo<=theta+i*180<=hi]
        return min((fn(t),t) for t in candidates),max((fn(t),t) for t in candidates)
    for name,fn,lo,hi in [('A-B',lambda t: spans(t,0)[0],-90,90),('B-C',lambda t: spans(0,t)[1],0,90)]:
        e=extrema(fn,lo,hi)
        print(name,'neutral',fn(0),'min,max',e,'max-neutral',e[1][0]-fn(0))
    with (Path(__file__).resolve().parent/'cable-chords.csv').open('w',newline='',encoding='utf-8') as out:
        writer=csv.writer(out)
        writer.writerow(('s1_deg','s2_deg','A_to_B_mm','B_to_C_mm'))
        for a,b,d,e in rows:
            if a%5==0 and b%5==0:writer.writerow((a,b,round(d,4),round(e,4)))

if __name__ == '__main__':
    main()
