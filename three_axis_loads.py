"""Output-side loads for separate R3 elbow motions at the registered horizontal forearm pose."""
import json,math
from pathlib import Path

ROOT=Path(__file__).resolve().parent
G=9.80665
POINT=(.511214365,-.000413479,-.010955685)
def calculate(rows,payload,centered=False):
    rows=list(rows)+[dict(group="G3",mass_kg=payload,xyz_m=POINT,centroid_inertia_kgm2=[0]*3)]
    out={}
    # Joint Yaw axis is vertical, so its gravity drive torque is zero. It still carries a base moment.
    for name,groups,ax,origin,angle,duration in [
        ("yaw",{"G1","G2","G3"},1,(0,.101,0) if centered else (-.145,-.101,0),80,2.4),
        ("pitch",{"G2","G3"},2,(0,0,0),90,1.2),
        ("roll",{"G3"},0,(0,0,0),180,2.4)]:
        data=[r for r in rows if r["group"] in groups]
        M=sum(r["mass_kg"] for r in data)
        first=[sum(r["mass_kg"]*(r["xyz_m"][i]-origin[i]) for r in data) for i in range(3)]
        J=sum(r["centroid_inertia_kgm2"][ax]+r["mass_kg"]*sum((r["xyz_m"][i]-origin[i])**2 for i in range(3) if i!=ax) for r in data)
        def grav(q):
            if ax==1:return 0
            x,y,z=first
            if ax==2:return G*(x*math.cos(q)-y*math.sin(q))
            return -G*(y*math.sin(q)+z*math.cos(q))
        peak=hold=0
        for i in range(2401):
            u=i/2400
            # Palm-down to palm-up has negative roll; pitch goes from vertical down to horizontal.
            span=math.radians(-angle if ax==0 else angle)
            q=(-math.pi/2 if ax==2 else 0)+span*(10*u**3-15*u**4+6*u**5)
            acc=span*60*u*(1-u)*(1-2*u)/duration**2
            hold=max(hold,abs(grav(q)));peak=max(peak,abs(grav(q)+J*acc))
        out[name]=dict(moving_mass_kg=M,center_from_axis_origin_mm=[1000*x/M for x in first],inertia_kgm2=J,
                       horizontal_gravity_Nm=grav(0),maximum_static_Nm=hold,peak_sampled_Nm=peak,
                       design_peak_Nm=peak*1.5,peak_speed_rpm=angle/360*60*1.875/duration,
                       horizontal_overturning_Nm=G*math.hypot(first[0],first[2]) if ax==1 else None)
    return out

if __name__=="__main__":
    results={}
    for name,path in (("offset","rigid-groups.json"),("center","center/rigid-groups.json")):
        rows=json.loads((ROOT/"data/three-axis"/path).read_text())
        results[name]={str(g):calculate(rows,g/1000,name=="center") for g in (0,150)}
    print(json.dumps(results,indent=2))
