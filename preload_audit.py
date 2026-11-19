"""Audit the measured release normals and read back independently recorded acceptance traces."""
import csv
import gzip
import json
from pathlib import Path

import numpy as np
from scipy.optimize import linprog

from preload_design import DIGITS, MU


def release_bound(row, mass_g):
    center=np.array([float(row[k]) for k in ("world_x_m","world_down_m","world_z_m")])
    forces=np.array([float(row[n+"_N"]) for n in DIGITS])
    normals=[]
    for n in DIGITS:
        delta=center-np.array([float(row[f"{n}_pad_{k}_m"]) for k in "xyz"])
        normals.append(delta/np.linalg.norm(delta))
    normals=np.array(normals)
    # Fixed normal magnitudes. Bound all tangents by exact circular Coulomb disks.
    required=np.array([0.,-mass_g*.001*9.80665,0.])-forces@normals
    length=np.linalg.norm(required)
    direction=required/length if length else np.zeros(3)
    capacity=MU*forces@np.sqrt(np.clip(1-(normals@direction)**2,0.,1.))
    # Outer 32-gon also tests the full force and zero-moment conditions.
    cols=[]
    for n in normals:
        u=np.cross(n,np.eye(3)[np.argmin(abs(n))])
        u/=np.linalg.norm(u)
        v=np.cross(n,u)
        for phi in np.arange(32)*2*np.pi/32:
            f=n+MU/np.cos(np.pi/32)*(np.cos(phi)*u+np.sin(phi)*v)
            cols.append(np.r_[f,np.cross(-n,f)])
    eq=np.vstack((np.array(cols).T,np.repeat(np.eye(3),32,axis=1)))
    need=np.r_[0.,-mass_g*.001*9.80665,0.,0.,0.,0.,forces]
    solution=linprog(np.zeros(96),A_eq=eq,b_eq=need,bounds=(0,None),method="highs")
    if solution.status not in (0,2):
        raise RuntimeError(solution.message)
    return dict(required_tangential_projection_N=length,max_available_projection_N=capacity,
                outward_relaxed_wrench_feasible=bool(solution.success),world_inward_normals=normals.tolist())


def audit(folder):
    folder=Path(folder)
    records=[]
    for file in sorted(folder.glob("*.csv.gz")):
        with gzip.open(file,"rt",encoding="utf-8",newline="") as source:
            rows=list(csv.DictReader(source))
        name,stage,mass,dt=file.name.split(".csv")[0].split("-")
        mass=int(mass.removesuffix("g"))
        log=folder/(stage+("150" if mass==150 else "")+".jsonl")
        recorded=(json.loads(line) for line in log.read_text().splitlines())
        nominal=next(r for r in recorded if r["tag"]==name and r["dt"]==float(dt))
        def invalid(row):
            return ([n for n in DIGITS if float(row[n+"_N"])<=.01]+
                    [n for n in ("ring_N","hard_N","plane_N") if float(row[n])>(.001 if n=="ring_N" else 0.)])
        failed=next((r for r in rows if invalid(r)),None)
        when=float(failed["t_s"]) if failed else None
        assert when==nominal["first_invalid_s"] and len(rows)==nominal["samples"]
        hard=next((r for r in rows if float(r["hard_N"])>0.),None)
        records.append(dict(tag=name,stage=stage,dt=float(dt),mass_g=mass,
                            first_invalid_s=when,failed_channels=invalid(failed) if failed else [],
                            first_hardware_s=float(hard["t_s"]) if hard else None,
                            end_hardware_N=float(rows[-1]["hard_N"]),end_floor_N=float(rows[-1]["plane_N"]),
                            end_parts=rows[-1]["ball_hardware"],**release_bound(rows[0],mass)))
    return records

if __name__=="__main__":
    import sys
    print(json.dumps(audit(sys.argv[1]),indent=2))
