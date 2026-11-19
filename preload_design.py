"""Inverse static design for a free 33 g sphere in the registered hanging posture."""
import csv
import json

import mujoco as mj
import numpy as np
from scipy.optimize import linprog

from free_ball import A, H, RC, PALM, build
from inverse_release import DATA

DIGITS = ("index", "thumb", "middle")
K_MIN, K_MAX = .002, .12  # Nm/rad, spring design search bounds
MU = .28


def inverse(normal_min, reserve=0., mu=MU):
    hand = json.loads(DATA.read_text())["hand"]
    with DATA.with_name("target-contacts.csv").open(encoding="utf-8-sig") as stream:
        targets = list(csv.DictReader(stream))
    model = build(hand, .00025, MU, .003, ball_mass=.033)
    state = mj.MjData(model)
    jids, angles = [], []
    for i, entry in enumerate(hand["digits"][:3]):
        q = np.deg2rad(np.fromstring(targets[i]["flexion_deg"], sep=","))
        ids = [model.joint(f"q{i}{j}").id for j in range(len(q))]
        state.qpos[model.jnt_qposadr[ids]] = q - entry["reference_rad"]
        jids.append(model.jnt_dofadr[ids])
        angles.append(q)
    mj.mj_forward(model, state)
    center = PALM + hand["ball_in_palm"]
    kfaces = 32
    # Variables: 96 nonnegative friction rays, 3 tensions, 3 index springs, worst joint residual.
    size = 3*kfaces + 7
    ball = np.zeros((6,size))
    joints, loads, normals, directions = [], [], [], []
    cub = []
    for i,(q,ids) in enumerate(zip(angles,jids)):
        geom = model.geom("pad_"+DIGITS[i]).id
        delta = center-state.geom_xpos[geom]
        assert abs(np.linalg.norm(delta)-.0192)<2e-7
        n = delta/np.linalg.norm(delta)
        u = np.cross(n,np.eye(3)[np.argmin(np.abs(n))])
        u /= np.linalg.norm(u)
        v = np.cross(n,u)
        phi = np.arange(kfaces)*2*np.pi/kfaces
        rays = n + mu*(np.outer(np.cos(phi),u)+np.outer(np.sin(phi),v))
        directions.append(rays)
        sl = slice(i*kfaces,(i+1)*kfaces)
        ball[:3,sl] = rays.T
        ball[3:,sl] = np.cross(-n,rays).T  # moment divided by the sphere radius
        jac = np.zeros((3,model.nv))
        mj.mj_jac(model,state,jac,None,center-.010*n,model.geom_bodyid[geom])
        eq = np.zeros((len(q),size))
        eq[:,sl] = jac[:,ids].T@rays.T
        eq[:,96+i] = -(A*np.sin(q/2)+H*np.cos(q/2)-RC)
        bias = state.qfrc_bias[ids]+.002
        if i == 0:
            eq[:,99:102] = np.diag(q)
        else:
            bias = bias+np.asarray(hand["digits"][i]["spring_Nm_rad"])*q
        joints.append(eq)
        loads.extend(-bias)
        normal = np.zeros(size)
        normal[sl] = 1
        normals.append(normal)
        cub.extend([-normal,normal])
    joints = np.vstack(joints)
    loads = np.array(loads)
    # Index joints must balance exactly. Fit worst thumb/middle residual in N mm.
    inequalities = np.vstack((cub,joints[3:]*1000,-joints[3:]*1000))
    inequalities[6:,-1] = -1
    upper = np.r_[[-normal_min,2.2]*3,loads[3:]*1000,-loads[3:]*1000]
    eq = np.vstack((ball,joints[:3]*1000))
    rhs = np.r_[0,-.033*9.80665,0,0,0,0,loads[:3]*1000]
    bounds = [(0,None)]*96+[(0,25)]*3+[(K_MIN,K_MAX)]*3+[(0,None)]
    def optimize(cost):
        res = linprog(cost,A_eq=eq,b_eq=rhs,A_ub=inequalities,b_ub=upper,bounds=bounds,method="highs")
        if not res.success:
            raise ValueError(res.message)
        return res.x
    objective = np.zeros(size)
    objective[-1] = 1
    first = optimize(objective)
    bounds[-1] = (0,first[-1]+1e-6)
    # Choose minimum total pull; sweep the free index tension without changing its posture.
    objective[-1] = 0
    objective[96:99] = 1
    x = optimize(objective)
    if reserve:
        bounds[96] = (x[96]+reserve,x[96]+reserve)
        x = optimize(objective)
    stroke = [(sum(2*A*(1-np.cos(q/2))+2*H*np.sin(q/2)-RC*q)+x[96+i]/10000)*1000
              for i,q in enumerate(angles)]
    forces = [x[i*32:(i+1)*32]@directions[i] for i in range(3)]
    assert max(abs(ball@x-rhs[:6]))<1e-7
    return dict(normal_min_N=normal_min, reserve_index_N=reserve, design_mu=mu,
                spring_Nm_rad=x[99:102].tolist(), stroke_mm=stroke, tension_N=x[96:99].tolist(),
                target_normal_N=(np.array(normals)@x).tolist(), force_N=np.array(forces).tolist(),
                residual_Nmm=((joints@x-loads)*1000).tolist(), max_residual_Nmm=float(first[-1]))
