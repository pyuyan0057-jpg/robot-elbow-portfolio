"""Reach a candidate grip, copy its free state, and independently push along each world axis.
Development screen uses a shortened open-loop replay only; final trials run closed-loop from rest.
"""

import sys, json
from concurrent.futures import ProcessPoolExecutor
from normalized_screen import (
    env,
    SPEC,
    integrate,
    mj,
    np,
    RESULT,
    grip_offsets,
    REF,
    CASES,
)
from control import ease

AXES = np.r_[np.eye(3), -np.eye(3)]


def investigate(job):
    i, p = job
    case = "v_side_h"
    m, d, r, s, t, cmd, _ = env(case)
    mj.mj_setState(m, d, s["x"], SPEC)
    mj.mj_forward(m, d)
    r.last_velocity[:] = s["vel"]
    u = grip_offsets(t, p)
    refs = cmd.copy()
    refs[:, 14:20] += u
    refs[:, 19] = np.clip(refs[:, 19], -0.571595, 0.014563)
    sense = np.array([7.5])
    buf = np.empty((24, 44))
    end = np.abs(t - 3.40).argmin()
    for k in range(end + 1):
        integrate(m, d, r, refs[k], 24, sense, buf)
        if np.linalg.norm(buf[-1, 18:21]) > 0.11:
            return dict(idx=i, params=p, rank=999, axes=[])
    snapshot = np.empty(mj.mj_stateSize(m, SPEC))
    mj.mj_getState(m, d, snapshot, SPEC)
    vel = list(r.last_velocity)
    sensor = sense.copy()
    ref = refs[end].copy()
    ref[7:14] = 0.0
    origin = buf[-1, 18:21].copy()
    scores = []
    wave = np.sin(np.pi * np.clip(np.arange(102) * 0.006 / 0.18, 0.0, 1.0)) ** 2
    for axis in AXES:
        mj.mj_setState(m, d, snapshot, SPEC)
        mj.mj_forward(m, d)
        r.last_velocity[:] = vel
        sense[:] = sensor
        worst = 0.0
        gap = 0
        for w in wave:
            d.xfrc_applied[r.body, :3] = axis * 5.0 * w
            integrate(m, d, r, ref, 24, sense, buf)
            dist = np.linalg.norm(buf[-1, 18:21] - origin) * 1000
            worst = max(worst, dist)
            gap += np.count_nonzero(buf[:, :6].sum(1) < 0.001)
            if dist > 85:
                break
        scores.append(round(min(100.0, worst + 0.01 * gap), 2))
    return dict(
        idx=i, params=p, axes=scores, rank=round(max(scores) + 0.9 * np.mean(scores), 4)
    )


if __name__ == "__main__":
    rng = np.random.default_rng(811439)
    jobs = []
    seed = json.loads((RESULT / "norm240.json").read_text())
    for i in range(800):
        if i == 0:
            p = seed.copy()
        else:
            if i < 270:
                mm = np.array(seed["mm"]) + rng.uniform(
                    [-2, -2, -2, 0, -3], [2, 4, 3, 0, 2.5]
                )
                ang = rng.uniform(-8, 2.7)
            else:
                mm = rng.uniform([-5, -2, -5, 0, -5], [7, 10, 7, 0, 10])
                ang = rng.uniform(-20, 2.7)
            p = dict(
                mm=mm.round(2).tolist(),
                deg=round(ang, 2),
                entry_mm=[0.6, 0, 0, 0, 0],
                register=True,
            )
        jobs.append((i, p))
    rows = []
    with ProcessPoolExecutor(7) as pool:
        for row in pool.map(investigate, jobs, chunksize=2):
            rows.append(row)
            if len(rows) % 20 == 0:
                (RESULT / "axis_sweep.json").write_text(
                    json.dumps(sorted(rows, key=lambda a: a["rank"]), indent=1)
                )
