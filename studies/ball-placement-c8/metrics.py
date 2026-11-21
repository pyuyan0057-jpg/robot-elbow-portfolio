"""Independent audit of MuJoCo contacts, glove vertices and free sphere motion."""

from pathlib import Path
from geometry import np, mj, Glove, plant, plan
import json


def evaluate(tag):
    res = json.loads(tag.with_suffix(".json").read_text())
    cfg = res["case"]
    z = np.load(tag.with_suffix(".npz"))
    a = z["log"]
    t = a[:, 9]
    down = z["times"][2]
    near = t >= down
    load = a[:, 6]
    other = a[:, 8]
    finger = a[:, :5].sum(1)
    pl = plan("ball", cfg, 0.002)
    m, d, r, ty, _ = plant("ball", pl, cfg["dt"])
    g = Glove(m)

    def event(mask):
        ix = np.flatnonzero(near & mask)
        return int(ix[0]) if len(ix) else None

    i = event(load > 0.03)
    j = event(other > 0.03)
    if i is None:
        return dict(landed=False)
    p = a[:, [15, 17]]
    base = p[i]
    radial = np.linalg.norm(p[i:] - base, axis=1) * 1000
    trace = []
    for k, q in enumerate(z["q"]):
        sec = k * 0.04
        if sec < down:
            continue
        d.qpos[:] = q
        mj.mj_kinematics(m, d)
        w = g.world(d)
        n = np.argmax(w[:, 1])
        ball = ty - d.xpos[r.body, 1] - 0.02
        glove = ty - w[n, 1]
        trace.append((sec, ball * 1000, glove * 1000, 1000 * (glove - ball)))
    tr = np.array(trace)
    near_frame = tr[np.argmin(abs(tr[:, 0] - t[i]))]
    close = tr[(tr[:, 0] <= t[i]) & (tr[:, 1] >= 0) & (tr[:, 1] <= 15)]
    target = pl["place_target"][[0, 2]]
    measured = Path(str(tag) + "_clearance.npz")
    if measured.exists():
        g = np.load(measured)["trace"]
        around = g[(g[:, 0] >= t[i] - 0.004) & (g[:, 0] <= t[i] + 0.004)]
        if len(around):
            near_frame = around[np.argmin(abs(around[:, 0] - t[i]))]
        close = g[(g[:, 0] <= t[i] + 0.001) & (g[:, 1] >= -0.1) & (g[:, 1] <= 15)]
    return dict(
        landed=True,
        physical=res["passed"],
        retained=res["retained"],
        ball_touch_s=float(t[i]),
        hand_touch_s=float(t[j]) if j is not None else None,
        ball_first=bool(j is None or j > i),
        hand_peak_N=float(other[near].max()),
        min_finger_load_before_N=float(finger[near & (t <= t[i])].min()),
        touchdown_vertical_speed_mm_s=abs(a[max(0, i - 1), 25]) * 1000,
        touchdown_xz_error_mm=float(1000 * np.linalg.norm(base - target)),
        final_xz_error_vector_mm=((p[-1] - target) * 1000).tolist(),
        release_motion_mm=float(radial[-1]),
        release_peak_excursion_mm=float(radial.max()),
        final_error_mm=res["placement_error_mm"],
        near_margin_min_mm=float(min(close[:, 3])) if len(close) else None,
        nearest_frame_s=near_frame[0],
        nearest_glove_gap_mm=near_frame[2],
        final_table_N=float(load[-1]),
        quiet=res["released"],
        max_finger_while_seated_N=float(finger[-100:].max()),
    )
