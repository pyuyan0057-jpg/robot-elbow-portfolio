"""Screen support-preserving finger configurations in free space before full-trial verification."""

import json, sys
from concurrent.futures import ProcessPoolExecutor
from placement import c7, np, OUT, make_plan, ease, Glove, execute
from geometry import plant, mj
from physics import integrate
from functools import lru_cache

NORMALS = [[0.18, -0.43, -0.885], [0.32, -0.55, -0.77], [0.03, -0.23, -0.973]]
SPEC = mj.mjtState.mjSTATE_INTEGRATION


class Saved(Exception):
    pass


def checkpoint(j):
    cfg, force, offset, duration, seed = c7.recipe("push_y3")
    cfg.update(normal=NORMALS[j], shape=[0] * 5)
    rec = c7.Recorder()
    rec.library = c7.c6.LIB
    fb = c7.Feedback(rec, "window6", seed)

    def pulse(d, r, t, ph):
        u = t - ph[1] - offset
        d.xfrc_applied[r.body, :3] = force * (
            np.sin(np.pi * u / duration) ** 2 if 0 <= u <= duration else 0.0
        )
        if t >= 6.24:
            saved.update(d=d, r=r)

    saved = {}
    oldsample = rec.sample

    def sample(m, d, r, k, t, ph):
        oldsample(m, d, r, k, t, ph)
        if t >= 6.24:
            saved["m"] = m

    rec.sample = sample

    def reflex(k, t, ph, ref, blocked):
        fb(k, t, ph, ref, blocked)
        if t < 6.24:
            return
        m, d, r = saved["m"], saved["d"], saved["r"]
        state = np.zeros(mj.mj_stateSize(m, SPEC))
        mj.mj_getState(m, d, state, SPEC)
        np.savez(
            OUT / f"checkpoint{j}.npz",
            state=state,
            ref=ref,
            velocity=list(r.last_velocity),
        )
        raise Saved

    try:
        execute(
            "ball",
            cfg,
            observer=rec,
            disturbance=pulse,
            reflex=reflex,
            planner=make_plan,
        )
    except Saved:
        return


@lru_cache(3)
def cached(j):
    cfg, *_ = c7.recipe("push_y3")
    cfg.update(normal=NORMALS[j], shape=[0] * 5)
    pl = make_plan("ball", cfg, 0.002)
    rec = c7.Recorder()
    m, d, r, ty, _ = plant("ball", pl, cfg["dt"], rec.configure)
    return m, d, r, Glove(m), np.load(OUT / f"checkpoint{j}.npz"), pl


def screen(job):
    j, delta, opp = job
    m, d, r, g, s, pl = cached(j)
    mj.mj_setState(m, d, s["state"], SPEC)
    mj.mj_forward(m, d)
    r.last_velocity[:] = s["velocity"]
    sense = np.array([7.5])
    buf = np.zeros((40, 44))
    base = s["ref"]
    margins = []
    speeds = []
    for k in range(155):
        t = 6.24 + k * 0.010
        ref = pl["tape"][round(t / 0.002)].copy()
        frac = ease((t - 6.25) / 0.85)
        ref[14:19] = np.maximum(0, base[14:19] + frac * np.array(delta) * 0.001)
        ref[19] = base[19] + frac * np.deg2rad(opp)
        integrate(m, d, r, ref, 40, sense, buf)
        if np.linalg.norm(d.xpos[r.body] - d.xpos[r.palm]) > 0.13:
            return None
        if k >= 120 and k % 5 == 0:
            margins.append(1000 * (d.xpos[r.body, 1] + 0.02 - max(g.world(d)[:, 1])))
            speeds.append(1000 * np.linalg.norm(d.qvel[r.ballv : r.ballv + 3]))
            if buf[-1, :5].sum() < 0.06:
                return None
    return dict(
        j=j,
        shape=delta,
        opp=opp,
        margin=round(min(margins), 2),
        speed=round(max(speeds), 2),
    )


if __name__ == "__main__":
    for j in range(3):
        checkpoint(j)
    rng = np.random.default_rng(798)
    jobs = []
    for k in range(int(sys.argv[1]) if len(sys.argv) > 1 else 150):
        delta = rng.uniform([-10, -5, -6, 0, -9], [4, 2, 3, 0, 6]).round(2).tolist()
        jobs.append((k % 3, delta, round(rng.uniform(-8, 8), 2)))
    with ProcessPoolExecutor(7) as pool:
        results = [v for v in pool.map(screen, jobs, chunksize=4) if v]
    results.sort(key=lambda v: -v["margin"])
    (OUT / "screen.json").write_text(json.dumps(results, indent=1))
    print(results[:10])
