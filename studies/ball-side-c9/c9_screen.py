"""Fast archived-command screening; selected policies are separately solved from rest with feedback."""

from functools import lru_cache
from concurrent.futures import ProcessPoolExecutor
import json, sys
from control import *
from geometry import plant, OUT
from physics import integrate

SPEC = mj.mjtState.mjSTATE_INTEGRATION
BASE = 1.96


class Saved(Exception):
    pass


def checkpoint(case):
    cfg, force, offset, duration, seed = prepare(case)
    pl = make_plan("ball", cfg, 0.002)
    rec = c7.Recorder()
    rec.library = c7.c6.LIB
    orig = rec.sample

    def sample(m, d, r, k, t, ph):
        orig(m, d, r, k, t, ph)
        if t >= BASE:
            state = np.empty(mj.mj_stateSize(m, SPEC))
            mj.mj_getState(m, d, state, SPEC)
            np.savez(
                RESULT / f"start_{case}.npz", state=state, vel=list(r.last_velocity)
            )
            raise Saved

    rec.sample = sample
    try:
        execute(
            "ball",
            cfg,
            observer=rec,
            reflex=c7.Feedback(rec, "window6", seed),
            planner=lambda *args: pl,
        )
    except Saved:
        pass


@lru_cache(8)
def env(case):
    cfg, force, offset, duration, seed = prepare(case)
    pl = make_plan("ball", cfg, 0.002)
    m, d, r, ty, _ = plant("ball", pl, cfg["dt"], c7.Recorder().configure)
    ar = np.load(OUT / (case + "_final.npz"))
    command = ar["command"]
    secs = ar["commands_time"]
    step = 0.008
    ts = np.arange(BASE, 7.65, step)
    refs = np.column_stack([np.interp(ts, secs, command[:, i]) for i in range(20)])
    u = (ts + 0.004 - 3.2 - offset) / duration
    weight = np.sin(np.pi * np.clip(u, 0, 1)) ** 2
    return (
        m,
        d,
        r,
        np.load(RESULT / f"start_{case}.npz"),
        ts,
        refs,
        weight[:, None] * force,
    )


def score(case, p):
    m, d, r, state, ts, raw, force = env(case)
    mj.mj_setState(m, d, state["state"], SPEC)
    mj.mj_forward(m, d)
    r.last_velocity[:] = state["vel"]
    refs = raw.copy()
    refs[:, 14:20] += grip_offsets(ts, p)
    refs[:, 19] = np.clip(refs[:, 19], -0.571595, 0.014563)
    count = 32
    buf = np.empty((count, 44))
    sense = np.array([7.5])
    start = None
    span = 0.0
    for k, ref in enumerate(refs):
        d.xfrc_applied[r.body, :3] = force[k]
        integrate(m, d, r, ref, count, sense, buf)
        b = buf[-1, 18:21] * 1000
        if np.linalg.norm(b) > 115:
            return 100.0 + (7.65 - ts[k]) * 4.0
        if ts[k] >= 3.26 and start is None:
            start = b.copy()
        if 3.26 < ts[k] < 5.85:
            span = max(span, np.linalg.norm(b - start))
    return float(np.linalg.norm(b - [-13.1, -18.0, -64.3]) + 0.3 * span)


def screen(job):
    idx, p = job
    s = []
    for case in ["v_side_h", "v_return_side", "v_three", "v_heavy_z", "v_quiet"]:
        v = score(case, p)
        s.append(round(v, 3))
        if v >= 100 and len(s) <= 2:
            break
    return dict(
        idx=idx,
        params=p,
        scores=s,
        rank=sum(s) / len(s) + max(s) * 0.35 + (5 - len(s)) * 32,
    )


if __name__ == "__main__":
    cases = ["v_side_h", "v_return_side", "v_three", "v_heavy_z", "v_quiet"]
    with ProcessPoolExecutor(5) as pool:
        list(pool.map(checkpoint, cases))
    rng = np.random.default_rng(93612)
    jobs = []
    for j in range(int(sys.argv[1]) if len(sys.argv) > 1 else 128):
        vals = rng.uniform([-0.8, 0.3, -2, 0, -3], [6, 6, 4, 0, 6]).round(2)
        p = dict(mm=vals.tolist(), deg=round(rng.uniform(-16, 3.5), 2))
        jobs.append((j, p))
    with ProcessPoolExecutor(8) as pool:
        rows = []
        for row in pool.map(screen, jobs, chunksize=1):
            rows.append(row)
            if len(rows) % 10 == 0 or len(rows) == len(jobs):
                best = sorted(rows, key=lambda r: r["rank"])
                (RESULT / "screen.json").write_text(json.dumps(best, indent=1))
                print(len(rows), best[0], flush=True)
