"""Screen grip candidates after the same tabletop acquisition; original test names drive forces only."""

from functools import lru_cache
from concurrent.futures import ProcessPoolExecutor
from control import *
from geometry import plant
from physics import integrate
from c9_screen import SPEC, Saved

BASE = 1.96
REF = dict(mm=[0.12, 3.33, 1.38, 0, 2.87], deg=-6.65, entry_mm=[0.6, 0, 0, 0, 0])
CASES = ["v_high", "v_side_h", "v_side_l", "v_return_side", "v_heavy_z2"]


def checkpoint(case):
    cfg, force, offset, duration, seed = prepare(case)
    pl = make_plan("ball", cfg, 0.002)
    rec = c7.Recorder()
    rec.library = c7.c6.LIB
    orig = rec.sample
    fb = c7.Feedback(rec, "window6", seed)

    def sample(m, d, r, k, t, ph):
        orig(m, d, r, k, t, ph)
        if t >= BASE:
            x = np.empty(mj.mj_stateSize(m, SPEC))
            mj.mj_getState(m, d, x, SPEC)
            np.savez(RESULT / f"early_{case}.npz", x=x, vel=list(r.last_velocity))
            raise Saved

    def ctrl(k, t, ph, ref, blocked):
        fb(k, t, ph, ref, blocked)
        if not blocked:
            ref[14] += 0.0006 * ease((t - 0.62) / 0.42) * (1 - ease((t - 1.65) / 0.52))

    rec.sample = sample
    try:
        execute("ball", cfg, observer=rec, reflex=ctrl, planner=lambda *args: pl)
    except Saved:
        pass


@lru_cache(6)
def env(case):
    cfg, force, offset, duration, seed = prepare(case)
    pl = make_plan("ball", cfg, 0.002)
    m, d, r, ty, _ = plant("ball", pl, cfg["dt"], c7.Recorder().configure)
    z = np.load(RESULT / "aim108" / (case + ".npz"))
    t = np.arange(BASE, 7.42, 0.006)
    cmd = np.column_stack(
        [np.interp(t, z["commands_time"], z["command"][:, i]) for i in range(20)]
    )
    # After a failed trial C8 could hold the ring; for the screening window use
    # the same healthy, measured ring tension reference for all trials.
    cmd[:, 14:20] -= grip_offsets(t, REF)
    phase = (t + 0.003 - 3.2 - offset) / duration
    return (
        m,
        d,
        r,
        np.load(RESULT / f"early_{case}.npz"),
        t,
        cmd,
        np.sin(np.pi * np.clip(phase, 0, 1))[:, None] ** 2 * force,
    )


def simulate(case, p):
    m, d, r, s, t, cmd, force = env(case)
    mj.mj_setState(m, d, s["x"], SPEC)
    mj.mj_forward(m, d)
    r.last_velocity[:] = s["vel"]
    refs = cmd.copy()
    refs[:, 14:20] += grip_offsets(t, p)
    refs[:, 19] = np.clip(refs[:, 19], -0.571595, 0.014563)
    sense = np.array([7.5])
    buf = np.empty((24, 44))
    worst = 0.0
    breaks = 0.0
    for k, ref in enumerate(refs):
        d.xfrc_applied[r.body, :3] = force[k]
        integrate(m, d, r, ref, 24, sense, buf)
        center = buf[-1, 18:21] * 1000
        if np.linalg.norm(center) > 100.0 or (buf[-1, 6] > 0.05 and t[k] > 3.3):
            return 100 + 4 * (7.42 - t[k])
        if 3.2 < t[k] < 5.85:
            worst = max(worst, np.linalg.norm(center - [-10, -20, -54]))
            breaks += np.count_nonzero(buf[:, 0:6].sum(1) < 0.001) * 0.00025
    return float(np.linalg.norm(center - [-13, -18, -64]) + 0.14 * worst + 24 * breaks)


def assess(job):
    i, p = job
    v = []
    for case in CASES:
        v.append(round(simulate(case, p), 3))
        if v[-1] > 100:
            break
    return dict(
        idx=i, params=p, scores=v, rank=max(v) + 0.3 * np.mean(v) + (5 - len(v)) * 38
    )


if __name__ == "__main__":
    with ProcessPoolExecutor(5) as pool:
        list(pool.map(checkpoint, CASES))
    rng = np.random.default_rng(93178)
    jobs = []
    for i in range(300):
        if i < 130:
            delta = np.array(REF["mm"]) + rng.uniform(
                [-0.9, -2.0, -1.55, 0, -1.4], [1.05, 3.4, 1.8, 0, 2.5]
            )
            angle = rng.uniform(-9.0, -3.4)
        else:
            delta = rng.uniform([-3.0, -0.5, -1.5, 0, -1.2], [3, 6.5, 4, 0, 6.5])
            angle = rng.uniform(-13.0, 3.0)
        jobs.append(
            (
                i,
                dict(
                    mm=delta.round(2).tolist(),
                    deg=round(angle, 2),
                    entry_mm=REF["entry_mm"],
                    register=True,
                ),
            )
        )
    with ProcessPoolExecutor(7) as pool:
        rows = []
        for row in pool.map(assess, jobs):
            rows.append(row)
            if len(rows) % 10 == 0:
                (RESULT / "normalized.json").write_text(
                    json.dumps(sorted(rows, key=lambda a: a["rank"]), indent=1)
                )
