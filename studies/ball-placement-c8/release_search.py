"""Replay from a real C8 support-transfer state to search release timing. Verify optima from t=0."""

from grip_screen import Saved, SPEC
from functools import lru_cache
from concurrent.futures import ProcessPoolExecutor
import json, sys
from placement import c7, np, OUT, make_plan, plan, ease
from geometry import plant, mj
from physics import integrate
from support import SetDown
from trial import execute


def release_state(cfg):
    base, force, offset, duration, seed = c7.recipe("push_y3")
    base.update(cfg)
    pl = make_plan("ball", base, 0.002)
    rec = c7.Recorder()
    rec.library = c7.c6.LIB
    gate = SetDown(rec, c7.Feedback(rec, "window6", seed), base, pl)
    store = {}

    def pulse(d, r, t, ph):
        u = t - ph[1] - offset
        d.xfrc_applied[r.body, :3] = force * (
            np.sin(np.pi * u / duration) ** 2 if 0 <= u <= duration else 0.0
        )
        store.update(d=d, r=r)

    orig = rec.sample

    def sample(m, *args):
        store.update(m=m)
        orig(m, *args)

    rec.sample = sample

    def step(k, t, ph, ref, blocked):
        gate(k, t, ph, ref, blocked)
        if gate.release_time is None:
            return
        m, r, d = store["m"], store["r"], store["d"]
        state = np.zeros(mj.mj_stateSize(m, SPEC))
        mj.mj_getState(m, d, state, SPEC)
        np.savez(
            OUT / "release_state.npz",
            state=state,
            ref=ref,
            velocity=list(r.last_velocity),
            lateral=pl["lateral_mm"],
            dest=pl["tape"][-1, :7],
            target=pl["place_target"],
            anchor=gate.anchor,
            fingers=gate.fingers,
        )
        raise Saved

    try:
        execute(
            "ball",
            base,
            observer=rec,
            disturbance=pulse,
            reflex=step,
            planner=lambda *x: pl,
        )
    except Saved:
        pass


@lru_cache(1)
def env():
    cfg, *_ = c7.recipe("push_y3")
    pl = plan("ball", cfg, 0.002)
    m, d, r, ty, _ = plant("ball", pl, cfg["dt"], c7.Recorder().configure)
    return m, d, r, ty, np.load(OUT / "release_state.npz")


def sweep(opts):
    m, d, r, ty, s = env()
    mj.mj_setState(m, d, s["state"], SPEC)
    mj.mj_forward(m, d)
    r.last_velocity[:] = s["velocity"]
    anc = s["anchor"]
    dst = s["dest"]
    f = s["fingers"]
    target = s["target"][[0, 2]]
    lateral = s["lateral"]
    v0 = d.xpos[r.body, [0, 2]].copy()
    sense = np.array([7.5])
    ref = s["ref"].copy()
    step = 0.002
    count = round(step / m.opt.timestep)
    buf = np.empty((count, 44))
    maxhand = 0.0
    lowest_load = 1e5
    maxrad = 0.0
    for k in range(1450):
        t = k * step
        u = np.clip((t - opts["retreat_start"]) / opts["retreat_s"], 0.0, 1.0)
        ref[:7] = anc + (dst - anc) * ease(u)
        ref[7:14] = (dst - anc) * 30 * u**2 * (1 - u) ** 2 / opts["retreat_s"]
        ref[14:19] = f[:5] * (
            1 - ease((t - np.array(opts["peel_delay"])) / opts["unwind_s"])
        )
        ref[19] = f[5]
        span = opts["slide_time"]
        w = np.clip(t / span, 0.0, 1.0)
        slide = lateral @ opts["slide_mm"]
        ref[:7] += slide * ease(w) * (1 - ease(u))
        ref[7:14] += slide * (
            30 * w**2 * (1 - w) ** 2 / span * (1 - ease(u))
            - ease(w) * 30 * u**2 * (1 - u) ** 2 / opts["retreat_s"]
        )
        integrate(m, d, r, ref, count, sense, buf)
        maxhand = max(maxhand, buf[:, 8].max())
        lowest_load = min(lowest_load, buf[:, 6].min())
        maxrad = max(maxrad, np.linalg.norm(buf[-1, [15, 17]] - v0) * 1000)
        if maxrad > 4.2 or lowest_load < 0.025 or maxhand > 0.03:
            return None
    return dict(
        opts=opts,
        hand=maxhand,
        load=lowest_load,
        slide=maxrad,
        error=np.linalg.norm(buf[-1, [15, 17]] - target) * 1000,
    )


if __name__ == "__main__":
    from pathlib import Path

    cfg = json.loads((Path(__file__).with_name("frozen.json")).read_text())
    release_state(cfg)
    rng = np.random.default_rng(9971)
    opts = []
    for i in range(int(sys.argv[1]) if len(sys.argv) > 1 else 160):
        delays = rng.uniform([0, 0, 0, 0, 0], [0.2, 0.44, 0.20, 0.12, 0.35])
        delays -= min(delays[1], delays[2], delays[4])
        delays = np.maximum(0, delays).round(3)
        opts.append(
            dict(
                peel_delay=delays.tolist(),
                unwind_s=round(rng.uniform(0.65, 2.2), 3),
                retreat_start=round(rng.uniform(0, 0.2), 3),
                retreat_s=round(rng.uniform(0.72, 1.5), 3),
                slide_mm=rng.uniform([-3, -2], [3, 6]).round(3).tolist(),
                slide_time=round(rng.uniform(0.32, 0.65), 3),
            )
        )
    with ProcessPoolExecutor(7) as pool:
        rows = [v for v in pool.map(sweep, opts, chunksize=5) if v]
    rows.sort(key=lambda v: (v["hand"] > 0.03, v["error"] + 0.5 * v["slide"]))
    (OUT / "release_search.json").write_text(json.dumps(rows, indent=1))
    print(rows[:5])
