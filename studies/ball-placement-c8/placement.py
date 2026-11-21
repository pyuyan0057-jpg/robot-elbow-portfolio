"""C8 ball set-down. Compare all policies against identical C7 pickup and disturbance."""

import argparse, json
from concurrent.futures import ProcessPoolExecutor
from geometry import c7, np, OUT, Glove, hand, plan
from motion import ease
from scipy.optimize import least_squares
from trial import execute

# Approach geometry in palm coordinates (world gravity expressed in the hand).
NORMAL = np.array([0.08, -0.315, -0.946])
ANCHOR = np.array([-0.0096, -0.01987, -0.05345])
# Command adjustments (index,thumb,middle,ring,little) in mm at staging height.
SHAPES = {
    "same": [0, 0, 0, 0, 0],
    "curl": [4, 1, 1, 0, 3],
    "open_side": [-8, 0, 0, 0, -6],
    "side": [-12, 1, 1, 0, 3],
    "tip": [-4, -2, -2, 0, -3],
    "tuck": [6, 2, -1, 0, 6],
    "ease_thumb": [2, -3, 0, 0, 4],
    "relax": [-6, -3, 0, 0, 3],
}


def solve_pose(goal, seed, limits, n):
    def residual(x):
        p, r = hand(x)
        return np.r_[
            60 * (p + r.apply(ANCHOR) - goal), r.as_matrix()[1] - n, 0.012 * (x - seed)
        ]

    fit = least_squares(
        residual,
        np.clip(seed, limits[0] + 1e-7, limits[1] - 1e-7),
        bounds=limits,
        max_nfev=180,
    )
    if np.linalg.norm(residual(fit.x)[:3]) / 60 > 0.001:
        raise ValueError("target unreachable")
    return fit.x


def make_plan(o, c, period):
    pl = plan(o, c, period)
    old = pl["tape"]
    lt, high, down, *_ = pl["times"]
    pre, stage, seat, release, end = 6.7, 7.45, 9.40, 9.95, 14.9
    n = np.asarray(c.get("normal", NORMAL))
    n = n / np.linalg.norm(n)
    goal = pl["place_target"] + np.array(c.get("trim", [0, 0, 0])) * 0.001
    seed = old[round(6.2 / period), :7]

    def at(offset, seed):
        return solve_pose(goal + [0, offset * 0.001, 0], seed, pl["limits"], n)

    lower = at(c.get("depth", 2), seed)
    above = solve_pose(pl["place_target"] + [0, -0.032, 0], lower, pl["limits"], n)
    pl["per_mm"] = at(c.get("depth", 2) + 0.25, lower) - lower
    pl["per_mm"] *= 4
    basis = np.eye(3) * 0.0005
    center = goal + [0, c.get("depth", 2) * 0.001, 0]
    pl["lateral_mm"] = np.column_stack(
        [
            solve_pose(center + v, lower, pl["limits"], n)
            - solve_pose(center - v, lower, pl["limits"], n)
            for v in basis[[0, 2]]
        ]
    )
    retreat = at(-65, above)
    t = np.arange(round(end / period) + 1) * period
    tape = old[np.minimum(np.rint(t / period).astype(int), len(old) - 1)].copy()
    peak = old[round(down / period), :7]
    q = np.tile(peak, (len(t), 1))
    for start, finish, a, b in (
        (peak, above, down, pre),
        (above, lower, stage, seat),
        (lower, retreat, release + 1.45, release + 2.6),
    ):
        q += ease((t - a) / (b - a))[:, None] * (finish - start)
    vel = np.gradient(q, period, axis=0)
    after = t >= down
    tape[after, :7] = q[after]
    tape[after, 7:14] = vel[after]
    if np.any(
        abs(np.rad2deg(tape[:, 7:14])) > np.array([125, 125, 125, 140, 160, 110, 110])
    ) or np.any(abs(np.rad2deg(np.gradient(tape[:, 7:14], period, axis=0))) > 650):
        raise ValueError("motion limits exceeded")
    nominal = old[round(down / period), 14:20]
    tape[after, 14:20] = nominal
    delta = np.asarray(c["shape"]) * 0.001
    tape[after, 14:19] += ease((t[after] - pre + 0.45) / 0.85)[:, None] * delta
    tape[after, 19] += np.radians(c.get("opp", 0)) * ease(
        (t[after] - pre + 0.45) / 0.85
    )
    tape[after, 14:19] = np.maximum(0, tape[after, 14:19])
    tape[:, 14:19] *= 1 - ease((t - release) / 1.20)[:, None]
    pl.update(t=t, tape=tape, times=(lt, high, down, seat, release, end))
    return pl


def audit(tag):
    z = np.load(str(tag) + ".npz")
    a = z["log"]
    t = a[:, 9]
    down = z["times"][2]
    i = np.where((t > down) & (a[:, 6] > 0.03))[0]
    if not len(i):
        return {}
    i = i[0]
    j = np.where((t > down) & (a[:, 8] > 0.03))[0]
    window = (t > down) & (t <= t[i] + 0.002)
    return dict(
        ball_touch_s=round(t[i], 4),
        hand_touch_s=round(t[j[0]], 4) if len(j) else None,
        hand_before_N=round(float(max(a[window, 8])), 4),
        roll_mm=round(1000 * np.linalg.norm(a[-1, [15, 17]] - a[i, [15, 17]]), 3),
        contact_until_touch=float(np.min(a[window, :5].sum(1))),
        touchdown_speed_mm_s=round(abs(a[max(0, i - 1), 25]) * 1000, 2),
    )


class ClearanceRecorder(c7.Recorder):
    def sample(self, m, d, r, k, t, ph):
        super().sample(m, d, r, k, t, ph)
        if k == 0:
            self.geometry = Glove(m)
            self.clearance = []
        if t < ph[2]:
            return
        top = d.geom_xpos[r.table, 1] - m.geom_size[r.table, 1]
        ball_y = d.geom_xpos[r.ball, 1] + m.geom_size[r.ball, 0]
        if abs(top - ball_y) > 0.019:
            return
        g = self.geometry
        world_y = d.geom_xpos[g.ids, 1] + np.einsum(
            "ij,ij->i", d.geom_xmat[:, 3:6][g.ids], g.vertices
        )
        worst = int(np.argmax(world_y))
        self.clearance.append(
            (
                t,
                (top - ball_y) * 1000,
                (top - world_y[worst]) * 1000,
                (ball_y - world_y[worst]) * 1000,
                int(g.ids[worst]),
                self.data[k, 21],
            )
        )


def run(job):
    case, shape, *opts = job
    cfg, force, offset, duration, seed = c7.recipe(case)
    cfg.update(shape=SHAPES.get(shape, [0] * 5))
    if opts:
        cfg.update(opts[0])
    rec = ClearanceRecorder() if cfg.get("measure_full") else c7.Recorder()
    rec.library = c7.c6.LIB
    fb = c7.Feedback(rec, "window6", seed)

    def pulse(d, r, t, ph):
        u = t - ph[1] - offset
        d.xfrc_applied[r.body, :3] = force * (
            np.sin(np.pi * u / duration) ** 2 if 0 <= u <= duration else 0.0
        )

    tag = OUT / (case + "_" + cfg.get("label", shape))
    from support import SetDown

    pl = make_plan("ball", cfg, 0.002)
    gate = SetDown(rec, fb, cfg, pl) if cfg.get("gate") else fb
    res = execute(
        "ball",
        cfg,
        tag=str(tag),
        observer=rec,
        disturbance=pulse,
        reflex=gate,
        planner=lambda *args: pl,
    )
    if cfg.get("measure_full"):
        np.savez_compressed(
            str(tag) + "_clearance.npz", trace=np.array(rec.clearance).reshape(-1, 6)
        )
    met = audit(tag)
    met.update(
        error_mm=round(res["placement_error_mm"], 3),
        physical=res["passed"],
        retained=res["retained"],
    )
    if cfg.get("gate"):
        met.update(
            supported_s=gate.onset, opening_s=gate.release_time, timed_out=gate.latched
        )
    (OUT / (tag.name + "_audit.json")).write_text(json.dumps(met, indent=1))
    print(tag.name, met, flush=True)
    return met


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("shapes", nargs="*", default=list(SHAPES))
    p.add_argument("--cases", nargs="+", default=["push_y3"])
    p.add_argument("--workers", type=int, default=4)
    a = p.parse_args()
    OUT.mkdir(exist_ok=True, parents=True)
    with ProcessPoolExecutor(a.workers) as pool:
        list(pool.map(run, [(c, s) for c in a.cases for s in a.shapes]))
