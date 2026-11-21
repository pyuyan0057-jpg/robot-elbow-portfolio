"""Repeat the original C7 with an extended stationary observation after retreat."""

from placement import c7, np, plan, ClearanceRecorder, OUT, execute


def extended_plan(o, c, period):
    pl = plan(o, c, period)
    tail = np.arange(1, round(5.0 / period) + 1) * period
    pl["t"] = np.r_[pl["t"], pl["t"][-1] + tail]
    pl["tape"] = np.vstack((pl["tape"], np.tile(pl["tape"][-1], (len(tail), 1))))
    pl["times"] = (*pl["times"][:-1], pl["times"][-1] + 5.0)
    return pl


if __name__ == "__main__":
    cfg, force, offset, duration, seed = c7.recipe("push_y3")
    rec = ClearanceRecorder()
    rec.library = c7.c6.LIB
    fb = c7.Feedback(rec, "window6", seed)

    def pulse(d, r, t, ph):
        u = t - ph[1] - offset
        d.xfrc_applied[r.body, :3] = force * (
            np.sin(np.pi * u / duration) ** 2 if 0 <= u <= duration else 0.0
        )

    tag = OUT / "c7_extended"
    OUT.mkdir(exist_ok=True, parents=True)
    execute(
        "ball",
        cfg,
        tag=str(tag),
        observer=rec,
        reflex=fb,
        disturbance=pulse,
        planner=extended_plan,
    )
    np.savez_compressed(str(tag) + "_clearance.npz", trace=np.array(rec.clearance))
    archived = np.load(c7.OUT / "push_y3_window6.npz")["log"]
    repeat = np.load(tag.with_suffix(".npz"))["log"]
    np.testing.assert_allclose(archived, repeat[: len(archived)], rtol=0, atol=1e-9)
