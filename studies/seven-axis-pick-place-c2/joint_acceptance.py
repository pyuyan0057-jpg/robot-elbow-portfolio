import json
from motion import S, D, plan, ease, lo, hi
from physics import plant, integrate
import numpy as np, mujoco as mj


def run():
    c = json.loads((D / "recipe_ball.json").read_text())
    pl = plan("ball", c, 0.002)
    rows = []
    base = np.deg2rad([15, 45, -10, 60, 90, 0, 0])
    dt = 0.0005
    for j, name in enumerate(S["names"]):
        m, d, r, _, _ = plant("ball", pl, dt)
        d.qpos[list(r.armq)] = base
        d.qpos[
            m.joint("ball_free").qposadr[0] : m.joint("ball_free").qposadr[0] + 3
        ] = [0, -2, 2]
        m.geom_pos[r.table, 1] = 3
        mj.mj_forward(m, d)
        v = np.zeros(1)
        hist = []
        t = np.arange(1201) * 0.002
        ref = np.tile(base, (len(t), 1))
        ref[:, j] += np.deg2rad(8) * (ease((t - 0.15) / 0.7) - ease((t - 1.25) / 0.7))
        speed = np.gradient(ref, 0.002, axis=0)
        out = np.zeros((4, 44))
        for p, vel in zip(ref, speed):
            integrate(m, d, r, np.r_[p, vel, [0.0] * 5, 0.0], 4, v, out)
            hist.append(out[-1, 35:42].copy())
        q = np.array(hist)
        err = np.abs(np.degrees(q - ref))
        rows.append(
            dict(
                axis=name,
                excursion_deg=np.degrees(max(q[:, j]) - base[j]),
                active_error_deg=max(err[:, j]),
                other_error_deg=np.delete(err, j, axis=1).max(),
                return_error_deg=err[-1].max(),
            )
        )
    assert all(r["excursion_deg"] > 7.7 and r["other_error_deg"] < 1.0 for r in rows)
    tests = {}
    for k, extra in [
        (
            "user_elbow_80",
            {
                "user_deg": [
                    [-75, 75],
                    [0, 80],
                    [-25, 10],
                    [0, 80],
                    [0, 180],
                    [-24, 24],
                    [-16, 16],
                ]
            },
        ),
        ("unreachable_object_position", {"shift_mm": [900, 0, 900]}),
        ("low_velocity", {"speed_limit_deg_s": [4] * 7}),
    ]:
        try:
            plan("ball", dict(c, **extra), 0.002)
            tests[k] = "accepted"
        except ValueError as e:
            tests[k] = str(e)
    assert "accepted" not in tests.values()
    (D / "七轴单动与限位.json").write_text(
        json.dumps({"axes": rows, "rejected_commands": tests}, indent=2)
    )


if __name__ == "__main__":
    run()
