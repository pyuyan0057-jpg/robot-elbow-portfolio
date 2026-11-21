"""Static directional load balance at a measured undisturbed hold posture."""

import json
from scipy.optimize import linprog
from control import c7, run, np, mj, HERE
from exit_path import DIGITS


def analyze(m, d, r):
    weights = []
    wrenches = []
    sources = []
    caps = np.zeros(6)
    for k in range(d.ncon):
        ct = d.contact[k]
        if r.ball not in (ct.geom1, ct.geom2) or ct.efc_address < 0:
            continue
        other = ct.geom1 if ct.geom2 == r.ball else ct.geom2
        label = r.labels[other]
        if not 0 <= label < 6:
            continue
        load = np.empty(6)
        mj.mj_contactForce(m, d, k, load)
        if load[0] < 0.00001:
            continue
        caps[label] += load[0]
        sign = 1 if ct.geom2 == r.ball else -1
        axes = sign * ct.frame.reshape(3, 3)
        lever = ct.pos - d.xpos[r.body]
        sources.append(
            dict(
                pad=DIGITS[label],
                load_N=float(load[0]),
                normal_world=axes[0].tolist(),
                mu=ct.friction[:2].tolist(),
            )
        )
        for a in np.arange(16) * 2 * np.pi / 16:
            f = (
                axes[0]
                + ct.friction[0] * np.cos(a) * axes[1]
                + ct.friction[1] * np.sin(a) * axes[2]
            )
            wrenches.append(np.r_[f, np.cross(lever, f)])
            weights.append(np.eye(6)[label])
    results = {}
    for ix, name in enumerate("XYZ"):
        for s in (1, -1):
            external = np.eye(3)[ix] * s
            mat = np.column_stack((np.array(wrenches).T, np.r_[external, np.zeros(3)]))
            ub = np.column_stack((np.array(weights).T, np.zeros(6)))
            b = np.r_[-m.body_mass[r.body] * m.opt.gravity, np.zeros(3)]
            res = linprog(
                np.r_[np.zeros(len(weights)), -1.0],
                A_eq=mat,
                b_eq=b,
                A_ub=ub,
                b_ub=caps,
                bounds=(0, None),
                method="highs",
            )
            results[("+" if s > 0 else "-") + name] = (
                float(res.x[-1]) if res.success else None
            )
    return dict(
        time_s=d.time,
        ball_mass_kg=float(m.body_mass[r.body]),
        gravity=m.opt.gravity.tolist(),
        contacts=sources,
        capacity_N=results,
    )


if __name__ == "__main__":
    original = c7.Recorder.sample
    answer = []

    def sample(self, m, d, r, k, t, ph):
        original(self, m, d, r, k, t, ph)
        if abs(t - 3.58) < 0.0003:
            answer.append(analyze(m, d, r))

    c7.Recorder.sample = sample
    run(("v_quiet", json.loads((HERE / "frozen.json").read_text()), "capacity"))
    assert len(answer) == 1
    (HERE / "evidence/static_directions.json").write_text(
        json.dumps(answer[0], indent=2)
    )
    print(answer[0]["capacity_N"])
