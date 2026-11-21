"""Nearest actual surfaces and trajectory through the opening at loss of support."""

import json
from concurrent.futures import ProcessPoolExecutor
from scipy.spatial.transform import Rotation
from control import prepare, make_plan, SetDown, c7, mj, np, execute
from exit_path import RESULT, C8, OUT, DIGITS, SIDES


def diagnose(name):
    cfg, force, offset, duration, seed = prepare(name)
    pl = make_plan("ball", cfg, 0.002)
    start = pl["times"][1] + offset
    loss = next(
        g["loss_s"]
        for g in json.loads((C8 / "evidence/touchdown_review.json").read_text())[
            "graded"
        ]
        if g["case"] == name
    )
    rec = c7.Recorder()
    rec.library = c7.c6.LIB
    sample = rec.sample
    rows = []
    positions = []
    angles = []

    def inspect(m, d, r, k, t, ph):
        sample(m, d, r, k, t, ph)
        if t < start - 0.002 or t > loss + 0.034:
            return
        R = d.xmat[r.palm].reshape(3, 3)
        b = d.geom_xpos[r.ball]
        positions.append((t, *((b - d.xpos[r.palm]) @ R * 1000)))
        if t <= start + duration + 0.001:
            angles.append(R.T @ d.geom_xmat[r.ball].reshape(3, 3))
        if abs(t - loss) > 0.018:
            return
        near = []
        for digit in range(6):
            best = (1.0, None, None)
            for i in range(m.ngeom):
                if r.labels[i] != digit:
                    continue
                pair = np.empty(6)
                gap = mj.mj_geomDistance(m, d, r.ball, i, 0.25, pair)
                if gap < best[0]:
                    best = (gap, m.geom(i).name, ((pair[3:] - b) @ R * 1000).tolist())
            near.append(
                dict(
                    digit=DIGITS[digit],
                    gap_mm=1000 * best[0],
                    geom=best[1],
                    offset_mm=best[2],
                )
            )
        rows.append((t, near))

    rec.sample = inspect

    def pulse(d, r, t, ph):
        u = t - ph[1] - offset
        d.xfrc_applied[r.body, :3] = force * (
            np.sin(np.pi * u / duration) ** 2 if 0 <= u <= duration else 0.0
        )

    tag = RESULT / "diagnosis" / name
    tag.parent.mkdir(parents=True, exist_ok=True)
    gate = SetDown(rec, c7.Feedback(rec, "window6", seed), cfg, pl)
    execute(
        "ball",
        cfg,
        tag=str(tag),
        observer=rec,
        disturbance=pulse,
        reflex=gate,
        planner=lambda *args: pl,
    )
    with (
        np.load(tag.with_suffix(".npz")) as run,
        np.load(OUT / (name + "_final.npz")) as old,
    ):
        np.testing.assert_allclose(run["log"], old["log"], rtol=0, atol=1e-9)
    xyz = np.array(positions)

    def at(t):
        return xyz[np.abs(xyz[:, 0] - t).argmin(), 1:]

    motion = at(loss + 0.016) - at(loss - 0.016)
    u = motion / np.linalg.norm(motion)
    chosen = min(rows, key=lambda v: abs(v[0] - loss))[1]
    a, b = np.array(chosen[1]["offset_mm"]), np.array(chosen[4]["offset_mm"])
    a -= u * (u @ a)
    b -= u * (u @ b)
    angle = np.degrees(
        np.arccos(np.clip(a @ b / np.linalg.norm(a) / np.linalg.norm(b), -1, 1))
    )
    ori = Rotation.from_matrix(np.array(angles))
    return dict(
        case=name,
        relative_center_push_mm=(at(start + duration) - at(start)).tolist(),
        rotation_during_push_deg=float(
            np.rad2deg((ori[1:] * ori[:-1].inv()).magnitude().sum())
        ),
        exit_unit_palm=u.tolist(),
        thumb_little_angle_across_escape_deg=float(angle),
        surfaces_at_loss=chosen,
        loss_s=loss,
        series=rows,
    )


if __name__ == "__main__":
    with ProcessPoolExecutor(3) as pool:
        data = list(pool.map(diagnose, SIDES))
    (RESULT / "exit_2ms.json").write_text(json.dumps(data, indent=1))
    for g in data:
        print(
            g["case"],
            g["thumb_little_angle_across_escape_deg"],
            g["rotation_during_push_deg"],
            [(s["digit"], round(s["gap_mm"], 2)) for s in g["surfaces_at_loss"]],
        )
