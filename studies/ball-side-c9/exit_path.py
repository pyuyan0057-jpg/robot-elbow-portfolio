"""C8 escape geometry in the moving palm frame (mm), from saved qpos frames."""

from pathlib import Path
import json, sys

HERE = Path(__file__).resolve().parent
C8 = HERE.parent / "ball-placement-c8"
sys.path.insert(0, str(C8))
from geometry import c7, mj, np, plant, plan, OUT
from touchdown_review import DIGITS, SIDES

RESULT = HERE.parents[1] / "outputs" / "ball-c9"


def exit_path(case):
    archive = np.load(OUT / (case + "_final.npz"))
    cfg = json.loads((OUT / (case + "_final.json")).read_text())["case"]
    m, d, r, _, _ = plant("ball", plan("ball", cfg, 0.002), cfg["dt"])
    geoms = [[i for i in range(m.ngeom) if r.labels[i] == j] for j in range(6)]
    a = archive["log"]
    t = a[:, 9]
    start = archive["times"][1] + c7.recipe(case)[2]
    loss = next(
        row["loss_s"]
        for row in json.loads((C8 / "evidence/touchdown_review.json").read_text())[
            "graded"
        ]
        if row["case"] == case
    )

    def local_at(sec):
        return a[np.abs(t - sec).argmin(), 18:21] * 1000

    end = start + c7.recipe(case)[3]
    rows = []
    for sec in (
        np.arange(round((loss - 0.24) / 0.04), round((loss + 0.12) / 0.04) + 1) * 0.04
    ):
        d.qpos[:] = archive["q"][round(sec / 0.04)]
        mj.mj_forward(m, d)
        R = d.xmat[r.palm].reshape(3, 3)
        b = d.geom_xpos[r.ball]
        digits = {}
        for name, group in zip(DIGITS, geoms):
            options = []
            for i in group:
                ft = np.zeros(6)
                gap = mj.mj_geomDistance(m, d, r.ball, i, 0.5, ft)
                options.append((gap, i, ft[3:].copy()))
            gap, i, wp = min(options, key=lambda v: v[0])
            digits[name] = dict(
                gap_mm=gap * 1000,
                skin=m.geom(i).name,
                ball_to_skin_mm=((wp - b) @ R * 1000).tolist(),
            )
        rows.append(
            dict(
                t=sec,
                center_mm=((b - d.xpos[r.palm]) @ R * 1000).tolist(),
                digits=digits,
            )
        )
    return dict(
        case=case,
        start_s=start,
        end_s=end,
        loss_s=loss,
        shift_during_push_mm=(local_at(end) - local_at(start)).tolist(),
        shift_push_to_loss_mm=(local_at(loss) - local_at(start)).tolist(),
        frames=rows,
    )


if __name__ == "__main__":
    data = [exit_path(n) for n in SIDES]
    (RESULT / "escape.json").write_text(json.dumps(data, indent=1))
    for v in data:
        print(
            v["case"],
            np.round(v["shift_during_push_mm"], 1),
            np.round(v["shift_push_to_loss_mm"], 1),
        )
        for row in v["frames"][::2]:
            print(
                round(row["t"], 2),
                np.round(row["center_mm"], 1),
                [
                    (
                        n,
                        round(b["gap_mm"], 1),
                        np.round(b["ball_to_skin_mm"], 0).tolist(),
                    )
                    for n, b in row["digits"].items()
                ],
            )
