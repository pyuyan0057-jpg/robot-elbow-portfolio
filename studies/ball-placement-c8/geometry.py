"""World +Y points toward the horizontal table. All internal lengths are metres."""

from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "seven-axis-ball-c7"))
import study as c7
import numpy as np
import mujoco as mj
from scipy.spatial import ConvexHull
from motion import hand, plan, lo, hi
from physics import plant

OUT = HERE.parents[1] / "outputs" / "ball-c8"


class Glove:
    def __init__(self, m):
        ids, vertices = [], []
        for i in range(m.ngeom):
            if not m.geom(i).name.startswith("skin_"):
                continue
            mesh = m.geom_dataid[i]
            start, count = m.mesh_vertadr[mesh], m.mesh_vertnum[mesh]
            v = m.mesh_vert[start : start + count]
            hull = v[ConvexHull(v).vertices]
            ids.extend([i] * len(hull))
            vertices.extend(hull)
        self.ids, self.vertices = np.array(ids), np.array(vertices)

    def world(self, d):
        rot = d.geom_xmat.reshape(-1, 3, 3)[self.ids]
        return np.einsum("nij,nj->ni", rot, self.vertices) + d.geom_xpos[self.ids]


def calibration():
    cfg, *_ = c7.recipe("push_y3")
    pl = plan("ball", cfg, 0.002)
    m, d, r, ty, _ = plant("ball", pl, cfg["dt"])
    z = np.load(c7.OUT / "push_y3_window6.npz")
    glove = Glove(m)
    d.qpos[:] = z["q"][100]  # 4.0 s, before descent
    mj.mj_forward(m, d)
    R = d.xmat[r.palm].reshape(3, 3)
    v = (glove.world(d) - d.xpos[r.palm]) @ R
    v = v[ConvexHull(v).vertices]
    ball = (d.xpos[r.body] - d.xpos[r.palm]) @ R
    return pl, v, ball


if __name__ == "__main__":
    pl, v, b = calibration()
    print(
        "base angles",
        np.degrees(pl["q"]),
        "limits",
        np.degrees([lo, hi]),
        "target",
        pl["place_target"],
        "ball",
        b,
    )
    n = np.random.default_rng(23).normal(size=(6000, 3))
    n /= np.linalg.norm(n, axis=1)[:, None]
    margins = 0.02 - np.max((v - b) @ n.T, axis=0)
    for j in np.argsort(margins)[-1:-5:-1]:
        print("ideal down vector in palm", n[j], 1000 * margins[j])
    q = pl["tape"][3100, :7]
    print("old down vector", hand(q)[1].as_matrix()[1], "angles", np.degrees(q))
