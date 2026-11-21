"""Plot actual gloved mesh projections and trajectories, then write matched evidence."""

import json
from pathlib import Path
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle

from scipy.spatial import ConvexHull
from metrics import evaluate
from geometry import np, mj, plant, plan, OUT, Glove

HERE = Path(__file__).resolve().parent
EVIDENCE = HERE / "evidence"


def snapshot(ax, tag, sec, heading):
    z = np.load(tag.with_suffix(".npz"))
    cfg = json.loads(tag.with_suffix(".json").read_text())["case"]
    pl = plan("ball", cfg, 0.002)
    m, d, r, ty, _ = plant("ball", pl, cfg["dt"])
    g = Glove(m)
    frame = round(sec / 0.04)
    d.qpos[:] = z["q"][frame]
    mj.mj_kinematics(m, d)
    verts = g.world(d)
    ball = d.geom_xpos[r.ball]
    horizontal = np.array([0.75, 0, -0.66144])
    horizontal /= np.linalg.norm(horizontal)
    depth = np.cross(horizontal, [0, -1, 0])

    def project(points):
        return np.column_stack(((points - ball) @ horizontal, ty - points[:, 1])) * 1000

    v = project(verts)
    bc = project(ball[None, :])[0]
    ax.axhspan(-8, 0, color="#dce9e5")
    ax.axhline(0, color="#33796d", lw=1.4)
    for i in sorted(np.unique(g.ids), key=lambda i: d.geom_xpos[i] @ depth):
        poly = v[g.ids == i]
        ax.add_patch(
            Polygon(
                poly[ConvexHull(poly).vertices],
                facecolor="#afbdd1",
                edgecolor="#60778f",
                lw=0.45,
                alpha=0.82,
            )
        )
    ax.add_patch(Circle(bc, 20, facecolor="#f5b64f", alpha=0.43, edgecolor="none"))
    ax.add_patch(Circle(bc, 20, fill=False, edgecolor="#ad6200", lw=1.8))
    ax.plot(0, bc[1] - 20, "o", ms=4, color="#ad6200")
    idx = v[:, 1].argmin()
    ax.plot(*v[idx], "o", color="#244d83", ms=4)
    ax.set(
        xlim=(-66, 66),
        ylim=(-5, 79),
        aspect="equal",
        ylabel="距桌面高度（mm）",
        xlabel="侧向投影（mm）",
    )
    ax.set_title(heading, loc="left", weight="bold")
    ax.text(
        -62,
        71,
        "球底 %.1f mm    手套最低处 %.1f mm" % (bc[1] - 20, v[idx, 1]),
        fontsize=10,
    )


def matched(tag):
    met = evaluate(tag)
    z = np.load(tag.with_suffix(".npz"))
    a = z["log"]
    t = a[:, 9]
    i = abs(t - met["ball_touch_s"]).argmin()
    j = abs(t - t[i] - 5.0).argmin()
    cfg = json.loads(tag.with_suffix(".json").read_text())["case"]
    p = plan("ball", cfg, 0.002)
    target = p["place_target"][[0, 2]]
    pos = a[:, [15, 17]]
    return (
        {
            "touch_s": t[i],
            "at_5s_error_mm": np.linalg.norm(pos[j] - target) * 1000,
            "at_5s_shift_mm": np.linalg.norm(pos[j] - pos[i]) * 1000,
        },
        (t[i : j + 1] - t[i]),
        1000 * np.linalg.norm(pos[i : j + 1] - pos[i], axis=1),
    )


if __name__ == "__main__":
    EVIDENCE.mkdir(exist_ok=True)
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    old = OUT / "c7_extended"
    new = OUT / "push_y3_final"
    fig = plt.figure(figsize=(10.2, 7.3), layout="constrained")
    gs = fig.add_gridspec(2, 2, height_ratios=[1.25, 1])
    # Closest available 40-ms frames to a common ball-bottom gap of 4 mm, before first contact.
    for ax, tag, title in [
        (fig.add_subplot(gs[0, 0]), old, "原方案 · 指套已经压桌"),
        (fig.add_subplot(gs[0, 1]), new, "新方案 · 球底先靠近桌面"),
    ]:
        z = np.load(tag.with_suffix(".npz"))
        event = evaluate(tag)["ball_touch_s"]
        ix = np.arange(len(z["q"]))
        t = ix * 0.04
        valid = (t < event) & (t >= z["times"][2])
        idx = ix[valid]
        height = z["height"][np.minimum(idx * 20, len(z["height"]) - 1)]
        sec = idx[np.argmin(abs(height - 4))] * 0.04
        snapshot(ax, tag, sec, title)
    ax = fig.add_subplot(gs[1, :])
    metrics = {}
    for tag, label, color in [(old, "原方案", "#9a5e48"), (new, "新方案", "#267968")]:
        met, t, y = matched(tag)
        metrics[tag.name] = met
        ax.plot(t, y, label=label, color=color, lw=2.3)
        ax.text(5.02, y[-1], f"{y[-1]:.2f} mm", va="center", color=color)
    ax.set(
        xlim=(-0.05, 5.55),
        ylim=(-0.65, 15),
        xlabel="球首次接触桌面后的时间（s）",
        ylabel="球心相对首次触桌位置的水平偏移（mm）",
    )
    ax.grid(axis="y", alpha=0.2)
    ax.legend(loc="center right", frameon=False)
    ax.set_title("同样观察 5 秒，比较球被带走了多远", loc="left", weight="bold")
    fig.suptitle(
        "Ø40 mm 球放回桌面    |    3 N 扰动工况的 MuJoCo 复测", size=17, weight="bold"
    )
    fig.savefig(EVIDENCE / "placement.png", dpi=155)
    (EVIDENCE / "matched.json").write_text(json.dumps(metrics, indent=2))
    rows = json.loads((OUT / "verification_all.json").read_text())
    (EVIDENCE / "verification.json").write_text(json.dumps(rows, indent=2))
