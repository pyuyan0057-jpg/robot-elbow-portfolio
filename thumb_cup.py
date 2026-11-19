"""Matched hanging-release test of a fixed concave thumb pad."""

import argparse
import json
import math

import numpy as np
from calculate import ROOT
from elbow_control import write_csv
from free_ball import run
from pickup_pose import pickup, rotation, PREP_S, CLOSE_S

ANGLE = 32.75
OFFSET = (0.69, -0.59, 2.28)
# Axis of the permanent thumb surface in the archived hand frame.
AXIS = np.array([0.6384882803744408, 0.5398108895866398, -0.5485771771666865])
RAISED_AXIS = np.array([0.66292567, 0.50872500, -0.54929812])
RIM, RADIUS, FLOOR, BASE = 0.007, 0.0107, 0.0094, 0.0056
CUT_RIM, CUT_RADIUS = 0.004, 0.0083
WAIT = 1.0


def cup_cells(center, axis, sectors, rings, cut=False):
    axis = axis / np.linalg.norm(axis)
    u = np.cross(axis, [0, 1, 0])
    u /= np.linalg.norm(u)
    v = np.cross(axis, u)
    cells = []
    rim = CUT_RIM if cut else RIM
    z_rim = math.sqrt(0.0092**2 - rim**2)
    radius = CUT_RADIUS if cut else RADIUS
    floor = z_rim - radius + math.sqrt(radius**2 - rim**2) if cut else FLOOR
    base = floor - 0.0002 if cut else BASE
    radii = list(np.linspace(0, rim, rings + 1))
    if cut:
        # The remaining outer cap meets the bowl at the rim. A lower convex body
        # stops below the bowl floor, so a single hull cannot bridge the hollow.
        radii.extend(np.linspace(rim, math.sqrt(0.0092**2 - base**2), 4)[1:])
        phi = np.linspace(-math.pi / 2, math.asin(base / 0.0092), 17)
        theta = np.arange(sectors) * 2 * math.pi / sectors
        directions = np.outer(np.cos(theta), u) + np.outer(np.sin(theta), v)
        cells.append(
            np.vstack(
                [
                    center + 0.0092 * (np.sin(p) * axis + np.cos(p) * directions)
                    for p in phi
                ]
            )
        )
    for i in range(sectors):
        angles = 2 * math.pi * np.array([i, i + 1]) / sectors
        dirs = np.outer(np.cos(angles), u) + np.outer(np.sin(angles), v)
        for r0, r1 in zip(radii, radii[1:]):
            vertices = []
            for r in (r0, r1):
                h = (
                    floor + radius - math.sqrt(radius**2 - r**2)
                    if r <= rim
                    else math.sqrt(0.0092**2 - r**2)
                )
                for radial in (dirs[:1] if r == 0 else dirs):
                    vertices.extend([center + r * radial + s * axis for s in (base, h)])
            cells.append(np.array(vertices))
    return cells


def configured_hand(cup, sectors, rings, offset, cut):
    cfg = pickup(
        json.loads((ROOT / "data/free-ball-model.json").read_text()), offset, ANGLE
    )
    if cup:
        cfg["cup_cells"] = cup_cells(
            np.array(cfg["digits"][1]["pad_center"]),
            rotation(ANGLE) @ (AXIS if cut else RAISED_AXIS),
            sectors,
            rings,
            cut,
        )
        cfg["cut_cup"] = cut
        if cut:
            from pickup_pose import PIVOT

            cfg["pad_delta_kg"] = -0.00005725291690832915
            material_center = np.array(
                [-0.046421613289039265, -0.026709018062755503, -0.051905064157416844]
            )
            cfg["pad_delta_com_m"] = PIVOT + rotation(ANGLE) @ (material_center - PIVOT)
    return cfg


def trial(
    cup,
    mass_g,
    sectors=24,
    rings=4,
    dt=0.00025,
    hard=None,
    mu=0.35,
    offset=OFFSET,
    cut=False,
):
    cfg = configured_hand(cup, sectors, rings, offset, cut)
    options = dict(
        ball_mass=mass_g / 1000,
        dt=dt,
        mu=mu,
        prep=PREP_S,
        close_s=CLOSE_S,
        hanging_s=WAIT,
        private_hulls=hard,
    )

    def check(rows, angle):
        tail = [r for r in rows if r["t_s"] >= rows[-1]["t_s"] - 0.25]
        return all(
            min(r["index_N"], r["thumb_N"]) > 0.01
            and r["hard_N"] == 0
            and r["plane_N"] == 0
            and max(r["index_pull_N"], r["thumb_pull_N"]) <= 25
            for r in rows
        ) and all(
            abs(r["elbow_deg"] - angle) < 1
            and r["ball_rel_speed_m_s"] < 0.002
            and r["spin_rel_rad_s"] < 0.2
            for r in tail
        )

    res, rows = run(cfg, hanging_only=True, **options)
    hanging = check(rows, 0)
    lift = None
    if hanging:
        # Replay exactly the same approach and transfer, then continue from the hanging state.
        _, full = run(cfg, **options)
        assert np.allclose(
            [rows[-1][k] for k in ("index_N", "thumb_N", "travel_rel_mm")],
            [full[len(rows) - 1][k] for k in ("index_N", "thumb_N", "travel_rel_mm")],
            atol=1e-7,
        )
        rows = full
        lift = check(rows, 90)
    first = next((r for r in rows if min(r["index_N"], r["thumb_N"]) <= 0.01), None)
    floor = next((r["t_s"] for r in rows if r["plane_N"] > 0.001), None)
    tail = [r for r in rows if r["t_s"] >= rows[-1]["t_s"] - 0.25]
    summary = dict(
        pad=("recess" if cut else "raised") if cup else "convex",
        mass_g=mass_g,
        hardware=hard is not None,
        sectors=sectors if cup else 0,
        rings=rings if cup else 0,
        dt_s=dt,
        mu=mu,
        hanging_pass=bool(hanging),
        lift_pass=bool(lift) if lift is not None else None,
        first_loss_s=first["t_s"] if first else None,
        plane_s=floor,
        index_release_N=rows[0]["index_N"],
        thumb_release_N=rows[0]["thumb_N"],
        max_drift_mm=max(r["travel_rel_mm"] for r in rows),
        end_elbow_deg=rows[-1]["elbow_deg"],
        end_index_N=rows[-1]["index_N"],
        end_thumb_N=rows[-1]["thumb_N"],
        tail_speed_mm_s=1000 * max(r["ball_rel_speed_m_s"] for r in tail),
        tail_spin_rad_s=max(r["spin_rel_rad_s"] for r in tail),
    )
    return summary, rows


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--mass-g", type=float, choices=[33, 150], default=33)
    p.add_argument("--convex", action="store_true")
    a = p.parse_args()
    summary, rows = trial(not a.convex, a.mass_g, cut=not a.convex)
    dest = ROOT / "outputs" / "thumb-cup" / f"{summary['pad']}-{a.mass_g}g"
    dest.mkdir(parents=True, exist_ok=True)
    write_csv(dest / "trajectory.csv", rows)
    (dest / "result.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
