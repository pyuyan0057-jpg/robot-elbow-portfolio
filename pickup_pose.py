"""Repeat the hanging-arm lift after changing the supported pickup and thumb opposition."""

import argparse
import copy
import json
import math

import numpy as np
from calculate import ROOT
from elbow_control import write_csv
from free_ball import run

BASE_DEG = math.degrees(0.5861476048505048)
# Centreline of the archived thumb rotor in palm coordinates. Positive opposition is about -Y.
PIVOT = np.array(
    [-0.043 + 0.015 * math.sin(0.9), -0.050 - 0.015 * math.cos(0.9), -0.014]
)
CLOSE_S, PREP_S = 0.50, 0.85


def rotation(angle):
    a = math.radians(angle - BASE_DEG)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])


def pickup(base, offset_mm, angle):
    cfg = copy.deepcopy(base)
    R = rotation(angle)
    thumb = cfg["digits"][1]

    def move(point):
        return (PIVOT + R @ (np.array(point) - PIVOT)).tolist()

    for link in thumb["links"]:
        link["root"] = move(link["root"])
        link["com"] = move(link["com"])
        link["axis"] = (R @ link["axis"]).tolist()
    thumb["pad_center"] = move(thumb["pad_center"])
    cfg["ball_in_palm"] = (
        np.array(base["ball_in_palm"]) + np.array(offset_mm) / 1000
    ).tolist()
    # Contacts are now determined by supported closure rather than the archived fixed points.
    cfg.pop("contact_in_palm", None)
    return cfg


def evaluate(offset, angle, dt=0.00025, private_hulls=None, **kwargs):
    base = json.loads((ROOT / "data/free-ball-model.json").read_text())
    cfg = pickup(base, offset, angle)
    result, rows = run(
        cfg, dt=dt, prep=PREP_S, close_s=CLOSE_S, private_hulls=private_hulls, **kwargs
    )
    tail = [row for row in rows if row["t_s"] >= rows[-1]["t_s"] - 0.25]
    first = next((r for r in rows if min(r["index_N"], r["thumb_N"]) < 0.01), None)
    no_hardware = all(r["hard_N"] == 0 and r["plane_N"] == 0 for r in rows)
    hold_err = max(abs(r["elbow_deg"] - 90) for r in tail)
    peak_tension = max(max(r["index_pull_N"], r["thumb_pull_N"]) for r in rows)
    passed = (
        first is None
        and no_hardware
        and result["status"] == "settled pinch"
        and hold_err <= 1
        and peak_tension <= 25
    )
    measure = dict(
        dx_mm=offset[0],
        dy_mm=offset[1],
        dz_mm=offset[2],
        opposition_deg=angle,
        dt_s=dt,
        mu=kwargs.get("mu", 0.35),
        index_start_N=rows[0]["index_N"],
        thumb_start_N=rows[0]["thumb_N"],
        first_interrupted_s=first["t_s"] if first else None,
        final_pad_exit_s=result["final_pad_exit_s"],
        exit_elbow_deg=result["exit_elbow_deg"],
        plane_s=result["floor_s"],
        first_hard_s=result["first_hard_s"],
        peak_tendon_N=peak_tension,
        hold_error_deg=hold_err,
        hold_speed_m_s=max(r["ball_rel_speed_m_s"] for r in tail),
        hold_spin_rad_s=max(r["spin_rel_rad_s"] for r in tail),
        outcome=result["status"],
        passed=passed,
    )
    return measure, rows


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--offset-mm",
        type=float,
        nargs=3,
        default=[0.0, 0.0, 0.0],
        metavar=("X", "Y", "Z"),
    )
    ap.add_argument("--opposition-deg", type=float, default=BASE_DEG)
    ap.add_argument("--dt", type=float, default=0.00025)
    a = ap.parse_args()
    if not 0 <= a.opposition_deg <= BASE_DEG + 1e-7:
        ap.error("Choose an angle within the archived 0 to 33.5844 degree sweep.")
    result, rows = evaluate(a.offset_mm, a.opposition_deg, a.dt)
    out = (
        ROOT
        / "outputs"
        / "pickup"
        / ("_".join(map(str, [*a.offset_mm, a.opposition_deg, a.dt])))
    )
    out.mkdir(parents=True, exist_ok=True)
    write_csv(out / "trajectory.csv", rows)
    (out / "summary.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
