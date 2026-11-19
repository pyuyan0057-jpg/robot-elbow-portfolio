"""Gated tendon-driven tripod trial with a manually indexed middle MCP."""

import argparse
import json
import math

import numpy as np
from calculate import ROOT
from elbow_control import write_csv
from free_ball import run
from pickup_pose import PREP_S, CLOSE_S
from thumb_cup import configured_hand, OFFSET

NAMES = ("index", "thumb", "middle")
PIVOT = np.array([-0.011, 0.003, 0.0])
CASES = {"straight": (0, 12), "indexed": (-15, 5)}


def yaw_matrix(deg):
    t = math.radians(deg)
    return np.array(
        [[math.cos(t), -math.sin(t), 0], [math.sin(t), math.cos(t), 0], [0, 0, 1.0]]
    )


def solve(
    case,
    stroke,
    mass=33,
    dt=0.00025,
    hulls=None,
    springs=None,
    setup_samples=None,
    ring_schedule=None,
    observe_failed_lift=False,
):
    deg, shift = CASES[case]
    yaw = yaw_matrix(deg)
    offset = (OFFSET[0] + shift, *OFFSET[1:])
    cfg = configured_hand(True, 24, 4, offset, True)
    middle = json.loads((ROOT / "data/middle-finger.json").read_text())
    for link in middle["links"]:
        for key in ("root", "com"):
            link[key] = PIVOT + yaw @ (link[key] - PIVOT)
        link["axis"] = yaw @ link["axis"]
    middle["pad_center"] = PIVOT + yaw @ (middle["pad_center"] - PIVOT)
    middle["locked_stroke_m"] = stroke / 1000
    if springs is not None:
        middle["spring_Nm_rad"] = springs
    cfg["digits"].append(middle)
    cfg["adjacent_contact"] = hulls is not None
    opt = dict(
        ball_mass=mass / 1000,
        dt=dt,
        prep=PREP_S,
        close_s=CLOSE_S,
        hanging_s=1.0,
        private_hulls=hulls,
    )

    if ring_schedule is not None:
        for digit in cfg["digits"]:
            digit.update(start_s=-PREP_S, initial_stroke_m=0.0)
        cfg["digits"].append(ring_schedule)
        # Common 1.85 s preparation across all leads, release and grasp timing unchanged.
        opt["prep"] = PREP_S + 1.0

    def gate(rows, result, target):
        return (
            result["approach_peak_N"] <= 25
            and all(
                min(r[n + "_N"] for n in NAMES) > 0.01
                and max(r[n + "_pull_N"] for n in NAMES) <= 25
                and r.get("ring_N", 0.0) <= 0.001
                and r.get("ring_pull_N", 0.0) <= 25
                and r["plane_N"] == 0
                and r["hard_N"] == 0
                for r in rows
            )
            and all(
                abs(r["elbow_deg"] - target) < 1
                and r["ball_rel_speed_m_s"] < 0.002
                and r["spin_rel_rad_s"] < 0.2
                for r in rows
                if r["t_s"] >= rows[-1]["t_s"] - 0.25
            )
        )

    result, rows = run(cfg, hanging_only=True, setup_samples=setup_samples, **opt)
    held = gate(rows, result, 0)
    lift = None
    if held or observe_failed_lift:
        result, rows = run(cfg, **opt)
        lift = held and gate(rows, result, 90)
    floor = next((r["t_s"] for r in rows if r["plane_N"] > 0.001), None)
    return (
        dict(
            case=case,
            shift_mm=shift,
            yaw_deg=deg,
            mass_g=mass,
            middle_stroke_mm=stroke,
            dt_s=dt,
            cad_hulls=hulls is not None,
            **{n + "_release_N": rows[0][n + "_N"] for n in NAMES},
            tripod_samples=sum(min(r[n + "_N"] for n in NAMES) > 0.01 for r in rows),
            plane_s=floor,
            peak_approach_tension_N=result["approach_peak_N"],
            peak_adjacent_normal_N=result["adjacent_peak_N"],
            peak_adjacent_pair=result["adjacent_pair"],
            hanging_pass=held,
            lift_pass=lift,
        ),
        rows,
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("case", choices=CASES)
    p.add_argument("--stroke-mm", type=float, default=28)
    a = p.parse_args()
    s, rows = solve(a.case, a.stroke_mm)
    out = ROOT / "outputs" / "tripod" / f"{a.case}-{a.stroke_mm:g}mm"
    out.mkdir(exist_ok=True, parents=True)
    write_csv(out / "trajectory.csv", rows)
    print(json.dumps(s, indent=2))
