"""Schedule the existing ring flexor independently of the three grasping flexors."""

import argparse
import json

import numpy as np
from scipy.optimize import minimize

from calculate import ROOT
from elbow_control import write_csv
from free_ball import A, H, RC
from pickup_pose import PREP_S
from tripod_trial import solve
from ring_mount import reorient_ring


def initial_angles(ring, stroke_mm):
    """Same unloaded tendon-energy convention as the parked-ring control."""
    stiffness = np.array(ring["spring_Nm_rad"])

    def energy(q):
        short = np.sum(2 * A * (1 - np.cos(q / 2)) + 2 * H * np.sin(q / 2) - RC * q)
        return (
            0.5 * np.dot(stiffness, q * q)
            + 0.002 * q.sum()
            + 5000 * max(0, stroke_mm / 1000 - short) ** 2
        )

    fit = minimize(
        energy,
        [1.0, 0.6, 0.6],
        bounds=list(zip([0.0] * 3, ring["upper_rad"])),
        tol=1e-13,
    )
    if not fit.success:
        raise RuntimeError(fit.message)
    return fit.x.tolist()


def schedule(lead, initial_mm, hardware):
    if not 0 <= lead <= 0.75 or not 0 <= initial_mm < 20:
        raise ValueError("Use 0–0.75 s of lead and less than 20 mm of initial take-up.")
    ring = json.loads((ROOT / "data/ring-finger.json").read_text())
    ring.update(
        start_s=-PREP_S - lead,
        initial_stroke_m=initial_mm / 1000,
        locked_stroke_m=0.020,
        initial_rad=initial_angles(ring, initial_mm),
        hardware_pad=hardware,
    )
    return ring


def revised_middle_rates():
    """Installed torsion springs: wire and mean coil dimensions in millimetres."""
    wire = np.array([0.50, 0.60, 0.60])
    turns = np.array([8.25, 5.75, 2.75])
    diameter = 6.4
    active_turns = turns + 13.0 / (3 * np.pi * diameter)
    return 200000 * wire**4 / (10.8 * diameter * active_turns * 2 * np.pi * 1000)


def trial(
    lead,
    initial_mm,
    dt=0.00025,
    hulls=None,
    mass_g=33,
    setup_samples=None,
    observe_failed_lift=False,
    revised_middle=False,
    revised_ring_mount=False,
):
    ring = schedule(lead, initial_mm, hulls is not None)
    if revised_ring_mount:
        reorient_ring(ring)
    return solve(
        "indexed",
        28,
        mass_g,
        dt,
        hulls,
        springs=revised_middle_rates() if revised_middle else None,
        setup_samples=setup_samples,
        ring_schedule=ring,
        observe_failed_lift=observe_failed_lift,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lead", type=float, default=0.50)
    parser.add_argument("--initial-mm", type=float, default=12)
    parser.add_argument(
        "--observe-lift",
        action="store_true",
        help="Continue elbow motion after a failed grasp for diagnosis.",
    )
    parser.add_argument(
        "--revised-middle",
        action="store_true",
        help="Use the revised middle MCP and DIP springs.",
    )
    parser.add_argument(
        "--r4-mount", action="store_true",
        help="Use the fixed rolled ring mount and revised middle springs (public pad-only geometry).",
    )
    args = parser.parse_args()
    result, rows = trial(
        args.lead,
        args.initial_mm,
        observe_failed_lift=args.observe_lift,
        revised_middle=args.revised_middle or args.r4_mount,
        revised_ring_mount=args.r4_mount,
    )
    print(json.dumps(result, indent=2))
    suffix = "-r4" if args.r4_mount else ("-revised" if args.revised_middle else "")
    out = (
        ROOT
        / "outputs"
        / "ring-timing"
        / f"{args.initial_mm:g}mm-{args.lead:g}s{suffix}"
    )
    out.mkdir(exist_ok=True, parents=True)
    write_csv(out / "trajectory.csv", rows)
