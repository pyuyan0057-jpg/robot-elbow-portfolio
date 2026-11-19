"""Run one locked-angle and hand-controller combination in the same hanging pickup."""

import argparse
import json
import numpy as np
from ring_timing import revised_middle_rates, schedule
from ring_mount import reorient_ring
from tripod_trial import NAMES, tripod_model
from tripod_coordination import ThreePadFeedback, lock_thumb
from free_ball import run

# Parts supported by the additional carrier, downstream of the existing rotor.
CARRIER = {
    "E4_Root_Optimized-5",
    "E3_M3x12-1",
    "E3_M3x12-2",
    "E3_Guide-33",
    "E3_Pin-13",
    "E3_M2_Retainer-25",
    "E3_M2_Retainer-26",
}
PROFILES = {
    "preset": None,
    "balanced": (1.2, 1.2, 1.2),
    "middle": (1.0, 1.2, 1.7),
    "light": (0.75, 0.75, 0.9),
    "firm": (1.6, 1.6, 1.8),
}


def trial(
    angle, profile, mass=33, dt=0.00025, transfer=0.15, full=True, hulls=None, ring=None
):
    if ring is None:
        ring = schedule(0.5, 12, hulls is not None)
        reorient_ring(ring)
    cfg, opt = tripod_model(
        "indexed", 28, mass, dt, hulls, revised_middle_rates(), ring
    )
    lock_thumb(cfg, angle, hulls or (), CARRIER)
    control = (
        ThreePadFeedback(PROFILES[profile]) if PROFILES[profile] is not None else None
    )
    metrics = {}
    opt.update(
        transfer_s=transfer, hanging_s=1.0 if full else 0.22, trial_metrics=metrics
    )
    res, rows = run(cfg, hanging_only=not full, hand_feedback=control, **opt)

    def clean(r):
        return (
            min(r[n + "_N"] for n in NAMES) > 0.01
            and r["ring_N"] <= 0.001
            and r["hard_N"] == 0
            and r["plane_N"] == 0
        )

    def settled(r, target):
        return (
            abs(r["elbow_deg"] - target) < 1
            and r["ball_rel_speed_m_s"] < 0.002
            and r["spin_rel_rad_s"] < 0.2
        )

    def first(predicate, subset):
        return next((r["t_s"] for r in subset if predicate(r)), None)

    free = [r for r in rows if r["t_s"] >= transfer - 1e-9]
    stand = [r for r in free if r["t_s"] <= transfer + 1 + 1e-9]
    hanging = (
        full
        and metrics["all_step_peak_pull_N"] <= 25
        and all(clean(r) for r in rows if r["t_s"] <= transfer + 1)
        and all(settled(r, 0) for r in stand if r["t_s"] >= transfer + 0.75)
    )
    passed = (
        hanging
        and all(clean(r) for r in rows)
        and all(settled(r, 90) for r in rows if r["t_s"] >= rows[-1]["t_s"] - 0.25)
    )
    lost = first(lambda r: not clean(r), free)
    return (
        dict(
            angle_deg=angle,
            profile=profile,
            mass_g=mass,
            dt_s=dt,
            transfer_s=transfer,
            hardware_hulls=hulls is not None,
            full=full,
            **metrics,
            index_initial_N=rows[0]["index_N"],
            thumb_initial_N=rows[0]["thumb_N"],
            middle_initial_N=rows[0]["middle_N"],
            first_invalid_s=first(lambda r: not clean(r), rows),
            clean_free_prefix_s=(lost if lost is not None else free[-1]["t_s"])
            - transfer,
            free_clean_samples=sum(clean(r) for r in free),
            first_plane_s=first(lambda r: r["plane_N"] > 0.001, rows),
            end_elbow_deg=rows[-1]["elbow_deg"],
            end_hardware_N=rows[-1]["hard_N"],
            end_plane_N=rows[-1]["plane_N"],
            hanging_pass=bool(hanging) if full else None,
            lift_pass=bool(passed) if full else None
        ),
        rows,
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--angle", type=float, default=0)
    p.add_argument("--profile", choices=PROFILES, default="preset")
    args = p.parse_args()
    print(json.dumps(trial(args.angle, args.profile)[0], indent=2))
