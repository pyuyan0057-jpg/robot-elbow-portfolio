"""Run the frozen policy, then independently score the complete trajectories."""

import argparse, json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from placement import c7, OUT, run
from metrics import evaluate
from geometry import np

HERE = Path(__file__).resolve().parent


def verified(tag):
    out = evaluate(tag)
    if not out["landed"]:
        return dict(**out, accepted=False)
    ctl = json.loads(Path(str(tag) + "_audit.json").read_text())
    z = np.load(tag.with_suffix(".npz"))
    a = z["log"]
    t = a[:, 9]
    release = ctl.get("opening_s")
    after = t >= release if release is not None else np.zeros(len(t), bool)
    table_min = float(a[after, 6].min()) if np.any(after) else None
    out.update(opening_s=release, min_table_after_open_N=table_min)
    out["accepted"] = bool(
        out["physical"]
        and out["ball_first"]
        and out["hand_peak_N"] < 0.03
        and out["min_finger_load_before_N"] > 0.015
        and table_min is not None
        and table_min > 0.025
        and out["near_margin_min_mm"] is not None
        and out["near_margin_min_mm"] > 0
        and out["final_error_mm"] <= 2
        and out["release_peak_excursion_mm"] <= 2
    )
    return out


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--all", action="store_true")
    p.add_argument("--workers", type=int, default=4)
    a = p.parse_args()
    cfg = json.loads((HERE / "frozen.json").read_text())
    OUT.mkdir(exist_ok=True, parents=True)
    jobs = [("push_y3", "same", {**cfg, "label": "final"})]
    if a.all:
        jobs.extend((case, "same", {**cfg, "label": "final"}) for case in c7.FINAL_TEST)
        jobs.append(("push_y3", "same", {**cfg, "dt": 0.000125, "label": "halfstep"}))
    with ProcessPoolExecutor(a.workers) as pool:
        list(pool.map(run, jobs))
    rows = {}
    for case, _, options in jobs:
        tag = OUT / (case + "_" + options["label"])
        rows[tag.name] = verified(tag)
    (
        OUT / ("verification_all.json" if a.all else "verification_reference.json")
    ).write_text(json.dumps(rows, indent=2))
