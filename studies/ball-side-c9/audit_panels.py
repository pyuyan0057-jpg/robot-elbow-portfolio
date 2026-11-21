"""Summarize held-to-touch and horizontal impact error from the original state logs."""

import csv
import json
from pathlib import Path

from cases import parameters, c7, np, CHALLENGES, INDEPENDENT, DIRECTIONS
from exit_path import HERE, RESULT, OUT
from touch_audit_c9 import record


def audit():
    suites = {
        "original": (c7.FINAL_TEST, OUT, RESULT / "final"),
        "expanded": (CHALLENGES, RESULT / "c8_new", RESULT / "final"),
        "blind": (INDEPENDENT, RESULT / "blindC8", RESULT / "blindA"),
        "directions": (DIRECTIONS, None, RESULT / "directional"),
        "light": (DIRECTIONS, None, RESULT / "light_panel"),
        "no_push": (INDEPENDENT, None, RESULT / "blind_no_push"),
    }
    rows = []
    for group, (cases, baseline, current) in suites.items():
        for case in cases:
            cfg, f, delay, duration, seed = parameters(case)
            if group == "light":
                f = f / 6
            if group == "no_push":
                f = np.zeros(3)
            for model, folder in (("C8", baseline), ("C9", current)):
                if folder is None:
                    continue
                suffix = "_final" if group == "original" and model == "C8" else ""
                score = record(folder / (case + suffix))
                impact = score.get("impact_xz_mm", [None, None])
                rows.append(
                    dict(
                        panel=group,
                        case=case,
                        controller=model,
                        peak_N=round(float(np.linalg.norm(f)), 4),
                        force_X_N=f[0],
                        force_Y_N=f[1],
                        force_Z_N=f[2],
                        delay_from_high_s=delay,
                        duration_s=duration,
                        start_dx_mm=cfg["unknown_mm"][0],
                        start_dz_mm=cfg["unknown_mm"][2],
                        seed=seed,
                        held_to_touch=score["held"],
                        touch_error_mm=score["error_mm"],
                        touch_dx_mm=impact[0],
                        touch_dz_mm=impact[1],
                        passed=score["passed"],
                        strict=score["continuous"],
                        zero_gap_ms=score["zero_gap_ms"],
                        touch_s=score["touchdown_s"],
                        loss_s=score["loss_s"],
                    )
                )
    evidence = HERE / "evidence"
    with (evidence / "all_trials.csv").open(
        "w", encoding="utf-8-sig", newline=""
    ) as file:
        w = csv.DictWriter(file, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    stat = {}
    for group in suites:
        for model in ("C8", "C9"):
            sub = [x for x in rows if x["panel"] == group and x["controller"] == model]
            if not sub:
                continue
            categories = [("all", sub)]
            if group == "blind":
                categories += [
                    (str(mag), [x for x in sub if x["peak_N"] == mag])
                    for mag in (0.6, 2.5, 5.0)
                ]
            if group in ("directions", "light"):
                categories += [
                    (name, [x for x in sub if x["case"].endswith("_" + str(i))])
                    for i, name in enumerate(("lift", "hold", "return"), 1)
                ]
            for key, part in categories:
                errs = [x["touch_error_mm"] for x in part if x["held_to_touch"]]
                stat[f"{group}/{model}/{key}"] = dict(
                    n=len(part),
                    held=sum(x["held_to_touch"] for x in part),
                    passed=sum(x["passed"] for x in part),
                    strict=sum(x["strict"] for x in part),
                    error_held_mm=[min(errs), max(errs)] if errs else [],
                )
    (evidence / "panel_summary.json").write_text(json.dumps(stat, indent=2))
    print(json.dumps(stat, indent=1))


if __name__ == "__main__":
    audit()
