"""Exact paired 0.30 and 0.45 sweep with unchanged sphere, table and controller."""

import csv, json
from cases import c7, parameters, INDEPENDENT, CHALLENGES, DIRECTIONS, np
from exit_path import RESULT, HERE
from touch_audit_c9 import record


def main(include_light=False):
    rows = []
    overview = {}
    panels = [
        ("original", c7.FINAL_TEST, "final", "muSides45"),
        ("expanded", CHALLENGES, "final", "muSides45"),
        ("blind", INDEPENDENT, "blindA", "mu45"),
        ("directional", DIRECTIONS, "directional", "direction45"),
    ]
    if include_light:
        panels.append(("light", DIRECTIONS, "light_panel", "light45"))
    for label, cases, old, new in panels:
        acceptance = json.loads((RESULT / new / "scores.json").read_text())
        for name in cases:
            d = acceptance[name]["effective_friction"]
            assert d["hand"][:2] == [0.45, 0.45] and d["table"][:2] == [0.45, 0.45]
            b = record(RESULT / old / name)
            n = record(RESULT / new / name)
            cfg, f, delay, duration, seed = parameters(name)
            if label == "light":
                f = f / 6
            rows.append(
                dict(
                    panel=label,
                    case=name,
                    peak_N=round(float(np.linalg.norm(f)), 4),
                    force_X=f[0],
                    force_Y=f[1],
                    force_Z=f[2],
                    offset_from_high_s=delay,
                    baseline_held=b["held"],
                    baseline_error_mm=b["error_mm"],
                    baseline_pass=b["passed"],
                    baseline_strict=b["continuous"],
                    new_held=n["held"],
                    new_error_mm=n["error_mm"],
                    new_pass=n["passed"],
                    new_strict=n["continuous"],
                    new_low_load_gap_ms=n["zero_gap_ms"],
                    new_loss_s=n["loss_s"],
                    new_touch_s=n["touchdown_s"],
                )
            )
        sub = [r for r in rows if r["panel"] == label]

        def summarize(part):
            return dict(
                n=len(part),
                totals={
                    key: sum(row[key] for row in part)
                    for key in (
                        "baseline_held",
                        "new_held",
                        "baseline_pass",
                        "new_pass",
                        "new_strict",
                    )
                },
                gained=sum(r["new_pass"] and not r["baseline_pass"] for r in part),
                lost=sum(r["baseline_pass"] and not r["new_pass"] for r in part),
            )

        overview[label] = summarize(sub)
        if label == "blind":
            for mag in (0.6, 2.5, 5.0):
                overview["blind_" + str(mag)] = summarize(
                    [r for r in sub if r["peak_N"] == mag]
                )
    with (HERE / "evidence/matched_045.csv").open(
        "w", encoding="utf-8-sig", newline=""
    ) as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    (HERE / "evidence/matched_045.json").write_text(json.dumps(overview, indent=2))
    print(overview)


if __name__ == "__main__":
    import sys

    main("--include-light" in sys.argv)
