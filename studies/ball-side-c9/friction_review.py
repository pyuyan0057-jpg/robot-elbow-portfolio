"""Matched sliding-friction comparisons, with zero-difference original-material controls."""

import csv, json
from cases import INDEPENDENT
from exit_path import RESULT, HERE, SIDES, np
from touch_audit_c9 import record


def main():
    out = HERE / "evidence"
    checks = {}
    for name in ("oblique_1_1", "oblique_6_2", "v_side_h"):
        base = (
            RESULT / ("final" if name.startswith("v_") else "blindA") / (name + ".npz")
        )
        with np.load(base) as a, np.load(RESULT / "mu30" / (name + ".npz")) as b:
            checks[name] = float(np.max(abs(a["log"] - b["log"])))
            assert checks[name] == 0
    rows = []
    stats = {}
    for group, cases in [("blind", list(INDEPENDENT)), ("side", ["v_quiet", *SIDES])]:
        for mu in (30, 45, 60, 90):
            folder = (
                {"blind": "blindA", "side": "final"}[group]
                if mu == 30
                else ("mu" if group == "blind" else "muSides") + str(mu)
            )
            scores = json.loads((RESULT / folder / "scores.json").read_text())
            for n in cases:
                s = record(RESULT / folder / n)
                if mu != 30:
                    actual = scores[n]["effective_friction"]
                    assert (
                        actual["hand"][:2] == [mu / 100] * 2
                        and actual["table"][:2] == [0.45] * 2
                    )
                rows.append(
                    dict(
                        group=group,
                        mu=mu / 100,
                        case=n,
                        held=s["held"],
                        error_mm=s["error_mm"],
                        passed=s["passed"],
                        strict=s["continuous"],
                        loss_s=s["loss_s"],
                        touch_s=s["touchdown_s"],
                    )
                )
            sub = [r for r in rows if r["group"] == group and r["mu"] == mu / 100]
            stats[f"{group}/{mu}"] = {
                k: sum(r[k] for r in sub) for k in ("held", "passed", "strict")
            }
    with (out / "friction_trials.csv").open("w", encoding="utf-8-sig", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    (out / "friction_summary.json").write_text(
        json.dumps(dict(checks=checks, summary=stats), indent=2)
    )
    print(stats)


if __name__ == "__main__":
    main()
