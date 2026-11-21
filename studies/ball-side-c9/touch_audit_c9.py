"""Paired acceptance up to first table touch only. The same rules apply to both policies."""

import csv, json
from pathlib import Path
from exit_path import RESULT, OUT, HERE, C8, SIDES, c7, np, plan
from control import grade_tag
from cases import CHALLENGES, parameters


def record(tag):
    cfg = json.loads(tag.with_suffix(".json").read_text())["case"]
    target = plan("ball", cfg, 0.002)["place_target"]
    row = grade_tag(tag, target)
    with np.load(tag.with_suffix(".npz")) as z:
        a = z["log"]
        time = a[:, 9]
        lift, high, down = z["times"][:3]
        flight = np.flatnonzero((time >= lift) & (z["height"] > 10))
        if not flight.size:
            return {**row, "continuous": False, "zero_gap_ms": None}
        start = (
            np.flatnonzero((np.arange(len(a)) < flight[0]) & (a[:, 6] > 0.03))[-1] + 1
        )
        stop = (
            np.argmin(abs(time - row["touchdown_s"]))
            if row["touchdown_s"] is not None
            else len(time)
        )
        load = a[start:stop, :6].sum(1)
        zero = load <= 0.001
        transition = np.diff(np.r_[False, zero, False].astype(int))
        runs = np.flatnonzero(transition == -1) - np.flatnonzero(transition == 1)
        gap = int(max(runs, default=0)) * 2
        row.update(
            continuous=bool(row["passed"] and gap == 0),
            zero_gap_ms=gap,
            min_hand_contact_N=float(min(load)) if len(load) else None,
        )
        if row["touchdown_s"] is not None:
            row["impact_xz_mm"] = ((a[stop, [15, 17]] - target[[0, 2]]) * 1000).tolist()
    return row


def main():
    evidence = HERE / "evidence"
    evidence.mkdir(exist_ok=True)
    flat = []
    details = {}
    for name in list(c7.FINAL_TEST) + list(CHALLENGES):
        old = (
            OUT / (name + "_final")
            if name in c7.FINAL_TEST
            else RESULT / "c8_new" / name
        )
        new = RESULT / "final" / name
        base = record(old)
        candidate = record(new)
        cfg, force, off, duration, seed = parameters(name)
        group = "original" if name in c7.FINAL_TEST else "new"
        details[name] = dict(
            group=group,
            force_xyz_N=force.tolist(),
            offset_s=off,
            duration_s=duration,
            shift_mm=cfg["unknown_mm"],
            noise_seed=seed,
            baseline=base,
            candidate=candidate,
        )
        flat.append(
            [
                group,
                name,
                base["held"],
                base["error_mm"],
                base["passed"],
                candidate["held"],
                candidate["error_mm"],
                candidate["passed"],
                candidate["continuous"],
                candidate["zero_gap_ms"],
            ]
        )
    with (evidence / "paired.csv").open("w", encoding="utf-8-sig", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "组别",
                "工况",
                "C8持至触桌",
                "C8首次触桌偏差_mm",
                "C8通过",
                "C9持至触桌",
                "C9首次触桌偏差_mm",
                "C9通过",
                "C9全程有手部接触且通过",
                "C9最长无手部载荷_ms",
            ]
        )
        w.writerows(flat)
    (evidence / "paired.json").write_text(json.dumps(details, indent=2))
    tree = json.loads((RESULT / "exit_2ms.json").read_text())
    (evidence / "escape.json").write_text(
        json.dumps(
            [{k: v for k, v in case.items() if k != "series"} for case in tree],
            indent=2,
        )
    )
    for g in ["original", "new"]:
        rows = [r for r in details.values() if r["group"] == g]
        for which in ("baseline", "candidate"):
            selected = [r[which] for r in rows]
            print(
                g,
                which,
                len(rows),
                "held",
                sum(r["held"] for r in selected),
                "passed",
                sum(r["passed"] for r in selected),
                "continuous",
                sum(r["continuous"] for r in selected),
            )
        for r in rows:
            a = r["candidate"]
            print(
                a["case"],
                a["held"],
                round(a["error_mm"], 2) if a["error_mm"] is not None else None,
                a["zero_gap_ms"],
            )


if __name__ == "__main__":
    main()
