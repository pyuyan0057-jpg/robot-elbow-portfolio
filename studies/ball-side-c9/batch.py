"""Run one frozen controller through matched cases, retaining original solver logs."""

from concurrent.futures import ProcessPoolExecutor
import argparse, json
from control import run, RESULT, HERE, c7
from cases import CHALLENGES, INDEPENDENT, DIRECTIONS

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("folder")
    parser.add_argument("--ids", nargs="+", type=int)
    parser.add_argument("--cases", nargs="+")
    parser.add_argument("--params")
    parser.add_argument("--workers", type=int, default=6)
    parser.add_argument(
        "--suite",
        choices=["development", "original", "expanded", "blind", "directions"],
        default="development",
    )
    a = parser.parse_args()
    suites = {
        "development": list(c7.FINAL_TEST) + list(CHALLENGES),
        "original": list(c7.FINAL_TEST),
        "expanded": list(CHALLENGES),
        "blind": list(INDEPENDENT),
        "directions": list(DIRECTIONS),
    }
    cases = a.cases or suites[a.suite]
    configs = (
        [(a.folder, json.loads(open(a.params).read()))]
        if a.params
        else [
            (
                f"{a.folder}_{i}",
                next(
                    r["params"]
                    for r in json.loads((RESULT / "screen.json").read_text())
                    if r["idx"] == i
                ),
            )
            for i in a.ids
        ]
    )
    jobs = [(n, p, label) for label, p in configs for n in cases]
    with ProcessPoolExecutor(a.workers) as pool:
        for job, row in zip(jobs, pool.map(run, jobs)):
            label = job[-1]
            p = RESULT / label / "scores.json"
            rows = json.loads(p.read_text()) if p.exists() else {}
            rows[row["case"]] = row
            staging = p.with_suffix(".tmp")
            staging.write_text(json.dumps(rows, indent=1))
            staging.replace(p)
            print(label, row, flush=True)
