from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import os

os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"
import numpy as np, json, sys
from trial import execute
from motion import S, D


def run(arg):
    i, o, c = arg
    try:
        return dict(id=i, **execute(o, c))
    except ValueError as e:
        return dict(id=i, case=c, error=str(e))


def batch(tasks, name):
    with (
        (D / (name + ".jsonl")).open("w") as out,
        ProcessPoolExecutor(max_workers=6) as pool,
    ):
        for r in pool.map(run, tasks):
            out.write(json.dumps(r) + "\n")
            out.flush()
            if r.get("hold") or r.get("passed"):
                print(
                    r["id"],
                    r.get("passed"),
                    r.get("hold"),
                    r.get("released"),
                    flush=True,
                )


if __name__ == "__main__":
    rng = np.random.default_rng(443)
    mode = sys.argv[1]
    tasks = []
    if mode == "ball":
        for i in range(54):
            dx, dz = rng.uniform(-1.8, 1.8, 2) if i > 10 else (0, 0)
            c = dict(
                trim_mm=float(rng.choice([0, 1.5, 2.5, 3.5, 4.5, 5.0])),
                target_N=float(rng.choice([5.0, 6.0, 7.5, 9.0])),
                unknown_mm=[dx, 0, dz],
                early_before=0.65,
                seat_mm=0,
            )
            tasks.append((i, "ball", c))
    elif mode == "cylinder":
        old = S["initial"]["cylinder"]["cfg"]
        for i in range(160):
            center = np.array(old["center"]) + rng.uniform(
                [-0.024, -0.035, -0.022], [0.018, 0.035, 0.022]
            )
            c = dict(
                center=center.tolist(),
                swing=float(rng.uniform(-7, 1)),
                delay=rng.uniform(0.50, 0.84, 5).tolist(),
                target_N=float(rng.uniform(5, 14)),
                early_before=0.47,
                extra_mm=rng.uniform(-2, 4, 5).tolist(),
                end_op=float(rng.uniform(19, 33)),
                close_s=float(rng.uniform(0.63, 1)),
                dt=0.001,
            )
            tasks.append((i, "cylinder", c))
    else:
        tasks = json.loads((D / (mode + "_input.json")).read_text("utf-8-sig"))
    batch(tasks, mode)
