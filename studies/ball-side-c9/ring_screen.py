"""Allow the ring finger to open into a lateral guard. Screen all six directions."""

from concurrent.futures import ProcessPoolExecutor
from all_directions import investigate, np, json, RESULT

if __name__ == "__main__":
    seeds = [json.loads((RESULT / "norm240.json").read_text())] + [
        r["params"] for r in json.loads((RESULT / "axis_sweep.json").read_text())[:4]
    ]
    rng = np.random.default_rng(45671)
    jobs = []
    for i in range(125):
        p = dict(seeds[i % len(seeds)])
        mm = np.array(p["mm"]) + rng.uniform([-1, -1, -1, -18, -1], [1, 1, 1, -0.3, 1])
        p["mm"] = mm.round(2).tolist()
        jobs.append((i, p))
    with ProcessPoolExecutor(5) as pool:
        rows = list(pool.map(investigate, jobs))
    (RESULT / "ring_sweep.json").write_text(
        json.dumps(sorted(rows, key=lambda x: x["rank"]), indent=1)
    )
