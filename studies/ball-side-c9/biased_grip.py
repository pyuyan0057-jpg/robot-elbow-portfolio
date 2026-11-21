"""Search the grip adjustment for an initially high thumb-pad load."""

from c9_screen import score, RESULT, np, json, ProcessPoolExecutor


def assess(job):
    i, p = job
    values = []
    for case in ["v_heavy_z", "v_three"]:
        values.append(score(case, p))
        if values[-1] > 100:
            break
    return dict(
        idx=i,
        params=p,
        scores=values,
        rank=max(values) + sum(values) * 0.3 + (2 - len(values)) * 32,
    )


if __name__ == "__main__":
    rng = np.random.default_rng(66351)
    jobs = []
    for i in range(360):
        vals = rng.uniform([-5, -3, -4, 0, -4], [6.0, 7.0, 5.0, 0, 8.0]).round(2)
        p = dict(
            mm=vals.tolist(),
            deg=round(rng.uniform(-17, 2.8), 2),
            relax=5.85,
            unwind=1.0,
        )
        jobs.append((i, p))
    with ProcessPoolExecutor(6) as pool:
        rows = []
        for row in pool.map(assess, jobs):
            rows.append(row)
            if len(rows) % 10 == 0:
                best = sorted(rows, key=lambda r: r["rank"])
                (RESULT / "biased.json").write_text(json.dumps(best, indent=1))
                print(len(rows), best[0], flush=True)
