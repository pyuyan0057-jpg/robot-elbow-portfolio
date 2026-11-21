from c9_screen import screen, RESULT, np, json, ProcessPoolExecutor

if __name__ == "__main__":
    rng = np.random.default_rng(77578)
    jobs = []
    for i in range(320):
        vals = rng.uniform([-0.75, 3.0, 0.1, 0, -0.5], [0.95, 8.0, 2.9, 0, 6.5]).round(
            2
        )
        jobs.append(
            (
                1000 + i,
                dict(
                    mm=vals.tolist(),
                    deg=round(rng.uniform(-6.0, 2.8), 2),
                    relax=round(rng.uniform(5.4, 6.4), 2),
                    unwind=round(rng.uniform(0.6, 1.4), 2),
                ),
            )
        )
    raw = json.loads((RESULT / "screen.json").read_text())
    rows = []
    with ProcessPoolExecutor(8) as pool:
        for row in pool.map(screen, jobs):
            rows.append(row)
            if len(rows) % 10 == 0:
                (RESULT / "screen.json").write_text(
                    json.dumps(sorted(raw + rows, key=lambda z: z["rank"]), indent=1)
                )
                print(len(rows), min(rows, key=lambda r: r["rank"]), flush=True)
