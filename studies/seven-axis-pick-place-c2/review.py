import json
from pathlib import Path

D = Path(__file__).parent


def records(key):
    return [
        json.loads(l)
        for l in (D / (key + ".jsonl")).read_text().splitlines()
        if l.strip()
    ]


def best(key):
    pool = records(key)
    return sorted(
        pool,
        key=lambda r: (
            r.get("passed", False),
            r.get("hold", False) and r.get("retained", False),
            r.get("reseated", False),
            r.get("hold", False),
            -abs(r.get("hold_span_mm", 999.0)),
        ),
    )[::-1][0]


if __name__ == "__main__":
    for key in ["ball", "cylinder"]:
        b = best(key)
        print(key, b["id"], b["case"], b["passed"])
