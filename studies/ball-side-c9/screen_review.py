"""Compare finalists from the high-hold six-axis search using full pickup-to-touch trials."""

import json
from exit_path import RESULT, HERE


def main():
    axis = json.loads((RESULT / "axis_sweep.json").read_text())
    ring = json.loads((RESULT / "ring_sweep.json").read_text())
    data = {}
    for folder, short in [
        ("axis541", axis[0]),
        ("axis290", axis[1]),
        ("ring28", ring[0]),
    ]:
        rows = json.loads((RESULT / folder / "scores.json").read_text())
        data[folder] = {
            "setup": short,
            "full_trials": {
                name: {k: v[k] for k in ("held", "passed", "error_mm")}
                for name, v in rows.items()
            },
        }
    sample = list(next(iter(data.values()))["full_trials"])
    orig = json.loads((RESULT / "final/scores.json").read_text())
    report = {
        "six_axis_screen": {
            "ordinary_n": len(axis),
            "ring_release_n": len(ring),
            "directions": ["+X", "+Y", "+Z", "-X", "-Y", "-Z"],
            "peak_N": 5.0,
            "duration_s": 0.18,
        },
        "alternatives": data,
        "retained_candidate": {
            name: {k: orig[name][k] for k in ("held", "passed", "error_mm")}
            for name in sample
        },
    }
    (HERE / "evidence/screen_review.json").write_text(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
