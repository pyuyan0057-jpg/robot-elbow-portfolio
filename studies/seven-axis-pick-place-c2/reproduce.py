"""Recompute the two reference pick-and-place cases without starting ROS middleware."""

import argparse
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", choices=("both", "ball", "cylinder"), default="both")
    parser.add_argument("--shift-x-mm", type=float, default=0.0)
    parser.add_argument("--shift-z-mm", type=float, default=0.0)
    parser.add_argument(
        "--elbow-max-deg",
        type=float,
        help="User elbow upper limit, clipped to mechanical range.",
    )
    parser.add_argument(
        "--speed-limit-deg-s",
        type=float,
        help="Additional speed cap on every arm joint.",
    )
    parser.add_argument("--output", type=Path, default=ROOT / "recomputed.json")
    args = parser.parse_args()
    if not (ROOT / ("dynamics.dll" if os.name == "nt" else "dynamics.so")).is_file():
        from build_runtime import build

        build()
    from trial import execute

    tasks = ("ball", "cylinder") if args.task == "both" else (args.task,)
    results = []
    for task in tasks:
        config = json.loads((ROOT / f"recipe_{task}.json").read_text())
        config["shift_mm"] = [args.shift_x_mm, 0.0, args.shift_z_mm]
        if args.elbow_max_deg is not None:
            ranges = json.loads((ROOT / "settings.json").read_text())["limits_deg"]
            ranges[3][1] = min(ranges[3][1], args.elbow_max_deg)
            config["user_deg"] = ranges
        if args.speed_limit_deg_s is not None:
            config["speed_limit_deg_s"] = [
                min(x, args.speed_limit_deg_s)
                for x in (125, 125, 125, 140, 160, 110, 110)
            ]
        try:
            result = execute(task, config)
        except ValueError as exc:
            results.append(dict(task=task, passed=False, refused=str(exc)))
            print(f"{task}: {exc}")
            continue
        results.append(result)
        print(
            f"{task}: passed={result['passed']}  hold={result['hold_height_mm']:.1f} mm  placement_error={result['placement_error_mm']:.1f} mm"
        )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(results, indent=2), encoding="utf-8")
    return 0 if all(row["passed"] for row in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
