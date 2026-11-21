"""Matched trials: identical mechanics, hand travel and task per comparison."""

import json
from search import batch
from motion import D


def main():
    trials = []
    for task in ("ball", "cylinder"):
        recipe = json.loads((D / f"recipe_{task}.json").read_text())
        for condition, change in [
            ("center", {}),
            ("shift", {"shift_mm": [3, 0, 0]}),
            ("low_friction", {"mu_override": 0.25}),
            ("transmission80", {"ring_efficiency": 0.8}),
            ("lag16", {"lag_ms": 16.0}),
            (
                "combined",
                {
                    "unknown_mm": [1, 0, 0],
                    "mu_override": 0.25,
                    "lag_ms": 16.0,
                    "ring_efficiency": 0.8,
                },
            ),
            ("jam", {"jam": True}),
        ]:
            for law, flags in [
                ("stroke", {"feedback": False}),
                ("guard", {"feedback": False, "legacy_guard": True}),
                ("closed", {}),
            ]:
                trials.append(
                    (f"{task}/{condition}/{law}", task, dict(recipe, **change, **flags))
                )
    batch(trials, "law_comparison")


if __name__ == "__main__":
    main()
