"""Solve, compare and check the archived cylinder task. All traces are fresh simulations."""
import csv
import json
import os
from pathlib import Path
import sys

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

STUDY = Path(__file__).resolve().parent
ENGINE = STUDY.parent / "seven-axis-pick-place-c2"
OUTPUT = STUDY.parents[1] / "outputs" / "cylinder-control"
sys.path.insert(0, str(ENGINE))
from build_runtime import build
from motion import hand, plan


def solve(label, config):
    from trial import execute
    tag = str(OUTPUT / label)
    result = execute("cylinder", config, tag=tag)
    with np.load(tag + ".npz") as run:
        log = run["log"]
    path = plan("cylinder", config, .002)
    times = log[:, 9]
    indices = np.rint(times / .002).astype(int)
    desired = [hand(q)[0] for q in path["tape"][indices, :7]]
    actual = [hand(q)[0] for q in log[:, 35:42]]
    error = 1000 * np.linalg.norm(np.subtract(actual, desired), axis=1)
    metrics = dict(passed=bool(result["passed"]), duration_s=path["times"][-1],
                   peak_palm_mm=float(error.max()), rms_palm_mm=float(np.sqrt(np.mean(error**2))),
                   peak_time_s=float(times[error.argmax()]), placement_mm=result["placement_error_mm"],
                   hold_excursion_mm=result["hold_span_mm"], peak_torque_Nm=result["peak_torque"],
                   peak_tendon_N=result["peak_tension"])
    return metrics, times, error


def main():
    OUTPUT.mkdir(exist_ok=True, parents=True)
    build()
    base = json.loads((ENGINE / "recipe_cylinder.json").read_text())
    changes = json.loads((STUDY / "cylinder_changes.json").read_text())
    metrics, traces = {}, {}
    for label, config in [("C2", base), ("C4", {**base, **changes})]:
        metrics[label], times, traces[label] = solve(label, config)
        if label == "C2":
            reference_times = times.copy()
        elif not np.array_equal(times, reference_times):
            raise RuntimeError("Baseline and tuned sample times differ.")
        row = metrics[label]
        print(f"{label}: pass={row['passed']}, peak={row['peak_palm_mm']:.3f} mm, "
              f"RMS={row['rms_palm_mm']:.3f} mm, placement={row['placement_mm']:.3f} mm", flush=True)
    (OUTPUT / "summary.json").write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")
    with (OUTPUT / "tracking.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["sim_time_s", "c2_palm_error_mm", "c4_palm_error_mm"])
        writer.writerows(zip(times, traces["C2"], traces["C4"]))
    fig, ax = plt.subplots(figsize=(8.7, 3.4), constrained_layout=True)
    for label, color in [("C2", "#677785"), ("C4", "#00746f")]:
        ax.plot(times, traces[label], color=color, lw=1.7,
                label=f"{label}    peak {metrics[label]['peak_palm_mm']:.2f} mm")
    for t, name in zip(plan("cylinder", base, .002)["times"][:3], ["Lift", "Hold", "Return"]):
        ax.axvline(t, color="#a9b7ba", lw=.8, ls=":")
        ax.text(t + .05, .78, name, transform=ax.get_xaxis_transform(), color="#425958", fontsize=9)
    ax.set(xlabel="Simulation time (s)", ylabel="Palm position error (mm)", ylim=(0,None),
           title="Cylinder pick and place | same 9.47 s cycle")
    ax.grid(axis="y", alpha=.18)
    ax.spines[["right", "top"]].set_visible(False)
    ax.legend(loc="upper right", frameon=False)
    fig.savefig(OUTPUT / "tracking.png", dpi=145)
    plt.close(fig)
    # Nominal references are independent recorded ROS runs. Leave modest numerical
    # tolerance for platform/solver differences, especially after free contact.
    reference = json.loads((STUDY / "evidence" / "cylinder_reference.json").read_text())
    tolerances = {"duration_s":1e-8, "peak_palm_mm":.15, "rms_palm_mm":.08, "placement_mm":.6}
    for label, row in metrics.items():
        if not row["passed"]:
            raise RuntimeError(f"{label}: physical pick/hold/reseat/release checks failed")
        for key, tol in tolerances.items():
            if not np.isclose(row[key], reference[label][key], atol=tol, rtol=0):
                raise RuntimeError(f"{label}: {key}={row[key]} outside reference tolerance {tol}")
    if not all(metrics["C4"][k] < metrics["C2"][k] for k in ("peak_palm_mm","rms_palm_mm","placement_mm")):
        raise RuntimeError("Expected improvements in tracking and placement were not reproduced")
    print(f"PASS. Generated JSON, CSV, PNG and raw state archives in {OUTPUT}")


if __name__ == "__main__":
    main()
