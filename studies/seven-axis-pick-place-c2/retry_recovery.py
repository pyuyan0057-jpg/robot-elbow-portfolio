"""Optional supervised retry after a cleared finger jam. Preserve the actual plant state."""

import json
import numpy as np
from motion import D, plan, ease
from physics import plant, integrate
from supervisor import Supervisor


def run(task):
    recipe = json.loads((D / f"recipe_{task}.json").read_text())
    period = 0.002
    path = plan(task, recipe, period)
    model, data, reg, table_y, _ = plant(task, path, 0.00025)
    joints = [model.joint(f"q3{k}") for k in range(3)]
    saved = [model.jnt_range[j.id].copy() for j in joints]
    for j in joints:
        start = data.qpos[j.qposadr[0]]
        model.jnt_range[j.id] = [start - 0.0005, start + 0.0005]
    tape = path["tape"]
    law = Supervisor(recipe, tape, path["times"], period)
    sensor = np.zeros(1)
    step = np.empty((8, 44))
    clock = 0.0
    stopped = None
    retry = None
    out = []
    for k in range(len(tape) + 1600):
        now = k * period
        q = data.qpos[list(reg.armq)].copy()
        if stopped is not None and retry is None:
            since = now - stopped
            ref = last.copy()
            ref[:7] = stop_q + (path["entry"] - stop_q) * ease((since - 0.35) / 0.8)
            ref[7:14] = 0.0
            ref[14:19] = 0.0
            if since >= 0.35:
                for j, limits in zip(joints, saved):
                    model.jnt_range[j.id] = limits
            if since >= 1.25:
                delta = np.linalg.norm(data.xpos[reg.body] - path["object_pos"]) * 1000
                retry = now
                clock = 0.0
                law = Supervisor(recipe, tape, path["times"], period)
        else:
            ref, abort = law.step(clock, sensor[0], q)
            if abort:
                if retry is not None:
                    return dict(
                        task=task,
                        initial_stop_s=stopped,
                        retry_s=retry,
                        passed=False,
                        reason="blocked again",
                    )
                stopped = now
                stop_q = q.copy()
                last = ref.copy()
            clock += period
        integrate(model, data, reg, np.ascontiguousarray(ref), 8, sensor, step)
        if retry is not None:
            out.append(step[-1].copy())
            if clock >= path["times"][-1]:
                break
    if retry is None:
        return dict(task=task, passed=False, reason="jam was not detected")
    a = np.array(out)
    t = a[:, 9] - retry
    high, down = path["times"][1:3]
    hold = (t > high + 0.08) & (t < down)
    height = 1000 * (
        table_y
        - a[:, 16]
        - (
            0.02
            if task == "ball"
            else 0.085 * abs(a[:, 21])
            + 0.02 * np.sqrt(np.maximum(0, 1 - a[:, 21] ** 2))
        )
    )
    final = t > path["times"][-1] - 0.25
    kept = bool(
        hold.any()
        and min(height[hold]) > 35
        and min(a[hold, :5].sum(1)) > 0.025
        and max(abs(np.degrees(a[hold, 38]) - 100)) < 1
    )
    release = bool(
        final.any()
        and min(a[final, 6]) > 0.28
        and max(a[final, :6].sum(1)) < 0.025
        and max(abs(height[final])) < 1.5
    )
    return dict(
        task=task,
        initial_stop_s=stopped,
        retry_s=retry,
        object_shift_before_retry_mm=delta,
        hold=kept,
        released=release,
        passed=kept and release and law.fault is None,
    )


if __name__ == "__main__":
    results = [run(task) for task in ("ball", "cylinder")]
    (D / "cleared_jam_retry.json").write_text(json.dumps(results, indent=2), "utf-8")
