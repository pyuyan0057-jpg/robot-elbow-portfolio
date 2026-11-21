import sys, json, os

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
from motion import plan, D, S
import numpy as np, mujoco as mj
from physics import plant, integrate


def execute(o, c, tag=None, remote=None, observer=None, disturbance=None):
    dt = c.get("dt", 0.0005)
    period = 0.002
    sub = round(period / dt)
    assert abs(sub * dt - period) < 1e-9
    pl = plan(o, c, period)
    m, d, r, table_y, over = plant(o, pl, dt, observer.configure if observer is not None else None)
    lt, high, down, seat, release, end = pl["times"]
    from supervisor import Supervisor

    supervis = (
        Supervisor(pl["c"], pl["tape"], pl["times"], period)
        if pl["c"].get("supervised", True) and c.get("feedback", True)
        else None
    )
    sense = np.zeros(1)
    guard_dwell = 0.0
    guard_limit = None
    register = None
    frames = []
    status = []
    out = np.zeros((len(pl["tape"]), sub, 44))
    q0 = pl["entry"]
    blocked = False
    last_ring = None
    stopref = None
    stop_t = None
    for k, (t, reference) in enumerate(zip(pl["t"], pl["tape"])):
        ref = reference.copy()
        measured = sense[0]
        if c.get("sensor_fault") and high + 0.22 < t < high + 0.32:
            measured = 30.0
        if remote is not None:
            ref, blocked = remote(k, t, ref, measured, release, d.qpos[list(r.armq)])
        elif supervis is not None:
            ref, blocked = supervis.step(t, measured, d.qpos[list(r.armq)])
        else:
            blocked = False
            if c.get("legacy_guard"):
                guard_dwell = guard_dwell + period if measured > 8.0 else 0.0
                if guard_limit is None and guard_dwell >= 0.010:
                    # Reproduce the archived guard law. Tendon take-up is used only by this comparison.
                    takeup = out[k - 1, -1, 43] if k else 0.0
                    guard_limit = takeup + 3.5 / r.elastic
                if guard_limit is not None:
                    ref[17] = min(ref[17], guard_limit)
        if blocked:
            if stopref is None:
                stopref = d.qpos[list(r.armq)].copy()
                stop_t = t
            ref[:7] = stopref
            ref[7:14] = 0.0
            ref[14:19] = 0.0
        if register is None and t >= high + 0.60:
            R = d.xmat[r.palm].reshape(3, 3)
            register = dict(
                point=(R.T @ (d.xpos[r.body] - d.xpos[r.palm])).tolist(),
                axis=(R.T @ d.geom_xmat[r.ball].reshape(3, 3)[:, 2]).tolist(),
            )
        if tag and k % 20 == 0:
            frames.append(d.qpos.copy())
        if disturbance is not None:
            disturbance(d, r, t, pl["times"])
        if observer is not None:
            observer.sample(m, d, r, k, t, pl["times"])
        integrate(m, d, r, np.ascontiguousarray(ref), sub, sense, out[k])
        if tag and k % 5 == 0:
            status.append(ref.copy())
        if np.any(d.warning.number):
            raise RuntimeError("MuJoCo warning")
        if not tag and (
            t > lt + 0.2
            and (
                abs(d.xpos[r.body, 1] - table_y) > 0.8
                or np.linalg.norm(d.xpos[r.body] - d.xpos[r.palm]) > 0.6
            )
            or (blocked and t > lt + 0.5)
        ):
            out = out[: k + 1]
            break
    a = out.reshape(-1, 44)
    t = a[:, 9]
    cyl = o == "cylinder"
    h = 1000 * (
        table_y
        - a[:, 16]
        - (
            0.085 * abs(a[:, 21]) + 0.02 * np.sqrt(np.maximum(0, 1 - a[:, 21] ** 2))
            if cyl
            else 0.02
        )
    )
    rest = (t > 0.05) & (t < 0.16)
    hold = (t > high + 0.08) & (t < down)
    placed = t > end - 0.25
    air = (h > 10) & (t < release)
    touch = np.flatnonzero((a[:, 6] > 0.03) & (t > down) & (h < 2))
    seated = bool(
        touch.size and t[touch[0]] >= seat - 0.55 and t[touch[0]] <= release + 0.55
    )
    released = bool(
        placed.any()
        and np.min(a[placed, 6]) > 0.28
        and np.max(a[placed, :6].sum(1)) < 0.025
        and np.max(abs(h[placed])) < 1.5
        and np.max(np.linalg.norm(a[placed, 24:27], axis=1)) < 0.025
    )
    # A falling object must not qualify merely by landing under a nearby hand.
    holdspan = np.ptp(a[hold, 18:21], axis=0).max() * 1000 if hold.any() else 999.0
    okhold = bool(
        hold.any()
        and min(h[hold]) > 35
        and max(abs(np.degrees(a[hold, 38]) - 100)) < 1
        and holdspan < 2
        and min(a[hold, :5].sum(1)) > 0.025
    )
    keep = bool(air.any() and np.min(a[air, :5].sum(1)) > 0.015)
    start = bool(
        rest.any()
        and min(a[rest, 6]) > 0.28
        and max(a[rest, :5].sum(1)) < 0.03
        and over < 0.00015
    )
    # Contact with the tabletop before descent, after initial acquisition, indicates a failed lift.
    no_retouch = True
    if air.any():
        i = np.where(air)[0][0]
        seg = (t > t[i]) & (t < down)
        if seg.any():
            no_retouch = max(a[seg, 6]) < 0.025
    orient = (
        bool(not cyl or np.min(a[(t > 0.85) & (t < release), 22]) > 0.015)
        if (t > 0.85).any()
        else False
    )
    maxerr = np.max(
        np.abs(
            np.degrees(a[:, 35:42] - np.repeat(pl["tape"][: len(out), :7], sub, axis=0))
        ),
        axis=0,
    )
    result = dict(
        task=o,
        case=c,
        passed=bool(
            start
            and okhold
            and keep
            and seated
            and released
            and no_retouch
            and orient
            and not blocked
        ),
        rest=start,
        initial_overlap_mm=over * 1000,
        hold=okhold,
        hold_span_mm=holdspan,
        hold_height_mm=np.mean(h[hold]) if hold.any() else None,
        peak_height_mm=max(h),
        retained=keep,
        no_retouch=bool(no_retouch),
        reseated=seated,
        released=released,
        thumb_up=orient,
        blocked=blocked,
        stop_s=stop_t,
        peak_tension=a[:, 10:15].max(0).tolist(),
        peak_torque=np.max(abs(a[:, 28:35]), axis=0).tolist(),
        peak_joint_error_deg=maxerr.tolist(),
        table_hand_N=max(a[:, 8]),
        land_t=t[touch[0]] if touch.size else None,
        hold_side_mm=min(a[hold, 22]) * 1000 if hold.any() else None,
        object_end_offset_mm=(1000 * (a[-1, 15:18] - pl["object_pos"])).tolist(),
    )
    result["fault"] = (
        supervis.fault if remote is None and supervis is not None else None
    )
    result["event_s"] = (
        supervis.event if remote is None and supervis is not None else None
    )
    result["touchdown_speed_mm_s"] = (
        float(1000 * abs(a[max(0, touch[0] - 1), 25])) if touch.size else None
    )
    result["registration"] = register
    result["final_upright_deg"] = (
        float(np.degrees(np.arccos(min(1.0, abs(a[-1, 21]))))) if cyl else None
    )
    result["placed_speed_mm_s"] = (
        float(1000 * np.max(np.linalg.norm(a[placed, 24:27], axis=1)))
        if placed.any()
        else None
    )
    result["placement_error_mm"] = float(
        np.linalg.norm(
            (a[-1, 15:18] - pl.get("place_target", pl["object_pos"]))[[0, 2]]
        )
        * 1000
    )
    if tag:
        (D / (tag + ".json")).write_text(json.dumps(result, indent=1))
        np.savez_compressed(
            D / (tag + ".npz"),
            log=a[:: max(1, round(0.002 / dt))],
            q=frames,
            command=status,
            commands_time=pl["t"][: len(out) : 5],
            height=h[:: max(1, round(0.002 / dt))],
            times=pl["times"],
        )
    return result


def mini(r):
    return {
        k: r[k]
        for k in (
            "passed",
            "rest",
            "hold",
            "hold_span_mm",
            "peak_height_mm",
            "retained",
            "reseated",
            "released",
            "thumb_up",
            "blocked",
            "hold_side_mm",
            "land_t",
            "object_end_offset_mm",
        )
    }


if __name__ == "__main__":
    raise SystemExit(
        "Use reproduce.py for current cases; early calibration is archived."
    )
