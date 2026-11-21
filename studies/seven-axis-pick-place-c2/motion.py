from pathlib import Path
import numpy as np, json
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R

D = Path(__file__).resolve().parent
S = json.loads((D / "settings.json").read_text())
lo, hi = np.deg2rad(S["limits_deg"]).T


def hand(q):
    a, b, c, e, r, f, g = q
    sh = R.from_euler("XZ", [a, -b])
    sw = R.from_rotvec([0, -c, 0])
    el = R.from_rotvec([e, 0, 0])
    roll = R.from_rotvec([0, r, 0])
    flex = R.from_rotvec([f, 0, 0])
    dev = R.from_rotvec([0, 0, g])
    fore = sh * sw * el * roll
    pos = (
        np.array([0, -0.36, 0])
        + sh.apply([0, 0.36, 0])
        + fore.apply([0, 0.306, 0])
        + (fore * flex).apply([0, 0.023, 0])
        + (fore * flex * dev).apply([0.000001149, 0.130000019, 0.000000020])
    )
    return pos, fore * flex * dev


def ik(q, delta, limits):
    p, ori = hand(q)
    target = p + delta

    def residual(x):
        v, r = hand(x)
        return np.r_[(v - target) * 10, (ori.inv() * r).as_rotvec(), 0.003 * (x - q)]

    if np.linalg.norm(delta) == 0:
        return q.copy()
    out = least_squares(
        residual,
        np.clip(q, limits[0] + 1e-9, limits[1] - 1e-9),
        bounds=limits,
        ftol=1e-10,
        gtol=1e-10,
        xtol=1e-10,
    )
    pos, rot = hand(out.x)
    if np.linalg.norm(pos - target) > 0.0003 or (ori.inv() * rot).magnitude() > 0.002:
        raise ValueError("target outside configured joint workspace")
    return out.x


def ease(x):
    u = np.clip(x, 0, 1)
    return u**3 * (10 + u * (-15 + 6 * u))


def plan(o, settings, period):
    c = S["initial"][o]["cfg"].copy()
    c.update(settings)
    cyl = o == "cylinder"
    limits = (lo.copy(), hi.copy())
    if "user_deg" in c:
        ul, uh = np.deg2rad(c["user_deg"]).T
        limits = (np.maximum(lo, ul), np.minimum(hi, uh))
    base = np.deg2rad(
        [
            c["sh"],
            c["out"],
            c.get("sweep_deg", 0),
            0 if cyl else 45,
            c["roll"],
            c["fx"],
            c["dv"],
        ]
    )
    p, r = hand(base)
    wp = p + r.apply(c["center"])
    delta = np.array(c.get("shift_mm", [0, 0, 0])) * 0.001
    requested = (
        delta * c.get("approach_fraction", 1.0)
        + np.array(c.get("pose_adjust_mm", [0, 0, 0])) * 0.001
    )
    try:
        q = ik(base, requested, limits)
    except ValueError:
        # Near full elbow extension: match the object location in the hand, keep initial
        # elbow fixed, and allow a small change of approach orientation.
        free = [0, 1, 2, 4, 5, 6]

        def packed(x):
            q = base.copy()
            q[free] = x
            return q

        def residual(x):
            pos, ori = hand(packed(x))
            return np.r_[
                70 * (pos + ori.apply(c["center"]) - wp - requested),
                0.20 * (r.inv() * ori).as_rotvec(),
                0.003 * (x - base[free]),
            ]

        z = least_squares(
            residual,
            base[free],
            bounds=(limits[0][free], limits[1][free]),
            max_nfev=170,
        )
        q = packed(z.x)
        if (
            np.linalg.norm(residual(z.x)[:3]) / 70 > 0.0008
            or (r.inv() * hand(q)[1]).magnitude() > 0.18
        ):
            raise ValueError("pickup point outside permitted approach range")
    entry = q.copy()
    entry[0] += np.deg2rad(c["swing"])
    peak = q.copy()
    peak[3] = np.deg2rad(100)

    def tool_move(origin, displacement):
        try:
            return ik(origin, displacement, limits)
        except ValueError:
            pos, ori = hand(origin)

            def cost(x):
                p, r = hand(x)
                return np.r_[
                    100 * (p - pos - displacement),
                    0.2 * (ori.inv() * r).as_rotvec(),
                    0.05 * (x - origin),
                ]

            out = least_squares(
                cost, np.clip(origin, limits[0] + 1e-8, limits[1] - 1e-8), bounds=limits
            )
            if np.linalg.norm(hand(out.x)[0] - pos - displacement) > 0.0004:
                raise ValueError("placement out of reach")
            return out.x

    place_target = wp + delta
    if c.get("register"):
        # Feed-forward grasp transform identified in a separate nominal trial.
        # Inverse kinematics constrains cylinder axis; free yaw about the cylinder is redundant.
        reg = c["register"]
        radial = place_target[[0, 2]]
        radial = radial / np.linalg.norm(radial)
        place_target = place_target.copy()
        place_target[[0, 2]] -= radial * c.get("place_in_mm", 50) * 0.001
        if "drop_delta_mm" in c:
            place_target = wp + np.array(c["drop_delta_mm"]) * 0.001
        trim_x, trim_z = c.get("placement_trim_xz_mm", [0., 0.])
        goal = place_target + .001 * np.array([trim_x, c.get("preload_mm", 0.25), trim_z])
        axis_local = np.array(reg["axis"])
        axis_goal = np.array([0, -1 if (hand(q)[1].apply(axis_local))[1] < 0 else 1, 0])

        def placement(x):
            pos, rot = hand(x)
            err = (pos + rot.apply(reg["point"]) - goal) * 15
            align = (
                (rot.apply(axis_local) - axis_goal)
                if cyl
                else (
                    (
                        hand(q)[1]
                        * R.from_euler(
                            "xyz", c.get("tilt_deg", [0, 0, 0]), degrees=True
                        )
                    ).inv()
                    * rot
                ).as_rotvec()
            )
            return np.r_[err, align, 0.004 * (x - q)]

        seed = q.copy()
        seed[3] = max(0.24, seed[3])
        seed = np.deg2rad(c["place_seed_deg"]) if "place_seed_deg" in c else seed
        fit = least_squares(placement, seed, bounds=limits, max_nfev=250)
        if np.linalg.norm(placement(fit.x)[:3]) / 15 > 0.001:
            raise ValueError("set-down target unreachable")
        lower = fit.x
    else:
        lower = tool_move(q, np.array([0, c.get("seat_mm", 0) * 0.001, 0]))
    preplace = lower.copy()
    if c.get("preplace"):
        preplace = tool_move(lower, np.array([0, -c["preplace"] * 0.001, 0]))
    withdraw = lower.copy()
    if c.get("register"):
        # Retreat over the support plane after opening, without sweeping through the object.
        withdraw = tool_move(lower, np.array(c.get("retreat_mm", [0, -35, 0])) * 0.001)
    if np.any(np.array([entry, q, peak, lower]) < limits[0] - 1e-8) or np.any(
        np.array([entry, q, peak, lower]) > limits[1] + 1e-8
    ):
        raise ValueError("trajectory violates configured ranges")
    lt = c.get("lift_time", 1.7)
    high = lt + 1.5
    down = high + 1.15
    seat = down + 1.85
    release = seat + 0.32
    opening = c.get("opening_s", 0.85)
    end = release + opening + 1.65
    t = np.arange(int(np.ceil(end / period)) + 1) * period
    ref = np.tile(entry, (len(t), 1))
    start_open = c.get("entry_open", False)
    for i, j, a, b in [
        (entry, q, 0.19, 0.79),
        (q, peak, lt, high),
        (peak, preplace, down, seat - 0.55),
        (preplace, lower, seat - 0.55, seat),
        (
            lower,
            withdraw if c.get("register") else entry,
            release + opening + 0.12,
            release + opening + 1.08,
        ),
    ]:
        ref += ease((t - a) / (b - a))[:, None] * (j - i)
    vel = np.gradient(ref, period, axis=0)
    if np.any(ref < limits[0] - 0.000001) or np.any(ref > limits[1] + 0.000001):
        raise ValueError("waypoint outside user limits")
    vmax = c.get("speed_limit_deg_s", [125, 125, 125, 140, 160, 110, 110])
    amax = c.get("accel_limit_deg_s2", [650] * 7)
    if np.any(np.abs(np.degrees(vel)) > vmax):
        raise ValueError("planned joint speed above configured limit")
    if np.any(np.abs(np.degrees(np.gradient(vel, period, axis=0))) > amax):
        raise ValueError("planned joint acceleration above configured limit")
    st = np.array(c["st"])
    pre = np.maximum(0, st - c["open_mm"])
    maximum = st + np.array(c["extra_mm"])
    maximum[4] = max(pre[4], maximum[4] - c.get("trim_mm", 0))
    close = ease((t[:, None] - np.array(c["delay"])) / c.get("close_s", 0.72))
    unwind = ease(
        (t[:, None] - release - np.array(c.get("release_delay", [0] * 5))) / opening
    )
    line = (pre + close * (maximum - pre)) * (1 - unwind) * 0.001
    op = c.get("start_op", c["op"]) + (
        c.get("end_op", c["op"]) - c.get("start_op", c["op"])
    ) * ease((t - c.get("op_delay", 0.62)) / 0.55)
    op = op + (c.get("start_op", c["op"]) - op) * ease((t - release) / 0.55)
    return dict(
        place_target=place_target,
        q=q,
        wp=wp,
        object_pos=wp + delta + np.array(c.get("unknown_mm", [0, 0, 0])) * 0.001,
        c=c,
        times=(lt, high, down, seat, release, end),
        t=t,
        tape=np.column_stack((ref, vel, line, np.deg2rad(op - 32.75))),
        entry=entry,
        limits=limits,
    )
