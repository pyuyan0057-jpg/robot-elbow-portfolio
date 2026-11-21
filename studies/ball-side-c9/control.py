"""C9 grip scheduling and matched full-physics playback."""

import json
from pathlib import Path
from exit_path import HERE, C8, RESULT, c7, mj, np
from cases import parameters
from placement import make_plan
from support import SetDown
from trial import execute
from motion import ease, hand
from registration import SphereFit


def grip_offsets(times, p):
    """Stagger the little finger and thumb, then unwind auxiliary grip before touchdown."""
    t = np.asarray(times)
    other = ease((t - p.get("start", 2.10)) / 0.34)
    little = ease((t - p.get("start", 2.10) + 0.12) / 0.32)
    release = 1 - ease((t - p.get("relax", 5.85)) / p.get("unwind", 0.95))
    s = np.outer(other * release, p["mm"]) * 0.001
    s[:, 4] = little * release * p["mm"][4] * 0.001
    opp = other * release * np.radians(p["deg"])
    return np.column_stack((s, opp))


def prepare(case):
    cfg, force, offset, duration, seed = parameters(case)
    cfg.update(json.loads((C8 / "frozen.json").read_text()))
    return cfg, force, offset, duration, seed


def run(job):
    case, p, folder = job
    cfg, force, offset, duration, seed = prepare(case)
    force = force * p.get("disturbance_scale", 1.0)
    if "dt" in p:
        cfg["dt"] = p["dt"]
    rec = c7.Recorder()
    rec.library = c7.c6.LIB
    seen = {}
    if "skin_ball_mu" in p:
        from friction import set_ball_friction, verify_contacts

        configure = rec.configure

        def configured(root):
            configure(root)
            set_ball_friction(root, p["skin_ball_mu"])

        rec.configure = configured
        before = rec.sample

        def checked(m, d, r, k, t, ph):
            before(m, d, r, k, t, ph)
            if len(seen) < 2:
                verify_contacts(m, d, r, seen, p["skin_ball_mu"])

        rec.sample = checked
    pl = make_plan("ball", cfg, 0.002)
    feedback = c7.Feedback(rec, "window6", seed)
    gate = SetDown(rec, feedback, cfg, pl)
    extra = grip_offsets(pl["t"], p)
    clock = pl["t"]
    if "entry_mm" in p:
        early = ease((clock - 0.62) / 0.42) * (1 - ease((clock - 1.65) / 0.52))
        extra[:, :5] += np.outer(early, p["entry_mm"]) * 0.001
    from registration import Aim

    aim = Aim(pl) if p.get("register") else None
    if aim:
        original_sample = rec.sample

        def sample(m, d, r, k, t, ph):
            original_sample(m, d, r, k, t, ph)
            aim.observe(m, d, r, k, t, rec.data[k, 21])

        rec.sample = sample
    response = p.get("reflex")
    alarm_at = -10.0
    events = []

    def controller(k, t, ph, ref, fault):
        nonlocal alarm_at
        gate(k, t, ph, ref, fault)
        if not fault:
            if aim:
                aim.apply(t, ref)
            ref[14:20] += extra[k]
            if response and 2.42 < t < 6.25:
                thumb, speed, _, index, slip, _ = feedback.log[-1][5:]
                if (
                    t - alarm_at > 0.70
                    and thumb > response.get("threshold", 2.0)
                    and speed < 6.0
                    and index > 0.065
                    and slip > response.get("slip", 35)
                ):
                    alarm_at = t
                    events.append(t)
                elapsed = t - alarm_at
                gain = ease(elapsed / 0.065) * (1 - ease((elapsed - 0.20) / 0.28))
                ref[14:19] += 0.001 * gain * np.array(response["mm"])
                ref[19] += gain * np.radians(response.get("deg", 0.0))
            ref[14:19] = np.maximum(ref[14:19], 0.0)
            ref[19] = np.clip(ref[19], -0.571595, 0.014563)

    def disturbance(d, r, t, ph):
        u = t - ph[1] - offset
        d.xfrc_applied[r.body, :3] = force * (
            np.sin(np.pi * u / duration) ** 2 if 0 <= u <= duration else 0.0
        )

    tag = RESULT / folder / case
    tag.parent.mkdir(parents=True, exist_ok=True)
    res = execute(
        "ball",
        cfg,
        tag=str(tag),
        observer=rec,
        reflex=gate if p.get("baseline") else controller,
        disturbance=disturbance,
        planner=lambda *args: pl,
    )
    if p.get("trace"):
        np.savez_compressed(str(tag) + "_pads.npz", data=feedback.log)
    row = grade_tag(tag, pl["place_target"])
    if "skin_ball_mu" in p:
        assert set(seen) == {"hand", "table"}
        row["effective_friction"] = seen
    if aim:
        row["targeting"] = aim.summary
    if response:
        row["tactile_events_s"] = events
    return row


def grade_tag(tag, target):
    with np.load(tag.with_suffix(".npz")) as data:
        a = data["log"]
        t = a[:, 9]
        lift, high, down = data["times"][:3]
        air = np.flatnonzero((t >= lift) & (data["height"] > 10))
        if not len(air):
            return dict(
                case=tag.name,
                held=False,
                error_mm=None,
                passed=False,
                touchdown_s=None,
                loss_s=None,
            )
        first = air[0]
        hits = np.flatnonzero((np.arange(len(a)) > first) & (a[:, 6] > 0.03))
        touch = int(hits[0]) if len(hits) else None
        finger = a[first:touch, :5].sum(1)
        gap = (
            np.flatnonzero(
                np.convolve((finger < 0.015).astype(int), np.ones(20), "valid") == 20
            )
            if len(finger) >= 20
            else []
        )
        held = bool(touch is not None and t[touch] > down and not len(gap))
        err = (
            float(np.linalg.norm(a[touch, [15, 17]] - target[[0, 2]]) * 1000)
            if touch is not None
            else None
        )
        return dict(
            case=tag.name,
            held=held,
            error_mm=err,
            passed=bool(held and err <= 2.0),
            loss_s=float(t[first + gap[0]]) if len(gap) else None,
            touchdown_s=float(t[touch]) if touch is not None else None,
        )
