"""Score recovery up to first table contact; inspect directional contact forces in paired reruns."""
from concurrent.futures import ProcessPoolExecutor
import json
from pathlib import Path

from geometry import OUT, c7, mj, np, plan
from placement import make_plan
from support import SetDown
from trial import execute

HERE = Path(__file__).resolve().parent
RESULTS = OUT / "touchdown-review"
SIDES = ("v_side_l", "v_side_h", "v_return_side")
DIGITS = ("index", "thumb", "middle", "ring", "little", "palm")


def grade(name):
    tag = OUT / (name + "_final")
    result = json.loads(tag.with_suffix(".json").read_text())
    with np.load(tag.with_suffix(".npz")) as archive:
        log = archive["log"]
        lift, high, down = archive["times"][:3]
        airborne = np.flatnonzero((log[:, 9] >= lift) & (archive["height"] > 10))
    if not airborne.size:
        raise ValueError("No lift recorded in " + name)
    first = airborne[0]
    t = log[:, 9]
    touchdowns = np.flatnonzero((np.arange(len(log)) > first) & (log[:, 6] > .03))
    touch = int(touchdowns[0]) if len(touchdowns) else None
    # A 40 ms absence of digit contact flags an escape. In these seven cases
    # the escaped sphere also separates by well over one radius from the palm.
    end = touch if touch is not None else len(log)
    force = log[first:end, :5].sum(axis=1)
    lost = np.flatnonzero(np.convolve((force < .015).astype(int), np.ones(20), "valid") == 20)
    loss = float(t[first + lost[0]]) if len(lost) else None
    error = None
    if touch is not None:
        target = plan("ball", result["case"], .002)["place_target"]
        error = float(1000 * np.linalg.norm(log[touch, [15, 17]] - target[[0, 2]]))
    qualified = bool(touch is not None and loss is None and t[touch] > down and error <= 2.)
    return dict(case=name, loss_s=loss, first_table_s=float(t[touch]) if touch is not None else None,
                first_contact_error_mm=error, accepted=qualified)


class ContactRecorder(c7.Recorder):
    def __init__(self, start, finish):
        self.start, self.finish = start, finish
        self.rows = []
        self.library = c7.c6.LIB

    def sample(self, m, d, r, k, t, ph):
        super().sample(m, d, r, k, t, ph)
        at_rest = abs(t - .12) < .0001
        if not at_rest and not self.start - .022 <= t <= self.finish + .022:
            return
        world = np.zeros((8, 3))
        for ci, contact in enumerate(d.contact):
            a, b = contact.geom
            if r.ball not in (a, b) or contact.efc_address < 0:
                continue
            other = a if b == r.ball else b
            wrench = np.zeros(6)
            mj.mj_contactForce(m, d, ci, wrench)
            sign = 1 if b == r.ball else -1
            world[r.labels[other]] += sign * (contact.frame.reshape(3, 3).T @ wrench[:3])
        if at_rest:
            np.testing.assert_allclose(world[6], [0., -.033 * 9.80665, 0.], atol=.01)
        else:
            self.rows.append((t, world))


def examine(name):
    cfg, force, offset, duration, seed = c7.recipe(name)
    cfg.update(json.loads((HERE / "frozen.json").read_text()))
    pl = make_plan("ball", cfg, .002)
    start = pl["times"][1] + offset
    rec = ContactRecorder(start, start + duration)
    gate = SetDown(rec, c7.Feedback(rec, "window6", seed), cfg, pl)

    def pulse(d, r, t, ph):
        u = t - ph[1] - offset
        d.xfrc_applied[r.body, :3] = force * (np.sin(np.pi*u/duration)**2 if 0<=u<=duration else 0.)

    tag = RESULTS / name
    execute("ball", cfg, tag=str(tag), observer=rec, disturbance=pulse, reflex=gate, planner=lambda *args: pl)
    with np.load(tag.with_suffix(".npz")) as fresh, np.load(OUT / (name + "_final.npz")) as old:
        np.testing.assert_allclose(fresh["log"], old["log"], atol=1e-9, rtol=0)
    times = np.array([t for t, _ in rec.rows])
    vectors = np.array([w for _, w in rec.rows])
    peak = np.argmin(abs(times - start - duration/2))
    # All three pulses point mostly in world -X, so +X opposes the lateral push.
    return dict(case=name, sample_s=times[peak], applied_x_N=force[0]*np.sin(np.pi*(times[peak]-start)/duration)**2,
                opposing_x_by_digit_N=dict(zip(DIGITS, vectors[peak, :6, 0].tolist())),
                sum_hand_x_N=float(vectors[peak, :6, 0].sum()))


if __name__ == "__main__":
    RESULTS.mkdir(parents=True, exist_ok=True)
    grades = [grade(n) for n in c7.FINAL_TEST]
    with ProcessPoolExecutor(3) as pool:
        directions = list(pool.map(examine, SIDES))
    (HERE / "evidence" / "touchdown_review.json").write_text(json.dumps(dict(
        scope="Touchdown-only review; 2 mm horizontal tolerance; strict C8 retained separately.",
        graded=grades, side_push_forces=directions), indent=2))
    print(json.dumps([g for g in grades if not g["accepted"] or g["case"]=="v_high"], indent=2))
    print(directions)
