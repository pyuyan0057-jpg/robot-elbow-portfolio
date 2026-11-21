"""Fit the known sphere to the thumb, middle and little finger skins using only joint poses."""

from exit_path import np, mj


class SphereFit:
    def __init__(self, m, r):
        self.d = mj.MjData(m)
        self.joints = [*r.armq, *r.fq, r.oppq]
        self.groups = [
            [i for i in range(m.ngeom) if r.labels[i] == label] for label in (1, 2, 4)
        ]
        self.free = m.joint("ball_free").qposadr[0]

    def estimate(self, m, live, r):
        d = self.d
        d.qpos[self.joints] = live.qpos[self.joints]
        mj.mj_kinematics(m, d)
        R = d.xmat[r.palm].reshape(3, 3)
        center = d.xpos[r.palm] + R @ np.array([-0.013, -0.018, -0.064])
        for _ in range(14):
            d.qpos[self.free : self.free + 3] = center
            mj.mj_kinematics(m, d)
            J = []
            values = []
            for group in self.groups:
                nearest = 0.10
                surface = None
                for i in group:
                    pair = np.empty(6)
                    dst = mj.mj_geomDistance(m, d, r.ball, i, 0.12, pair)
                    if dst < nearest:
                        nearest = dst
                        surface = pair[3:].copy()
                if surface is None:
                    return np.full(3, np.nan), np.inf
                axis = center - surface
                if np.linalg.norm(axis) < 1e-10:
                    return np.full(3, np.nan), np.inf
                J.append(axis / np.linalg.norm(axis))
                values.append(nearest + 0.00003)
            step = np.linalg.lstsq(
                np.vstack((J, 0.008 * np.eye(3))), np.r_[values, [0] * 3], rcond=None
            )[0]
            center -= step * min(1.0, 0.002 / max(1e-12, np.linalg.norm(step)))
            if max(abs(np.array(values))) < 0.000018:
                break
        return (center - d.xpos[r.palm]) @ R, max(abs(np.array(values)))


class Aim:
    """Position the encoder-estimated held sphere above the known table mark."""

    def __init__(self, pl):
        self.pl = pl
        self.fit = None
        self.goal = np.zeros(7)
        self.delta = np.zeros(7)
        self.velocity = np.zeros(7)
        self.stop = False
        self.summary = {}

    def observe(self, m, d, r, k, t, table_load):
        if t < 7.42 or self.stop:
            return
        if table_load > 0.03:
            self.stop = True
            self.goal = self.delta.copy()
            return
        if k % 20 != 10:
            return
        from motion import hand
        from scipy.optimize import lsq_linear

        if self.fit is None:
            self.fit = SphereFit(m, r)
        local, err = self.fit.estimate(m, d, r)
        if err > 0.0005 or np.linalg.norm(local - [-0.013, -0.018, -0.064]) > 0.016:
            return
        q = self.pl["tape"][k, :7]
        pos, rot = hand(q)
        baseline = pos + rot.apply(local)
        requested = self.pl["place_target"] - baseline
        requested[1] = 0.0
        requested = np.clip(requested, -0.012, 0.012)
        J = []
        for axis in np.eye(7):
            pert, R = hand(q + 0.0001 * axis)
            J.append(
                np.r_[
                    pert + R.apply(local) - baseline,
                    0.035 * (rot.inv() * R).as_rotvec(),
                ]
                / 0.0001
            )
        lo, hi = self.pl["limits"]
        self.goal = lsq_linear(
            np.vstack([np.array(J).T, 0.006 * np.eye(7)]),
            np.r_[requested, np.zeros(10)],
            bounds=(
                np.maximum(lo - q, -0.08) - 1e-10,
                np.minimum(hi - q, 0.08) + 1e-10,
            ),
        ).x
        self.summary = dict(
            last_fit_s=t,
            local_mm=(1000 * local).tolist(),
            fit_residual_mm=float(err * 1000),
        )

    def apply(self, t, ref):
        if t < 7.42:
            return
        lo, hi = self.pl["limits"]
        desired = np.clip((self.goal - self.delta) / 0.075, -0.12, 0.12)
        self.velocity += np.clip(desired - self.velocity, -0.0020, 0.0020)
        nxt = np.clip(self.delta + 0.002 * self.velocity, lo - ref[:7], hi - ref[:7])
        self.velocity = (nxt - self.delta) / 0.002
        self.delta = nxt
        ref[:7] += self.delta
        ref[7:14] += self.velocity
