"""Task and feedback logic shared by the ROS node and responsibility-isolated batch tests."""

import numpy as np
from tension_control import RingController


class Supervisor:
    def __init__(self, config, tape, times, dt):
        self.tape = tape
        self.dt = dt
        self.times = times
        self.c = config
        self.law = RingController(config["target_N"], config["early_before"])
        self.last = tape[0].copy()
        self.fault = None
        self.event = None
        self.bad = 0.0
        self.lastq = None
        self.valid = 0.0

    def reference(self, t):
        return self.tape[min(len(self.tape) - 1, max(0, round(t / self.dt)))].copy()

    def step(self, t, feedback, q):
        lt, high, down, seat, release, end = self.times
        ref = self.reference(t)
        q = np.asarray(q)
        velocity = (q - self.lastq) / self.dt if self.lastq is not None else np.zeros(7)
        self.lastq = q.copy()
        if self.fault is None:
            sane = np.isfinite(feedback) and -0.4 <= feedback <= 25.5
            self.bad = 0 if sane else self.bad + self.dt
            if sane and t < release:
                ref[17] = self.law.step(ref[17], feedback, t, self.dt)
                self.valid = self.valid + self.dt if feedback > 1 else 0.0
            elif t < release:
                ref[17] = self.last[17]
            elif t >= release:
                duration = self.c.get("opening_s", 0.85)
                delay = self.c.get("release_delay", [0] * 5)[3]
                u = np.clip((t - release - delay) / duration, 0, 1)
                ref[17] = self.ring_at_release * (1 - u**3 * (10 - 15 * u + 6 * u * u))
            if t < release:
                self.ring_at_release = ref[17] if sane else self.last[17]
            no_grip = abs(t - lt) < self.dt * 0.51 and (
                self.law.phase != "regulate" or self.valid < 0.022
            )
            # At the end of the low-speed initial lift require tension and encoder tracking.
            probe_finish = lt + 0.32
            probe_failed = abs(t - probe_finish) < self.dt * 0.51 and (
                self.valid < 0.022 or np.max(np.abs(ref[:7] - q)) > np.deg2rad(1.5)
            )
            if (
                self.law.phase == "blocked"
                or no_grip
                or probe_failed
                or self.bad >= 0.024
            ):
                self.event = t
                self.fault = "abort" if t <= lt else "set_down"
                self.at_q = q.copy()
                self.v = velocity.copy()
                self.spool = self.last[14:].copy()
                self.stop_q = q + self.v * 0.07
        if self.fault == "abort":
            ref[:7] = self.at_q
            ref[7:14] = 0
            ref[14:19] = 0
        if self.fault == "set_down":
            elapsed = t - self.event
            if elapsed < 0.14:
                u = elapsed / 0.14
                ref[:7] = self.at_q + 0.14 * self.v * (u - u * u / 2)
                ref[7:14] = self.v * (1 - u)
            elif elapsed < 1.94:
                target = self.reference(seat - 0.55)[:7]
                u = (elapsed - 0.14) / 1.8
                ref[:7] = self.stop_q + (target - self.stop_q) * u**3 * (
                    10 - 15 * u + 6 * u * u
                )
                ref[7:14] = (target - self.stop_q) * 30 * u * u * (1 - u) ** 2 / 1.8
            else:
                ref = self.reference(seat - 0.55 + elapsed - 1.94)
            vtime = seat - 0.55 + elapsed - 1.94
            u = np.clip((vtime - release) / self.c.get("opening_s", 0.85), 0, 1)
            ref[14:] = self.spool
            ref[14:19] *= 1 - u**3 * (10 - 15 * u + 6 * u * u)
        self.last = ref.copy()
        return ref, self.fault == "abort"
