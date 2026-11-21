"""Measured-contact release for the virtual test bench. Force unit: newtons."""

import numpy as np
from motion import ease


class SetDown:
    def __init__(self, tactile, feedback, recipe, plan):
        self.rec = tactile
        self.feedback = feedback
        self.recipe = recipe
        self.trajectory = plan
        self.release_time = None
        self.onset = None
        self.dwell = 0.0
        self.anchor = None
        self.hold_stroke = None
        self.fingers = None
        self.latched = False
        self.press = 0.0

    def __call__(self, k, t, ph, ref, faulted):
        self.feedback(k, t, ph, ref, faulted)
        if faulted:
            return
        if t < ph[2] + 2.6:
            self.hold_stroke = self.feedback.stroke.copy()
            return
        ref[14:16] += 0.001 * (self.hold_stroke - self.feedback.stroke)
        table_force = self.rec.data[k, 21]
        if self.anchor is None:
            if t < ph[4] - 0.004:
                self.fingers = ref[14:20].copy()
            if table_force > 0.08 and self.rec.data[k, 19] > 0.04:
                self.dwell += 0.002
            else:
                self.dwell = 0.0
            if self.dwell >= 0.010:
                self.anchor = ref[:7].copy()
                self.fingers = ref[14:20].copy()
                self.onset = t
            elif t >= ph[4] - 0.002:
                self.latched = True
        if self.latched:
            ref[:7] = self.trajectory["tape"][round((ph[4] - 0.004) / 0.002), :7]
            ref[7:14] = 0
            ref[14:20] = self.fingers
            return
        if self.anchor is None:
            return
        ref[:7] = self.anchor
        ref[7:14] = 0.0
        ref[14:20] = self.fingers
        if self.release_time is None:
            if t > self.onset + 0.03 and table_force < 0.23 and self.press < 2.0:
                self.press += 0.003
                self.anchor += 0.003 * self.trajectory["per_mm"]
            self.dwell = self.dwell + 0.002 if table_force >= 0.16 else 0.0
            if t > self.onset + 0.060 and self.dwell >= 0.024:
                self.release_time = t
            return
        elapsed = t - self.release_time
        opening = self.recipe.get("unwind_s", 1.3)
        delays = np.array(self.recipe.get("peel_delay", [0] * 5))
        ref[14:19] = self.fingers[:5] * (1 - ease((elapsed - delays) / opening))
        # Begin lifting the open hand along the approach route.
        start = self.recipe.get("retreat_start", 1.0)
        travel = self.recipe.get("retreat_s", 1.40)
        u = np.clip((elapsed - start) / travel, 0.0, 1.0)
        dest = self.trajectory["tape"][-1, :7]
        ref[:7] = self.anchor + (dest - self.anchor) * ease(u)
        ref[7:14] = (dest - self.anchor) * 30 * u**2 * (1 - u) ** 2 / travel

        slide = self.trajectory["lateral_mm"] @ self.recipe.get("slide_mm", [0.0, 0.0])
        span = self.recipe.get("slide_time", 0.42)
        w = np.clip(elapsed / span, 0.0, 1.0)
        ref[:7] += slide * ease(w) * (1 - ease(u))
        ref[7:14] += slide * (
            30 * w**2 * (1 - w) ** 2 / span * (1 - ease(u))
            - ease(w) * 30 * u**2 * (1 - u) ** 2 / travel
        )
