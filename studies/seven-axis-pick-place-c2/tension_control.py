"""Ring feedback uses only filtered proximal tension, time and spool command."""


class RingController:
    def __init__(self, target=3.5, early_before=0.80):
        self.target = target
        self.early_before = early_before
        self.phase = "approach"
        self.position = 0.0
        self.entered = None
        self.age = 0.0

    def step(self, reference, measured, t, dt):
        if self.phase == "approach":
            self.age = self.age + dt if measured > 1.8 else 0.0
            if self.age >= 0.004:
                self.entered = t
                self.phase = "blocked" if t < self.early_before else "regulate"
            else:
                self.position = reference
                return reference
        if self.phase == "blocked":
            self.position = max(0.0, self.position - 0.030 * dt)
        else:
            speed = max(-0.028, min(0.018, 0.003 * (self.target - measured)))
            self.position = max(0.0, min(reference, self.position + dt * speed))
        return self.position
