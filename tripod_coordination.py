"""Locked thumb spread and causal three-tendon feedback for the R4 factorial trial."""

import numpy as np


class ThreePadFeedback:
    """A 500 Hz simulated load loop. Corrections are metres of extra tendon take-up."""

    def __init__(self, targets=(1.2, 1.2, 1.2), gain=5.0):
        self.targets = np.array(targets, dtype=float)
        self.gain = gain
        self.offset = np.zeros(3)
        self.filtered = None

    def update(self, t, normals, tension):
        measured = np.asarray(normals, dtype=float)
        if self.filtered is None:
            self.filtered = measured.copy()
        self.filtered += 0.28 * (measured - self.filtered)
        if t < -0.30:
            return self.offset
        error = self.targets - self.filtered
        # Coordinate relative load sharing as well as the common preload.
        rate = self.gain * (error + 0.5 * (error - error.mean()))
        rate = np.clip(rate, -8.0, 8.0)  # mm/s, design assumption
        rate = np.minimum(rate, np.clip((24.0 - np.asarray(tension)) * 4, -8, 8))
        self.offset = np.clip(self.offset + 0.002 * 0.001 * rate, -0.003, 0.003)
        return self.offset


def lock_thumb(cfg, angle_deg, hulls=(), thumb_parts=()):
    """Second axis through thumb MCP, parallel to palm Z; setup rotation only."""
    if angle_deg == 0:
        return
    th = cfg["digits"][1]
    origin = np.array(th["links"][0]["root"])
    c, s = np.cos(np.deg2rad(angle_deg)), np.sin(np.deg2rad(angle_deg))
    rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    def move(points):
        return (np.asarray(points) - origin) @ rotation.T + origin

    for link in th["links"]:
        link["ledger_com_m"] = list(link["com"])
        link["root"] = move(link["root"]).tolist()
        link["com"] = move(link["com"]).tolist()
        link["axis"] = (rotation @ link["axis"]).tolist()
    th["pad_center"] = move(th["pad_center"]).tolist()
    cfg["cup_cells"] = [move(cell) for cell in cfg["cup_cells"]]
    cfg["pad_delta_com_m"] = move(cfg["pad_delta_com_m"]).tolist()
    for part in hulls:
        if part["parent"].startswith("E3_F4_") or part["name"] in thumb_parts:
            part["vertices"] = move(part["vertices"]).tolist()
