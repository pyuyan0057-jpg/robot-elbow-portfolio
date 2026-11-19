"""Fixed mount of the ring finger. Distances in metres in the palm frame."""
import numpy as np


def reorient_ring(ring, roll_deg=-17.0, shift_mm=(2.5, -3.0, 0.0)):
    """Apply to a freshly loaded ring schedule and return its rigid transform."""
    angle = np.deg2rad(roll_deg)
    c, s = np.cos(angle), np.sin(angle)
    rotation = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    guide = np.array([0.0074, 0, -0.0063])
    translation = guide - rotation @ guide + 0.001 * np.asarray(shift_mm)
    for link in ring["links"]:
        link["ledger_com_m"] = list(link["com"])
        for key in ("root", "com"):
            link[key] = (rotation @ link[key] + translation).tolist()
        link["axis"] = (rotation @ link["axis"]).tolist()
    ring["pad_center"] = (rotation @ ring["pad_center"] + translation).tolist()
    return rotation, translation
