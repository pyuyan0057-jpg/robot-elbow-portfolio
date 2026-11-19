"""Force targets from gravity equilibrium at measured contacts of the supported sphere."""

import numpy as np
from scipy.optimize import linprog
from tripod_trial import NAMES


def targets(points, minimum, mass=0.033, mu=0.28):
    cols = []
    loads = []
    for p in points:
        n = np.asarray(p["normal"])
        r = np.asarray(p["arm_m"])
        axis = np.eye(3)[np.argmin(abs(n))]
        u = np.cross(n, axis)
        u /= np.linalg.norm(u)
        v = np.cross(n, u)
        for theta in np.arange(32) * 2 * np.pi / 32:
            f = n + mu * (np.cos(theta) * u + np.sin(theta) * v)
            cols.append(np.r_[f, np.cross(r, f) / 0.01])
            loads.append([float(p["digit"] == k) for k in NAMES])
    M = np.asarray(cols).T
    L = np.asarray(loads).T
    fit = linprog(
        np.ones(len(cols)),
        A_ub=-L,
        b_ub=[-minimum] * 3,
        A_eq=M,
        b_eq=[0, -mass * 9.80665, 0, 0, 0, 0],
        bounds=(0, None),
        method="highs",
    )
    if not fit.success:
        raise ValueError(fit.message)
    return L @ fit.x
