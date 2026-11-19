"""Gravity-only point-contact equilibrium at one measured sphere pose."""

import numpy as np
from scipy.optimize import linprog


def supports_weight(contacts, mass_kg, mu, outer=False, facets=32, fixed_normals=False):
    columns = []
    for point in contacts:
        n = np.array(point["normal"])
        r = np.array(point["arm_m"])
        if np.dot(n, r) >= 0:
            raise ValueError("Contact normal must point into the ball")
        axis = np.eye(3)[np.argmin(abs(n))]
        u = np.cross(n, axis)
        u /= np.linalg.norm(u)
        v = np.cross(n, u)
        slope = mu / np.cos(np.pi / facets) if outer else mu
        for theta in np.arange(facets) * 2 * np.pi / facets:
            force = n + slope * (u * np.cos(theta) + v * np.sin(theta))
            columns.append(np.r_[force, np.cross(r, force) / 0.01])
    if not columns:
        return False
    # Moment divided by radius to keep force and moment rows on comparable scales.
    A = np.array(columns).T
    rhs = [0, -mass_kg * 9.80665, 0, 0, 0, 0]
    if fixed_normals:
        normal_rows = np.repeat(np.eye(len(contacts)), facets, axis=1)
        # Minimise worst unbalanced wrench component with measured pressures fixed.
        # This phase-I problem stays feasible even for a badly unbalanced pinch.
        rhs = np.asarray(rhs)
        inequalities = np.column_stack((np.vstack((A, -A)), -np.ones(12)))
        fixed = np.column_stack((normal_rows, np.zeros(len(contacts))))
        fit = linprog(
            np.r_[np.zeros(A.shape[1]), 1.0],
            A_ub=inequalities,
            b_ub=np.r_[rhs, -rhs],
            A_eq=fixed,
            b_eq=[p["normal_N"] for p in contacts],
            bounds=(0, None),
            method="highs",
        )
        if not fit.success:
            raise RuntimeError(fit.message)
        return bool(fit.fun < 1e-7)
    fit = linprog(
        np.ones(A.shape[1]), A_eq=A, b_eq=rhs, bounds=(0, None), method="highs"
    )
    if fit.status not in (0, 2):
        raise RuntimeError(fit.message)
    return fit.success
