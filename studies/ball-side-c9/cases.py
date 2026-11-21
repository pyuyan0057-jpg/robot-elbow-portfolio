"""Fixed challenge suite defined before selecting C9 finger commands. Forces in world XYZ (+Y down)."""

from exit_path import c7, np


def challenges():
    rng = np.random.default_rng(902163)
    axes = np.r_[
        np.eye(3),
        -np.eye(3),
        np.array([[1, 1, 1], [-1, 1, 1], [1, -1, 1], [-1, -1, -1]]),
    ]
    rows = {}
    for i, direction in enumerate(axes):
        unit = direction / np.linalg.norm(direction)
        for j, magnitude in enumerate((2.5, 5.0)):
            phase = (i + 2 * j) % 3
            offset = (-0.65, 0.34, 1.47)[phase] + rng.uniform(-0.09, 0.09)
            shift = rng.uniform(-0.9, 0.9, 2)
            rows[f"new_{i + 1:02d}_{j + 1}"] = (
                [shift[0], 0.0, shift[1]],
                (unit * magnitude).tolist(),
                offset,
                rng.uniform(0.15, 0.20),
                62000 + 2 * i + j,
            )
    return rows


CHALLENGES = challenges()


def independent_panel():
    """Eight previously unused oblique directions at each of three impulse magnitudes."""
    rng = np.random.default_rng(230650)
    result = {}
    for i in range(8):
        direction = rng.uniform(0.2, 1.0, 3) * [
            1 if i & (1 << j) else -1 for j in range(3)
        ]
        direction /= np.linalg.norm(direction)
        for j, mag in enumerate((0.6, 2.5, 5.0)):
            delay = (-0.41, 0.58, 1.36)[(i + j) % 3] + rng.uniform(-0.06, 0.06)
            shift = rng.uniform(-0.85, 0.85, 2)
            result[f"oblique_{i + 1}_{j + 1}"] = (
                [shift[0], 0.0, shift[1]],
                (direction * mag).tolist(),
                delay,
                rng.uniform(0.16, 0.195),
                77500 + 3 * i + j,
            )
    return result


INDEPENDENT = independent_panel()


def directional_panel():
    """A bounded operating test covering axis, edge and corner directions at three phases."""
    from itertools import product

    rows = {}
    rng = np.random.default_rng(334781)
    axes = [np.array(v, dtype=float) for v in product((-1, 0, 1), repeat=3) if any(v)]
    for i, vec in enumerate(axes):
        for j, offset in enumerate((-0.41, 0.58, 1.36)):
            shift = rng.uniform(-0.85, 0.85, 2)
            force = 0.6 * vec / np.linalg.norm(vec)
            rows[f"direction_{i + 1:02d}_{j + 1}"] = (
                [shift[0], 0.0, shift[1]],
                force.tolist(),
                offset,
                0.18,
                85100 + 3 * i + j,
            )
    return rows


DIRECTIONS = directional_panel()


def parameters(case):
    suite = CHALLENGES | INDEPENDENT | DIRECTIONS
    if case not in suite:
        return c7.recipe(case)
    cfg, *_ = c7.recipe("v_quiet")
    shift, force, offset, duration, seed = suite[case]
    cfg["unknown_mm"] = shift
    return cfg, np.array(force), offset, duration, seed
