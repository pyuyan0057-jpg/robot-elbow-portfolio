"""Isolate sliding friction between the free ball and hand; preserve explicit table contact."""

import numpy as np


def set_ball_friction(root, mu):
    ball = root.find(".//geom[@name='ball_surface']")
    table_pair = root.find(".//pair[@geom1='ball_surface'][@geom2='table']")
    if table_pair is None:
        table_pair = root.find(".//pair[@geom1='table'][@geom2='ball_surface']")
    if table_pair is None:
        raise ValueError("An explicit ball-table pair is required")
    if not np.isfinite(mu) or mu <= 0:
        raise ValueError("Expected positive sliding coefficient")
    # All 50 skin geoms have the same contact constants in the compiled source.
    # Giving only the ball priority over the skin localizes the parameter change.
    for name, value in dict(
        priority="2",
        condim="3",
        solref=".008 1",
        solimp=".97 .995 .0005 .5 2",
        solmix="1",
        margin="0",
        gap="0",
        friction=f"{mu} 0 0",
    ).items():
        ball.set(name, value)


def verify_contacts(m, d, r, seen, mu):
    for c in d.contact:
        if r.ball not in (c.geom1, c.geom2) or c.efc_address < 0:
            continue
        other = c.geom1 if c.geom2 == r.ball else c.geom2
        kind = "table" if other == r.table else "hand"
        if kind == "hand" and r.labels[other] > 5:
            raise AssertionError("Unexpected ball contact")
        if kind in seen:
            continue
        expected = 0.45 if kind == "table" else mu
        np.testing.assert_allclose(c.friction[:2], expected)
        seen[kind] = list(map(float, c.friction))
