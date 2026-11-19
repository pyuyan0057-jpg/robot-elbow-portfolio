"""Middle-finger pad reach prior to tendon, collision and grasp-force checks. Units: mm."""
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
MODEL = json.loads((ROOT / "data/free-ball-model.json").read_text())
BALL = 1000 * np.array(MODEL["ball_in_palm"]) + [.69, -.59, 2.28]
MCP = np.array([-11., 3., 0.])
TOUCH = 1000 * (MODEL["radius_m"] + MODEL["pad_radius_m"])


def screen():
    # Archived middle phalanges 43/28/23 plus 4 mm to the pad sphere centre.
    # Independent angles form a relaxed workspace, accessed here on a 1-degree grid.
    q = np.ogrid[:66, :86, :71]
    a, b, c = [np.deg2rad(i) for i in q]
    y = 43*np.cos(a) + 28*np.cos(a+b) + 27*np.cos(a+b+c)
    z = -43*np.sin(a) - 28*np.sin(a+b) - 27*np.sin(a+b+c)
    for shift, maximum in [(0, 0), (0, 15), (0, 20), (0, 25), (5, 15), (12, 0)]:
        dx, dy, dz = BALL + [shift, 0, 0] - MCP
        best = (math.inf, None)
        for deg in range(-maximum, maximum + 1):
            t = math.radians(deg)
            squared = (dx+y*math.sin(t))**2 + (dy-y*math.cos(t))**2 + (dz-z)**2
            gap = math.sqrt(squared.min()) - TOUCH
            if gap < best[0]:
                best = (gap, deg)
        print(f"{shift:+3d} mm   ±{maximum:2d} deg   min sampled gap {best[0]:7.2f} mm   yaw {best[1]:+3d} deg")


if __name__ == "__main__":
    dx, dy, _ = np.abs(BALL - MCP)
    print(f"Straight plane clearance lower bound: {dx-TOUCH:.3f} mm")
    angle = math.acos(TOUCH/math.hypot(dx,dy))-math.atan2(dy,dx)
    print(f"Minimum absolute basal yaw to bring the infinite plane within contact distance: {math.degrees(angle):.3f} deg")
    screen()
