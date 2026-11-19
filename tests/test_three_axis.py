import csv
import json
import math
import unittest
from pathlib import Path

from three_axis_pose import elbow_and_tip
from three_axis_loads import calculate

DATA = Path(__file__).resolve().parents[1] / "data" / "three-axis"


class ThreeAxisArchive(unittest.TestCase):
    def test_fk_matches_saved_sw_configurations(self):
        for centered, directory in ((False, DATA), (True, DATA / "center")):
            with (directory / "poses.csv").open(encoding="utf-8-sig") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 6)
            for row in rows:
                origin, tip = elbow_and_tip(float(row["yaw_deg"]),
                    float(row["pitch_from_horizontal_deg"])+90, -float(row["roll_deg"]), centered)
                measured = [float(row["tip_"+k+"_mm"]) for k in "xyz"]
                self.assertLess(math.dist(tip, measured), .001)
                if centered:
                    measured_origin = [float(row["pitch_origin_"+k+"_mm"]) for k in "xyz"]
                    self.assertLess(math.dist(origin, measured_origin), .001)

    def test_added_load_at_horizontal_fingertip(self):
        for centered, directory in ((False,DATA),(True,DATA/"center")):
            bodies = json.loads((directory/"rigid-groups.json").read_text())
            empty = calculate(bodies,0,centered)
            loaded = calculate(bodies,.15,centered)
            delta = loaded["pitch"]["horizontal_gravity_Nm"]-empty["pitch"]["horizontal_gravity_Nm"]
            self.assertAlmostEqual(delta,.15*9.80665*.511214365,places=8)
            self.assertEqual(loaded["yaw"]["horizontal_gravity_Nm"],0)
