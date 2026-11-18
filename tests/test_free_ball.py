import csv
import importlib.util
import json
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class FreeBallTest(unittest.TestCase):
    def test_archived_separation_and_flight(self):
        with (ROOT / "examples/free-ball-cases.csv").open() as stream:
            cases = {r["case"]: r for r in csv.DictReader(stream)}
        self.assertEqual(len(cases), 18)
        for row in cases.values():
            self.assertEqual(row["outcome"], "dropped", row["case"])
            self.assertGreater(float(row["plane_s"]), float(row["final_pad_exit_s"]))
            self.assertAlmostEqual(float(row["flight_accel_m_s2"]), 9.80665, places=3)
        self.assertAlmostEqual(
            float(cases["cad-half-step"]["final_pad_exit_s"]),
            float(cases["cad-quarter-step"]["final_pad_exit_s"]),
            places=3,
        )

    @unittest.skipUnless(
        importlib.util.find_spec("mujoco"),
        "Install requirements-free-ball.txt to run dynamics",
    )
    def test_public_free_release(self):
        from free_ball import run

        cfg = json.loads((ROOT / "data/free-ball-model.json").read_text())
        result, trajectory = run(cfg)
        self.assertEqual(result["status"], "dropped")
        self.assertGreater(min(result["release"]["pad_N"]), 1.0)
        self.assertAlmostEqual(result["final_pad_exit_s"], 0.104, delta=0.01)
        self.assertAlmostEqual(result["floor_s"], 0.386, delta=0.01)
        self.assertAlmostEqual(trajectory[-1]["plane_N"], 0.15 * 9.80665, places=3)
        self.assertGreater(
            max(s["spin_rel_rad_s"] for s in trajectory if s["t_s"] < 0.1), 1.0
        )
