import csv
import importlib.util
import json
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class FreeBallTest(unittest.TestCase):
    @unittest.skipUnless(
        importlib.util.find_spec("mujoco"),
        "Install requirements-free-ball.txt to run dynamics",
    )
    def test_integral_cup_and_hanging_gate(self):
        from thumb_cup import trial

        for grams in (33, 150):
            result, rows = trial(True, grams, cut=True)
            self.assertFalse(result["hanging_pass"])
            self.assertIsNone(result["lift_pass"])
            self.assertLess(result["first_loss_s"], result["plane_s"])
            self.assertAlmostEqual(rows[-1]["plane_N"], grams * 0.00980665, places=4)
            self.assertEqual(rows[-1]["t_s"], 1.0)

    def test_pickup_comparison_archive(self):
        with (ROOT / "examples/pickup-pose-cases.csv").open() as stream:
            cases = {row["case"]: row for row in csv.DictReader(stream)}
        self.assertEqual(len(cases), 390)
        self.assertTrue(
            all(row["plane_s"] and row["passed"] == "False" for row in cases.values())
        )
        self.assertEqual(
            sum(row["outcome"] == "missed pickup" for row in cases.values()), 37
        )
        for prefix in ("p0", "p1"):
            row = cases[f"verify-{prefix}-nominal"]
            self.assertLess(
                float(row["first_interrupted_s"]), float(row["final_pad_exit_s"])
            )
            self.assertLess(float(row["final_pad_exit_s"]), float(row["plane_s"]))

    @unittest.skipUnless(
        importlib.util.find_spec("mujoco"),
        "Install requirements-free-ball.txt to run dynamics",
    )
    def test_adjusted_supported_pickup(self):
        from pickup_pose import evaluate

        result, rows = evaluate((3.63, -3.48, 4.16), 31.31)
        self.assertFalse(result["passed"])
        self.assertEqual(result["outcome"], "dropped")
        self.assertAlmostEqual(result["final_pad_exit_s"], 0.164, delta=0.01)
        self.assertGreater(min(rows[0]["index_N"], rows[0]["thumb_N"]), 0.8)
        self.assertAlmostEqual(rows[-1]["plane_N"], 0.150 * 9.80665, places=3)

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
