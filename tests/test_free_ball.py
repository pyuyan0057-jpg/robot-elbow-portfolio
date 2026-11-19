import csv
import importlib.util
import json
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class FreeBallTest(unittest.TestCase):
    def test_pair_replay_bounds_include_reversal(self):
        from swept_phalanges import replay_bounds

        rows = []
        for t, angle in [(0, 0), (0.1, 0.1), (0.2, 0)]:
            rows.append(dict(t_s=t, angle_2_0_rad=angle, angle_2_1_rad=0,
                             angle_3_0_rad=0, angle_3_1_rad=0))
        checks = [dict(release_relative_s=t, mid2_ring2_mm=15) for t in (0, 0.2)]
        self.assertAlmostEqual(replay_bounds(rows, checks)[0]["gap_lower_mm"], 5)

    @unittest.skipUnless(importlib.util.find_spec("mujoco"), "MuJoCo required")
    def test_ring_mount_geometry_and_mass_reference(self):
        import numpy as np
        from ring_timing import schedule
        from ring_mount import reorient_ring

        ring = schedule(0.5, 12, False)
        old = np.array([link["root"] for link in ring["links"]])
        com = np.array([link["com"] for link in ring["links"]])
        U, b = reorient_ring(ring)
        guide = np.array([0.0074, 0, -0.0063])
        np.testing.assert_allclose(U @ guide + b, guide + [0.0025, -0.003, 0])
        np.testing.assert_allclose([link["root"] for link in ring["links"]], old @ U.T + b)
        np.testing.assert_allclose([link["ledger_com_m"] for link in ring["links"]], com)
        np.testing.assert_allclose(U @ U.T, np.eye(3), atol=1e-14)

    @unittest.skipUnless(importlib.util.find_spec("mujoco"), "MuJoCo required")
    def test_revised_middle_coils_at_equal_takeup(self):
        from ring_timing import revised_middle_rates, trial

        rates = revised_middle_rates()
        self.assertAlmostEqual(rates[0], 0.003399951, places=8)
        self.assertAlmostEqual(rates[2], 0.020125663, places=8)
        ref, new = [], []
        trial(0.50, 12, setup_samples=ref)
        trial(0.50, 12, setup_samples=new, revised_middle=True)

        def equal_stroke(rows):
            return next(r for r in rows if abs(r["t_s"] + 0.50) < 1e-8)

        a, b = equal_stroke(ref), equal_stroke(new)
        self.assertGreater(b["angle_2_0_rad"], a["angle_2_0_rad"])
        self.assertLess(b["angle_2_2_rad"], a["angle_2_2_rad"])

    @unittest.skipUnless(importlib.util.find_spec("mujoco"), "MuJoCo required")
    def test_ring_lead_and_failed_lift_replay(self):
        from ring_timing import trial

        prep = []
        control, _ = trial(0, 12, setup_samples=prep)
        early = []
        result, rows = trial(0.5, 12, setup_samples=early, observe_failed_lift=True)
        self.assertFalse(control["hanging_pass"])
        self.assertIsNone(control["lift_pass"])
        self.assertFalse(result["lift_pass"])
        self.assertEqual(rows[-1]["t_s"], 4.2)
        self.assertAlmostEqual(rows[-1]["elbow_deg"], 90, delta=1)
        # Before tripod closure the ring responds to its earlier take-up.
        before = next(r for r in prep if abs(r["t_s"] + 0.852) < 1e-8)
        after = next(r for r in early if abs(r["t_s"] + 0.852) < 1e-8)
        self.assertGreater(
            sum(
                after[f"angle_3_{i}_rad"] - before[f"angle_3_{i}_rad"] for i in range(3)
            ),
            0.1,
        )
        self.assertEqual(result["middle_release_N"], 0)

    @unittest.skipUnless(importlib.util.find_spec("mujoco"), "MuJoCo required")
    def test_middle_index_stations_release_gate(self):
        from tripod_trial import CASES, solve

        with (ROOT / "examples/tripod-cases.csv").open() as stream:
            archived = list(csv.DictReader(stream))
        for name in CASES:
            with self.subTest(station=name):
                result, samples = solve(name, 28)
                reference = next(
                    r
                    for r in archived
                    if r["case"] == name and r["cad_hulls"] == "False"
                )
                self.assertAlmostEqual(
                    result["index_release_N"],
                    float(reference["index_release_N"]),
                    delta=0.03,
                )
                self.assertEqual(result["tripod_samples"], 0)
                self.assertEqual(result["middle_release_N"], 0)
                self.assertFalse(result["hanging_pass"])
                self.assertIsNone(result["lift_pass"])
                self.assertAlmostEqual(samples[-1]["t_s"], 1.0)
                self.assertTrue(all("middle_pull_N" in row for row in samples))

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
