import importlib.util
import unittest

import numpy as np


class CoordinationTest(unittest.TestCase):
    @unittest.skipUnless(importlib.util.find_spec("scipy"), "SciPy required")
    def test_wrench_normal_direction_and_fixed_load(self):
        from tripod_wrench import supports_weight

        point = {"normal": [0, -1, 0], "arm_m": [0, 0.01, 0], "normal_N": 0.1}
        self.assertTrue(supports_weight([point], 0.033, 0.35))
        self.assertFalse(
            supports_weight([point], 0.033, 0.35, outer=True, fixed_normals=True)
        )
        point["normal_N"] = 0.033 * 9.80665
        self.assertTrue(supports_weight([point], 0.033, 0.35, fixed_normals=True))

    def test_feedback_limits_and_unloading(self):
        from tripod_coordination import ThreePadFeedback

        control = ThreePadFeedback()
        np.testing.assert_allclose(control.update(-0.5, [0] * 3, [0] * 3), 0)
        for i in range(300):
            before = control.offset.copy()
            control.update(-0.3 + i * 0.002, [2, 0, 1], [25, 0, 0])
            self.assertLessEqual(max(abs(control.offset - before)), 0.000016000001)
        self.assertLess(control.offset[0], 0)
        self.assertGreater(control.offset[1], 0)
        self.assertLessEqual(max(abs(control.offset)), 0.003)

    @unittest.skipUnless(importlib.util.find_spec("mujoco"), "MuJoCo required")
    def test_lock_keeps_pickup_and_moves_cup_with_thumb(self):
        from ring_timing import schedule
        from tripod_trial import tripod_model
        from tripod_coordination import lock_thumb

        cfg, _ = tripod_model(
            "indexed", 28, 33, 0.00025, None, None, schedule(0.5, 12, False)
        )
        digit = cfg["digits"][1]
        ball = np.array(cfg["ball_in_palm"])
        pivot = np.array(digit["links"][0]["root"])
        pad = np.array(digit["pad_center"])
        gap = np.linalg.norm(cfg["cup_cells"][1][0] - pad)
        lock_thumb(cfg, 8)
        np.testing.assert_allclose(cfg["ball_in_palm"], ball)
        np.testing.assert_allclose(digit["links"][0]["root"], pivot)
        self.assertGreater(np.linalg.norm(digit["pad_center"] - pad), 0.001)
        self.assertAlmostEqual(
            np.linalg.norm(cfg["cup_cells"][1][0] - digit["pad_center"]), gap
        )
