import math
import unittest

import elbow_control as control


class ControlTests(unittest.TestCase):
    def test_held_output_matches_weight(self):
        for g in (0, 150, 300):
            inertia, moment_u, _ = control.plant_terms(g)
            self.assertGreater(inertia,0)
            self.assertAlmostEqual(moment_u,1.808471918+g*9.80665*0.3952143649/1000,places=4)
            self.assertAlmostEqual(control.gravity(math.pi/2,control.plant_terms(g)),moment_u)
            q,v,t = control.advance(math.pi/2,0,moment_u,moment_u,0,control.plant_terms(g),.001)
            self.assertAlmostEqual(q,math.pi/2)
            self.assertAlmostEqual(v,0)
            self.assertAlmostEqual(t,moment_u)

    def test_integrator_winds_down_but_not_up_at_limit(self):
        integral,request,limited=control.command("PIDG",1,0,0,0,2)
        self.assertEqual(integral,0)
        self.assertGreater(request,limited)
        integral,_,_=control.command("PIDG",-.01,0,4,.01,2)
        self.assertLess(integral,.01)

    def test_pulse_timing_and_tracking_bounds(self):
        case=control.Experiment("pulse","PDG",150,150,-1)
        rows=control.simulate(case)
        active=[r for r in rows if r["external_nm"]]
        self.assertEqual(len(active),200)
        self.assertAlmostEqual(active[0]["time_s"],2.5)
        self.assertLess(max(abs(r["applied_nm"]) for r in rows),case.limit_nm+1e-9)
        measured=control.metrics(rows,case)
        self.assertAlmostEqual(measured["recovery_s"],.137,places=3)
        self.assertGreater(measured["pulse_max_abs_deg"],control.BAND_DEG)

    def test_underpowered_arm_cannot_reach_horizontal(self):
        case=control.Experiment("limited","PIDG",300,150,0,2.5)
        self.assertLess(case.limit_nm,control.plant_terms(case.actual_g)[1])
        r=control.metrics(control.simulate(case),case)
        self.assertGreater(r["hold_mean_abs_deg"],35)
        self.assertGreater(r["saturated_fraction"],0.5)
