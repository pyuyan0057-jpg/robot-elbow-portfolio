import csv
import json
import math
import unittest
from calculate import ROOT


class GraspAndStructureEvidence(unittest.TestCase):
    def rows(self, name):
        with (ROOT/"data"/name).open(encoding="utf-8-sig") as f:
            return list(csv.DictReader(f))

    def test_moment_direction_and_contact_conclusion(self):
        reg=json.loads((ROOT/"data/registered-pinch.json").read_text())
        _, y, z=reg["contact_line_moment_arm_hand_mm"]
        rows=self.rows("pinch-angle-screen.csv")
        self.assertEqual(len(rows),11)
        for r in rows:
            a=math.radians(float(r["angle_deg"]))
            expected=.15*9.81*(y*math.cos(a)-z*math.sin(a))
            self.assertAlmostEqual(expected,float(r["gravity_axis_moment_Nmm"]),places=10)
        self.assertEqual([r["angle_deg"] for r in rows if r["local_stability"]=="True"],["90"])

    def test_fit_coverage(self):
        rows=self.rows("assembly-fit.csv")
        self.assertEqual(len(rows),21)
        for case in ("Baseline","Fixed_Yoke","Moving_Adaptor"):
            selected=[r for r in rows if r["variant"]==case]
            self.assertEqual(sorted(int(r["elbow_deg"]) for r in selected),list(range(0,91,15)))
            self.assertTrue(all(int(r["resolved_active_references"])==263 and int(r["volume_interference_count"])==0 for r in selected))

    def test_candidate_gains_match_native_fields(self):
        fields=self.rows("structure-mesh-comparison.csv")
        results=self.rows("candidate-tradeoff.csv")
        for r in results:
            case=r["candidate"]
            fine=min((f for f in fields if f["part"]==case),key=lambda f:float(f["mesh_mm"]))
            self.assertAlmostEqual(float(fine["sag_mm"]),float(r["candidate_sag_mm"]))
            gain=(float(r["original_sag_mm"])-float(fine["sag_mm"]))/float(r["added_mass_g"])
            self.assertAlmostEqual(gain,float(r["saved_mm_per_added_g"]))
