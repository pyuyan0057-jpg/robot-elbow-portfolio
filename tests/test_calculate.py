import csv
import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from calculate import LoadCase, PARAMETERS, ROOT, solve


def case(g, pos=None, time=1.2, factor=1.5):
    return LoadCase(g,tuple(PARAMETERS["payload_position_mm"] if pos is None else pos),time,factor)


class ElbowTests(unittest.TestCase):
    def test_archive_regression(self):
        for g,hold,peak in [(0,1.80847,1.74634),(150,2.38983,2.30773),(300,2.97119,2.86912)]:
            with self.subTest(g=g):
                rows,s=solve(case(g))
                self.assertAlmostEqual(s["total_moving_kg"],1.60106+g/1000,delta=5e-6)
                self.assertAlmostEqual(s["hold_nm"],hold,delta=1e-5)
                self.assertAlmostEqual(s["lift_peak_abs_nm"],peak,delta=2e-5)
                self.assertAlmostEqual(s["output_max_rpm"],16.67,delta=.005)
                self.assertAlmostEqual(rows[-1]["angle_deg"],90)
                self.assertEqual(rows[-1]["phase"],"horizontal_hold")
        self.assertAlmostEqual(solve(case(150))[1]["design_hold_nm"],3.5847,delta=.00005)

    def test_independent_load_increment(self):
        _,light=solve(case(150))
        _,heavy=solve(case(300))
        # Independent free-body calculation; constants deliberately taken from the archive.
        expected=.150*9.80665*.39521436491736295
        self.assertAlmostEqual(heavy["hold_nm"]-light["hold_nm"],expected,delta=1e-9)

    def test_ledger(self):
        with (ROOT/"data/moving-mass-ledger.csv").open() as f:
            rows=list(csv.DictReader(f))
        for key,expected in [("mass_kg",PARAMETERS["moving_mass_kg"]),
                             ("axis_inertia_kg_m2",PARAMETERS["axis_inertia_kg_m2"])]:
            self.assertAlmostEqual(sum(float(r[key]) for r in rows),expected,places=12)
        for i,axis in enumerate("uvw"):
            self.assertAlmostEqual(sum(float(r[f"first_{axis}_kg_m"]) for r in rows),
                                   PARAMETERS["moving_mass_kg"]*PARAMETERS["com_m"][i],places=12)

    def test_parameter_response(self):
        b=solve(case(150))[1]
        p=PARAMETERS["payload_position_mm"]
        long=solve(case(150,(p[0]+100,p[1],p[2])))[1]
        wide=solve(case(150,(p[0],p[1],p[2]+10)))[1]
        self.assertAlmostEqual(long["hold_nm"]-b["hold_nm"],.15*9.80665*.1)
        self.assertGreater(long["axis_inertia_kg_m2"],b["axis_inertia_kg_m2"])
        self.assertAlmostEqual(wide["axis_inertia_kg_m2"],b["axis_inertia_kg_m2"])
        self.assertGreater(wide["bearing_plus_n"],b["bearing_plus_n"])
        rows,fast=solve(case(150,time=.2,factor=2))
        self.assertAlmostEqual(fast["hold_nm"],b["hold_nm"])
        self.assertAlmostEqual(fast["output_max_rpm"],6*b["output_max_rpm"])
        self.assertAlmostEqual(fast["design_hold_nm"],2*b["hold_nm"])
        self.assertGreater(fast["lift_peak_abs_nm"],b["lift_peak_abs_nm"])
        self.assertTrue(any(r["output_nm"]<0 for r in rows))

    def test_one_sided_switches(self):
        rows,s=solve(case(150))
        for t in (0,.3,.9,1.2):
            vals=[r for r in rows if abs(r["time_s"]-t)<1e-12]
            self.assertGreaterEqual(len(vals),2)
            self.assertAlmostEqual(vals[0]["angle_deg"],vals[-1]["angle_deg"])
            self.assertNotEqual(vals[0]["acceleration_rad_s2"],vals[-1]["acceleration_rad_s2"])
        self.assertAlmostEqual(s["peak_time_s"],.9)
        self.assertAlmostEqual(s["peak_angle_deg"],75)

    def test_invalid(self):
        for arg in [case(-10),case(math.inf),case(1,time=0),case(1,factor=.9),case(1,(0,math.nan,0))]:
            with self.assertRaises(ValueError):
                solve(arg)

    def test_public_cli(self):
        with tempfile.TemporaryDirectory() as tmp:
            for args in [[],["--payload-position-mm","250","0","0"]]:
                cmd=[sys.executable,str(ROOT/"calculate.py"),"--payload-g","150","--output-dir",tmp,*args]
                ran=subprocess.run(cmd,capture_output=True,text=True,check=True)
                self.assertIn("Horizontal hold",ran.stdout)
            outputs=list(Path(tmp).iterdir())
            self.assertEqual(len(outputs),2)
            for out in outputs:
                self.assertEqual((out/"torque.png").read_bytes()[:8],b"\x89PNG\r\n\x1a\n")
                self.assertEqual(json.loads((out/"inputs.json").read_text())["case"]["payload_g"],150)
                with (out/"trajectory.csv").open() as f: rows=list(csv.DictReader(f))
                self.assertEqual(rows[-1]["phase"],"horizontal_hold")
                self.assertAlmostEqual(float(rows[-1]["output_nm"]),json.loads((out/"summary.json").read_text())["hold_nm"])


if __name__=="__main__":
    unittest.main()
