import csv
import re
import unittest
from pathlib import Path
from urllib.parse import urlsplit
from PIL import Image

from calculate import ROOT, LoadCase, PARAMETERS, solve


class ReleaseTests(unittest.TestCase):
    def test_document_links(self):
        docs=[p for p in [ROOT/"README.md",ROOT/"README.zh-CN.md",ROOT/"LICENSE"]]
        docs.extend((ROOT/"docs").glob("*.md"))
        for doc in docs:
            text=doc.read_text(encoding="utf-8")
            links=re.findall(r'\]\(([^)\s]+)\)|(?:src|href)="([^"]+)"',text)
            for first,second in links:
                url=urlsplit(first or second)
                if url.scheme or not url.path: continue
                with self.subTest(doc=doc.name,link=url.path):
                    path=(doc.parent/url.path).resolve()
                    self.assertTrue(path.is_relative_to(ROOT))
                    self.assertTrue(path.is_file())

    def test_public_figures(self):
        for name in ["assets/bracket-comparison.png","assets/free-body.png","examples/torque-overlay.png"]:
            with Image.open(ROOT/name) as image:
                self.assertGreaterEqual(image.width,800)
                image.verify()
        with Image.open(ROOT/"assets/cad-lift.gif") as image:
            self.assertGreater(image.n_frames,20)
            for k in range(image.n_frames):
                image.seek(k)
                image.load()

    def test_examples_match_code(self):
        with (ROOT/"examples/cases.csv").open() as stream:
            rows=list(csv.DictReader(stream))
        self.assertEqual([int(row["payload_g"]) for row in rows],[0,150,300])
        for row in rows:
            _,s=solve(LoadCase(float(row["payload_g"]),tuple(PARAMETERS["payload_position_mm"]),1.2,1.5))
            for key in ("hold_nm","lift_peak_abs_nm","design_hold_nm"):
                self.assertAlmostEqual(float(row[key]),s[key],delta=1e-9)
