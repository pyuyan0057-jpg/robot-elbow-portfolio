"""Instrument the released C4 MuJoCo tasks without altering their commands or contacts."""
import argparse
import ctypes as ct
import json
import os
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

os.environ["OPENBLAS_NUM_THREADS"] = "1"
import mujoco as mj
import numpy as np

HERE = Path(__file__).resolve().parent
ENGINE = HERE.parent / "seven-axis-pick-place-c2"
C4 = HERE.parent / "seven-axis-control-c4"
OUT = HERE.parents[1] / "outputs" / "tactile-c5"
ZONES = json.loads((HERE / "zones.json").read_text())
CASES = {
    "nominal": {}, "right2": {"unknown_mm":[2,0,0]}, "left2":{"unknown_mm":[-2,0,0]},
    "delay12":{"lag_ms":12}, "mu055":{"mu_override":.55}, "mu035":{"mu_override":.35},
    "mu075":{"mu_override":.75}, "mu045":{"mu_override":.45}, "mu025":{"mu_override":.25},
    "mu055_right2":{"mu_override":.55,"unknown_mm":[2,0,0]},
}
# Independent validation set: 180 ms external load during the high hold in world X or +Y (gravity).
PULSES = {"push_x1":[1,0,0], "push_x3":[3,0,0], "push_nx3":[-3,0,0],
          "push_y1":[0,1,0], "push_y3":[0,3,0], "push_y6":[0,6,0]}
CASES.update({name:{} for name in PULSES})
VALIDATION={"val_mu062_offset":{"mu_override":.62,"unknown_mm":[-1,0,1]},
            "val_mu040_offset":{"mu_override":.40,"unknown_mm":[1,0,-1]},
            "val_push_diag":{"unknown_mm":[-1,0,0]}, "val_push_down":{"unknown_mm":[1,0,0]}}
PULSES.update({"val_push_diag":[-2.7,1.4,0], "val_push_down":[0,4.5,.8]})
CASES.update(VALIDATION)
sys.path.insert(0, str(ENGINE))
LIB = OUT / ("read_zones.dll" if os.name=="nt" else "read_zones.so")


def build():
    from build_runtime import build as build_engine
    OUT.mkdir(parents=True, exist_ok=True)
    build_engine()
    sdk = Path(mj.__file__).parent
    flags = ["-static"] if os.name=="nt" else ["-fPIC","-Wl,-rpath,"+str(sdk)]
    lib = sdk/"mujoco.dll" if os.name=="nt" else next(sdk.glob("libmujoco.so*"))
    subprocess.run([os.environ.get("CXX","g++"),"-std=c++17","-O2","-shared","-DREPRODUCE_C5_LEGACY_ORIGIN",*flags,
                    "-I"+str(sdk/"include"),str(HERE/"read_zones.cpp"),str(lib),"-o",str(LIB)],check=True)


class Recorder:
    def configure(self, root):
        for name, spec in ZONES.items():
            body = root.find(f".//body[@name='{spec['body']}']")
            def metres(key):
                return " ".join(str(x/1000) for x in spec[key])
            ET.SubElement(body, "site", name="tactile_"+name, type="ellipsoid",
                          pos=metres("center_mm"), size=metres("radii_mm"), rgba=".1 .8 .68 .35", group="4")

    def sample(self, m, d, r, k, t, times):
        if k == 0:
            self.times = times
            n = round(times[-1]/.002)+4
            self.data = np.zeros((n, 25))
            self.sites = np.array([m.site("tactile_"+z).id for z in ZONES],np.int32)
            owners = {m.body(spec["body"]).id:j for j,spec in enumerate(ZONES.values())}
            self.glove = np.array([m.geom(i).name.startswith("skin_") for i in range(m.ngeom)],np.int32)
            self.zmap = np.array([owners.get(m.geom_bodyid[i],-1) if self.glove[i] else -1
                                  for i in range(m.ngeom)],np.int32)
            self.lib = ct.CDLL(str(getattr(self, "library", LIB)))
            ni = np.ctypeslib.ndpointer(np.int32,flags="C_CONTIGUOUS")
            nd = np.ctypeslib.ndpointer(np.float64,flags="C_CONTIGUOUS")
            self.lib.read_zones.argtypes = [ct.c_void_p,ct.c_void_p,ni,ni,ct.c_int,ct.c_int,ni,nd,nd]
            self.lib.read_zones.restype = None
        row = self.data[k]
        row[0] = t
        self.lib.read_zones(m._address,d._address,self.zmap,self.sites,r.ball,r.table,self.glove,
                            row[1:19],row[19:22])
        row[22:25] = 1000*d.xmat[r.palm].reshape(3,3).T @ (d.xpos[r.body]-d.xpos[r.palm])
        self.count=k+1


def solve(pair):
    from trial import execute
    obj, case = pair
    cfg = json.loads((ENGINE/f"recipe_{obj}.json").read_text())
    cfg.update(json.loads((C4/f"{obj}_changes.json").read_text()))
    cfg.update(CASES[case])
    recorder = Recorder()
    tag = OUT/f"{obj}_{case}"
    # A tag keeps the complete trajectory even when the object escapes the hand.
    def pulse(d,r,t,times):
        elapsed=t-times[1]-.35
        weight=np.sin(np.pi*elapsed/.18)**2 if 0<=elapsed<=.18 else 0.
        d.xfrc_applied[r.body,:3]=np.asarray(PULSES[case])*weight

    result = execute(obj,cfg,tag=str(tag),observer=recorder,
                     disturbance=pulse if case in PULSES else None)
    np.savez_compressed(str(tag)+"_sensors.npz",trace=recorder.data[:recorder.count],times=recorder.times)
    print(obj,case,result["passed"],round(result["hold_span_mm"],2),flush=True)


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--workers",type=int,default=3)
    parser.add_argument("--pulses-only", action="store_true")
    parser.add_argument("--validation-only",action="store_true")
    args=parser.parse_args()
    selected=VALIDATION if args.validation_only else PULSES if args.pulses_only else CASES
    build()
    with ProcessPoolExecutor(args.workers) as pool:
        list(pool.map(solve,[(obj,case) for case in selected for obj in ("cylinder","ball")]))
