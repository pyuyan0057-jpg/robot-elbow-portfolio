from pathlib import Path
import ctypes as C
from functools import lru_cache
import numpy as np, mujoco as mj, json, sys
from motion import S, D, hand

_lib = C.CDLL(str(D / ("dynamics.dll" if sys.platform == "win32" else "dynamics.so")))
arr = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")


class Reg(C.Structure):
    _fields_ = (
        [
            (k, C.c_int * n)
            for k, n in [
                ("armq", 7),
                ("armv", 7),
                ("fq", 14),
                ("fv", 14),
                ("digit", 14),
            ]
        ]
        + [
            (s, C.c_int)
            for s in "oppq oppv ball ballv table body palm little thumb".split()
        ]
        + [("labels", C.c_int * 1024), ("spring", C.c_double * 14)]
        + [
            (s, C.c_double)
            for s in "A H radius elastic dt tau sensorTransmission".split()
        ]
        + [(s, C.c_double * 7) for s in ["maxTorque", "kp", "kd"]]
        + [("feedforward", C.c_double), ("last_velocity", C.c_double * 7)]
    )


_lib.advance.argtypes = [C.c_void_p, C.c_void_p, C.POINTER(Reg), arr, C.c_int, arr, arr]
_lib.advance.restype = None


@lru_cache(1)
def source():
    return (D / "arm_hand.xml").read_text("utf-8")


def plant(o, pl, dt, configure=None):
    import xml.etree.ElementTree as E

    root = E.fromstring(source())
    cylinder = o == "cylinder"
    if cylinder:
        geom = root.find(".//geom[@name='ball_surface']")
        geom.set("type", "cylinder")
        geom.set("size", ".020 .085")
    if pl["c"].get("rolling_mm", 0) > 0:
        # MuJoCo rolling/torsional friction parameters have units of length.
        roll = pl["c"]["rolling_mm"] * 0.001
        E.SubElement(
            root.find("contact"),
            "pair",
            geom1="ball_surface",
            geom2="table",
            condim="6",
            friction=f".45 .45 .0004 {roll} {roll}",
            solref=".008 1",
        )
    if configure is not None:
        configure(root)
    m = mj.MjModel.from_xml_string(E.tostring(root, encoding="unicode"))
    m.opt.timestep = dt
    d = mj.MjData(m)
    reg = Reg()
    joints = [m.joint(n) for n in S["names"]]
    reg.armq[:] = [v.qposadr[0] for v in joints]
    reg.armv[:] = [v.dofadr[0] for v in joints]
    finger = [
        (i, m.joint(f"q{i}{k}")) for i, L in enumerate(S["lengths"]) for k in range(L)
    ]
    reg.digit[:] = [i for i, j in finger]
    reg.fq[:] = [j.qposadr[0] for i, j in finger]
    reg.fv[:] = [j.dofadr[0] for i, j in finger]
    reg.spring[:] = np.concatenate(S["springs"])
    reg.tau = pl["c"].get("lag_ms", 8.0) * 0.001
    reg.sensorTransmission = pl["c"].get("ring_efficiency", 1.0)
    if not (reg.tau > 0 and 0 < reg.sensorTransmission <= 1):
        raise ValueError("invalid ring transmission or sensor time constant")
    reg.A, reg.H, reg.radius = S["A"], S["H"], S["RC"]
    reg.elastic = 1 / (1 / 10000 + 0.003009214 * 0.813473 * 0.001)
    reg.maxTorque[:] = [48, 48, 8.3, 8.3, 8.3, 1.3, 1.3]
    reg.kp[:] = [950, 950, 400, 340, 110, 75, 75]
    reg.kd[:] = [55, 55, 15, 13, 3, 1.45, 1.45]
    scale = np.asarray(pl["c"].get("stiffness_scale", [1.] * 7))
    if scale.shape != (7,) or not np.all(np.isfinite(scale) & (scale > 0)):
        raise ValueError("Expected seven positive joint gain scales")
    reg.kp[:] = np.asarray(reg.kp) * scale
    reg.kd[:] = np.asarray(reg.kd) * np.sqrt(scale)
    reg.feedforward = pl["c"].get("inertia_gain", 0.)
    opp = m.joint("thumb_opposition")
    reg.oppq = opp.qposadr[0]
    reg.oppv = opp.dofadr[0]
    reg.table = m.geom("table").id
    reg.ball = m.geom("ball_surface").id
    ob = m.body("ball")
    reg.body = ob.id
    reg.palm = m.body("palm").id
    reg.little = m.geom("skin_f3j2_1").id
    reg.thumb = m.geom("skin_thumb1_1").id
    ids = [0, 2, 3, 4]
    reg.labels[:] = [7] * 1024
    assert m.ngeom < 1024
    for j in range(m.ngeom):
        n = m.geom(j).name
        if n.startswith("skin_f") and n[6].isdigit():
            reg.labels[j] = ids[int(n[6])]
        elif n.startswith("skin_thumb"):
            reg.labels[j] = 1
        elif n.startswith("skin_palm"):
            reg.labels[j] = 5
    reg.labels[reg.table] = 6
    cyl = o == "cylinder"
    r = 0.02
    h = 0.085 if cyl else 0.02
    table_y = pl["wp"][1] + h
    table = m.geom("table")
    p = pl["wp"]
    _, ori = hand(pl["q"])
    from scipy.spatial.transform import Rotation as Rot

    sh = Rot.from_euler("XZ", [pl["c"]["sh"], -pl["c"]["out"]], degrees=True)
    off = sh.apply([0, 0.36, 0]) - [0, 0.36, 0]
    toward = (p - off).copy()
    toward[1] = 0
    toward /= np.linalg.norm(toward)
    wide = pl["c"].get("wide_table", False)
    m.geom_size[table.id] = [0.35 if wide else 0.25, 0.015, 0.36 if wide else 0.25]
    m.geom_rbound[table.id] = np.linalg.norm(m.geom_size[table.id])
    m.geom_aabb[table.id, 3:] = m.geom_size[table.id]
    m.geom_pos[table.id] = p + toward * (
        (0.35 if wide else 0.25) - pl["c"]["inset_mm"] * 0.001
    )
    m.geom_pos[table.id, 1] = table_y + 0.015
    m.geom_quat[table.id] = Rot.from_euler(
        "y", np.arctan2(-toward[2], toward[0])
    ).as_quat()[[3, 0, 1, 2]]
    if cyl:
        m.geom_type[reg.ball] = mj.mjtGeom.mjGEOM_CYLINDER
        m.geom_size[reg.ball] = [r, h, 0]
        m.geom_rbound[reg.ball] = np.hypot(r, h)
        m.geom_aabb[reg.ball] = [0, 0, 0, r, r, h]
        m.body_inertia[ob.id] = [0.033 * (3 * r * r + 4 * h * h) / 12] * 2 + [
            0.033 * r * r / 2
        ]
    if pl["c"].get("mu_override") is not None:
        for j in range(m.ngeom):
            if m.geom(j).name.startswith("skin_") or j == reg.ball:
                m.geom_friction[j, 0] = pl["c"]["mu_override"]
    mj.mj_setConst(m, d)
    mj.mj_resetData(m, d)
    b = m.joint("ball_free")
    bq = b.qposadr[0]
    reg.ballv = b.dofadr[0]
    d.qpos[bq : bq + 3] = pl["object_pos"]
    d.qpos[bq + 3 : bq + 7] = [2**-0.5, 2**-0.5, 0, 0]
    d.qpos[list(reg.armq)] = pl["entry"]
    d.qpos[list(reg.fq)] = S["initial"][o]["angles"]
    d.qpos[reg.oppq] = pl["tape"][0, -1]
    if pl["c"].get("jam"):
        for i, j in finger:
            if i == 3:
                q = d.qpos[j.qposadr[0]]
                m.jnt_range[j.id] = [q - 0.0005, q + 0.0005]
                m.jnt_limited[j.id] = 1
    mj.mj_forward(m, d)
    assert (
        m.nmocap == 0
        and m.neq == 0
        and np.linalg.norm(d.xpos[reg.palm] - hand(pl["entry"])[0]) < 2e-6
    )
    initial = max(
        [0]
        + [-c.dist for c in d.contact if reg.ball in c.geom and reg.table not in c.geom]
    )
    return m, d, reg, table_y, initial


def integrate(m, d, r, ref, n, sensor, out):
    _lib.advance(m._address, d._address, C.byref(r), ref, n, sensor, out)
