"""Release a preloaded 6-DOF ball and lift a tendon-driven two-pad pinch (MuJoCo)."""

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

from calculate import PARAMETERS, ROOT
from elbow_control import reference, write_csv

PALM = np.array([0.0, 0.290, 0.0])  # axes X,Y,Z = elbow W,U,V at horizontal
A, RC = 0.00898, 0.001655
H = 0.0063 + 0.000195 + RC


def vec(x):
    return " ".join(f"{i:.15g}" for i in x)


def link_name(i, j):
    return f"E3_F{(0,4,1,2)[i]}_{j}-1"


def build(
    cfg,
    dt,
    mu,
    response,
    private_hulls=None,
    ball_mass=0.150,
    inertia_factor=0.4,
    damping_scale=1.0,
):
    xml = ET.Element("mujoco", model="Free ball lift")
    ET.SubElement(
        xml, "compiler", angle="radian", inertiafromgeom="false", autolimits="true"
    )
    ET.SubElement(
        xml,
        "option",
        gravity="0 9.80665 0",
        timestep=str(dt),
        integrator="implicitfast",
        solver="Newton",
        iterations="80",
        tolerance="1e-11",
        cone="elliptic",
    )
    defaults = ET.SubElement(xml, "default")
    ET.SubElement(
        defaults,
        "geom",
        contype="2",
        conaffinity="1",
        condim="3",
        friction=f"{mu} 0 0",
        solref=f"{response} 1",
        solimp=".97 .995 .0005",
    )
    ET.SubElement(
        defaults,
        "joint",
        solreflimit=".001 1",
        solimplimit=".99 .999 .0001",
        margin="0",
    )
    world = ET.SubElement(xml, "worldbody")
    arm = ET.SubElement(world, "body", name="arm")
    ET.SubElement(
        arm,
        "joint",
        name="elbow",
        axis="1 0 0",
        damping=".045",
        armature=str(
            PARAMETERS["motor_rotor_inertia_kg_m2"] * PARAMETERS["gear_ratio"] ** 2
        ),
    )
    palm = ET.SubElement(arm, "body", name="palm", pos=vec(PALM))
    bodies = {"palm": (palm, np.zeros(3))}
    finger_mass = 0.0
    first = np.zeros(3)
    finger_inertia = 0.0
    for i, digit in enumerate(cfg["digits"]):
        parent = palm
        previous = np.zeros(3)
        for j, link in enumerate(digit["links"]):
            root = np.array(link["root"])
            name = link_name(i, j)
            bone = ET.SubElement(parent, "body", name=name, pos=vec(root - previous))
            bodies[name] = (bone, root)
            angle = digit["reference_rad"][j]
            ET.SubElement(
                bone,
                "joint",
                name=f"q{i}{j}",
                axis=vec(link["axis"]),
                range=vec([-angle, digit["upper_rad"][j] - angle]),
                limited="true",
                damping=str(0.0007 * damping_scale),
                armature=".0000008",
            )
            m = link["mass"]
            com = np.array(link["com"])
            global_com = PALM + com
            # Approximate the centroidal link moment with a finite-width rod.
            inertia = m * (link["length_m"] ** 2 + 0.012**2) / 12
            finger_mass += m
            first += m * global_com
            finger_inertia += inertia + m * np.dot(global_com[1:], global_com[1:])
            if i == 1 and j == len(digit["links"]) - 1 and "pad_delta_kg" in cfg:
                dm = cfg["pad_delta_kg"]
                com = (m * com + dm * np.array(cfg["pad_delta_com_m"])) / (m + dm)
                m += dm
                inertia = m * (link["length_m"] ** 2 + 0.012**2) / 12
            ET.SubElement(
                bone,
                "inertial",
                mass=str(m),
                pos=vec(com - root),
                diaginertia=vec([inertia] * 3),
            )
            parent, previous = bone, root
        pad_geom = ET.SubElement(
            parent,
            "geom",
            name="pad_" + digit["name"],
            type="sphere",
            size=str(cfg["pad_radius_m"]),
            pos=vec(np.array(digit["pad_center"]) - previous),
        )
        if digit.get("hardware_pad"):
            pad_geom.set("contype", "0")
            pad_geom.set("conaffinity", "0")
        if i == 1 and cfg.get("cut_cup"):
            pad_geom.set("contype", "0")
            pad_geom.set("conaffinity", "0")
        if cfg.get("adjacent_contact") and i in (0, 2):
            pad_geom.set("contype" if i == 2 else "conaffinity", "6" if i == 2 else "5")
    reg = json.loads((ROOT / "data/registered-pinch.json").read_text())
    m = reg["moving_mass_empty_kg"] - finger_mass
    com = (
        reg["moving_mass_empty_kg"] * np.array(reg["held_empty_com_m"])[[2, 0, 1]]
        - first
    ) / m
    inertia = reg["empty_axis_inertia"] - finger_inertia - m * np.dot(com[1:], com[1:])
    assert inertia > 0
    ET.SubElement(
        arm, "inertial", mass=str(m), pos=vec(com), diaginertia=vec([inertia] * 3)
    )

    if private_hulls:
        asset = ET.SubElement(xml, "asset")
        for i, entry in enumerate(private_hulls):
            body, origin = bodies[entry["parent"]]
            name = f"hull{i}"
            vertices = np.array(entry["vertices"]) - origin
            ET.SubElement(asset, "mesh", name=name, vertex=vec(vertices.ravel()))
            geom = ET.SubElement(
                body, "geom", name="hard_" + entry["name"], type="mesh", mesh=name
            )
            if entry.get("side") == "middle":
                geom.set("contype", "6")
            elif entry.get("side") == "adjacent":
                geom.set("conaffinity", "5")
    if "cup_cells" in cfg:
        asset = xml.find("asset")
        if asset is None:
            asset = ET.SubElement(xml, "asset")
        thumb, origin = bodies[link_name(1, 1)]
        for number, vertices in enumerate(cfg["cup_cells"]):
            name = f"cup_{number}"
            ET.SubElement(
                asset, "mesh", name=name, vertex=vec((vertices - origin).ravel())
            )
            ET.SubElement(thumb, "geom", name=name, type="mesh", mesh=name)
    ET.SubElement(
        world,
        "geom",
        name="catch_plane",
        type="plane",
        size="1 1 .01",
        pos="0 .80 0",
        quat=vec([math.sqrt(0.5), math.sqrt(0.5), 0, 0]),
    )
    ball = ET.SubElement(
        world, "body", name="ball", pos=vec(PALM + cfg["ball_in_palm"])
    )
    ET.SubElement(ball, "freejoint", name="ball_free")
    I = inertia_factor * ball_mass * cfg["radius_m"] ** 2
    ET.SubElement(
        ball, "inertial", mass=str(ball_mass), pos="0 0 0", diaginertia=vec([I] * 3)
    )
    ET.SubElement(
        ball,
        "geom",
        name="ball_surface",
        type="sphere",
        size=str(cfg["radius_m"]),
        contype="1",
        conaffinity="2",
    )
    eq = ET.SubElement(xml, "equality")
    ET.SubElement(
        eq,
        "weld",
        name="pickup_fixture",
        body1="palm",
        body2="ball",
        solref=".0007 1",
        solimp=".999 .9999 .0001",
    )
    return mj.MjModel.from_xml_string(ET.tostring(xml, encoding="unicode"))


def angular_velocity(model, data, body):
    value = np.empty(6)
    mj.mj_objectVelocity(model, data, mj.mjtObj.mjOBJ_BODY, body, value, 0)
    return value[:3], value[3:]


def point_velocity(model, data, body, point):
    omega, velocity = angular_velocity(model, data, body)
    # mj_objectVelocity uses the body's inertial-frame centre for mjOBJ_BODY.
    return velocity + np.cross(omega, point - data.xipos[body])


def run(
    cfg,
    *,
    dt=0.00025,
    mu=0.35,
    lift_s=1.2,
    response=0.003,
    prep=0.35,
    dwell=2.0,
    horizontal=False,
    private_hulls=None,
    inertia_factor=0.4,
    damping_scale=1.0,
    transfer_s=0.0,
    close_s=0.0,
    ball_mass=0.150,
    hanging_s=0.0,
    hanging_only=False,
    setup_samples=None,
):
    m = build(
        cfg,
        dt,
        mu,
        response,
        private_hulls,
        inertia_factor=inertia_factor,
        damping_scale=damping_scale,
        ball_mass=ball_mass,
    )
    d = mj.MjData(m)
    ball = m.body("ball").id
    palm = m.body("palm").id
    ballgeom = m.geom("ball_surface").id
    names = [digit["name"] for digit in cfg["digits"]]
    padids = [m.geom("pad_" + s).id for s in names]
    padbodies = [m.geom_bodyid[g] for g in padids]
    pad_owner = {g: i for i, g in enumerate(padids)}
    for k in range(len(cfg.get("cup_cells", []))):
        pad_owner[m.geom(f"cup_{k}").id] = 1
    initial = np.array(cfg["ball_in_palm"])
    reg = json.loads((ROOT / "data/registered-pinch.json").read_text())
    q0 = math.pi / 2 if horizontal else 0.0
    arm = m.jnt_qposadr[m.joint("elbow").id]
    d.qpos[arm] = q0
    R = np.array(
        [[1, 0, 0], [0, math.cos(q0), -math.sin(q0)], [0, math.sin(q0), math.cos(q0)]]
    )
    free = m.jnt_qposadr[m.joint("ball_free").id]
    d.qpos[free : free + 3] = R @ (PALM + initial)
    d.qpos[free + 3 : free + 7] = [math.cos(q0 / 2), math.sin(q0 / 2), 0, 0]
    mj.mj_forward(m, d)
    for g, expected in zip(padids, cfg["digits"]):
        assert (
            np.linalg.norm(d.geom_xpos[g] - (R @ (PALM + expected["pad_center"])))
            < 1e-8
        )
        if not close_s:
            assert abs(np.linalg.norm(d.geom_xpos[g] - d.xpos[ball]) - 0.0192) < 1e-8
    assert (
        abs(
            m.body_mass.sum()
            - reg["moving_mass_empty_kg"]
            - ball_mass
            - cfg.get("pad_delta_kg", 0)
        )
        < 1e-8
    )
    digits = []
    for i, entry in enumerate(cfg["digits"]):
        ids = [m.joint(f"q{i}{j}").id for j in range(len(entry["links"]))]
        digits.append(
            (
                m.jnt_qposadr[ids],
                m.jnt_dofadr[ids],
                np.array(entry["reference_rad"]),
                np.array(entry["spring_Nm_rad"]),
                entry["locked_stroke_m"],
            )
        )
    if close_s:
        if not 0 < close_s < prep:
            raise ValueError("The supported closing ramp must finish before release.")
        for entry, (qa, _, ref, _, _) in zip(cfg["digits"], digits):
            d.qpos[qa] = np.asarray(entry.get("initial_rad", 0.0)) - ref
        mj.mj_forward(m, d)
    first = reg["loaded_com_m"]
    loaded_mass = reg["moving_mass_empty_kg"] + 0.15
    if ball_mass != 0.150:
        loaded_mass = reg["moving_mass_empty_kg"] + ball_mass
        first = (
            reg["moving_mass_empty_kg"] * np.array(reg["held_empty_com_m"])
            + ball_mass * np.array(reg["ball_uvw_m"])
        ) / loaded_mass
    ctl_steps = round(0.001 / dt)
    sample_steps = round(0.002 / dt)
    if (
        ctl_steps < 1
        or not math.isclose(ctl_steps * dt, 0.001)
        or not math.isclose(sample_steps * dt, 0.002)
    ):
        raise ValueError("Choose a timestep dividing both 1 and 2 ms.")
    prep_ticks = round(prep / dt)
    lift_start = transfer_s + hanging_s
    end = lift_start if hanging_only else lift_start + lift_s + dwell
    torque = request = 0.0
    prev = q0
    vel = 0.0
    trace = []
    start_ball = None
    first_floor = None
    first_loss = None
    first_hard = None
    free_spin_path = 0.0
    rolling = [0.0] * len(names)
    swipe = [0.0] * len(names)
    previous_local = [None] * len(names)
    last_pad_time = None
    at_release = None
    first_pad_break = None
    first_hard_time = None
    fixture_reaction = np.zeros(m.nv)
    ball_dof = m.jnt_dofadr[m.joint("ball_free").id]
    approach_peak = 0.0
    adjacent_pair = ""
    adjacent_peak = 0.0
    for k in range(prep_ticks + round(end / dt) + 1):
        t = (k - prep_ticks) * dt
        if k == prep_ticks:
            mj.mj_forward(m, d)
            multipliers = np.zeros(d.nefc)
            eqrows = d.efc_type == int(mj.mjtConstraint.mjCNSTR_EQUALITY)
            assert eqrows.sum() == 6
            multipliers[eqrows] = d.efc_force[eqrows]
            mj.mj_mulJacTVec(m, d, fixture_reaction, multipliers)
            d.eq_active[0] = False
        if t >= 0 and transfer_s:
            # Linearly remove the last measured fixture wrench in the hanging pose.
            fraction = max(0.0, 1.0 - t / transfer_s)
            d.qfrc_applied[ball_dof : ball_dof + 6] = (
                fraction * fixture_reaction[ball_dof : ball_dof + 6]
            )
        # Forward call places contacts, velocities and forces at the same integration time.
        if k % ctl_steps == 0:
            target, speed = (
                (q0, 0.0)
                if t < lift_start or horizontal or hanging_only
                else reference(t - lift_start, lift_s)
            )
            vel += (1 - math.exp(-0.001 / 0.006)) * ((d.qpos[arm] - prev) / 0.001 - vel)
            prev = d.qpos[arm]
            grav = (
                loaded_mass
                * 9.80665
                * (first[0] * math.sin(prev) + first[1] * math.cos(prev))
            )
            request = np.clip(
                18 * (target - prev) + 1.8 * (speed - vel) + grav, -5.0, 5.0
            )
        torque += (1 - math.exp(-dt / 0.012)) * (request - torque)
        d.qfrc_applied[0] = torque
        tensions = []
        closure = min(1.0, (k * dt) / close_s) if close_s else 1.0
        for entry, (qa, da, ref, ks, stroke) in zip(cfg["digits"], digits):
            takeup = closure * stroke
            if "start_s" in entry:
                start = entry["initial_stroke_m"]
                fraction = np.clip((t - entry["start_s"]) / close_s, 0.0, 1.0)
                takeup = start + fraction * (stroke - start)
            q = ref + d.qpos[qa]
            short = np.sum(2 * A * (1 - np.cos(q / 2)) + 2 * H * np.sin(q / 2) - RC * q)
            tension = 10000 * max(0.0, takeup - short)
            tensions.append(tension)
            d.qfrc_applied[da] = (
                tension * (A * np.sin(q / 2) + H * np.cos(q / 2) - RC) - ks * q - 0.002
            )
        if k % sample_steps == 0 or k == prep_ticks:
            if t <= 0:
                approach_peak = max(approach_peak, max(tensions))
            mj.mj_forward(m, d)
            if not np.all(np.isfinite(d.qpos)) or any(w.number for w in d.warning):
                raise RuntimeError(f"Physics warning at {t} s")
            normal = [0.0] * len(names)
            slip = [0.0] * len(names)
            local = [None] * len(names)
            floor_force = 0.0
            hard_force = 0.0
            geo_names = []
            for ic, c in enumerate(d.contact):
                if ballgeom not in (c.geom1, c.geom2):
                    if cfg.get("adjacent_contact"):
                        force = np.empty(6)
                        mj.mj_contactForce(m, d, ic, force)
                        if force[0] > adjacent_peak:
                            adjacent_peak = float(force[0])
                            adjacent_pair = " / ".join(
                                m.geom(g).name for g in (c.geom1, c.geom2)
                            )
                    continue
                other = c.geom2 if c.geom1 == ballgeom else c.geom1
                f = np.empty(6)
                mj.mj_contactForce(m, d, ic, f)
                if f[0] <= 0.001:
                    continue
                if other in pad_owner:
                    i = pad_owner[other]
                    normal[i] += float(f[0])
                    n = c.frame[:3]
                    rel = point_velocity(m, d, ball, c.pos) - point_velocity(
                        m, d, padbodies[i], c.pos
                    )
                    slip[i] = max(slip[i], np.linalg.norm(rel - n * np.dot(n, rel)))
                    if i != 1 or "cup_cells" not in cfg:
                        local[i] = d.geom_xmat[other].reshape(3, 3).T @ (
                            c.pos - d.geom_xpos[other]
                        )
                        local[i] /= np.linalg.norm(local[i])
                elif other == m.geom("catch_plane").id:
                    floor_force += f[0]
                else:
                    hard_force += f[0]
                    geo_names.append(m.geom(other).name)
            R = d.xmat[palm].reshape(3, 3)
            offset = R.T @ (d.xpos[ball] - d.xpos[palm])
            omega, velocity = angular_velocity(m, d, ball)
            palmomega, _ = angular_velocity(m, d, palm)
            palm_rel = velocity - point_velocity(m, d, palm, d.xpos[ball])
            if setup_samples is not None and t <= 0:
                setup_samples.append(
                    dict(
                        t_s=t,
                        **{
                            f"angle_{i}_{j}_rad": float(ref[j] + d.qpos[adr])
                            for i, (qa, _, ref, _, _) in enumerate(digits)
                            for j, adr in enumerate(qa)
                        },
                    )
                )
            if k == prep_ticks:
                start_ball = offset.copy()
                at_release = {
                    "pad_N": normal.copy(),
                    "fixture_offset_mm": float(1000 * np.linalg.norm(offset - initial)),
                    "tendon_N": tensions.copy(),
                }
            if t >= 0:
                if any(normal):
                    last_pad_time = t
                if not any(normal) and first_loss is None:
                    first_loss = t
                if floor_force and first_floor is None:
                    first_floor = t
                if min(normal) <= 0 and first_pad_break is None:
                    first_pad_break = t
                if hard_force and first_hard is None:
                    first_hard = ",".join(geo_names)
                    first_hard_time = t
                if t > 0 and first_floor is None:
                    free_spin_path += np.linalg.norm(omega - palmomega) * 0.002
                    for i in range(len(names)):
                        if local[i] is not None and previous_local[i] is not None:
                            rolling[i] += 0.0092 * math.acos(
                                np.clip(np.dot(local[i], previous_local[i]), -1, 1)
                            )
                            swipe[i] += slip[i] * 0.002
                previous_local = local
                trace.append(
                    dict(
                        t_s=t,
                        elbow_deg=math.degrees(d.qpos[arm]),
                        palm_x_mm=offset[0] * 1000,
                        palm_y_mm=offset[1] * 1000,
                        palm_z_mm=offset[2] * 1000,
                        world_down_m=d.xpos[ball][1],
                        world_x_m=d.xpos[ball][0],
                        world_z_m=d.xpos[ball][2],
                        travel_rel_mm=1000 * np.linalg.norm(offset - start_ball),
                        ball_rel_speed_m_s=np.linalg.norm(palm_rel),
                        spin_rel_rad_s=np.linalg.norm(omega - palmomega),
                        index_N=normal[0],
                        thumb_N=normal[1],
                        index_slip_m_s=slip[0],
                        thumb_slip_m_s=slip[1],
                        index_pull_N=tensions[0],
                        thumb_pull_N=tensions[1],
                        **{f"{names[i]}_N": normal[i] for i in range(2, len(names))},
                        **{
                            f"{names[i]}_pull_N": tensions[i]
                            for i in range(2, len(names))
                        },
                        hard_N=hard_force,
                        **(
                            {"ball_hardware": "|".join(sorted(set(geo_names)))}
                            if "ring" in names
                            else {}
                        ),
                        plane_N=floor_force,
                        **{
                            f"{name}_pad_{axis}_m": float(value)
                            for name, g in zip(names, padids)
                            for axis, value in zip("xyz", d.geom_xpos[g])
                        },
                        **{
                            f"angle_{i}_{j}_rad": float(ref[j] + d.qpos[adr])
                            for i, (qa, _, ref, _, _) in enumerate(digits)
                            for j, adr in enumerate(qa)
                        },
                        **{f"ball_quat_{s}": v for s, v in zip("wxyz", d.xquat[ball])},
                    )
                )
        if k < prep_ticks + round(end / dt):
            mj.mj_step(m, d)
    tail = [r for r in trace if r["t_s"] >= end - 0.25]
    stable = all(
        min(r[name + "_N"] for name in names) > 0.01
        and r["ball_rel_speed_m_s"] < 0.002
        and r["spin_rel_rad_s"] < 0.2
        and r["plane_N"] < 0.001
        for r in tail
    )
    sustained = next(
        (r for r in trace if last_pad_time is not None and r["t_s"] > last_pad_time),
        None,
    )
    caught = all(
        r["plane_N"] > 0.01
        and sum(r[name + "_N"] for name in names) + r["hard_N"] == 0
        and r["travel_rel_mm"] > 2 * cfg["radius_m"] * 1000
        for r in tail
    )
    if caught and first_floor is not None:
        status = "missed pickup" if last_pad_time is None else "dropped"
    else:
        status = "settled pinch" if stable else "unresolved"
    result = dict(
        approach_peak_N=approach_peak,
        adjacent_peak_N=adjacent_peak,
        adjacent_pair=adjacent_pair,
        status=status,
        release=at_release,
        first_single_pad_break_s=first_pad_break,
        first_both_pads_off_s=first_loss,
        final_pad_exit_s=sustained["t_s"] if sustained else None,
        exit_elbow_deg=sustained["elbow_deg"] if sustained else None,
        floor_s=first_floor,
        first_hard_contact=first_hard,
        first_hard_s=first_hard_time,
        pad_path_mm=[1000 * x for x in rolling],
        integrated_slip_mm=[1000 * x for x in swipe],
        ball_relative_spin_path_rad=free_spin_path,
        final_elbow_deg=trace[-1]["elbow_deg"],
        tail_max_speed_m_s=max(r["ball_rel_speed_m_s"] for r in tail),
    )
    return result, trace


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dt", type=float, default=0.00025)
    parser.add_argument("--friction", type=float, default=0.35)
    parser.add_argument("--lift-s", type=float, default=1.2)
    parser.add_argument("--response-s", type=float, default=0.003)
    parser.add_argument("--horizontal", action="store_true")
    parser.add_argument(
        "--transfer-s",
        type=float,
        default=0.0,
        help="duration of gradual fixture unloading before arm motion",
    )
    args = parser.parse_args()
    cfg = json.loads((ROOT / "data/free-ball-model.json").read_text())
    if (
        args.dt <= 0
        or args.friction < 0
        or args.lift_s <= 0
        or args.response_s <= 0
        or args.transfer_s < 0
    ):
        parser.error(
            "Check timestep, friction, lift time, contact response and transfer time."
        )
    result, rows = run(
        cfg,
        dt=args.dt,
        mu=args.friction,
        lift_s=args.lift_s,
        response=args.response_s,
        horizontal=args.horizontal,
        transfer_s=args.transfer_s,
    )
    out = (
        ROOT
        / "outputs"
        / "free-ball"
        / f"{'horizontal' if args.horizontal else 'lift'}-{args.lift_s}-{args.dt}-{args.friction}-{args.response_s}-{args.transfer_s}"
    )
    out.mkdir(parents=True, exist_ok=True)
    write_csv(out / "trajectory.csv", rows)
    (out / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
