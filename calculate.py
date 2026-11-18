"""Gravity, inertia and drive-side torque for the R2 elbow lift."""
import argparse
import csv
import hashlib
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parent
PARAMETERS = json.loads((ROOT / "data/r2.json").read_text(encoding="utf-8"))
TRAVEL = math.pi / 2


@dataclass(frozen=True)
class LoadCase:
    payload_g: float
    position_mm: tuple
    lift_time_s: float
    design_factor: float

    def validate(self):
        values = [self.payload_g, *self.position_mm, self.lift_time_s, self.design_factor]
        if len(self.position_mm) != 3 or not all(math.isfinite(x) for x in values):
            raise ValueError("Supply finite numbers and three payload coordinates (U V W).")
        if self.payload_g < 0 or self.lift_time_s <= 0 or self.design_factor < 1:
            raise ValueError("Payload must be nonnegative, lift time positive and design factor at least 1.")


def solve(case):
    case.validate()
    p = PARAMETERS
    mass = p["moving_mass_kg"]
    added = case.payload_g / 1000
    pos = [v / 1000 for v in case.position_mm]
    total = mass + added
    u, v, w = [mass * c + added * x for c, x in zip(p["com_m"], pos)]
    inertia = p["axis_inertia_kg_m2"] + added * (pos[0] ** 2 + pos[1] ** 2)
    duration = case.lift_time_s
    ramp = duration / 4
    speed = TRAVEL / (duration - ramp)
    accel = speed / ramp
    g = p["gravity_m_s2"]
    rows = []

    def record(phase, t, angle, velocity, acceleration):
        gravity = g * (u * math.sin(angle) + v * math.cos(angle))
        inertial = inertia * acceleration
        torque = gravity + inertial
        motor = torque / (p["gear_ratio"] * p["equivalent_efficiency"])
        motor += p["motor_rotor_inertia_kg_m2"] * p["gear_ratio"] * acceleration
        rows.append(dict(phase=phase, time_s=t, angle_deg=math.degrees(angle),
                         velocity_rad_s=velocity, acceleration_rad_s2=acceleration,
                         gravity_nm=gravity, inertia_nm=inertial, output_nm=torque, motor_nm=motor))

    def angle_at(t, phase):
        if phase == "accelerate":
            return 0.5 * accel * t*t, accel*t
        if phase == "cruise":
            return 0.5 * accel*ramp*ramp + speed*(t-ramp), speed
        remaining = duration - t
        return TRAVEL - 0.5*accel*remaining**2, accel*remaining

    phases = [("accelerate", 0, ramp, accel), ("cruise", ramp, 3*ramp, 0),
              ("decelerate", 3*ramp, duration, -accel)]
    record("initial_rest", 0, 0, 0, 0)
    for phase, start, end, acceleration in phases:
        # Include each one-sided switch value and the exact internal gravity extremum.
        times = {start + (end-start)*i/400 for i in range(1,400)} | {start,end}
        angle = math.atan2(u,v) % math.pi
        if angle_at(start,phase)[0] < angle < angle_at(end,phase)[0]:
            if phase == "accelerate":
                exact = math.sqrt(2*angle/accel)
            elif phase == "cruise":
                exact = ramp + (angle - 0.5*accel*ramp**2)/speed
            else:
                exact = duration - math.sqrt(2*(TRAVEL-angle)/accel)
            times.add(exact)
        for t in sorted(times):
            theta, velocity = angle_at(t,phase)
            record(phase,t,theta,velocity,acceleration)
    record("horizontal_hold",duration,TRAVEL,0,0)
    lifting = rows[1:-1]
    peak = max(lifting,key=lambda r:abs(r["output_nm"]))
    hold = g*u
    summary = dict(total_moving_kg=total, combined_com_mm=[1000*j/total for j in (u,v,w)],
                   axis_inertia_kg_m2=inertia, hold_nm=hold, lift_peak_abs_nm=abs(peak["output_nm"]),
                   signed_lift_peak_nm=peak["output_nm"], peak_time_s=peak["time_s"],
                   peak_angle_deg=peak["angle_deg"], peak_phase=peak["phase"],
                   output_max_rpm=speed*60/(2*math.pi), motor_max_rpm=speed*p["gear_ratio"]*60/(2*math.pi),
                   design_hold_nm=abs(hold)*case.design_factor,
                   design_lift_peak_nm=abs(peak["output_nm"])*case.design_factor,
                   motor_hold_nm=rows[-1]["motor_nm"],
                   motor_lift_peak_abs_nm=max(abs(r["motor_nm"]) for r in lifting),
                   bearing_minus_n=g*(total/2-w/p["bearing_spacing_m"]),
                   bearing_plus_n=g*(total/2+w/p["bearing_spacing_m"]))
    summary_numbers = [value for value in summary.values() if isinstance(value, (int, float))]
    summary_numbers.extend(summary["combined_com_mm"])
    if not all(map(math.isfinite, summary_numbers)) or not all(
        math.isfinite(value) for row in rows for value in list(row.values())[1:]
    ):
        raise ValueError("Input magnitude exceeds the numerical range.")
    return rows,summary


def save_plot(rows,case,path):
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    fig, ax = plt.subplots(figsize=(8.2,4.5),layout="constrained")
    for key,label,color in [("output_nm","Output torque","#124765"),
                            ("gravity_nm","Gravity","#db782b"),("inertia_nm","Inertia","#21877a")]:
        ax.plot([r["time_s"] for r in rows],[r[key] for r in rows],label=label,color=color,
                lw=2.2 if key=="output_nm" else 1.5,ls="-" if key=="output_nm" else "--")
    ax.scatter(rows[-1]["time_s"],rows[-1]["output_nm"],color="#124765",zorder=4,label="Horizontal hold")
    ax.set(xlabel="Time (s)",ylabel="Joint output torque (N m)",
           title=f"Elbow lift  |  {case.payload_g:g} g at U={case.position_mm[0]:g} mm")
    ax.grid(alpha=.19);ax.legend(frameon=False,loc="best",ncol=2)
    fig.savefig(path,dpi=150);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--payload-g",type=float,default=150)
    parser.add_argument("--payload-position-mm",nargs=3,type=float,metavar=("U","V","W"),
                        default=PARAMETERS["payload_position_mm"])
    parser.add_argument("--lift-time-s",type=float,default=PARAMETERS["lift_time_s"])
    parser.add_argument("--design-factor",type=float,default=PARAMETERS["design_factor"])
    parser.add_argument("--output-dir",type=Path,default=Path("outputs"))
    args=parser.parse_args()
    case=LoadCase(args.payload_g,tuple(args.payload_position_mm),args.lift_time_s,args.design_factor)
    try:
        rows,summary=solve(case)
    except (ValueError, OverflowError, ZeroDivisionError) as err:
        parser.error(str(err))
    snapshot=dict(model="R2",constants=PARAMETERS,case=vars(case),travel_deg=90,
                  phase_fractions=[.25,.5,.25])
    digest=hashlib.sha256(json.dumps(snapshot,sort_keys=True).encode()).hexdigest()[:12]
    out=args.output_dir / f"payload-{case.payload_g:g}g-{digest}"
    out.mkdir(parents=True,exist_ok=True)
    save_plot(rows,case,out/"torque.png")
    for name,content in [("inputs",snapshot),("summary",summary)]:
        (out/f"{name}.json").write_text(json.dumps(content,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    with (out/"trajectory.csv").open("w",newline="",encoding="utf-8") as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    print(f"Payload {case.payload_g:g} g | moving mass {summary['total_moving_kg']:.5f} kg")
    print(f"Horizontal hold {summary['hold_nm']:.5f} N m | design hold {summary['design_hold_nm']:.5f} N m")
    print(f"Lift peak (magnitude) {summary['lift_peak_abs_nm']:.5f} N m at {summary['peak_time_s']:.4f} s "
          f"({summary['peak_angle_deg']:.2f} deg) | maximum speed {summary['output_max_rpm']:.2f} rpm")
    print(f"Saved {out}")


if __name__=="__main__":
    main()
