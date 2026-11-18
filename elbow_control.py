"""Reproduce the R2 joint tracking, load mismatch and disturbance experiments."""
import csv
import math
from dataclasses import dataclass, asdict
from pathlib import Path

from calculate import PARAMETERS, TRAVEL, LoadCase, solve

SAMPLE_S = 0.001
LIFT_S = PARAMETERS["lift_time_s"]
END_S = 5.0
PULSE_START, PULSE_END = 2.5, 2.7
KP, KD, KI = 18.0, 1.8, 20.0
LAG_S, DAMPING = 0.012, 0.045
TORQUE_LIMIT = 5.0
BAND_DEG = 0.5


@dataclass(frozen=True)
class Experiment:
    name: str
    controller: str
    actual_g: float
    estimate_g: float
    pulse_nm: float = 0
    limit_nm: float = TORQUE_LIMIT


def cases():
    result = []
    for payload in (0, 150, 300):
        for mode in ("PD", "PDG"):
            result.append(Experiment(f"{mode}-{payload}", mode, payload, payload))
    result.extend([
        Experiment("mismatch-PDG", "PDG", 300, 150),
        Experiment("mismatch-PIDG", "PIDG", 300, 150),
        Experiment("pulse-PDG", "PDG", 150, 150, -1.0),
        Experiment("pulse-PIDG", "PIDG", 150, 150, -1.0),
        Experiment("limited-PIDG", "PIDG", 300, 150, 0, 2.5)])
    return result


def reference(t, lift_s=LIFT_S):
    ramp = lift_s / 4
    vmax = TRAVEL / (lift_s - ramp)
    acc = vmax / ramp
    if t < ramp:
        return 0.5 * acc * t*t, acc*t
    if t < lift_s-ramp:
        return vmax*(t-ramp/2), vmax
    if t < lift_s:
        left = lift_s-t
        return TRAVEL-0.5*acc*left**2, acc*left
    return TRAVEL, 0.0


def plant_terms(payload_g):
    m = PARAMETERS["moving_mass_kg"]
    p = payload_g/1000
    u,v,_ = PARAMETERS["com_m"]
    x,y,_ = [a/1000 for a in PARAMETERS["payload_position_mm"]]
    inertia = PARAMETERS["axis_inertia_kg_m2"] + p*(x*x+y*y)
    inertia += PARAMETERS["motor_rotor_inertia_kg_m2"]*PARAMETERS["gear_ratio"]**2
    return inertia, PARAMETERS["gravity_m_s2"]*(m*u+p*x), PARAMETERS["gravity_m_s2"]*(m*v+p*y)


def gravity(q, terms):
    return terms[1]*math.sin(q) + terms[2]*math.cos(q)


def advance(q, w, torque, cmd, disturbance, terms, dt):
    # Exact first-order actuator response at RK4 stages; ZOH command and disturbance.
    tm = cmd + (torque-cmd)*math.exp(-dt/(2*LAG_S))
    te = cmd + (torque-cmd)*math.exp(-dt/LAG_S)
    def acc(pos, vel, effort):
        return (effort+disturbance-gravity(pos,terms)-DAMPING*vel)/terms[0]
    a1 = acc(q,w,torque)
    v2 = w+dt*a1/2
    a2 = acc(q+dt*w/2,v2,tm)
    v3 = w+dt*a2/2
    a3 = acc(q+dt*v2/2,v3,tm)
    v4 = w+dt*a3
    a4 = acc(q+dt*v3,v4,te)
    return q+dt*(w+2*v2+2*v3+v4)/6, w+dt*(a1+2*a2+2*a3+a4)/6, te


def command(mode, error, velocity_error, g_estimate, integral, limit):
    feedforward = 0.0 if mode == "PD" else g_estimate
    candidate = integral+error*SAMPLE_S if mode == "PIDG" else 0.0
    effort = KP*error + KD*velocity_error + feedforward + KI*candidate
    if abs(effort)>limit and effort*error>0:
        candidate = integral
        effort = KP*error + KD*velocity_error + feedforward + KI*candidate
    return candidate, effort, max(-limit,min(limit,effort))


def simulate(case, subdivisions=2, *, actual_terms=None, estimate_terms=None, lift_s=LIFT_S):
    if subdivisions < 1 or lift_s <= 0 or lift_s >= END_S:
        raise ValueError("Use a positive substep count and a lift shorter than the recording.")
    actual = plant_terms(case.actual_g) if actual_terms is None else actual_terms
    estimate = plant_terms(case.estimate_g) if estimate_terms is None else estimate_terms
    q = w = torque = integral = 0.0
    previous_reading = velocity_estimate = 0.0
    alpha = 1-math.exp(-SAMPLE_S/0.006)
    rows = []
    for k in range(round(END_S/SAMPLE_S)+1):
        t = k*SAMPLE_S
        target, speed = reference(t, lift_s)
        reading = q  # ideal encoder angle; velocity estimated only from past samples
        velocity_estimate += alpha*((reading-previous_reading)/SAMPLE_S-velocity_estimate)
        previous_reading = reading
        error = target-reading
        ghat = gravity(reading,estimate)
        integral,request,cmd = command(case.controller,error,speed-velocity_estimate,ghat,integral,case.limit_nm)
        disturbance = case.pulse_nm if PULSE_START <= t < PULSE_END else 0.0
        accel=(torque+disturbance-gravity(q,actual)-DAMPING*w)/actual[0]
        rows.append(dict(time_s=t,target_rad=target,encoder_rad=reading,velocity_est_rad_s=velocity_estimate,
                         plant_speed_rad_s=w,plant_acceleration_rad_s2=accel,
                         target_velocity_rad_s=speed,gravity_est_nm=ghat,request_nm=request,command_nm=cmd,
                         applied_nm=torque,external_nm=disturbance,error_deg=math.degrees(error)))
        if k < round(END_S/SAMPLE_S):
            for _ in range(subdivisions):
                q,w,torque = advance(q,w,torque,cmd,disturbance,actual,SAMPLE_S/subdivisions)
    return rows


def metrics(rows,case):
    errors = [r["error_deg"] for r in rows]
    lift = [r["error_deg"] for r in rows if r["time_s"]<=LIFT_S]
    tail = errors[-round(0.25/SAMPLE_S):]
    result = dict(lift_rms_deg=math.sqrt(sum(e*e for e in lift)/len(lift)),
                  max_abs_deg=max(map(abs,errors)),hold_mean_abs_deg=sum(map(abs,tail))/len(tail),
                  peak_applied_nm=max(abs(r["applied_nm"]) for r in rows),
                  saturated_fraction=sum(abs(r["request_nm"])>case.limit_nm for r in rows[:-1])/(len(rows)-1))
    if case.pulse_nm:
        after = [r for r in rows if r["time_s"]>=PULSE_END]
        last_bad = max((i for i,r in enumerate(after) if abs(r["error_deg"])>BAND_DEG),default=-1)
        result["pulse_max_abs_deg"] = max(abs(r["error_deg"]) for r in rows if r["time_s"]>=PULSE_START)
        recovered = after[last_bad+1]["time_s"] if last_bad+1 < len(after) else END_S
        result["recovery_s"] = recovered-PULSE_END if END_S-recovered >= 0.2 else None
    return result


def write_csv(path,rows):
    with path.open("w",encoding="utf-8",newline="") as f:
        writer=csv.DictWriter(f,fieldnames=list(dict.fromkeys(key for row in rows for key in row)))
        writer.writeheader()
        writer.writerows(rows)


def main():
    from matplotlib import use
    use("Agg")
    from matplotlib import pyplot as plt
    out=Path(__file__).resolve().parent/"outputs"/"controls"
    out.mkdir(exist_ok=True,parents=True)
    summaries=[]
    fig,axes=plt.subplots(2,2,figsize=(11,6.8),layout="constrained")
    selected={
        "PD-150":(0,"PD"), "PDG-150":(0,"PD + gravity"),
        "mismatch-PDG":(1,"PD + gravity"),"mismatch-PIDG":(1,"PID + gravity"),
        "pulse-PDG":(2,"PD + gravity"),"pulse-PIDG":(2,"PID + gravity"),
        "limited-PIDG":(3,"Torque-limited")}
    for case in cases():
        rows=simulate(case)
        result=metrics(rows,case)
        refined=metrics(simulate(case,4),case)
        change=abs(refined["max_abs_deg"]-result["max_abs_deg"])
        assert change<0.001, (case.name,change)
        summary=dict(**asdict(case),**result,refine_max_error_change_deg=change)
        summaries.append(summary)
        write_csv(out/(case.name+".csv"),rows)
        if case.name in selected:
            idx,label=selected[case.name]
            ax=axes.flat[idx]
            ts=[r["time_s"] for r in rows]
            ax.plot(ts,[r["error_deg"] for r in rows],label=label)
    for ax,title in zip(axes.flat,["150 g | nominal load","300 g | controller assumes 150 g","150 g | external torque pulse","300 g | 2.5 N m torque ceiling"]):
        ax.set(title=title,xlabel="Time (s)",ylabel="Target minus encoder (deg)")
        ax.axhline(0,lw=.6,color="grey")
        ax.grid(alpha=.2)
        ax.legend(frameon=False)
    axes.flat[2].axvspan(PULSE_START,PULSE_END,color="gray",alpha=.18)
    write_csv(out/"summary.csv",summaries)
    fig.savefig(out/"tracking.png",dpi=150)
    plt.close(fig)
    for p in (0,150,300):
        case=LoadCase(p,tuple(PARAMETERS["payload_position_mm"]),LIFT_S,1.0)
        assert abs(solve(case)[1]["hold_nm"]-gravity(TRAVEL,plant_terms(p)))<1e-10
    print(f"{len(summaries)} experiments, equilibrium and integration checks passed: {out}")


if __name__=="__main__":
    main()
