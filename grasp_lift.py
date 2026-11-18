"""Necessary contact-line moment test for a fixed two-point pinch on a moving elbow."""
import json
import math
from calculate import PARAMETERS, ROOT
from elbow_control import Experiment, simulate, write_csv


def run():
    reg = json.loads((ROOT/"data"/"registered-pinch.json").read_text())
    u,v,_ = reg["ball_uvw_m"]
    mass = reg["moving_mass_empty_kg"] + reg["held_ball_kg"]
    g = PARAMETERS["gravity_m_s2"]
    rotor = PARAMETERS["motor_rotor_inertia_kg_m2"]*PARAMETERS["gear_ratio"]**2
    terms = (reg["loaded_axis_inertia"]+rotor, mass*g*reg["loaded_com_m"][0],
             mass*g*reg["loaded_com_m"][1])
    _,lever_y,lever_z = reg["contact_line_moment_arm_hand_mm"]
    assert abs(terms[1]-reg["horizontal_hold_nm"])<1e-10
    out=ROOT/"outputs"/"grasp-lift"
    out.mkdir(parents=True,exist_ok=True)
    summaries=[]
    for seconds, pulse in [(2.4,0),(1.2,0),(.6,0),(1.2,-1)]:
        case=Experiment("registered","PDG",150,150,pulse)
        rows=simulate(case,actual_terms=terms,estimate_terms=terms,lift_s=seconds)
        trace=[]
        for r in rows:
            q=r["encoder_rad"]
            w=r["plant_speed_rad_s"]
            a=r["plant_acceleration_rad_s2"]
            # Moving-frame components of sphere-centre acceleration for rigid transport.
            acceleration_u=-a*v-w*w*u
            acceleration_v=a*u-w*w*v
            fu=reg["held_ball_kg"]*(g*math.cos(q)-acceleration_u)
            fv=reg["held_ball_kg"]*(-g*math.sin(q)-acceleration_v)
            residual=lever_y*fu+lever_z*fv
            trace.append(dict(time_s=r["time_s"],elbow_deg=math.degrees(q),required_axis_moment_Nmm=residual))
        name=f"{seconds:g}s"+("-pulse" if pulse else "")
        write_csv(out/(name+".csv"),trace)
        peak=max(trace,key=lambda row:abs(row["required_axis_moment_Nmm"]))
        summaries.append(dict(lift_s=seconds,pulse_Nm=pulse,peak_abs_Nmm=abs(peak["required_axis_moment_Nmm"]),
                              peak_time_s=peak["time_s"],start_Nmm=trace[0]["required_axis_moment_Nmm"]))
    write_csv(out/"summary.csv",summaries)
    return summaries


if __name__=="__main__":
    print(run())
