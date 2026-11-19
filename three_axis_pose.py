"""Forward kinematics of the archived R3 open hand. Coordinates and output are mm."""
import argparse
import json
import math

TIP = (511.21, -0.41, -10.96)


def elbow_and_tip(yaw_deg, lift_deg, roll_deg, centered=True):
    """Angles in the user-facing convention; lift=0 hangs, positive roll turns palm up."""
    yaw, pitch, roll = map(math.radians, (yaw_deg, lift_deg - 90, -roll_deg))
    pivot_x = 0 if centered else -145
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    x, y, z = TIP
    y, z = cr*y-sr*z, sr*y+cr*z
    x, y = cp*x-sp*y, sp*x+cp*y
    elbow = (pivot_x*(1-cy), 0, pivot_x*sy)
    tip = (elbow[0]+cy*x+sy*z, y, elbow[2]-sy*x+cy*z)
    return elbow, tip


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--yaw", type=float, default=0, help="left/right heading, degrees")
    parser.add_argument("--lift", type=float, default=90, help="0 hanging, 90 horizontal")
    parser.add_argument("--roll", type=float, default=0, help="0 palm down, 180 palm up with arm horizontal")
    parser.add_argument("--offset", action="store_true", help="use the low-mounted yaw turntable")
    args = parser.parse_args()
    elbow,tip = elbow_and_tip(args.yaw,args.lift,args.roll,not args.offset)
    print(json.dumps({"elbow_center_mm":elbow,"middle_fingertip_mm":tip}))
