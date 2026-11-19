"""Replay the target-contact acquisition with the matching private collision hulls."""
import argparse
import json
import math
from pathlib import Path
from free_ball import run
from tripod_coordination import ThreePadFeedback

DATA = Path(__file__).resolve().parent/"examples"/"inverse-release"/"setup.json"

def smooth(t, duration):
    u = min(1.,max(0.,t/duration))
    return math.pi/2*(10*u**3-15*u**4+6*u**5), math.pi/2/duration*30*u*u*(1-u)**2

def replay(hulls, profile, dt):
    setup=json.loads(DATA.read_text(encoding="utf-8"))
    hand=setup["hand"]
    if profile=="index-spring-swap":
        ks=hand["digits"][0]["spring_Nm_rad"]
        ks[1],ks[2]=ks[2],ks[1]
    metrics={}
    result, rows=run(hand, **setup["options"], dt=dt, private_hulls=hulls,
                     elbow_reference=smooth, trial_metrics=metrics,
                     hand_feedback=ThreePadFeedback() if profile=="balanced" else None)
    return dict(**metrics,**result,three_pad_samples=sum(
        min(r[n+"_N"] for n in ("index","thumb","middle"))>.01 for r in rows))

if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("hulls", type=Path, help="Prepared, registered collision hulls from the private assembly")
    parser.add_argument("--profile", choices=("preset","balanced","index-spring-swap"),default="preset")
    parser.add_argument("--dt", type=float, choices=(.000125,.00025),default=.000125)
    args=parser.parse_args()
    hulls=json.loads(args.hulls.read_text(encoding="utf-8"))
    if not hulls:
        parser.error("The matching collision geometry is required.")
    print(json.dumps(replay(hulls,args.profile,args.dt),indent=2))
