from motion import plan, S, D
from trial import execute
import numpy as np, sys, json

task, action = sys.argv[1:]
recipe = json.loads((D / ("recipe_" + task + ".json")).read_text())


def remote(k, t, ref, measured, release, q):
    print("@" + json.dumps([k, t, measured, release, *ref[14:], *q]), flush=True)
    values = json.loads(sys.stdin.readline())
    assert values[0] == k
    return np.array(values[1:21]), bool(values[21])


if action == "plan":
    pl = plan(task, recipe, 0.002)
    np.save(D / (task + "_target.npy"), pl["tape"][:, :14])
    np.save(D / (task + "_spools.npy"), pl["tape"][:, 14:])
    (D / (task + "_timeline.json")).write_text(json.dumps(pl["times"]))
else:
    execute(task, recipe, "final_" + task, remote)
