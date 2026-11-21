from review import D
from search import batch
import json

if __name__ == "__main__":
    tasks = []
    for o in ["ball", "cylinder"]:
        p = D / ("recipe_" + o + ".json")
        c = json.loads(p.read_text())
        for i, dx, dz in [
            ("nominal", 0, 0),
            ("x+1", 1, 0),
            ("x-1", -1, 0),
            ("z+1", 0, 1),
            ("z-1", 0, -1),
            ("x+3", 3, 0),
            ("x-3", -3, 0),
            ("z+3", 0, 3),
            ("z-3", 0, -3),
        ]:
            tasks.append([o + "_" + i, o, dict(c, shift_mm=[dx, 0, dz])])
        for dx, dz in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            tasks.append([f"{o}_unknown{dx}_{dz}", o, dict(c, unknown_mm=[dx, 0, dz])])
        for key, params in [
            ("fault", {"sensor_fault": True}),
            ("jam", {"jam": True}),
            ("fine", {"dt": 0.000125}),
            ("coarse", {"dt": 0.0005}),
            ("mu25", {"mu_override": 0.25}),
            ("mu35", {"mu_override": 0.35}),
        ]:
            tasks.append([o + "_" + key, o, dict(c, **params)])
    batch(tasks, "acceptance")
