"""Build the C++17 joint controller, compare every tick and audit the closed-loop runs."""
import csv
import json
import os
import subprocess
from pathlib import Path

import elbow_control as model


def run():
    root = Path(__file__).resolve().parent
    out = root / "outputs" / "controls"
    out.mkdir(parents=True, exist_ok=True)
    binary = out / ("replay.exe" if os.name == "nt" else "replay")
    subprocess.run([os.environ.get("CXX","g++"), "-std=c++17", "-O2", "-Wall", "-Wextra",
                    str(root/"controls"/"replay.cpp"), "-o", str(binary)], check=True)
    fields = ["velocity_est_rad_s", "request_nm", "command_nm"]
    report = []
    for case in model.cases():
        rows = model.simulate(case)
        _, gsin, gcos = model.plant_terms(case.estimate_g)
        inputs = "".join(f"{r['target_rad']} {r['target_velocity_rad_s']} {r['encoder_rad']} "
                         f"{gsin} {gcos} {case.limit_nm}\n" for r in rows)
        output = subprocess.run([str(binary),case.controller], input=inputs, check=True,
                                capture_output=True, text=True)
        ticks = output.stdout.splitlines()
        if len(ticks) != len(rows):
            raise RuntimeError(f"Missing C++ samples in {case.name}")
        worst = dict.fromkeys(fields,0.0)
        for line, row in zip(ticks, rows):
            values = [float(x) for x in line.split()]
            if len(values) != len(fields):
                raise ValueError("Invalid controller output")
            for key, value in zip(fields, values):
                error = abs(value-row[key])
                if not error <= 1e-9:
                    raise AssertionError((case.name,row["time_s"],key,error))
                worst[key] = max(worst[key],error)
        report.append(dict(case=case.name,samples=len(rows),**worst))
    model.write_csv(out/"cpp-parity.csv",report)
    print(f"{len(report)} cases, {sum(r['samples'] for r in report)} matched ticks.")
    return report


if __name__=="__main__":
    run()
