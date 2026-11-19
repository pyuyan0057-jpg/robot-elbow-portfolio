"""Lower bound on the gap of the two middle phalanges along an angle-log replay."""
import csv
import math


def replay_bounds(angles, checks):
    """r*|dq| bounds rigid point travel. All lengths in mm, angles in radians."""
    times = [round(float(row["t_s"]), 9) for row in angles]
    travel = [0.0]
    for a, b in zip(angles, angles[1:]):
        step = 0.0
        for digit in (2, 3):
            for joint, radius in enumerate((100.0, 50.0)):
                key = f"angle_{digit}_{joint}_rad"
                step += radius * abs(float(b[key]) - float(a[key]))
        travel.append(travel[-1] + step)
    lookup = dict(zip(times, travel))
    measured = sorted(
        (round(float(row["release_relative_s"]), 9), float(row["mid2_ring2_mm"]))
        for row in checks
    )
    if measured[0][0] != times[0] or measured[-1][0] != times[-1]:
        raise ValueError("Native checks must include both ends of the replay")
    results = []
    for (start, d0), (end, d1) in zip(measured, measured[1:]):
        length = lookup[end] - lookup[start]
        if not math.isfinite(d0 + d1 + length) or abs(d0 - d1) > length + 0.0001:
            raise ValueError(f"Check frame registration and radius bounds at {start}")
        # Distance is bounded below from both ends. Their mean is valid throughout.
        results.append(dict(start_s=start, end_s=end, start_gap_mm=d0, end_gap_mm=d1,
                            travel_bound_mm=length, gap_lower_mm=(d0 + d1 - length) / 2))
    return results


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("angles")
    parser.add_argument("clearances")
    args = parser.parse_args()
    with open(args.angles, newline="") as a, open(args.clearances, newline="") as c:
        rows = replay_bounds(list(csv.DictReader(a)), list(csv.DictReader(c)))
    print(f"Lowest bound on reconstructed path: {min(r['gap_lower_mm'] for r in rows):.3f} mm")
