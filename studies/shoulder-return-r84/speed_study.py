"""Vary travel time only. Both axes follow the same half-cosine clock."""
import csv
from concurrent.futures import ProcessPoolExecutor
from itertools import product

from solver import DEST, simulate

def solve(case):
    motion, mu, travel_s = case
    job = ('speed', motion, mu, .35, False, 2, True)
    row = simulate(job, travel_s=travel_s)
    return (motion, mu, travel_s, *row[8:14])

def main():
    cases = product(('full', 'reverse'), (.065, .25), (2, 3, 4, 6, 8))
    DEST.mkdir(parents=True, exist_ok=True)
    with ProcessPoolExecutor(max_workers=4) as workers:
        rows = list(workers.map(solve, cases))
    with (DEST/'speed.csv').open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(('motion','mu_liner','one_way_s','min_tension_N','peak_slack_mm',
                    'return_slack_mm','end_slack_mm','stop_margin_mm','slider_residual_mm'))
        w.writerows(rows)
if __name__=='__main__':
    main()
