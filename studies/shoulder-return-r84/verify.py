"""Recompute the largest payout excursion and compare with archived Windows results."""
import csv
from pathlib import Path
from solver import simulate

def main():
    with (Path(__file__).parent/'evidence'/'convergence.csv').open() as f:
        reference = {float(row['mu_liner']): row for row in csv.DictReader(f)}
    for mu in (.065,.25):
        actual = simulate(('verify', 'full', mu, .35, False, 2, True))
        ref = reference[mu]
        # The unilateral cable and friction switch make exact cross-platform equality unsuitable.
        for pos, field, tolerance in ((8,'min_all_N',.03),(9,'slack_max_mm',.20),(11,'slack_end_mm',.02)):
            if abs(actual[pos]-float(ref[field])) > tolerance:
                raise ValueError(f'{field}: got {actual[pos]}, archived {ref[field]}')
        if mu == .065 and (actual[8] <= 0 or actual[9] > .001):
            raise ValueError('Target cable lost tension')
        print(f'mu={mu:.3f}  minimum tension {actual[8]:.3f} N  maximum slack {actual[9]:.2f} mm')
if __name__=='__main__':
    main()
