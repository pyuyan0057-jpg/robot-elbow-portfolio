# Reproducing the release

Start in a fresh clone. The public entry point uses Python and the packages in the root requirements file.

| Environment | Scope |
|---|---|
| Windows 11 x64, OS build 26200; CPython 3.14.5, Matplotlib 3.10.8 | CLI, example plots, CSVs, JSON, regression tests, local documentation links and image decoding |

```sh
python -m venv .venv
# Activate the environment as shown in the README
python -m pip install -r requirements.txt
python calculate.py --payload-g 0
python calculate.py --payload-g 150
python calculate.py --payload-g 300
python calculate.py --payload-g 300 --payload-position-mm 350 0 -11 --lift-time-s 1.5 --design-factor 1.8
python -m unittest discover -s tests -v
python examples.py
```

Expect four case folders with four output files each. Ten tests check archived cases, the independent 150→300 g torque increment, signed mass ledger, parameter response, switch values, invalid input, isolated CLI outputs, local documentation links, image files and stored case results. The additional 150 g at U=395.2143649 mm produces **0.5813593428 N·m** more holding torque (acceptance tolerance 1×10⁻⁹ N·m). Display values in the READMEs are rounded.


The CAD lift is a presentation of prescribed assembly poses. The mesh convergence values come from historical native studies. These release checks reproduce calculations, and check digital assets. They do not perform a physical experiment or a new finite-element solve.
