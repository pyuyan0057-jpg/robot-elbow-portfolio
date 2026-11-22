# Seven-axis biomimetic arm | tabletop grasp and trajectory control

[中文首页](README.zh-CN.md) · [Reproduce the cylinder experiment](studies/seven-axis-control-c4/README.md) · [Model, sources and contributions](docs/sources-and-rights.md)

A seven-axis arm and a six-drive tendon hand pick up a free object, lift it to 100° elbow flexion and set it down on the table. The cylinder starts upright, with the thumb above the little finger. The replay is from the ROS 2 Jazzy / MuJoCo simulation.

[![Full cylinder pick, lift, hold and release](studies/seven-axis-control-c4/media/cylinder.gif)](studies/seven-axis-control-c4/media/cylinder.mp4)

**Full videos** · [Cylinder pick and place (MP4)](studies/seven-axis-control-c4/media/cylinder.mp4) · [Sphere pick and place (MP4)](studies/seven-axis-control-c4/media/sphere.mp4)

### Can the hand keep a sphere through a sideways push?

A 40 mm, 33 g sphere was pushed from 26 directions during lifting, holding and lowering in MuJoCo. With a hand–sphere sliding friction coefficient of 0.45, the hand retained the sphere until first table contact and landed within 2 mm of the target in **78/78 trials at 0.10 N** and **61/78 at 0.60 N**. At 0.30 friction the same controller passed **53/78** at 0.60 N. The friction values are simulation assumptions. [Grasp changes and failure analysis](studies/ball-side-c9/README.md) · [214 paired trials](studies/ball-side-c9/evidence/matched_045.csv)

### What changed in the controller?

C4 combines inertial feedforward, joint-specific stiffness tuning and a small correction to the placement command. Both versions run the same 9.47 s cylinder task and are scored against the same physical placement target.

| Cylinder simulation | C2 baseline | C4 tuned |
|---|---:|---:|
| Peak palm tracking error | 5.04 mm | 2.90 mm |
| RMS palm tracking error | 1.65 mm | 0.89 mm |
| Final cylinder placement error | 3.47 mm | 0.36 mm |
| Cylinder excursion during hold | 0.49 mm | 0.62 mm |

![Palm position error over the full cylinder cycle](studies/seven-axis-control-c4/media/cylinder-error.png)

**Run it from the repository root.** Windows: `py -3.11 run_cylinder.py`. Linux: `python3.11 run_cylinder.py`. Requires Python 3.11 and a C++17 g++ compiler. The command builds the solver, runs both grasps and writes the comparison plot, CSV and acceptance results to `outputs/cylinder-control/`. [Experiment and failure cases](studies/seven-axis-control-c4/README.md) · [ROS action adapter](studies/seven-axis-control-c4/trajectory_action.py)

## How the mechanical design evolved

**C2 · ROS 2 pick and place.** A seven-axis arm with a six-drive hand lifts and sets down two free tabletop objects. The sphere and upright cylinder were held 229 and 298 mm above the desk. [Videos, controller comparisons and runnable physics](studies/seven-axis-pick-place-c2/README.md)

**R8.9 · The little finger now carries load.** An 18° root cartridge brings it into the tabletop ball grasp. At 100° the simulated 33 g ball was held by the thumb, index, middle and little fingers. [Motion replay, CAD part and force traces](studies/little-finger-r89/README.md)

**R8.8 · Right-hand forearm rotation.** The thumb now passes upward through the handshake position as the palm turns up. All 38 SolidWorks samples on the forward and return paths were free of detected solid interference. [What changed and how it was checked](studies/right-forearm-r88/README.md)

[![R8.8 palm down, thumb up and palm up](studies/right-forearm-r88/solidworks-poses.png)](studies/right-forearm-r88/README.md)

**R8.4 shoulder cable return** · Five paths stayed taut at the target drag. Slowing the high-friction case to 8 seconds each way still left 35 mm of free cable. [SolidWorks images, return traces and reproducible model](studies/shoulder-return-r84/README.md)

**R7 shoulder harness** · The arm keeps its full shoulder range. See the revised routing, the 25 mm displacement cases and the measured gaps at the previous collision points. [R7 shoulder study](docs/shoulder-harness-r7.md).

**R6 shoulder harness** · Power and CAN clear the nominal route in 70 sampled arm poses. A 25 mm downward displacement produces contact at two shoulder limits. [Inspect the clamp positions, cable lengths and native clearance checks](docs/shoulder-harness-r6.md).

**R5 shoulder travel** · Inboard ±90°, outboard 0–90°. See the native CAD end poses, interference samples and the open cable-length question. [Shoulder travel review](docs/shoulder-r5.md)

**R4 · Integrated arm.** The inboard shoulder turns about the body flange normal. A second, perpendicular axis sits outboard. Watch the seven joints move separately and together, with the fingers open. [SolidWorks motion (MP4)](assets/full-arm-r4/motion.mp4) · [Design and CAD checks](docs/full-arm-r4.md)

[![R4 SolidWorks assembly](assets/full-arm-r4/arm.jpg)](docs/full-arm-r4.md)

[R3 upper-arm-mounted, fixed-center elbow](docs/three-axis-elbow.md) · The robot upper arm carries the yaw unit above the elbow. See the CAD poses, interference samples and horizontal holding loads.

[中文说明](README.zh-CN.md) · [Hardware permissions](docs/sources-and-rights.md) · [Mechanics and FEA](docs/mechanics.md)

[Integral thumb recess and hanging-release results](docs/fixed-thumb.md) · A fixed concave pad prolonged initial two-finger contact with a 33 g ball. Both 33 g and 150 g spheres fell during the hanging test, before the lift.

[Pickup-position and thumb-angle study](docs/pickup-pose.md) · With the same open-hand closure, moving the pickup and changing thumb opposition delayed final pad separation from 0.070 to 0.168 s in a selected case. The thumb first lost contact at 0.022 s. The ball fell before the horizontal hold.

[Watch the three-cycle SolidWorks arm and hand demonstration (MP4)](assets/arm-hand-motion.mp4) · [Run the elbow lab in Colab](https://colab.research.google.com/github/pyuyan0057-jpg/robot-elbow-portfolio/blob/main/notebooks/elbow_lab.ipynb)

**Free-ball lift result.** When the pickup fixture releases the 150 g equivalent sphere with the arm hanging, the ball rolls and slips. It leaves both pads after **0.108 s at 1.22° of elbow lift** in the CAD-collision simulation. The publicly runnable two-pad model gives 0.104 s. [Watch the solver replay](assets/free-ball-lift.mp4) · [Method and 18 sensitivity cases](docs/free-ball.md) · [Nominal trajectory](examples/free-ball-cad-trajectory.csv)

**Can the joint track a loaded lift, and can the hand retain a ball throughout it?** For a controller that underestimates the fingertip load by 150 g, integral action reduces final hold error from 1.85° to 0.011° in simulation. On the hand, the specified horizontal pinch admits local equilibrium; tilting the elbow introduces a moment that the same two fixed point contacts cannot balance. The whole-arm movie shows prescribed CAD motion. [Controllers, grasp checks and structural trade-offs](docs/controls-and-grasp.md)

A single-axis elbow raises an existing hand and six-drive forearm through 90°. With 150 g at the middle fingertip, it needs **2.390 N·m to hold horizontal**. Widening the fixed motor ring reduced the support's contribution to fingertip sag from **0.2669 to 0.0576 mm** under the archived static load.

<a href="assets/cad-lift.gif"><img src="assets/cad-lift.gif" width="30%" alt="R2 CAD assembly lifting the forearm from vertical to horizontal"></a> <a href="assets/bracket-comparison.png"><img src="assets/bracket-comparison.png" width="67%" alt="Same-scale R1 and R2 yokes, with added ring and spine material highlighted"></a>

*Left: actual SolidWorks assembly motion. Right: matched bracket views and mount sections. The ring OD grew from 94 to 106 mm and the local spine tie widened. The extra 26.93 g stays on the fixed upper arm. Click either figure to enlarge.*

## Standalone elbow torque example

Use Python 3.11–3.14 with pip in the repository root. The calculator reads local JSON and produces its plot headlessly; the command line requires no CAD software. Tested interpreter and validation commands are listed in [verification](docs/verification.md).

```sh
python -m venv .venv
# Windows PowerShell
.\.venv\Scripts\Activate.ps1
# macOS / Linux: source .venv/bin/activate
python -m pip install -r requirements.txt
python calculate.py --payload-g 150
python calculate.py --payload-g 300
```

Each run writes **torque.png**, **trajectory.csv**, **inputs.json** and **summary.json** under `outputs/payload-<mass>g-<input-hash>/`. Changing payload location, time or design factor creates a distinct case. Identical inputs regenerate that case. In PowerShell with activation restricted, substitute `.\.venv\Scripts\python.exe` for `python` after creating the environment.

```sh
python calculate.py --payload-g 300 --payload-position-mm 350 0 -11 --lift-time-s 1.5 --design-factor 1.8
python -m unittest discover -s tests -v
python examples.py
```

## What changes with the payload?

The unloaded moving group is **1.60106 kg**, with its centre 115.18 mm along the horizontal reference axis. The default point load acts at U=395.214 mm. Each lift takes 1.20 s with 0.30 s acceleration, 0.60 s cruise and 0.30 s deceleration.

| Added mass | Horizontal hold (N·m) | Lift peak (N·m) | Design hold, ×1.5 (N·m) | Max speed (rpm) |
|---:|---:|---:|---:|---:|
| 0 g | 1.80847 | 1.74634 | 2.71271 | 16.67 |
| 150 g | 2.38983 | 2.30773 | 3.58475 | 16.67 |
| 300 g | 2.97119 | 2.86912 | 4.45679 | 16.67 |

The lift peak occurs at 75°, on the cruise side of the deceleration switch. At horizontal rest gravity reaches its maximum and the inertia term vanishes. Independently, the extra 150 g adds **0.150 × 9.80665 × 0.3952143649 = 0.58135934 N·m** to holding torque; the regression compares the full-precision difference to this calculation within **10⁻⁹ N·m**. [Machine-readable cases](examples/cases.csv)

![Output torque for the 0, 150 and 300 g lifts](examples/torque-overlay.png)

## Design evidence

- **Drive and load path.** An AK70-10 mounted on the upper arm transmits torque through a sliding coupling. Two external bearings support the forearm spindle. The 150 g design hold is 3.585 N·m; the smaller AK60-6 has a 3 N·m rated output. Zero-speed thermal endurance remains unverified. [Force diagram](assets/free-body.png) · [Model and assumptions](docs/mechanics.md)
- **Historical FEA.** With identical loads on both revisions, fixed-yoke mass changes from 293.80 to 320.73 g and its mapped fingertip sag falls by **78.4%**. Two aluminium submodels together contribute about **0.179 mm** in R2. Gear, bearing, connection and forearm flexibility are additional unknowns. A custom CLI run does not rerun FEA. [Nine mesh results](data/mesh-convergence.csv)
- **Project scope and attribution.** This is a CAD and simulation project. The applicant set the goals and acceptance criteria; modelling, calculations, solver runs and preparation of the repository used tool assistance. The public materials include design comparisons, load cases and numerical summaries. The original hardware design files are available for review by arrangement. Manufacturer material is linked at source. [Sources, contributions and permissions](docs/sources-and-rights.md)
