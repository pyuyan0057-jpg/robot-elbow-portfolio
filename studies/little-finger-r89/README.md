# R8.9 · Little-finger opposition at the palm

[Watch the solved tabletop lift](grasp.mp4) · [Compare finger-root settings](root-sweep.csv) · [Open-palm SolidWorks view](palm.png) · [250 µs force and height trace](r89-250us.csv)

**小指现在能和拇指、食指、中指一起承力。** 在原分段手套模型中，Ø40 mm、33 g 的自由球从桌上被拾起。肘角由 45° 升到 99.98°，末态球底高出桌面 226.7 mm，小指对球的法向力为 0.75 N。无名指末态为 0 N，离桌初段也曾只有一指超过计数阈值。全程五指持续夹持的目标尚未通过。软套邻指接触对结果有显著影响，详见下方。

## What changed

The root cartridge keeps the old two-screw M3 mounting interface. It turns the little finger’s flexion plane 18° toward the palm and shifts its pivot 6 mm distally and 13 mm toward the palm surface. The preset is fixed during a grasp. The existing six hand drives are retained. The palm has a local relief; a curved tendon guide replaces the old straight entry. Its 1.36 mm bore takes the nominal 0.60 mm tendon. The final cartridge adds screw-head clearance and ends the guide before the proximal phalanx. The local archive contains the full revised assembly, palm, open-position tendon and the original R8.8 comparison. [Hardware permissions](../../docs/sources-and-rights.md)

## What carried the ball

| Run | Input and contact model | Outcome |
|---|---|---|
| R8.8 archived demonstration | Original root, its own tuned commands | Ball bottom finished 229.8 mm above the tabletop. All digit contacts briefly fell below the combined contact-loss threshold. No simultaneous index/little-finger load. |
| R8.8 paired control | Original root, R8.9 commands and the same adjacent-glove contact handling | The ball dropped. Maximum rise was 1.5 mm. The later free fall is outside the finite tabletop. |
| R8.9 | Revised root, same paired commands, 250 µs integration | Ball bottom finished 226.7 mm above the tabletop. No return to the table. The final 1.05 s hold had at most 0.80 mm peak-to-peak movement on any one palm-relative coordinate axis. |

At the last sample, normal forces on the ball were **thumb 1.15 N, index 0.53 N, middle 0.49 N, ring 0.00 N and little finger 0.75 N**. These magnitudes act in different directions. The ball weighs 0.324 N. End height alone does not show which fingers contributed. The contact count uses a 0.015 N per-digit threshold. Four or more fingers carried load for 3.24 s in total. All five carried load together for 0 s. For all three integration steps (500, 250 and 125 µs), the final height was 226.0–227.5 mm and the minimum airborne count was one. Glove/ball sliding friction was set to 0.30 and tabletop friction to 0.45. These are simulation assumptions.

## CAD and glove checks

The revised assembly reopened and rebuilt in SolidWorks with 518 resolved, local active component instances. In the open palm, the native solid-interference check found zero overlaps above 0.000001 mm³. In a separate solid-geometry check, the revised cartridge and little proximal phalanx had 0.15 mm minimum nominal clearance at each of 26 angles from 0° to 50° (2° steps). Separately, the six native phalanges of the ring and little fingers were placed at seven recorded solver poses. Across the nine rigid phalanx pairs the smallest measured distance was 7.05 mm. The tests do not cover manufacturing tolerances or tendon wear. [Root sweep](root-sweep.csv)

The contact solver used segmented glove collision surfaces. In the nominal simulation the ring and little proximal sleeves already overlap numerically by 3.39 mm at the starting pose. The model responds to that compression. Trimming the little sleeve on its ring-facing side and repeating the same commands caused the ball to fall. With the trimmed sleeve and retuned commands, the 250 µs run finished at 227.8 mm with 0.53 N on the little finger. Maximum adjacent-sleeve penetration in 109 replay samples fell to 1.84 mm. This is an alternative collision geometry, not a fitted CAD glove. [Retuned trace](trimmed-retuned.csv) · [Timestep and glove sensitivity](glove-sensitivity.json). Friction and sleeve compliance have not been measured; the observed four-finger hold depends on this contact model. The CAD check above covers the rigid parts. The moving model does not solve tendon rubbing in the new conduit. The replay uses the solver coordinates and is coloured by digit (green little finger, blue thumb).

## Data

- [Solver-step metrics for all five runs](results.json) and [nominal control and geometry inputs](inputs.json)
- Selected height and force traces at 5 ms intervals: [R8.8 original](r88-original.csv), [R8.8 with R8.9 commands](r88-paired.csv), [R8.9 500 µs](r89-500us.csv), [R8.9 250 µs](r89-250us.csv), [R8.9 125 µs](r89-125us.csv)
- [Ring/little contact samples](neighbors.csv) and [rigid phalanx distances at seven solved poses](cad-digit-gaps.csv). The latter uses exported native solids; the former uses the solver collision proxies.
