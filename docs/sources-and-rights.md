# Sources, contributions and rights

| Item | Origin and public scope |
|---|---|
| Calculator and tests | Project-specific implementation of the gravitational moment, parallel-axis and rigid-body motion equations; numerical reference values come from the archived R2 mass ledger. |
| CAD | Original elbow support and forearm adaptor, reconstructed from the project's editable feature dimensions. Editable solids and their geometry-generation scripts remain in the private engineering archive. |
| Media | The GIF uses the project's R2 assembly viewport frames. Commercial actuators in the assembly are simplified project-built envelopes. The bracket image projects exterior triangles from the archived 2.8 mm study meshes with a common orthographic camera; the orange section overlay uses the nominal mount-face geometry. |
| FEA data | Summarized outputs of nine archived, solved SolidWorks studies: three mesh levels each for the R1/R2 yokes and the unchanged adaptor. Raw licensed software and proprietary simulation project files are outside this public subset. |
| Applicant and tools | The applicant defined the lift goal, payload comparison and review criteria, and directed the project's design and presentation. CAD API automation, calculation implementation, simulation execution, plots and documentation used AI/software assistance. No physical test campaign was performed. |

### Manufacturer and method references

- Housing dimensions, reduction, mass and operating ratings. [CubeMars AK70-10 KV100](https://www.cubemars.com/product/ak70-10-kv100-robotic-actuator.html) and [AK60-6 V3 KV80](https://www.cubemars.com/product/ak60-6-v3-0-kv80-robotic-actuator.html)
- External bearing envelope and ratings. [SKF 6002-2Z](https://www.skf.com/group/products/rolling-bearings/ball-bearings/deep-groove-ball-bearings/productid-6002-2Z)
- The six inherited forearm actuators each use the listed complete motor mass. [Pololu #4847 specifications](https://www.pololu.com/product/4847/specs)
- Project organization follows the practice of making robotics calculations runnable from a short example, with the model and example data available for inspection. The implementation here was written for this elbow. [Robotics Toolbox for Python, project repository](https://github.com/petercorke/robotics-toolbox-python)
- Vendor drawings, manuals, logos and vendor STEP downloads are referenced at their source. Product designations identify geometrical interfaces. Matplotlib is installed separately and retains its own licence.

### Software and hardware permissions

The [MIT licence](../LICENSE) covers the listed Python programs and tests, the C++ replay controller and the notebook code cells. Rights to the hardware design, media, numerical data and other project materials are reserved. Contact the repository owner for prior written permission to use the project design for commercial manufacture, or to request access to private CAD files for evaluation. Vendor rights remain with their owners.
