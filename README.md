
#  Modelling & Simulation of a Spot-Inspired Quadruped Robot

<img width="477" height="260" alt="image" src="https://github.com/user-attachments/assets/25187fa9-504e-4f87-a207-76ab11567d3d" />


A SolidWorks CAD model and motion study of a 12-DOF quadruped robot, inspired by Boston Dynamics' **Spot**. Includes an earlier 8-DOF 3D-printed working prototype that can stand and sit.

**Author:** Aditya Ganta, B.Tech Mechanical Engineering, NIT Tiruchirappalli
**Program:** ROBO AI (Robotics & AI training, in collaboration with PRAGYAN, NIT Trichy)

---

## Objective
Model and simulate a quadruped robot using SolidWorks (part modelling, assembly modelling, motion study) and build a working prototype.

## Tools & Tech
- **CAD / Simulation:** SolidWorks (Part, Assembly, Motion Study), Onshape (prototype)
- **Kinematics:** MATLAB (forward and inverse kinematics)
- **Prototype hardware:** Waveshare ST3215 servos, servo drivers, ESP32, PLA parts printed on a Creality K1C

---

## Phase 1: 8-DOF Prototype (5-Bar Legs)
- 2 DOF per leg using a closed-loop **5-bar mechanism** with two servos per leg (8 servos total).
- Lower inertia and lower torque requirements, with no gearboxes needed, so it is cheap and simple.
- Link lengths were tuned in MATLAB. Standing and sitting were achieved through inverse kinematics.
- **Limitation:** closed-loop kinematics and dynamics are complex, and 2 DOF per leg restricts walking.

##  Phase 2: 12-DOF SolidWorks Model
Each leg has 3 DOF. A **helical screw-and-nut mechanism** inside the hollow upper limb converts motor rotation into linear motion that drives the lower limb.

| Part | Material | Notes |
|---|---|---|
| Main body | ABS | Houses 4 motors, electronics and camera |
| Upper limb | Magnesium alloy | Hollow, for motors and the screw-nut drive, with good heat dissipation |
| Lower limb | Magnesium alloy | Foot grips for stairs and rough terrain |
| Screw and nut | Titanium-aluminium alloy | M5 tapped nut, 2 mm/rev pitch |
| Connecting link | Alumina alloy | 2.5 mm thick, joins upper and lower limbs |
| Shoulder shaft and CSK key | Magnesium alloy AZ31B | Joins the leg to the body |
| Bearings | SKF 61804 ball bearings and brass bushes | Reduce friction |

##  Assembly
- Main body used as the base part. Standard, advanced and mechanical mates were used.
- **Helical (screw) mate** converts rotary to linear motion.
- **Limiting-angle mates** restrict joint ranges (shoulder limited to ±10°).

  <img width="682" height="407" alt="image" src="https://github.com/user-attachments/assets/7d2afb0a-9797-4cfd-b119-46a4069084e3" />


## Mass Properties (SolidWorks)
| Property | Value |
|---|---|
| Mass | ~24.07 kg |
| Volume | 17,355,697 mm³ |
| Surface area | 2,389,296 mm² |
| Center of mass (mm) | X = -186.19, Y = 194.51, Z = 318.70 |
| Principal moments (g·mm²) | 4.42e8, 2.07e9, 2.27e9 |

##  Simulation
SolidWorks **Motion Study** with a rotary motor applied to the lower limb (5 s duration) to verify limb movement.

<img width="652" height="390" alt="image" src="https://github.com/user-attachments/assets/c9fdadd1-8962-42a4-9419-c83263d3241b" />


##  Future Work
- Select motors and integrate control systems
- Build a working 12-DOF prototype
- Improve stability and energy efficiency, and add sensors

## Demo Videos
- `Quadruped standing and sitting.mp4`
- `Leg movement.mp4`

## Acknowledgements
Thanks to the ROBO AI mentors and Dr. D. Ezhilarasi (Dept. of Instrumentation and Control Engineering, NIT Trichy), and teammates at Robotics and Machine Intelligence.


The CAD assemblies for the quadruped robot can be installed from the below links:
12 dof quadruped robot:
https://drive.google.com/drive/folders/15yNEa6HppzghK7HqHzqY5oAtGcAB1F9y?usp=sharing

8 DOF quadruped robot:
https://drive.google.com/drive/folders/1WK4c62Ob5OVSfts7Na62EBVl4cEYe_o9?usp=sharing


