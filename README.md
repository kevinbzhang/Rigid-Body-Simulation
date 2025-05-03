# SSHRBIP: Simulating Solid and Hollow Rigid Bodies on Inclined Planes

Welcome to the **SSHBRIP** repository: **Simulating Solid and Hollow Rigid Bodies on Inclined Planes**. This MATLAB project provides an extensive simulation environment for modeling, visualizing, and analyzing the dynamics of rolling bodies such as spheres and cylinders on inclined planes.

---

## 🔍 Project Overview

This simulation models the motion of rigid bodies (solid and hollow) under the influence of gravity, accounting for:

* Translational and rotational motion
* Coulomb friction and slip dynamics
* Collision detection and response
* Energy diagnostics: kinetic, potential, and frictional loss

All physical behaviors are calculated using symplectic Euler integration for stability and accuracy over time.

---

## 📁 Repository Structure

```
SSHBRIP/
├── source/
│   └── rigid_body_sim.m          # Main MATLAB simulation script
├── data/
│   ├── screenshot1.png           # Example simulation output
│   ├── ...                       # More images/data used in documentation
├── SSHRBIP Documentation.pdf     # Research paper with full documentation
└── README.md                     # This file
```

---

## 🚀 How to Run

1. Clone or download this repository.
2. Open MATLAB and navigate to the `source` directory.
3. Run `rigid_body_sim.m` to start the simulation.

Ensure MATLAB has access to the `data` folder if you are referencing visual outputs.

---

## 📚 Documentation

All theoretical framework, implementation details, and validation experiments are **extensively explained** in the research paper:

**📝 SSHRBIP Documentation — Simulating Solid and Hollow Rigid Bodies on Inclined Planes**

Please refer to this document for:

* Equations of motion and physics background
* Design decisions and methodology
* Performance and energy conservation analysis
* Visual diagnostics and data interpretation

The PDF is included in the root directory as `SSHBRIP Documentation.pdf`.

---

## 📸 Visual Data

Simulation outputs, diagnostic plots, and explanatory images are stored in the `data/` directory. These are referenced throughout the documentation and support the analysis of rolling behavior under various friction and geometry conditions.

---

## ✅ Features

* Energy decomposition: translational, rotational, and frictional loss
* Angular momentum tracking
* Slip diagnostics for rolling contact
* Collision restitution with incline boundaries

---

## 📌 Acknowledgements

This project was developed as part of a detailed simulation study in classical mechanics. Deep appreciation to all contributors to the SSHRBIP documentation.

---

## 📃 License

This project is open for academic and educational use. For other purposes, please contact the repository owner.

---

Happy simulating!

— SSHRBIP Team
