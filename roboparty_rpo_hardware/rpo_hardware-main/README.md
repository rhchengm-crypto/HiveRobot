# rpo_hardware - RoboParty RPO Humanoid Robot Hardware

**[中文](./README_cn.md)** | English

> **Project Introduction:** This repository is the open-source hardware library for the **RPO / Roboto Origin** humanoid robot platform. It provides mechanical design files, PCB schematics, manufacturing assets, and versioned hardware documentation for developers and builders.

---

## 📂 Repository Overview (Structure)

The project is organized by **hardware version**, with two primary branches of design files: **V1.0** and **V2.0**.

### 📊 V1.0 vs V2.0 Feature Comparison

| Feature | V1.0 (Legacy) | V2.0 (Current / Recommended) |
| :--- | :--- | :--- |
| **Mechanical Design** | Original Design | **Reinforced Structure** |
| **Hardware Architecture** | Discrete Modules | **Highly Integrated Single Board** |
| **Core Improvements** | - | **Waist Limiting, Pin Calibration, Backplate Switch** |
| **Use Case** | Legacy User Maintenance | New User Development, Mass Production |

---

### 🔧 V1.0 Legacy Hardware (`atom01_mechanic` / `atom01_pcb`)
**Positioning:** Original legacy version containing the early mechanical structure and discrete circuit board design. The historical directory names are retained for compatibility with existing files and references.

-   **Core Components:**
    -   **Mechanical:** Basic bipedal structure design, including early assembly documentation and URDF files.
    -   **Hardware:** Separate Power and Communication boards (Roboto_Power, Roboto_Usb2Can).
-   **Target Audience:** Limited to legacy users still utilizing the first-generation hardware for reference.

### 🚀 V2.0 RPO / Roboto Origin Hardware (`roboto_origin_mechanic` / `roboto_origin_pcb`)
**Positioning:** Current recommended version, comprehensively restructured for usability, stability, and integration.

#### ✨ Core Update Highlights
-   **🚀 Hardware System: Single Board Integration**
    -   **Optimization:** Integrated modules such as power management and communication interfaces into a **single core circuit board**.
    -   **Advantage:** Significantly simplifies internal cabling, reduces connector failure points, and enhances system stability and EMC (Electromagnetic Compatibility).
-   **⚙️ Mechanical System: Structural Reinforcement & Calibration Upgrade**
    -   **Waist Limiting:** Added a **mechanical limit switch** to ensure absolute safety for waist rotation angles.
    -   **Calibration Method:** Introduced **limit pin calibration** process, replacing the generic calibration method of the old version.
    -   **Arm Optimization:** Corrected issues present in the arm components of the previous version.
    -   **Backplate Switch:** Added a physical **main control board switch** to the robot's backplate.
-   **Target Audience:** All new users, developers, and users looking to experience the latest features.

---

## 📚 Quick Start

Please enter the corresponding directory based on your hardware version to obtain detailed documentation:

1. **V2.0 Users (Recommended):**
    -   **Mechanical Drawings:** Enter `V2.0/roboto_origin_mechanic/` to view assembly drawings and STL files.
    -   **PCB Files:** Enter `V2.0/roboto_origin_pcb/` to obtain Gerber files and schematics.
2. **V1.0 Legacy Users:**
    -   **Mechanical Drawings:** Enter `V1.0/atom01_mechanic/`.
    -   **PCB Files:** Enter `V1.0/atom01_pcb/`.

---

> **Note:** The mechanical structure and hardware interfaces of **V2.0 are not compatible with V1.0**. Please confirm the version before assembly or purchasing materials.

## Naming Notes

This repository was migrated from the historical Atom01 naming scheme to `rpo_hardware`.
Some V1.0 directories and model files still use historical `atom01` names as compatibility paths.
Do not rename those paths unless the related CAD, URDF, PCB, and documentation references are updated and validated together.
