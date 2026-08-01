<p align="center">
    <img src="assets/banner.png" width="100%">
</p>

<h3 align="center">
All Magnetic Terrain Quadruped
</h3>

<p align="center">
Magnetic Quadruped Robotic Platform for Industrial Maintenance Applications
</p>

<p align="center">
<img src="https://img.shields.io/badge/MATLAB-R2025b-orange?style=for-the-badge&logo=mathworks">
<img src="https://img.shields.io/badge/Simulink-Model--Based%20Design-orange?style=for-the-badge">
<img src="https://img.shields.io/badge/ROS2-Humble-22314E?style=for-the-badge&logo=ros">
<img src="https://img.shields.io/badge/ESP32-Embedded-red?style=for-the-badge">
<img src="https://img.shields.io/badge/SolidWorks-CAD-red?style=for-the-badge">
<img src="https://img.shields.io/badge/Siemens-NX-blue?style=for-the-badge">
<img src="https://img.shields.io/badge/Python-3.x-blue?style=for-the-badge&logo=python">
<img src="https://img.shields.io/badge/C++-Embedded-blue?style=for-the-badge&logo=cplusplus">
<img src="https://img.shields.io/badge/License-MIT-success?style=for-the-badge">
<img src="https://img.shields.io/badge/Status-Active%20Development-success?style=for-the-badge">
</p>

<p align="center">
<img src="assets/AMTQ Information.jpg" width="800">
</p>

---

# Table of Contents
- [Overview](#overview)
- [Developer Role](#developer-role)
- [The Problem & The Solution](#the-problem--the-solution)
- [Project Highlights](#project-highlights)
- [Technical Specifications](#technical-specifications)
- [Project Evolution](#project-evolution)
- [System Architecture](#system-architecture)
- [Mechanical Design](#mechanical-design)
- [Electronics & Embedded System](#electronics--embedded-system)
- [Kinematics](#kinematics)
- [MATLAB & Simulink](#matlab--simulink)
- [ROS2 Integration](#ros2-integration)
- [Manufacturing Process](#manufacturing-process)
- [Experimental Validation](#experimental-validation)

---

# Overview
**AMTQ (All Magnetic Terrain Quadruped)** is a multidisciplinary robotics project focused on the design, development, and experimental validation of a magnetic quadruped robotic platform intended for future industrial maintenance and inspection applications on ferromagnetic structures.

<p align="center">
<img src="assets/AMTQ_COMPLETE.jpg" width="800">
</p>

Unlike simulation-only academic projects, AMTQ has been physically designed, manufactured, assembled, programmed, and experimentally validated through multiple prototype iterations.

The project demonstrates the complete engineering workflow:
**Concept → Design → Simulation → Manufacturing → Integration → Testing**

---

# The Problem & The Solution

Large ferromagnetic structures (storage tanks, bridges, offshore platforms) require periodic maintenance. Traditional inspection methods (rope access, scaffolding) increase operational costs and expose workers to hazardous environments.

AMTQ proposes a magnetic quadruped robotic platform capable of controlled locomotion while maintaining adhesion on metallic surfaces, improving safety and enabling future autonomous maintenance operations.

<p align="center">
<img src="images/AMTQ_IN_WALL.jpeg" width="700">
<br>
<em>AMTQ tested on a vertical ferromagnetic surface</em>
</p>

---

# Project Evolution

## AMTQ V1 - Experimental Prototype
Focused on validating the mechanical concept and proving the feasibility of magnetic locomotion.
<p align="center">
<img src="images/ATMQ_NO-Electromagnets.JPG" width="700">
</p>

## AMTQ V2 - Advanced Robotics Platform
Focuses on improving system integration, control performance, and future autonomous operation with electromagnetic modules.
<p align="center">
<img src="images/AMTQ_Electromagnets.JPG" width="700">
</p>

---

# System Architecture
The system is divided into four main layers: Mechanical, Control, Embedded, and Software.

---

# Mechanical Design
The mechanical system was designed using CAD tools with an emphasis on modular construction, structural rigidity, and weight optimization.

<p align="center">
<img src="assets/AMTQ DESIGN.png" width="900">
</p>

---

# Electronics & Embedded System
The embedded architecture is based on an **ESP32 controller** and custom hardware specifically designed for the robotic platform requirements, handling motor control, real-time commands, and electromagnetic adhesion.

<p align="center">
<img src="assets/Hardware.png" width="800">
</p>

---

# Kinematics
The robotic platform implements mathematical models for Forward and Inverse Kinematics, as well as trajectory generation to ensure smooth and precise movements.

<p align="center">
<img src="assets/AMTQ FOOT TRAJECTORIES 3D.jpg" width="48%">
<img src="assets/CARTESIAN ERROR IK-DAR AMTQ.jpg" width="48%">
</p>
<p align="center">
<img src="assets/SUAVIDAD ARTICULAR.jpg" width="60%">
</p>

---

# MATLAB & Simulink
MATLAB and Simulink are used for robot modeling, kinematic analysis, controller development, and simulation validation using a model-based design approach.

<p align="center">
<img src="assets/SIMULINK DIAGRAM.png" width="900">
</p>
<p align="center">
<img src="assets/BODY DIAGRAM.png" width="48%">
<img src="assets/LEG DIAGRAM.png" width="48%">
</p>

### Simulation in Action
<p align="center">
<img src="assets/AMTQ.gif" width="700">
<br>
<em>Simulated walking gait of the AMTQ platform</em>
</p>

---

# ROS2 Integration
ROS2 integration provides the foundation for future autonomous capabilities, including sensor integration, autonomous navigation, and computer vision.

---

# Manufacturing Process
The prototype was developed through CAD design, 3D printing/manufacturing, mechanical assembly, and electronics integration. 

---

# Experimental Validation
The physical prototype has been validated through multiple tests:

* **Mechanical Tests:** Individual leg movement, structural verification.
* **Locomotion Tests:** Walking sequence, trajectory execution.
* **Magnetic Adhesion Tests:** Electromagnetic attachment and vertical stability.
* **Control Tests:** Embedded control validation and MATLAB generated trajectories.

---
