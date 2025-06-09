# Gazebo Simulator Overview

<img src="https://upload.wikimedia.org/wikipedia/commons/1/13/Gazebo_logo.svg" alt="Gazebo Logo" width="200" />


---

## What is Gazebo?

Gazebo is a **powerful robotic simulator** that allows us to:

- Model different environments with detailed **visual features**  
- Animate various **obstacles** dynamically  
- Integrate multiple sensors such as **sonar, cameras, and LiDAR**

---

## Relationship with ArduPilot

- Gazebo is a **general-purpose robotic simulator**, not directly tied to ArduPilot.  
- The **ArduPilot community has developed a plugin** that allows Gazebo drone models to interface with ArduPilot software seamlessly.

---

## Key Feature: SITL Testing

This integration enables **SITL (Software In The Loop)** testing, where:

- The ArduPilot flight stack runs alongside the Gazebo simulation  
- Developers can perform **realistic and safe drone testing** in virtual environments  
- It helps in **validating drone software** without risking real hardware

---

> For more details, visit the [Gazebo Official Site](https://gazebosim.org/) and the [ArduPilot SITL Documentation](https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html).
