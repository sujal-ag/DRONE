## Tutorial 1: System Overview of Autonomous Drones

### Overview
Introduction to drone software development, focusing on programming autonomous drones and understanding the main system components.

**What You'll Learn:**
- Architecture of an autonomous drone system
- How different components work together
- Why we use companion computers alongside flight controllers
- Choosing between PX4 and ArduPilot

### Key Components

#### 1. Flight Board (Flight Controller)
- **Hardware:** Accelerometer, IMU, and compass for stability and leveling
- **Software:** Runs autopilot firmware (PX4 or ArduPilot)

#### 2. Flight Sensors
- Enhance position and altitude accuracy
- **Examples:**
  - LiDAR (altitude measurement)
  - GPS (global position)
  - Optical Flow Sensor (precise indoor navigation)

#### 3. Telemetry Radio
- Enables communication between drone and Ground Control Station (GCS)
- Sends real-time flight data for monitoring and intervention

#### 4. Ground Control Station (GCS)
- Laptop/software interface displaying telemetry
- Used to control, monitor, and configure missions

#### 5. Companion Computer
- Onboard Linux computer (e.g., NVIDIA Jetson TX2, Raspberry Pi 4)
- **Why do we need this?** Flight controllers are optimized for real-time control loops (keeping drone stable) but lack the processing power for AI, computer vision, and complex decision-making. The companion computer handles:
  - Object detection and tracking
  - Path planning algorithms
  - Machine learning inference
  - Complex navigation logic
- **Division of Labor:**
  - Flight Controller: "Keep the drone stable, maintain altitude, execute basic commands"
  - Companion Computer: "Avoid that obstacle, follow that person, map the environment"
- Interfaces with sensors: LiDARs, cameras, IR modules

### Autopilot Software Comparison

**What is Autopilot Software?**
Autopilot firmware runs on the flight controller and handles low-level flight control, sensor fusion, and safety features. Think of it as the "operating system" for your drone's flight controller.

| Feature   | PX4                             | ArduPilot                         |
|-----------|----------------------------------|-----------------------------------|
| License   | No need to share source code     | Open-source (share modifications) |
| Focus     | Experimental, research-oriented  | Reliable, application-oriented    |
| Community | Smaller                          | Larger, stable, bug-tracked       |

**Recommended:** ArduPilot — reliable, tested, and widely supported.

---