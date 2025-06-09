# About ROS and Catkin

## What is ROS?

**ROS (Robot Operating System)** is an open-source software framework that offers a comprehensive set of tools, libraries, and conventions to develop robot applications.  
It simplifies robot programming by facilitating communication between various hardware and software components such as sensors, actuators, and algorithms in a modular and scalable manner.

---

## What is Catkin?

**Catkin** is the official build system of ROS, built on top of CMake.  
It is used to efficiently compile and manage custom ROS packages. Catkin helps developers organize their code into reusable and modular packages, making development, testing, and maintenance easier.

---

## MAVLink and MAVROS in Drone Applications

When working with drones in ROS, two important components come into play:

- **MAVLink**: A lightweight communication protocol widely used in drones for exchanging telemetry data, commands, and status information between the drone and ground control stations.

- **MAVROS**: A ROS package that serves as a bridge between the MAVLink protocol and the ROS ecosystem.  
  It receives telemetry data from the drone — such as GPS coordinates, attitude, battery status, and more — and converts these into ROS messages.  
  This integration allows ROS-based software to access and utilize drone data seamlessly.

---

## Benefits of Using MAVROS with ROS

By leveraging MAVROS within ROS, developers can build intelligent and autonomous functionalities like:

- Path planning  
- Obstacle avoidance  
- Mission management  

All powered by real-time telemetry data from the drone.

---
