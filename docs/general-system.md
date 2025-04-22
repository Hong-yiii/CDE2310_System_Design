---
title: General System
---

# 🔗 Navigation

- [Home](index.md)
- [The Challenge](challenge.md)
- [General System](general-system.md)
- [Software Subsystem](software.md)
- [Mechanical Subsystem](mechanical.md)
- [Electrical Subsystem](electrical.md)
- [Thermal Subsystem](thermal.md)
- [Testing & Validation](testing.md)
- [Areas for Improvement](improvements.md)

---

# General System

![System breakdown](assets/images/general_system/System_Diagram_2310.drawio.png)

We have decomposed the system into four interdependent subsystems:

- **Navigation (Bot)**  
  Handles real-time path planning, localization (via LIDAR and odometry), and map generation using the ROS2 Nav2 stack.

- **Navigation (Laptop)**  
  Visualizes map data, monitors robot state via RViz and topics, and can support external decision logging or override mechanisms.

- **Heat Detection**  
  Uses dual AMG8833 thermal sensors, feeding temperature grids into the controller. Detection triggers further localization using filtered LIDAR data to identify global heat source coordinates.

- **Launcher**  
  A dual flywheel launching mechanism actuated by motor commands. The launcher is synchronized with navigation goals to engage detected heat targets.

---

Each Subsystem is controlled by a global controller that handles all high level logic, this was conveyed to each subsystem using ROS2 Topics, the RQT graph for communication is below:
![RQT](assets/images/general_system/RQT.png)

## GlobalController: The Brain of the Bot

The `GlobalController` Python node runs in multi-threaded execution, enabling:

- **Sensor polling at 10 Hz** via the fast loop for IMU, LIDAR, and temperature grid updates
- **State decision-making at 1 Hz** in the control loop
- **Concurrent execution** of callbacks and services

It handles:
- State transitions across states such as:
  - `Exploratory_Mapping`
  - `Goal_Navigation`
  - `Launching_Balls`
  - `Imu_Interrupt`
  - `Attempting_Ramp`
- Autonomous heat source targeting using a KMeans-based clustering algorithm

## System Flow Overview

1. **Startup Phase**  
   Upon initialization, the robot begins in the `Initializing` state, waiting for valid map and sensor data. Once available, it enters the `Exploratory_Mapping` phase.

2. **Exploration Phase**  
   The robot autonomously explores the maze using frontier-based navigation. As thermal targets are detected, their global coordinates are recorded and visualized in RViz.

3. **Clustering & Goal Planning**  
   After the map is completed, detected heat points are clustered into goal positions. The robot then sequentially navigates to each target.

4. **Launching Phase**  
   At each goal, the robot stops, aligns itself, and launches a ping pong ball toward the heat source using the dual flywheel launcher.

5. **IMU Interrupt Handling**  
   During any phase, if a pitch anomaly is detected (e.g., going up a ramp), the system triggers the `Imu_Interrupt` state. The robot cancels current goals, marks unsafe zones in the occupancy grid, and replans its route.

6. **Ramp Engagement (Optional)**  
   If enabled, after all heat targets are engaged, the robot attempts to approach a ramp zone and perform a final launch

---

## Sensor Fusion & Decision Logic

- **Thermal + LIDAR Fusion**  
  When heat is detected, a laser scan in the direction of the sensor’s FOV is filtered and binned. The average angle and distance are computed and transformed into world coordinates.

- **IMU Feedback Loop**  
  Pitch data is processed in real-time. A rolling average is used to detect sudden inclinations that may signal a ramp or collision.

- **Occupancy Grid Manipulation**  
  The robot actively updates its map, sealing off dangerous or previously explored zones using adaptive flood-fill techniques and direct occupancy marking.

---

## Communication Infrastructure

All modules interact via ROS2 topics with appropriate QoS profiles. Key topics include:

- `cmd_vel`: Motion commands
- `scan`: Laser data
- `odom`: Odometry
- `temperature_sensor_1/2`: Heat grid data
- `flywheel`: Launch trigger
- `/visualization_markers`: Real-time RViz feedback for heat and sealed regions

> Communication is optimized for real-time response and modularity, with feedback loops embedded into every decision node.

---

## Why This Design?

This architecture allows us to:
- Run real-time behaviors in parallel
- Decompose complex behavior
- Replan dynamically based on ramp location, sensor feedback, or BT tree failure