---
title: Thermal Subsystem
---

# 🔗 Navigation

- [Home](index.md)
- [The Challenge](challenge.md)
- [General System](general-system.md)
- [Software Subsystem](software.md)
- [Mechanical Subsystem](mechanical.md)
- [Electrical Subsystem](electrical.md)
- [Thermal Subsystem](thermal.md)
- [End User Documentation & BOM](user_docs.md)
- [Areas for Improvement](improvements.md)

---

# Thermal Subsystem

---


## System Overview and Design Rationale

The heat detection subsystem plays a critical role in the robot’s mission, enabling it to autonomously detect, localize, and interact with simulated thermal targets in the environment. The system utilizes two AMG8833 8×8 thermal sensors, each connected to the Raspberry Pi via I²C and publishing to ROS2 topics `temperature_sensor_1` and `temperature_sensor_2`. These sensors are positioned at the front-left and rear-right corners of the robot to provide complementary coverage across the robot’s frontal arc.

The `GlobalController` node subscribes to both sensor topics using the `sensor1_callback()` and `sensor2_callback()` functions. Within these callbacks, a 4×4 subset of the 8×8 grid is extracted using hardcoded indices (e.g., 18–21, 26–29...) to avoid fringe readings and focus on the center of the robot’s forward and left field of view. Each extracted grid is passed to the `valid_heat()` function, which checks if any temperature exceeds the detection threshold (`self.temp_threshold = 27°C`).

If a heat signature is detected, the robot immediately initiates a multi-step spatial localization routine that includes LIDAR sampling, polar coordinate estimation, and world-frame projection.

---

# Heat Source Localization Pipeline

When a valid thermal reading is detected in either sensor callback, the robot performs the following steps to calculate the heat source's global coordinates:

1. **Laser Filtering and Angular Binning**  
   The function `laser_avg_angle_and_distance_in_mode_bin()` is invoked with a ±7° angular window centered around the thermal sensor's expected field of view. This function filters the LIDAR scan data to isolate a forward-facing sector and bins valid distances at 0.1-meter intervals. The most populated bin is selected to ensure a robust estimate against outliers and reflections.

2. **Coordinate Transformation**  
   The average angle and distance from the mode bin are converted to global (x, y) coordinates via the `calculate_heat_world()` method. This internally calls `polar_to_world_coords()` to perform the trigonometric projection:

   ```text
   x = xr + d⋅cos(θ+ψ)  
   y = yr + d⋅sin(θ+ψ)
   ```

   where `xr`, `yr` is the robot's current position obtained from `get_robot_global_position()` and `ψ` (psi) is its yaw orientation.

3. **Data Logging**  
   Validated coordinates are stored in `self.heat_left_world_x_y` or `self.heat_right_world_x_y` depending on which sensor was triggered. This continues throughout the mapping phase.

Markers for heat sources are published to `/visualization_markers` via:

```python
self.publish_visualization_markers()
```

These appear in RViz as green spheres at detected coordinates. The robot’s heat detection and goal navigation are managed using a thread-safe finite state machine.

---

# Clustering and Navigation Target Generation

Once exploratory mapping completes (`self.finished_mapping = True`), all stored heat detections are passed to the `find_centers()` method, which applies the KMeans clustering algorithm (via `sklearn.cluster.KMeans`) to group spatially close detections into N distinct thermal targets (`N = self.clusters`, default 3). The centroids of each cluster are stored in `self.max_heat_locations`, which form the robot’s firing targets.

The transition to the `Goal_Navigation` state is handled in the `control_loop()` and follows this sequence:

- The robot calls `nav_to_goal(x, y)` for each heat location.
- Upon arrival (verified in `goal_result_callback()`), the robot stops via `stopbot()` and transitions to `Launching_Balls`.
- It then fires a ping pong ball using `launch_ball()`, which publishes a value to the `flywheel` topic to trigger the dual-flywheel launcher mechanism.

---

# Heat Detection Testing and Performance

## Objective

The goal of testing was to validate the effectiveness of the robot’s heat detection system, which uses two AMG8833 thermal cameras mounted at the front-left and rear-right of the robot. The system needed to:

- Reliably detect thermal sources within a specified angular and distance range,
- Differentiate between genuine heat sources and ambient interference,
- Produce consistent, accurate coordinates for downstream navigation and launching tasks.

## Methodology

Testing was carried out in a controlled indoor setting using a stationary 60W light bulb as a simulated heat source. The tests included:

- **Static Evaluation**:  
  The robot was placed at known distances from the heat source (0.5m to 2.0m) in a stationary pose. The thermal sensors sampled data while the robot remained still to determine detection range and latency.

- **Dynamic Evaluation**:  
  The robot performed full autonomous runs through a maze-like test environment. During movement, the robot logged thermal detections, paired them with filtered LIDAR measurements, and calculated world-frame positions for each detected heat source.

Each AMG8833 sensor publishes an 8×8 thermal grid at 10 FPS. Inside the `sensor1_callback()` and `sensor2_callback()`, a central 4×4 region is extracted and evaluated using the `valid_heat()` method. If a reading exceeds the `self.temp_threshold` (27°C by default), the LIDAR scan is processed using `laser_avg_angle_and_distance_in_mode_bin()`, which returns an angle and distance estimate based on the most frequent bin in the relevant field of view.

The heat source’s global position is then determined through `calculate_heat_world()` using the robot’s pose from TF. Valid points (within `self.heat_distance_max = 2.5m`) are stored in `self.heat_left_world_x_y` or `self.heat_right_world_x_y`.

After the exploration phase, all detected coordinates are clustered using KMeans (`find_centers()`), generating target points used for autonomous navigation and ball launching.

## Performance Metrics Evaluated

- **Detection Success Rate**  
  Measures the system’s ability to correctly detect thermal sources across runs and orientations.

- **Response Time**  
  Time between when the heat source enters the field of view and when a valid detection is logged. Targeted to be under 500 milliseconds.

- **False Positive Rejection**  
  Ability to suppress detections from ambient reflections or metallic heat signatures, especially in dynamic lighting conditions.

## Results and Interpretation

| Metric                            | Target         | Achieved |
|----------------------------------|----------------|----------|
| Max Reliable Range               | ≥ 1.5 m        | ☑️       |
| Detection Success Rate (static)  | ≥ 95%          | ☑️       |
| Response Time                    | ≤ 500 ms       | ☑️       |
| False Positive Rate              | ≤ 10%          | ☑️       |
| Clustering Accuracy              | 3 Distinct groups | ☑️    |

- In static tests, both sensors demonstrated 100% accuracy in detecting the light bulb within 1.5 meters.  
- In dynamic tests, the robot consistently detected and logged thermal signatures, even when one sensor was momentarily obstructed.  
- Early-stage false positives were minimized through:
  - Selective frame filtering (center region only),
  - Tight angular windows in LIDAR processing,
  - Dual-sensor redundancy.

This combination of real-time filtering, angle-based fusion, and dual-perspective sensing significantly increased the reliability of heat localization.

---

# Key Implementation References in Code

| Feature                               | Function(s) Involved                              |
|--------------------------------------|---------------------------------------------------|
| Thermal grid filtering                | `sensor1_callback()`, `sensor2_callback()`        |
| Heat thresholding                     | `valid_heat()`                                    |
| LIDAR-assisted angle and distance extraction | `laser_avg_angle_and_distance_in_mode_bin()` |
| Polar to world transformation         | `calculate_heat_world()`, `polar_to_world_coords()` |
| Clustering                            | `find_centers()`                                  |
| State transitions                     | `control_loop()` and `set_state()`                |
| Target navigation                     | `nav_to_goal()`, `goal_result_callback()`         |
| Ball launching                        | `launch_ball()`                                   |
| Visualization                         | `publish_visualization_markers()`                 |

---