---
title: Areas for Improvement
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

# Areas for Improvement



Despite strong preparation and a robust system design, our final run encountered a critical failure that impacted performance. The main issue stemmed from an **unsynchronized clock**, which resulted in **odom frame drops** that severely affected the robot’s ability to localize and navigate accurately.

### 1. **Clock Synchronization Issues**
Although the system was synced to Google's NTP servers, odometry frames were still intermittently dropped during runtime. This suggests that the synchronization may not have been tight enough to ensure real-time performance, particularly under load.

- **Improvement:** In future iterations, a local NTP server or a dedicated hardware clock could be used to provide more reliable and consistent synchronization, reducing network-induced timing jitter.

### 2. **Performance Impact from Screen Recording**
One hypothesis is that **screen recording during the final run**, which was not extensively tested prior, may have introduced additional system load. This could have skewed the OS clock on the laptop, causing inconsistencies in timestamp generation for ROS2 messages.

- **Improvement:** We will include screen recording and other auxiliary processes in all future test runs to simulate real operating conditions more accurately. This will help us preemptively catch any performance bottlenecks or timing issues.

### 3. **Hardware-Level Uncertainty on OpenCR Board**
There is also a possibility that **hardware limitations or malfunctions on the OpenCR board** affected the stability and consistency of the odometry data. This includes erratic publishing rates or internal delays.

- **Improvement:** We plan to perform targeted hardware diagnostics on the OpenCR board and consider switching out to a new board if issues persist. Testing metrics for Odom and Clk failures can be developed

---

## Alternative Strategy

Our current approach involved **fully scanning the maze** for potential heat sources, clustering the most probable detections using K-Means, and returning to those points for a final launch. While this provides **high confidence in the target’s location**, it is also **time-consuming and potentially inefficient**, especially in scenarios where rapid response is crucial.

A more **reactive and opportunistic strategy** could be adopted: instead of completing a full scan, the robot could **immediately navigate toward the first significant heat detection** and launch upon reaching a high-certainty threshold. This could reduce traversal time and decrease time and increase the possibility of finding any heat source if there were to be a catastrophic system failure similar to our last run.

- **Trade-off Consideration:** Although this sacrifices some global confidence for speed, it might align better with time-constrained tasks where partial detection is acceptable or where victims are likely to be located near entry points or known zones.

- **Implementation Direction:** This strategy could be implemented with dynamic priority queues, adjusting navigation goals in real-time based on live sensor input, and applying thresholds or Bayesian filters to trigger launches upon strong localized detections.
