---
title: Electrical Subsystem
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

# Electrical Subsystem

The electrical subsystem outlines the required components that are critical for the mission. This includes the flywheel sizing, power budget, flowchart, electrical circuit diagram and connections.

### Flywheels

The launcher must propel a standard ping pong ball to a vertical height of at least 1.5 meters at the heat source.

Using the kinematics formula:

$$
v^2 = u^2 + 2as
$$

$$
v^2 = 0^2 + 2(-9.81 \times 1.5)
$$

$$
V = 5.4249\, \text{m/s}
$$

Using the linear-to-angular velocity relation:

$$
N = \frac{60V_f}{\pi d}
$$

Where:  
- $d$ = flywheel diameter  
- $V_f$ = exit velocity  
- $N$ = RPM

| Flywheel Diameter (mm) | RPM      |
|------------------------|----------|
| 30                     | 3449 RPM |
| 35                     | 2956 RPM |
| 40                     | 2587 RPM |
| 45                     | 2300 RPM |
| 50                     | 2069 RPM |
| 57                     | 1820 RPM |

Our flywheel size will be 57mm.

---

In normal operation, the likely throttle will be 10% since it is a large flywheel.

Assumed overall efficiency: 50%, to account for:
- Frictional losses  
- Suboptimal contact with ping pong ball  
- Interference fit inefficiencies

We are using two flywheels spinning in opposite directions.

Thus, each flywheel must run at:

$$
\frac{1820}{2} = 910\,\text{RPM}
$$

This gives us a total RPM of 910 x 2 = **1820 RPM**

#### Motor Performance and Power Budget

- Motor rated max RPM: 24,975 RPM
- Motor current at max RPM = 6.5 A
- Expected throttle in real usage: 10%
  - At 10% throttle:
    - $$\frac{24,975}{100} \times 10 = 2497.5\,\text{RPM}$$
    - > 1820 RPM required → Sufficient for mission
- Motor current at 10% throttle:
  - $$\frac{6.5}{10} = 0.65 A$$
- Total current for 2 motors:
  - $$0.65 \times 2 = 1.3 A$$
- Power at 11.1V (3S LiPo):
  - $$1.3 A \times 11.1 V = 14.43 W$$

### Power Budget

| Components                      | Voltage(V) | Current(A) | Wattage(Maximum)(W) |
|-------------------------------|------------|------------|----------------------|
| Turtlebot max speed           | 11.1       | 0.99       | 12.3                 |
| Turtlebot idle                | 11.1       | 0.81       | 9                   |
| Feetech FS90R Micro Servo     | 5          | 0.8        | 4                   |
| AMG8866 x 2                   | 3.3V       | 0.0045     | 0.0297              |
| Motor of flywheel x2          | 11.1       | 1.68       | 14.43               |
| Speed controller x2 (90% eff) | 11.1       | 0.12       | 1.33                |
| TXS0108E Level Converter      | 5          | ~0         | ~0                  |

> The logic converter is set to 0 as it draws 8x10^-6 A at worst.

### Battery Life Estimation

#### Battery Specifications (TurtleBot3 Standard LiPo)
- Voltage: 11.1V
- Capacity: 1800mAh (1.8Ah)
- Energy:
  - $$11.1 V \times 1.8 Ah = 19.98 Wh$$
- Average Power Usage (From Testing): 11W

#### Ideal Runtime (100% Efficiency)
- $$\text{Battery Life} = \frac{19.98Wh}{11W} = 1.816 \text{ hours} = 109 \text{ minutes}$$

#### Adjusted Runtime (70% Real-World Efficiency)
- $$109 \times 0.7 = 76.3 \text{ minutes}$$

#### Mission Duration Feasibility
- Single mission duration: 25 minutes
- Available runtime: ~76 minutes
- 76.3/25 = **~3 full missions possible**

> The TurtleBot3 battery provides enough power to complete the mission at least three times under real-world conditions, at 30% inefficiency.

---

### Electrical Subsystem Connections

The electrical subsystem integrates sensor feedback, PWM motor control, and power distribution for the launcher mechanism and TurtleBot3 operations as shown here:

*overall wiring.drawio (4).pdf insert the file here*

*The circuit of the turtlebot is shown here*

*CDE2310-PDR.pdf insert the file here*

#### IR Sensor (AMG8833) Connection

- Sensor Type: AMG8833 IR thermal camera
- Interface: I2C
- Wiring:
  - SDA (Data) → GPIO2 (BCM)
  - SCL (Clock) → GPIO3 (BCM)

> These sensors detect the heat signature of the target area and are connected directly to the Raspberry Pi I2C interface.

#### Flywheel Motor Control

- Motors: 2 × Brushless DC Motors
- Control Interface: ESC (Electronic Speed Controller)
- Signal Requirement: 50Hz PWM, with a 1ms–2ms pulse width:
  - 1ms = minimum throttle
  - 2ms = maximum throttle
- PWM Control Pins (BCM mode):
  - Left Motor → GPIO18
  - Right Motor → GPIO12

> Raspberry Pi GPIO outputs are 3.3V logic, but ESCs require a 5V signal. A logic level shifter is used to step up the PWM signals to 5V.


