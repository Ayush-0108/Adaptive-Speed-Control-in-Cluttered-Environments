# Adaptive Speed Control in Cluttered Environments (ROS 2 & Gazebo)

## Problem Statement
Fixed-speed navigation strategies compromise safety and efficiency when mobile robots operate in environments with varying obstacle density:
- **High Danger in Dense Clutter**: A fixed fast speed (e.g. $0.40 - 0.50\text{ m/s}$) causes dangerous overshoots, high centrifugal forces during turns, and catastrophic collisions due to insufficient stopping distance in tight corridors.
- **Severe Inefficiency in Open Space**: A fixed slow speed (e.g. $0.10\text{ m/s}$) guarantees safety in tight spots but imposes massive time penalties and battery waste when traversing wide open spaces.

## Objective
To develop an **Adaptive Speed Control Strategy** in ROS 2 that dynamically and continuously modulates robot velocity based on real-time **Obstacle Proximity** and **Environmental Clutter Complexity**, fully simulated and visualized in **Gazebo Sim** (Harmonic).

---

## Architecture Overview

```
                                  +------------------------------------+
                                  |        Gazebo Sim Harmonic         |
                                  |   (World: cluttered_arena.world)   |
                                  +-----------------+------------------+
                                                    |  /scan (LaserScan)
                                                    v
                                  +------------------------------------+
                                  |      Adaptive Speed Controller     |
                                  |   - Proximity Estimator (S_prox)   |
                                  |   - Clutter Density Metric (rho)   |
                                  |   - Turn Curvature Damper (S_turn) |
                                  |   - Dynamic Accel/Brake Limiter    |
                                  +-----------------+------------------+
                                                    |  /cmd_vel (TwistStamped)
                                                    v
                                  +------------------------------------+
                                  |  Robot Diff-Drive Controller (GZ)  |
                                  +------------------------------------+
```

### Speed Modulation Formulation
The composite target velocity is calculated via multi-criteria sensor filtering:
$$v_{\text{target}} = v_{\text{nominal}} \cdot S_{\text{prox}}(d_{\text{min}}) \cdot S_{\text{clutter}}(\rho_{\text{clutter}}) \cdot S_{\text{turn}}(\omega)$$

1. **Proximity Scaling ($S_{\text{prox}}$)**:
   $$S_{\text{prox}} = \begin{cases} 
   0.0 & d_{\text{min}} \le d_{\text{critical}} \quad (0.35\text{ m, Emergency Stop}) \\
   \left(\frac{d_{\text{min}} - d_{\text{critical}}}{d_{\text{safe}} - d_{\text{critical}}}\right)^{1.2} & d_{\text{critical}} < d_{\text{min}} < d_{\text{safe}} \quad (\text{Smooth Braking Zone}) \\
   1.0 & d_{\text{min}} \ge d_{\text{safe}} \quad (1.50\text{ m, Free Corridor})
   \end{cases}$$

2. **Environmental Clutter Index ($\rho_{\text{clutter}}$)**:
   Measures obstacle beam density in the forward hemisphere within $R_{\text{eval}} = 2.0\text{ m}$:
   $$\rho_{\text{clutter}} = \frac{1}{N} \sum_{i \in \text{FrontHemi}} \max\left(0, 1.0 - \frac{r_i}{R_{\text{eval}}}\right)$$
   $$S_{\text{clutter}} = \max\left(0.25, \; 1.0 - 0.65 \cdot \rho_{\text{clutter}}\right)$$

3. **Angular Steering Scaling ($S_{\text{turn}}$)**:
   Prevents skidding and tipping during tight steering maneuvers:
   $$S_{\text{turn}} = \frac{1}{1.0 + 1.2 \cdot |\omega|}$$

4. **Dynamic Acceleration & Braking Limiter**:
   - Forward acceleration limit: $0.80\text{ m/s}^2$
   - Braking deceleration limit: $2.20\text{ m/s}^2$ (proactive high-torque braking)

---

## Gazebo Cluttered Arena Layout
The simulation world (`cluttered_arena.world`) is a calibrated $18\text{m} \times 12\text{m}$ proving ground featuring three distinct zones:
- **Zone 1: Open Expressway ($x \in [-8, -3]$)**: Wide-open floor. The robot reaches peak cruise velocity ($0.50\text{ m/s}$) with $0\%$ clutter penalty.
- **Zone 2: Staggered Slalom ($x \in [-2, +2]$)**: Medium obstacle density with pillars spaced $\sim 1.5 - 2.0\text{m}$. Speed modulates smoothly to $0.25 - 0.35\text{ m/s}$.
- **Zone 3: Dense Maze ($x \in [+3, +8]$)**: Tight obstacle forest with cylindrical pillars and barriers spaced $0.8 - 1.1\text{m}$. Clutter density reaches $> 60\%$, automatically throttling velocity down to a safe crawl ($0.08 - 0.15\text{ m/s}$) to guarantee collision-free navigation.

---

## How to Run the Simulation

### 1. Launch with One Click (Windows)
Double-click `run_simulation.bat` or run in PowerShell:
```powershell
.\run_simulation.ps1
```

### 2. Launch from WSL / Linux Terminal
```bash
./run_simulation.sh
```

### 3. Launch with RViz Visualization
To launch Gazebo and RViz together (with real-time safety bubble markers and HUD text):
```bash
./run_simulation.sh use_rviz:=true
```

### 4. Benchmark Comparison: Fixed Speed vs. Adaptive Speed
To observe how a fixed-speed navigation strategy performs under identical conditions:
```bash
./run_comparison_fixed_speed.sh
```
*(In fixed-speed mode, the robot attempts to traverse the dense maze at fixed cruise speed, leading to aggressive near-misses and collisions at tight corners).*

---

## Live Telemetry HUD
The simulation streams a real-time ASCII telemetry dashboard directly into the console:
```text
┌─────────────────────────────────────────────────────────────┐
│ Velocity:   0.38 m/s |███████████████░░░░░| Req: 0.45 m/s │
│ Clutter:    28.4 %   |█████░░░░░░░░░░░░░░░| Scale: 84%     │
│ Front Dist: 1.42 m   │ Turn: +0.12 rad/s  │ State:[  APPROACHING   ]│
└─────────────────────────────────────────────────────────────┘
```

## ROS 2 Topics Interface
| Topic | Type | Description |
|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | 360° LiDAR range readings from Gazebo |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Safe modulated velocity driving the robot in Gazebo |
| `/cmd_vel_raw` | `geometry_msgs/msg/TwistStamped` | Desired nominal navigation commands |
| `/speed_controller/metrics` | `std_msgs/msg/String` | Real-time JSON telemetry (speed, clutter, distance) |
| `/speed_controller/markers` | `visualization_msgs/msg/MarkerArray` | 3D RViz safety zone bubble & floating HUD |
| `/odom` | `nav_msgs/msg/Odometry` | Robot odometry and position ground truth |
