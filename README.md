# SiMMER — Supervisory Control of Legged Mobile Manipulator

> A ROS-based supervisory control framework for a hexapod robot with an integrated manipulator arm, featuring adaptive stroke length control, full-cycle gait planning, and optimization-based redundancy resolution.

**Version:** 1.1  
**Date:** 26 February 2020  
**Maintainer:** Sulthan S F (<sulthansf95@gmail.com>)

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Architecture](#architecture)
- [Directory Structure](#directory-structure)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Package Details](#package-details)
- [Configuration](#configuration)
- [Control Modules](#control-modules)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

SiMMER provides real-time supervisory control for a **Legged Mobile Manipulator (LMM)** — a hexapod robot equipped with a 4-DOF manipulator arm. The system coordinates whole-body motion planning so the robot can walk and manipulate objects simultaneously, automatically repositioning its body when the manipulator approaches workspace limits.

The control pipeline runs as a set of ROS nodes that perform:

1. **End-effector command processing** — Accepting incremental position commands for the manipulator.
2. **Redundancy resolution** — Deciding when and where the trunk body must move to maintain manipulability.
3. **Gait planning** — Generating leg trajectories using full-cycle quadruped gait patterns.
4. **Inverse kinematics** — Computing joint angles for all 22 servos in real time.

---

## Features

- **Adaptive Stroke Length (ASL):** Dynamically adjusts the walking stride based on the required trunk displacement, optimized via SciPy's `fsolve`.
- **Full-Cycle Gait Planning:** Generates smooth quadruped gait patterns with configurable swing/support phase timing.
- **Optimization-Based Redundancy Resolution:** Minimizes a weighted cost function to determine optimal trunk repositioning while respecting manipulability thresholds.
- **Real-Time Inverse Kinematics:** Solves the full hexapod + manipulator IK at each control step.
- **Gazebo & RViz Integration:** Simulate and visualize the robot using standard ROS tools.
- **Joystick Teleoperation:** Incremental end-effector control via joystick input.
- **Data Logging:** Automatic CSV recording of joint states, end-effector positions, and manipulability metrics.

---

## Architecture

```
┌──────────────┐     ┌──────────────────┐     ┌────────────────────┐
│   Joystick   │────▶│  joy_incremental  │────▶│  lmm_incremental   │
│   Input      │     │     node          │     │    _inputs (topic) │
└──────────────┘     └──────────────────┘     └─────────┬──────────┘
                                                        │
                                                        ▼
                                              ┌──────────────────┐
                                              │   master node    │
                                              │                  │
                                              │ • Redundancy     │
                                              │   Resolution     │
                                              │ • TB Trajectory  │
                                              │ • Leg Tip Traj   │
                                              │ • Inverse Kinem. │
                                              └────────┬─────────┘
                                                       │
                                                       ▼
                                    ┌──────────────────────────────────┐
                                    │   lmm_joint_states (topic)       │
                                    └──────────┬───────────────────────┘
                                               │
                              ┌────────────────┼────────────────┐
                              ▼                ▼                ▼
                      ┌──────────────┐ ┌──────────────┐ ┌──────────────┐
                      │   Motor      │ │   Gazebo     │ │   RViz       │
                      │  Controller  │ │  Simulation  │ │  Visualizer  │
                      └──────────────┘ └──────────────┘ └──────────────┘
```

---

## Directory Structure

```
SiMMER/
├── README.md
├── Readme.txt                  # Original readme
│
├── lmm_model/                  # Robot description package
│   ├── package.xml
│   ├── config/                 # Controller gains & joint names
│   │   ├── controllers.yaml
│   │   └── joint_names.yaml
│   ├── launch/                 # Launch files for visualization & simulation
│   │   ├── display.launch      #   RViz visualization
│   │   └── gazebo.launch       #   Gazebo simulation
│   ├── meshes/                 # 3D STL models for robot links
│   ├── rviz/                   # RViz configuration files
│   └── urdf/
│       └── lmm.urdf            # Full robot kinematic description
│
├── lmm_sc/                     # Supervisory control package
│   ├── package.xml
│   ├── config/
│   │   └── joints.yaml         # Servo calibration (PWM limits, angle ranges)
│   ├── launch/
│   │   ├── start.launch        # Hardware deployment
│   │   └── simulation.launch   # Simulated control
│   └── src/
│       ├── master.py           # Main ROS node — real-time motion planner
│       ├── motor_controller.py # Servo command interface (I2C PWM)
│       ├── joy_incremental.py  # Joystick input handler
│       ├── tip_tf_broadcaster.py  # TF broadcaster for leg tips
│       └── modules/            # Core computational modules
│           ├── inputs.py
│           ├── initialize_lmm.py
│           ├── tb_time_calculation.py
│           ├── tb_trajectory.py
│           ├── leg_tip_traj_local.py
│           ├── leg_tip_traj_local_to_global.py
│           ├── redundancy_resolution.py
│           └── inverse_kinematics.py
```

---

## Prerequisites

| Requirement | Tested Version |
|---|---|
| **ROS** | Kinetic / Melodic |
| **Python** | 2.7 / 3.x |
| **NumPy** | ≥ 1.11 |
| **SciPy** | ≥ 0.18 |
| **Gazebo** | 7+ |
| **catkin** | (bundled with ROS) |

### ROS Dependencies

```
roscpp  rospy  std_msgs  sensor_msgs  tf2_ros  i2cpwm_board
robot_state_publisher  joint_state_publisher  rviz  gazebo
```

---

## Installation

1. **Create (or use an existing) catkin workspace:**

   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   ```

2. **Clone the repository:**

   ```bash
   git clone https://github.com/rohi021/SiMMER.git
   ```

3. **Install dependencies:**

   ```bash
   cd ~/catkin_ws
   rosdep install --from-paths src --ignore-src -r -y
   pip install numpy scipy
   ```

4. **Build the workspace:**

   ```bash
   catkin_make
   source devel/setup.bash
   ```

---

## Usage

### Visualize in RViz

```bash
roslaunch lmm_model display.launch
```

### Run Gazebo Simulation

```bash
roslaunch lmm_model gazebo.launch
```

### Launch Supervisory Controller (simulation)

```bash
roslaunch lmm_sc simulation.launch
```

### Launch Supervisory Controller (hardware)

```bash
roslaunch lmm_sc start.launch
```

### Run Individual Nodes

```bash
rosrun lmm_sc master.py              # Main motion planner
rosrun lmm_sc motor_controller.py    # Servo interface
rosrun lmm_sc joy_incremental.py     # Joystick teleoperation
```

### Joystick Setup

The system uses the standard [ROS `joy` package](http://wiki.ros.org/joy) to read gamepad/joystick input. The `joy_node` reads from a Linux joystick device (`/dev/input/js0` by default) using the Linux joystick API.

**Supported connection types** (all handled transparently by the OS):

| Connection | Example | How it works |
|---|---|---|
| **Wired USB** | Xbox/PS controller via USB cable | Plug in; appears as `/dev/input/js0` |
| **Wireless USB dongle** | Logitech controller with USB receiver | Plug in dongle; appears as `/dev/input/js0` |
| **Bluetooth** | PS4/PS5 DualShock via Bluetooth | Pair via OS Bluetooth settings; appears as `/dev/input/js0` |

The code is **connection-agnostic** — wired and wireless controllers are treated identically because the Linux kernel presents all joystick devices uniformly as `/dev/input/js*`.

**Verifying your joystick:**

```bash
# Check that the OS detects the controller
ls /dev/input/js*

# Test joystick input (install jstest-gtk or use jstest)
jstest /dev/input/js0
```

**Using a different device:** If your controller is not on `/dev/input/js0`, update the `dev` parameter in the launch file:

```xml
<param name="dev" value="/dev/input/js1" />
```

---

## Package Details

### `lmm_model` — Robot Description

Contains the URDF model, 3D meshes, and launch files needed to visualize and simulate the hexapod manipulator.

- **URDF** — Full kinematic chain: 6 legs × 3 joints + 4-DOF manipulator (22 actuated joints).
- **Meshes** — STL files for each link, used by RViz and Gazebo.
- **Controllers** — PID gains defined in `config/controllers.yaml`.

### `lmm_sc` — Supervisory Control

Implements the real-time control pipeline. The `master.py` node subscribes to incremental end-effector commands, plans whole-body motion, and publishes joint states at the configured control rate (default 10 Hz).

---

## Configuration

### `lmm_sc/config/joints.yaml`

Maps each joint to its servo channel, I2C board, angle limits, and PWM range. Example:

```yaml
joint_1_1:
  servo: 1
  board: 1
  min_angle: -1.5708    # rad
  max_angle:  1.5708
  min_input:  96         # PWM
  max_input:  510
```

### `lmm_sc/src/modules/inputs.py`

Central parameter file for the controller. Key parameters include:

| Parameter | Description | Default |
|---|---|---|
| `vel_tb_max` | Maximum trunk body velocity (m/s) | `0.04` |
| `f_in` | Control loop frequency (Hz) | `10` |
| `T_stroke` | Stroke duration (s) | `1` |
| `m_t_man` | Manipulability threshold | `0.25` |
| `Hmi1` | Swing height (m) | `0.015` |
| `lt_time_ratio` | Swing-to-stroke time ratio | `0.2` |
| `li` | Leg link lengths (m) | `[0.095, 0.105, 0.100]` |
| `l_man` | Manipulator link lengths (m) | `[0.0395, 0.250, 0.290]` |

### `lmm_model/config/controllers.yaml`

PID controller gains for all 22 position-controlled joints.

---

## Control Modules

| Module | Purpose |
|---|---|
| `inputs.py` | Robot dimensions, control parameters, and initial conditions |
| `initialize_lmm.py` | Computes initial joint angles to place the robot in its home pose |
| `tb_time_calculation.py` | Calculates timing for trunk body displacement based on velocity limits |
| `tb_trajectory.py` | Generates smooth trunk body position trajectories |
| `leg_tip_traj_local.py` | Creates local (body-frame) swing and support phase leg-tip trajectories |
| `leg_tip_traj_local_to_global.py` | Transforms local leg-tip trajectories to the global frame using TF |
| `redundancy_resolution.py` | Solves the weighted optimization problem to resolve kinematic redundancy |
| `inverse_kinematics.py` | Full-body IK solver (`ik_lmm_solver` class) for all legs + manipulator |

---

## Contributing

Contributions are welcome! To get started:

1. Fork the repository.
2. Create a feature branch: `git checkout -b feature/my-feature`.
3. Commit your changes: `git commit -m "Add my feature"`.
4. Push to the branch: `git push origin feature/my-feature`.
5. Open a Pull Request.

Please ensure your code follows the existing style and includes appropriate ROS logging.

---

## License

See the individual `package.xml` files for license information.  
`lmm_model` is released under the **BSD** license.
