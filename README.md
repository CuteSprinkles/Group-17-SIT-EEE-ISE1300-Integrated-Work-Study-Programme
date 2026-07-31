# 🐢 TurtleBot3 Line Following, Barcode Navigation & Board Toppling

> **EEE/ISE1300 Innovation Project — Group 17, Singapore Institute of Technology**

A ROS 2 controller for a TurtleBot3 Burger that follows a black line, reads Pixy2 barcodes, performs odometry-based turns, detects a toppling board using LiDAR, pushes it over with a linear actuator, and safely returns to the track. `(ง •̀_•́)ง`

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros\&logoColor=white)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10%2B-3776AB?logo=python\&logoColor=white)](https://www.python.org/)
[![Platform](https://img.shields.io/badge/Platform-TurtleBot3%20Burger-FF6F00)](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
[![Camera](https://img.shields.io/badge/Camera-Pixy2-6A5ACD)](https://pixycam.com/pixy2/)

---

## 🌟 Project Overview

The robot follows a **6 mm-wide black line** using a Pixy2 camera operating in line-tracking mode. Barcodes placed beside the line provide navigation commands for the next junction or trigger a special task.

| Barcode | Command         | Robot behaviour                                                              |
| :-----: | --------------- | ---------------------------------------------------------------------------- |
|   `0`   | ⬅️ **LEFT**     | Stores the command, enters the next junction, and turns left using odometry  |
|   `1`   | ➡️ **RIGHT**    | Stores the command, enters the next junction, and turns right using odometry |
|   `2`   | 🎯 **OBSTACLE** | Locates, approaches, and topples a board before returning to the line        |
|   `3`   | 🛑 **STOP**     | Stops the robot and completes the run                                        |

The controller is implemented as a single ROS 2 node named `TurtleBot3CombinedController`. It uses **27 explicit finite-state-machine states** for line following, turning, obstacle handling, recovery, and safety.

---

## ✨ Main Features

* 📷 **Pixy2 line tracking** using filtered line vectors near the bottom of the image
* 🧭 **Odometry-based junction turns** for LEFT and RIGHT commands
* 🗳️ **Barcode majority voting** to reduce incorrect command acceptance
* 📥 **Command queuing** for barcodes placed close together
* 🎯 **LiDAR-guided board detection and alignment**
* 📏 **Distance-based obstacle approach and return**
* 🔧 **Linear-actuator extension and board-toppling sequence**
* ↩️ **Automatic return to the saved track position and heading**
* 🚦 **Left, right, stop, line-loss, and fail-stop LED indications**
* 🔊 **Distinct buzzer patterns** for commands, states, success, and faults
* 🕳️ **Close-board NaN handling** for objects inside the LiDAR’s valid minimum range
* 👁️ **Sensor freshness watchdog** for `/scan` and `/odom`
* 🛡️ **Latched fail-stop system** that requires human recovery
* 🧾 **Automatically generated HTML fail-stop report**
* 🎨 **Friendly and debug console-output modes**
* ⚙️ **Environment-variable tuning** without modifying the Python file

---

## 🧠 Line-Following Controller

The Pixy2 line-tracking frame has a coordinate field of approximately `79 × 52`. The desired line position is the central image column at approximately `x = 39`.

The tracking error is evaluated near the lower part of the frame at `y = 45`, representing the section of the line closest to the robot.

```text
e[k] = measured line position - desired centre position
```

The controller applies proportional-derivative control:

```text
u[k] = -(Kp × e[k] + Kd × (e[k] - e[k-1]))
```

Default gains:

```text
Kp = 0.050
Kd = 0.012
```

The negative sign converts the Pixy2 image error into the TurtleBot3 angular-velocity convention.

Angular velocity is limited to prevent excessively aggressive steering. The robot also reduces its forward speed as the magnitude of the line error increases, helping to reduce corner cutting when travelling around bends.

---

## 🗳️ Barcode Voting and Command Handling

A command is not accepted from a single Pixy2 reading.

By default, the controller collects up to three valid readings within a short voting window. A barcode normally requires at least two matching readings to be accepted.

```text
Maximum samples : 3
Required votes  : 2
Voting window   : 0.30 s
```

The result may be accepted as soon as two identical readings are obtained because the majority is already mathematically decided.

Additional protection includes:

* separate handling for repeated and different barcode IDs;
* distance-based barcode latch clearing;
* storage of up to two pending LEFT or RIGHT commands;
* no unnecessary post-turn barcode blackout;
* immediate handling of Barcode 3 STOP;
* spatial re-arming protection for Barcode 2;
* stricter voting for other commands while the robot remains near a recently used Barcode 2.

These measures help the robot detect barcodes placed close together while reducing repeated or incorrect command execution.

---

## 🎯 Board-Toppling Routine

When Barcode 2 is confirmed, the robot performs the following sequence:

1. Stops and stores its current position and heading.
2. Allows the LiDAR readings to settle.
3. Scans the left and right sides of the robot.
4. Uses several LiDAR scans to vote for the most likely board side.
5. Turns approximately 90° towards the selected side.
6. Detects the front-board LiDAR cluster.
7. Fine-aligns the actuator axis with a valid region of the board.
8. Approaches the board while continuously checking distance and alignment.
9. Stops at the calculated actuator deployment distance.
10. Extends the actuator fully.
11. Drives forward to push the board beyond its tipping point.
12. Uses board-distance and wheel-stall evidence to determine contact or toppling.
13. Reverses by the measured travelled distance.
14. Returns to the saved obstacle-start position.
15. Restores the robot’s original heading.
16. Reacquires the black line and continues the course.

The controller uses a **board hit window** rather than requiring the actuator to point at the exact geometric centre. This allows small alignment errors while ensuring that the actuator remains safely within the detected board edges.

---

## 🕳️ Close-Board LiDAR Handling

The board may sometimes be closer than the LiDAR’s minimum valid measurement range. Depending on the sensor driver, this may appear as:

* `NaN`;
* zero or near-zero values; or
* invalid sub-minimum readings.

The `BLIND_VALUE_MODE` environment variable controls how these readings are interpreted:

```text
nan   = count only NaN readings
zero  = count only zero or near-zero readings
both  = count NaN, zero, and sub-minimum readings
```

If a close board is detected after the robot turns towards it, the robot:

1. confirms the repeated invalid front-reading pattern;
2. checks whether there is sufficient clearance behind the robot;
3. reverses by a controlled distance;
4. waits for the board to reappear within the valid LiDAR range;
5. performs front alignment again.

Run the controller with `LIDAR_DIAGNOSTIC=1` to inspect how the actual LiDAR driver reports close objects.

---

## 🔌 Hardware

| Component                                | Purpose                                                                                  |
| ---------------------------------------- | ---------------------------------------------------------------------------------------- |
| **TurtleBot3 Burger**                    | Mobile robot platform containing the Raspberry Pi, OpenCR, motors, and wheel odometry    |
| **Pixy2 camera**                         | Black-line, intersection, and barcode detection                                          |
| **LDS-02 / LD08 LiDAR**                  | Board-side selection, front alignment, distance measurement, and rear-clearance checking |
| **Linear actuator / servo-style pusher** | Extends and pushes the board beyond its tipping point                                    |
| **Left and right LEDs**                  | Navigation and turn indication                                                           |
| **Red LED**                              | STOP, line-loss, sensor-pause, and fail-stop indication                                  |
| **TurtleBot3 sound service**             | Command, state, success, and fault buzzer patterns when available                        |

### 📍 GPIO Connection Pins on Raspberry Pi 4

```text
Left LED   : GPIO 23
Right LED  : GPIO 24
Red LED    : GPIO 25
Actuator   : GPIO 18 at 50 Hz PWM
```

Default actuator pulse widths:

```text
Retract : 1.0 ms
Extend  : 2.0 ms
```

> [!IMPORTANT]
> The geometry values in the controller are specific to the physical robot. Measure the LiDAR-to-front distance, actuator reach, and rear overhang before final testing.

---

## 🧰 Software Requirements

* Ubuntu 22.04 or another environment compatible with **ROS 2 Humble**
* ROS 2 Humble
* TurtleBot3 ROS 2 packages
* Python 3.10 or later
* TurtleBot3 Burger bring-up workspace
* Pixy2 Python bindings
* `RPi.GPIO` for LED and actuator control
* `turtlebot3_msgs` for optional sound-service support

The controller expects the Pixy2 Python bindings at:

```text
~/pixy2/build/python_demos
```

Change `PIXYPATH` near the top of the controller when the bindings are installed elsewhere.

---

## 🔗 ROS 2 Interfaces

| Type            | Interface                                                                  | Purpose                                     |
| --------------- | -------------------------------------------------------------------------- | ------------------------------------------- |
| Publisher       | `/cmd_vel` — `geometry_msgs/msg/Twist`                                     | Robot linear and angular velocity           |
| Subscriber      | `/odom` — `nav_msgs/msg/Odometry`                                          | Position, distance, and heading measurement |
| Subscriber      | `/scan` — `sensor_msgs/msg/LaserScan`                                      | Board detection and clearance measurement   |
| Service         | `/turtlebot3_combined_controller/reset_fail_stop` — `std_srvs/srv/Trigger` | Clears a latched fail-stop                  |
| Optional client | `/sound` — `turtlebot3_msgs/srv/Sound`                                     | Plays buzzer patterns                       |

The `/scan` subscription uses **BEST_EFFORT** reliability and **VOLATILE** durability to match the LiDAR publisher.

All motion commands pass through `publish_motion()`, which is the only method that publishes to `/cmd_vel`. This prevents individual FSM branches from bypassing the controller’s safety overrides.

---

## 📂 Repository Structure

```text
.
├── Group17Turtlebot3proj.py    # Main combined ROS 2 controller
├── README.md                   # Project documentation
├── error_sample.html           # Example generated fail-stop report
└── Tweaking_Parameters.pdf     # Parameters to tweak

```
---

## 🚀 Running the Controller

For a clean repository, rename the main controller to:

```text
Group17Turtlebot3proj.py
```

Three terminal windows are recommended:

### Terminal 1 — TurtleBot3 Bring-Up

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash

export ROS_DOMAIN_ID=21
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02

ros2 launch turtlebot3_bringup robot.launch.py
```

### Terminal 2 — Recovery Teleoperation

**Terminal 2 is reserved for recovery teleoperation and does not need to run continuously.**

Keep this terminal available. Start teleoperation only after the autonomous controller has entered a fail-stop and stopped publishing repeated zero-velocity commands.

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash

export ROS_DOMAIN_ID=21
export TURTLEBOT3_MODEL=burger

ros2 run turtlebot3_teleop teleop_keyboard
```

> [!CAUTION]
> Do not send teleoperation commands while the robot is operating autonomously. Reposition the robot only after it has entered fail-stop, is stationary, and is safe to approach.

### Terminal 3 — Combined Controller

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash

export ROS_DOMAIN_ID=21
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02

python3 Group17Turtlebot3proj.py
```

Stop the program using `Ctrl+C`.

During cleanup, the controller:

* publishes repeated zero-velocity commands;
* disables the actuator signal;
* switches off the LEDs;
* stops PWM output;
* releases the GPIO pins; and
* closes the Pixy2 connection when supported.

---

## 🧪 Recommended Dry-Run Test

Disable the physical actuator while testing the navigation and obstacle routine:

```bash
ENABLE_ACTUATOR=0 \
CONSOLE_STYLE=debug \
python3 Group17Turtlebot3proj.py
```

This allows the controller to run through the obstacle states without powering the actuator.

> [!WARNING]
> A dry run disables actuator movement but does not disable wheel motion. Keep the robot in a clear test area and remain prepared to stop it.

### 📡LiDAR Diagnostic Test

To inspect how the LiDAR reports close objects:

```bash
ENABLE_ACTUATOR=0 \
LIDAR_DIAGNOSTIC=1 \
CONSOLE_STYLE=debug \
python3 Group17Turtlebot3proj.py
```

Place the board at the intended competition distance and check whether the close readings appear as `NaN`, zero, or another invalid value. Set `BLIND_VALUE_MODE` accordingly.

---

## 🛡️ Fail-Stop and Recovery

A fail-stop may be triggered by conditions such as:

* stale or repeatedly interrupted sensor data;
* sensor recovery exceeding its allowed grace period;
* an obstacle manoeuvre exceeding its timeout or distance limit;
* failure to identify or align safely with the board;
* insufficient rear clearance during close-board recovery;
* failure to complete the return manoeuvre safely; or
* an unexpected FSM transition during a sensor pause.

During fail-stop, the controller:

1. commands zero velocity;
2. cancels queued commands and incomplete barcode votes;
3. disables the actuator signal;
4. rapidly flashes the red LED;
5. plays a distinct siren pattern when the sound service is available;
6. prints a recovery card in the terminal;
7. optionally writes an HTML fault report; and
8. waits for a human reset rather than guessing or resuming mid-manoeuvre.

The controller publishes repeated zero-velocity commands for a short hold period. It then releases `/cmd_vel`, allowing a teleoperation node to reposition the robot without fighting the autonomous controller.

After manually repositioning the robot, clear the fail-stop using:

```bash
ros2 service call \
  /turtlebot3_combined_controller/reset_fail_stop \
  std_srvs/srv/Trigger "{}"
```

The reset:

* clears queued commands;
* clears stale manoeuvre and sensor-pause data;
* resets the PD-controller memory;
* stops the actuator signal; and
* enters line-reacquisition mode.

The robot does **not** resume from the middle of an interrupted manoeuvre. Autonomous movement resumes only after the Pixy2 confirms the line for several consecutive frames.

A sample generated report is included in [`error_sample.html`](error_sample.html).

---

## ⚙️ Configuration

Most tuning values read an environment variable of the same name, allowing parameters to be changed without editing the Python file.

Example:

```bash
FORWARD_SPEED=0.12 \
LINE_KP=0.045 \
LINE_KD=0.010 \
CONSOLE_STYLE=debug \
python3 Group17Turtlebot3proj.py
```

### Common Environment Variables

| Variable                         |            Default            | Description                                                                 |
| -------------------------------- | :---------------------------: | --------------------------------------------------------------------------- |
| `CONSOLE_STYLE`                  |           `friendly`          | `friendly` for concise output or `debug` for detailed sensor and FSM values |
| `NO_COLOUR`                      |              `0`              | Set to `1` to disable ANSI terminal colours                                 |
| `CONTROL_PERIOD_SEC`             |             `0.05`            | Main control-loop period; `0.05 s` corresponds to 20 Hz                     |
| `ENABLE_ACTUATOR`                |              `1`              | Set to `0` for a motion-only dry run                                        |
| `FAIL_STOP_HTML`                 |              `1`              | Enables the HTML fail-stop report                                           |
| `FAIL_STOP_HTML_PATH`            | `~/turtlebot3_fail_stop.html` | Output path for the HTML report                                             |
| `FAIL_STOP_COMMAND_HOLD_SEC`     |             `1.0`             | Duration for which fail-stop repeatedly publishes zero velocity             |
| `FORWARD_SPEED`                  |             `0.16`            | Normal line-following speed in m/s                                          |
| `ARMED_FORWARD_SPEED`            |             `0.15`            | Speed while a LEFT or RIGHT command is stored                               |
| `CURVE_FORWARD_SPEED`            |             `0.14`            | Reduced speed for larger line errors                                        |
| `LINE_KP`                        |            `0.050`            | Proportional line-following gain                                            |
| `LINE_KD`                        |            `0.012`            | Derivative line-following gain                                              |
| `BARCODE_VOTE_SAMPLES`           |              `3`              | Maximum valid barcode reads collected per vote                              |
| `BARCODE_MIN_VOTES`              |              `2`              | Normal number of matching votes required                                    |
| `BARCODE_VOTE_WINDOW_SEC`        |             `0.30`            | Duration of one barcode-voting window                                       |
| `BARCODE_LATCH_CLEAR_DISTANCE_M` |             `0.06`            | Distance used to clear a physical barcode latch                             |
| `MAX_QUEUED_COMMANDS`            |              `2`              | Maximum number of stored LEFT or RIGHT commands                             |
| `BARCODE2_REARM_DISTANCE_M`      |             `0.10`            | Minimum movement required before Barcode 2 may re-arm                       |
| `SIDE_MIN_RANGE_M`               |             `0.16`            | Minimum valid side-LiDAR range                                              |
| `SIDE_MAX_RANGE_M`               |             `0.60`            | Maximum side-board search range                                             |
| `FRONT_MIN_RANGE_M`              |             `0.16`            | Minimum valid front-LiDAR range                                             |
| `FRONT_MAX_RANGE_M`              |             `0.60`            | Maximum front-board range                                                   |
| `BLIND_VALUE_MODE`               |             `nan`             | Interpretation of too-close LiDAR readings                                  |
| `APPROACH_MAX_DISTANCE_M`        |             `0.75`            | Maximum permitted board-approach travel                                     |
| `TACKLE_PUSH_SPEED`              |             `0.04`            | Linear speed used while pushing the board                                   |
| `SENSOR_STALE_TIMEOUT_SEC`       |             `0.75`            | Maximum age of required sensor data                                         |
| `SENSOR_RECOVERY_GRACE_SEC`      |             `3.0`             | Time allowed for stale sensors to recover                                   |
| `SENSOR_MAX_RECOVERIES`          |              `2`              | Number of automatic sensor recoveries allowed                               |

### Robot Geometry Variables

<details>
<summary><strong>Show geometry settings</strong></summary>

| Variable                       | Default | Meaning                                            |
| ------------------------------ | :-----: | -------------------------------------------------- |
| `LIDAR_TO_FRONT_EDGE_M`        |  `0.14` | LiDAR axis to the retracted actuator tip           |
| `ACTUATOR_REACH_M`             |  `0.05` | Additional actuator reach after full extension     |
| `TOPPLE_OVERTRAVEL_M`          |  `0.04` | Extra push distance beyond estimated first contact |
| `REAR_OVERHANG_M`              |  `0.14` | LiDAR axis to the rearmost robot structure         |
| `STOP_DISTANCE_M`              |  `0.20` | Requested LiDAR-to-board stopping distance         |
| `FRONT_NAN_REVERSE_DISTANCE_M` |  `0.10` | Close-board reverse distance                       |

Measure these values on the completed robot instead of relying blindly on the defaults.

</details>

### List All Supported Environment Variables

Run the following command from the repository directory:

```bash
python3 - <<'PY'
import re
from pathlib import Path

source = Path("Group17Turtlebot3proj.py").read_text(encoding="utf-8")

variables = sorted(set(re.findall(
    r'os\.environ\.get\(\s*"([A-Z0-9_]+)"',
    source,
)))

print("\n".join(variables))
PY
```

---

## 🗺️ Code Guide

Recommended reading order:

| Order | Section                                   | Purpose                                                                  |
| :---: | ----------------------------------------- | ------------------------------------------------------------------------ |
|   1   | `RobotState` and `PHASE_LABELS`           | Defines the FSM states and operator-friendly labels                      |
|   2   | `TurtleBot3CombinedController.__init__()` | Creates ROS interfaces, initialises hardware, and loads tuning values    |
|   3   | `read_pixy_frame()`                       | Converts Pixy2 features into filtered line, barcode, and junction data   |
|   4   | `control_loop_from_frame()`               | Runs line following, barcode handling, junction turns, and line recovery |
|   5   | `process_obstacle_scan()`                 | Runs the LiDAR-guided board-toppling sequence                            |
|   6   | `publish_motion()`                        | Applies safety overrides and publishes `/cmd_vel`                        |
|   7   | `watchdog_callback()`                     | Detects stale sensor data and manages controlled recovery                |
|   8   | `enter_fail_stop()`                       | Latches a safety fault and presents recovery information                 |
|   9   | `reset_fail_stop_callback()`              | Clears a fault and returns to safe line reacquisition                    |
|   10  | `cleanup()`                               | Stops the robot and releases hardware resources                          |

Naming convention used in the controller:

```text
CAPITAL_NAMES = fixed settings loaded during startup
lowercase_names = live values that change while the robot operates
```

---

## 🧩 Finite-State Machine Groups

The 27 states are organised into the following functional groups:

### Normal Navigation

* `LINE_FOLLOW`
* `LINE_FOLLOW_ARMED`
* `MOVE_TO_TURN_CENTER`
* `EXECUTE_TURN_ODOM`
* `SNAP_BACK_TO_LINE`

### Obstacle Detection and Alignment

* `OBSTACLE_SCAN_SETTLE`
* `OBSTACLE_SEARCH`
* `OBSTACLE_RESCAN_SETTLE`
* `OBSTACLE_TURN_LEFT`
* `OBSTACLE_TURN_RIGHT`
* `OBSTACLE_FINE_ALIGN_SETTLE`
* `OBSTACLE_FINE_ALIGN`
* `OBSTACLE_FRONT_NAN_REAR_CHECK`
* `OBSTACLE_FRONT_NAN_REVERSE`
* `OBSTACLE_CENTERING_EXTRA`

### Obstacle Approach and Toppling

* `OBSTACLE_APPROACH`
* `OBSTACLE_BRAKE`
* `OBSTACLE_ACTUATOR_EXTEND`
* `OBSTACLE_TACKLE`
* `OBSTACLE_TACKLE_HOLD`

### Return and Recovery

* `OBSTACLE_RETURN`
* `OBSTACLE_FRONT_NAN_RETURN_FORWARD`
* `OBSTACLE_RETURN_YAW`
* `OBSTACLE_REACQUIRE_LINE`

### Safety and Completion

* `OBSTACLE_FAIL_STOP`
* `LINE_LOST_ALARM`
* `STOP_COMMAND`

---

## ⚠️ Limitations and Tuning Notes

* Pixy2 performance depends on lighting, camera height, line contrast, shadows, and glare.
* Barcode performance depends on print quality, placement, viewing angle, and robot speed.
* The configured minimum valid LiDAR range is `0.16 m`; close-board behaviour must be verified using the actual sensor and driver.
* Wheel odometry can drift, so it is used only for short local distances and turns rather than full-course localisation.
* LiDAR cluster accuracy can be affected by board angle, surface reflectivity, surrounding walls, and nearby objects.
* Board width, actuator reach, battery level, floor friction, and wheel slip can affect toppling performance.
* The sensor watchdog reduces risk but does not replace physical supervision or an accessible emergency stop.
* Safety thresholds and geometry values must be checked again whenever the robot’s hardware layout changes.
* The controller is designed for the Group 17 course robot and should be tested carefully before use on another platform.

> [!WARNING]
> This is an academic robotics prototype. Always test it in a clear area, keep people away from the actuator and wheels, and remain ready to stop the robot.

---

## 👥 Team

Proudly developed by **EEE/ISE1300 Group 17** at the **Singapore Institute of Technology**:

* Cute Sprinkles
* Heng Heng
* World Cup Champ
* Mamy Poko
* Alcoholic Guy

This repository documents an academic robotics project involving autonomous navigation, sensor integration, finite-state-machine control, hardware actuation, repeated testing, troubleshooting, and safety recovery.

---

## 🙏 Acknowledgements

* [ROBOTIS TurtleBot3 Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
* [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
* [Pixy2 / CMUcam5 Documentation](https://pixycam.com/pixy2/)

---

Made with ROS 2, many test runs, several difficult bugs and one slightly stubborn turtle. 🐢✨
