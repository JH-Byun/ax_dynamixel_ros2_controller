# Dynamixel Basic Controller — AX-18A (ROS 2 + GUI)

A ROS 2 Humble Python package for controlling DYNAMIXEL AX-18A servos through
one serial adapter. It provides a controller node, a Tkinter GUI, and ROS topics
for position commands, wheel speed commands, and feedback.

The current implementation targets **AX-18A**, replacing the original XM430
controller. It uses **Protocol 1.0** and individual READ requests. AX-18A does
not provide current control or measured-current feedback; the GUI displays
inferred load instead.

## Hardware configuration

| Setting | Value |
| --- | --- |
| Servo | AX-18A (model number 18) |
| Protocol | 1.0 |
| Serial port | `/dev/ttyUSB0` |
| Baud rate | 1,000,000 bps |
| Interface | Half-duplex TTL |
| Supply voltage | 9–12 V DC; 11.1 V recommended; 11.5 V is within range |

Use an external motor power supply with sufficient current for the connected
servos. **5 V is below the AX-18A operating range.** Switch power off before
changing connections, verify polarity, and use the adapter's TTL connection.
See the [ROBOTIS AX-18A manual](https://emanual.robotis.com/docs/en/dxl/ax/ax-18a/)
for wiring, electrical specifications, and the control table.

Configure the serial port and baud rate through `DEVICENAME` and `BAUDRATE`
in `dynamixel_test_ctrl/dynamixel_master_node.py`.

### Servo IDs

Both launch files pass the same ID list to the controller and GUI:

| Launch file | Current configured IDs |
| --- | --- |
| `launch/jh.launch.py` | `[3]` |
| `launch/main.launch.py` | `[11, 12, 13, 14, 15, 16]` |

Edit `common_params` in the launch file to match the actual servo IDs.
For the two-servo setup using IDs 4 and 7:

```python
common_params = [{'ids': [4, 7]}]
```

If run directly without an `ids` parameter, both nodes default to `[4, 7]`.
Each servo on the bus must have a unique ID.

## Install and build

These instructions assume ROS 2 Humble is installed and this repository is
located at `~/dynamixel_ws/src/Dynamixel_basic_controller`.

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install ros-humble-dynamixel-sdk python3-tk python3-colcon-common-extensions

cd ~/dynamixel_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select dynamixel_test_ctrl --symlink-install
source install/setup.bash
```

If `rosdep` has not been initialized on this machine, run `sudo rosdep init`
once, then `rosdep update`, before installing workspace dependencies.

If the workspace already contains the DYNAMIXEL SDK source, build that package
before the controller instead of relying only on the system SDK:

```bash
cd ~/dynamixel_ws
colcon build --packages-select dynamixel_sdk dynamixel_test_ctrl --symlink-install
source install/setup.bash
```

Dynamixel Workbench is not required by this controller.

## Run

Close DYNAMIXEL Wizard and other applications using the serial port, then run:

```bash
source /opt/ros/humble/setup.bash
source ~/dynamixel_ws/install/setup.bash
ros2 launch dynamixel_test_ctrl jh.launch.py
```

This starts `master_node` and `gui_node`. A graphical desktop is required for
the Tkinter GUI. Rebuild and source the workspace after changing launch files.

To run only the controller, for example on a machine without a display:

```bash
ros2 run dynamixel_test_ctrl master_node --ros-args -p 'ids:=[4, 7]'
```

### Controller behavior

- Pings each configured ID and checks for AX-18A model number 18. IDs that fail
  the ping, model check, or initial angle-limit read are skipped until restart.
- Starts in Position mode, writes the measured position as the goal, and then
  enables torque. It does not command position zero on startup.
- Reads position, speed, and load using a six-byte READ starting at address 36,
  on a nominal 50 ms timer. Communication delays can reduce the update rate.
- Checks SDK communication results and servo error flags. Startup errors identify
  each ID; repeated state-read errors are throttled.
- Attempts to disable torque and closes the port during normal shutdown.

### Operating modes

| Mode | Behavior | Command range |
| --- | --- | --- |
| `position` | Joint mode; holds a target angle | Raw position within the saved joint limits, normally 0–1023 |
| `velocity` | Wheel mode; continuous rotation | Signed raw command −1023 to 1023; 0 stops rotation |

Positive wheel commands select CCW and negative commands select CW according to
the servo's direction convention. In wheel mode the command sets output level;
it is not a closed-loop target in degrees/second or RPM.

Mode changes disable torque, update the EEPROM angle limits, prepare a position
hold or wheel stop, and enable torque again. Joint limits read at startup are
retained in memory and restored when returning from wheel mode. If the servo
starts with a zero CCW limit, the controller uses joint limits 0–1023.
Wheel mode writes both limits to zero; those EEPROM values persist across power
cycles. Saved custom limits are not retained by this application across restarts.

Position mode sets the Moving Speed register using the startup-only ROS parameter
`position_speed` (default: 100). Edit `position_speed = 100` in either launch file
to set a shared speed for its servos. Valid values are 1–1023; lower values move
slower. Zero is excluded because it means maximum available speed in joint mode.
Rebuild and restart after editing a launch file. For controller-only operation:

```bash
ros2 run dynamixel_test_ctrl master_node --ros-args -p 'ids:=[4, 7]' -p position_speed:=30
```

Commands for a
mode other than the active mode are ignored. There is no current-control mode.

## GUI

Each servo row provides Position and Velocity buttons, `+` and `−` buttons,
and live raw position, speed, load, and local goal displays.

| GUI control | Step | GUI goal range |
| --- | --- | --- |
| Position | 10 raw units (about 2.93°) | 0–1023 |
| Velocity | 20 raw units | −100 to 100 |

Controls wait for the first position sample. Selecting Position initializes the
local goal from the latest position; selecting Velocity initializes it to zero.
The controller rejects position commands outside its saved joint limits, even
if they are within the GUI's 0–1023 range. The displayed goal is local to the GUI
and does not track commands sent from another ROS publisher.

## Position values and degrees

In joint mode, raw positions 0–1023 span approximately 0–300°:

```python
degrees = position * 300.0 / 1023.0
position = round(degrees * 1023.0 / 300.0)

# Optional reference with raw position 512 as the center:
degrees_from_center = (position - 512) * 300.0 / 1023.0
```

| Raw position | Approximate angle |
| --- | --- |
| 0 | 0° |
| 256 | 75.1° |
| 512 | 150.1° |
| 768 | 225.2° |
| 1023 | 300° |

The mechanism's physical zero depends on horn mounting and calibration.
Wheel mode does not provide a multi-turn position count.

## ROS 2 topics

Replace `<ID>` with the servo ID, such as `4` or `7`. Numeric values are raw
servo units, with speed and load direction decoded into a sign.

| Topic | Message type | Meaning |
| --- | --- | --- |
| `/dxl<ID>/set_mode` | `std_msgs/msg/String` | `position` or `velocity` |
| `/dxl<ID>/goal_position` | `std_msgs/msg/Int32` | Joint position command |
| `/dxl<ID>/goal_velocity` | `std_msgs/msg/Int32` | Signed wheel command |
| `/dxl<ID>/state/position` | `std_msgs/msg/Int32` | Raw position |
| `/dxl<ID>/state/velocity` | `std_msgs/msg/Int32` | Signed raw present speed |
| `/dxl<ID>/state/load` | `std_msgs/msg/Int32` | Signed inferred load, not measured current or torque |
| `/dxl<ID>/state/mode` | `std_msgs/msg/String` | Last successfully configured mode, republished with feedback |
| `/dxl<ID>/state/torque_enabled` | `std_msgs/msg/Bool` | Torque state reported during mode changes; not continuously read |

The old `goal_current` and `state/current` topics are not supported.

### Command from a terminal

Keep the controller running. In a second terminal, source the environment:

```bash
source /opt/ros/humble/setup.bash
source ~/dynamixel_ws/install/setup.bash
```

The following examples require ID 4 in the configured ID list. Replace `dxl4`
with your configured servo, such as `dxl3` or `dxl7`.

Select Position mode, wait for the mode-change confirmation, then command
approximately 150°:

```bash
ros2 topic pub --once /dxl4/set_mode std_msgs/msg/String "{data: 'position'}"
ros2 topic pub --once /dxl4/goal_position std_msgs/msg/Int32 "{data: 512}"
```

Select wheel mode, wait for confirmation, then command rotation:

```bash
ros2 topic pub --once /dxl4/set_mode std_msgs/msg/String "{data: 'velocity'}"
ros2 topic pub --once /dxl4/goal_velocity std_msgs/msg/Int32 "{data: 100}"
```

Stop the wheel while it is still in Velocity mode:

```bash
ros2 topic pub --once /dxl4/goal_velocity std_msgs/msg/Int32 "{data: 0}"
```

Inspect feedback:

```bash
ros2 topic echo /dxl4/state/position
ros2 topic echo /dxl4/state/mode
```

## Troubleshooting communication

If the controller reports `There is no status packet!`:

1. Stop ROS and scan with DYNAMIXEL Wizard using Protocol 1.0, the actual serial
   port, and 1,000,000 bps. Verify the detected IDs match the launch file.
2. If no servo responds, check external power, polarity, TTL wiring, and cables.
   A USB connection alone may not supply motor power. Do not use a 5 V motor supply.
3. Connect and scan one servo at a time to isolate wiring problems or duplicate IDs.
4. Close Wizard before relaunching ROS. Check for competing processes with
   `fuser -v /dev/ttyUSB0`.
5. After correcting an ID that failed startup, restart the controller so it is
   pinged and initialized again.

If opening the port fails with a permissions error, check serial-device group
membership; Ubuntu commonly uses `dialout`. After adding your user to that group,
log out and back in.

Initialization uses acknowledged writes. If ping and reads work but writes time
out, inspect Status Return Level (address 16) in Wizard: value **2** returns
status packets for writes as well as reads and pings.

A successful port open or the controller's startup summary alone does not prove
that a servo responded. Check for a successful per-ID mode-change message and
live feedback.
