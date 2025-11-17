# 🔧 Dynamixel Basic Controller (ROS 2 + GUI)

This package provides a **ROS 2-based GUI + controller** for managing **Dynamixel XM430-W210-T** motors.\
It features an intuitive **Tkinter GUI** that allows real-time control of each motor's **Position / Velocity / Current** modes.

---

## ⚙️ Features

### `dynamixel_master_node.py`

- Opens a single `/dev/ttyUSB0` port to communicate with multiple motors
- Sets **operating mode** per motor (`Position`, `Velocity`, `Current`)
- Uses **BulkRead** to read states and publishes to `/dxl<ID>/state/*` topics
- Listens to `/dxl<ID>/goal_*` and `/dxl<ID>/set_mode` topics to send commands

### `dynamixel_gui_node.py`

- For each motor ID:
  - Buttons to switch modes (`Position`, `Velocity`, `Current`)
  - Buttons to increase/decrease goal value (`+`, `−`)
  - Live display of current mode, position, velocity, current, and goal
- Mode-specific initialization and step sizes
- Publishes control commands via ROS topics

---

## 🚀 How to Run

### 1. Install Dependencies

```bash
sudo apt install ros-${ROS_DISTRO}-rclpy python3-tk
pip install dynamixel-sdk
```

### 2. Build the Package

```bash
mkdir -p ~/dynamixel_ws/src
cd ~/dynamixel_ws/src
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble-devel https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
cd ~/dynamixel_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
cd ~/dynamixel_ws
colcon build --packages-select dynamixel_test_ctrl
source install/setup.bash
```

### 3. Launch the Nodes

```bash
ros2 launch dynamixel_test_ctrl main.launch.py
```

---

## 🔌 Configuration

- **Model**: `XM430-W210-T`
- **Port**: `/dev/ttyUSB0`
- **Baudrate**: `1,000,000`
- Motor ID list can be configured in `launch/main.launch.py`:

```python
common_params = [{'ids': [11, 12, 13, 14, 15, 16]}]
```

---

## 📜 ROS 2 Topics

| Category       | Topic Examples                                                                                                       | Description        |
| -------------- | -------------------------------------------------------------------------------------------------------------------- | ------------------ |
| State Topics   | `/dxl11/state/position``/dxl11/state/velocity``/dxl11/state/current``/dxl11/state/mode``/dxl11/state/torque_enabled` | Real-time feedback |
| Command Topics | `/dxl11/goal_position``/dxl11/goal_velocity``/dxl11/goal_current``/dxl11/set_mode`                                   | Control commands   |

---# dynamixel_basic_controller
# dynamixel_basic_controller
