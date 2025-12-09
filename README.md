# MECH659 Aerial Robotics – Aerial Project (ROS 2 Jazzy)

This repository contains the ROS 2 workspace for the MECH659 Aerial Robotics project. It targets a PX4-based aerial platform with an onboard Raspberry Pi running Ubuntu 22 and ROS 2 Jazzy.

## System Overview

**Platform:** PX4-based quadcopter with Raspberry Pi 4  
**OS:** Ubuntu 22.04  
**ROS 2:** Jazzy  
**Camera:** Raspberry Pi Camera v2 (IMX219) via libcamera  
**Main Package:** `aerial_project`  
**Main Entrypoint:** `mission_manager` node + `bringup.launch.py`

The system integrates:
- Camera-based visual target detection
- Visual servoing for precision positioning
- Waypoint navigation
- Robotic arm for "pierce" maneuvers
- Mission manager orchestrating the full autonomous sequence

---

## Repository Structure

```
ros2_ws/
├── src/
│   └── aerial_project/
│       ├── config/
│       │   └── navigation_params.yaml
│       ├── launch/
│       │   └── bringup.launch.py
│       ├── src/
│       │   ├── mission_manager.py
│       │   ├── detection_node.cpp/.py
│       │   ├── navigation_node.cpp/.py
│       │   ├── arm_node.cpp/.py
│       │   └── visual_survoing.cpp/.py
│       ├── CMakeLists.txt
│       └── package.xml
└── ...
```

---

## Prerequisites

### Operating System & ROS 2
- Ubuntu 22.xx
- ROS 2 Jazzy installed and sourced
- Workspace path: `~/ros2_ws`

### Dependencies
- `camera_ros` (libcamera-based node for IMX219)
- `mavros` (for flight controller communication)
- `web_video_server` (for HTTP video streaming)
- `aerial_project` (this package)

### Hardware
- Raspberry Pi 4 with Camera v2 (IMX219) connected and enabled
- PX4 flight controller connected via `/dev/ttyAMA0` at 921600 baud
- Robotic arm actuated via ROS topics
- Network access to the Pi (SSH / video streaming)

### Network Addresses
- **BOB Network:** `10.189.32.89`
- **Drone Network 2.4G:** `192.168.0.123`

---

## Building the Workspace

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Add to `.bashrc` for automatic sourcing:
```bash
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
```

---

## Core System Components

### 1. Mission Manager (`mission_manager.py`)

The mission manager implements a finite-state machine coordinating the complete mission sequence:

1. **Initialize:** Force detection OFF during waypoint navigation
2. **Navigate:** Send navigation START command
3. **Wait for Hover:** Wait for `STATE:HOVER` from navigation node
4. **Hover Delay:** Hold position for configurable duration
5. **Detection Phase:** Enable detection and visual servoing
6. **Visual Servo:** Wait for `ARRIVED` status from visual servoing
7. **Arm Pierce:** Command arm to pierce configuration (`pierce_0`, `pierce_45`, etc.)
8. **Hold Pierce:** Maintain pierce pose for configured time
9. **Retract:** Fold arm back to stowed position
10. **Complete:** End mission (RTL intentionally disabled)

#### Key Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/mission/command` | `std_msgs/String` | External mission control ("START", "ABORT") |
| `/mission/status` | `std_msgs/String` | Mission state reporting |
| `/aerial_project/nav/command` | `std_msgs/String` | Navigation commands ("START", "ABORT") |
| `/aerial_project/nav/status` | `std_msgs/String` | Navigation states (e.g., "STATE:HOVER") |
| `/detection/enable` | `std_msgs/Bool` | Control detection ON/OFF |
| `/visual_servo/command` | `std_msgs/String` | Visual servo commands ("ENABLE", "DISABLE") |
| `/visual_servo/status` | `std_msgs/String` | Visual servo status (e.g., "ARRIVED") |
| `/arm/config` | `std_msgs/String` | Arm configurations ("folded", "pierce_0", "pierce_45", "pierce_minus_45") |
| `/arm/status` | `std_msgs/String` | Arm status (e.g., "DONE:folded", "DONE:pierce_0") |

#### Parameters
- `pierce_config` (string): Arm configuration for pierce maneuver (default: "pierce_45")

### 2. Launch File (`bringup.launch.py`)

Starts the complete aerial stack:
- Camera node (IMX219, 1024×768, XRGB8888)
- Detection node
- Visual servoing node
- Navigation node (with `navigation_params.yaml`)
- Arm node
- Mission manager

**Default camera device path:**  
`/base/axi/pcie@120000/rp1/i2c@80000/imx219@10`

---

## Running the Complete Mission

### Step 1: Build and Source
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Step 2: Start System Bringup
```bash
ros2 launch aerial_project bringup.launch.py
```

This launches all required nodes simultaneously.

### Step 3: Start MAVROS
In a separate terminal:
```bash
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=/dev/ttyAMA0:921600 \
  -r /uas1:=/mavros
```

### Step 4: Start Mission
```bash
ros2 topic pub --once /mission/command std_msgs/String "data: 'START'"
```

### Step 5: Monitor Mission Progress
```bash
# Mission state machine
ros2 topic echo /mission/status

# Navigation status
ros2 topic echo /aerial_project/nav/status

# Visual servo status
ros2 topic echo /visual_servo/status

# Arm status
ros2 topic echo /arm/status
```

### Step 6: Abort Mission (If Needed)
```bash
ros2 topic pub --once /mission/command std_msgs/String "data: 'ABORT'"
```

On abort, the system:
- Disables detection
- Disables visual servoing
- Commands arm to folded position
- Sends ABORT to navigation

---

## Testing Individual Subsystems

### Camera Node

**Low-resolution mode (720×576):**
```bash
ros2 run camera_ros camera_node --ros-args \
  -p camera:="/base/axi/pcie@120000/rp1/i2c@80000/imx219@10" \
  -p format:="XRGB8888" \
  -p width:=720 \
  -p height:=576
```

**Detection-resolution mode (1024×768):**
```bash
ros2 run camera_ros camera_node --ros-args \
  -p camera:="/base/axi/pcie@120000/rp1/i2c@80000/imx219@10" \
  -p format:="XRGB8888" \
  -p width:=1024 \
  -p height:=768
```

### Detection Node
```bash
ros2 run aerial_project detection_node
```

### Visual Servoing

**Start node:**
```bash
ros2 run aerial_project visual_survoing
```

**Enable visual servoing:**
```bash
ros2 topic pub --once /visual_servo/command std_msgs/String "data: 'ENABLE'"
```

**Disable visual servoing:**
```bash
ros2 topic pub --once /visual_servo/command std_msgs/String "data: 'DISABLE'"
```

### Navigation Node

**Start with parameters:**
```bash
ros2 run aerial_project navigation_node \
  --ros-args \
  --params-file ~/ros2_ws/src/aerial_project/config/navigation_params.yaml
```

**Start navigation:**
```bash
ros2 topic pub --once /aerial_project/nav/command std_msgs/msg/String "data: 'START'"
```

### Arm Control

**Start node:**
```bash
ros2 run aerial_project arm_node
```

**Command configurations:**
```bash
# Fold arm
ros2 topic pub --once /arm/config std_msgs/String "data: 'folded'"

# Pierce poses
ros2 topic pub --once /arm/config std_msgs/String "data: 'pierce_0'"
ros2 topic pub --once /arm/config std_msgs/String "data: 'pierce_45'"
ros2 topic pub --once /arm/config std_msgs/String "data: 'pierce_minus_45'"
```

**Monitor status:**
```bash
ros2 topic echo /arm/status
```

---

## Video Streaming & Remote Access

### Start Video Server
```bash
ros2 run web_video_server web_video_server
```

### SSH Access
```bash
# BOB network
ssh -X aerial-pi@10.189.32.89

# Drone Network 2.4G
ssh -X aerial-pi@192.168.0.123
```

Access video stream in browser at: `http://<pi_ip_address>:8080`

---

## Configuration

### Mission Manager Parameters

Configured in `bringup.launch.py`:
```python
mission_manager_node = Node(
    package="aerial_project",
    executable="mission_manager",
    name="mission_manager",
    output="screen",
    parameters=[{
        "pierce_config": "pierce_45"
    }],
)
```

Additional timing parameters (hover wait times, arm timeouts, post-arrival delays) are defined in `mission_manager.py`.

### Navigation Parameters

Located at: `~/ros2_ws/src/aerial_project/config/navigation_params.yaml`

Contains mission-specific waypoints, speeds, frame IDs, and navigation behavior settings.

---

## Troubleshooting

### Camera not detected
- Verify camera connection: `libcamera-hello`
- Check device path in launch file matches your system

### MAVROS connection failed
- Verify serial port: `ls -l /dev/ttyAMA0`
- Check baud rate matches flight controller configuration (921600)
- Ensure user is in `dialout` group: `sudo usermod -a -G dialout $USER`

### Mission stuck in state
- Check topic echoes for all subsystems
- Verify each node is publishing status updates
- Review node logs for errors

### Arm not responding
- Verify arm hardware connections
- Check `/arm/status` for error messages
- Test arm node independently before full mission



## License

[Add appropriate license information]
