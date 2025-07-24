# DG SDK ROS2 Bridge

## Overview
The DG SDK ROS2 Bridge provides a ROS2 interface for the DELTO gripper, allowing Python developers to control the gripper without needing C++ knowledge. This bridge wraps the DG SDK (v1.4.0) and exposes its functionality through ROS2 services and messages.

## Features
- **Python-friendly Interface**: Control the gripper entirely through Python using ROS2 services
- **Bridge Architecture**: C++ bridge handles low-level SDK communication while exposing high-level ROS2 interface
- **Multiple Communication Modes**: Supports both serial (USB) and Ethernet (TCP/IP) connections
- **Complete Gripper Control**: System configuration, connection management, and operation control

## Requirements
- ROS2 (tested on Humble)
- DG SDK library (libDGSDK.so v1.4.0 included)
- Python 3
- dg_msgs package (custom messages and services)

## Installation
```bash
# Clone the repository into your ROS2 workspace
cd ~/your_ros2_ws/src
git clone <repository_url>

# Build the package
cd ~/your_ros2_ws
colcon build --packages-select dg_sdk_ros2_bridge dg_msgs

# Source the workspace
source install/setup.bash
```

## Usage

### 1. Launch the Bridge Node
```bash
ros2 run dg_sdk_ros2_bridge dg_sdk_ros2_bridge_node
```

### 2. Python Example
The package includes a complete Python example (`example/dg3fb.py`) demonstrating how to:
- Configure gripper system settings
- Connect to the gripper
- Set gripper options
- Start the system

### 3. Available Services

#### System Configuration
- **Service**: `/dg/set_gripper_system`
- **Type**: `dg_msgs/srv/SetGripperSystem`
- **Purpose**: Configure communication parameters (port, IP, baudrate, etc.)

#### Connection Management
- **Service**: `/dg/connect_to_gripper`
- **Type**: `dg_msgs/srv/ConnectToGripper`
- **Purpose**: Establish connection with the gripper

#### Gripper Options
- **Service**: `/dg/set_gripper_option`
- **Type**: `dg_msgs/srv/SetGripperOption`
- **Purpose**: Configure gripper-specific settings (model, joint count, finger count, etc.)

#### System Control
- **Service**: `/dg/system_start`
- **Type**: `dg_msgs/srv/SystemStart`
- **Purpose**: Start the gripper system operation

## Example Code

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from dg_msgs.srv import SetGripperSystem, ConnectToGripper
from dg_msgs.msg import GripperSystemSetting

# Initialize ROS2
rclpy.init()
node = Node("gripper_control_node")

# Configure system settings
system_setting = GripperSystemSetting()
system_setting.comport = "/dev/ttyUSB0"  # For serial connection
system_setting.ip = "169.254.5.72"      # For Ethernet connection
system_setting.port = 502
system_setting.communication_mode = 0    # 0: Serial, 1: Ethernet
system_setting.control_mode = 1
system_setting.baudrate = 115200

# Create service client
client = node.create_client(SetGripperSystem, "dg/set_gripper_system")
client.wait_for_service()

# Send configuration request
request = SetGripperSystem.Request()
request.setting = system_setting
future = client.call_async(request)
rclpy.spin_until_future_complete(node, future)

# Check result
result = future.result()
print("Configuration result:", result.result)
```

## Communication Modes

### Serial (USB) Connection
```python
system_setting.communication_mode = 0
system_setting.comport = "/dev/ttyUSB0"  # Adjust to your device
system_setting.baudrate = 115200
```

### Ethernet (TCP/IP) Connection
```python
system_setting.communication_mode = 1
system_setting.ip = "169.254.5.72"  # Gripper IP address
system_setting.port = 502            # Modbus TCP port
```

## Architecture
```
Python Application
       ↓
ROS2 Services/Messages (dg_msgs)
       ↓
C++ Bridge Node (dg_sdk_ros2_bridge_node)
       ↓
DG SDK Library (libDGSDK.so v1.4.0)
       ↓
DELTO Gripper Hardware
```

This architecture allows Python developers to control the gripper without dealing with C++ code or SDK integration details.

## Troubleshooting

### Connection Issues
- Ensure the gripper is powered on and connected
- Check cable connections (USB or Ethernet)
- Verify the correct port/IP settings
- Check permissions for serial port access: `sudo chmod 666 /dev/ttyUSB0`

### Service Not Available
- Ensure the bridge node is running
- Check that the workspace is sourced correctly
- Verify dg_msgs package is built and available

## License
BSD-clause3 