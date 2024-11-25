# ROS2 Walker Algorithm with Finite State Machine (FSM)

---
![Screenshot from 2024-11-24 17-39-18](https://github.com/user-attachments/assets/8eac3691-4d57-4a58-9108-fd5990a065a7)
---
[Watch the video here](https://drive.google.com/file/d/1c8wnbFrWuQQHwjEAGf4uR8vH-duCqmn8/view?usp=sharing)
---

A ROS2 implementation of a Roomba-like walker algorithm for the Turtlebot3 using the State Design Pattern. The robot autonomously navigates by moving forward until it encounters obstacles, then rotates in alternating directions until finding a clear path.

The FSM used has two states that control the robot's movement:

ForwardState: 
Moves straight (0.5 m/s)
Switches to rotation if obstacle detected within 0.8m

RotationState: 
Rotates in place (±0.3 rad/s)
Initial 10-second rotation
Continues rotating until path is clear
Returns to forward state when no obstacles ahead
The robot uses laser scanning in a 90° front arc to detect obstacles and alternates between these states for basic obstacle avoidance behavior.

---

## Prerequisites

- ROS2 Humble
- Turtlebot3 packages
- Gazebo Simulator

## Features
- State Pattern implementation for clean, maintainable behavior management
- Alternating rotation direction for better obstacle avoidance
- Configurable movement parameters
- ROS2 bag recording support
- Structured for unit testing capabilities

### Installation
```bash
# Install ROS2 Turtlebot3 packages if not already installed
sudo apt update
sudo apt install ros-humble-turtlebot3*

# Create workspace
mkdir -p ~/ros2_ws
cd ~/ros2_ws

# Clone the repository

# Install dependencies
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src -y -i

# Build the package
colcon build

# Source the workspace in every terminal you use
source install/setup.bash
```

## Usage

1. Set the Turtlebot3 model:
```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

2. Launch Turtlebot3 in Gazebo (Terminal 1):
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

3. Launch the walker node (Terminal 2):
```bash
# Without recording
ros2 launch walker walker_launch.py

# With rosbag recording
ros2 launch walker walker_launch.py record:=true
```

4. (Optional) Visualize in RViz2 (Terminal 3):
```bash
ros2 launch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch.py
```

## Implementation Details

### Architecture
The implementation follows the State Design Pattern with two main states:
- **Forward State**: Robot moves forward until obstacle detection
- **Rotating State**: Robot rotates in place until path is clear

### Key Classes
- `WalkerNode`: Main ROS2 node handling communications
- `WalkerState`: Abstract base class defining state interface
- `ForwardState`: Implements forward movement behavior
- `RotatingState`: Implements rotation behavior

### Parameters
- Linear velocity: 0.2 m/s
- Angular velocity: 0.5 rad/s
- Minimum obstacle distance: 0.5 m

## Recording and Playing Back Data

Record data:
```bash
ros2 launch walker walker_launch.py record_bag:=true
```

Play back recorded data:
```bash
ros2 bag play path/to/your/bag
```

## Troubleshooting

### Common Issues and Solutions

1. If Gazebo client crashes:
```bash
export SVGA_VGPU10=0
export MESA_GL_VERSION_OVERRIDE=3.3
```

2. Check if topics are being published:
```bash
# List all topics
ros2 topic list

# Monitor velocity commands
ros2 topic echo /cmd_vel

# Monitor laser scan data
ros2 topic echo /scan
```

3. Verify node is running:
```bash
ros2 node list
```
